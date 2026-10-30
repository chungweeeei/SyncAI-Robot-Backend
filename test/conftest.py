"""Shared pytest fixtures for syncai_backend unit tests.

These tests are meant to run inside the ROS 2 devcontainer (via ``colcon test``
or ``pytest``), where rclpy / nav_msgs / syncai_common / opencv are available.
The database layer is exercised against an in-memory SQLite engine instead of
PostgreSQL, so no database server is required.
"""

import asyncio
import builtins
import io
import math
import os
import sys

import pytest
import structlog

from sqlalchemy import create_engine
from sqlalchemy.pool import StaticPool

import syncai_backend
from syncai_backend.database.models import Base


# --- No disk I/O on the event loop -----------------------------------------
#
# The REST server's loop is shared by every async route, teleop and the
# telemetry streams; one blocking read there (an INI, a map walk) stalls all of
# them for as long as the disk takes. Blocking routes are plain `def` (the
# threadpool) and async ones put disk work in `asyncio.to_thread` (CLAUDE.md).
# This guard holds every test to that: a file opened or stat'ed by package code
# on a thread with a running loop fails the test that did it.
#
# Python-level entry points only, which is what package code calls: `open`,
# and the `os` functions behind os.path.isdir / exists / getmtime / realpath and
# os.walk. Imports go through `posix` directly and are not seen.

_PACKAGE_DIR = os.path.dirname(syncai_backend.__file__) + os.sep

_GUARDED = (
    (builtins, "open"),
    (io, "open"),
    (os, "stat"),
    (os, "lstat"),
    (os, "scandir"),
    (os, "listdir"),
    (os, "access"),
)


def _package_caller():
    """The innermost package frame on this thread's stack, or None: a test
    body's own file access on the loop is the test's business."""
    frame = sys._getframe(2)
    while frame is not None:
        filename = frame.f_code.co_filename
        if filename.startswith(_PACKAGE_DIR):
            return f"{filename[len(_PACKAGE_DIR):]}:{frame.f_lineno} ({frame.f_code.co_name})"
        frame = frame.f_back
    return None


@pytest.fixture(autouse=True)
def no_disk_io_on_the_event_loop(monkeypatch):
    """Yields what it found, {caller: [first call, count]}, for its own test."""
    found = {}

    def _guard(name, real):
        def guarded(*args, **kwargs):
            try:
                asyncio.get_running_loop()
            except RuntimeError:
                return real(*args, **kwargs)
            caller = _package_caller()
            if caller is not None:
                target = args[0] if args else kwargs.get("file", kwargs.get("path"))
                found.setdefault(caller, [f"{name}({target!r})", 0])[1] += 1
            return real(*args, **kwargs)

        return guarded

    for module, name in _GUARDED:
        monkeypatch.setattr(
            module, name, _guard(f"{module.__name__}.{name}", getattr(module, name))
        )

    yield found

    assert not found, (
        "Disk I/O on the event loop (an async route reads the disk; put it in "
        "asyncio.to_thread or make the route a plain def):\n  "
        + "\n  ".join(
            f"{caller}: {first}" + (f" and {count - 1} more" if count > 1 else "")
            for caller, (first, count) in found.items()
        )
    )


@pytest.fixture
def logger():
    return structlog.get_logger()


class StubInternalApiGateway:
    """The internal service the map switch tells: knows every map as
    ``id-<name>`` unless ``maps`` says otherwise; records each activate."""

    def __init__(self):
        self.maps = None  # None: every name is known
        self.lookup_error = None
        self.activate_error = None
        self.activated = []

    def find_map_id(self, name):
        if self.lookup_error is not None:
            return False, self.lookup_error, None
        if self.maps is not None and name not in self.maps:
            return True, "", None
        return True, "", f"id-{name}"

    def activate_map(self, map_id):
        self.activated.append(map_id)
        if self.activate_error is not None:
            return False, self.activate_error, None
        return True, "", {"id": map_id, "switched": True}


@pytest.fixture
def internal_api_gw():
    return StubInternalApiGateway()


@pytest.fixture
def engine():
    """An SQLAlchemy engine bound to a shared in-memory SQLite database.

    ``StaticPool`` keeps a single connection alive so every session sees the
    same in-memory schema/data; the DB is torn down after each test.
    """
    engine = create_engine(
        "sqlite://",
        connect_args={"check_same_thread": False},
        poolclass=StaticPool,
    )
    Base.metadata.create_all(bind=engine)

    yield engine

    Base.metadata.drop_all(bind=engine)
    engine.dispose()


@pytest.fixture
def map_repo(logger, engine):
    """A MapRepo backed by the in-memory SQLite engine.

    The engine is not optional — MapRepo cannot be constructed without one — so
    there is no engine-less variant of this fixture to pair with.
    """
    from syncai_backend.repositories.map.map import init_map_repo

    return init_map_repo(logger=logger, engine=engine)


@pytest.fixture
def task_template_repo(logger, engine):
    """A TaskTemplateRepo backed by the in-memory SQLite engine.

    Shares the ``engine`` fixture with ``map_repo`` on purpose: a template's
    MOVE steps reference rows in ``map_vertices``, and the resolution path under
    test is exactly the join between the two tables.

    The ``engine`` fixture itself needs no change for this table --
    ``TaskTemplate`` registers on the same ``Base`` as ``MapPoint`` the moment
    ``database.models`` is imported, so ``create_all`` already emits it.
    """
    from syncai_backend.repositories.task.task_template import (
        init_task_template_repo,
    )

    return init_task_template_repo(logger=logger, engine=engine)


@pytest.fixture
def make_pgm():
    """Factory writing a binary PGM, with hooks for the malformed cases.

    ``comment`` inserts a ``# ...`` line after the magic (what GIMP does);
    ``body_bytes`` overrides the pixel body so a truncated file can be built.
    """
    def _make(path, width, height, fill=254, comment=None, maxval=255,
              magic=b"P5", body_bytes=None):
        header = magic + b"\n"
        if comment is not None:
            header += b"# " + comment.encode("ascii") + b"\n"
        header += f"{width} {height}\n{maxval}\n".encode("ascii")
        body = bytes([fill]) * (width * height) if body_bytes is None else body_bytes
        path.write_bytes(header + body)
        return path

    return _make


@pytest.fixture
def make_gridmap_yaml():
    """Factory writing a map YAML in the shape pcd_to_gridmap.py emits."""
    def _make(path, resolution=0.05, origin=(-1.0, -2.0, 0.0)):
        path.write_text(
            "image: gridmap.pgm\n"
            "mode: trinary\n"
            f"resolution: {resolution}\n"
            f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
        return path

    return _make


@pytest.fixture
def make_pcd():
    """Factory writing a minimal ``DATA ascii`` PCD with xyz fields.

    Real enough for read_pcd_xyz: the map catalogue's pointcloud endpoint parses
    the file it serves, so a fixture of filler bytes would only ever exercise
    the failure path.
    """
    def _make(path, points=((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0))):
        rows = "\n".join(f"{x} {y} {z}" for x, y, z in points)
        path.write_text(
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {len(points)}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(points)}\n"
            "DATA ascii\n"
            f"{rows}\n"
        )
        return path

    return _make


@pytest.fixture
def make_bt():
    """Factory writing a minimal OcTree ``.bt``: the text header, then a tree.

    The default tree is one root record whose first child is an occupied
    leaf — two bytes, what a console's decoder needs to accept it. The
    backend never parses past the header (it reads ``res`` and serves the
    file byte for byte), so ``body`` is only ever compared, not decoded.
    ``banner=False`` builds a file that is not a tree at all.
    """
    def _make(path, res=0.1, banner=True, body=b"\x02\x00"):
        header = b""
        if banner:
            header += (
                b"# Octomap OcTree binary file\n"
                b"# (feel free to add / change comments, but leave the first line as it is!)\n"
            )
        header += b"id OcTree\nsize 2\n"
        if res is not None:
            header += f"res {res}\n".encode("ascii")
        header += b"data\n"
        path.write_bytes(header + body)
        return path

    return _make


@pytest.fixture
def maps_dir(tmp_path, make_pcd, make_bt):
    """A maps root laid out the way the robot's ``map/`` directory is.

    ``full`` is a navigable map (pcd + octomap.bt), ``rawonly`` is one straight
    out of pgo/save_maps whose 3D map is not built yet, and a loose file sits
    at the root to prove only directories are listed.
    """
    root = tmp_path / "map"
    root.mkdir()

    full = root / "full"
    full.mkdir()
    make_pcd(full / "map.pcd")
    make_bt(full / "octomap.bt")

    rawonly = root / "rawonly"
    rawonly.mkdir()
    make_pcd(rawonly / "map.pcd", points=((0.0, 0.0, 0.0),))

    (root / "stray.pgm").write_bytes(b"not a map directory")

    return root


@pytest.fixture
def catalog_repo(logger, maps_dir):
    """A MapCatalogRepo re-pointed at the fake maps tree.

    Constructed and then re-pointed, rather than told where to look: the repo
    hardcodes ``~/robot_ws/map``, and the alternative was a constructor argument
    plus an environment variable that existed only for these tests.
    """
    from syncai_backend.repositories.map.catalog import init_map_catalog_repo

    repo = init_map_catalog_repo(logger=logger)
    repo.maps_dir = str(maps_dir)
    return repo


@pytest.fixture
def make_occupancy_grid():
    """Factory building a nav_msgs/OccupancyGrid from raw cell values.

    nav_msgs is imported lazily so tests that never touch a grid (and the host
    machine without ROS 2) do not require it at collection time.
    """
    from nav_msgs.msg import OccupancyGrid

    def _make(width, height, data, resolution=0.05, origin=(-1.0, -2.0, 0.0),
              frame_id="map"):
        grid = OccupancyGrid()
        grid.header.frame_id = frame_id
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.position.z = float(origin[2])
        grid.data = list(data)
        return grid

    return _make


@pytest.fixture
def make_robot_state():
    """Factory building a syncai_common/RobotState the way the aggregator does.

    ``syncai_common`` is imported lazily for the same reason as the grid factory
    above: a host without the built interfaces must still collect.

    Defaults describe a healthy, localized robot, because the interesting tests
    are the ones that override one thing. Note what the defaults encode about the
    message's own conventions, all of which the REST projection has to respect:

    * ``timestamp`` is SECONDS, ``motor_status.timestamp`` is also seconds *here*
      (the ``motor_states`` topic carries nanoseconds; syncai_robot_state
      rescales) — so a test asserting the payload does not need to convert.
    * ``localization_status.position.yaw`` is RADIANS; the payload is degrees.
    * ``battery_status.battery_percentage`` is 0–100 as a float; the payload is
      an int.
    * ``low_level_mode`` carries the controller's own integers, which may be
      values the command surface refuses — pass ``policy_state=2`` to get the
      CHAMP case that must not 500 the endpoint.
    """
    from syncai_common.msg import MotorState, RobotState as RobotStateMsg

    def _make(
        robot_id="robot01",
        timestamp=1754000000,
        map_name="dp2f",
        mode=2,  # RobotMode.AUTO
        state=1,  # RobotStatus.IDLE
        localization_valid=True,
        position=(1.5, -2.5, 0.0, math.pi / 2),
        velocity=0.25,
        wifi_info='{"ssid": "net", "rssi": -40, "ip_address": "10.0.0.2"}',
        battery_percentage=87.6,
        motors=(("FL_HipX_joint", 41, 0),),
        motor_timestamp=1754000000,
        policy_state=1,
        motion_state=1,
        safety_state=False,
    ):
        msg = RobotStateMsg()
        msg.robot_id = robot_id
        msg.timestamp = timestamp
        msg.map = map_name
        msg.mode = mode
        msg.state = state
        msg.localization_valid = localization_valid
        msg.localization_status.position.x = float(position[0])
        msg.localization_status.position.y = float(position[1])
        msg.localization_status.position.z = float(position[2])
        msg.localization_status.position.yaw = float(position[3])
        msg.localization_status.velocity = float(velocity)
        msg.network_status.wifi_info = wifi_info
        msg.battery_status.battery_percentage = float(battery_percentage)

        msg.motor_status.timestamp = motor_timestamp
        states = []
        for name, temperature, error in motors:
            motor = MotorState()
            motor.name = name
            motor.temperature = temperature
            motor.error = error
            # Kinematics deliberately non-zero: the payload must not carry them,
            # and a zero would make a leak indistinguishable from a default.
            motor.q = 0.75
            motor.dq = -0.5
            states.append(motor)
        msg.motor_status.states = states

        msg.low_level_mode.policy_state = policy_state
        msg.low_level_mode.motion_state = motion_state
        msg.low_level_mode.safety_state = safety_state
        return msg

    return _make
