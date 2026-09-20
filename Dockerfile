# syncai_backend — a test/dev image for working on this package off the robot.
#
# This repo cannot be tested standalone: it imports rclpy, nav2_msgs and tf2_ros,
# plus generated interfaces from two other colcon packages (`syncai_common` and
# FAST-LIO2's `interface`) that live in their own repositories. This image
# supplies the first group; the second is optional and mounted in when you have
# it, which is the difference between 252 and 614 tests actually running.
#
# The source is NOT copied in. It is bind-mounted, so an edit on the host is
# visible to the next pytest run with no rebuild — the reason to rebuild is a
# change to requirements.txt or package.xml, nothing else.
#
#   docker build -t syncai-backend-dev .
#
#   # Without the interface packages: 252 tests run, 11 files cannot even be
#   # collected because they reach syncai_common or interface through an import
#   # rather than through an importorskip. Useful for the helpers, the repos and
#   # the catalogue; not enough to trust a change to a router or a gateway.
#   docker run --rm -v "$PWD":/ros2_ws/src/syncai_backend syncai-backend-dev
#
#   # With them: 614 pass, 1 skips. WS is the workspace this repo is vcs-imported
#   # into; both packages are built from source into /ros2_ws/install, which takes
#   # about 17 s. Mounted read-only -- the build writes only to /ros2_ws.
#   WS=~/Desktop/SyncAI-Robot-Workspace
#   docker run --rm \
#       -v "$PWD":/ros2_ws/src/syncai_backend \
#       -v "$WS/src/syncai_common":/ros2_ws/src/syncai_common:ro \
#       -v "$WS/src/third-party/FASTLIO2_ROS2/interface":/ros2_ws/src/interface:ro \
#       syncai-backend-dev bash -lc '
#           colcon-build --packages-select syncai_common interface >/dev/null
#           source /ros2_ws/install/setup.bash
#           cd /ros2_ws/src/syncai_backend && python3 -m pytest test/ -q'
#
#   # Add `-v syncai-ws:/ros2_ws/install` to keep that build between runs, or use
#   # `docker run -d --name … sleep infinity` + `docker exec` and build once.
#
#   # A shell to poke around in.
#   docker run --rm -it -v "$PWD":/ros2_ws/src/syncai_backend syncai-backend-dev bash
#
# ros-base rather than ros-core: rosbag2 (the recording gateway spawns
# `ros2 bag record`) and tf2 come with it, which package.xml's ros2bag and
# tf2_ros exec_depends both want.
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Two groups, and the split is worth keeping straight:
#
#   * ROS interfaces this package imports that ros-base does not carry.
#     nav2_msgs is the one that actually matters (the robot gateway's
#     NavigateToPose action and the map gateway's LoadMap service); the others
#     are cheap and explicit rather than relied on transitively.
#   * the three ament linters, because test_copyright.py / test_flake8.py /
#     test_pep257.py are part of this suite and silently vanish without them —
#     a green run that skipped the linters is exactly the kind of false green
#     this image exists to avoid.
#
# python3-pytest comes in anyway as an ament-linter dependency; it is not what
# actually runs the suite (see the pip install of pytest further down).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-pytest \
        python3-colcon-common-extensions \
        ros-humble-nav2-msgs \
        ros-humble-sensor-msgs-py \
        ros-humble-tf2-ros-py \
        ros-humble-ament-copyright \
        ros-humble-ament-flake8 \
        ros-humble-ament-pep257 \
        alsa-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copied on its own, ahead of any source, so editing the package does not
# invalidate this layer. requirements.txt is the single source of truth for the
# python deps (it says so itself); nothing is duplicated here.
COPY requirements.txt /tmp/requirements.txt

# open3d is installed separately and is allowed to fail, which is a deliberate
# choice rather than sloppiness. It ships no aarch64 Linux wheel for every
# version in the pinned range, and this image is built on Apple Silicon as often
# as not. The package already treats it as optional at runtime -- helpers/
# traversable.py is imported inside the conversion thread's try and an
# ImportError is recorded as a per-map failure -- so the cost of not having it
# is test_traversable.py skipping and the traversability recipe being
# unavailable. The z-band recipe, which is what every save actually uses, is
# open3d-free by design. Read the warning; do not let it become invisible.
#
# --no-deps for kokoro-onnx, matching the robot's Dockerfile: its metadata
# demands onnxruntime>=1.20.1 and numpy>=2, both of which are wrong here, and
# requirements.txt spells out the dependencies it actually needs instead.
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && grep -v '^open3d' /tmp/requirements.txt > /tmp/requirements.core.txt \
    && python3 -m pip install --no-cache-dir -r /tmp/requirements.core.txt \
    && python3 -m pip install --no-cache-dir --no-deps kokoro-onnx \
    && { python3 -m pip install --no-cache-dir "$(grep '^open3d' /tmp/requirements.txt)" \
         || echo "WARNING: open3d unavailable for this platform — test_traversable.py will skip and the traversability recipe is disabled"; }

# open3d's native dependencies, deliberately here rather than up with the ROS
# packages: they belong to the wheel installed directly above, and grouping them
# with it keeps the pair legible -- an `import open3d` failing on a missing
# shared library looks nothing like a missing package, and this is the comment
# that says where to look. `libGL.so.1` is needed even though nothing here ever
# renders: open3d links it unconditionally, so without it every test in
# test_traversable.py skips with "open3d is not installed" while pip insists it
# is. libgomp is its OpenMP runtime.
#
# Placement also keeps the requirements install above cached -- that layer takes
# ~20 minutes on aarch64 and an apt line in front of it would rebuild the lot.
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 \
        libgomp1 \
    && rm -rf /var/lib/apt/lists/*


# pytest from pip rather than the apt python3-pytest that the ament linters drag
# in. Ubuntu 22.04 ships pytest 6.2.5, and anyio -- which arrives with
# fastapi/starlette and registers itself as a pytest11 plugin, so it is loaded
# whether or not a test asks for it -- imports `_pytest.scope`, which did not
# exist before pytest 7. On the apt version the suite does not fail a test, it
# fails to collect at all, with a ModuleNotFoundError out of plugin loading.
#
# Its own layer, after the requirements install above: that one takes ~20
# minutes to build on aarch64, and test tooling has no business invalidating it.
# pip installs into /usr/local/lib/python3.10/dist-packages, which precedes the
# apt path on sys.path, so `python3 -m pytest` picks this one up.
# Both ends of the range are load-bearing, and each was found by a failing run:
#
#   * the floor, because `_pytest.scope` landed in 7.0. It is also written as a
#     constraint rather than a bare `pytest` because apt's 6.2.5 satisfies an
#     unconstrained requirement -- pip reports "already satisfied", installs
#     nothing, and the collection error survives the fix meant to cure it.
#   * the ceiling, because ROS 2 Humble's own `launch_testing` registers a
#     pytest11 plugin whose `pytest_pycollect_makemodule(path, parent)` uses the
#     `path` argument that pytest 8 removed. It is loaded whether or not this
#     suite uses it, so an unpinned install takes pytest 9 and dies in plugin
#     validation before collecting a single test.
#
# 7.x is the whole of the window where modern anyio and Humble's ROS plugins
# both work.
RUN python3 -m pip install --no-cache-dir "pytest>=7,<8"


# A convenience wrapper rather than something the entrypoint does on its own:
# building takes minutes and most runs of this image do not need it, so it is
# opt-in. --symlink-install because setup.py strips sources on a non-symlink
# install (InstallNoSource) and that would leave the tests importing bytecode.
#
# `set -e` without `-u`: ROS's own setup.bash reads AMENT_TRACE_SETUP_FILES
# unguarded, so nounset turns sourcing it into an immediate failure.
RUN printf '%s\n' \
        '#!/bin/bash' \
        'set -eo pipefail' \
        'source /opt/ros/humble/setup.bash' \
        'cd /ros2_ws' \
        'colcon build --symlink-install "$@"' \
        > /usr/local/bin/colcon-build \
    && chmod +x /usr/local/bin/colcon-build

# Sources ROS, then the workspace install space if colcon-build has been run.
# The stock /ros_entrypoint.sh does only the first half, and without the second
# a built syncai_common would be invisible to pytest.
RUN printf '%s\n' \
        '#!/bin/bash' \
        'set -e' \
        'source /opt/ros/humble/setup.bash' \
        'if [ -f /ros2_ws/install/setup.bash ]; then' \
        '  source /ros2_ws/install/setup.bash' \
        'fi' \
        'exec "$@"' \
        > /usr/local/bin/entrypoint.sh \
    && chmod +x /usr/local/bin/entrypoint.sh

# Not root, and this is not hygiene theatre: root bypasses file-permission
# checks entirely, so os.access(path, os.W_OK) answers True even for a mode-0444
# file. test_activate_refuses_when_the_ini_is_not_writable chmods the system INI
# to 0444 and asserts the route refuses the switch -- as root the preflight sees
# a writable INI, the refusal never happens, and the test fails for a reason
# that has nothing to do with the code under test. An image that reports a
# failure the robot does not have is worse than no image.
#
# Override USER_UID at build time if your host uid is not 1000 and bind-mounted
# files come out unwritable.
ARG USER_UID=1000
RUN useradd --create-home --uid ${USER_UID} dev \
    && chown -R dev:dev /ros2_ws
USER dev

# pytest is run as `python3 -m pytest test/` from here, and the `python3 -m`
# half is load-bearing: it puts the repo root on sys.path, which is how
# `import syncai_backend` resolves without the package being installed.
WORKDIR /ros2_ws/src/syncai_backend

# Humble registers two launch-testing plugins as pytest11 entry points, so they
# load whether or not anything here uses them -- and `launch_ros` declares a hook
# that only `launch_testing` provides, so they have to be disabled as a pair or
# pytest dies in plugin validation before collecting anything. Worse, with them
# enabled `launch_testing` eagerly imports every test module looking for launch
# entrypoints, which turns a module that would have been skipped into a fatal
# collection error.
#
# In the environment variable rather than in CMD so it still applies when you
# pass your own pytest arguments, and overridable with `-e PYTEST_ADDOPTS=` if
# you ever do want them.
# --continue-on-collection-errors is the third piece: without the interface
# packages mounted, 11 files reach syncai_common or interface through a plain
# import (not an importorskip) and fail to collect. Left fatal, those 11 abort
# the run and the 241 tests that would have passed never execute. They are still
# reported as errors and the run still exits non-zero -- nothing is hidden, the
# rest just gets to finish.
ENV PYTEST_ADDOPTS="-p no:launch_testing -p no:launch_ros --continue-on-collection-errors"

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["python3", "-m", "pytest", "test/", "-q"]
