# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

`syncai_backend` — the robot's operator-facing process: a **FastAPI REST/WebSocket
server, an rclpy ROS 2 node and a Temporal worker in one Python process**. It is a
ROS 2 `ament_python` package (`package.xml`, `setup.py`), split out of
`SyncAI-Robot-Workspace` into its own repository with the package at the repo root.

It **cannot run or be tested standalone**. It imports generated interfaces from two
other colcon packages — `syncai_common` (`RobotState`, `RobotMode`, `MotorStates`,
`WifiNetwork`; `SwitchMode`, `SetMotionKey`, `SetPolicyMode`, `Scan/ConnectWifiNetwork`)
and `interface` from the FAST-LIO2 fork (`SaveMaps`, `ResetMapping`, `Relocalize`,
`IsValid`) — plus `rclpy`, `nav2_msgs`, `tf2_ros`. Keep those imports as they are; the
workspace pulls this repo and the interface repos side by side with vcstool into
`<workspace>/src/`, and that is where builds, tests and runs happen (inside the robot
container, ROS 2 Humble / Python 3.10).

`README.md` is the long-form reference (every REST route, ROS interface + QoS, Temporal
semantics, gotchas). Read the relevant section before changing a route or a subscriber;
it records *why* things are the way they are. Keep it in sync when you change behavior.

## Commands

All of these run inside the robot container, from the **workspace root** after
`source install/setup.bash`, with this repo checked out at `src/syncai_backend`.

```bash
# Python deps (not rosdep-managed; requirements.txt is the single source of truth)
pip install -r src/syncai_backend/requirements.txt

# Build
colcon build --packages-select syncai_backend --symlink-install

# Run
ros2 launch syncai_backend backend.launch.py                                   # robot_id from ~/robot_ws/config/system.ini
ros2 launch syncai_backend backend.launch.py system_config:=config/instances/robot01.ini
ros2 run syncai_backend backend                                                # no namespace -> default_robot

# Tests (from src/syncai_backend/)
python3 -m pytest test/
python3 -m pytest test/test_map_router.py                # one file
python3 -m pytest test/test_robot_router.py -k whitelist # one test by name
colcon test --packages-select syncai_backend && colcon test-result --verbose

# Lint (ruff.toml in this repo pins the rule set; isort is deliberately off)
ruff check .
```

Test notes: tests `importorskip` `rclpy` / `syncai_common` / `httpx` etc., so on a machine
without ROS most of them skip rather than fail — a green run outside the container proves
little. The DB layer is tested against in-memory SQLite (`StaticPool`); no PostgreSQL
needed. `test_traversable.py` needs open3d, `test_pcd_to_gridmap.py` needs scipy. The
ament linters (`test_copyright`, `test_flake8`, `test_pep257`) are part of the suite.

## Architecture

### Process model (`syncai_backend/main.py`)

`SyncAIBackend(Node)` is constructed once; its constructor wires everything and starts
two daemon threads. Main thread: `MultiThreadedExecutor.spin()` (all ROS callbacks, TF,
service/action clients). uvicorn thread: FastAPI on `0.0.0.0:3000`. Temporal worker
thread: polls `<robot_id>.ROBOT_TASK_QUEUE`, runs `RobotWorkflow` + activities in a
single-worker `ThreadPoolExecutor`.

Consequences:
- **Blocking REST handlers are plain `def`, not `async def`** (ROS service calls up to
  ~70 s, psycopg2, PNG encoding) so FastAPI runs them in its thread pool. Keep that
  distinction when adding endpoints.
- Heavy point-cloud callbacks each get their own `MutuallyExclusiveCallbackGroup` so they
  cannot starve `robot_state` / telemetry / TF.
- Postgres is a hard dependency (20 retries × 5 s, then the process exits); Temporal is
  soft — `TemporalWorkerHandle` records `connecting`/`running`/`dead` and `/health`
  reports `degraded` instead of crashing.

### Layering (convention, not enforced)

```
interfaces/rest/routers/   HTTP + WS surface; pydantic schemas; raises domain exceptions
        │
services/                  domain work that outlives a request: gridmap_conversion (recipes,
        │                  the registry of running threads, the gridmap.recipe.json protocol)
        │
gateways/                  outbound: robot (ROS srv/action/pub), map (ROS srv), workflow (Temporal),
        │                  tts (kokoro-onnx → aplay), webrtc (dlopen'd Go worker), recording
        │                  (supervised `ros2 bag record` child). tts/webrtc/recording hold no node.
repositories/              state: in-memory single-slot caches (robot, pointcloud, telemetry),
        │                  PostgreSQL CRUD (map vertices, task_templates), on-disk catalogues (map/, record/)
database/                  SQLAlchemy engine + ORM (models.py: MapPoint, TaskTemplate)

subscribers/               ROS topics → repositories (ingest side)
temporal/                  worker, RobotWorkflow, activities
helpers/                   occupancy_grid, pointcloud, pgm, pcd_to_gridmap (z-band), traversable, system_config
```

**Wiring is explicit constructor injection.** `main.py` builds every repo/gateway/
subscriber via `init_<x>(...)` factories and passes them into `start_rest_server(...)`,
which hands them to `init_<x>_router(...)`. No DI container, no module-level singletons.
Adding a dependency to a router means threading it through `main.py` → `server.py`.
`repositories/base.py` and `jobs/base.py` are unused scaffolding.

That rule is why the gridmap conversion is a `services/` object rather than the
module-level `_ACTIVE_CONVERSIONS` set it used to be inside `routers/map.py`: it is
built in `main.py` like everything else, so tests get a fresh registry per test and
nothing reaches into another module's globals to say "a conversion is running".

**Imports are grouped by layer, with blank lines carrying meaning** (see `main.py`).
That is why `ruff.toml` excludes isort — do not "organize imports".

### Errors

Routers raise from `exceptions.py`; `server.py:register_exception_handlers` maps them:
`NotFoundError`→404, `BadRequestError`→400, `ConflictError`→409 (optional
machine-readable `code` beside `detail`, e.g. `conversion_running`, `map_active`),
`UnauthorizedError`→401, `UpstreamError`→502 (a downstream — Temporal or a ROS service —
failed). Cross-field validation errors answer 400 with a sentence, not 422.

`WorkflowGateway` raises those directly. The rest return `(success, message, …)` and the
router decides, uniformly 502 — **except** for the handful of failures a caller answers
differently, which `gateways/failure.py` tags with a `Failure` code that rides on the
message (a `str` subclass, so prose still reads as prose). Read it with `failure_code()`;
never match on the sentence. That module is also the single home of the one rule two
consumers share: an unknown TTS voice is a 400 for the router **and** non-retryable for
the SPEAK activity.

### robot_id scopes everything

The launch file reads `[system] robot_id` from the system INI and uses it as the **node
namespace**; the node reads it back with `self.get_namespace()`. That one value scopes
ROS names (all topics/services/actions in this package are **relative** — never hardcode
`/<robot_id>/…`), the PostgreSQL database `<robot_id>_db`, and the Temporal task queue.
TF frame names are not namespaced.

### Configuration

Environment only (`TEMPORAL_ADDRESS`, `POSTGRES_*`, `SYNCAI_SYSTEM_INI`), loaded from the
cwd `.env` via python-dotenv at import time — see `.env.example`. **No ROS parameters**
anywhere. Everything is read once at startup. Several paths are absolute on purpose
(`~/robot_ws/config/system.ini`, `~/robot_ws/map`, `~/robot_ws/record`,
`~/robot_ws/models/kokoro/`, `~/robot_ws/lib/libsyncai_worker.so`) because entrypoints do
not reliably run from the workspace root.

### Persistence rules worth knowing before touching `database/` or `repositories/`

- **No migrations.** Schema is whatever `create_all` produced on a robot. Adding a column
  or constraint has no path to existing deployments; `TaskTemplate.steps` is a JSON column
  for this reason. Only ever *add optional keys* to a stored step — never rename/retype.
- JSON columns are not mutation-tracked: assign a fresh list, never mutate in place.
- `Base.metadata` is shared: adding a model changes what every `init_*_repo`'s
  `create_all` does.
- REST says **vertex** (`VertexType`), ORM/repo say `MapPoint` (table `map_vertices`).
  Intentional; do not "fix" by migrating.
- Map directory name is a foreign key by convention in `map_vertices.map` and
  `task_templates.map_name` (no constraint). Rename: filesystem first, DB second, rollback
  by renaming back. Delete: DB rows first, `rmtree` last (irreversible step goes last).
- Long-running outcomes that must outlive the process live **on disk**, not in memory:
  gridmap conversion status in `<map>/gridmap.recipe.json`; a bag with no `metadata.yaml`
  and no live process is `interrupted`. `switch_mode` kills the byobu session this
  process runs in, so in-memory registries die with it.

### Temporal specifics

Activities are synchronous; cancellation arrives as `CancelledError` thrown into the
thread — cleanup goes in `except CancelledError`, not an `is_cancelled()` poll. `SPEAK`
cannot heartbeat (blocks on `aplay`) so it runs on `start_to_close` alone. Per-step state
is a workflow **query**, not a table. Schedules use `SKIP` overlap; their steps are frozen
at registration; the original cron string and `map_name`/template ids ride in the schedule
**memo**. `ARTIFACT` steps were removed 2026-08 — stored templates carrying one fail
validation.

### Heavy imports

`helpers/traversable.py` is the **only** module that imports open3d (~100 MB), and only
inside `GridmapConversionService.start`'s conversion thread, in its `try`. Keep
`pcd_to_gridmap.py` open3d-free. The kokoro TTS
model (~310 MB) and the WebRTC Go worker are lazy-loaded on first use, each owned by
exactly one gateway instance whose internal lock serialises REST and Temporal callers.

## Dependency pins that are not negotiable without reading `requirements.txt`

`onnxruntime==1.18.1` (newer versions corrupt the heap on the Orin with cores offlined),
`scipy>=1.8,<1.11` and `open3d>=0.18,<0.20` (both exist to stop pip dragging numpy past
1.26, which the ROS ecosystem tolerates). `kokoro-onnx` is deliberately absent — the
Dockerfile installs it `--no-deps`. `setup.py` ships **bytecode only** on a non-symlink
install (`InstallNoSource`); `--symlink-install` dev builds are unaffected.
