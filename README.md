# syncai_backend

The robot's operator-facing backend: a **FastAPI REST/WebSocket server, an rclpy
ROS 2 node and a Temporal worker in one Python process**.

It is the robot's whole application surface — robot state, maps, point clouds,
tasks, schedules, mode switching, wifi, speech, recordings, camera — and every
ROS interaction on behalf of an operator (nav goals, motion keys, map saves and
switches) happens here.

```
                    HTTP :3000 / WebSocket
  any API client   ────────────────────────►  syncai_backend  ──── ROS 2 ────►  nav stack,
                                                    │                            robot_state,
                                                    │                            LIO / localizer,
                                                    ├──── gRPC ──►  Temporal     system_manager
                                                    └──── SQL  ──►  PostgreSQL
```

## Contents

- [Features](#features)
- [Requirements](#requirements)
- [Quick start](#quick-start)
- [Configuration](#configuration)
- [API](#api)
- [Project layout](#project-layout)
- [Tests and lint](#tests-and-lint)
- [Documentation](#documentation)

## Features

- **Sign-in** — one console account, a JWT on every route but `/health`,
  renewed while the console is open, up to a session cap.
- **Robot** — state, teleop (WebSocket), localization, mode switch / restart,
  safety lock, wifi.
- **Maps** — 3D maps (`octomap.bt`) saved from a mapping run, switched live,
  renamed, deleted, exported / imported as archives; vertices, forbidden zones
  (`keepout.json`), floors.
- **Tasks** — multi-step jobs (`MOVE`, `PATROL`, `CHARGE`, `STANDUP`, `LIEDOWN`,
  `SPEAK`, `WAIT`) run as Temporal workflows: pause / resume / cancel, step
  groups, schedules, templates, history and a CSV report.
- **Streams** — live point cloud, mapping-mode map cloud and telemetry over
  WebSocket; camera (both of an M20's) and speaker over WebRTC.
- **Tools** — text-to-speech, `ros2 bag` recordings, controller / planner
  settings pages.

## Requirements

This is a ROS 2 `ament_python` package (Humble, Python 3.10). It **builds and
tests standalone**, but **running** it needs the rest of the robot stack:

| Needs | Why |
|---|---|
| `syncai_common` | msgs / srvs / actions; fetched with `vcs import < interface.repos` (HTTPS, no credentials) |
| The robot's ROS stack | nav stack, localizer, `robot_state`, system manager — topics, services and actions |
| PostgreSQL | **hard** dependency (vertices, templates, run records); retried 20 × 5 s at startup |
| Temporal | soft dependency (tasks, schedules); `/health` reports `degraded` without it |
| `~/robot_ws/{config,map,record,lib}` | the per-robot INI (`robot_id`, `robot_model`), maps, bags and the WebRTC worker |
| `~/robot_ws/log/stack`, `/var/log/journal` | optional: the log export's two sources |
| `syncai_tts`, `libsyncai_worker.so` | optional: speech and camera; everything else works without them |

## Quick start

### As a container (recommended)

PostgreSQL, Temporal and the robot container must already be up. Only one
backend may run per robot.

```bash
# once per host: the robot model's camera file (g23 or m20), read with the base
echo 'COMPOSE_FILE=docker-compose.yml:docker-compose.g23.yml' >> .env
docker compose up -d --build        # or pull ghcr.io/chungweeeei/syncai-backend:dev
docker compose logs -f
curl http://127.0.0.1:3000/health   # always 200; status is ok or degraded
```

Every other route needs a token from `POST /api/v1/auth/login` (see [API](#api)).
Set `AUTH_PASSWORD` and `JWT_SECRET` before a robot leaves the bench.

### Inside a colcon workspace

With this repo at `<workspace>/src/syncai_backend`, from the workspace root:

```bash
pip install -r src/syncai_backend/requirements.txt
vcs import < src/syncai_backend/interface.repos        # syncai_common
colcon build --packages-select syncai_common
colcon build --packages-select syncai_backend --symlink-install
source install/setup.bash

ros2 launch syncai_backend backend.launch.py           # robot_id from ~/robot_ws/config/system.ini
```

Mirrors, image tags, compose knobs (the per-model camera file, the DDS
wiring), `docker run` and the pitfalls (host networking, `ipc: host`) are in
[docs/deployment.md](docs/deployment.md).

## Configuration

Environment variables only, loaded from `.env` in the working directory and
read once at startup — see [`.env.example`](.env.example). There are no ROS
parameters.

| Variable | Default | |
|---|---|---|
| `AUTH_USERNAME` / `AUTH_PASSWORD` | `admin` / `admin` | the one console account; the default password is warned about at startup |
| `JWT_SECRET` | drawn per process | unset, a restart signs everyone out |
| `JWT_EXPIRE_SECONDS` / `JWT_MAX_SESSION_SECONDS` | `3600` / `43200` | a token's life; how long renewals may carry one sign-in |
| `TEMPORAL_ADDRESS` | `127.0.0.1:7233` | Temporal client + worker |
| `POSTGRES_HOST` / `POSTGRES_PORT` | `localhost` / `5432` | database `<robot_id>_db`, created if absent |
| `POSTGRES_USER` / `POSTGRES_PASSWORD` | `syncrobotic` / `syncrobotic` | |
| `SYNCAI_SYSTEM_INI` | `~/robot_ws/config/system.ini` | per-robot INI: `[system] robot_id`, `[system] robot_model` (**required**: `g23`, `m20`; the backend does not start without it, and reports it on the robot state and `/health` for the console to draw the right robot), `[map] name` |
| `TTS_SERVICE_URL` | `http://syncai_tts:8080` | the speech service |
| `SYNCAI_WEBRTC_LIB` | `~/robot_ws/lib/libsyncai_worker.so` | the WebRTC worker |
| `LOG_LEVEL` / `LOG_FORMAT` | `info` / `console` | `json` for a log collector |

`robot_id` scopes everything: it is the ROS namespace, the database name and
the Temporal task queue prefix. Full reference:
[docs/configuration.md](docs/configuration.md).

## API

Interactive docs (OpenAPI): `http://<robot>:3000/docs` (sign in with its
Authorize button). Every route, its
refusal codes and the reasons behind them: [docs/rest-api.md](docs/rest-api.md).

| Area | Prefix |
|---|---|
| Health | `GET /health` (no token) |
| Sign-in | `/api/v1/auth/login`, `/token` (the `/docs` form), `/refresh`, `/me` |
| Tasks, history, route preview | `/api/v1/tasks`, `/active_tasks`, `/task_history`, `/task_steps`, `/route_preview` |
| Schedules, templates | `/api/v1/schedules`, `/task_templates` |
| Robot, teleop, localization | `/api/v1/robot/*`, `/localization/auto`, `/landmarks/detect` |
| Maps, vertices, keepout, floors | `/api/v1/maps`, `/maps/{name}/…` |
| Mapping run | `/api/v1/mapping` |
| Settings pages | `/api/v1/settings/controller`, `/planner` |
| Wifi | `/api/v1/network/wifi/*` |
| Speech | `/api/v1/tts/*` |
| Recordings | `/api/v1/recordings` |
| Log export | `/api/v1/logs/export` |
| WebRTC signalling | `/api/v1/webrtc/*` |
| WebSocket streams | `/api/v1/robot/teleop`, `/robot/pointcloud/stream`, `/robot/pointcloud/map/stream`, `/robot/pointcloud/map/saved/stream`, `/robot/telemetry/stream` |

Every route but `/health` and the sign-ins takes `Authorization: Bearer <token>`;
a WebSocket takes `?token=` and is closed with 1008 without a valid one.

Errors are JSON with `detail` and, where a client should branch, a
machine-readable `code`: 400 bad request, 401 no valid token, 404 not found, 409 conflict, 413
payload too large, 502 a downstream (Temporal or a ROS service) failed.

## Project layout

```
syncai_backend/
├── main.py              the node: wires everything, starts uvicorn and the Temporal worker
├── interfaces/rest/     FastAPI app, routers, schemas, sign-in and its guard
├── services/            domain work that outlives a request (map switch, import, safety lock, …)
├── gateways/            outbound: ROS, Temporal, TTS, WebRTC, recordings, logs
├── repositories/        state: in-memory caches, PostgreSQL CRUD, on-disk catalogues
├── database/            SQLAlchemy engine + models
├── subscribers/         ROS topics → repositories
├── tasks/               the task model and its Temporal contract
├── temporal/            worker, RobotWorkflow, activities
└── helpers/             point clouds, INI, map archives, …
launch/                  backend.launch.py
test/                    pytest suite (one file per router / gateway / repo)
docs/                    the long-form reference
```

Dependencies are wired by explicit constructor injection in `main.py` — no DI
container, no module-level singletons. See
[docs/architecture.md](docs/architecture.md).

## Tests and lint

```bash
python3 -m pytest test/                          # from this repo's root, inside the container
python3 -m pytest test/test_robot_router.py -k whitelist
ruff check .
```

Off the robot, the `dev` image supplies ROS 2 and the dependencies:
`docker build --target dev -t syncai-backend-dev .`. Tests skip rather than
fail without ROS, so a green run outside the container proves little — see
[docs/testing.md](docs/testing.md).

## Documentation

The long-form reference lives in [`docs/`](docs/). It records *why* things are
the way they are; read the relevant page before changing a route or a
subscriber, and keep it in sync when behaviour changes.

| Page | Covers |
|---|---|
| [architecture.md](docs/architecture.md) | process model and threads, shutdown order, layering, map storage (octomap, keepout, floors, archives), `robot_id` scoping |
| [ros-interfaces.md](docs/ros-interfaces.md) | subscriptions and QoS, the map cloud file, action / service clients, publishers, TF |
| [rest-api.md](docs/rest-api.md) | sign-in, every REST / WebSocket route, errors, wire formats, settings pages, recordings, the log export, WebRTC, templates, the run record |
| [tasks.md](docs/tasks.md) | Temporal: step types, retries, groups, pause / resume, continuing a failed task, history, schedules |
| [configuration.md](docs/configuration.md) | every environment variable (sign-in included), the system INI, the loaded map, Postgres |
| [deployment.md](docs/deployment.md) | build and run, in a workspace and as its own container |
| [testing.md](docs/testing.md) | running the suite on and off the robot |
| [gotchas.md](docs/gotchas.md) | things that have bitten before |
