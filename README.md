# syncai_backend

> **Standalone repository of a colcon package.** This repo is the source of
> truth for `syncai_backend`. It is a ROS 2 `ament_python` package that imports
> `syncai_common` (msgs/srvs, including the map srvs `StartMapping` / `SaveMaps`
> / `ResetMapping` / `Relocalize` / `IsValid` that used to live in FAST-LIO2's
> `interface` package). It is named in `interface.repos`, so `vcs import < interface.repos`
> from a colcon workspace root fetches it over HTTPS with no
> `SyncAI-Robot-Workspace` checkout and no credentials; see the `Dockerfile`.
>
> **Running** it is the other half and still needs the rest of the robot stack
> around it: the nav stack's topics and services, Postgres, and the workspace
> laid out at `~/robot_ws` (see *Configuration*). On a robot it is vcs-imported
> into `SyncAI-Robot-Workspace/src/syncai_backend`, and the workspace-relative
> paths below (`src/syncai_backend/…`, the workspace `CLAUDE.md`, `.env`) assume
> that placement.

The robot's application-layer process: a **FastAPI REST/WebSocket server and an
rclpy ROS 2 node running inside one Python process**, plus a **Temporal worker**
that executes multi-step tasks.

It is the whole operator-facing surface of the robot. Everything a client needs
— robot state, the map, the point clouds, task submission, mode switching, wifi
setup, speech — is served from here, and every ROS interaction (nav goals,
motion keys, wifi services, `switch_mode`, `save_maps`) happens on this side of
the boundary.

```
                    HTTP :3000 / WebSocket
  any API client   ────────────────────────►  syncai_backend  ──── ROS 2 ────►  nav stack,
                                                    │                            robot_state,
                                                    │                            LIO / localizer,
                                                    ├──── gRPC ──►  Temporal     system_manager
                                                    └──── SQL  ──►  PostgreSQL
```

## Process model

`main.py` builds one `rclpy` node (`syncai_backend_node`) and starts two extra
threads from inside its constructor:

| Thread | What runs there |
|---|---|
| main | `MultiThreadedExecutor.spin()` — all ROS subscriptions, TF, service/action clients |
| uvicorn (daemon) | The FastAPI app on `0.0.0.0:3000` |
| Temporal worker (daemon) | Polls `<robot_id>.ROBOT_TASK_QUEUE`, runs `RobotWorkflow` + activities |

Two consequences worth remembering:

- The executor is **multi-threaded on purpose**. Each point-cloud callback
  (`body_cloud`, and the multi-MB PCD reads behind `pgo/map_cloud_file`) sits
  in its own `MutuallyExclusiveCallbackGroup` so a busy cloud frame cannot
  starve the `robot_state` / telemetry / TF callbacks — or each other.
- REST handlers that block — ROS service calls (up to ~70 s for wifi), psycopg2
  queries, OccupancyGrid→PNG encoding — are declared as **plain `def`, not
  `async def`**, so FastAPI runs them in its worker thread pool instead of
  stalling the event loop. Keep that distinction when adding endpoints.

## Layering

The layering is a convention, not something tooling enforces:

```
interfaces/rest/routers/   HTTP + WS surface; pydantic schemas; no business logic
        │
services/                  domain work that outlives a request: gridmap_conversion owns the
        │                  two recipes, the registry of conversions running right now, and
        │                  the gridmap.recipe.json protocol they write; mode_restart holds the
        │                  latest restart_mode dispatch and how it ended
        │
gateways/                  outbound integrations: ROS (robot, map), Temporal (workflow),
        │                  speech (tts: HTTP to the syncai_tts container), camera/speaker
        │                  streams (webrtc: a dlopen'd Go worker), bags (recording: a
        │                  supervised `ros2 bag record` child) — the last three hold no
        │                  ROS handle at all
repositories/              state stores: in-memory caches, PostgreSQL CRUD, and the two
        │                  on-disk catalogues (map/, record/)
        │
database/                  SQLAlchemy engine + ORM models

subscribers/               ROS topics → repositories (the ingest side) — with one exception:
                           the map cloud's topic only names a PCD, which the subscriber
                           reads from the /dev/shm both containers share
temporal/                  worker, RobotWorkflow, activities
helpers/                   occupancy_grid (OccupancyGrid→PNG), pointcloud (read a binary
                           PCD, downsample / transform / pack), pgm, pcd_to_gridmap
                           (z-band recipe), keepout (forbidden-zone mask rasteriser),
                           traversable (traversability recipe), system_config (INI reader),
                           map_archive (checksummed zip / tar.gz of a map directory),
                           move_guard (a run's TaskMap against the loaded map)
```

`gateways/tts` is a gateway like `robot` / `map`, but its downstream is neither
a ROS service nor anything in this process: it is an HTTP client for the
**syncai_tts container** (the `SyncAI-TTS` repo), which owns the kokoro session
and the speaker.

It used to be the engine itself, and one instance was load-bearing — its
internal lock was the only thing keeping a scheduled `SPEAK` step and a manual
`POST /api/v1/tts/speak` off the speaker at once. That guarantee moved into the
service, in front of the single piece of hardware, which is what lets the
Temporal worker become its own process later without two locks in two processes
failing to see each other. The service also serialises as a FIFO queue rather
than a lock, so utterances come out in the order they were accepted, and refuses
a backlog past its `TTS_MAX_QUEUE` — which this side reports as a 409, not a
502. Along with the engine went `onnxruntime`, `kokoro-onnx` and a ~310 MB model
that no longer sits in the rclpy process's address space.

There are **two pcd → gridmap recipes** in `helpers/`, and the default is
z-band: `pcd_to_gridmap.py` slices the cloud into floor / obstacle height bands
(trinary occupied / free / unknown) and then reverts free cells not connected to
the keyframe trajectory back to unknown. Since 2026-09 those bands are offsets
from the **local** floor: the floor is measured around every keyframe in
`poses.txt` and each point is banded against the floor near it
(`local_floor_levels` / `flatten_to_local_floor`), because a LIO map of a large
venue can drift a metre in z across the site (0917_TP1F_test1: keyframe z from
-0.48 to +0.51) and one floor level for the whole cloud then puts half the
venue's floor in the obstacle band. `floor_reference: global` on `grid/convert`
restores the single-level behaviour, and `local` falls back to it — recorded
as `floor_reference_fallback` in the sidecar — when there is no readable
`poses.txt`. `traversable.py` segments the floor by
intensity / normal / height and projects it, leaving **no unknown cells** — it
runs only when an operator asks for it via `grid/convert`. There is deliberately
no automatic pick between them; the workspace `CLAUDE.md` ("Backend
architecture") records why and what each conversion writes to disk.
`traversable.py` is the **only** module that imports open3d, and nothing imports
it at module scope — `GridmapConversionService.start` imports it inside the
conversion thread's `try`, so a backend start never pays the ~100 MB import and
an `ImportError` lands as a per-map failure instead of a bare thread traceback.
Keep `pcd_to_gridmap.py` open3d-free.

The conversion itself lives in `services/gridmap_conversion.py`, not in the map
router: the recipes, the in-process registry that refuses a second concurrent
conversion of the same map, and the sidecar protocol below are one piece of
domain behaviour, and none of it is HTTP. `main.py` builds one instance and
hands it to the router, which validates requests against it and turns its
refusals into status codes.

**A conversion's outcome lives on disk, in `gridmap.recipe.json`.** The thread
writes that sidecar three times — `status: converting` before it starts any
work, then `ok` (with the recipe, its parameters and the area diagnostics) or
`failed` (with the pipeline's own `error` and a `hint`) on the way out — and
`MapCatalogRepo` reads it back so the catalogue can report `grid_status` /
`grid_error` per map. Disk rather than a job table or an in-memory registry
because the record has to outlive both the thread and the **process**: the
conversion takes tens of seconds, `switch_mode` tears down the byobu session
this backend is a pane of, and the service's in-memory registry dies with it.
That registry is still the authority while the process is up, and a sidecar left
saying `converting` with no entry in it is what the router reports as
`interrupted`. Before this existed a failed conversion was indistinguishable
from a map nobody had converted, and its reason existed only in
`log/stack/<robot_id>/backend/current`.

**Forbidden zones live beside the gridmap as a keepout mask.** Since 2026-09 the
planner's global costmap runs a nav2-style `KeepoutFilter` over the
`OccupancyGrid` that a second map_server instance, `filter_mask_server`, serves
from `map/<name>/keepout.yaml` + `keepout.pgm`. The backend writes that pair
from polygons the console draws in map-frame metres (`PUT
/api/v1/maps/{name}/keepout`), and keeps the polygons themselves in
`keepout.json` — **that file is the source of truth**, the pgm/yaml are derived
from it by `MapCatalogRepo.write_keepout`, because a mask cannot be turned back
into the polygons the editor needs. `helpers/keepout.py` does the rasterising
(cv2, open3d-free) in the gridmap's exact geometry: row 0 is max y, corners are
snapped to cell centres on the console side and `floor`ed here, the fill is
boundary-inclusive. **The background is 205 (unknown), never 254 (free)**: the
filter skips unknown mask cells but lets a *free* mask cell overwrite an
*unknown* costmap cell, so an all-white mask would make every unexplored cell
inside the map's bounding box plannable. The nav session's
`costmap_filter_info.launch.py` writes a blank pair of the same 205 when it boots
a map that has none (workspace `c8d2558`) and never touches an existing one, so
a map that has been booted into always has the pair, and a pair without a json
means "no zones drawn here yet" (or a GIMP-drawn mask, which the next save
overwrites). Write order is pgm → yaml → json — whoever sees the yaml finds the
image it names, and a crash mid-write leaves the robot enforcing more than the
console shows, not less. Nothing in the three files names the map, so rename and
delete carry them along untouched. Not done yet: re-rasterising the mask when
`grid/convert` changes the gridmap's geometry — the filter looks cells up by
world coordinate so the old mask still applies where it has cells, and the next
save rewrites it from `keepout.json`.

**Maps travel as archives.** Since 2026-10 a map can leave the robot as one
`.zip` / `.tar.gz` (`GET /api/v1/maps/{name}/export`) and arrive on another
(`POST /api/v1/maps/import`). `helpers/map_archive.py` owns the format: the
directory's files relative to the map root, plus a `syncai_map.json` manifest
with an md5 per file, the map's name and its vertices. The manifest is archive
metadata — it is never extracted into the map directory and never lists itself,
so a round trip leaves the directory byte-identical and a re-export has nothing
to exclude. The import side **inspects before it extracts**: every member's name
and type and the manifest's file set are checked without writing a byte, and
only the files the manifest lists are then streamed out, hashed as they land.
Nothing ever calls `extractall` (on Python 3.10 it honours absolute names and
creates links). Unpacking happens in `map/.import-<token>/` and a replaced map
is parked as `map/.import-old-<token>/` — both *inside* `map/`, because it is a
bind mount and a sibling path would make the one-`rename` publish a copy across
filesystems. The `.import-` prefix is what hides them: `list_maps` skips it and
`resolve_dir` refuses it, so no route sees a half-unpacked map. An `.import-*`
directory left behind by a crash can simply be deleted (`.import-old-*` is the
previous version of a map, recoverable by renaming it back).

Wiring is explicit: `main.py` constructs every repo/gateway/subscriber and passes
them down as constructor arguments. There is no DI container and no module-level
singleton — if a router needs something, it arrives through
`init_<x>_router(...)`.

## robot_id, namespaces, and per-robot isolation

The launch file reads `[system] robot_id` from the system INI — by default the
**absolute** `~/robot_ws/config/system.ini` (bind-mounted per robot from
`config/instances/robotNN.ini` inside the container), overridable with the
`system_config:=` launch argument — and uses it as the **node namespace**. The
node then reads it back out of its own namespace:

```python
robot_id = self.get_namespace().strip("/") or "default_robot"
```

That single value scopes three things:

| Scoped by robot_id | Value |
|---|---|
| ROS topics/services/actions | relative names inherit the `/<robot_id>` namespace |
| PostgreSQL database | `<robot_id>_db` (auto-created on first connect) |
| Temporal task queue | `<robot_id>.ROBOT_TASK_QUEUE` |

**All ROS names in this package are relative** (`map`, `robot_state`,
`navigate_to_pose`, `pointlio/body_cloud`). Never hardcode `/<robot_id>/…` — a
subscriber with an absolute topic name is a bug that has already been fixed once
here.

TF frame names are *not* namespaced by ROS, so the cloud subscriber takes the
source frame from the message header and only pins the target frame (`map`).

## ROS interfaces

**Subscriptions**

| Topic | Type | QoS | Goes to |
|---|---|---|---|
| `robot_state` | `syncai_common/RobotState` | BEST_EFFORT, depth 3 | `RobotRepo` → `GET /api/v1/robot/state` |
| `odom` | `nav_msgs/Odometry` | BEST_EFFORT, depth 5 | composed with TF `map→odom` → telemetry WS |
| `motor_states` | `syncai_common/MotorStates` | BEST_EFFORT, depth 5 | reduced to `{joint: radians}` → telemetry WS |
| `plan` | `nav_msgs/Path` | BEST_EFFORT, depth 1 | thinned to ≤512 xy pairs → telemetry WS |
| `pointlio/body_cloud` | `sensor_msgs/PointCloud2` | BEST_EFFORT, depth 5 | TF→`map`, thinned, packed → WS `pointcloud/stream` |
| `pgo/map_cloud_file` | `std_msgs/String` (JSON notice) | RELIABLE, **TRANSIENT_LOCAL**, depth 1 | names a PCD in the shared `/dev/shm`; read, stride-capped, packed → WS `pointcloud/map/stream` (mapping mode only) |
| `pgo/mapping_status` | `syncai_common/MappingStatus` | RELIABLE, **TRANSIENT_LOCAL**, depth 1 | pgo's run state (`IDLE` / `MAPPING` / `RESETTING`, keyframe and loop counts) → `MappingStatusRepo` → `GET /api/v1/mapping`, and the 409s on the run routes. Aged out after 5 s of silence; `unknown` outside a mapping session |

Every subscription here is BEST_EFFORT — except pgo's two latched control
messages, the map-cloud notice and the run status, which are discussed below —
`plan` included. Its
publisher is a
`rclcpp::QoS(1)` — RELIABLE — and a BEST_EFFORT subscriber still matches a
RELIABLE publisher, so the topic connects either way; what the request gives up
is retransmission. That costs more on this topic than on the others: they read
20 Hz feeds where the next sample is 50 ms behind the one that was dropped,
while a plan arrives once per BT replan (~3 s), so a dropped one leaves the
operator looking at a route the robot has already left until the next replan.

Two properties of `syncai_planner`'s publisher are worth knowing before debugging
a missing route: it skips the publish entirely while nothing is subscribed, and
its QoS is VOLATILE, so there is no last-value replay. After a backend restart
mid-run the route is blank until the next replan.

The saved map is *not* subscribed. `map` and `localizer/map_cloud` used to be
(both TRANSIENT_LOCAL, to match their latched publishers), but the map endpoints
read the files on disk on request now — see the note at the top of
`routers/map.py`. The one map-shaped cloud that **is** consumed live is pgo's
merged "map so far" during a MANUAL (mapping) session, and it arrives as a
**file, not a topic payload**. pgo writes each merge as a binary PCD to
`/dev/shm/syncai_pgo/<robot_id>/map_cloud_<seq>.pcd` (tmp + rename, newest two
kept) and publishes a ~200 B JSON notice naming it on `pgo/map_cloud_file`;
`MapCloudSubscriber` reads the file with the same `read_pcd_xyz` the map
endpoints use. The reason is a size cliff, not taste: a large floor at pgo's
0.2 m voxel is ~16 MB per merge (44.7 MB at full resolution — 2.79 M points,
2026-09), CycloneDDS sends that over UDP on `lo` as tens of thousands of
datagrams in one burst, the kernel's default receive buffer
(`net.core.rmem_max`, 208 KB) overflows, fragments drop, and a BEST_EFFORT
reader loses the whole sample — the preview simply stopped once the map grew.
Raising the sysctl is host state on every robot and only moves the cliff; a
tmpfs write has none. The `PointCloud2` on `pgo/map_cloud` still exists for
rviz, subscriber-gated as before, and nothing here reads it.

Both containers must see the same `/dev/shm` for the path in the notice to
mean anything, which is what `ipc: host` on this compose service **and** the
workspace's robot service is for (a private container `/dev/shm` is 64 MB and
would not hold two merges anyway). Without it the notices arrive and every
read is `ENOENT`, logged as a warning, and the preview never updates.

The notice is the one subscription here that is RELIABLE and
**TRANSIENT_LOCAL** (depth 1, matching pgo's publisher exactly — a VOLATILE
reader would match but never get the replay): latching 200 bytes is free, and
it means a backend (re)started mid-mapping draws the current map at once
instead of waiting for the next keyframe. Every notice is a complete
loop-closure-corrected replacement of the last, so the subscriber uses depth 1
(a queued older merge is never worth delivering), skips TF (the points were
placed with corrected global poses at merge time) and skips voxel
downsampling (pgo already voxelised at its publish resolution); `cap_points`
stays as the wire-size guard. pgo publishes subscriber-gated, so this
subscription is what un-gates it. The topic simply does not exist under
`AUTO`, which is why the stream is silent on a navigating robot. An **empty**
notice (`points: 0`, `path: ""`) is a message rather than a non-event: pgo
publishes one from `reset_mapping` to say the map has been discarded, and it
is the only thing that stops a console drawing a map that no longer exists, so
the subscriber clears its slot on it instead of skipping it — and, being
latched, it replaces the notice that named the files the reset deleted. Since
2026-10 a **successful `save_maps` sends the same notice**: pgo ends the run
(it goes idle, frees its keyframes) and empties `/dev/shm` itself, so the
"map so far" layer clears on a save and **this process never deletes a file
there**. (A
NaN-only file lands on the same clearing path; both mean "nothing to draw".)
That signal travels the topic on purpose, so it reaches every dashboard and
rviz alike rather than only the client that pressed the button — which is why
the reset's REST handler does not touch the repo itself. Every other failure
in that callback — a file pgo already pruned, bad JSON, a path outside
`/dev/shm`, a file that is not a PCD — is a warning that leaves the slot as it
was: an exception out of a callback would end the executor and the process.

**`RobotState` carries more than `GET /api/v1/robot/state` exposes.**
`motor_status`' kinematic half (`q` / `dq` / `ddq` / `tau_est`), its source
`timestamp` and `localization_valid` are there for operators only.
`routers/robot.py` names its response fields one by one, and that is the *only*
thing keeping them out of a frozen public payload.

So that list is a **whitelist, not a mirror**: a field added to the message does
not appear in the response until somebody decides it should. `low_level_mode` is
the one field that decision has been made for — the gait controller's own state
machine, which the console has no other way to read because
`set_motion_key` / `set_policy_mode` are one-way UDP whose 200 only means a
datagram went out. It is decoded to **labels only** (`PPO` / `LOCOMOTION` / …, with `UNKNOWN` for a
code this backend cannot name). Motion `8` is `IDLE` — the motors are not
driven by any controller — and is labelled rather than left to the fallback
because the console ends a commanded MPC when the motion returns to it, and
MPC's own motion code is unknown and reads `UNKNOWN`. The controller's raw
integers stay on the `robot_state` topic, so `ros2 topic echo
/<robot_id>/robot_state --field low_level_mode` is what tells one unmapped code
from another.

`RobotStateSubscriber` **drops samples whose `localization_valid` is false**
before they reach `RobotRepo`. The publisher now emits on every tick, including
before the localizer has been relocalized, where `localization_status` is zeroed
rather than a real pose. Without that guard the endpoint would return 200 with
the robot apparently parked on the map origin instead of the 404 a client can
gate on.

**Action client:** `navigate_to_pose` (`nav2_msgs/NavigateToPose`) — served by
`syncai_task_runner`. `RobotGateway` keeps a goal-id → `MoveGoal` table so an
activity can poll status and cancel.

**Service clients.** `RobotGateway`: `scan_wifi`, `connect_wifi`, `switch_mode`,
`restart_mode` (`syncai_common/srv`, served by `syncai_sys_manager`) and `set_motion_key` /
`set_policy_mode` (`syncai_common/srv`, served by `syncai_driver_manager`).
`MapGateway`: `map_server/load_map` (`syncai_map_server`, so an edited or
re-converted gridmap reaches the running map_server),
`filter_mask_server/load_map` (the same executable run as the keepout pane, so
a saved forbidden-zone mask reaches the planner's `KeepoutFilter` without a
restart; the mask topic is latched, and a map *switch* has to call it too
because the launch derives the mask path once at boot), `pgo/save_maps`
(`syncai_mapping`'s `pgo_node`, the only thing that serialises a mapping run),
`pgo/reset_mapping` (the same node, the only thing that un-does one),
`pgo/start_mapping` (the same node; what moves it from idle to mapping — a
mapping session comes up banking nothing, and a successful save returns it to
idle) and `relocalize` / `relocalize_check` (`syncai_localizer`, the map switch). The
last two are **bare** names in the robot namespace — `/<robot_id>/relocalize`,
not `/<robot_id>/localizer/relocalize`; `gateways/map/map.py` records why. The map
gateway is separate on purpose — the map router has no business holding a
handle that can command the robot to move.

**Publishers:** `initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, the
localization seed) and `cmd_vel` (`geometry_msgs/Twist`, the teleop channel's
output — also where the zero-velocity watchdog publishes).

**TF:** a `TransformListener` with `spin_thread=False` (it rides the node's own
executor rather than spawning another GIL-contending thread), used only to bring
`body_cloud` into the `map` frame.

## REST API

Interactive docs are generated by FastAPI at `http://<robot>:3000/docs`.

| Method | Path | Notes |
|---|---|---|
| GET | `/health` | Liveness probe (always 200); `status` is `degraded` while `task_server` is not `running` (`connecting`/`dead` + `task_server_error`) |
| POST | `/api/v1/tasks` | Start a `RobotWorkflow`; body `{id, steps[], kind?, name?, map_name?}` (a legacy `timestamp` field is ignored). `kind` (`goal` / `standup` / `liedown` / `task`; `schedule` is the backend's and is refused) and `name` are stamped on the run as search attributes for the history. A job with a MOVE step is checked against the loaded map — 409 `map_mismatch`, 409 `conversion_running`, see *A job's map is checked twice*; stand / lie down are never refused for either. 409 `task_running` while any task is already running — one robot does one task at a time |
| GET | `/api/v1/tasks/{id}` | Overall status + per-step state (workflow query). A held run answers `status: PAUSED` with the step it is holding at `PAUSED` — the only place a hold is visible, see *Task orchestration* |
| DELETE | `/api/v1/tasks/{id}` | Request cancellation; answers `status: CANCELING` — the final state (possibly still `COMPLETED`) comes from GET |
| POST | `/api/v1/tasks/{id}/pause` · `/resume` | Hold / release the running task. Both are *requests* (workflow signals): pause answers `status: PAUSING`, resume `IN_PROGRESS`, and the hold itself is read back from GET. A `MOVE` is interrupted at once and re-sent on resume, a `WAIT` freezes its countdown, a `SPEAK` or posture step finishes first — see *Task orchestration*. 409 `task_not_running` once the run has closed; 404 for an id that is not this robot's |
| GET | `/api/v1/active_tasks` | What is executing on this robot's Temporal queue right now, *whoever* started it (direct or schedule), with `source` / `schedule_id` / `kind` / `name` / `map_name` — the map the run holds (its `TaskMap`; for a run older than the attribute, the loaded map unless it is a stand / lie down; `null` for a run that drives nowhere). A held run is still listed, still `IN_PROGRESS` — the list cannot see inside the run. Not `/tasks/active`, which would shadow `/tasks/{id}` |
| GET | `/api/v1/task_history` | This robot's **finished** runs, newest close first, one Temporal visibility page per call. Query: `page_size` (1–100, default 20), `page_token` (the previous response's `next_page_token`), `status` (`COMPLETED` / `FAILED` / `CANCELED`), `since` / `until` (close-time bounds; naive is UTC, `until` before `since` is 400), `kind` (`goal` / `standup` / `liedown` / `task` / `schedule`), `name` (exact template name). Rows carry `kind` / `name` / `map_name`, `null` for a run dispatched without them or older than the attribute — no fallback here. Reaches back only as far as the namespace retention — see *Task orchestration* |
| GET | `/api/v1/task_history/stats` | The same runs, counted instead of listed: `total`, `by_status` and `success_rate` (`COMPLETED` / `total`, `null` when nothing finished). Same `status` / `since` / `until` / `kind` / `name` query as `task_history`. One visibility count RPC per call |
| POST | `/api/v1/schedules` | Create a Temporal schedule (cron **or** interval). A cron carrying a `#` of its own, a `CRON_TZ=`/`TZ=` prefix (use the `timezone` field) or `@every` (use `interval_seconds`) is 400 — see *Task orchestration* for why the string has to survive intact |
| GET | `/api/v1/schedules` | List schedules with next run times |
| GET | `/api/v1/schedules/{id}` | Describe one schedule |
| PATCH | `/api/v1/schedules/{id}` | Change when it fires: body `{trigger: {cron, timezone}}` or `{trigger: {interval_seconds}}`, replacing the old rule whole. In place — the id, the frozen steps, the provenance and the paused state stay, and `SKIP` holds across the edit. The id cannot be renamed (delete + create). Same cron restrictions as the create |
| DELETE | `/api/v1/schedules/{id}` | Delete |
| POST | `/api/v1/schedules/{id}/pause` · `/resume` | Pause / unpause |
| POST | `/api/v1/task_templates` | Store a step list so it can be re-dispatched |
| GET | `/api/v1/task_templates` | List, optional `?map_name=` (that map's **plus** the map-independent ones) |
| GET | `/api/v1/task_templates/{id}` | One template, with its vertex references resolved |
| PUT | `/api/v1/task_templates/{id}` | Partial update; `steps` replaces the whole list |
| DELETE | `/api/v1/task_templates/{id}` | Delete |
| POST | `/api/v1/task_templates/{id}/schedule` | Freeze the current resolution into a Temporal schedule. Registers through the same path as `POST /api/v1/schedules`, so the same cron restrictions apply |
| GET | `/api/v1/robot/state` | Latest robot state (pose in degrees, wifi, battery, byobu-session `mode`, and `low_level_mode` — what the gait controller reports); 404 until localization is valid |
| POST | `/api/v1/robot/mode` | `switch_mode` on `syncai_sys_manager`. A real switch kills the byobu session **this backend is a pane of**, so the client usually sees a dropped connection, not a body — treat that as success-in-progress. The body reliably arrives only for the no-op (already in that mode) and a refusal |
| POST | `/api/v1/robot/restart` | `restart_mode` on `syncai_sys_manager`: kill and rebuild the byobu session of the mode already live — the rebuild `/robot/mode` refuses as a no-op. No body. Answers inside the 2 s ack window with `restarting: true`; sys_manager only replies once the rebuild is over, and that reply is kept for the `GET` below. **409 `restart_refused`** with sys_manager's sentence, nothing touched — in MAINTENANCE, with both sessions up, and always in MANUAL (`pgo_node` may hold an unsaved map in RAM), so in practice it restarts AUTO only. **409 `restart_running`** while one is under way; 502 if sys_manager is unreachable |
| GET | `/api/v1/robot/restart` | How the latest restart this process dispatched went: `status` `idle` / `restarting` / `succeeded` / `failed`, sys_manager's `message`, `started_at` / `finished_at`. This, not `/robot/state`, is how a console learns the outcome — the state endpoint keeps serving the last frame through the whole rebuild. Unanswered for 3 minutes reads back as `failed`; in memory, so a backend restarted meanwhile says `idle` |
| WS | `/api/v1/robot/teleop` | Inbound manual-control channel: client sends `{vx, vy, wz}` JSON frames at ~10 Hz; each axis is clamped to [-1, 1] and published as-is (m/s / rad/s — full stick is 1.0). Refused (`{"error": ...}` frame, socket stays open) while an autonomous MOVE is executing. A 0.5 s stale-input watchdog and the disconnect path both publish zero velocity — the driver manager has no cmd_vel watchdog of its own |
| POST | `/api/v1/robot/set_initial_pose` | Seed localization with a map-frame pose (degrees in, radians out); fire-and-forget |
| POST | `/api/v1/robot/set_motion_key` | Gait key `"0"`–`"5"`; `"4"` (ESTOP) is accepted but **not** forwarded — 200 with `sent: false` |
| POST | `/api/v1/robot/set_policy_mode` | Gait-controller policy index; only `0` (PPO) and `1` (HIMLOCO) are accepted |
| GET | `/api/v1/network/wifi/scan` | Scan visible networks (blocks up to 45 s) |
| POST | `/api/v1/network/wifi/connect` | Connect via `nmcli` (blocks up to 70 s) |
| GET | `/api/v1/maps` | The map directories on disk, with geometry, vertex counts and each map's `grid_status` — **this is the conversion-status surface**, there is no job resource and no status endpoint. `none` / `converting` / `ok` / `failed` (reason in `grid_error`) / `interrupted` (the backend was restarted mid-conversion). `grid_converting` is the deprecated boolean it replaces |
| GET | `/api/v1/maps/{name}` | One map's summary |
| POST | `/api/v1/maps` | Save the current mapping run **and end it**: `pgo/save_maps` into a new map dir, then the z-band pcd→gridmap conversion in a background thread. `grid_pending` only means "started" — poll `grid_status` for the outcome. On success pgo returns to idle and clears the "map so far" layer and `/dev/shm` itself, so the next run waits for `POST /api/v1/mapping/start`. 409 `mapping_idle` when nothing has been started (before any directory is created), 409 `mapping_busy` mid start/reset. Mapping mode only in practice — pgo is the producer |
| POST | `/api/v1/mapping/start` | Begin a mapping run: `pgo/start_mapping`, idle → mapping. A mapping session comes up with pgo banking nothing, so the drive to the starting point is not part of any map. No body; always resets the LIO front end (the map's origin is the odometry origin), so the robot must be **stationary** until it returns. 409 `mapping_running` / `mapping_busy`; 502 with pgo's sentence otherwise. Touches no file |
| GET | `/api/v1/mapping` | pgo's run state as it last reported it: `state` `idle` / `mapping` / `resetting` / `unknown`, `key_poses`, `loop_closures`. `unknown` is a real answer — every navigating robot (pgo exists only in a mapping session), and a mapping robot whose pgo has been silent for 5 s. Latched, so it answers after a console reload |
| POST | `/api/v1/mapping/reset` | Discard the run held in the robot's memory and start a new map, restarting nothing: `pgo/reset_mapping` pauses intake, resets the LIO front end and rebuilds the pose graph; pgo stays mapping. No body, and **no save** — saving is `POST /api/v1/maps` and stays a separate act, because the common use is abandoning a run that went wrong early. Touches no file, hence `/mapping/` rather than `/maps/`. 409 `mapping_idle` / `mapping_busy`; 502 with pgo's sentence, and a failure means the graph was left exactly as it was. The robot must be **stationary** — the LIO re-init is static and gravity-aligning |
| PATCH | `/api/v1/maps/{name}` | Rename a map: moves `map/<old>/` to `map/<new>/` and re-keys its `map_vertices` and `task_templates` rows. 409 `map_active` for the map the stack is running on (it opened those files at launch — switch the robot to another map first), 409 `conversion_running`, 409 `name_taken`. Temporal schedule memos keep the old `map_name` label; they are display-only |
| DELETE | `/api/v1/maps/{name}` | Delete a map: `shutil.rmtree` of `map/<name>/` plus its `map_vertices` rows, no undo. Same first two refusals as the rename, plus 409 `template_bound` while any `task_templates` row still names the map (it lists them). Rows are deleted **before** the directory — `rmtree` has no compensation, so the irreversible step goes last. Schedules already registered keep firing their frozen steps |
| GET | `/api/v1/maps/{name}/export?format=zip\|tar.gz` | The map directory as one archive (default `zip`; `Content-Disposition: attachment; filename="<name>.<ext>"`). Members are the map dir's files relative to its root — no top-level folder, `traversable_debug/` left out — plus a `syncai_map.json` manifest: `{format, version: 1, name, exported_at, files: {relpath: md5}, vertices: [...]}`. Vertex **ids are not exported**; they are regenerated on import. Task templates are not included (their steps bind vertex ids). 409 `conversion_running`; the active map may be exported |
| POST | `/api/v1/maps/import?name=` | Create or **replace** a map from such an archive. Raw `application/octet-stream` body, zip or tar.gz told apart by magic bytes. **Nothing is written until the archive is judged**: the manifest is required (version 1; a newer one is refused), every file is md5-verified, and an archive with an unlisted member, a listed member missing, a link/device member, an absolute or `..` path, no `map.pcd`, or an unknown vertex type is a 400. Target name is `?name=` else the manifest's. Replacing takes the delete's refusals — 409 `map_active` / `conversion_running` / `template_bound` — plus 409 `disk_low`. 201 `{name, replaced, files, bytes, vertices_created, vertices_deleted, message}`. No conversion is started; an imported `gridmap.recipe.json` is kept as it came. The staging-and-rename dance is in *Layering* |
| POST | `/api/v1/maps/{name}/activate` | Switch the robot onto this map, live — no session restart: `relocalize` first (the localizer's refusals happen before it mutates anything), then map_server `load_map`, then `[map] name` into the instance INI so the choice survives a restart (`[initial_pose]` is zeroed, so the operator re-seeds). `switched: false` is the no-op for the map already active. `localized` is a best-effort `relocalize_check` poll — `false` means "not yet" (registration retries indefinitely), `null` means it could not be asked; do **not** substitute `RobotState.localization_valid`, which is TF-presence only. Refusals, all before any mutation: 409 `grid_missing`, `pointcloud_missing`, `conversion_running`, `ini_not_writable`, `task_running` (any job), `tasks_unknown` (refuses rather than assuming idle), `stack_not_ready` (service discoverability, which is how mapping mode is detected — not the cached mode). Lifts the `map_active` refusal on rename and delete. It also hands `filter_mask_server` this map's `keepout.yaml` — blank when the map has never been booted into — because a `load_map` leaves the *previous* map's zones in force; best-effort and after the commit point, so `keepout_reloaded: false` means the switch succeeded but the old zones may still apply (`null` on the no-op) |
| POST | `/api/v1/maps/{name}/grid/convert` | (Re)build the gridmap from `map.pcd` with a chosen recipe (`z-band` default / `traversability`), optional `z_band_offsets` and `floor_reference` (`local` default / `global`) for z-band or `gap_fill_size` for traversability, `debug` for intermediate clouds. `started: true` means the thread launched, nothing more — the outcome arrives as `grid_status`. 409 `conversion_running`, or 409 `gridmap_hand_edited` (confirm with `overwrite_edits`; the edited grid survives as `gridmap_prev.pgm`, its recipe as `gridmap_prev.recipe.json`). Reloads map_server when the map is active |
| GET | `/api/v1/maps/{name}/image` · `/thumbnail` | The gridmap as a full-size / downscaled PNG, content-hash ETag'd |
| PUT | `/api/v1/maps/{name}/grid` | Write edited cells back (raw `application/octet-stream`); reloads map_server when the map is active. 409 `conversion_running` while that map is being converted — the conversion would overwrite the edit |
| GET | `/api/v1/maps/{name}/keepout` | The map's forbidden zones as saved through this API: `zones: [{id, points: [{x, y}, …]}]` in map-frame metres, plus `active`. Read from `keepout.json`, so a map carrying only the blank mask the nav session writes at boot (or a hand-drawn one) answers `[]` |
| PUT | `/api/v1/maps/{name}/keepout` | Replace the map's forbidden zones with the **whole** list in the body (`{zones: [{id?, points: [{x, y}, …]}]}`; ids are kept when given, generated when not). Rasterises them into `keepout.pgm` + `keepout.yaml` in the gridmap's geometry and records the polygons in `keepout.json`; then, when the map is active, `filter_mask_server/load_map` — `reloaded: true` means the planner enforces them now. `[]` **clears** the zones: the mask is rewritten all-unknown and reloaded, and the files stay (deleting them would change nothing in the running filter). A failed or unavailable reload is still a **200** with `reloaded: false` and the reason in `message` — the zones are on disk and apply when the nav stack next boots this map. 400 for a zone under three points, a non-finite coordinate, an empty or duplicate id; 404 for a map with no gridmap; 409 `conversion_running` |
| GET | `/api/v1/maps/{name}/pointcloud` | The saved `map.pcd`, packed binary |
| POST · GET | `/api/v1/maps/{name}/vertices` | Batch-create (single transaction) / list with an optional `?type=` filter. The create is held by a running job — see *A job's map is read-only while it runs* |
| GET · PUT · DELETE | `/api/v1/maps/{name}/vertices/{id}` | Read / partial update / delete. Update and delete are held by a running job, as above |
| GET | `/api/v1/tts/voices` | The voice ids the speech service's model carries |
| POST | `/api/v1/tts/synthesize` | Render `{text, voice, speed}` and return the WAV (`audio/wav`) without playing it |
| POST | `/api/v1/tts/speak` | Same body, played on the robot speaker; blocks for the utterance (`duration` in the response). `unknown voice` is a 400; a full speech queue is a 409 with `code: tts_queue_full`; everything else (the speech service unreachable, its weights missing, a wedged speaker) a 502. Text is English only, ≤1000 chars; `speed` 0.5–2.0 |
| POST | `/api/v1/recordings` | Start `ros2 bag record` into `record/<name>/`. Body `{name?, topics?, compression?}`; `name` defaults to `rec_<UTC timestamp>` and `topics` to the LIO inputs (`livox/lidar`, `livox/imu`). A topic without a leading slash is resolved under this robot's namespace, so `/tf` and `/tf_static` are how fleet-wide topics are asked for. 201 means the recorder survived its liveness probe. Refusals, all before any spawn: 409 `recording_running` (its name is in the message), `name_taken`, `disk_low` (< 2 GB free); 400 for a reserved or malformed name or an empty `topics` |
| GET | `/api/v1/recordings/active` | The live recording with `elapsed_seconds` and `size_bytes`, or `null`. Its own route rather than a filter over the catalogue (the map router's rule) because a 1 Hz poll would otherwise walk every bag on disk; also where a recorder that died on its own is reaped |
| POST | `/api/v1/recordings/stop` | SIGINT the recorder and block until it has flushed: with compression on that includes zstd-compressing the open split, about a minute for a full 2 GB one on the Orin. The recorder is escalated to SIGTERM/SIGKILL only after 15 s with no write under its directory (or 5 min in all), so a slow flush is never cut short — set client timeouts accordingly. `complete: false` means it had to be killed and the bag needs `ros2 bag reindex` — the messages are still there. 409 `not_recording` when nothing is running |
| GET | `/api/v1/recordings` | Every bag under `record/`, newest first, with `status` `recording` / `ok` / `interrupted` and, once finished, `duration_seconds`, `message_count` and the topics actually recorded. A finished bag with `message_count: 0` is the sign of a topic typo — nothing refuses a topic that does not exist yet |
| DELETE | `/api/v1/recordings/{name}` | `shutil.rmtree`, no undo. 409 `recording_active` for the one being written, 404 otherwise |
| POST | `/api/v1/webrtc/whep` | Camera stream to the browser (WHEP): body is the SDP offer (`application/sdp`), answer is **201** with the answer SDP and a **relative** `Location: /api/v1/webrtc/whep/{session_id}`. Preempts a live `video` or `duplex` session (one camera). 400 for an empty, oversized (> 64 KiB), non-UTF-8 or media-less offer; 409 `whep_session_pending` while another create is still mid-handshake (no id to preempt yet — retry); everything else, a missing `.so` included, 502 with the worker's sentence. Blocks for ICE gathering (0.1–5 s) |
| POST | `/api/v1/webrtc/whip` | Browser microphone to the robot speaker (WHIP), same shape; `Location: /api/v1/webrtc/whip/{session_id}`. Preempts `audio` or `duplex` (one speaker); coexists with `video` |
| POST | `/api/v1/webrtc/duplex` | Both directions over one peer connection, same shape; holds the camera **and** the speaker, so it preempts every other session |
| DELETE | `/api/v1/webrtc/{whep,whip,duplex}/{session_id}` | Tear one down. **204 even for an unknown id** — idempotent like the worker, which cannot tell "already self-closed" from "never existed" |
| GET | `/api/v1/webrtc/status` | `library_loaded` plus the `video` / `audio` / `duplex` slots (`session_id`, `age_seconds`) as this backend believes them. There is no callback from the worker, so an abandoned session reads as live until the next create preempts it. Deliberately not part of `/health` |
| WS | `/api/v1/robot/pointcloud/stream` | Live `body_cloud`, ~10 Hz |
| WS | `/api/v1/robot/pointcloud/map/stream` | pgo's merged "map so far" cloud, every few seconds at most, mapping mode only; each frame replaces the whole layer |
| WS | `/api/v1/robot/telemetry/stream` | JSON frames keyed by `type`: `pose` (~20 Hz), `joints`, `path` (~0.333 Hz) |

**A job's map is read-only while it runs.** Vertex create / update / delete,
`PUT …/grid`, `PUT …/keepout` and `POST …/grid/convert` answer 409 `task_running`
while a running job holds the map — its `map_name` in `GET /api/v1/active_tasks`,
the same rule (`ActiveTask.map_in_use`) — and 409 `tasks_unknown` when the running
jobs cannot be listed (refuse rather than assume idle). Each of those routes either
reloads into the running stack or rewrites the rows a job's MOVE steps came from,
so mid-run it would change the map under a robot driving on it. One async FastAPI
dependency (`_refuse_while_a_job_holds`) in front of the plain `def` routes: it
awaits Temporal on the loop and the route still runs in the threadpool. A map
other than the loaded one is let through without asking — activate refuses while
anything runs, so only the loaded map can be held, and editing an idle map should
not depend on Temporal being up. Reads are never held. It is check-then-act, not a
lock: a schedule firing between the check and the write is not stopped, which
`execute_move`'s own check covers for a rebuild. Rename, delete and import-replace
need none of this — they already refuse the loaded map outright (`map_active`).

The telemetry stream is the internal visualization channel and deliberately
shares no models with `GET /api/v1/robot/state` — that payload is a frozen
contract, this one may change shape freely. It is a separate socket from the
point cloud so a 360 kB cloud frame cannot head-of-line block pose; `path` rides
this one because a thinned route is ~8 kB every 3 s. An **empty** `path.points`
is a real sample meaning "no route" — the planner never publishes an empty plan,
so a route is cleared by a TTL in `TelemetryRepo` (arrival, cancellation and
abort are indistinguishable silence from the backend's side).

**Errors.** Routers raise domain exceptions from `exceptions.py`; handlers
registered in `server.py` map them to HTTP:

| Exception | Status |
|---|---|
| `NotFoundError` | 404 |
| `BadRequestError` | 400 |
| `ConflictError` | **409**, with an optional machine-readable `code` next to `detail` — e.g. `conversion_running` / `gridmap_hand_edited` on the re-convert route, `map_active`, `restart_refused` / `restart_running`, `recording_running`, `tts_queue_full`, `whep_session_pending`; each route's row above lists its own. A client branches on the code (the re-convert route's confirm-and-retry is only for `gridmap_hand_edited`), and matching on prose that exists to be reworded was the rejected alternative |
| `UnauthorizedError` | 401 |
| `UpstreamError` | **502 Bad Gateway** (these all mean a downstream — Temporal or a ROS service — failed) |

**Point-cloud wire format** (both WS streams and `GET /api/v1/maps/{name}/pointcloud`):

```
[ uint32 LE point_count ][ float32 LE x, y, z ] * point_count      # map frame
```

A browser client can read this straight into a typed array. Both WS
streams share one `_pump`: it is **frame-driven**, waiting on the single-slot
repo's notification rather than polling (polling cost ~50 ms of queueing latency
and ~5 % dropped frames from two unsynchronised 10 Hz clocks beating), and the
single-slot repo still does the dropping, so a slow client sees the newest frame
instead of a backlog.

### Recordings

Bags land in `~/robot_ws/record/<name>/` (gitignored), the same layout `ros2 bag
record` writes by hand: `<name>_N.db3` splits cut at 2 GB plus a `metadata.yaml`.
As its own container that directory is bind-mounted from `record/` beside the
compose file rather than from the workspace, because — unlike `config/` and
`map/` — nothing outside this process touches it; `RECORD_DIR` points it at a
bigger disk. Nothing else in the stack reads them — a bag is
insurance, and the thing it insures against is a mapping run that ends without a
save, since `pgo_node` holds its keyframes in RAM and replaying `livox/lidar` +
`livox/imu` is the only way to get one back.

Two properties of the recorder are load-bearing and easy to undo by accident:

- **It is not in its own session.** The child shares the backend's process
  group, so a `switch_mode` — which kills the byobu session this backend is a
  pane of — takes it down too. Given its own session it would outlive the
  backend, keep writing, and be unstoppable through any route, because the only
  handle on it is a slot in memory.
- **Its output is inherited, not piped.** A pipe nobody drains deadlocks the
  recorder at 64 KiB; inherited, `rosbag2_recorder`'s lines land in the backend's
  multilog (`log/stack/<robot_id>/backend/current`).

**A topic is only recorded if its type is installed here.** `ros2 bag record`
subscribes generically, but it still looks the type's package up in the ament
index and loads its `rosidl_typesupport_cpp` library; a type it cannot find is
skipped with a warning in the log, and the bag simply has no such topic.
`syncai_common/msg/*` is covered by the install space the backend runs from.
`livox/lidar` is a `livox_ros_driver2/msg/CustomMsg` (`xfer_format: 1` in the
bringup), and the real driver package needs Livox-SDK2 to build, so the image
builds `docker/livox_ros_driver2` instead — the two messages and nothing else,
under the same package name and with byte-identical `.msg` files, because DDS
matches on the type name and the bag records it for replay against the real
driver. A type from any other package needs the same treatment. Running from a
robot workspace instead of the image, the real driver's install space already
provides it.

`interrupted` in the catalogue is derived, never stored: a directory with no
`metadata.yaml` and no live process behind it. That is what a bag looks like
when the backend went away mid-recording, and it is the same
disk-outlives-the-process split as the gridmap conversion sidecar.

### WebRTC

`/api/v1/webrtc/*` is signalling only. The media path — capture, H.264 encode,
pion — lives in `libsyncai_worker.so`, built in the `SyncAI-WebRTC-Worker` repo
and copied in by hand (see *As its own container*), which `gateways/webrtc`
dlopens. That is the one FFI in this process, and it is not free: loading it
installs the Go runtime's signal handlers into a process that already holds
rclpy and CycloneDDS, a Go c-shared library cannot be `dlclose`d, and a fault
inside it takes the whole backend down. Hence the **lazy load** — construction
touches nothing and the first create pays for it, so a boot that never streams
never carries any of the above.

- **One slot per kind, arbitrated by device.** There is one camera and one
  speaker: `whep` holds the camera, `whip` the speaker, `duplex` both. A create
  preempts every live session whose devices overlap its own, so a browser tab
  that died without sending DELETE never locks the camera out. The worker knows
  none of this — left alone it would build a second session and fail deep
  inside GStreamer with an ALSA or V4L2 code.
- **Handlers are plain `def`** and the offer arrives as a `bytes` body
  parameter rather than through `Request.body()` (a coroutine), because a
  create blocks in a cgo call until ICE gathering completes; an async handler
  would stall the telemetry, point-cloud and teleop sockets meanwhile.
- **Neither of the gateway's two locks is held across a call into the worker**,
  so a DELETE is never queued behind a multi-second create.
- A camera held by something else (e.g. `scripts/publish_camera_crop.sh`) is a
  plain 502 with the worker's sentence and no `code`: it cannot be told apart
  from any other pipeline failure in the Go error string.

### Vertex vs. MapPoint

The REST vocabulary is **"vertex"** with a `VertexType` enum
(`GENERAL` / `ARTIFACT` / `CHARGER` / `HOME` / `WAITING`), while the ORM model and
repository still say `MapPoint` (table `map_vertices`). The mismatch is
intentional — no migration was done. `type` is validated at the REST boundary and
stored as a plain string.

### Task templates

`POST /api/v1/tasks` creates *and dispatches* and persists nothing, and Temporal
is not a library: namespace `default` keeps closed workflows only for its
retention (a day on a fresh install — see *Task orchestration* for lengthening
it) with no archival, so a dispatched step list is gone once that passes.
`task_templates` is
where the operator's re-dispatchable step lists live. The prefix is
`/api/v1/task_templates`, not `/api/v1/tasks/templates`, because the latter
would collide with `/api/v1/tasks/{id}` (see the include order note in
`server.py`).

- **`steps` is one JSON column, not a child table.** This package has no
  migrations — the schema is whatever `create_all` produced — so a column list is
  a shape that can never be altered again, while a JSON array can grow an optional
  key. It is also the only place the step *order* is recorded, and a child table
  would still be rewritten whole on every edit, because that is what editing a
  step list is. **Forward-compat rule: only ever add optional keys to a stored
  step; never rename, retype, or repurpose one.**
- **A template's MOVE step keeps both a `vertex_id` and a `params` snapshot**, and every
  read reports `resolved_params` — the vertex's *current* pose when it still
  exists (`vertex_status: CURRENT`), the snapshot when it does not (`MISSING`).
  Moving a dock on the map therefore updates every template that references it.
  Resolution is server-side so the rule has one implementation; the client
  dispatches by sending `resolved_params` through the ordinary `POST /api/v1/tasks`.
- **Map scoping keys off "does it contain a MOVE", not "does it reference a
  vertex"** — a hand-typed `(x, y, theta)` is in a map's frame just as much as a
  vertex is. Any MOVE step ⇒ `map_name` required; no MOVE step ⇒ `map_name` must be
  absent, and the task runs anywhere. A template whose map is not the active one still
  saves (authoring for a map you are about to load is legitimate) and is reported
  with `map_matches_active: false` for the client to gate on.
- **Cross-field rules answer 400 with a sentence**, not 422 with a validation
  array: the array is unreadable to an operator, and a `PUT` may conflict with the
  *stored* row rather than with its own body, which no request-schema validator can
  see.

## Task orchestration (Temporal)

A task is an ordered list of steps. `RobotWorkflow` walks them one at a time and
dispatches by `StepType`:

| StepType | Activity | What it does |
|---|---|---|
| `MOVE` | `execute_move` | Send a `NavigateToPose` goal, poll to a terminal state, heartbeat every 0.25 s |
| `STANDUP` / `LIEDOWN` | `execute_stand` / `execute_lie_down` | Send the motion key; fire-and-forget (see the note in `activities.py`) |
| `SPEAK` | `execute_speak` | `TtsGateway.speak()` — synthesise and play on the robot speaker, blocking for the utterance. `SpeakParams`: `text` (1–1000 chars, English only), `voice` (default `af_heart`; list at `GET /api/v1/tts/voices`), `speed` (0.5–2.0) — the same constraints as the REST route, because both drive one gateway |
| `WAIT` | — (workflow timer) | Do nothing for `WaitParams.seconds` (0 < s ≤ 3600) before the next step. Not an activity: a durable Temporal timer inside `RobotWorkflow._run_wait`, so it takes no slot of the one-thread activity executor, needs no heartbeat, and survives a backend restart with the time already served |

(`ARTIFACT` — conveyor pickup/drop over the artifact backend's REST API — was
removed in 2026-08 along with `gateways/artifact/`; task templates or schedules
that still carry an ARTIFACT step must be purged before deploying.)

Details that matter when editing this path:

- **Activities are synchronous** and run in a single-worker `ThreadPoolExecutor`,
  matching the one-thing-at-a-time reality of a robot. The worker also declares
  `max_concurrent_activities=1`, so Temporal holds a second activity server-side
  rather than handing it over to queue behind the thread with its timeouts
  already ticking. On cancellation Temporal
  *throws* `CancelledError` into the thread wherever it happens to be (it
  lands once a blocking call such as `time.sleep` returns), so cleanup lives in
  an `except CancelledError:` block, not in an `is_cancelled()` poll. `execute_move` wraps the whole of the send and
  the poll loop in one `except CancelledError` that calls
  `cancel_active_moves()` under `activity.shield_thread_cancel_exception()`, so
  the goal is really cancelled before the activity dies. By goal *state* rather
  than by id, because the cancel can land inside `move()` before nav2 has
  answered — and for the goal nav2 accepts a moment after that, the gateway
  itself disowns it: whichever of the two threads (the waiter, the rclpy
  response callback) is second sees what the first did and cancels.
- **A cancel only reaches an activity on a heartbeat.** The server never pushes
  it: it answers the worker's next heartbeat with "cancel requested", and the
  worker then throws `CancelledError` into the thread. So how long a paused or
  cancelled MOVE keeps driving is the heartbeat *send* interval plus the poll
  sleep. The SDK throttles sending to 0.8 × `heartbeat_timeout` (2.4 s under
  the 3 s MOVE timeout) by default; the worker caps it at
  `HEARTBEAT_THROTTLE_MAX` (0.5 s, `temporal/worker.py`) and `_wait_for_nav_goal`
  polls every `NAV_POLL_INTERVAL_S` (0.25 s), so nav2 hears the cancel within
  about 0.75 s instead of ~3.5 s. The heartbeat *timeout* is untouched;
  `test_activities.py` pins the relation between the three.
- **MOVE heartbeats before it sends.** The heartbeat clock starts at activity
  start, and `move()` has no loop to heartbeat from, so its two waits (server
  ready, goal accepted) are bounded by `NAV_GOAL_SEND_BUDGET_S` in the gateway
  and `MOVE_HEARTBEAT_TIMEOUT` in the workflow must stay above it —
  `test_activities.py` pins that. The old 30 s / 10 s waits were unreachable
  under a 3 s heartbeat anyway; a nav2 that is slow to come up gets its chance
  from the retry policy, not from a wait the heartbeat would have killed.
- **`SPEAK` does not heartbeat.** `execute_speak` sits in one blocking gateway
  call — a single HTTP request to the speech service, held open for the whole
  utterance by `wait=true` — so the 3 s `heartbeat_timeout` the other activities
  run under would kill every attempt before its first heartbeat. The workflow
  therefore drops the heartbeat for SPEAK and relies on a **5-minute
  `start_to_close`** alone — much shorter than the heartbeating activities'
  hour, because a dead worker holding a SPEAK step would otherwise go unnoticed
  for that hour. The same fact makes it effectively not cancellable
  mid-utterance: without heartbeats the worker never learns of the cancel, so a
  cancelled task finishes the sentence it is on (a *pause* never tries — it
  waits the step out and holds before the next, see below).

  This is now a **choice, not a constraint.** The speech service's playback is a
  job — POST returns an id, GET reports its state, DELETE stops it — so
  rewriting `execute_speak` to enqueue and poll once a second would make the
  heartbeat real and let `except CancelledError` cut the utterance, exactly as
  `_wait_for_nav_goal` already does for MOVE. It was left blocking so that
  moving speech out of this process changed nothing about the task path.
- **Per-step state is a workflow query** (`get_step_states`), not a database
  table. `GET /api/v1/tasks/{id}` degrades to an empty step list if the query
  fails (no worker polling yet), rather than erroring the whole request.
- **The hold is two workflow signals** (`pause` / `resume`, delivered by
  `POST /api/v1/tasks/{id}/pause` · `/resume`) and a flag in the workflow; both
  are idempotent, so a second pause or a stray resume changes nothing. The run
  checks the flag before every step and, when set, parks that step as `PAUSED`
  in a `wait_condition` until the resume — which is the whole of the hold for
  `SPEAK` and the posture steps: they are never interrupted, they finish and
  the run stops at the next boundary. A `WAIT` freezes its countdown at once
  (its `wait_condition` returns, the step reads `PAUSED`) and on resume waits
  out only the remainder, measured on `workflow.now()`; a cancel during it is
  `CANCELED` / "Task canceled", as during any hold. Only a `MOVE` is cut short: the `pause`
  signal cancels the in-flight activity handle, which lands in `execute_move`'s
  `except CancelledError` and cancels the nav2 goal exactly as a task cancel
  would, shield and all; on resume the step is dispatched again from scratch
  (same target, fresh attempts, from wherever the robot now is). Consequences:
  - **Temporal has no paused status.** A held run is `RUNNING` to the server,
    so the gateway derives the task-level `PAUSED` from the step list the one
    existing query returns (the held step carries it) — the console's poll
    stays one query per read, and if the query fails the answer is honestly
    `IN_PROGRESS`. The REST acks say `PAUSING` / `IN_PROGRESS`, never `PAUSED`:
    like `CANCELING`, they describe the request, and a step being spoken
    finishes before the hold lands. `GET /api/v1/active_tasks`, `_require_idle`
    (so `POST /api/v1/tasks` still answers 409 `task_running`) and the map
    activation gate all keep treating a held run as running — one robot does
    one task at a time, held or not.
  - **Teleop opens up during a hold.** The MOVE's goal is gone, so
    `_autonomous_move_active()` is false and `cmd_vel` is accepted; the resume
    re-sends the goal from wherever the operator drove to.
  - **A cancelled step reads `CANCELED` / "Task canceled"** whatever it was
    doing — held, waiting, mid-MOVE or mid-posture. It used to read `FAILED` /
    "Cancelled" mid-step, when the SDK delivered the cancel wrapped in the
    activity's `ActivityError`; the workflow now folds that shape too.
  - **The MOVE wait never `await`s the activity handle.** `_run_move` waits
    on `workflow.wait_condition(handle.done() or paused)`, so a task cancel
    always reaches the run as `asyncio.CancelledError` (the activity is then
    cancelled and waited out, `WAIT_CANCELLATION_COMPLETED`), and an
    `ActivityError(cause=CancelledError)` can only be the pause's own
    interruption. A pause and a cancel landing in the same breath therefore
    end the run canceled, never held — without depending on
    `workflow.cancellation_reason()`. (`temporalio` is floored at 1.33 in
    `requirements.txt`; the suite pins this behaviour on 1.34.)
  - **Replay of runs started before the hold existed is safe** as long as
    nobody pauses them: the happy path emits the same commands as before (one
    activity per step; `wait_condition` emits none).
- **Task history is Temporal's visibility index, not a table.**
  `GET /api/v1/task_history` lists closed executions on this robot's task queue
  (the same `WorkflowType` + `TaskQueue` scope as `active_tasks`); rows carry no
  steps — a row's detail is `GET /api/v1/tasks/{id}`, which works on a closed run
  too while a worker is polling. Consequences:
  - **It is bounded by the namespace retention.** The auto-setup default is
    one day, and until 2026-10 it was left there on purpose; the history
    dashboard's time-range filter is what made a longer window worth having.
    Nothing in this repo sets it — the Temporal container does. On a running
    stack (the namespace already exists, so an env var does nothing):
    `docker exec temporal sh -c 'temporal operator namespace update --address $(hostname -i):7233 --namespace default --retention 30d'`
    — immediate for runs that close from then on; already-closed runs keep
    their original expiry. For a fresh install, `DEFAULT_NAMESPACE_RETENTION`
    on the `temporal` service in the workspace compose file. The backend
    needs no change either way — the endpoint just reaches further.
  - **A run's provenance is three custom search attributes**, `TaskKind`,
    `TaskName` and `TaskMap` (Keyword), stamped by `start_task` from the
    request's `kind` / `name` and the map loaded at dispatch, and by every
    schedule's action (`kind = schedule` plus the template's name and the
    schedule's `map_name`). The worker registers them on the namespace at connect
    (`ensure_search_attributes`, idempotent, non-fatal); by hand it is
    `docker exec temporal sh -c 'temporal operator search-attribute create --address $(hostname -i):7233 --namespace default --name TaskKind --type Keyword --name TaskName --type Keyword --name TaskMap --type Keyword'`.
    A start with an attribute the namespace does not know is refused, so until
    `TaskMap` exists every dispatch that stamps it answers 502.
    Until they exist, `kind` / `name` filters and `/stats` answer 502 and the
    log names the attribute. `kind=schedule` is answered from
    `TemporalScheduledById IS NOT NULL`, the attribute the server itself
    stamps, so schedules registered before these existed still count as
    scheduled (their runs carry no name).
  - `status` folds Temporal statuses the same way `_WORKFLOW_STATUS_MAP` does,
    so `FAILED` also asks for `TimedOut` and `CANCELED` for `Terminated`; the
    `/stats` counts fold them back the same way.
  - No `ORDER BY` in the query: SQL visibility rejects a custom one, and its
    default order is already newest `CloseTime` first. `/stats` is one
    `count_workflows` with `GROUP BY ExecutionStatus`, the only `GROUP BY` the
    server allows — a breakdown by kind or name would be one count per value.
  - The page token is Temporal's, base64url'd; it is only valid for the same
    `status` / `since` / `until` / `kind` / `name`. A token Temporal rejects
    answers 400, a malformed one too.
- **A job's map is checked twice.** MOVE coordinates are a place in one map's
  frame, and on another map they are a real place, just the wrong one. The
  rule is `helpers/move_guard.py`: positions planned on another map
  (`map_mismatch`), or the loaded map's floor plan being rebuilt
  (`conversion_running`), and an unknown map is never refused. `POST /api/v1/tasks`
  asks it before starting a run, and `execute_move` asks it again before every
  goal, with the run's own `TaskMap` (the workflow reads it off
  `workflow.info()` and passes it as MOVE's optional second argument). The
  second check is the one a schedule meets — it fires without passing through
  the first — and its refusal is a non-retryable step failure, so the history
  row says why. A schedule registered before `TaskMap` carries none and drives
  as it always did.
- **Schedules use `SKIP` overlap policy**: a robot can only do one thing at a
  time, so a new run never starts while the previous one is still executing.
- **The trigger is editable in place** (`PATCH /api/v1/schedules/{id}`), through
  `ScheduleHandle.update()`: the callback gets the described schedule back and
  swaps only its `spec`; action, policy and state go back as described. The
  ownership gate runs inside that callback (the SDK's update already describes,
  so no second RPC) and raises 404 for another robot's schedule before anything
  is sent.
- Temporal compiles a cron expression into a calendar spec and forgets the
  string, but it copies whatever follows `#` into that calendar's `comment`, and
  the comment is echoed by both describe and list and survives an update. So the
  gateway registers every cron as `"<cron> # <cron>"` and reads it back from the
  comment. Three spellings are refused with 400 — on **every** path that
  registers a trigger, the two creates as much as the edit, since they share
  `_build_schedule_spec` — because they would break that echo: a `#` of the
  caller's own, a `CRON_TZ=`/`TZ=` prefix (the body has a `timezone` field), and
  `@every` (Temporal compiles it into an interval, which has no comment;
  `interval_seconds` is that trigger). The trigger used to live in the memo
  instead; it cannot any more, because **a schedule memo is immutable** —
  `UpdateScheduleRequest` has a `memo` field and server 1.29.7 ignores it — so a
  copy there would have gone stale on the first edit. Schedules
  registered before this change compiled to a comment-less calendar and still
  carry the memo copy; the reader falls back to it only when the spec has
  nothing to say, so editing one of those flips it onto the spec for good.
- The memo carries `robot_id` / `map_name` / `task_template_id` /
  `task_template_name`, because the memo is readable from `list_schedules()`
  while the start-workflow args are not. (`saved_task_id` / `saved_task_name` are
  still accepted on *read* for schedules registered before the rename.)
- **A schedule's steps are readable from `describe()` but never from `list`.**
  `GET /api/v1/schedules/{id}` decodes them out of `ScheduleActionStartWorkflow.args`
  (raw `Payload` protos plus the description's `data_converter`);
  `GET /api/v1/schedules` always answers `steps: []`, because a schedule *list*
  element carries only the workflow type name and faking it would cost a describe
  RPC per row on first paint. A decode failure degrades to `[]` with a warning,
  never a 502 — same policy as the per-step workflow query.
- **A scheduled run's steps are frozen at registration.** The action args hold a
  concrete `WorkflowTask` and nothing re-reads it, so later vertex edits reach
  saved tasks and immediate dispatches but *not* an already-registered schedule.
  `POST /api/v1/task_templates/{id}/schedule` therefore refuses a template whose map is
  not active, and refuses one with a `MISSING` vertex — an unattended run does not
  get the snapshot fallback an operator watching the screen is allowed.

## Configuration

| Env var | Default | Used by |
|---|---|---|
| `TEMPORAL_ADDRESS` | `127.0.0.1:7233` | Temporal client + worker |
| `POSTGRES_HOST` | `localhost` | `database/postgres.py` |
| `POSTGRES_PORT` | `5432` | ditto |
| `POSTGRES_USER` | `syncrobotic` | ditto |
| `POSTGRES_PASSWORD` | `syncrobotic` | ditto |
| `SYNCAI_SYSTEM_INI` | `~/robot_ws/config/system.ini` | `helpers/system_config.py` (per-robot INI reads, e.g. `[map]`) |
| `TTS_SERVICE_URL` | `http://syncai_tts:8080` | `gateways/tts` — the syncai_tts container, by its compose service name; `http://127.0.0.1:8080` when the backend runs on the host (the compose service uses `http://127.0.0.1:9090`, the port syncai_tts publishes) |
| `SYNCAI_WEBRTC_LIB` | `~/robot_ws/lib/libsyncai_worker.so` | `gateways/webrtc` — the Go worker to dlopen. Pointing it at a path that does not exist is the only way to disable WebRTC: a loaded c-shared library cannot be unloaded |
| `VIDEO_RTP_PORT` · `VIDEO_AUDIO_RTP_PORT` · `AUDIO_RTP_PORT` · `STUN_SERVERS` | `5006` · `5007` · `5004` · `stun:stun.l.google.com:19302` | Read by the **worker**, not by Python; `gateways/webrtc` only `setdefault`s them right before `InitWorker`, which freezes them for the process's life. The compose service sets `STUN_SERVERS=""` (LAN robot). `TURN_SERVERS` / `TURN_USERNAME` / `TURN_PASSWORD` have no default on purpose — `.env` is their only source |

`.env` in the workspace root is loaded via `python-dotenv` at import time.

`[system] robot_id` is read from the system INI rather than the environment (by
the launch file). Unlike the rest of the stack, this package resolves that INI
by an **absolute** default path (`~/robot_ws/config/system.ini`, the workspace
inside the robot container), in both `launch/backend.launch.py` and
`helpers/system_config.py`: the old relative `config/system.ini` only worked
because every entrypoint happened to run from the workspace root, and the
backend is also started from tests and shells that do not. Override with the
`system_config:=` launch argument or `SYNCAI_SYSTEM_INI` respectively.

There are **no ROS parameters** in this package — nothing calls
`declare_parameter`. Everything configurable is the INI, the environment, or a
constant with a comment explaining it.

One of those constants spans two containers and is worth naming here:
`MapCloudSubscriber`'s `allowed_root`, `/dev/shm`. pgo writes the mapping-mode
map cloud to `/dev/shm/syncai_pgo/<robot_id>` and names the file in its notice;
this side only checks the path sits under that root before reading it. Neither
end reads the root from the INI or the environment — they agree on it by
convention, and `ipc: host` on both compose services is what makes it the same
tmpfs (see *ROS interfaces*). Tests pass a tmp dir; nothing else overrides it.

Postgres connection is retried 20× at 5 s intervals on startup, and the
`<robot_id>_db` database is created if absent — the backend can therefore come up
before the `postgres` container is ready.

CORS is currently wide open (`allow_origins=["*"]`).

## Build and run

Builds run **inside the robot container** — see the workspace `CLAUDE.md`.

```bash
colcon build --packages-select syncai_backend --symlink-install
source install/setup.bash
```

Python deps are **not** managed by rosdep (jammy has no reliable key for
fastapi); `requirements.txt` is the single source of truth and every Docker
stage in this repo installs from it:

```bash
pip install -r src/syncai_backend/requirements.txt
```

Run it:

```bash
ros2 launch syncai_backend backend.launch.py                     # namespaced from config/system.ini
ros2 launch syncai_backend backend.launch.py system_config:=config/instances/robot01.ini
ros2 run syncai_backend backend                                  # no namespace -> default_robot
```

In practice `NodeManager` starts it from **both** session specs,
`config/sessions/start_nav.yaml` and `start_mapping.yaml`, in the `backend`
window (pane 2, behind `robot_state`, `sleep: 2`). Earlier revisions left it out
of the mapping spec as nav-oriented; it is there now because the operator
console *is* the mapping UI — mode switching, teleop, the live cloud view and
the save-map call all go through it. It still hard-requires postgres, which
lives in the infra compose stack and is up regardless of which session exists.

> `setup.py` installs **compiled bytecode only** (`InstallNoSource`): after a
> normal install it byte-compiles the package and deletes the `.py` sources from
> the install space, so deployments ship no source. The step self-disables when
> the installed modules are symlinks, so `--symlink-install` developer builds are
> unaffected.

### As its own container

`Dockerfile` builds four stages — `base` (ROS + the pip deps, the expensive one),
`builder` (the colcon install space), `runtime` (the service) and `dev` (the test
image) — and `docker-compose.yml` runs the `runtime` one as a single service.
There is no default target; name it.

#### Step by step

**1. Before the first start.** Everything the service talks to must already be
up on the host: postgres and temporal (the workspace's infra stack), syncai_tts
(SyncAI-TTS) and the ROS side (the robot container). Compose starts none of
them. Then:

```bash
# The infra, if it is not up.
docker compose -f ~/SyncAI-Robot-Workspace/docker-compose.yml up -d postgres temporal

# The WebRTC worker, built in SyncAI-WebRTC-Worker and copied in by hand.
# Optional: without it the camera stream 502s and everything else works.
cp /path/to/libsyncai_worker.so lib/

# Stop the byobu-pane backend inside the robot container first (see
# "One backend at a time" below) — two backends on one robot is an outage.
```

**2. Build the image.** syncai_common is cloned by the builder stage from
`interface.repos` (HTTPS, no credentials), so
nothing needs copying in. The `base` stage is the slow one (~20 min on aarch64
the first time); after that only a change to `requirements.txt` rebuilds it.

```bash
docker compose build                                    # tags syncai-backend, USER_UID=${UID:-1000}
# or, without compose — name the target, there is no default:
docker build --target runtime --build-arg USER_UID=$(id -u) -t syncai-backend .
```

`USER_UID` must match the owner of the host's `config/`, `map/` and `record/`,
or the container user cannot write them.

**3. Run it.** Compose is the supported way to run the image — it carries the
host networking, `ipc: host`, the nvidia runtime, the camera devices, the
mounts and the environment, each commented in `docker-compose.yml`.

```bash
docker compose up -d                # add --build to rebuild first
docker compose logs -f              # rclpy + uvicorn + Temporal worker, one stream
```

The knobs, all optional, set in the shell or a `.env` beside the compose file:

| variable | default | what it moves |
|---|---|---|
| `ROBOT_WS` | `~/SyncAI-Robot-Workspace` | where `config/` and `map/` come from |
| `ROBOT_INSTANCE` | `robot01` | which `config/instances/<name>.ini` is mounted as `system.ini` |
| `RECORD_DIR` | `./record` | where bags land on the host |
| `WEBRTC_LIB` | `./lib/libsyncai_worker.so` | the WebRTC worker `.so` |
| `POSTGRES_USER` / `POSTGRES_PASSWORD` | `postgres` / `postgres` | the infra stack's credentials |
| `UID` / `GID` / `VIDEO_GID` | `1000` / `1000` / `44` | container user and the host's `video` group (`getent group video`) |

```bash
ROBOT_INSTANCE=robot02 RECORD_DIR=/mnt/ssd/record docker compose up -d
```

**4. Check it.** `/health` is always 200; the body says whether it is `ok` or
`degraded` (Temporal down, say). The container's healthcheck turns healthy
after up to 120 s, because postgres is retried 20 × 5 s before giving up.

```bash
docker compose ps                   # STATUS: healthy
curl http://127.0.0.1:3000/health
docker exec syncai_backend /usr/local/bin/entrypoint.sh ros2 node list   # /<robot_id>/... present
```

If the node comes up as `default_robot`, the per-robot INI was not mounted; if
`ros2 topic list` in here is empty, check `ROS_DOMAIN_ID` / the RMW (both
explained below and in the compose file).

**5. Stop / restart.**

```bash
docker compose restart              # e.g. after changing .env or the WebRTC .so
docker compose down                 # stop and remove the container
```

**Without compose.** A plain `docker run` works if it reproduces what the
compose service sets; this is the equivalent, and it drifts the moment the
compose file changes, so prefer compose:

```bash
WS=${ROBOT_WS:-$HOME/SyncAI-Robot-Workspace}
R=/home/syncrobotic/robot_ws
docker run -d --name syncai_backend --restart unless-stopped \
    --network host --ipc host --runtime nvidia \
    --user "$(id -u):$(id -g)" --group-add "$(getent group video | cut -d: -f3)" \
    --workdir $R --stop-timeout 30 \
    -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=$R/config/cyclonedds.xml -e ROS_LOG_DIR=/tmp/ros_log \
    -e POSTGRES_HOST=127.0.0.1 -e POSTGRES_PORT=5432 \
    -e POSTGRES_USER=postgres -e POSTGRES_PASSWORD=postgres \
    -e TEMPORAL_ADDRESS=127.0.0.1:7233 -e TTS_SERVICE_URL=http://127.0.0.1:9090 \
    -e STUN_SERVERS= -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 \
    -v /dev/syncai:/dev/syncai:ro \
    -v $WS/config:$R/config \
    -v $WS/config/instances/robot01.ini:$R/config/system.ini \
    -v $WS/map:$R/map \
    -v "$PWD/record":$R/record \
    -v "$PWD/lib/libsyncai_worker.so":$R/lib/libsyncai_worker.so:ro \
    --tmpfs /tmp/ros_log:size=128m,mode=1777 \
    syncai-backend
```

Drop the `--device` lines for cameras that are not plugged in (`docker run`
refuses a missing device), and `--runtime nvidia` off a Jetson.

The INI is mounted the way the workspace's compose mounts it: the per-robot
`config/instances/robotNN.ini` goes **over** `config/system.ini` as a single
file (the checked-out `system.ini` is an empty placeholder). Miss that and the
launch file finds no `[system] robot_id`, and the namespace, the database and
the task queue all move to `default_robot`.

What that service is, and why each piece is the way it is, is commented in the
compose file; the four that bite are:

- **`network_mode: host`.** DDS discovery with the nav stack runs over `lo` with
  multicast off (`config/cyclonedds.xml`), so both sides have to share the host's
  network namespace. It also means compose service names do not resolve —
  postgres, temporal and syncai_tts are addressed as `127.0.0.1:<published port>`,
  and the REST API lands on the host's `:3000` with no `ports:` mapping.
- **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,** with the package installed in the
  image. A backend left on the default fastrtps starts cleanly, logs nothing
  alarming and sees not one topic.
- **Three bind mounts from `ROBOT_WS`** carry what this package shares with the
  rest of the stack: `config/` (read-write — activating a map rewrites
  `system.ini` in place, and the per-robot `instances/robotNN.ini` goes over
  `config/system.ini` as a single file) and `map/` (must be the same directory
  the nav stack's `map_server` reads). `[system] robot_id` in the mounted INI is
  still what namespaces the node, the database and the task queue. **`record/`
  and `lib/libsyncai_worker.so` are not among them**: no other process reads a
  bag or dlopens the worker, so both default to a directory beside the compose
  file — `record/`, moved with `RECORD_DIR` (`export
  RECORD_DIR=/mnt/ssd/record`), and `lib/libsyncai_worker.so`, moved with
  `WEBRTC_LIB`. Both are tracked as empty directories (`.gitkeep`; the `.so`
  itself is gitignored) so Docker never creates the bind source itself — a
  source it creates is `root:root`, which the uid-1000 container user cannot
  write, and a missing *single-file* source is created as a directory, which
  dlopen then fails on.
- **`ipc: host`.** pgo hands this process the mapping-mode map cloud as a PCD
  under the host's `/dev/shm/syncai_pgo/<robot_id>` (see *ROS interfaces*),
  and the path in its notice only names the same file if both containers
  share the host's IPC namespace — the workspace's robot service sets this
  too. Not a bind mount of a `/dev/shm` subdirectory, for the reason `record/`
  and `lib/` give above: Docker would create the host source `root:root`.
- **One backend at a time.** `NodeManager` starts this process as a byobu pane
  inside the robot container. Running the service alongside it gives two
  processes on `:3000` and, less visibly, two Temporal workers polling the same
  `<robot_id>.ROBOT_TASK_QUEUE`. Take it out of the session spec first.

Note the knock-on for `switch_mode`: as a byobu pane the process dies with the
session and `NodeManager` restarts it, which is the mechanism the route's
"success looks like a dropped connection" semantics rest on. In its own
container it survives the session teardown instead, and `restart:
unless-stopped` only covers a crash — the mode switch itself no longer recycles
it.

## Tests

```bash
colcon test --packages-select syncai_backend
colcon test-result --verbose
# or, inside the container, from src/syncai_backend/:
pytest test/
```

**Off the robot**, the `Dockerfile`'s `dev` target (`docker build --target dev
-t syncai-backend-dev .`) builds an image that supplies ROS 2 Humble and this
package's pip dependencies; the source is bind-mounted rather than copied, so an
edit is picked up by the next run with no rebuild. Its header comment carries the
exact commands. Two things are worth
knowing before reading a result from it:

- **Get the interface package in, or a third of the suite does not run.**
  `vcs import < interface.repos` materialises `syncai_common`; build it with
  `--packages-select syncai_common`. Measured 2026-09-23 (when the map srvs
  still came from FAST-LIO2's `interface` package, now folded into
  `syncai_common`):

  | in the image | result |
  |---|---|
  | present | 704 passed, 1 skipped (`test_copyright`, skipped on purpose) |
  | absent | 315 passed, 6 skipped, **11 collection errors** — those files reach `syncai_common` through a plain import rather than an `importorskip` |

  Anything that touches a router or a gateway needs it for the run to mean
  anything. (The `_INTERFACE_SRVS` guard and its
  `test_map_gateway_no_interface.py` are gone: the map srvs are a hard import,
  and an image without them fails in the builder.)
- **The image runs as a non-root user on purpose.** Root bypasses file
  permission checks, so `os.access(W_OK)` answers `True` on a read-only file and
  the map router's `ini_not_writable` refusal test fails against working code.

`test/` holds ~40 files, roughly one per router / gateway / subscriber / repo /
helper (`ls src/syncai_backend/test/` is the index). They must run where `rclpy`
/ `nav_msgs` / `syncai_common` / OpenCV are importable. The database layer is
exercised against an **in-memory SQLite** engine (`StaticPool`, so every session
shares one connection), so no PostgreSQL server is needed. The conversion tests
additionally need scipy (`test_pcd_to_gridmap.py` imports the z-band helper,
which imports `scipy.ndimage`) and open3d (`test_traversable.py`
`importorskip`s `helpers.traversable` per test, so its cases skip rather than
fail where open3d is absent). Alongside the unit tests are the standard ament
linters (`test_copyright`, `test_flake8`, `test_pep257`).

## Gotchas

- **Configuration is read once, at startup.** There are no ROS parameters to
  change, but the INI (`[map]`, `robot_id`), `.env` and the environment are all
  read during construction — a change to any of them needs a backend restart.
- A relative topic name is not optional — see the namespace section above.
- **Exactly two latched subscriptions, both pgo's: `pgo/map_cloud_file` and
  `pgo/mapping_status`.** `map` and `localizer/map_cloud` (both
  TRANSIENT_LOCAL) went away with the file-based map endpoints; the map-cloud
  notice came back TRANSIENT_LOCAL on purpose, the run status arrived the same
  way, and their durability has to match pgo's publisher exactly — a VOLATILE
  reader connects and gets nothing replayed. Both exist only under `MANUAL`; a
  silent `pointcloud/map/stream` and an `unknown` run state on a navigating
  robot are the expected state, not a QoS mismatch.
- **Mapping mode comes up idle.** Since 2026-10 pgo banks nothing until
  `POST /api/v1/mapping/start`, and a successful save returns it to idle. A
  save or reset in that state is a 409 `mapping_idle` from the latched status,
  not pgo's `NO POSES!` as a 502 — and an older console with no Start control
  cannot map against this stack at all. The status is aged out after 5 s
  (`MAPPING_STATUS_TTL_S`): this container outlives the mapping session, and a
  latched `mapping` from a pgo that is gone must not refuse a Start forever.
- **The map cloud is a file in the host's `/dev/shm`, and both containers need
  `ipc: host` to see it.** The symptom without it is not silence: notices
  arrive, every read logs `map cloud file unreadable`, and the preview never
  updates. Do not "fix" the size cliff by raising `net.core.rmem_max` and
  going back to the `PointCloud2` — that is host state on every robot, and
  16-45 MB per merge only moves the cliff.
- `map -> pointlio_odom` exists once `syncai_localizer` has had its first
  synced `body_cloud` + `lio_odom` pair — **not** once it has converged: it
  broadcasts from that first sample on, starting at an identity offset, whether
  or not a relocalize has happened. So a missing transform means the localizer
  (or pointlio under it) is not running, and a present one proves nothing about
  the pose — `relocalize_check` is the convergence check. Until the transform
  exists the live cloud stream is silent; the subscriber logs once on the first
  drop and once on recovery rather than per frame, so check the log if the 3D
  view is empty.
- **Deleting `keepout.*` clears nothing.** The running `KeepoutFilter` keeps
  the last mask it was handed, and the nav session writes a blank pair back at
  the next boot anyway. Clearing zones is `PUT .../keepout` with `[]` — an
  all-unknown mask, reloaded. And it is *unknown* (205), not free (254): a free
  mask cell overwrites unknown costmap cells and would make unexplored space
  plannable.
- **A map switch does not move the keepout mask along.** `filter_mask_server`
  serves whatever it was last told to; `activate` reloads the new map's mask
  itself, writing a blank one first for a map that has never been booted into.
  A `filter_mask_server/load_map` that is "not available" means MANUAL mode or
  a dead keepout pane — since workspace `c8d2558` the launch starts it on every
  map, blank mask included, so it is not the normal state of a map without zones.
- The `sqlalchemy` session convention is per-repo: `init_map_repo` creates the
  schema and builds its own `sessionmaker` from the injected engine.
