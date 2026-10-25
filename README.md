# syncai_backend

> **Standalone repository of a colcon package**, and the source of truth for
> `syncai_backend`: a ROS 2 `ament_python` package that imports `syncai_common`
> (msgs/srvs, including the map srvs `StartMapping` / `SaveMaps` /
> `ResetMapping` / `Relocalize` / `IsValid`, formerly in FAST-LIO2's `interface`
> package). `vcs import < interface.repos` from a colcon workspace root fetches
> it over HTTPS — no `SyncAI-Robot-Workspace` checkout, no credentials; see the
> `Dockerfile`.
>
> **Running** it still needs the rest of the robot stack: the nav stack's topics
> and services, Postgres, and the `~/robot_ws` layout (see *Configuration*). On a
> robot it is vcs-imported into `SyncAI-Robot-Workspace/src/syncai_backend`, and
> the workspace-relative paths below (`src/syncai_backend/…`, the workspace
> `CLAUDE.md`, `.env`) assume that placement.

The robot's application-layer process: a **FastAPI REST/WebSocket server and an
rclpy ROS 2 node in one Python process**, plus a **Temporal worker** that runs
multi-step tasks.

It is the robot's whole operator-facing surface — robot state, the map, point
clouds, tasks, mode switching, wifi, speech — and every ROS interaction (nav
goals, motion keys, wifi services, `switch_mode`, `save_maps`) happens here.

```
                    HTTP :3000 / WebSocket
  any API client   ────────────────────────►  syncai_backend  ──── ROS 2 ────►  nav stack,
                                                    │                            robot_state,
                                                    │                            LIO / localizer,
                                                    ├──── gRPC ──►  Temporal     system_manager
                                                    └──── SQL  ──►  PostgreSQL
```

## Process model

`main.py` builds one `rclpy` node (`syncai_backend_node`), whose constructor
starts two extra threads:

| Thread | What runs there |
|---|---|
| main | `MultiThreadedExecutor.spin()` — all ROS subscriptions, TF, service/action clients |
| uvicorn (daemon) | The FastAPI app on `0.0.0.0:3000` |
| Temporal worker (daemon) | Polls `<robot_id>.ROBOT_TASK_QUEUE` (`RobotWorkflow` + the body activities) and `<robot_id>.ROBOT_IO_QUEUE` (`SPEAK`) |

- The executor is **multi-threaded on purpose**. Each point-cloud callback
  (`body_cloud`, and the multi-MB PCD reads behind `pgo/map_cloud_file`) has its
  own `MutuallyExclusiveCallbackGroup`, so a busy cloud frame cannot starve
  `robot_state` / telemetry / TF — or the other cloud.
- Blocking REST handlers — ROS service calls (up to ~70 s for wifi), psycopg2,
  OccupancyGrid→PNG encoding — are **plain `def`, not `async def`**, so FastAPI
  runs them in its thread pool instead of stalling the event loop. Keep that
  distinction when adding endpoints.
- **Shutdown is ordered: the Temporal worker first, the ROS context last.**
  `main.py` installs its own handlers for SIGTERM (docker stop), SIGINT (`ros2
  launch`, Ctrl-C) and SIGHUP (the byobu session being killed by `switch_mode`)
  and asks rclpy for none (`SignalHandlerOptions.NO`). A stop signal starts a
  shutdown thread that calls `TemporalWorkerHandle.request_shutdown`
  (`Worker.shutdown` on both workers, up to `WORKER_SHUTDOWN_TIMEOUT_S`, 20 s)
  while the main thread keeps spinning the executor; only once the worker
  thread has ended does the spin loop exit and `rclpy.shutdown()` run. The
  reason is the in-flight MOVE: the SDK cancels it on shutdown and its `except
  CancelledError` cancels the nav2 goal and restores locomotion — ROS traffic
  that needs the executor alive. With rclpy's default handlers the context went
  first, the daemon worker thread died with the process, and the goal drove on
  unsupervised until the next worker re-sent it. A second signal exits at once.
  A requested stop exits 0, anything else 1. `docker-compose.yml`'s
  `stop_grace_period: 30s` is sized to this budget.

## Layering

A convention, not enforced by tooling:

```
interfaces/rest/routers/   HTTP + WS surface; pydantic schemas; no business logic
        │
services/                  domain work that outlives a request: gridmap_conversion (the two
        │                  recipes, the registry of running conversions, the
        │                  gridmap.recipe.json protocol); mode_restart (the latest
        │                  restart_mode dispatch and how it ended); safety_lock (the driver
        │                  safety lock's rising edge → cancel every task); map_switch (the
        │                  live map switch's steps and their undos); dock_state (the last
        │                  DOCK's pose, for the auto-undock); map_usage (whether a map may
        │                  be edited, destroyed, switched to or driven on); map_import (an
        │                  archive's directory and vertex rows, published together)
        │
gateways/                  outbound: ROS (robot, map), Temporal (workflow), speech (tts: HTTP
        │                  to the syncai_tts container), camera/speaker (webrtc: a dlopen'd Go
        │                  worker), bags (recording: a supervised `ros2 bag record` child) —
        │                  the last three hold no ROS handle
repositories/              state: in-memory caches, PostgreSQL CRUD, and the two on-disk
        │                  catalogues (map/, record/)
        │
database/                  SQLAlchemy engine + ORM models; the shared session factory

tasks/                     the task model and its Temporal contract (steps, groups, queues,
                           signals, search attributes) — REST, the workflow gateway and the
                           worker all import it, and it imports none of them

subscribers/               ROS topics → repositories (ingest) — except the map cloud, whose
                           topic only names a PCD the subscriber reads from the shared /dev/shm;
                           QoS from qos.py (latched / best_effort), per the table below
temporal/                  worker, RobotWorkflow, activities
helpers/                   occupancy_grid (OccupancyGrid→PNG), pointcloud (read a binary PCD,
                           downsample / transform / pack), pgm, pcd_to_gridmap (z-band
                           recipe), keepout (forbidden-zone mask rasteriser), traversable
                           (traversability recipe), system_config (INI reader), map_archive
                           (checksummed zip / tar.gz of a map dir), move_guard (a run's
                           TaskMap against the loaded map), disk (directory size + newest
                           mtime), single_flight (concurrent callers share one slow read)
```

Wiring is explicit: `main.py` constructs every repo/gateway/subscriber and passes
them down as constructor arguments. No DI container, no module-level singletons
— a router gets what it needs through `init_<x>_router(...)`.

**`gateways/tts` is an HTTP client** for the **syncai_tts container** (the
`SyncAI-TTS` repo), which owns the kokoro session and the speaker. It used to be
the engine itself, and its internal lock was the only thing keeping a scheduled
`SPEAK` and a manual `POST /api/v1/tts/speak` off the speaker at once. That
guarantee now lives in the service, in front of the one piece of hardware — so
the Temporal worker can later become its own process without two locks failing
to see each other. The service serialises as a FIFO queue (utterances play in
the order accepted) and refuses a backlog past `TTS_MAX_QUEUE`, which this side
reports as 409, not 502. `onnxruntime`, `kokoro-onnx` and the ~310 MB model left
the rclpy process with it. Every request bounds *connecting* at 5 s separately
from its own wait (60 s for a request, 180 s for a held `speak`): httpx takes a
per-request timeout instead of the client's, so the bound is re-stated on each
call — an address that never answers the SYN fails as "could not reach" in 5 s
rather than holding the SPEAK slot or a threadpool thread for the full wait.

**Two pcd → gridmap recipes.** The default, z-band (`pcd_to_gridmap.py`), slices
the cloud into floor / obstacle height bands (trinary occupied / free / unknown),
then reverts free cells not connected to the keyframe trajectory to unknown.
Since 2026-09 the bands are offsets from the **local** floor, measured around
every keyframe in `poses.txt` (`local_floor_levels` / `flatten_to_local_floor`):
a LIO map of a large venue can drift a metre in z (0917_TP1F_test1: keyframe z
from -0.48 to +0.51), and one floor level for the whole cloud puts half the
floor in the obstacle band. `floor_reference: global` on `grid/convert` restores
the single level; `local` falls back to it — recorded as
`floor_reference_fallback` in the sidecar — when `poses.txt` is unreadable.
`traversable.py` segments the floor by intensity / normal / height and projects
it, leaving **no unknown cells**; it runs only when an operator asks via
`grid/convert`. There is deliberately no automatic pick; the workspace
`CLAUDE.md` ("Backend architecture") records why and what each writes.
`traversable.py` is the **only** module importing open3d, and nothing imports it
at module scope — `GridmapConversionService.start` imports it inside the
conversion thread's `try`, so a backend start never pays the ~100 MB import and
an `ImportError` is a per-map failure, not a bare thread traceback. Keep
`pcd_to_gridmap.py` open3d-free.

The conversion lives in `services/gridmap_conversion.py`, not the map router:
the recipes, the registry refusing a second concurrent conversion of one map,
and the sidecar protocol are domain behaviour, not HTTP. `main.py` builds one
instance; the router validates against it and maps its refusals to status codes.

**A conversion's outcome lives on disk, in `gridmap.recipe.json`.** The thread
writes it three times — `status: converting` before any work, then `ok` (recipe,
parameters, area diagnostics) or `failed` (the pipeline's `error` and a `hint`)
— and `MapCatalogRepo` reads it back as `grid_status` / `grid_error`. Disk,
because the record must outlive the thread and the **process**: a conversion
takes tens of seconds, and `switch_mode` tears down the byobu session this
backend is a pane of, taking the in-memory registry with it. While the process is
up the registry is still the authority; a sidecar saying `converting` with no
registry entry is reported as `interrupted`. Before this, a failed conversion
looked like a map nobody had converted, its reason only in
`log/stack/<robot_id>/backend/current`.

**Every conversion first waits for the robot's map cleaning.** pgo's `save_maps`
answers as soon as the *raw* `map.pcd` is written, then spawns a detached
`clean_map` that replaces it — seconds to minutes later, atomic rename — with a
copy without people and one-off returns, recording its progress in
`map_clean.recipe.json` (`converting` → `ok` / `failed`, written `converting`
before the save answers). A grid converted straight after the save was made from
the raw cloud: on `test` (2026-10-12) a person walking beside the robot was a
trail of obstacles, and re-converting after the clean removed 4090 of 9932
occupied cells. So the conversion thread polls that sidecar (2 s) before reading
the cloud, holding the conversion slot meanwhile (`grid_status: converting`;
rename / delete / activate get `conversion_running`), and converts once it says
`ok`. No sidecar (no patches, cleaning off, an older map) converts at once;
`failed`, a `converting` sidecar older than 15 min (a clean killed hard) and a
15 min wait all convert the raw cloud, as before. Which of those it was is
recorded as `map_clean` in `gridmap.recipe.json`. Waiting was chosen over
"convert now, re-convert when the clean lands", which converts twice, shows a
grid with people in it meanwhile, and collides with a hand-edit made in between.

**One `keepout.json` for 2D and 3D maps (2026-10-23).** The polygons below are
also a 3D map's forbidden zones: the robot's 3D planner (`Octo3D`) reads
`keepout.json` beside `octomap.bt` itself, no mask involved. Each zone may carry
`name` and a `z_min` / `z_max` band — the ground heights (map-frame metres) of
the floor it forbids, `floor_z ± 0.5`; without a band a zone covers every floor,
which is what a 2D zone means, so a file the 2D editor wrote applies unchanged on
a 3D map, and the mask ignores the bands. Unset keys are left out of the file.
`PUT …/keepout` on a 3D-only map writes the json alone (no grid to rasterise
onto), and on the **active** map reloads by the map's kind, as `activate` does
("3D wins"): `planner_server/Octo3D/load_keepout` (`MapGateway.reload_keepout_3d`,
`map_url` = the json) for a 3D map, `filter_mask_server/load_map` for a 2D one. A
3D switch needs no keepout call: Octo3D reads the new map's json as part of
`load_map`. **A client that edits only 2D must send back the `name` / `z_min` /
`z_max` it read**, since a save replaces the whole list — dropping them widens a
3D zone to every storey.

**Forbidden zones are a keepout mask beside the gridmap.** Since 2026-09 the
planner's global costmap runs a nav2-style `KeepoutFilter` over the
`OccupancyGrid` a second map_server, `filter_mask_server`, serves from
`map/<name>/keepout.yaml` + `keepout.pgm`. The backend writes that pair from
polygons the console draws in map-frame metres (
`PUT /api/v1/maps/{name}/keepout`) and keeps the polygons in `keepout.json` — **the
source of truth**; `MapCatalogRepo.write_keepout` derives the pgm/yaml from it,
because a mask cannot be turned back into editable polygons. `helpers/keepout.py`
rasterises (cv2, open3d-free) in the gridmap's exact geometry: row 0 is max y,
corners are snapped to cell centres console-side and `floor`ed here, the fill is
boundary-inclusive. **The background is 205 (unknown), never 254 (free)**: the
filter skips unknown mask cells but lets a *free* one overwrite an *unknown*
costmap cell, so an all-white mask would make every unexplored cell in the map's
bounding box plannable. The nav session's `costmap_filter_info.launch.py` writes
a blank 205 pair when it boots a map that has none (workspace `c8d2558`) and
never touches an existing one — so a booted map always has the pair, and a pair
without a json means "no zones drawn yet" (or a GIMP-drawn mask, which the next
save overwrites). Write order is pgm → yaml → json: whoever sees the yaml finds
its image, and a crash mid-write leaves the robot enforcing more than the console
shows, not less. None of the three files names the map, so rename and delete
carry them along. Not done yet: re-rasterising when `grid/convert` changes the
gridmap's geometry — the filter looks cells up by world coordinate, so the old
mask still applies where it has cells, and the next save rewrites it.

**The 3D map is served as the robot wrote it.** With `map_clean_octomap` on,
the same `clean_map` that rewrites `map.pcd` also writes `map/<name>/octomap.bt`
— the `octomap::OcTree` (0.1 m, free *and* occupied) the 3D planner plans on —
and says so in `map_clean.recipe.json` (`params.octomap`,
`measurements.octomap_written` / `octomap_error`). The catalogue reads that
sidecar and the tree's text header only, and reports `octomap_status` the way it
reports `grid_status` (`none` / `converting` / `ok` / `failed` / `interrupted`,
derived in `_octomap_status`; a tree on disk is `ok` whatever the sidecar says,
since it is written by tmp + rename), with `octomap_version` — `size-mtime_ns`,
the file's ETag unquoted — and `octomap_resolution`. `GET /api/v1/maps/{name}/octomap`
is the `.bt` byte for byte, ETag'd on that version and revalidated to 304. Not
decoded here on purpose: the file is a third the size of its voxel centres as
floats, the console's decoder is a hundred lines, and the console's ground
editor needs the tree's own keys, not metres. **Ground patches** are the one
thing the console writes back: `PUT /api/v1/maps/{name}/octomap/edits` records
the floor voxels an operator painted in — absolute OcTree keys, one per column —
as `octomap_edits.json`, in the shape the standalone viewer exported, **the
record and nothing else**: the tree is untouched, and the robot's
`apply_octomap_edits` is what patches `octomap.bt` from it (setting aside the
original as `octomap.raw.bt`); the 3D planner then needs its session restarted
to load the result. Like `keepout.json`, nothing in the file names the map, and
the keys hold for exactly that save's map frame.

**Floors.** A map that spans storeys is one tree in one frame; the console
shows one storey at a time. `GET /api/v1/maps/{name}/floors` detects them from
`poses.txt` on every read — a height the robot spent at least 5 % of its walk at
is a floor, a stair or landing it passed through is not, levels under 2 m apart
are one — and `PUT` saves an operator's list (names, body heights) to
`floors.json`, which then wins. The trajectory rather than the ceiling, because
the robot stood on every floor it mapped and a ceiling is a different height on
every site. Only the console reads the file.

**Maps travel as archives.** Since 2026-10 a map can leave as one `.zip` /
`.tar.gz` (`GET /api/v1/maps/{name}/export`) and arrive on another robot (
`POST /api/v1/maps/import`). `helpers/map_archive.py` owns the format: the directory's
files relative to the map root, plus a `syncai_map.json` manifest (an md5 per
file, the map's name, its vertices). The manifest is archive metadata — never
extracted into the map directory, never listing itself — so a round trip is
byte-identical and a re-export excludes nothing. Import **inspects before it
extracts**: every member's name and type and the manifest's file set are checked
without writing a byte, then only listed files are streamed out, hashed as they
land. Nothing calls `extractall` (on Python 3.10 it honours absolute names and
creates links). Unpacking happens in `map/.import-<token>/`, a replaced map is
parked as `map/.import-old-<token>/` — both *inside* `map/`, a bind mount, so the
one-`rename` publish never becomes a cross-filesystem copy. The `.import-` prefix
hides them (`list_maps` skips it, `resolve_dir` refuses it). A leftover
`.import-*` from a crash can be deleted; `.import-old-*` is the previous version
of a map, recoverable by renaming it back.

## robot_id, namespaces, and per-robot isolation

The launch file reads `[system] robot_id` from the system INI — by default the
**absolute** `~/robot_ws/config/system.ini` (bind-mounted per robot from
`config/instances/robotNN.ini`), overridable with `system_config:=` — and uses it
as the **node namespace**. The node reads it back:

```python
robot_id = self.get_namespace().strip("/") or "default_robot"
```

| Scoped by robot_id | Value |
|---|---|
| ROS topics/services/actions | relative names inherit the `/<robot_id>` namespace |
| PostgreSQL database | `<robot_id>_db` (auto-created on first connect) |
| Temporal task queues | `<robot_id>.ROBOT_TASK_QUEUE` (workflow, body activities), `<robot_id>.ROBOT_IO_QUEUE` (`SPEAK`) |

**All ROS names here are relative** (`map`, `robot_state`, `navigate_to_pose`,
`pointlio/body_cloud`). Never hardcode `/<robot_id>/…` — that bug has been fixed
here once already.

TF frame names are *not* namespaced, so the cloud subscriber takes the source
frame from the message header and only pins the target frame (`map`).

## ROS interfaces

**Subscriptions**

| Topic | Type | QoS | Goes to |
|---|---|---|---|
| `robot_state` | `syncai_common/RobotState` | BEST_EFFORT, depth 3 | `RobotRepo` → `GET /api/v1/robot/state`; `low_level_mode.safety_state` → `SafetyLockService` (rising edge cancels every task) |
| `odom` | `nav_msgs/Odometry` | BEST_EFFORT, depth 5 | composed with TF `map→odom` → telemetry WS (x, y, yaw; z from `pointlio/lio_odom`, see *Telemetry*) |
| `pointlio/lio_odom` | `nav_msgs/Odometry` | BEST_EFFORT, depth 5 | composed through `map→pointlio_odom` and the lidar mount → telemetry WS: the whole pose while mapping, base_link's height in AUTO |
| `motor_states` | `syncai_common/MotorStates` | BEST_EFFORT, depth 5 | reduced to `{joint: radians}` → telemetry WS |
| `plan` | `nav_msgs/Path` | BEST_EFFORT, depth 1 | thinned to ≤512 xy pairs → telemetry WS |
| `local_costmap/costmap/raw` | `nav2_msgs/Costmap` | RELIABLE, **TRANSIENT_LOCAL**, depth 1 | the controller's rolling costmap, origin into `map` by TF → telemetry WS `local_costmap`, while a navigation runs |
| `local_plan` | `nav_msgs/Path` | BEST_EFFORT, depth 1 | MPPI's optimal trajectory, into `map` by TF, kept at 5 Hz → telemetry WS `local_plan`, while a navigation runs |
| `pointlio/body_cloud` | `sensor_msgs/PointCloud2` | BEST_EFFORT, depth 5 | TF→`map`, thinned, packed → WS `pointcloud/stream` |
| `pgo/map_cloud_file` | `std_msgs/String` (JSON notice) | RELIABLE, **TRANSIENT_LOCAL**, depth 1 | names a PCD in the shared `/dev/shm`; read, stride-capped, packed → WS `pointcloud/map/stream` (mapping mode only) |
| `pgo/mapping_status` | `syncai_common/MappingStatus` | RELIABLE, **TRANSIENT_LOCAL**, depth 1 | pgo's run state (`IDLE` / `MAPPING` / `RESETTING`, keyframe and loop counts) → `MappingStatusRepo` → `GET /api/v1/mapping`, and the 409s on the run routes. Aged out after 5 s of silence; `unknown` outside a mapping session |

Everything is BEST_EFFORT except pgo's two latched control messages (below) —
`plan` included. Its publisher is `rclcpp::QoS(1)` (RELIABLE), which a
BEST_EFFORT subscriber still matches; what it gives up is retransmission. That
costs more here than elsewhere: the other topics are 20 Hz feeds, while a plan
arrives once per BT replan (~3 s), so a dropped one shows a route the robot has
already left until the next replan. `syncai_planner` also skips publishing while
nothing is subscribed, and its QoS is VOLATILE (no replay): after a backend
restart mid-run, the route is blank until the next replan.

**The saved map is not subscribed.** `map` and `localizer/map_cloud` used to be
(TRANSIENT_LOCAL, matching their latched publishers); the map endpoints read the
files on disk on request now — see the note atop `routers/map.py`.

**The mapping-mode map cloud arrives as a file, not a topic payload.** pgo writes
each merge of its "map so far" as a binary PCD to
`/dev/shm/syncai_pgo/<robot_id>/map_cloud_<seq>.pcd` (tmp + rename, newest two
kept) and publishes a ~200 B JSON notice naming it on `pgo/map_cloud_file`;
`MapCloudSubscriber` reads it with the same `read_pcd_xyz` the map endpoints
use. The reason is a size cliff: a large floor at pgo's 0.2 m voxel is ~16 MB
per merge (44.7 MB at full resolution — 2.79 M points, 2026-09); CycloneDDS sends
that over UDP on `lo` as tens of thousands of datagrams in one burst, the
kernel's default receive buffer (`net.core.rmem_max`, 208 KB) overflows,
fragments drop, and a BEST_EFFORT reader loses the whole sample — the preview
simply stopped once the map grew. Raising the sysctl is host state on every robot
and only moves the cliff; a tmpfs write has neither problem. The `PointCloud2` on
`pgo/map_cloud` still exists for rviz (subscriber-gated); nothing here reads it.

Both containers must see the same `/dev/shm` — `ipc: host` on this compose
service **and** the workspace's robot service (a private container `/dev/shm` is
64 MB and would not hold two merges anyway). Without it, notices arrive, every
read is `ENOENT` (a warning), and the preview never updates.

The notice is RELIABLE + **TRANSIENT_LOCAL**, depth 1, matching pgo's publisher
exactly (a VOLATILE reader matches but never gets the replay): latching 200 bytes
is free, and a backend (re)started mid-mapping draws the current map at once.
Each notice is a complete, loop-closure-corrected replacement of the last, so
depth 1 (an older merge is never worth delivering), no TF (points were placed
with corrected global poses) and no voxel downsampling (pgo already voxelised);
`cap_points` stays as the wire-size guard. pgo publishes subscriber-gated, so
this subscription un-gates it. The topic does not exist under `AUTO`, hence the
silent stream on a navigating robot.

An **empty** notice (`points: 0`, `path: ""`) is a message, not a non-event: pgo
publishes it from `reset_mapping` to say the map was discarded — the only thing
that stops a console drawing a map that no longer exists — so the subscriber
clears its slot, and, being latched, it replaces the notice naming the deleted
files. Since 2026-10 a **successful `save_maps` sends the same notice**: pgo ends
the run (goes idle, frees its keyframes) and empties `/dev/shm` itself, so the
layer clears on a save and **this process never deletes a file there**. A
NaN-only file takes the same clearing path. The signal travels the topic on
purpose, reaching every dashboard and rviz rather than only the client that
pressed the button — which is why the reset's REST handler does not touch the
repo. Every other callback failure — a file pgo already pruned, bad JSON, a path
outside `/dev/shm`, a non-PCD — is a warning that leaves the slot as it was: an
exception out of a callback would end the executor and the process.

**`RobotState` carries more than `GET /api/v1/robot/state` exposes.**
`motor_status`' kinematic half (`q` / `dq` / `ddq` / `tau_est`) and its source
`timestamp` are for operators only. `routers/robot.py` names its response fields
one by one — the *only* thing keeping them out of a frozen public payload — so
that list is a **whitelist, not a mirror**: a new message field stays out until
somebody decides otherwise. That has been decided twice:

- **`localization_valid`.** `RobotStateSubscriber` **stores every sample**,
  including those where it is false — before relocalization, and for the whole
  of a mapping (MANUAL) run, whose TF chain never reaches `base_link` — when
  `localization_status` is a zeroed placeholder. It used to drop them so the
  endpoint 404'd instead of showing the robot parked on the map origin, which
  also hid battery, `low_level_mode` and above all `mode` exactly when they
  matter (the mapping console was blind). Pose honesty is kept by the field
  instead: gate on `localization_valid`, not on a 404, which now only means
  `robot_state` has never published.
- **`low_level_mode`** — the gait controller's own state machine, which the
  console cannot otherwise read: `set_motion_key` / `set_policy_mode` are one-way
  UDP whose 200 only means a datagram went out. It is decoded to **labels only**
  (`PPO` / `LOCOMOTION` / …, `UNKNOWN` for an unmapped code). Motion `8` is
  `IDLE` (no controller drives the motors), labelled rather than left to the
  fallback because the console ends a commanded MPC when the motion returns to
  it, and MPC's own code reads `UNKNOWN`. The raw integers stay on the topic:
  `ros2 topic echo /<robot_id>/robot_state --field low_level_mode` tells unmapped
  codes apart.

`low_level_mode.safety_locked` is **not the controller's**: it is
`syncai_driver_manager`'s software safety lock (`RobotLowLevelMode.safety_state`,
from its latched `safety_locked` topic). While engaged, the driver drops
`cmd_vel` and every motion key but ESTOP. `false` before the driver has ever
published looks the same as a real release.

**The lock's rising edge cancels every task** (`services/safety_lock.py`).
`RobotStateSubscriber` hands each sample's `safety_state` to `SafetyLockService`,
and `POST /api/v1/robot/estop` reports an operator engage; both share one
last-seen value, so only `false → true` acts — a lock repeated at 1 Hz, a second
engage or a release cancels nothing, and one engagement is one cancel whichever
side sees it first. A first sample already `true` counts (runs outlive a backend
restart). The cancel is `cancel_active_moves` (nav goals now, not on the
activity's next heartbeat), then `WorkflowGateway.cancel_active_tasks` — a
*fresh* visibility sweep plus the last task this process started, never the
console's TTL cache. It runs on the uvicorn loop, which `WorkflowGateway` belongs
to; the REST server's lifespan hands the loop in, and an edge seen earlier runs
once it does.

**Action client:** `navigate_to_goal` (`syncai_common/NavigateToGoal`), served by
`syncai_task_runner`. It was `navigate_to_pose` (`nav2_msgs/NavigateToPose`)
until 2026-10: the same navigation, but Humble's nav2 result is empty, so a
failed MOVE could only say "move ended in aborted". `NavigateToGoal`'s result
carries the robot's reason (`error_code` / `error_msg`, which the activity puts
in the step's `error_msg`), and its feedback's `status_msg` says what the
navigator is doing ("Following the path", "Path blocked; replanning", …),
which `GET /api/v1/tasks/{id}` returns as a running MOVE step's `message`.
`RobotGateway` keeps a goal-id → `MoveGoal` table so an activity can poll
status and cancel, and the latest feedback per goal for that read.

**Every goal reaches a terminal state, with or without the server.** The result
callback is the normal way out of `EXECUTING`, and it only runs when the task
runner delivers a result — which one that restarted or crashed mid-goal
(`POST /api/v1/robot/restart` rebuilds the AUTO session under a running backend)
never does. Such a goal used to stay `EXECUTING` for the life of the process:
teleop refused with "autonomous move in progress", the telemetry stream's local
picture gated open, `cancel_active_moves` failing on every safety-lock trip. The
gateway now marks a goal **lost** — `aborted`, `error_msg` the reason, no
`error_code` — wherever it learns no result is coming: the result future
completing with an exception or an `UNKNOWN` status, a cancel unanswered within
`CANCEL_RESPONSE_WAIT_S` (10 s), a cancel refused with `UNKNOWN_GOAL_ID` (a
restarted server; `GOAL_TERMINATED` means the result is on its way and is left
alone), or no action server at the next `move()`. A real result arriving late
replaces the loss. What still needs a nudge: a server that restarted while no
task was cancelling — the next cancel, safety-lock trip or `move()` is what
resolves it.

`DOCK` / `UNDOCK` steps are goals on the **same** client: `move(...,
behavior_tree=...)` sets the goal's `behavior_tree` to `dock.xml` / `undock.xml`
(`DOCK_BEHAVIOR_TREE` / `UNDOCK_BEHAVIOR_TREE`; `""` is the task runner's default
`move.xml`). A DOCK's `pose` is the **dock pose**: dock.xml computes the staging
pose from it, drives there, switches to MPC and calls the controller's `DockToPose`
action; it ends **standing on the dock in MPC** (the task runner stopped lying it
down on 2026-10-15 — a LIEDOWN step after the DOCK does that). An UNDOCK's `pose`
is **empty** (zeros): undock.xml ignores it since task runner `e027dbe` and backs
out 1 m from where the robot stands, switching back to RL at the end; it has no
stand-up step, so a robot lying on the dock is stood up first (see *Auto-undock*
below). This process never calls `DockToPose`. Sharing the client is what puts docks in the goal table, so the
status poll, `status_msg`, `cancel_active_moves` (safety lock included) and the
teleop refusal cover them with no extra code. A docking-branch failure ends the
goal `DOCK_FAILED` (300) with the failing node's reason in `error_msg`; an
unreachable staging pose is `PLAN_FAILED` / `FOLLOW_PATH_FAILED` as for a MOVE.
Needs `syncai_common` at or after SyncAI-Robot-Interface `bbcc958`.

**Auto-undock (2026-10-15).** A MOVE after a DOCK gets the robot off the dock
by itself: a MOVE straight off one starts with the footprint on the dock's
lethal cells and fails. `execute_dock`'s success records the dock's target pose
and map (`services/dock_state.py`, `DockStateService`); `execute_move`, after
its map check, asks whether the robot is still on it — same map, a localized
pose within 0.2 m / 0.3 rad of the record (unmeasured) — and if so sends
`MotionKey.STAND` and waits up to 15 s (unmeasured) for `motion_state` STAND
when it lies there (a LIEDOWN after the DOCK), runs undock.xml, forgets the
dock, and then drives. The state is here and not in the task runner on purpose
(SyncAI-Robot-Workspace, 2026-10-15): only this process sees the LIEDOWN, and
the robot side keeps its trees stateless. Consequences:

- A record the robot is provably not at (another map loaded, or a localized
  pose outside the window — teleop drove it away, a cancelled undock moved it)
  is dropped, never acted on: a blind 1 m back-out from a stale record is the
  failure this guards against. No robot_state yet or an unlocalized pose
  proves nothing; the MOVE just drives and the record stays.
- **It is in memory.** A backend restart forgets the dock, and the next MOVE
  fails on the dock's lethal cells as it did before this existed; an UNDOCK
  step through `POST /api/v1/tasks` still gets the robot off.
- The auto-undock runs inside the MOVE's attempt, so the MOVE's retry, pause
  and cancel cover it. A cancel or pause during the back-out cancels the goal
  and restores RL (as for an UNDOCK); the next attempt, or the resumed step,
  asks again, and a robot stopped within the window backs out again from
  there. A failed back-out fails the attempt with `undock ended in …` and keeps
  the record. In a group, the undock goal's acceptance reports the MOVE
  started, so a grouped SPEAK starts with the back-out.
- The UNDOCK step stays (API, stored templates and schedules), and its success
  forgets the dock too; the console no longer offers it.

**Service clients.** Every call goes through `gateways/ros.py`'s `call_service`: send, wait off the executor thread, and on timeout `remove_pending_request` the abandoned request, which otherwise sat in the client's pending table for the life of the process whenever its server was restarted under it (`switch_mode` / `restart_mode` rebuild them all). `restart_mode` alone keeps its request pending, for the late answer that reports the restart's outcome. `test_ros_call_service.py` fails on a new raw `call_async`.

- `RobotGateway`: `scan_wifi`, `connect_wifi`, `switch_mode`, `restart_mode`
  (`syncai_common/srv`, `syncai_sys_manager`); `set_motion_key` /
  `set_policy_mode` (`syncai_common/srv`) and `set_safety_lock`
  (`std_srvs/SetBool`, replacing the C++ driver's release-only `reset_safety`),
  all three `syncai_driver_manager`.
- `MapGateway`:
  - `map_server/load_map` (`syncai_map_server`) — an edited or re-converted
    gridmap reaches the running map_server.
  - `filter_mask_server/load_map` (the same executable, run as the keepout pane)
    — a saved mask reaches the planner's `KeepoutFilter` without a restart. The
    mask topic is latched, and a map *switch* must call it too because the
    launch derives the mask path once at boot.
  - `pgo/save_maps`, `pgo/reset_mapping`, `pgo/start_mapping` (`syncai_mapping`'s
    `pgo_node`) — the only things that serialise a run, undo one, and move pgo
    from idle to mapping (a mapping session comes up banking nothing; a
    successful save returns it to idle).
  - `relocalize` / `relocalize_check` (`syncai_localizer`, the map switch) —
    **bare** names in the robot namespace (`/<robot_id>/relocalize`, not
    `/<robot_id>/localizer/relocalize`); `gateways/map/map.py` records why.
  - `reset_odometry` (`syncai_localizer`, kidnap recovery) — the same bare name
    and the same mutually exclusive callback group as `relocalize`. Called with
    `skip_seed: true`: it resets Point-LIO (`pointlio/reset`, owned by the
    localizer so its offset follows the new origin), then runs the hint-free
    global search (`auto_localize`'s) on new-run scans, and blocks until the
    timer has converged on the winner (2 + 10 + 10 + 5 s localizer deadlines;
    the client waits 40 s), so its `success` is a result, not a receipt. Not
    `auto_localize` itself since 2026-10-15: a carry often diverges Point-LIO,
    and that search cannot recover from it on its own. Needs a map saved with
    `save_patches` (`poses.txt`) and a robot standing still.

  The map gateway is separate on purpose: the map router has no business holding
  a handle that can make the robot move.

**Publishers:** `initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, the
localization seed) and `cmd_vel` (`geometry_msgs/Twist`, teleop output and the
zero-velocity watchdog).

**TF:** a `TransformListener` with `spin_thread=False` (it rides the node's
executor instead of adding another GIL-contending thread), used only to bring
`body_cloud` into `map`.

## REST API

Interactive docs: `http://<robot>:3000/docs`.

| Method | Path | Notes |
|---|---|---|
| GET | `/health` | Liveness probe (always 200); `status` is `degraded` while `task_server` is not `running` (`connecting`/`dead` + `task_server_error`) |
| POST | `/api/v1/tasks` | Start a `RobotWorkflow`; body `{id, steps[], kind?, name?, map_name?}` (legacy `timestamp` ignored). `kind` (`goal` / `standup` / `liedown` / `task`; `schedule` is the backend's and refused) and `name` are stamped as search attributes for the history. A job with a MOVE / DOCK is checked against the loaded map — 409 `map_mismatch` / `conversion_running`, see *A job's map is checked twice*; stand / lie down / UNDOCK never are. 409 `task_running` while any task runs — one task at a time |
| GET | `/api/v1/tasks/{id}` | Overall status + per-step state (workflow query): `id`, `status`, `error_msg`, and `message` — for a running `MOVE` / `DOCK` / `UNDOCK`, the robot navigator's live status sentence (`NavigateToGoal` feedback `status_msg`, read from `RobotGateway`), empty otherwise. A held run answers `status: PAUSED`, its held step `PAUSED` — the only place a hold is visible |
| DELETE | `/api/v1/tasks/{id}` | Request cancellation; answers `CANCELING` — the final state (possibly still `COMPLETED`) comes from GET |
| POST | `/api/v1/tasks/{id}/pause` · `/resume` | Hold / release the running task — *requests* (workflow signals): pause answers `PAUSING`, resume `IN_PROGRESS`; the hold is read back from GET. A `MOVE` / `DOCK` / `UNDOCK` is interrupted at once and re-sent on resume, a `WAIT` freezes its countdown, a `SPEAK` or posture step finishes first — see *Task orchestration*. 409 `task_not_running` once closed; 404 for another robot's id |
| GET | `/api/v1/active_tasks` | What runs on this robot's queue now, *whoever* started it, with `source` / `schedule_id` / `kind` / `name` / `map_name` (the run's `TaskMap`; for a run older than the attribute, the loaded map unless it is a stand / lie down; `null` for a run that drives nowhere). A held run is still listed as `IN_PROGRESS`. Not `/tasks/active`, which would shadow `/tasks/{id}` |
| GET | `/api/v1/task_history` | **Finished** runs, newest close first, one visibility page per call. Query: `page_size` (1–100, default 20), `page_token` (previous `next_page_token`), `status` (`COMPLETED` / `FAILED` / `CANCELED`), `since` / `until` (close time; naive is UTC; `until` before `since` is 400), `kind` (`goal` / `standup` / `liedown` / `task` / `schedule`), `name` (exact template name). `kind` / `name` / `map_name` are `null` for runs dispatched without them or predating the attribute — no fallback. Bounded by namespace retention |
| GET | `/api/v1/task_history/stats` | The same runs counted: `total`, `by_status`, `success_rate` (`COMPLETED` / `total`, `null` when nothing finished). Same filters as `task_history`; one count RPC |
| POST | `/api/v1/schedules` | Create a schedule (cron **or** interval). A cron with its own `#`, a `CRON_TZ=`/`TZ=` prefix (use `timezone`) or `@every` (use `interval_seconds`) is 400 — see *Task orchestration* |
| GET | `/api/v1/schedules` | List with next run times. This robot's only, filtered by the server on the schedule's `RobotId` (see *Provenance* under task history) |
| GET | `/api/v1/schedules/{id}` | Describe one |
| PATCH | `/api/v1/schedules/{id}` | Change when it fires: `{trigger: {cron, timezone}}` or `{trigger: {interval_seconds}}`, replacing the old rule whole. In place — id, frozen steps, provenance, paused state and `SKIP` stay. No renaming (delete + create). Same cron restrictions |
| DELETE | `/api/v1/schedules/{id}` | Delete |
| POST | `/api/v1/schedules/{id}/pause` · `/resume` | Pause / unpause |
| POST | `/api/v1/task_templates` | Store a re-dispatchable step list |
| GET | `/api/v1/task_templates` | List; `?map_name=` gives that map's **plus** the map-independent ones |
| GET | `/api/v1/task_templates/{id}` | One, with vertex references resolved |
| PUT | `/api/v1/task_templates/{id}` | Partial update; `steps` replaces the whole list |
| DELETE | `/api/v1/task_templates/{id}` | Delete |
| POST | `/api/v1/task_templates/{id}/schedule` | Freeze the current resolution into a schedule, through the same path (and cron restrictions) as `POST /api/v1/schedules` |
| GET | `/api/v1/robot/state` | Latest state: pose in degrees, wifi, battery, byobu-session `mode`, `low_level_mode` (the gait controller's report plus the driver's `safety_locked`), and `localization_valid` — `false` means a zeroed placeholder pose (not relocalized, or mapping). 404 only until `robot_state` first publishes |
| POST | `/api/v1/robot/mode` | `switch_mode` on `syncai_sys_manager`. A real switch kills the byobu session **this backend is a pane of**, so the client usually sees a dropped connection — treat it as success-in-progress. A body reliably arrives only for the no-op (already in that mode) and a refusal |
| POST | `/api/v1/robot/restart` | `restart_mode` on `syncai_sys_manager`: rebuild the byobu session of the mode already live (what `/robot/mode` refuses as a no-op). No body. Answers within the 2 s ack window with `restarting: true`; sys_manager replies only when the rebuild ends, kept for the `GET` below. **409 `restart_refused`** with sys_manager's sentence, nothing touched — in MAINTENANCE, with both sessions up, and always in MANUAL (`pgo_node` may hold an unsaved map), so in practice AUTO only. **409 `restart_running`** while one is under way; 502 if sys_manager is unreachable |
| GET | `/api/v1/robot/restart` | How the latest restart this process dispatched went: `status` `idle` / `restarting` / `succeeded` / `failed`, `message`, `started_at` / `finished_at`. This, not `/robot/state` (which keeps serving the last frame through the rebuild), is how a console learns the outcome. Unanswered for 3 min reads `failed`; in memory, so a restarted backend says `idle` |
| WS | `/api/v1/robot/teleop` | Inbound manual control: `{vx, vy, wz}` JSON frames at ~10 Hz, each axis clamped to [-1, 1] and published as-is (m/s / rad/s). Refused (`{"error": ...}` frame, socket stays open) while an autonomous MOVE runs. A 0.5 s stale-input watchdog and disconnect both publish zero velocity — driver_manager has no cmd_vel watchdog |
| POST | `/api/v1/localization/auto` | Find the robot on the loaded map with **no hint** (`reset_odometry` with `skip_seed`: reset Point-LIO, then the global search — kidnap recovery) — the alternative to `set_initial_pose`. The robot must be standing still. No body; blocks up to ~30 s. 200 = localized, nothing to poll: `x` / `y` / `z`, `theta` (degrees), `fitness`, `inlier_ratio`, `candidates`. 409 `task_running` (any job — it moves the map frame), `tasks_unknown`, `localize_running` (one search at a time). Every localizer answer — not AUTO, no `poses.txt`, ambiguous, did not converge — is 502 with its sentence; the odometry has been reset by then, the map pose is left where it was. Writes nothing, not even `[initial_pose]` |
| POST | `/api/v1/robot/set_initial_pose` | Seed localization with a map-frame pose (degrees in, radians out); fire-and-forget. Optional `z`, the ground the pose stands on (a vertex's `z`), picks the storey: it goes out as `position.z` with the z variance (covariance cell 14) set, which is what tells `syncai_localizer` to use it. Omitted, the localizer keeps the height of its current estimate (RViz's 2D Pose Estimate does the same), so a pose on another storey needs `z` |
| POST | `/api/v1/robot/set_motion_key` | Gait key `"0"`–`"5"`; `"4"` (ESTOP) is accepted but **not** forwarded — 200 with `sent: false` |
| POST | `/api/v1/robot/estop` | `{"locked": bool}` → `set_safety_lock` on `syncai_driver_manager`. `true` engages the lock and, on the `false → true` edge only, starts cancelling every running task and nav goal in the background (`cancel_requested: true`; started, not finished). `false` releases it, cancelling nothing. 502 if the driver is unreachable, nothing cancelled. Not the ESTOP motion key |
| POST | `/api/v1/robot/set_policy_mode` | Gait-controller policy index; only `0` (PPO) and `1` (HIMLOCO) |
| GET | `/api/v1/network/wifi/scan` | Scan networks (blocks up to 45 s). A scan requested while one runs joins it and gets its answer, instead of holding a second thread and queueing a second scan in driver_manager |
| POST | `/api/v1/network/wifi/connect` | Connect via `nmcli` (blocks up to 70 s). Connect, activate and delete of a saved network change NetworkManager's state, so one runs at a time: the others are 409 `wifi_busy` |
| GET | `/api/v1/maps` | Map directories on disk with geometry, vertex counts and `grid_status` — **the conversion-status surface**; there is no job resource. `none` / `converting` / `ok` / `failed` (reason in `grid_error`) / `interrupted` (backend restarted mid-conversion). `grid_converting` is the deprecated boolean it replaces. The 3D map's twin: `octomap_status` (same five words, off `octomap.bt` and the cleaning sidecar), `octomap_error`, `octomap_version` (changes when the file does) and `octomap_resolution` |
| GET | `/api/v1/maps/{name}` | One map's summary |
| POST | `/api/v1/maps` | Save the current mapping run **and end it**: `pgo/save_maps` into a new map dir, then (unless `grid: false`) z-band conversion in a background thread, after the robot's map cleaning (`grid_pending` only means started — poll `grid_status`, which stays `converting` through the cleaning too). On success pgo goes idle and clears the "map so far" layer and `/dev/shm` itself, so the next run waits for `POST /api/v1/mapping/start`. `octomap: true / false` has the map cleaning build `octomap.bt` or not for this save (omitted or null: pgo's `map_clean_octomap`); watch `octomap_status`. 400 `nothing_to_build` for `grid: false, octomap: false`. 409 `mapping_idle` when nothing was started (before any directory is created), 409 `mapping_busy` mid start/reset. Mapping mode only in practice |
| POST | `/api/v1/mapping/start` | Begin a run: `pgo/start_mapping`, idle → mapping (the drive to the start point is not part of any map). No body; always resets the LIO front end (map origin = odometry origin), so the robot must be **stationary** until it returns. 409 `mapping_running` / `mapping_busy`; 502 with pgo's sentence. Touches no file |
| GET | `/api/v1/mapping` | pgo's last-reported run state: `state` `idle` / `mapping` / `resetting` / `unknown`, `key_poses`, `loop_closures`. `unknown` is a real answer — every navigating robot, or pgo silent for 5 s. Latched, so it survives a console reload |
| POST | `/api/v1/mapping/reset` | Discard the in-memory run and start a new map, restarting nothing: `pgo/reset_mapping` pauses intake, resets the LIO front end, rebuilds the pose graph; pgo stays mapping. No body and **no save** (saving stays a separate act — the usual case is abandoning a run gone wrong). Touches no file, hence `/mapping/`. 409 `mapping_idle` / `mapping_busy`; 502 with pgo's sentence, graph untouched. Robot must be **stationary** (static, gravity-aligning re-init) |
| PATCH | `/api/v1/maps/{name}` | Rename: moves `map/<old>/` to `map/<new>/`, re-keys `map_vertices` and `task_templates` rows. 409 `map_active` (the stack opened those files at launch — switch maps first), `conversion_running`, `name_taken`. Schedule memos keep the old `map_name` label (display-only) |
| DELETE | `/api/v1/maps/{name}` | `rmtree` of `map/<name>/` plus its `map_vertices` rows, no undo. The rename's first two refusals plus 409 `template_bound` while a template names the map (listed). Rows go **before** the directory — the irreversible step goes last. Registered schedules keep firing their frozen steps |
| GET | `/api/v1/maps/{name}/export?format=zip\|tar.gz` | The map dir as one archive (default `zip`; `Content-Disposition: attachment; filename="<name>.<ext>"`), built into an anonymous temp file on the maps' disk and streamed with its `Content-Length`, never held in memory: files relative to the root — no top-level folder, no `traversable_debug/` — plus `syncai_map.json` `{format, version: 1, name, exported_at, files: {relpath: md5}, vertices: [...]}`. Vertex **ids are not exported** (regenerated on import); templates are not included (their steps bind vertex ids). 409 `conversion_running`; the active map may be exported |
| POST | `/api/v1/maps/import?name=` | Create or **replace** a map from such an archive; raw `application/octet-stream`, zip / tar.gz by magic bytes. **Nothing is written until the archive is judged**: manifest required (version 1; newer refused), every file md5-verified; an unlisted or missing member, a link/device member, an absolute or `..` path, no `map.pcd`, or an unknown vertex type is 400. Name is `?name=` else the manifest's. Replacing takes the delete's refusals (`map_active` / `conversion_running` / `template_bound`) plus 409 `disk_low`. The body is streamed to an anonymous temp file on the maps' disk as it arrives (1 MB held at most); **413 `archive_too_large`** past 4 GiB (`MAP_IMPORT_MAX_BYTES`), on `Content-Length` before reading and on the bytes received while reading. 201 `{name, replaced, files, bytes, vertices_created, vertices_deleted, message}`. Starts no conversion; an imported `gridmap.recipe.json` is kept as-is. Staging: see *Layering* |
| POST | `/api/v1/maps/{name}/activate` | Switch maps live, no session restart: `relocalize` first (its refusals precede any mutation), then map_server `load_map`, then `[map] name` into the instance INI so it survives a restart (`[initial_pose]` zeroed — re-seed). `switched: false` is the no-op. `localized` is a best-effort `relocalize_check` poll: `false` = not yet (registration retries forever), `null` = could not ask; **not** `RobotState.localization_valid`, which is TF-presence only. Refusals, all before mutating: 409 `grid_missing`, `pointcloud_missing`, `conversion_running`, `ini_not_writable`, `task_running` (any job), `tasks_unknown` (refuse rather than assume idle), `stack_not_ready` (service discoverability — how mapping mode is detected, not the cached mode). Lifts `map_active` on rename and delete. Also hands `filter_mask_server` this map's `keepout.yaml` (blank if never booted), since `load_map` leaves the *previous* zones in force; best-effort and after the commit point, so `keepout_reloaded: false` = switched, old zones may still apply (`null` on the no-op). **3D maps (2026-10):** a map with an `octomap.bt` is 3D (with a grid too, still 3D — the robot's own rule), otherwise it needs a usable grid (`grid_missing` names both). After `load_map` (skipped when a 3D map has no grid; map_server then keeps the previous grid, which nothing plans on), the robot's 3D planner is pointed at the map: `planner_server/Octo3D/load_map` with the `octomap.bt` for a 3D map, **with an empty `map_url` for a 2D one** — that unload is what moves the robot off its 3D trees, which it runs whenever Octo3D has a map. It blocks until the tree is built (120 s bound) and its success is a result; a failure rolls back the localizer, the grid and Octo3D (onto the previous map's tree, or none). 409 `planner_3d_unavailable` refuses a 3D map on a stack without Octo3D; on such a stack a 2D switch skips the unload. A 3D map's forbidden zones come in with its tree (Octo3D reads `keepout.json` in `load_map`), so `keepout_reloaded` is `true` and no mask is touched. `map_kind` is `2d` / `3d` (`null` on the no-op) |
| POST | `/api/v1/maps/{name}/grid/convert` | (Re)build the gridmap from `map.pcd`: recipe `z-band` (default; optional `z_band_offsets`, `floor_reference` `local` default / `global`) or `traversability` (`gap_fill_size`), `debug` for intermediate clouds. `started: true` only means the thread launched — outcome in `grid_status`. 409 `conversion_running`, or `gridmap_hand_edited` (confirm with `overwrite_edits`; the edit survives as `gridmap_prev.pgm` + `gridmap_prev.recipe.json`). Reloads map_server when active — decided when the grid lands, not at the request: a switch away during the (minutes-long) conversion is not refused, and reloading then would put map_server on this grid while the localizer, Octo3D and the INI name the other map |
| GET | `/api/v1/maps/{name}/image` · `/thumbnail` | Gridmap as full-size / downscaled PNG, ETag'd on the `.pgm`'s `(inode, size, mtime)` — a save is a new inode (`write_pgm` renames), so the tag changes without the request reading the file; a 304 costs one `stat`. The encoded PNGs (and the packed cloud) are kept in an LRU of `RENDER_CACHE_MAX_BYTES` (64 MB, about a dozen maps), not for every map ever opened |
| PUT | `/api/v1/maps/{name}/grid` | Write edited cells (raw `application/octet-stream`); reloads map_server when active. 409 `conversion_running` (it would overwrite the edit) |
| GET | `/api/v1/maps/{name}/keepout` | Forbidden zones as saved through this API: `zones: [{id, points: [{x, y}, …], name, z_min, z_max}]` in map-frame metres (the last three `null` when unset; the band is the floor on a 3D map), plus `active`. Read from `keepout.json`, so a boot-time blank or hand-drawn mask answers `[]` |
| PUT | `/api/v1/maps/{name}/keepout` | Replace the zones with the **whole** body list (`{zones: [{id?, points: [{x, y}, …], name?, z_min?, z_max?}]}`; ids kept when given, generated when not). Rasterises into `keepout.pgm` + `keepout.yaml` (not on a 3D-only map, which has no grid), records `keepout.json`, then (map active) reloads by kind: `planner_server/Octo3D/load_keepout` for a 3D map, `filter_mask_server/load_map` for a 2D one — `reloaded: true` means enforced now. `[]` **clears**: mask rewritten all-unknown and reloaded, files kept (deleting them changes nothing in the running filter). A failed reload is still **200** with `reloaded: false` and the reason — zones apply at the next boot of this map. 400 for a zone under three points, a non-finite coordinate or height, `z_min` above `z_max`, an empty or duplicate id; 404 with neither a gridmap nor an `octomap.bt`; 409 `conversion_running` |
| GET | `/api/v1/maps/{name}/pointcloud` | The saved `map.pcd`, packed binary |
| GET | `/api/v1/maps/{name}/octomap` | The saved `octomap.bt` byte for byte (`application/octet-stream`), ETag = `"<octomap_version>"`, 304 on `If-None-Match`. 409 `octomap_converting` while the map cleaning is still building it; 404 with the reason otherwise (never built, `failed` with the cleaning's sentence, `interrupted`) |
| GET | `/api/v1/maps/{name}/octomap/edits` | The ground patches saved for the 3D map: `{name, res, fill_key: [[kx, ky, kz], …], created_at}`, absolute OcTree keys of the voxels to mark as floor. `fill_key: []` (and null `res` / `created_at`) when none are saved |
| GET | `/api/v1/maps/{name}/floors` | The storeys the map spans, lowest first: `{name, floors: [{name, level}], detected: [...], saved}`. `level` is the body height walked on that floor (the z of `poses.txt`), metres. `floors` is the saved list if there is one, else `detected`; `[]` is one storey |
| PUT | `/api/v1/maps/{name}/floors` | Replace the floors with the **whole** body list (`{floors: [{name, level}, …]}`); `[]` saves one storey. Writes `floors.json`, sorted by level. 400 with a sentence for an empty or over-long name (16), a repeated name, more than 16 floors, or two levels under 1 m apart. Not held by a running job: nothing on the robot reads it |
| PUT | `/api/v1/maps/{name}/octomap/edits` | Replace the patches with the **whole** body list (`{fill_key: [[kx, ky, kz], …]}`, 0–65535 each, one per `[kx, ky]` column); `[]` clears. Writes `octomap_edits.json` (tmp + rename, `res` from the tree's header) and **nothing else** — the tree is patched on the robot by `apply_octomap_edits`, the message says so. 400 with a sentence for an out-of-range key or a column named twice; 404 with no tree; 409 `octomap_converting`, `octomap_unreadable` (a tree without a readable header), `task_running` / `tasks_unknown` (held by a running job like every map write) |
| POST · GET | `/api/v1/maps/{name}/vertices` | Batch-create (one transaction) / list with optional `?type=`. Create is held by a running job (below) |
| GET · PUT · DELETE | `/api/v1/maps/{name}/vertices/{id}` | Read / partial update / delete; update and delete held by a running job |
| GET | `/api/v1/tts/voices` | Voice ids the speech service's model carries |
| POST | `/api/v1/tts/synthesize` | Render `{text, voice, speed}` to WAV (`audio/wav`) without playing |
| POST | `/api/v1/tts/speak` | Same body, played on the robot speaker; blocks for the utterance (`duration` returned). Unknown voice 400; full queue 409 `tts_queue_full`; anything else (service unreachable, weights missing, wedged speaker) 502. English only, ≤1000 chars; `speed` 0.5–2.0 |
| POST | `/api/v1/recordings` | Start `ros2 bag record` into `record/<name>/`. Body `{name?, topics?, compression?}`; `name` defaults to `rec_<UTC timestamp>`, `topics` to the LIO inputs (`livox/lidar`, `livox/imu`). A topic without a leading slash resolves under this robot's namespace (`/tf`, `/tf_static` for fleet-wide). 201 = the recorder survived its liveness probe. Refusals, before any spawn: 409 `recording_running` (name in message), `name_taken`, `disk_low` (< 2 GB free); 400 for a reserved or malformed name or empty `topics` |
| GET | `/api/v1/recordings/active` | The live recording with `elapsed_seconds` and `size_bytes`, or `null`. Its own route so a 1 Hz poll does not walk every bag on disk; also where a recorder that died on its own is reaped |
| POST | `/api/v1/recordings/stop` | SIGINT the recorder and block until it has flushed — with compression, zstd of the open split, about a minute for a full 2 GB one on the Orin. Escalates to SIGTERM/SIGKILL only after 15 s with no write under its directory (or 5 min total), so set client timeouts accordingly. `complete: false` = killed; run `ros2 bag reindex` (the messages are there). 409 `not_recording` |
| GET | `/api/v1/recordings` | Every bag under `record/`, newest first: `status` `recording` / `ok` / `interrupted` and, once finished, `duration_seconds`, `message_count`, topics recorded. `message_count: 0` signals a topic typo — nothing refuses a topic that does not exist yet |
| DELETE | `/api/v1/recordings/{name}` | `rmtree`, no undo. 409 `recording_active` for the live one, else 404 |
| POST | `/api/v1/webrtc/whep` | Camera to browser (WHEP): body is the SDP offer (`application/sdp`); **201** with the answer and a **relative** `Location: /api/v1/webrtc/whep/{session_id}`. Preempts a live `video` or `duplex` (one camera). 400 for an empty, oversized (> 64 KiB), non-UTF-8 or media-less offer; 409 `whep_session_pending` while another create is mid-handshake (retry); anything else, a missing `.so` included, 502 with the worker's sentence. Blocks for ICE gathering (0.1–5 s) |
| POST | `/api/v1/webrtc/whip` | Browser mic to robot speaker (WHIP), same shape; `Location: /api/v1/webrtc/whip/{session_id}`. Preempts `audio` or `duplex` (one speaker); coexists with `video` |
| POST | `/api/v1/webrtc/duplex` | Both directions on one peer connection; holds camera **and** speaker, so preempts everything |
| DELETE | `/api/v1/webrtc/{whep,whip,duplex}/{session_id}` | Tear down. **204 even for an unknown id** — the worker cannot tell "already self-closed" from "never existed" |
| GET | `/api/v1/webrtc/status` | `library_loaded` plus the `video` / `audio` / `duplex` slots (`session_id`, `age_seconds`) as this backend believes them. No callback from the worker, so an abandoned session reads live until the next create preempts it. Deliberately not in `/health` |
| WS | `/api/v1/robot/pointcloud/stream` | Live `body_cloud`, ~10 Hz |
| WS | `/api/v1/robot/pointcloud/map/stream` | pgo's merged "map so far", every few seconds at most, mapping mode only; each frame replaces the whole layer |
| WS | `/api/v1/robot/telemetry/stream` | JSON frames keyed by `type`: `pose` (~20 Hz), `joints`, `path` (1 Hz), `dock_path` (2 Hz), and while a navigation runs `local_costmap` (2 Hz) and `local_plan` (5 Hz) |

**A job's map is read-only while it runs.** Vertex create / update / delete,
`PUT …/grid`, `PUT …/keepout`, `PUT …/octomap/edits` and `POST …/grid/convert` answer 409 `task_running`
while a running job holds the map (its `map_name` in `GET /api/v1/active_tasks`;
`ActiveTask.map_in_use`), and 409 `tasks_unknown` when running jobs cannot be
listed — refuse rather than assume idle. Each of those reloads into the running
stack or rewrites the rows a job's MOVEs came from. One async dependency
(`JobGate.refuse_while_a_job_holds`, `services/map_usage.py`) sits in front of the plain `def` routes: it awaits
Temporal on the loop, the route still runs in the threadpool. A map other than
the loaded one passes without asking — activate refuses while anything runs, so
only the loaded map can be held, and editing an idle map should not need
Temporal. Reads are never held. Check-then-act, not a lock: a schedule firing in
between is not stopped, and `execute_move`'s own check covers a rebuild. Rename,
delete and import-replace already refuse the loaded map (`map_active`).

**Telemetry** is the internal visualization channel and shares no models with
`GET /api/v1/robot/state` — that payload is frozen, this one may change freely.
It is its own socket so a 360 kB cloud frame cannot head-of-line block pose;
`path` rides it because a thinned route is ~8 kB every 3 s. Its `points` are
`[x, y, z]` in map metres, z the ground each one stands on (OctomapPlanner3D's
floor surface, 0 from a 2D planner) — a waypoint's z — so a route that climbs a
stair can be drawn storey by storey. `pose.z` is base_link's height, not the
ground: in AUTO x, y and yaw are lio_bridge's, but its chain is projected onto
the plane (z = 0 on every storey), so the height is taken from the
`pointlio/lio_odom` chain while one is under a second old — the same number a
mapping run reports. An **empty**
`path.points` means "no route": the planner never publishes an empty plan, so
`TelemetryRepo` clears a route by TTL (arrival, cancel and abort are the same
silence from here).

**The local picture rides the same socket, only while a navigation runs**
(2026-10-20): `local_costmap`, the controller's 4 × 4 m rolling costmap (on a
2D map the obstacle layer's, on a 3D map the terrain layer's, inflated), and
`local_plan`, the 2 s trajectory MPPI is following. "Runs" is a MOVE, DOCK or
UNDOCK goal this process sent that has not ended (`RobotGateway.navigation_active`)
— not a route on `plan`, which the planner keeps re-sending after a success and
a failure. When the goal ends, however it ends, each client gets one empty
frame of each (`width` 0, `points` []), the clear; a goal sent from rviz or the
CLI shows neither. The costmap is read off `costmap/raw`, not the
OccupancyGrid: that one carries only incremental updates while the robot
stands. Its cells go out as base64 bytes with the costmap's own values (0 free,
1–252 inflation, 253 inscribed, 254 lethal, 255 unknown), row-major from
`origin`, the grid turned by `origin.yaw` in the map frame. Both are planar —
draw them on the ground the robot stands on. Each also expires by its own TTL
(3 s, 1 s) when its source stops: MPPI publishes no local plan on a stair, in
the final rotation or the direct approach onto the goal.

**Errors.** Routers raise domain exceptions from `exceptions.py`; `server.py`
maps them:

| Exception | Status |
|---|---|
| `NotFoundError` | 404 |
| `BadRequestError` | 400, with an optional `code` and `step_id` beside `detail` when the refusal is about one step of a task body (`invalid_step_group`; see Groups) |
| `ConflictError` | **409**, with an optional machine-readable `code` beside `detail` (e.g. `conversion_running`, `gridmap_hand_edited`, `map_active`, `restart_refused` / `restart_running`, `recording_running`, `tts_queue_full`, `whep_session_pending`; each route's row lists its own). Clients branch on the code — e.g. re-convert's confirm-and-retry is only for `gridmap_hand_edited` — never on prose, which exists to be reworded |
| `UnauthorizedError` | 401 |
| `UpstreamError` | **502** — a downstream (Temporal or a ROS service) failed |

**Point-cloud wire format** (both WS streams and `GET /api/v1/maps/{name}/pointcloud`):

```
[ uint32 LE point_count ][ float32 LE x, y, z ] * point_count      # map frame
```

A browser reads it straight into a typed array. Both streams share one `_pump`,
**frame-driven** — it waits on the single-slot repo's notification instead of
polling (polling cost ~50 ms latency and ~5 % dropped frames from two
unsynchronised 10 Hz clocks), and the single slot still drops, so a slow client
gets the newest frame, not a backlog.

### Recordings

Bags land in `~/robot_ws/record/<name>/` (gitignored), the layout
`ros2 bag record` writes by hand: `<name>_N.db3` splits cut at 2 GB plus `metadata.yaml`.
As its own container the directory is bind-mounted from `record/` beside the
compose file (`RECORD_DIR` moves it to a bigger disk), not from the workspace,
because nothing else in the stack reads bags. A bag is insurance against a
mapping run that ends without a save: `pgo_node` holds its keyframes in RAM, and
replaying `livox/lidar` + `livox/imu` is the only way back.

Two load-bearing properties, easy to undo by accident:

- **It is not in its own session.** The child shares the backend's process
  group, so a `switch_mode` (which kills the byobu session this backend is a pane
  of) takes it down too. In its own session it would outlive the backend, keep
  writing, and be unstoppable — the only handle is a slot in memory.
- **Its output is inherited, not piped.** An undrained pipe deadlocks the
  recorder at 64 KiB; inherited, `rosbag2_recorder`'s lines land in the backend's multilog
  (`log/stack/<robot_id>/backend/current`).

**A topic is only recorded if its type is installed here.** `ros2 bag record`
subscribes generically but still loads the type's `rosidl_typesupport_cpp`
library via the ament index; an unknown type is skipped with a log warning and is
simply absent from the bag. `syncai_common/msg/*` is in the install space.
`livox/lidar` is `livox_ros_driver2/msg/CustomMsg` (`xfer_format: 1`), and the
real driver needs Livox-SDK2 to build, so the image builds
`docker/livox_ros_driver2` — the two messages only, same package name,
byte-identical `.msg` files, because DDS matches on the type name and the bag
records it for replay against the real driver. Any other package's type needs
the same treatment; a robot workspace's install space already has the real driver.

`interrupted` is derived, never stored: a directory with no `metadata.yaml` and
no live process — a bag whose backend went away mid-recording. The same
disk-outlives-the-process split as the conversion sidecar.

### WebRTC

`/api/v1/webrtc/*` is signalling only. The media path — capture, H.264 encode,
pion — lives in `libsyncai_worker.so` (built in `SyncAI-WebRTC-Worker`, copied in
by hand — see *As its own container*), which `gateways/webrtc` dlopens. It is the
one FFI in this process and not free: loading installs the Go runtime's signal
handlers next to rclpy and CycloneDDS, a Go c-shared library cannot be
`dlclose`d, and a fault in it kills the backend. Hence the **lazy load** —
construction touches nothing, the first create pays, and a boot that never
streams carries none of it.

- **One slot per kind, arbitrated by device.** One camera, one speaker: `whep`
  holds the camera, `whip` the speaker, `duplex` both. A create preempts every
  live session sharing a device, so a dead tab that never sent DELETE cannot
  lock the camera out. The worker knows none of this; left alone it would build
  a second session and fail deep in GStreamer with an ALSA or V4L2 code.
- **Handlers are plain `def`**, the offer a `bytes` body parameter rather than
  `Request.body()` (a coroutine), because a create blocks in cgo until ICE
  gathering completes; an async handler would stall the telemetry, point-cloud
  and teleop sockets.
- **Neither of the gateway's two locks is held across a worker call**, so a
  DELETE never queues behind a multi-second create.
- A camera held by something else (e.g. `scripts/publish_camera_crop.sh`) is a
  plain 502 with the worker's sentence and no `code` — the Go error string
  cannot tell it from any other pipeline failure.

### Vertex vs. MapPoint

REST says **"vertex"** with a `VertexType` enum (`MOVE` /
`CHARGE`); the ORM and repository say `MapPoint` (table
`map_vertices`). Intentional — no migration was done. `type` is validated at the
REST boundary and stored as a plain string.

### Task templates

`POST /api/v1/tasks` creates *and dispatches* and persists nothing, and Temporal
keeps closed workflows only for the namespace retention (a day on a fresh
install — see *Task orchestration*), with no archival. `task_templates` is where
re-dispatchable step lists live. The prefix is `/api/v1/task_templates`, not
`/api/v1/tasks/templates`, which would collide with `/api/v1/tasks/{id}` (see
the include-order note in `server.py`).

- **`steps` is one JSON column, not a child table.** No migrations exist (the
  schema is whatever `create_all` produced), so a column list could never change
  again, while a JSON array can grow optional keys. It is also the only record of
  step *order*, and a child table would be rewritten whole on every edit anyway.
  **Forward-compat rule: only add optional keys to a stored step; never rename,
  retype or repurpose one.**
- **A MOVE / DOCK step keeps both a `vertex_id` and a `params` snapshot**
  (a DOCK typically references a `CHARGE` vertex; an UNDOCK has no params and
  cannot reference one); every read
  reports `resolved_params` — the vertex's *current* pose if it exists
  (`vertex_status: CURRENT`), else the snapshot (`MISSING`). Moving a dock
  updates every template using it. Resolution is server-side (one
  implementation); the client dispatches by sending `resolved_params` through
  `POST /api/v1/tasks`.
- **Map scoping keys off "contains a MOVE / DOCK", not "references a
  vertex"** — a hand-typed `(x, y, theta)` is in a map's frame too. Any such step ⇒
  `map_name`
  required; none ⇒ `map_name` absent, runs anywhere. A template for a map that is
  not active still saves (authoring ahead is legitimate), reported with
  `map_matches_active: false` for the client to gate on.
- **Cross-field rules answer 400 with a sentence**, not 422 with a validation
  array: the array is unreadable to an operator, and a `PUT` may conflict with
  the *stored* row, which no request-schema validator can see.

## Task orchestration (Temporal)

A task is an ordered list of steps; `RobotWorkflow` walks them one at a time by
`StepType` — or a few at a time, where adjacent steps share a `group` (below):

| StepType | Activity | What it does |
|---|---|---|
| `MOVE` | `execute_move` | Send a `NavigateToGoal` goal, poll to a terminal state, heartbeat every 0.25 s; a failure's message carries the robot's `error_msg`. A robot still on the dock the last DOCK ended on is stood up (if lying) and undocked first — *Auto-undock* under *Action client* |
| `DOCK` / `UNDOCK` | `execute_dock` / `execute_undock` | `execute_move` on the dock trees (`behavior_tree` `dock.xml` / `undock.xml`, see *Action client*). DOCK's `MoveParams` is the dock pose and gets MOVE's map check; UNDOCK takes no params, sends an empty pose and is checked against no map, since it backs out 1 m from wherever the robot stands. DOCK's success records the dock for the next MOVE's auto-undock, UNDOCK's forgets it; the console no longer offers UNDOCK, since a MOVE undocks by itself. Same poll, heartbeat and pause interrupt as MOVE; a cancel (or pause) also sends `MotionKey.LOCOMOTION`, best effort, because a halted dock tree stays in MPC |
| `STANDUP` / `LIEDOWN` | `execute_stand` / `execute_lie_down` | Send the motion key; fire-and-forget (see `activities.py`) |
| `SPEAK` | `execute_speak` | `TtsGateway.start_speak()` then `await_speak()` — queue the utterance on the speech service, then poll its job and heartbeat until it has played; a cancel `DELETE`s the job mid-sentence. `SpeakParams`: `text` (1–1000 chars, English only), `voice` (default `af_heart`; see `GET /api/v1/tts/voices`), `speed` (0.5–2.0) — the REST route's constraints, since both drive one gateway |
| `WAIT` | — (workflow timer) | Idle for `WaitParams.seconds` (0 < s ≤ 3600). Not an activity: a durable timer in `RobotWorkflow._run_wait`, so no slot of the body queue's one-thread executor, no heartbeat, and it survives a backend restart with the time already served |

(`ARTIFACT` — conveyor pickup/drop via the artifact backend's REST API — was
removed 2026-08 with `gateways/artifact/`; templates or schedules still carrying
one must be purged before deploying.)

**Groups — steps side by side.** A step may carry an optional `group` (any
string). Adjacent steps with the same `group` run side by side, and the next step
starts once all of them have finished — e.g. `MOVE(group=a)`, `SPEAK(group=a)`,
`STANDUP` speaks while it drives, then stands. Rules, checked on
`POST /api/v1/tasks`, schedules and templates (400 with a sentence, plus
`code: "invalid_step_group"` and the `step_id` that breaks the rule — the step
rejoining a closed group, or a group's second body step):

- a group's members must be **adjacent** — the same name again after another
  step is refused, not read as a second group;
- **at most one `MOVE` / `DOCK` / `UNDOCK` / `STANDUP` / `LIEDOWN`** per group: the body queue has one
  slot, so two would only run one after the other.

**Start order**: a group's members start **in list order**, each once the one
before it has *started* — not finished — and from then on they overlap:

| Member | Started when |
|---|---|
| `MOVE` / `DOCK` / `UNDOCK` | nav2 accepted the goal |
| `SPEAK` | its first utterance is queued (the speech service plays in accepted order) |
| `STANDUP` / `LIEDOWN` | the motion key is sent (the activity ends right there) |
| `WAIT` | its timer runs |

Scheduled all at once, as they were before, two `SPEAK`s raced through synthesis
and played in whichever order they finished, and a `SPEAK` could be heard
before the `STANDUP` listed ahead of it was sent. The workflow cannot see an
activity's heartbeats, so `MOVE` and `SPEAK` report their start with a
`step_started` workflow signal (`STEP_STARTED_SIGNAL`), sent from the activity
thread on the worker's own client; a group member is scheduled under an
activity id of the form `<step id>#<n>` (`group_activity_id`), which is both how
the activity knows to report and what the report is matched against — a lone
step keeps Temporal's id and reports nothing. A report that cannot be sent is
logged, not fatal: the next member then waits for that one to *end*, which keeps
the order and loses only the overlap. A member a pause holds before it starts
holds back the starts behind it; one that never started when a failure or a
cancel ends the group stays `PENDING`. Runs that scheduled a whole group at once
replay under the `group-start-in-order` patch.

**Fail-fast**: the first member to fail cancels the rest (a `MOVE` stops and a
`SPEAK` goes quiet, as on a task cancel) and fails the run; those siblings read
`CANCELED` / "Canceled: step <id> failed". A task cancel reaches every member. A
pause reaches each member on its own terms — the group's `MOVE` is interrupted
and re-sent on resume, a `WAIT` freezes, a `SPEAK` plays on — and the run holds
before the next step as usual. `GET /api/v1/tasks/{id}` echoes each step's
`group`. `group` is an optional key added to the stored step (no migration):
templates and schedules without it run exactly as before, and an ungrouped step
issues the same commands it always did, so pre-existing histories replay.

Details that matter when editing this path:

- **Two queues, split by what an activity occupies.** The body queue
  (`<robot_id>.ROBOT_TASK_QUEUE`, which also carries the workflow) runs `MOVE`,
  `DOCK` / `UNDOCK` and the posture steps in a single-worker `ThreadPoolExecutor` with
  `max_concurrent_activities=1` — one thing at a time, like the robot — so
  Temporal holds a second body activity server-side instead of handing it over
  to queue with its timeouts ticking, and two runs can never drive at once. The
  IO queue (`<robot_id>.ROBOT_IO_QUEUE`, `IO_ACTIVITY_SLOTS` = 4) runs `SPEAK`,
  which only talks to the speech service; that is what lets a grouped `SPEAK`
  play while its `MOVE` holds the body slot. Both workers live in the same
  thread and share one client. `SPEAK` moved queues behind the
  `speak-on-io-queue` patch, and the body worker still registers
  `execute_speak` for runs that scheduled it there before the upgrade.
  Two runs' `SPEAK`s may now overlap, as a task's `SPEAK` and a manual
  `POST /api/v1/tts/speak` always could.
- **Activities are synchronous**. Cancellation is
  *thrown* into the thread as `CancelledError` (landing once a blocking call like
  `time.sleep` returns), so cleanup is an `except CancelledError:`, not an
  `is_cancelled()` poll. `execute_move` wraps the send and the poll loop in one
  `except CancelledError` calling `cancel_active_moves()` under
  `activity.shield_thread_cancel_exception()`, so the goal really is cancelled
  before the activity dies. By goal *state*, not id, because the cancel can land
  inside `move()` before nav2 answers — and for a goal nav2 accepts just after,
  the gateway disowns it: whichever of the two threads (the waiter, the rclpy
  response callback) comes second sees what the first did and cancels.
- **A cancel only reaches an activity on a heartbeat.** The server never pushes;
  it answers the next heartbeat with "cancel requested". So a paused or cancelled
  MOVE keeps driving — and a cancelled SPEAK keeps talking — for the heartbeat
  *send* interval plus the poll sleep. The SDK throttles sends to 0.8 ×
  `heartbeat_timeout` by default: 2.4 s under MOVE's 3 s, a full minute under
  SPEAK's 75 s. Both workers cap it at `HEARTBEAT_THROTTLE_MAX` (0.5 s,
  `temporal/worker.py`), and both poll loops — `_wait_for_nav_goal`'s
  `NAV_POLL_INTERVAL_S` and `await_speak`'s `SPEAK_POLL_INTERVAL_S`, 0.25 s
  each — run at or under it, so nav2 hears a cancel within ~0.75 s instead of
  ~3.5 s, and the speaker goes quiet as fast. The heartbeat *timeouts* are
  untouched; `test_activities.py` pins the relations for both.
- **MOVE heartbeats before it sends.** The heartbeat clock starts at activity
  start and `move()` has no loop to heartbeat from, so its two waits (server
  ready, goal accepted) are bounded by `NAV_GOAL_SEND_BUDGET_S` in the gateway,
  and `MOVE_HEARTBEAT_TIMEOUT` must stay above it (pinned in
  `test_activities.py`). The old 30 s / 10 s waits were unreachable under a 3 s
  heartbeat anyway; a slow nav2 gets its chance from the retry policy.
- **`SPEAK` heartbeats while it plays, and a cancel stops it mid-sentence.**
  The speech service's playback is a job (POST queues it and returns an id, GET
  reports its state, DELETE stops it), so `execute_speak` queues with
  `wait=false` and polls the job every `SPEAK_POLL_INTERVAL_S` (0.25 s,
  `gateways/tts/tts.py`), heartbeating on each read — the shape
  `_wait_for_nav_goal` has for MOVE. A cancel (a task cancel, or a sibling in
  its group failing) lands in `except CancelledError`, which `DELETE`s the job;
  the service SIGTERMs `aplay`. A *pause* never cancels a SPEAK — it finishes
  and the run holds after it. The POST is the one stretch with no heartbeat (the
  service synthesises inline, and a cold one loads its model first), so
  `SPEAK_HEARTBEAT_TIMEOUT` (75 s, `temporal/workflows.py`) outlasts
  `SPEAK_START_TIMEOUT_S`; the POST is shielded, so a cancel arriving during it
  still finds the job id to stop. The long window does not slow a cancel
  only because the throttle is capped (*A cancel only reaches an activity on
  a heartbeat*, above). The **5-minute `start_to_close`** stays, far shorter
  than the others' hour; the gateway gives up on a backed-up speech queue at
  240 s with a sentence before it fires. `POST /api/v1/tts/speak` still uses
  the blocking `speak()` (`wait=true`).
- **Per-step state is a workflow query** (`get_step_states`), not a table.
  `GET /api/v1/tasks/{id}` degrades to an empty step list if the query fails (no
  worker polling yet) instead of failing the request.
- **The hold is two workflow signals** (`pause` / `resume`, via
  `POST /api/v1/tasks/{id}/pause` · `/resume`) plus a flag; both idempotent, so a
  second pause or stray resume changes nothing. The run checks the flag before
  every step and, if set, parks that step as `PAUSED` in a `wait_condition` until
  resume — the whole hold for `SPEAK` and posture steps, which finish and stop at
  the next boundary. A `WAIT` freezes at once (its `wait_condition` returns, the
  step reads `PAUSED`) and on resume waits out the remainder, measured on
  `workflow.now()`; a cancel during it, as during any hold, is `CANCELED` / "Task
  canceled". Only a `MOVE` is cut short: `pause` cancels the in-flight activity
  handle, landing in `execute_move`'s `except CancelledError` and cancelling the
  nav2 goal exactly as a task cancel would; resume re-dispatches the step from
  scratch (same target, fresh attempts, from wherever the robot is). `DOCK` /
  `UNDOCK` are interrupted the same way (`_run_move` serves all three): the dock
  tree is halted mid-approach, `execute_dock` puts the walking gait back
  (`MotionKey.LOCOMOTION` — the halted tree leaves the robot in MPC), and resume
  starts the tree over: a DOCK from its staging leg, an UNDOCK with a fresh 1 m
  back-out from wherever the pause stopped it — so a paused undock ends up to
  2 m off the dock, not at a fixed staging pose. Consequences:
  - **Temporal has no paused status.** A held run is `RUNNING` to the server;
    the gateway derives task-level `PAUSED` from the step list the existing
    query returns — one query per console poll, and a failed query honestly
    reads `IN_PROGRESS`. The REST acks say `PAUSING` / `IN_PROGRESS`, never
    `PAUSED`: like `CANCELING` they describe the request.
    `GET /api/v1/active_tasks`, `_require_idle` (so `POST /api/v1/tasks` still 409s
    `task_running`) and the map activation gate all treat a held run as running.
  - **Teleop opens during a hold.** The MOVE's goal is gone, so
    `_autonomous_move_active()` is false and `cmd_vel` is accepted; resume
    re-sends the goal from wherever the operator drove to.
  - **A cancelled step reads `CANCELED` / "Task canceled"** whatever it was
    doing. It used to read `FAILED` / "Cancelled" mid-step, when the SDK wrapped
    the cancel in the activity's `ActivityError`; the workflow folds that shape
    too.
  - **The MOVE wait never `await`s the activity handle.** `_run_move` waits on
    `workflow.wait_condition(handle.done() or paused)`, so a task cancel always
    arrives as `asyncio.CancelledError` (the activity is then cancelled and
    waited out, `WAIT_CANCELLATION_COMPLETED`), and an
    `ActivityError(cause=CancelledError)` can only be the pause's own
    interruption. A pause and cancel in the same breath end the run canceled,
    never held — without `workflow.cancellation_reason()`. (`temporalio` is
    floored at 1.33; the suite pins this on 1.34.)
  - **Replaying runs started before the hold existed is safe** as long as nobody
    pauses them: the happy path emits the same commands (one activity per step;
    `wait_condition` emits none).
- **Task history is Temporal's visibility index, not a table.**
  `GET /api/v1/task_history` lists closed executions on this robot's queue (the same
  `WorkflowType` + `TaskQueue` scope as `active_tasks`); rows carry no steps —
  detail is `GET /api/v1/tasks/{id}`, which works on a closed run while a worker
  is polling.
  - **Bounded by namespace retention.** The auto-setup default is one day, left
    on purpose until 2026-10, when the history dashboard's time-range filter made
    a longer window worthwhile. The Temporal container sets it, not this repo. On
    a running stack (the namespace exists, so an env var does nothing):
    `docker exec temporal sh -c 'temporal operator namespace update --address $(hostname -i):7233 --namespace default --retention 30d'`
    — applies to runs closing from then on; closed runs keep their expiry. For a
    fresh install, `DEFAULT_NAMESPACE_RETENTION` on the workspace compose's
    `temporal` service. The backend needs no change.
  - **Provenance is three Keyword search attributes**, `TaskKind`, `TaskName`,
    `TaskMap`, stamped by `start_task` from the request's `kind` / `name` and the
    map loaded at dispatch, and by every schedule's action (`kind = schedule`,
    the template's name, the schedule's `map_name`). A fourth, `RobotId`, is on
    the **schedule itself**, not its runs: `GET /api/v1/schedules` lists with
    `RobotId = '<robot_id>' OR RobotId IS NULL`, so the server drops the rest
    of the fleet's schedules instead of this process pulling and memo-filtering
    them all (`IS NULL` keeps the ones from before the attribute, which the memo
    filter still scopes). While `RobotId` is unregistered the server answers that
    filter INVALID_ARGUMENT and the listing falls back to the whole namespace,
    with a warning. Checked against 1.29.7 on Postgres visibility. The worker
    registers all four at connect (`ensure_search_attributes`, idempotent,
    non-fatal); by hand:
    `docker exec temporal sh -c 'temporal operator search-attribute create --address $(hostname -i):7233 --namespace default --name TaskKind --type Keyword --name TaskName --type Keyword --name TaskMap --type Keyword --name RobotId --type Keyword'`.
    A start naming an unknown attribute is refused, so until `TaskMap` exists
    every dispatch stamping it answers 502 (and until `RobotId` exists, every
    schedule create); until they all exist, `kind` / `name`
    filters and `/stats` answer 502 and the log names the attribute.
    `kind=schedule` is answered from `TemporalScheduledById IS NOT NULL` (stamped
    by the server), so pre-attribute schedules still count (their runs carry no
    name).
  - `status` folds Temporal statuses like `_WORKFLOW_STATUS_MAP`: `FAILED` also
    asks for `TimedOut`, `CANCELED` for `Terminated`; `/stats` folds them back
    the same way.
  - No `ORDER BY`: SQL visibility rejects a custom one, and its default is newest
    `CloseTime` first. `/stats` is one `count_workflows` with
    `GROUP BY ExecutionStatus`, the only `GROUP BY` allowed — a breakdown by kind or name
    would be one count per value.
  - The page token is Temporal's, base64url'd, valid only for the same `status` /
    `since` / `until` / `kind` / `name`. A rejected or malformed token is 400.
- **A job's map is checked twice.** MOVE / DOCK coordinates are a place in one map's
  frame; on another map they are a real place, just the wrong one. The rule is
  `helpers/move_guard.py`: refuse positions planned on another map
  (`map_mismatch`) or while the loaded map's floor plan is rebuilt
  (`conversion_running`); an unknown map is never refused. `POST /api/v1/tasks`
  asks before starting a run, and `execute_move` (and `execute_dock`) asks
  again before every goal with the run's own `TaskMap` (read off
  `workflow.info()`, passed as their optional second argument). An UNDOCK has
  no coordinates and is checked by neither. The second check is the one a schedule meets — it
  never passes the first — and its refusal is a non-retryable step failure, so
  the history row says why. A schedule registered before `TaskMap` drives as it
  always did.
- **Schedules use `SKIP` overlap** — a new run never starts while the previous
  one executes.
- **The trigger is editable in place** (`PATCH /api/v1/schedules/{id}`) via
  `ScheduleHandle.update()`: the callback gets the described schedule and swaps
  only its `spec`; action, policy and state go back as described. The ownership
  gate runs inside the callback (the update already describes — no second RPC)
  and 404s another robot's schedule before anything is sent.
- **The cron string rides in the calendar comment.** Temporal compiles a cron
  into a calendar spec and forgets the string, but copies whatever follows `#`
  into the calendar's `comment`, echoed by describe and list and kept across an
  update. So every cron is registered as `"<cron> # <cron>"` and read back from
  the comment. Three spellings would break that echo and are 400 on **every**
  registering path (both creates and the edit share `_build_schedule_spec`): a
  caller's own `#`, a `CRON_TZ=`/`TZ=` prefix (use `timezone`), and `@every`
  (compiled to an interval, which has no comment; use `interval_seconds`). It
  cannot live in the memo any more: **a schedule memo is immutable** — server
  1.29.7 ignores `UpdateScheduleRequest.memo` — so a copy there would go stale on
  the first edit. Older schedules compiled to a comment-less calendar still carry
  the memo copy; the reader falls back to it only when the spec says nothing, so
  editing one moves it onto the spec for good.
- The memo carries `robot_id` / `map_name` / `task_template_id` /
  `task_template_name`, readable from `list_schedules()` where the start-workflow
  args are not. (`saved_task_id` / `saved_task_name` are still accepted on
  *read* for schedules registered before the rename.)
- **A schedule's steps come from `describe()`, never from `list`.**
  `GET /api/v1/schedules/{id}` decodes them from `ScheduleActionStartWorkflow.args`
  (raw `Payload` protos plus the description's `data_converter`);
  `GET /api/v1/schedules` always answers `steps: []`, because a list element carries
  only the workflow type name and faking it would cost a describe per row. A
  decode failure degrades to `[]` with a warning, never a 502 — like the
  per-step query.
- **A scheduled run's steps are frozen at registration.** The action args hold a
  concrete `WorkflowTask` that nothing re-reads, so later vertex edits reach
  templates and immediate dispatches but *not* a registered schedule.
  `POST /api/v1/task_templates/{id}/schedule` therefore refuses a template whose map is
  not active, or with a `MISSING` vertex — an unattended run does not get the
  snapshot fallback a watching operator is allowed.

## Configuration

| Env var | Default | Used by |
|---|---|---|
| `TEMPORAL_ADDRESS` | `127.0.0.1:7233` | Temporal client + worker |
| `POSTGRES_HOST` | `localhost` | `database/postgres.py` |
| `POSTGRES_PORT` | `5432` | ditto |
| `POSTGRES_USER` | `syncrobotic` | ditto |
| `POSTGRES_PASSWORD` | `syncrobotic` | ditto |
| `SYNCAI_SYSTEM_INI` | `~/robot_ws/config/system.ini` | `helpers/system_config.py` (per-robot INI reads, e.g. `[map]`) |
| `TTS_SERVICE_URL` | `http://syncai_tts:8080` | `gateways/tts` — the syncai_tts container by compose service name; `http://127.0.0.1:8080` with the backend on the host (the compose service uses `http://127.0.0.1:9090`, the port syncai_tts publishes) |
| `LOG_LEVEL` | `info` | `logger.py`. Any stdlib level name; an unknown one logs a warning and keeps `info`. It was unfiltered (`NOTSET`) until 2026-10 |
| `LOG_FORMAT` | `console` | `logger.py`. `json` for one object per line (ISO UTC timestamps, tracebacks as text) for a log collector; `console` is the coloured human one |
| `SYNCAI_WEBRTC_LIB` | `~/robot_ws/lib/libsyncai_worker.so` | `gateways/webrtc` — the Go worker to dlopen. A nonexistent path is the only way to disable WebRTC: a loaded c-shared library cannot be unloaded |
| `VIDEO_RTP_PORT` · `VIDEO_AUDIO_RTP_PORT` · `AUDIO_RTP_PORT` · `STUN_SERVERS` | `5006` · `5007` · `5004` · `stun:stun.l.google.com:19302` | Read by the **worker**, not Python; `gateways/webrtc` only `setdefault`s them right before `InitWorker`, which freezes them for the process. The compose service sets `STUN_SERVERS=""` (LAN robot), `VIDEO_FORMAT=rtpjpeg`, so video comes from the camera feeder's RTP/JPEG on `127.0.0.1:5008` instead of each session opening `/dev/syncai/camera0` itself (which fails "busy" while the feeder holds it), and `VIDEO_FRAMERATE=30`, the feeder's rate with stabilization on, which under `rtpjpeg` only sets the half-second IDR interval. `TURN_SERVERS` / `TURN_USERNAME` / `TURN_PASSWORD` have no default on purpose — `.env` is their only source |

`.env` in the workspace root is loaded via `python-dotenv` at import time.

`[system] robot_id` comes from the system INI (read by the launch file), not the
environment. This package resolves that INI by an **absolute** default
(`~/robot_ws/config/system.ini`) in both `launch/backend.launch.py` and
`helpers/system_config.py`: the old relative `config/system.ini` only worked
because every entrypoint happened to run from the workspace root, and tests and
shells do not. Override with `system_config:=` or `SYNCAI_SYSTEM_INI`.

`set_active_map` (behind `POST /api/v1/maps/{name}/activate`) rewrites that INI
**in place** — it is a single-file bind mount, so a temp file + rename would
leave the container on the old inode — and in-place means truncate, then write.
Before it does, it copies the previous content to **`system.ini.bak`** beside
it (tmp + rename; a sibling file can be atomic). A rewrite that fails its
verifying re-read puts the previous content back itself; a process killed inside
the write (switch_mode kills the byobu session this process is a pane of, and
an OOM kill or a power cut will do) leaves an empty INI, which
`active_map_name` answers from the backup (with a warning) so this process keeps
working — but every launch file reads `[system] robot_id` from the INI itself,
so **restore it from the `.bak` before the next boot**. The backup is this
package's only INI-shaped file; nothing else reads or writes it.

There are **no ROS parameters** — nothing calls `declare_parameter`. Everything
configurable is the INI, the environment, or a commented constant.

One constant spans two containers: `MapCloudSubscriber`'s `allowed_root`,
`/dev/shm`. pgo writes the map cloud under `/dev/shm/syncai_pgo/<robot_id>` and
names the file in its notice; this side only checks the path is under the root.
Neither end reads it from INI or env — it is a convention, and `ipc: host` on
both compose services makes it the same tmpfs (see *ROS interfaces*). Only tests
override it.

Postgres is retried 20× at 5 s on startup and `<robot_id>_db` is created if
absent, so the backend may come up before the `postgres` container is ready.
After startup the pool runs with `pool_pre_ping` (a `SELECT 1` before each
checkout, so a postgres restart — its own compose stack — costs a reconnect, not
one 500 per pooled connection) and a 5 s libpq `connect_timeout`.

CORS is wide open (`allow_origins=["*"]`).

## Build and run

Builds run **inside the robot container** — see the workspace `CLAUDE.md`.

```bash
colcon build --packages-select syncai_backend --symlink-install
source install/setup.bash
```

Python deps are **not** rosdep-managed (jammy has no reliable key for fastapi);
`requirements.txt` is the single source of truth, used by every Docker stage:

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
window (pane 2, behind `robot_state`, `sleep: 2`). It is in the mapping spec
because the operator console *is* the mapping UI — mode switching, teleop, the
live cloud and save-map all go through it. It still hard-requires postgres, which
lives in the infra compose stack and is up regardless of session.

> `setup.py` installs **compiled bytecode only** (`InstallNoSource`): after a
> normal install it byte-compiles the package and deletes the `.py` sources from
> the install space. The step self-disables for symlinked modules, so
> `--symlink-install` builds are unaffected.

### As its own container

`Dockerfile` has four stages — `base` (ROS + pip deps, the expensive one),
`builder` (the colcon install space), `runtime` (the service) and `dev` (the test
image). `docker-compose.yml` runs `runtime` as one service. There is no default
target; name it.

#### Step by step

**1. Before the first start.** postgres and temporal (the workspace's infra
stack), syncai_tts (SyncAI-TTS) and the ROS side (the robot container) must
already be up; compose starts none of them.

```bash
# The infra, if it is not up.
docker compose -f ~/SyncAI-Robot-Workspace/docker-compose.yml up -d postgres temporal

# The WebRTC worker, built in SyncAI-WebRTC-Worker and copied in by hand.
# Optional: without it the camera stream 502s and everything else works.
cp /path/to/libsyncai_worker.so lib/

# Stop the byobu-pane backend inside the robot container first (see
# "One backend at a time" below) — two backends on one robot is an outage.
```

**2. Build the image.** The builder stage clones syncai_common from
`interface.repos` (HTTPS, no credentials); nothing needs copying in. `base` is
slow (~20 min on aarch64 the first time) and only rebuilds on a
`requirements.txt` change (or a different `PIP_INDEX_URL`).

Nearly all of that time is pip **downloading**, not building — every wheel is
prebuilt for aarch64. From some networks `files.pythonhosted.org` crawls at
40–70 kB/s (a 30 MB scipy wheel took 10 min), so a build that looks stuck in
`[base 4/4]` usually is just slow. Point it at a mirror instead:

```bash
PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple docker compose build
```

pip's cache is a BuildKit cache mount, so a cancelled build does not throw away
the wheels it already fetched; the next run picks up from there.

```bash
docker compose build                                    # tags syncai-backend, USER_UID=${UID:-1000}
# or, without compose — name the target, there is no default:
docker build --target runtime --build-arg USER_UID=$(id -u) -t syncai-backend .
```

`USER_UID` must own the host's `config/`, `map/` and `record/`, or the container
user cannot write them.

**3. Run it.** Compose is the supported way — host networking, `ipc: host`, the
nvidia runtime, camera devices, mounts and environment, each commented in
`docker-compose.yml`.

```bash
docker compose up -d                # add --build to rebuild first
docker compose logs -f              # rclpy + uvicorn + Temporal worker, one stream
```

Optional knobs, in the shell or a `.env` beside the compose file:

| variable | default | what it moves |
|---|---|---|
| `ROBOT_WS` | `~/SyncAI-Robot-Workspace` | where `config/` and `map/` come from |
| `ROBOT_INSTANCE` | `robot01` | which `config/instances/<name>.ini` is mounted as `system.ini` |
| `RECORD_DIR` | `./record` | where bags land on the host |
| `WEBRTC_LIB` | `./lib/libsyncai_worker.so` | the WebRTC worker `.so` |
| `POSTGRES_USER` / `POSTGRES_PASSWORD` | `postgres` / `postgres` | the infra stack's credentials |
| `UID` / `GID` / `VIDEO_GID` | `1000` / `1000` / `44` | container user and the host's `video` group (`getent group video`) |
| `PIP_INDEX_URL` | `https://pypi.org/simple` | build only: the PyPI index the base stage installs from (a mirror when PyPI is slow) |

```bash
ROBOT_INSTANCE=robot02 RECORD_DIR=/mnt/ssd/record docker compose up -d
```

**4. Check it.** `/health` is always 200; the body says `ok` or `degraded`
(e.g. Temporal down). The healthcheck turns healthy within 120 s (postgres is
retried 20 × 5 s).

```bash
docker compose ps                   # STATUS: healthy
curl http://127.0.0.1:3000/health
docker exec syncai_backend /usr/local/bin/entrypoint.sh ros2 node list   # /<robot_id>/... present
```

A node named `default_robot` means the per-robot INI was not mounted; an empty
`ros2 topic list` in here means check `ROS_DOMAIN_ID` / the RMW (below and in the
compose file).

**5. Stop / restart.**

```bash
docker compose restart              # e.g. after changing .env or the WebRTC .so
docker compose down                 # stop and remove the container
```

**Without compose.** A plain `docker run` must reproduce the compose service;
this equivalent drifts the moment the compose file changes, so prefer compose:

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
    -e STUN_SERVERS= -e VIDEO_FORMAT=rtpjpeg -e VIDEO_FRAMERATE=30 \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
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

Drop `--device` lines for unplugged cameras (`docker run` refuses a missing
device), and `--runtime nvidia` off a Jetson.

The per-robot `config/instances/robotNN.ini` is mounted **over**
`config/system.ini` as a single file (the checked-out `system.ini` is an empty
placeholder), as the workspace's compose does. Miss it and there is no `[system]
robot_id`: namespace, database and task queue all become `default_robot`.

The compose file comments every piece of the service; the ones that bite:

- **`network_mode: host`.** DDS discovery with the nav stack runs over `lo` with
  multicast off (`config/cyclonedds.xml`), so both sides share the host's network
  namespace. Compose service names therefore do not resolve — postgres, temporal
  and syncai_tts are `127.0.0.1:<published port>` — and the API lands on the
  host's `:3000` with no `ports:` mapping.
- **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`**, installed in the image. On the
  default fastrtps the backend starts cleanly, logs nothing alarming and sees not
  one topic.
- **Three bind mounts from `ROBOT_WS`** carry what this package shares with the
  stack: `config/` (read-write — activating a map rewrites `system.ini` in place;
  `instances/robotNN.ini` goes over `config/system.ini` as one file) and `map/`
  (must be the directory the nav stack's `map_server` reads). **`record/` and
  `lib/libsyncai_worker.so` are not among them**: nothing else reads a bag or
  dlopens the worker, so both default beside the compose file — moved with
  `RECORD_DIR` and `WEBRTC_LIB`. Both are tracked as empty directories
  (`.gitkeep`; the `.so` is gitignored) so Docker never creates a bind source
  itself: one it creates is `root:root`, unwritable for the uid-1000 user, and a
  missing *single-file* source becomes a directory that dlopen then fails on.
- **`ipc: host`.** pgo hands over the map cloud as a PCD under the host's
  `/dev/shm/syncai_pgo/<robot_id>` (see *ROS interfaces*); the path only names
  the same file if both containers share the host's IPC namespace (the
  workspace's robot service sets it too). Not a bind mount of a `/dev/shm`
  subdirectory, for the `root:root` reason above.
- **One backend at a time.** `NodeManager` starts this process as a byobu pane
  in the robot container. Running the service as well gives two processes on
  `:3000` and, less visibly, two Temporal workers polling the same
  `<robot_id>.ROBOT_TASK_QUEUE`. Take it out of the session spec first.

Knock-on for `switch_mode`: as a byobu pane the process dies with the session and
`NodeManager` restarts it — what the route's "success looks like a dropped
connection" rests on. In its own container it survives the teardown, and
`restart: unless-stopped` only covers a crash, so a mode switch no longer
recycles it.

## Tests

```bash
colcon test --packages-select syncai_backend
colcon test-result --verbose
# or, inside the container, from src/syncai_backend/:
pytest test/
```

**Off the robot**, the `Dockerfile`'s `dev` target (
`docker build --target dev -t syncai-backend-dev .`) supplies ROS 2 Humble and the pip dependencies, with the
source bind-mounted, so edits need no rebuild. Its header comment has the exact
commands. Before reading a result from it:

- **Get the interface package in, or a third of the suite does not run.**
  `vcs import < interface.repos` materialises `syncai_common`; build it with
  `--packages-select syncai_common`. Measured 2026-09-23 (when the map srvs still
  came from FAST-LIO2's `interface`, now folded into `syncai_common`):

  | in the image | result |
  |---|---|
  | present | 704 passed, 1 skipped (`test_copyright`, skipped on purpose) |
  | absent | 315 passed, 6 skipped, **11 collection errors** — those files import `syncai_common` plainly rather than via `importorskip` |

  Anything touching a router or gateway needs it. (The `_INTERFACE_SRVS` guard
  and `test_map_gateway_no_interface.py` are gone: the map srvs are a hard
  import, and an image without them fails in the builder.)
- **The image runs as non-root on purpose.** Root bypasses permission checks, so
  `os.access(W_OK)` is `True` on a read-only file and the map router's
  `ini_not_writable` test fails against working code.

`test/` holds ~40 files, roughly one per router / gateway / subscriber / repo /
helper (`ls src/syncai_backend/test/` is the index), and needs `rclpy` /
`nav_msgs` / `syncai_common` / OpenCV importable. The database layer runs
against **in-memory SQLite** (`StaticPool`, one shared connection), so no
PostgreSQL. The conversion tests also need scipy (`test_pcd_to_gridmap.py` →
`scipy.ndimage`) and open3d (`test_traversable.py` `importorskip`s
`helpers.traversable` per test, so it skips without open3d). The standard ament
linters (`test_copyright`, `test_flake8`, `test_pep257`) run alongside.

## Gotchas

- **Configuration is read once, at startup.** The INI (`[map]`, `robot_id`),
  `.env` and the environment are all read during construction — any change needs
  a backend restart.
- Relative topic names are not optional — see the namespace section.
- **Exactly two latched subscriptions, both pgo's: `pgo/map_cloud_file` and
  `pgo/mapping_status`.** (`map` and `localizer/map_cloud` went away with the
  file-based map endpoints.) Durability must match pgo's publisher exactly — a
  VOLATILE reader connects and gets nothing replayed. Both exist only under
  `MANUAL`; a silent `pointcloud/map/stream` and `unknown` run state on a
  navigating robot are expected, not a QoS mismatch.
- **Mapping mode comes up idle.** Since 2026-10 pgo banks nothing until
  `POST /api/v1/mapping/start`, and a successful save returns it to idle. A save or
  reset then is a 409 `mapping_idle` from the latched status, not pgo's
  `NO POSES!` as a 502 — and an older console with no Start control cannot map
  against this stack. The status ages out after 5 s (`MAPPING_STATUS_TTL_S`):
  this container outlives the mapping session, and a latched `mapping` from a
  pgo that is gone must not refuse a Start forever.
- **The map cloud is a file in the host's `/dev/shm`; both containers need
  `ipc: host`.** Without it, notices still arrive, every read logs
  `map cloud file unreadable`, and the preview never updates. Do not "fix" the size cliff by
  raising `net.core.rmem_max` and going back to the `PointCloud2` — host state on
  every robot, and 16–45 MB per merge only moves the cliff.
- `map -> pointlio_odom` exists once `syncai_localizer` has its first synced
  `body_cloud` + `lio_odom` pair — **not** once it has converged: it broadcasts
  from that sample on, starting at identity, relocalized or not. A missing
  transform means the localizer (or pointlio under it) is not running; a present
  one proves nothing about the pose — `relocalize_check` is the convergence
  check. Until it exists the live cloud stream is silent; the subscriber logs
  once on the first drop and once on recovery, so check the log if the 3D view
  is empty.
- **Deleting `keepout.*` clears nothing.** The running `KeepoutFilter` keeps the
  last mask, and the nav session writes a blank pair at the next boot anyway.
  Clear with `PUT .../keepout` and `[]` — an all-unknown (205, not free 254) mask,
  reloaded; a free cell would make unexplored space plannable.
- **A 2D-only editor that drops `name` / `z_min` / `z_max` widens 3D zones.**
  `PUT …/keepout` replaces the whole list, and a zone sent without a band
  covers every floor. Send back what `GET` returned.
- **A map switch does not move the keepout mask along.** `filter_mask_server`
  serves what it was last told; `activate` reloads the new map's mask itself,
  writing a blank one first for a never-booted map. A "not available"
  `filter_mask_server/load_map` means MANUAL mode or a dead keepout pane — since
  workspace `c8d2558` the launch starts it on every map, blank mask included.
- The `sqlalchemy` session convention is per-repo: `init_map_repo` creates the
  schema and builds its own `sessionmaker` from the injected engine.
