# Architecture

> Part of the `syncai_backend` reference — see the [documentation index](../README.md#documentation).

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
interfaces/rest/routers/   HTTP + WS surface; pydantic schemas; no business logic. The map
        │                  routes are routers/maps/, one module per area, composed by
        │                  routers/map.py's init_map_router
        │
services/                  domain work that outlives a request: mode_restart (the latest
        │                  restart_mode dispatch and how it ended); safety_lock (the driver
        │                  safety lock's rising edge → cancel every task); map_switch (the
        │                  live map switch's steps and their undos); map_usage (whether a
        │                  map may be edited, destroyed, switched to or driven on); map_import (an
        │                  archive's directory and vertex rows, published together);
        │                  task_record_retention (task_record's daily partitions: made
        │                  ahead, dropped after 30 days)
        │
gateways/                  outbound: ROS (robot, map, landmark), Temporal (workflow), speech (tts: HTTP
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
                           QoS from qos.py (latched / best_effort), per ros-interfaces.md
temporal/                  worker, RobotWorkflow, activities
helpers/                   pointcloud (read a binary PCD, downsample / transform / pack),
                           system_config (INI reader), map_archive (checksummed zip / tar.gz
                           of a map dir), move_guard (a run's TaskMap against the loaded
                           map), disk (directory size + newest mtime), single_flight
                           (concurrent callers share one slow read); and, called by nothing
                           since the 2D removal, kept as a library: occupancy_grid
                           (OccupancyGrid→PNG), pgm (whose write_text_atomic the catalogue
                           still uses), pcd_to_gridmap (z-band recipe), keepout (2D mask
                           rasteriser), traversable (traversability recipe)
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

**3D maps only (2026-10-28).** The stack navigates on `map/<name>/octomap.bt`
alone (SyncAI-Robot-Workspace removed map_server, the 2D planners, the 2D RPP
and the keepout mask), and so does this backend: there is no gridmap
conversion, no gridmap route (`grid/convert`, `PUT …/grid`, `…/image`,
`…/thumbnail`), no `grid` / `grid_status` in the catalogue, no `map_server` or
`filter_mask_server` client. A map is navigable when it has an `octomap.bt`;
`activate` refuses one without (409 `octomap_missing`), and every save asks pgo
for the tree (`SaveMaps.OCTOMAP_ON`). A map saved before still carries its
`gridmap.*`, `gridmap.recipe.json`, `keepout.pgm` / `.yaml` and maybe
`traversable_debug/`: plain files now, counted in its size and carried by its
export (bar `traversable_debug/`), read by nothing. `helpers/pcd_to_gridmap.py`,
`traversable.py`, `keepout.py`, `pgm.py` and `occupancy_grid.py` stay as a
library with their tests; no route calls them.

**Forbidden zones are `keepout.json`.** Polygons in map-frame metres, drawn by
the console (`PUT /api/v1/maps/{name}/keepout`), which the robot's 3D planner
(`Octo3D`) reads beside `octomap.bt` itself. Each zone may carry `name` and a
`z_min` / `z_max` band — the ground heights (map-frame metres) of the floor it
forbids, `floor_z ± 0.5`; without a band a zone covers every floor. Unset keys
are left out of the file. A save on the **active** map reloads the planner's
zones (`planner_server/Octo3D/load_keepout`, `MapGateway.reload_keepout_3d`,
`map_url` = the json); a switch needs no keepout call, since Octo3D reads the
new map's json as part of `load_map`. **A client must send back the `name` /
`z_min` / `z_max` it read**, since a save replaces the whole list — dropping
them widens a zone to every storey. Nothing in the file names the map, so
rename and delete carry it along. Until 2026-10-28 the same save also
rasterised a `keepout.pgm` + `keepout.yaml` mask for the 2D costmap's
`KeepoutFilter`; that pair is gone with the filter.

**The 3D map is served as the robot wrote it.** Every save asks for it, so
the same `clean_map` that rewrites `map.pcd` also writes `map/<name>/octomap.bt`
— the `octomap::OcTree` (0.1 m, free *and* occupied) the 3D planner plans on —
and says so in `map_clean.recipe.json` (`params.octomap`,
`measurements.octomap_written` / `octomap_error`). The catalogue reads that
sidecar and the tree's text header only, and reports `octomap_status` (`none` /
`converting` / `ok` / `failed` / `interrupted`,
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
are one, and so are two levels never walked one over the other (a long slope, a
ramp down to a bay: nothing to hide, so nothing to split); each floor also
carries `low` / `high`, the body heights walked on it, so a floor that is not
flat is drawn whole — and `PUT` saves an operator's list (names, body heights) to
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
`config/instances/robot.ini`), overridable with `system_config:=` — and uses it
as the **node namespace**. The node reads it back:

```python
robot_id = self.get_namespace().strip("/") or "default_robot"
```

| Scoped by robot_id | Value |
|---|---|
| ROS topics/services/actions | relative names inherit the `/<robot_id>` namespace |
| PostgreSQL database | `<robot_id>_db` (auto-created on first connect) |
| Temporal task queues | `<robot_id>.ROBOT_TASK_QUEUE` (workflow, body activities), `<robot_id>.ROBOT_IO_QUEUE` (`SPEAK`) |

**All ROS names here are relative** (`map`, `robot_state`, `navigate_to_goal`,
`pointlio/body_cloud`). Never hardcode `/<robot_id>/…` — that bug has been fixed
here once already.

TF frame names are *not* namespaced, so the cloud subscriber takes the source
frame from the message header and only pins the target frame (`map`).
