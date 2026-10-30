# REST API

> Part of the `syncai_backend` reference — see the [documentation index](../README.md#documentation).

Interactive docs: `http://<robot>:3000/docs`.

| Method | Path | Notes |
|---|---|---|
| GET | `/health` | Liveness probe (always 200); `status` is `degraded` while `task_server` is not `running` (`connecting`/`dead` + `task_server_error`) |
| POST | `/api/v1/tasks` | Start a `RobotWorkflow`; body `{id, steps[], kind?, name?, map_name?}` (legacy `timestamp` ignored). `kind` (`goal` / `standup` / `liedown` / `task`; `schedule` is the backend's and refused) and `name` are stamped as search attributes for the history. A job with a MOVE / PATROL is checked against the loaded map — 409 `map_mismatch`, see [*A job's map is checked twice*](tasks.md); stand / lie down / SPEAK / WAIT never are. 409 `task_running` while any task runs — one task at a time. **Or** `{id, continue_from, kind?, name?}` instead of `steps` (exactly one; 400 otherwise, and with `map_name`): a new run of a FAILED task from the failed step on — see [*Continuing a failed task*](tasks.md). The answer then carries `continued_from` and `from_step_id` (both `null` otherwise) |
| GET | `/api/v1/tasks/{id}` | Overall status + per-step state (workflow query): `id`, `status`, `error_msg`, and `message` — for a running `MOVE`, the robot navigator's live status sentence (`NavigateToGoal` feedback `status_msg`, read from `RobotGateway`), for a running `PATROL` one built from its feedback (`Patrolling: 3 of 5 waypoints left, 12.4 m to go`), empty otherwise — `charge` (a `CHARGE` step's charger waypoint as dispatched, `{x, y, theta}`: the marker's centre, `theta` out of its face, from which a console draws the 1.3 m staging pose; `null` for every other step) — plus `started_at` / `ended_at` (UTC, `null` until then and for runs older than the fields), `paused_seconds` (a hold while the step was under way; the step's own time is `ended_at − started_at − paused_seconds`) and `waypoints_passed` (a PATROL's progress, 0 for every other step). A step `SKIPPED` was got past by the failed run this one continues. A held run answers `status: PAUSED`, its held step `PAUSED` — the only place a hold is visible |
| DELETE | `/api/v1/tasks/{id}` | Request cancellation; answers `CANCELING` — the final state (possibly still `COMPLETED`) comes from GET |
| POST | `/api/v1/tasks/{id}/pause` · `/resume` | Hold / release the running task — *requests* (workflow signals): pause answers `PAUSING`, resume `IN_PROGRESS`; the hold is read back from GET. A `MOVE` / `PATROL` is interrupted at once and re-sent on resume (a PATROL past the waypoints it had passed), a `WAIT` freezes its countdown, a `SPEAK` or posture step finishes first — see [*Task orchestration*](tasks.md). 409 `task_not_running` once closed; 404 for another robot's id |
| GET | `/api/v1/active_tasks` | What runs on this robot's queue now, *whoever* started it, with `source` / `schedule_id` / `kind` / `name` / `map_name` (the run's `TaskMap`; for a run older than the attribute, the loaded map unless it is a stand / lie down; `null` for a run that drives nowhere). A held run is still listed as `IN_PROGRESS`. Beside `tasks`, `recently_ended`: the runs that closed in the last `RECENTLY_ENDED_WINDOW_S` (10 s), newest first, with `ended_at` and the map as stamped, from the same visibility query (an `OR CloseTime >=` in the filter, no second RPC). A run that starts and fails between two polls is never in `tasks`, so this is how a console that did not start it learns how it ended; nothing in it is running, holds a map or makes `_require_idle` refuse. Not `/tasks/active`, which would shadow `/tasks/{id}` |
| GET | `/api/v1/task_history` | **Finished** runs, newest close first, one visibility page per call. Query: `page_size` (1–100, default 20), `page_token` (previous `next_page_token`), `status` (`COMPLETED` / `FAILED` / `CANCELED`), `since` / `until` (close time; naive is UTC; `until` before `since` is 400), `kind` (`goal` / `standup` / `liedown` / `task` / `schedule`), `name` (exact template name). `kind` / `name` / `map_name` are `null` for runs dispatched without them or predating the attribute — no fallback. Bounded by namespace retention |
| GET | `/api/v1/task_history/stats` | The same runs counted: `total`, `by_status`, `success_rate` (`COMPLETED` / `total`, `null` when nothing finished). Same filters as `task_history`; one count RPC |
| GET | `/api/v1/task_history/export` | The report: every finished run in the filter as CSV (UTF-8 with BOM, for Excel), **one row per step**, oldest close first. Same filters as `task_history` (no paging) plus `tz` (IANA, e.g. `Asia/Taipei`; times are written in it, UTC when omitted; 400 for an unknown one). Read from PostgreSQL's `task_record`, not Temporal, so it reaches back 30 days — see *The run record* |
| POST | `/api/v1/route_preview` | The route the robot would drive from where it stands to a goal, planned as a MOVE's would be but never sent ([*Route preview*](ros-interfaces.md)). Body: a MOVE's params (`x`, `y`, `theta` in degrees, `z` the floor's ground) plus `map_name?`. 200 `{reachable, points, length_m, code, message}` — `points` the route's `[x, y, z]` (thinned like the live route), `length_m` along it in 3D; with `reachable: false` when there is no route, `code` (`goal_in_keepout`, `no_valid_path`, `map_not_loaded`, …) and `message` the sentence the job would have failed with. 409 `map_mismatch` (as `POST /api/v1/tasks`), 409 `preview_superseded` when a newer preview replaced it, 502 when there is no planner to ask (not in Navigation). Not refused while a job runs: it moves nothing. Not `/tasks/preview`, for the reason `active_tasks` gives |
| GET | `/api/v1/task_steps` | The steps an operator builds a job from, for a client to draw its editor with: each step's `type`, operator `label` and `hint`, `occupies_body` (`BODY_STEP_TYPES`: a group takes at most one), `while_walking` (offered on the walk to a stop: SPEAK alone — a WAIT in a MOVE's group only stretches the walk) and `params`, a list of fields of five kinds — `waypoint` (a pose on the job's map, spread into `x` / `y` / `theta` / `z` and the template's `vertex_id` on the wire; an optional `filter: "charger"` offers only charging waypoints — CHARGE's), `waypoints` (PATROL's ordered list of such poses, `min_items: 2`, sent as `params.waypoints` and the template's index-aligned `vertex_ids`), `text`, `integer`, `number` (plus `choice` for DEMO, demo branch only) — whose limits are read off `SpeakParams` / `WaitParams` / `PatrolParams`, so they cannot drift from the validators. Every `StepType` is listed (DOCK / UNDOCK were removed on 2026-11-12). No gateway behind it (`tasks/catalogue.py`). Not `/tasks/steps`, for the reason `active_tasks` gives |
| POST | `/api/v1/schedules` | Create a schedule (cron **or** interval). A cron with its own `#`, a `CRON_TZ=`/`TZ=` prefix (use `timezone`) or `@every` (use `interval_seconds`) is 400 — see [*Task orchestration*](tasks.md) |
| GET | `/api/v1/schedules` | List with next run times. This robot's only, filtered by the server on the schedule's `RobotId` (see [*Provenance*](tasks.md) under task history) |
| GET | `/api/v1/schedules/{id}` | Describe one. `steps` are those it was registered with: with a `task_template_id` only the fallback, since each run reads the template as it fires. `task_template_name` is the template's name now (the memo's once it is gone), here and in the list |
| PATCH | `/api/v1/schedules/{id}` | Change when it fires: `{trigger: {cron, timezone}}` or `{trigger: {interval_seconds}}`, replacing the old rule whole. In place — id, registered steps, provenance, paused state and `SKIP` stay. No renaming (delete + create). Same cron restrictions |
| DELETE | `/api/v1/schedules/{id}` | Delete |
| POST | `/api/v1/schedules/{id}/pause` · `/resume` | Pause / unpause |
| POST | `/api/v1/task_templates` | Store a re-dispatchable step list |
| GET | `/api/v1/task_templates` | List; `?map_name=` gives that map's **plus** the map-independent ones |
| GET | `/api/v1/task_templates/{id}` | One, with vertex references resolved |
| PUT | `/api/v1/task_templates/{id}` | Partial update; `steps` replaces the whole list |
| DELETE | `/api/v1/task_templates/{id}` | Delete, with every schedule that runs it — those first, so a 502 from Temporal deletes nothing |
| POST | `/api/v1/task_templates/{id}/schedule` | Register a schedule that runs this template as it is each time it fires (an edit reaches the next run), through the same path (and cron restrictions) as `POST /api/v1/schedules`. Refused (400) for another map than the loaded one or a `MISSING` vertex |
| GET | `/api/v1/robot/state` | Latest state: pose in degrees, wifi, battery, byobu-session `mode`, `low_level_mode` (the gait controller's report plus the driver's `safety_locked`), and `localization_valid` — `false` means a zeroed placeholder pose (not relocalized, or mapping). 404 only until `robot_state` first publishes |
| POST | `/api/v1/robot/mode` | `switch_mode` on `syncai_sys_manager`. A real switch kills the byobu session **this backend is a pane of**, so the client usually sees a dropped connection — treat it as success-in-progress. A body reliably arrives only for the no-op (already in that mode) and a refusal |
| POST | `/api/v1/robot/restart` | `restart_mode` on `syncai_sys_manager`: rebuild the byobu session of the mode already live (what `/robot/mode` refuses as a no-op). No body. Answers within the 2 s ack window with `restarting: true`; sys_manager replies only when the rebuild ends, kept for the `GET` below. **409 `restart_refused`** with sys_manager's sentence, nothing touched — in MAINTENANCE, with both sessions up, and always in MANUAL (`pgo_node` may hold an unsaved map), so in practice AUTO only. **409 `restart_running`** while one is under way; 502 if sys_manager is unreachable |
| GET | `/api/v1/robot/restart` | How the latest restart this process dispatched went: `status` `idle` / `restarting` / `succeeded` / `failed`, `message`, `started_at` / `finished_at`. This, not `/robot/state` (which keeps serving the last frame through the rebuild), is how a console learns the outcome. Unanswered for 3 min reads `failed`; in memory, so a restarted backend says `idle` |
| WS | `/api/v1/robot/teleop` | Inbound manual control: `{vx, vy, wz}` JSON frames at ~10 Hz, each axis clamped to [-1, 1] and published as-is (m/s / rad/s). Refused (`{"error": ...}` frame, socket stays open) while an autonomous MOVE runs. A 0.5 s stale-input watchdog and disconnect both publish zero velocity — driver_manager has no cmd_vel watchdog |
| POST | `/api/v1/localization/auto` | Find the robot on the loaded map with **no hint** (`reset_odometry` with `skip_seed`: reset Point-LIO, then the global search — kidnap recovery) — the alternative to `set_initial_pose`. The robot must be standing still. No body; blocks up to ~30 s. 200 = localized, nothing to poll: `x` / `y` / `z`, `theta` (degrees), `fitness`, `inlier_ratio`, `candidates`. 409 `task_running` (any job — it moves the map frame), `tasks_unknown`, `localize_running` (one search at a time). Every localizer answer — not AUTO, no `poses.txt`, ambiguous, did not converge — is 502 with its sentence; the odometry has been reset by then, the map pose is left where it was. Writes nothing, not even `[initial_pose]` |
| POST | `/api/v1/landmarks/detect` | Where the robot's camera sees a known landmark — the charger's marker — on the **loaded** map (`detect_landmark` on `landmark_detector`, asked for the pose in `map`), for the console's *Detect* on a `CHARGE` vertex. Not under `/maps/{name}`: it is asked of the robot, not of a stored map, so the answer names the map. Optional body `{landmark_id}` (default `charger_a`, the detector's registry name). Always answers, within ~9 s at worst (2 s for the service to appear, the 6 s poll, 1 s for its last call): the first call wakes the detector, then it is polled until a sighting is 3 frames in a row with no mirror ambiguity left. 200: `map`, `x` / `y` (the landmark's centre), `theta` (degrees, out of its face — towards a robot in front of it; the 6-DOF pose's +x projected onto the floor), `z` — **the robot's own `z`**, the ground it stands on and so the storey the landmark is on, what a vertex stores — and `marker_z` (the landmark's height), `consecutive`, `age`, `reprojection_error_px`. 409 `no_map_loaded`, `not_localized`, `detect_running` (one at a time), `landmark_not_detected` (not seen, or not steadily, within the budget — face the robot at it and ask again; the detector's sentence in `detail`). 400 `unknown_landmark`. 502 when the detector is not running, has no camera frames, or cannot transform into `map`. Reads only: nothing is saved until the console creates the vertex |
| POST | `/api/v1/robot/set_initial_pose` | Seed localization with a map-frame pose (degrees in, radians out); fire-and-forget. Optional `z`, the ground the pose stands on (a vertex's `z`), picks the storey: it goes out as `position.z` with the z variance (covariance cell 14) set, which is what tells `syncai_localizer` to use it. Omitted, the localizer keeps the height of its current estimate (RViz's 2D Pose Estimate does the same), so a pose on another storey needs `z` |
| POST | `/api/v1/robot/set_motion_key` | Gait key `"0"`–`"5"`; `"4"` (ESTOP) is accepted but **not** forwarded — 200 with `sent: false` |
| POST | `/api/v1/robot/estop` | `{"locked": bool}` → `set_safety_lock` on `syncai_driver_manager`. `true` engages the lock and, on the `false → true` edge only, starts cancelling every running task and nav goal in the background (`cancel_requested: true`; started, not finished). `false` releases it, cancelling nothing. 502 if the driver is unreachable, nothing cancelled. Not the ESTOP motion key |
| POST | `/api/v1/robot/set_policy_mode` | Gait-controller policy index; only `0` (PPO) and `1` (HIMLOCO) |
| GET | `/api/v1/network/wifi/scan` | Scan networks (blocks up to 45 s). A scan requested while one runs joins it and gets its answer, instead of holding a second thread and queueing a second scan in driver_manager |
| POST | `/api/v1/network/wifi/connect` | Connect via `nmcli` (blocks up to 70 s). Connect, activate and delete of a saved network change NetworkManager's state, so one runs at a time: the others are 409 `wifi_busy` |
| GET | `/api/v1/settings/controller` · `/planner` | A settings page (controller_server's, planner_server's): `groups` and `fields`, each field with its operator `label` / `description`, `kind`, `unit`, `min` / `max` / `step`, `apply` (`live` / `idle` / `restart`), and its `value` (the one in force; a `restart` field's the next start's), `current` (the running node's, `null` while the nav session is down), `default` (the factory params file's), `modified` and `pending_restart`. `live_available: false` outside AUTO — values are then the saved ones. See *Settings pages* |
| PUT | `/api/v1/settings/controller` · `/planner` | Body `{values: {<field id>: value}}`, only the fields being changed. Set live, read back, then saved to the robot's runtime overlay; answers the new state. 400 `invalid_setting` (range, type, or a rule tying it to another parameter — one sentence), 400 `parameter_refused` (the node's own refusal), 409 `controller_busy` (an `idle` field while the robot navigates — nothing changed), 409 `settings_busy`, `params_not_writable`, `params_unreadable`; 502 when the node does not take the value. With the nav session down a change is only saved |
| DELETE | `/api/v1/settings/controller` · `/planner` | Back to the factory values: `?id=<field>` (repeatable) or every field. The keys leave the overlay and the live fields are set to the factory file's values. 409 `defaults_unavailable` without the factory file mounted |
| GET | `/api/v1/maps` | Map directories on disk with vertex counts and the 3D map's state — **the build-status surface**; there is no job resource. `octomap_status` `none` / `converting` (the robot's map cleaning is building it) / `ok` / `failed` (reason in `octomap_error`) / `interrupted` (the cleaning never finished), read off `octomap.bt` and the cleaning sidecar; `octomap_version` (changes when the file does) and `octomap_resolution`. No `grid` / `thumbnail` / `grid_status` since the 2D removal (2026-10-28) |
| GET | `/api/v1/maps/{name}` | One map's summary |
| POST | `/api/v1/maps` | Save the current mapping run **and end it**: `pgo/save_maps` into a new map dir, always with `octomap: OCTOMAP_ON` (a map is navigable only with its `octomap.bt`); the tree lands minutes later, after the robot's map cleaning — watch `octomap_status`. Body `{name}`; the 2D-era `grid` / `octomap` keys are ignored. On success pgo goes idle and clears the "map so far" layer and `/dev/shm` itself, so the next run waits for `POST /api/v1/mapping/start`. 409 `mapping_idle` when nothing was started (before any directory is created), 409 `mapping_busy` mid start/reset. Mapping mode only in practice |
| POST | `/api/v1/mapping/start` | Begin a run: `pgo/start_mapping`, idle → mapping (the drive to the start point is not part of any map). No body; always resets the LIO front end (map origin = odometry origin), so the robot must be **stationary** until it returns. 409 `mapping_running` / `mapping_busy`; 502 with pgo's sentence. Touches no file |
| GET | `/api/v1/mapping` | pgo's last-reported run state: `state` `idle` / `mapping` / `resetting` / `unknown`, `key_poses`, `loop_closures`. `unknown` is a real answer — every navigating robot, or pgo silent for 5 s. Latched, so it survives a console reload |
| POST | `/api/v1/mapping/reset` | Discard the in-memory run and start a new map, restarting nothing: `pgo/reset_mapping` pauses intake, resets the LIO front end, rebuilds the pose graph; pgo stays mapping. No body and **no save** (saving stays a separate act — the usual case is abandoning a run gone wrong). Touches no file, hence `/mapping/`. 409 `mapping_idle` / `mapping_busy`; 502 with pgo's sentence, graph untouched. Robot must be **stationary** (static, gravity-aligning re-init) |
| PATCH | `/api/v1/maps/{name}` | Rename: moves `map/<old>/` to `map/<new>/`, re-keys `map_vertices` and `task_templates` rows. 409 `map_active` (the stack opened those files at launch — switch maps first), `octomap_converting` (the robot's map cleaning is still writing map.pcd / octomap.bt into it, for minutes after a save; a sidecar `converting` for over 30 min is taken as a dead clean and does not hold it), `name_taken`. Schedule memos keep the old `map_name` label (display-only) |
| DELETE | `/api/v1/maps/{name}` | `rmtree` of `map/<name>/` plus its `map_vertices` rows, no undo. The rename's first two refusals plus 409 `template_bound` while a template names the map (listed). Rows go **before** the directory — the irreversible step goes last. Schedules registered with their own steps keep firing them |
| GET | `/api/v1/maps/{name}/export?format=zip\|tar.gz` | The map dir as one archive (default `zip`; `Content-Disposition: attachment; filename="<name>.<ext>"`), built into an anonymous temp file on the maps' disk and streamed with its `Content-Length`, never held in memory: files relative to the root — no top-level folder, no `traversable_debug/` (a 2D-era map's tuning dumps) — plus `syncai_map.json` `{format, version: 1, name, exported_at, files: {relpath: md5}, vertices: [...]}`. Vertex **ids are not exported** (regenerated on import); templates are not included (their steps bind vertex ids). 409 `octomap_converting` while the robot's map cleaning is still writing into it. The active map may be exported |
| POST | `/api/v1/maps/import?name=` | Create or **replace** a map from such an archive; raw `application/octet-stream`, zip / tar.gz by magic bytes. **Nothing is written until the archive is judged**: manifest required (version 1; newer refused), every file md5-verified; an unlisted or missing member, a link/device member, an absolute or `..` path, no `map.pcd`, or an unknown vertex type is 400. Name is `?name=` else the manifest's. Replacing takes the delete's refusals (`map_active` / `octomap_converting` / `template_bound`) plus 409 `disk_low`. The body is streamed to an anonymous temp file on the maps' disk as it arrives (1 MB held at most); **413 `archive_too_large`** past 4 GiB (`MAP_IMPORT_MAX_BYTES`), on `Content-Length` before reading and on the bytes received while reading. 201 `{name, replaced, files, bytes, vertices_created, vertices_deleted, message}`. Starts no conversion; an imported `gridmap.recipe.json` is kept as-is. Staging: see [*Layering*](architecture.md) |
| POST | `/api/v1/maps/{name}/activate` | Switch maps live, no session restart: `relocalize` first (its refusals precede any mutation), then the robot's 3D planner (`planner_server/Octo3D/load_map` with the map's `octomap.bt`, which every goal plans on; it blocks until the tree is built, 120 s bound, and its success is a result), then `[map] name` into the instance INI so it survives a restart (`[initial_pose]` zeroed — re-seed), then the robot's internal service (`INTERNAL_API_URL`, `POST /internal/maps/{id}/activate`, waited on past its 240 s deadline because hanging up cancels it). That last call is outside the saga: its failure undoes nothing, the switch answers 200 with the reason appended to `message`. The `{id}` is the map's uuid there, looked up by name (`GET /internal/maps`) before anything moves. A later failure among the first three rolls back the earlier steps in the order done (Octo3D onto the previous map's tree, or none). `switched: false` is the no-op. `localized` is a best-effort `relocalize_check` poll: `false` = not yet (registration retries forever), `null` = could not ask; **not** `RobotState.localization_valid`, which is TF-presence only. Refusals, all before mutating: 409 `octomap_missing` (no `octomap.bt`), `pointcloud_missing`, `ini_not_writable`, `task_running` (any job), `tasks_unknown` (refuse rather than assume idle), `stack_not_ready` (the localizer's `relocalize` is not discoverable — how mapping mode is detected, not the cached mode), `planner_3d_unavailable` (a stack without Octo3D), `map_not_registered` (the internal service has no map of that name); 502 when the internal service cannot be asked for the uuid. Lifts `map_active` on rename and delete. The map's forbidden zones come in with its tree (Octo3D reads `keepout.json` in `load_map`), so no keepout call is made. No `map_kind` / `keepout_reloaded` since 2026-10-28 |
| GET | `/api/v1/maps/{name}/keepout` | Forbidden zones as saved through this API: `zones: [{id, points: [{x, y}, …], name, z_min, z_max}]` in map-frame metres (the last three `null` when unset; the band is the floor it forbids), plus `active`. Read from `keepout.json`; none answers `[]` |
| PUT | `/api/v1/maps/{name}/keepout` | Replace the zones with the **whole** body list (`{zones: [{id?, points: [{x, y}, …], name?, z_min?, z_max?}]}`; ids kept when given, generated when not). Writes `keepout.json`, then (map active) reloads the 3D planner's zones (`planner_server/Octo3D/load_keepout`) — `reloaded: true` means enforced now. `[]` **clears**: the json rewritten with none and reloaded. A failed reload is still **200** with `reloaded: false` and the reason — zones apply at the next load of this map. 400 for a zone under three points, a non-finite coordinate or height, `z_min` above `z_max`, an empty or duplicate id; 404 without an `octomap.bt` |
| GET | `/api/v1/maps/{name}/pointcloud` | The saved `map.pcd`, packed binary. The packed clouds are kept in an LRU of `RENDER_CACHE_MAX_BYTES` (64 MB, about a dozen maps), not for every map ever opened |
| GET | `/api/v1/maps/{name}/octomap` | The saved `octomap.bt` byte for byte (`application/octet-stream`), ETag = `"<octomap_version>"`, 304 on `If-None-Match`. 409 `octomap_converting` while the map cleaning is still building it; 404 with the reason otherwise (never built, `failed` with the cleaning's sentence, `interrupted`) |
| GET | `/api/v1/maps/{name}/octomap/edits` | The ground patches saved for the 3D map: `{name, res, fill_key: [[kx, ky, kz], …], created_at}`, absolute OcTree keys of the voxels to mark as floor. `fill_key: []` (and null `res` / `created_at`) when none are saved |
| GET | `/api/v1/maps/{name}/floors` | The storeys the map spans, lowest first: `{name, floors: [{name, level, low, high}], detected: [...], saved}`. `level` is the body height walked on that floor (the z of `poses.txt`), metres; `low` / `high` the lowest and highest walked on it, `null` on a floor named by hand. `floors` is the saved list if there is one, else `detected`; `[]` is one storey |
| PUT | `/api/v1/maps/{name}/floors` | Replace the floors with the **whole** body list (`{floors: [{name, level, low?, high?}, …]}`); `[]` saves one storey. A `low` / `high` pair that does not hold its `level` is dropped, not refused. Writes `floors.json`, sorted by level. 400 with a sentence for an empty or over-long name (16), a repeated name, more than 16 floors, or two levels under 1 m apart. Not held by a running job: nothing on the robot reads it |
| PUT | `/api/v1/maps/{name}/octomap/edits` | Replace the patches with the **whole** body list (`{fill_key: [[kx, ky, kz], …]}`, 0–65535 each, one per `[kx, ky]` column); `[]` clears. Writes `octomap_edits.json` (tmp + rename, `res` from the tree's header) and **nothing else** — the tree is patched on the robot by `apply_octomap_edits`, the message says so. 400 with a sentence for an out-of-range key or a column named twice; 404 with no tree; 409 `octomap_converting`, `octomap_unreadable` (a tree without a readable header), `task_running` / `tasks_unknown` (held by a running job like every map write) |
| POST · GET | `/api/v1/maps/{name}/vertices` | Batch-create (one transaction) / list with optional `?type=`. Create is held by a running job (below). `z` is the ground the vertex stands on, map frame, stored and sent with a MOVE as given: the console reads it off the 3D map under the vertex (on the floor picked; on a map with no floors, the ground there, so a stop on a slope carries its own height), and a vertex from before heights is 0. The robot's planner looks for the ground only 1 m either side of it, except where the ground under the stop is one layer |
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
| WS | `/api/v1/robot/telemetry/stream` | JSON frames keyed by `type`: `pose` (~20 Hz), `joints`, `path` (1 Hz), and while a navigation runs `local_costmap` (2 Hz) and `local_plan` (5 Hz) |

**A job's map is read-only while it runs.** Vertex create / update / delete,
`PUT …/keepout` and `PUT …/octomap/edits` answer 409 `task_running`
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
floor surface) — a waypoint's z — so a route that climbs a
stair can be drawn storey by storey. `pose.z` is base_link's height, not the
ground: in AUTO x, y and yaw are lio_bridge's, but its chain is projected onto
the plane (z = 0 on every storey), so the height is taken from the
`pointlio/lio_odom` chain while one is under a second old — the same number a
mapping run reports. An **empty**
`path.points` means "no route": the planner never publishes an empty plan, so
`TelemetryRepo` clears a route by TTL (arrival, cancel and abort are the same
silence from here).

**The local picture rides the same socket, only while a navigation runs**
(2026-10-20): `local_costmap`, the controller's 4 × 4 m rolling costmap (the
terrain layer's, inflated), and
`local_plan`, the 2 s trajectory MPPI is following. "Runs" is a MOVE or PATROL
goal this process sent that has not ended (`RobotGateway.navigation_active`)
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
| `ConflictError` | **409**, with an optional machine-readable `code` beside `detail` (e.g. `octomap_missing`, `map_active`, `restart_refused` / `restart_running`, `recording_running`, `tts_queue_full`, `whep_session_pending`; each route's row lists its own). Clients branch on the code, never on prose, which exists to be reworded |
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

## Settings pages

`/api/v1/settings/<page>` lets the console tune a node without a shell:
`controller` is `controller_server` (the walking speed, arrival tolerances,
stuck detection, path following, stairs; docking is deliberately not on it
yet), `planner` is `planner_server`'s `Octo3D` (route choice, obstacle
avoidance, destinations and the planning time limit). Three layers, and the
backend writes only the middle one:

| Layer | Where | Written by |
|---|---|---|
| Factory | `~/robot_ws/src/<package>/params/<node>_params.yaml` (the workspace's `syncai_controller` / `syncai_global_planner`, tracked; mounted read-only) | git, by hand |
| Runtime overlay | `~/robot_ws/config/params/<robot_id>/<node>.yaml` (gitignored there) | this backend |
| Live | the running node | this backend, over `<node>/get_parameters` / `set_parameters_atomically` |

Each node's launch loads the overlay after the factory file, so the overlay
holds **only the changed keys** and a key nobody touched follows the factory
file through a `git pull`. Deleting the overlay (and restarting the nav
session) is the fall back to the factory values; so is launching the node with
`use_runtime_params:=false`.

**Which parameters, and their limits, are this repo's schemas**
(`services/controller_schema.py`, `services/planner_schema.py`, on the types
in `services/settings_schema.py`): neither node declares descriptors, so
`DescribeParameters` has nothing to offer. One field can write several
parameters that are one calibration (the walking speed is MPPI's `vx_max` and
RPP 3D's `desired_linear_vel`). `apply` is read off each node's callbacks:
`live` parameters apply at once (the controller's plugins mid-goal, the
planner's at the next plan), `idle` ones (controller_server's own, undotted)
are refused while a goal runs, `restart` ones are read once at startup and are
only saved. Deliberately absent: the controller's `wz_max` (one calibration
with driver_manager's velocity scales) and the planner's robot geometry (built
into its surface map at startup, and one calibration with the controller's
local costmap terrain layer); each schema's docstring has the list.

A change (`services/node_settings.py`, one `NodeSettingsService` per page)
validates, sets the undotted parameters in a set of their own and then the
plugin ones, **reads them back** and saves what it read — a controller callback
that "rejected" a value used to report success — then writes the overlay
(tmp + rename, previous kept as `.bak`). A failure after the live set restores
the values read before it. The overlay is read before anything is set live, so
an unreadable one (409 `params_unreadable`) refuses with nothing to undo. Whether
the node is up is asked once per request: with the nav session down each ask
waits out the service timeout. `test_settings_schema.py` checks every parameter a
schema names against the workspace's factory file when
`SYNCAI_CONTROLLER_PARAMS` / `SYNCAI_PLANNER_PARAMS` point at them; run it after
a parameter is renamed.

## Recordings

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
disk-outlives-the-process split as the map cleaning's sidecar.

## WebRTC

`/api/v1/webrtc/*` is signalling only. The media path — capture, H.264 encode,
pion — lives in `libsyncai_worker.so` (built in `SyncAI-WebRTC-Worker`, copied in
by hand — see [*As its own container*](deployment.md#as-its-own-container)), which `gateways/webrtc` dlopens. It is the
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

## Vertex vs. MapPoint

REST says **"vertex"** with a `VertexType` enum (`MOVE` /
`CHARGE`); the ORM and repository say `MapPoint` (table
`map_vertices`). Intentional — no migration was done. `type` is validated at the
REST boundary and stored as a plain string.

A `CHARGE` vertex's `theta` is **the charger's own heading, out of its face** —
towards a robot in front of it — however it was placed. One placed with the
console's *Detect* (`POST /api/v1/landmarks/detect`) carries the charger's marker
as measured: its centre, `theta` out of its face, `z` the floor under the robot
that saw it.

## Task templates

`POST /api/v1/tasks` creates *and dispatches* and persists nothing, and Temporal
keeps closed workflows only for the namespace retention (5 days with the
workspace compose — see [*Task orchestration*](tasks.md)), with no archival. `task_templates` is where
re-dispatchable step lists live. The prefix is `/api/v1/task_templates`, not
`/api/v1/tasks/templates`, which would collide with `/api/v1/tasks/{id}` (see
the include-order note in `server.py`).

- **`steps` is one JSON column, not a child table.** No migrations exist (the
  schema is whatever `create_all` produced), so a column list could never change
  again, while a JSON array can grow optional keys. It is also the only record of
  step *order*, and a child table would be rewritten whole on every edit anyway.
  **Forward-compat rule: only add optional keys to a stored step; never rename,
  retype or repurpose one.**
- **A MOVE step keeps both a `vertex_id` and a `params` snapshot**; every read
  reports `resolved_params` — the vertex's *current* pose if it exists
  (`vertex_status: CURRENT`), else the snapshot (`MISSING`). Moving a vertex
  updates every template using it. Resolution is server-side (one
  implementation); the client dispatches by sending `resolved_params` through
  `POST /api/v1/tasks`.
- **A PATROL step keeps `vertex_ids` instead**, one per waypoint, index-aligned
  with `params.waypoints` (`null` = a point picked on the map, not a vertex;
  omitted = all free points, stored as a list of nulls). It never carries the
  single `vertex_id` (422), and a `vertex_ids` of another length is 422 too. The
  response adds the aligned `vertex_names` and `vertex_statuses` (`NONE` /
  `CURRENT` / `MISSING`, each meaning what MOVE's `vertex_status` does), swaps
  each CURRENT vertex's pose into its waypoint in `resolved_params`, and sets the
  step's single `vertex_status` to the worst of them (so `missing_vertex_count`
  counts the step once). An unknown or other-map vertex is 400 naming the
  waypoint (1-based, `Step 'p' waypoint 3 …`), and so is a MISSING one when the
  template is scheduled.
- **Map scoping keys off "contains a MOVE / PATROL / CHARGE", not "references a
  vertex"** — a hand-typed `(x, y, theta)` is in a map's frame too. Any such step ⇒
  `map_name`
  required; none ⇒ `map_name` absent, runs anywhere. A template for a map that is
  not active still saves (authoring ahead is legitimate), reported with
  `map_matches_active: false` for the client to gate on.
- **Cross-field rules answer 400 with a sentence**, not 422 with a validation
  array: the array is unreadable to an operator, and a `PUT` may conflict with
  the *stored* row, which no request-schema validator can see.

## The run record (`task_record`)

Temporal forgets a closed run after the namespace retention (5 days), too short for
a soak test's report. So every run, as it closes, also leaves one row in
PostgreSQL's `task_record` (`TaskRecordRow`): task id, run id, status
(`COMPLETED` / `FAILED` / `CANCELED`), start / close time, `kind` / `name` /
`map_name` / `schedule_id`, and `steps` — every `Step` as it ended, timing
included (JSON, so only ever add optional keys). `GET /api/v1/task_history/export`
reads it.

- **Written by the run itself**: `RobotWorkflow.run` ends with the `record_task`
  activity (IO queue, behind the `task-record` patch) on every way out —
  completed, failed, or cancelled (the run's cancel is consumed by the time it
  gets there, so the activity is not cancelled with it). Its insert is
  `ON CONFLICT DO NOTHING` on `(run_id, closed_at)`, and `closed_at` is
  `workflow.now()`, so a retry is a no-op. Failing three times is logged by the
  workflow and dropped: the record never changes how the run ends.
- **Not recorded**: a run *terminated* (or timed out) server-side runs no code;
  runs that closed before the table existed.
- **Partitioned by UTC day** on `closed_at` (`PARTITION BY RANGE`, created by
  `create_all` from the model's `postgresql_partition_by`; SQLite in the tests
  gets a plain table). Partitions are `task_record_pYYYYMMDD`; an insert creates
  its own if it is missing.
- **30 days, enforced by a routine** (`services/task_record_retention.py`): a
  daemon thread, every 30 minutes and once at start-up, creates today's and the
  next 3 days' partitions, then `DETACH`es and `DROP`s every partition older than
  today and the 29 days before it. A whole day goes at once, so the table never
  bloats the way a `DELETE` would. A thread rather than a Temporal schedule:
  PostgreSQL is a hard dependency, Temporal a soft one. Constants in
  `tasks/config.py` (`TASK_RECORD_*`).
- **Step timing** is set by the workflow on `workflow.now()` (no new commands,
  so no patch): `started_at` once the step is past the hold that may precede it,
  `ended_at` in every terminal branch (`_end`), and `paused_seconds` summed by
  `_hold` only while `started_at` is set — a hold before a step starts belongs
  to no step. The export's `step_active_s` is wall time less that.
