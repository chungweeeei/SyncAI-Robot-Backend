# Gotchas

> Part of the `syncai_backend` reference — see the [documentation index](../README.md#documentation).

- **Configuration is read once, at startup.** The INI (`[map]`, `robot_id`),
  `.env` and the environment are all read during construction — any change needs
  a backend restart. That includes restoring `[map] name` from `system.ini.bak`
  or editing it by hand: only `activate` changes the map this process reports.
- Relative topic names are not optional — see [*robot_id, namespaces, and per-robot isolation*](architecture.md#robot_id-namespaces-and-per-robot-isolation).
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
- **A resumed run is saved as another map.** `POST /api/v1/mapping/resume` loads
  a saved map and maps on in its frame; pgo refuses a save into that map's own
  directory (the save empties `patches/`, the only copy), and `POST /api/v1/maps`
  refuses an existing name anyway. The new map shares the old one's frame but
  not its rows: waypoints (`map_vertices`), `keepout.json`, `octomap_edits.json`
  and `floors.json` stay with the old name, and nothing copies them yet.
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
- **Deleting `keepout.json` clears nothing now.** The running 3D planner keeps
  the zones it last read. Clear with `PUT .../keepout` and `[]`, which rewrites
  the file with none and reloads it.
- **A client that drops `name` / `z_min` / `z_max` widens zones.**
  `PUT …/keepout` replaces the whole list, and a zone sent without a band
  covers every floor. Send back what `GET` returned.
- **The robot boots unconfirmed, and so do its schedules.** Since 2026-11
  `syncai_robot_state` comes up `UNINITIALIZED` and stays there until an
  operator confirms the position (`POST /api/v1/robot/state` with `IDLE`) — a
  boot guess that converges on the last saved pose does not count. Until then
  every run with a MOVE / PATROL / CHARGE fails on its first step
  (`not_initialized`), scheduled ones included; a run that only stands, lies
  down, speaks or waits still goes. A map switch and every new pose put the
  robot back there.
- **One failed task stops all the others.** A run that ends `FAILED` latches
  `ERROR` on the robot (`end_task`, whose `reason` the robot quotes verbatim
  in the sentence the console shows — keep it a sentence, with no error-type
  prefix), and every run after it — a posture or a
  SPEAK too, and every schedule — fails at once (`error_latched`) until an
  operator releases it with `POST /api/v1/robot/state`. That includes a MOVE
  that gave up after three tries on a person in the way. A cancel (the
  operator's, the safety lock's) and a run the robot refused never latch.
- **A `robot_state` sample is believed for 3 s, then it is no sample.**
  `RobotRepo` stamps each one on arrival (monotonic clock) and every default
  read — `get_robot_state()`, `status()`, `state_message()` — answers "none"
  once it is older than `ROBOT_STATE_MAX_AGE_S` (three periods of
  syncai_robot_state's 1 Hz, so two dropped best-effort frames are not a dead
  robot). Until 2026-11 it never aged, and a dead state node's last word went
  on answering for it. There is no accessor for the last sample whatever its
  age: no reader has a use for a state nobody is reporting. Raising the
  publish rate's period means raising this with it.
- **`POST /api/v1/tasks`' robot-state 409s are advisory.** They read the
  latest fresh `robot_state` sample, up to 3 s old; a stale one is no sample
  and passes, because the decision is `begin_task`'s, taken on the robot as
  the run starts, and it fails closed when the state service is gone.
- **A navigation goal waits for the robot to speak.** Before every MOVE /
  PATROL / CHARGE goal, no fresh sample (stale, or never received) holds the
  goal back as `robot_state_stale` — **retryable**, unlike the fresh
  `UNINITIALIZED`'s non-retryable `not_initialized`. A few seconds without a
  frame is more often a hiccup than a dead node, and the step's retry policy
  (three attempts, 5 s then 10 s) gives it about 15 s to come back; after
  that the step fails, and the run latches `ERROR` with that sentence. So a
  restart of `syncai_robot_state` mid-task (a mode switch does one) is
  survived only if it comes back within those ~15 s.
- **A new initial pose mid-task is refused, and pause does not lift that.**
  `POST /api/v1/robot/set_initial_pose` is 409 `task_running` while a job runs,
  held included: a new pose withdraws the confirmation, so the task's next
  navigation step would fail and latch `ERROR`. "Pause, re-seat the pose,
  resume" no longer works — cancel the task first, then set the pose and
  confirm.
- **`RUNNING` with no job is a robot nobody released.** A workflow terminated
  from the Temporal UI never sends `end_task`; the robot then refuses every
  task `robot_busy`. (A `begin_task` whose answer was lost is not that case any
  more: the run follows it with a best-effort `end_task(CANCELED)`, and only if
  that goes unanswered as well is the robot left counting it.)
  `POST /api/v1/robot/state` clears it — which is exactly why that route is
  refused while this backend's own job list has a running job, and reads that
  list fresh.
- **The backend, `syncai_common` and the robot stack move together.**
  `RobotState` grew two trailing fields, so a backend built against the new
  message cannot read an old `syncai_robot_state`'s topic, nor the reverse.
  Update with no task running; nothing stored needs migrating.
- The `sqlalchemy` session convention is per-repo: `init_map_repo` creates the
  schema and builds its own `sessionmaker` from the injected engine.
