# ROS interfaces

> Part of the `syncai_backend` reference — see the [documentation index](../README.md#documentation).

**Subscriptions**

| Topic | Type | QoS | Goes to |
|---|---|---|---|
| `robot_state` | `syncai_common/RobotState` | BEST_EFFORT, depth 3 | `RobotRepo` → `GET /api/v1/robot/state`; `low_level_mode.safety_state` → `SafetyLockService` (rising edge cancels every task) |
| `odom` | `nav_msgs/Odometry` | BEST_EFFORT, depth 5 | composed with TF `map→odom` → telemetry WS (x, y, yaw; z from `pointlio/lio_odom`, see [*Telemetry*](rest-api.md)) |
| `pointlio/lio_odom` | `nav_msgs/Odometry` | BEST_EFFORT, depth 5 | composed through `map→pointlio_odom` and the lidar mount → telemetry WS: the whole pose while mapping, base_link's height in AUTO |
| `motor_states_low_rate` | `syncai_common/MotorStates` | BEST_EFFORT, depth 5 | reduced to `{joint: radians}` → telemetry WS. driver_manager's 20 Hz copy of `motor_states` (~200 Hz) |
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

**PATROL: `navigate_through_goals` (`syncai_common/NavigateThroughGoals`), 2026-11.**
A second action client on the same server (`syncai_task_runner`), for the PATROL
step: `RobotGateway.patrol()` sends every waypoint as a `PoseStamped` (frame `map`,
`z` = that waypoint's storey, yaw-only quaternion) with `behavior_tree: ""`, the
task runner's `patrol.xml` — through every pose in order without stopping, only
the last reached with its heading (xy 0.10 m, 0.2 rad, as a MOVE). It was nav2's
`NavigateThroughPoses` on `navigate_through_poses` until SyncAI-Robot-Interface
grew `NavigateThroughGoals` (2026-11): the same goal with NavigateToGoal's
`error_code` / `error_msg` result, so **a PATROL failure reads as a MOVE's** —
`_move_failure`, "navigation did not start" for code 0 included, and the same
retry decision ([*Task orchestration*](tasks.md), "Which failures are retried"). The task
runner still serves the nav2 action beside it. Needs `syncai_common` with
`NavigateThroughGoals` and a task runner that serves it (SyncAI-Robot-Workspace
`feature/plan-error-codes`). The goal lands in the same `MoveGoal` table (`patrol_total` marks it), so the
status poll, `cancel_active_moves`, the teleop refusal and `navigation_active`
cover it unchanged. Its feedback has no sentence, so `get_move_message` builds
the step's `message` from it — `Patrolling: 3 of 5 waypoints left, 12.4 m to go`
(`number_of_poses_remaining` of the step's waypoints, `distance_remaining`
omitted until a route is planned) — and `get_move_status` adds
`poses_remaining`, which the activity turns into the waypoints already passed
([*Task orchestration*](tasks.md)). One navigation at a time across `navigate_to_pose` /
`navigate_to_goal` / `navigate_through_poses` / `navigate_through_goals`: the task runner rejects a goal
sent while another navigator runs, which the step reads as a retryable
`Patrol rejected`.

**Route preview: `preview_path_to_pose` (`syncai_common/ComputePathToPose`), 2026-11.**
The planner's own action, on the server `planner_server` added for the console's
preview (SyncAI-Robot-Workspace `feature/preview-path-to-pose`): the same plan a
MOVE's would get, answered to the caller and **never published on `plan` or
kept as the last plan**, so a preview is not drawn as the route being driven
and does not disturb a running navigation's `is_path_valid`. Not
`compute_path_to_pose`, which is the task runner's and takes one goal at a time.
`RobotGateway.preview_route()` sends the goal like a MOVE's (frame `map`, `z` the
floor, `use_start: false`, `planner_id: ""`), waits `NAV_ACCEPT_WAIT_S` for
acceptance and `PREVIEW_RESULT_WAIT_S` (6 s: Octo3D's 2 s search, behind a
navigation's plan in flight) for the answer, and cancels a goal it gave up on.
Nothing is tracked: a preview moves nothing. A failure keeps the planner's code
and is worded by `helpers/route_preview.py` in the task runner's sentences, so a
preview that fails reads as the MOVE would have. ABORTED with code `NONE` is a
preview a newer one preempted (the planner takes one at a time),
`preview_superseded`.

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
  - `planner_server/Octo3D/load_map` and `planner_server/Octo3D/load_keepout`
    (`nav2_msgs/LoadMap`, the robot's 3D planner) — a switch points Octo3D at
    the map's `octomap.bt` (`""` unloads, only ever a failed switch's undo onto
    a map without one); a keepout save on the active map re-reads its
    `keepout.json`. No `map_server` / `filter_mask_server` clients since the 2D
    removal (2026-10-28).
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
