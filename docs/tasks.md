# Task orchestration (Temporal)

> Part of the `syncai_backend` reference — see the [documentation index](../README.md#documentation).

A task is an ordered list of steps; `RobotWorkflow` walks them one at a time by
`StepType` — or a few at a time, where adjacent steps share a `group` (below):

| StepType | Activity | What it does |
|---|---|---|
| `MOVE` | `execute_move` | Send a `NavigateToGoal` goal, poll to a terminal state, heartbeat every 0.25 s; a failure's message carries the robot's `error_msg`. A robot still on the dock the last DOCK ended on is stood up (if lying) and undocked first — [*Auto-undock*](ros-interfaces.md) under [*Action client*](ros-interfaces.md) |
| `PATROL` | `execute_patrol` | One `NavigateThroughGoals` through `PatrolParams.waypoints` (at least two `MoveParams`; only the last `theta` is turned to) — *PATROL* under [*Action client*](ros-interfaces.md). MOVE's map check and auto-undock, poll, heartbeat and pause interrupt. Never starts the round over: the heartbeat details carry how many waypoints are behind the robot (from `number_of_poses_remaining`), so a retry sends only the rest, and the activity signals the count to the run (`patrol_progress`, forward only), which re-dispatches a resumed patrol with it as the activity's `start`. The step's own `params` keep every waypoint; the last one is always sent. An abort fails it with the robot's reason, as a MOVE's (`patrol ended in aborted: …`), retried or not by its `error_code` |
| `DOCK` / `UNDOCK` | `execute_dock` / `execute_undock` | `execute_move` on the dock trees (`behavior_tree` `dock.xml` / `undock.xml`, see [*Action client*](ros-interfaces.md)). DOCK's `MoveParams` is the dock pose and gets MOVE's map check; UNDOCK takes no params, sends an empty pose and is checked against no map, since it backs out 1 m from wherever the robot stands. DOCK's success records the dock for the next MOVE's auto-undock, UNDOCK's forgets it; the console no longer offers UNDOCK, since a MOVE undocks by itself. Same poll, heartbeat and pause interrupt as MOVE; a cancel (or pause) also sends `MotionKey.LOCOMOTION`, best effort, because a halted dock tree stays in MPC |
| `STANDUP` / `LIEDOWN` | `execute_stand` / `execute_lie_down` | Send the motion key; fire-and-forget (see `activities.py`) |
| `SPEAK` | `execute_speak` | `TtsGateway.start_speak()` then `await_speak()` — queue the utterance on the speech service, then poll its job and heartbeat until it has played; a cancel `DELETE`s the job mid-sentence. `SpeakParams`: `text` (1–1000 chars, English only), `voice` (default `af_heart`; see `GET /api/v1/tts/voices`), `speed` (0.5–2.0) — the REST route's constraints, since both drive one gateway |
| `WAIT` | — (workflow timer) | Idle for `WaitParams.seconds` (0 < s ≤ 3600). Not an activity: a durable timer in `RobotWorkflow._run_wait`, so no slot of the body queue's one-thread executor, no heartbeat, and it survives a backend restart with the time already served |

(`ARTIFACT` — conveyor pickup/drop via the artifact backend's REST API — was
removed 2026-08 with `gateways/artifact/`; templates or schedules still carrying
one must be purged before deploying.)

**Which failures are retried (2026-11).** Every step activity runs under the
workflow's `RetryPolicy` — three attempts, 5 s apart — and a failure raised
`non_retryable` skips the rest. For a MOVE / DOCK / UNDOCK / PATROL that the
robot aborted, the robot's `error_code` decides: `NON_RETRYABLE_NAV_CODES`
(activities.py) are the planning reasons for which the same goal cannot succeed
5 s later, and fail the step on the first attempt:

| `error_code` | Reason |
|---|---|
| 201 `INVALID_PLANNER` | the planner id names no loaded planner |
| 203 / 204 `START_` / `GOAL_OUTSIDE_MAP` | outside the map's grid |
| 209 `NO_VIAPOINTS_GIVEN` | a route with no waypoints |
| 220 `MAP_NOT_LOADED` | no 3D map loaded (until a map switch) |
| 222 `START_IN_KEEPOUT` | the robot stands inside a keepout zone (until it is moved) |
| 223 `GOAL_IN_KEEPOUT` | the goal, or a PATROL waypoint, is inside a keepout zone (until the zone is edited) |
| 224 `DISCONNECTED` | start and goal on ground no stair joins |

Everything else is retried: 200 (no finer reason), 202 TF, 205 / 206 nothing to
stand on at the start / goal — which includes a person on it — 207 search
timeout, 208 the way closed (a person across it, or a zone), 221 the map still
loading, 1xx path following, 3xx docking, and an abort with no code (superseded
or never started). Before this a goal inside a keepout zone was sent three
times and failed after ~10 s, for a reason the robot knew at once. The numbers
are SyncAI-Robot-Interface's planning table (`ComputePathToPose.action`), the
same on `NavigateToGoal` and `NavigateThroughGoals`; they are never renumbered,
so they are written here as literals.

**Groups — steps side by side.** A step may carry an optional `group` (any
string). Adjacent steps with the same `group` run side by side, and the next step
starts once all of them have finished — e.g. `MOVE(group=a)`, `SPEAK(group=a)`,
`STANDUP` speaks while it drives, then stands. Rules, checked on
`POST /api/v1/tasks`, schedules and templates (400 with a sentence, plus
`code: "invalid_step_group"` and the `step_id` that breaks the rule — the step
rejoining a closed group, or a group's second body step):

- a group's members must be **adjacent** — the same name again after another
  step is refused, not read as a second group;
- **at most one `MOVE` / `PATROL` / `DOCK` / `UNDOCK` / `STANDUP` / `LIEDOWN`** per group: the body queue has one
  slot, so two would only run one after the other.

**Start order**: a group's members start **in list order**, each once the one
before it has *started* — not finished — and from then on they overlap:

| Member | Started when |
|---|---|
| `MOVE` / `PATROL` / `DOCK` / `UNDOCK` | nav2 accepted the goal |
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
  worker polling yet) instead of failing the request. The query and the describe
  in front of it are bounded at `TASK_STATE_RPC_TIMEOUT_S` (3 s): with no worker
  polling, the pinned server held the query ~30 s before failing it (measured
  2026-10-25), so every poll of the detail page hung that long; bounded, it
  degrades in 2-3 s. A describe that times out is a 502.
- **Continuing a failed task** (`POST /api/v1/tasks` with `continue_from`) is a
  new run, under the caller's new id; the failed run stays FAILED. The backend
  reads the failed run's steps off its query (`WorkflowGateway.get_failed_run`:
  409 `task_not_failed` unless it failed; a failed query is a 502, never "no
  steps") and `continuation_steps` (schema.py) dispatches **every** step, so the
  list matches the failed run's: the block of the first FAILED step and all after
  it fresh and `PENDING` (a group again as a whole), those before it `SKIPPED`,
  which `RobotWorkflow.run` passes over without dispatching or timing. Not a copy
  of their `COMPLETED`: they did not run in this run, and the report would count
  them twice. 409 `no_failed_step` when none failed. A PATROL keeps its
  `waypoints_passed` and the run seeds its dispatch count from it, so it goes on
  past them. The positions are the failed run's, so its `TaskMap` is the
  expectation (409 `map_mismatch` when the loaded map changed), checked only
  against the steps still to run; `kind` / `name` default to the failed run's, a
  schedule's becoming `task`. Replay-safe without a patch: no run before it has a
  `SKIPPED` step or a nonzero `waypoints_passed` in its input.
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
  2 m off the dock, not at a fixed staging pose. A `PATROL` is interrupted the
  same way, but resumes **past the waypoints it had passed**: its activity reports
  each one on the `patrol_progress` signal, and `_run_move` hands the count to
  the next dispatch as its `start` (the last waypoint is always kept). The count
  is also kept on the step (`waypoints_passed`), for the query and for a run
  continuing this one.
  Consequences:
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
  - **Bounded by namespace retention: 5 days.** The Temporal container sets it,
    not this repo: the workspace compose's `temporal` service passes
    `DEFAULT_NAMESPACE_RETENTION=${TEMPORAL_NAMESPACE_RETENTION:-5d}` (since
    2026-10-29; auto-setup's own default is one day, too short for the history
    dashboard's time-range filter). That variable is read only when the
    namespace is created, so a stack whose namespace already exists keeps its
    old retention until it is updated by hand:
    `docker exec temporal sh -c 'temporal operator namespace update --address $(hostname -i):7233 --namespace default --retention 5d'`
    — applies to runs closing from then on; closed runs keep their expiry. The
    backend needs no change for another value: nothing here caps `since`.
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
- **A job's map is checked twice.** MOVE / PATROL / DOCK coordinates are a place in one map's
  frame; on another map they are a real place, just the wrong one. The rule is
  `helpers/move_guard.py`: refuse positions planned on another map
  (`map_mismatch`); an unknown map is never refused. `POST /api/v1/tasks`
  asks before starting a run, and `execute_move` (and `execute_dock`, `execute_patrol`) asks
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
- **A schedule registered from a template runs the template as it is when it
  fires.** The action's `WorkflowTask` carries `task_template_id` beside the steps
  resolved at registration, and the run's first act (`RobotWorkflow._resolve_template`,
  behind the `schedule-resolves-template` patch) is the IO-queue activity
  `resolve_template`: the template's steps, resolved against its vertices by the
  rule the template routes use (`tasks/template_resolution.py`), its name and
  map. The run then upserts `TaskName` / `TaskMap` to them, so the history names
  it as the operator does today and every MOVE checks the map the template is on
  now; another map loaded fails the first step with `move_guard`'s sentence
  before anything moves or speaks. A vertex deleted since resolves to its
  snapshot, as an immediate dispatch does; a template that is gone or cannot be
  read leaves the run on the registration's steps. Deleting a template deletes
  its schedules first (`WorkflowGateway.delete_template_schedules`), and the
  schedule routes answer `task_template_name` from the template, since the memo
  cannot be rewritten. A schedule registered with its own steps
  (`POST /api/v1/schedules`), and every one from before the patch, still runs the
  steps it was registered with.
  `POST /api/v1/task_templates/{id}/schedule` still refuses a template whose map is
  not active, or with a `MISSING` vertex, at registration.
