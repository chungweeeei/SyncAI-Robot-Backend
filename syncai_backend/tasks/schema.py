from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, ConfigDict, Field, model_validator
from pydantic.alias_generators import to_camel
from enum import Enum

from syncai_backend.exceptions import BadRequestError


class BaseSchema(BaseModel):
    model_config = ConfigDict(
        alias_generator=to_camel,
        validate_by_name=True,
        validate_by_alias=True,
    )


class StepStatus(str, Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    # Holding here (POST /api/v1/tasks/{id}/pause): interrupted (a MOVE, re-sent on
    # resume) or not yet started. At most one step at a time; the gateway derives the
    # task-level PAUSED from it.
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


# ARTIFACT was removed 2026-08; stored tasks / frozen schedules still carrying one
# fail validation, so purge them before deploying.
class StepType(str, Enum):
    MOVE = "MOVE"
    STANDUP = "STANDUP"
    LIEDOWN = "LIEDOWN"
    SPEAK = "SPEAK"
    WAIT = "WAIT"
    # A NavigateToGoal on the dock trees (gateways/robot DOCK_BEHAVIOR_TREE). DOCK's
    # x/y/theta is the dock pose, which dock.xml derives the staging pose from; UNDOCK
    # takes none, since undock.xml backs out 1 m from wherever the robot stands.
    DOCK = "DOCK"
    UNDOCK = "UNDOCK"


class MoveParams(BaseSchema):
    x: float = Field(
        ..., description="Target x-coordinate for the move step", examples=[1.0]
    )
    y: float = Field(
        ..., description="Target y-coordinate for the move step", examples=[2.0]
    )
    theta: float = Field(
        ...,
        gt=-180.0,
        le=180.0,
        description="Target orientation (in degrees) for the move step",
        examples=[90.0],
    )
    # The ground height of the goal's storey. The 3D planner snaps the goal to
    # the walkable surface nearest this height, so a fixed 0 would send every
    # goal on an upper floor to the one below it.
    z: float = Field(
        0.0,
        allow_inf_nan=False,
        description="Ground height (metres, map frame) of the storey the goal is on",
        examples=[0.0],
    )


# Constraints mirror routers/tts.py's SynthesizeRequest: both drive the same
# TtsGateway. Single-word field names, so camelCase aliasing is a no-op here.
SPEAK_REPEAT_MAX = 50


class SpeakParams(BaseSchema):
    text: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The text to speak. English only for now (see TtsGateway).",
        examples=["Delivery arrived"],
    )
    voice: str = Field(
        "af_heart",
        description="Kokoro voice id; the list is at GET /api/v1/tts/voices.",
    )
    speed: float = Field(
        1.0, ge=0.5, le=2.0, description="Playback rate multiplier (0.5–2.0)."
    )
    # Bounded against a slip; the workflow scales SPEAK's start_to_close by it, so it
    # also keeps that timeout finite.
    repeat: int = Field(
        1,
        ge=1,
        le=SPEAK_REPEAT_MAX,
        description=(
            "How many times to say the text, back to back (1–50). Omitted: once, "
            "so a step stored before this field speaks as it always did."
        ),
    )


# WAIT is a workflow timer (RobotWorkflow._run_wait); the bound only stops a typo
# from parking the robot for days while /tasks answers 409 task_running.
class WaitParams(BaseSchema):
    seconds: float = Field(
        ...,
        gt=0,
        le=3600,
        description="How long to wait before the next step, in seconds (max 1 hour).",
        examples=[10.0],
    )


# Pydantic's smart union tells the members apart by their disjoint required fields,
# so no discriminator is needed.
StepParams = MoveParams | SpeakParams | WaitParams


# Which params model each step type expects (None: takes none). Without it a MOVE
# with no params would validate and only fail inside the activity, after a 200.
STEP_PARAMS_TYPE: dict[StepType, Optional[type[BaseModel]]] = {
    StepType.MOVE: MoveParams,
    StepType.STANDUP: None,
    StepType.LIEDOWN: None,
    StepType.SPEAK: SpeakParams,
    StepType.WAIT: WaitParams,
    StepType.DOCK: MoveParams,
    StepType.UNDOCK: None,
}


# Steps whose params are a pose on the loaded map: they share MOVE's map stamping /
# check and its template vertex refs. UNDOCK is not one -- its goal carries no pose,
# so a job that only backs off the dock holds no map.
POSE_STEP_TYPES = frozenset({StepType.MOVE, StepType.DOCK})

# Steps that send a NavigateToGoal: they share MOVE's pause interrupt and its live
# status sentence, pose or not.
NAV_STEP_TYPES = POSE_STEP_TYPES | {StepType.UNDOCK}


# Steps on the one-slot body queue (config.TASK_QUEUE_SUFFIX). Two in one group would
# just run one after the other, so a group may hold at most one.
BODY_STEP_TYPES = NAV_STEP_TYPES | {StepType.STANDUP, StepType.LIEDOWN}


# The one ``code`` for both step_group_problem rules: a client just marks ``step_id``.
INVALID_STEP_GROUP = "invalid_step_group"


class StepGroupProblem(str):
    """step_group_problem's sentence, carrying the step it is about.

    A ``str`` so callers raising it as a message keep working; routers also read
    ``step_id`` into the 400. Raise it with ``refusal()``.
    """

    code = INVALID_STEP_GROUP
    step_id: str

    def __new__(cls, message: str, step_id: str) -> "StepGroupProblem":
        obj = super().__new__(cls, message)
        obj.step_id = step_id
        return obj

    def refusal(self) -> BadRequestError:
        return BadRequestError(self, code=self.code, step_id=self.step_id)


def step_group_problem(steps) -> Optional[StepGroupProblem]:
    """Why ``steps``' groups cannot run, as a sentence; None when they can.

    ``step_id`` names the step that rejoins a closed group, or the second
    body step (BODY_STEP_TYPES) of one. A group must be *adjacent* steps (RobotWorkflow.
    _run_group): the same name after another step would be a second group the
    operator takes for the first. Duck-typed on ``.id`` / ``.type`` / ``.group``
    so the request models and Step share it.
    """
    closed: set[str] = set()
    current: Optional[str] = None
    body: list[str] = []
    for step in steps:
        group = step.group
        if group != current:
            if current is not None:
                closed.add(current)
            if group is not None and group in closed:
                return StepGroupProblem(
                    f"Steps in group '{group}' must be adjacent; step "
                    f"'{step.id}' rejoins it after another step",
                    step.id,
                )
            current, body = group, []
        if group is not None and step.type in BODY_STEP_TYPES:
            body.append(step.id)
            if len(body) > 1:
                return StepGroupProblem(
                    f"Group '{group}' has more than one MOVE/DOCK/UNDOCK/STANDUP/LIEDOWN step "
                    f"({', '.join(body)}); the robot can do only one at a time",
                    step.id,
                )
    return None


def validate_step_params(
    step_type: StepType, params: Optional[StepParams]
) -> Optional[StepParams]:
    expected = STEP_PARAMS_TYPE[step_type]

    if expected is None:
        if params is not None:
            raise ValueError(f"{step_type.value} step takes no params")
        return params

    if not isinstance(params, expected):
        raise ValueError(f"{step_type.value} step requires {expected.__name__}")

    return params


# Shared by Step and the REST request/response models, which all carry it.
GROUP_DESCRIPTION = (
    "Adjacent steps with the same group run at the same time, started in "
    "list order (each once the one before it has started), and the next "
    "step starts once all of them have finished; if one fails the others are "
    "canceled. At most one MOVE/DOCK/UNDOCK/STANDUP/LIEDOWN per group. Omitted: the step "
    "runs on its own, after the one before it"
)


class Step(BaseSchema):
    id: str = Field(
        ..., description="Unique identifier of the step", examples=["step1"]
    )
    type: StepType = Field(..., description="Type of the step", examples=["MOVE"])
    params: Optional[StepParams] = Field(
        default=None,
        description=(
            "Parameters for the step, which vary based on the step type. "
            "Omitted for STANDUP/LIEDOWN/UNDOCK, required for MOVE, DOCK (x/y/theta: "
            "the dock pose), SPEAK and WAIT"
        ),
    )
    status: StepStatus = Field(
        StepStatus.PENDING,
        description="Current status of the step",
        examples=["PENDING"],
    )
    error_msg: Optional[str] = Field(
        default="",
        description="Error message if the step failed",
        json_schema_extra={"example": ""},
    )
    group: Optional[str] = Field(
        default=None,
        description=GROUP_DESCRIPTION,
    )

    @model_validator(mode="after")
    def _check_params(self) -> "Step":
        validate_step_params(self.type, self.params)
        return self


class WorkflowTaskDefinition(BaseSchema):
    steps: List[Step] = Field(..., description="Ordered list of steps to execute")

    # The routers answer 400 first; this keeps a run from ever starting with one.
    @model_validator(mode="after")
    def _check_groups(self) -> "WorkflowTaskDefinition":
        problem = step_group_problem(self.steps)
        if problem is not None:
            raise ValueError(problem)
        return self


class WorkflowTask(BaseSchema):
    id: str = Field(..., description="Unique identifier of the workflow task")
    definition: WorkflowTaskDefinition = Field(
        ..., description="Definition of the workflow task"
    )


class TaskState(BaseSchema):
    id: str = Field(..., description="Unique identifier of the task")
    status: str = Field(..., description="Overall task status")
    steps: List[Step] = Field(..., description="Per-step state")


class TaskSource(str, Enum):
    """Who started a running task.

    DIRECT, not OPERATOR: `POST /api/v1/tasks` has callers besides the console, so
    the backend can only tell "the endpoint was called" from "a schedule fired".
    """

    DIRECT = "DIRECT"
    SCHEDULE = "SCHEDULE"


class TaskKind(str, Enum):
    """How a run was started, as the console mints it into the task id.

    Lowercase: these are literally the ``<kind>`` in ``<robot_id>-<kind>-<ts>-<seq>``.
    SCHEDULE is stamped by the backend; ``TaskRequest`` refuses it from a direct POST.
    """

    GOAL = "goal"
    STANDUP = "standup"
    LIEDOWN = "liedown"
    TASK = "task"
    SCHEDULE = "schedule"


class TaskProvenance(BaseSchema):
    """What a run carries about its origin beyond the id Temporal keeps.

    Stamped as search attributes (search_attributes.py) so history can filter and
    count on them. Both optional: a run dispatched with neither is unlabelled.
    """

    kind: Optional[TaskKind] = Field(default=None)
    # TaskTemplate.name is String(255); the bound keeps the attribute in step.
    name: Optional[str] = Field(default=None, max_length=255)
    # Stamped as ``TaskMap``: the map loaded at dispatch, set by the task router,
    # never taken from the caller. None for a run with no MOVE.
    map_name: Optional[str] = Field(default=None)


# Kinds that never drive, so hold no map; only for runs predating ``TaskMap``.
POSTURE_KINDS = frozenset({TaskKind.STANDUP, TaskKind.LIEDOWN})


class ActiveTask(BaseSchema):
    """One execution running on this robot's task queue right now.

    Identity and provenance only, no steps: it comes from one visibility query, cheap
    enough for the console to poll (ACTIVE_TASK_CACHE_TTL_S). Steps are at
    GET /api/v1/tasks/{id}.
    """

    id: str = Field(..., description="Workflow id, i.e. the task id")
    run_id: str = Field(
        ...,
        description=(
            "Temporal run id. Distinguishes two runs of the same schedule, "
            "whose workflow ids differ only by nominal trigger time."
        ),
    )
    status: str = Field(..., description="Mapped through _WORKFLOW_STATUS_MAP")
    started_at: datetime = Field(..., description="Execution start time (UTC)")
    source: TaskSource = Field(..., description="Who started it")
    schedule_id: Optional[str] = Field(
        default=None,
        description="The schedule that started it, when source is SCHEDULE.",
    )
    kind: Optional[TaskKind] = Field(
        default=None,
        description="How it was started; None for a run dispatched without saying.",
    )
    name: Optional[str] = Field(
        default=None, description="The template it was dispatched from, if any."
    )
    map_name: Optional[str] = Field(
        default=None,
        description=(
            "The map its MOVE coordinates are in, off the TaskMap attribute; "
            "None for a run that stamped none. Read it through map_in_use."
        ),
    )

    def map_in_use(self, active_map: Optional[str]) -> Optional[str]:
        """The map this run holds while it runs, or None if it holds none.

        The attribute when present. Otherwise a posture run holds none, and any other
        run predates ``TaskMap`` and holds the map loaded *now*: activate refuses to
        switch while anything runs, so that is the one it started on. Over-claims for
        a map-less job (SPEAK-only), which only makes an edit wait.
        """
        if self.map_name:
            return self.map_name
        if self.kind in POSTURE_KINDS:
            return None
        return active_map


class TaskHistoryEntry(BaseSchema):
    """One finished execution on this robot's task queue.

    ActiveTask's shape plus close time; steps via GET /api/v1/tasks/{id} while
    retention has it.
    """

    id: str = Field(..., description="Workflow id, i.e. the task id")
    run_id: str = Field(..., description="Temporal run id")
    status: str = Field(..., description="Mapped through _WORKFLOW_STATUS_MAP")
    started_at: datetime = Field(..., description="Execution start time (UTC)")
    closed_at: Optional[datetime] = Field(
        default=None, description="Execution close time (UTC)"
    )
    source: TaskSource = Field(..., description="Who started it")
    schedule_id: Optional[str] = Field(
        default=None,
        description="The schedule that started it, when source is SCHEDULE.",
    )
    kind: Optional[TaskKind] = Field(
        default=None,
        description="How it was started; None for a run dispatched without saying.",
    )
    name: Optional[str] = Field(
        default=None, description="The template it was dispatched from, if any."
    )
    map_name: Optional[str] = Field(
        default=None,
        description="The map its MOVE coordinates were in; None if it stamped none.",
    )


class TaskHistoryStats(BaseSchema):
    """Finished runs matching a history filter, counted by ``count_workflows``:
    exact for the whole retention window."""

    as_of: datetime = Field(..., description="When the counts were taken (UTC)")
    total: int
    completed: int
    failed: int
    canceled: int


class ScheduleTrigger(BaseSchema):
    cron: Optional[str] = Field(
        default=None,
        description=(
            "Cron expression, e.g. '0 9 * * 1-5'. Mutually exclusive with intervalSeconds."
        ),
        examples=["*/3 * * * *"],
    )
    interval_seconds: Optional[int] = Field(
        default=None,
        gt=0,
        description="Fire every N seconds. Mutually exclusive with cron.",
        examples=[1800],
    )
    timezone: Optional[str] = Field(
        default=None,
        description="IANA timezone applied to cron, e.g. 'Asia/Taipei'. Ignored for interval.",
        examples=["Asia/Taipei"],
    )


class ScheduleTask(BaseSchema):
    id: str = Field(..., description="Unique identifier of the schedule")
    trigger: ScheduleTrigger = Field(..., description="When the schedule fires")
    definition: WorkflowTaskDefinition = Field(
        ..., description="Task definition executed on each trigger"
    )

    # Provenance, in the schedule memo (_schedule_to_memo), which the list path can
    # read. The trigger is not memo material: an update cannot rewrite a memo.
    map_name: Optional[str] = Field(
        default=None,
        description="Map whose frame this schedule's MOVE coordinates are in.",
    )
    task_template_id: Optional[str] = Field(
        default=None,
        description="The task template this schedule was frozen from, if any.",
    )
    task_template_name: Optional[str] = Field(
        default=None, description="That template's name at registration time."
    )


class ScheduleView(BaseSchema):
    id: str = Field(..., description="Unique identifier of the schedule")
    trigger: ScheduleTrigger = Field(..., description="When the schedule fires")
    paused: bool = Field(False, description="Whether the schedule is currently paused")
    next_run_times: List[datetime] = Field(
        default_factory=list,
        description="Upcoming trigger times (UTC)",
    )
    map_name: Optional[str] = Field(default=None)
    task_template_id: Optional[str] = Field(default=None)
    task_template_name: Optional[str] = Field(default=None)
    steps: List[Step] = Field(
        default_factory=list,
        description=(
            "The frozen step list, decoded from the schedule's start-workflow "
            "args. Populated by describe() only -- always empty from "
            "list_schedules(), whose ScheduleListActionStartWorkflow carries the "
            "workflow type name and nothing else. Frozen at registration: later "
            "vertex edits do not reach a scheduled run."
        ),
    )
