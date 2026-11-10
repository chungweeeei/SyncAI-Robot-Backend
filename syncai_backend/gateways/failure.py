"""Machine-readable discriminators for gateway failures.

A gateway reports trouble as ``(False, message, payload)`` with operator prose.
A few failures mean something specific to the caller (REST answers 400/409
instead of 502; the Temporal activity goes non-retryable). Keying that on the
prose couples callers to a sentence and lets two of them disagree after a
reword, so those failures carry a ``code`` instead.

``FailureMessage`` is a ``str`` subclass, so anything treating the message as
prose keeps working; read the code with :func:`failure_code` (``None`` for the
untagged majority).

Add a code only when some caller acts on the difference. The webrtc gateway's
camera-busy case stays untagged: the Go error string cannot tell it apart, and
a code that guesses is worse than prose.
"""

from enum import Enum
from typing import Optional


class Failure(str, Enum):
    """The gateway failures that some caller answers differently.

    The values are the wire strings REST publishes as ``ConflictError.code``.
    """

    # TtsGateway: the caller's to fix (400; non-retryable for SPEAK). Must equal
    # the code syncai_tts publishes, which the gateway re-tags with this enum;
    # renaming either side turns a 400 into a 502 and retries a typo'd voice.
    UNKNOWN_VOICE = "unknown_voice"

    # TtsGateway: the speech service's queue is full. A 409 ("wait or cancel"),
    # not a 502 that would say the robot is broken.
    TTS_QUEUE_FULL = "tts_queue_full"

    # RecordingGateway: 409s -- stop the running bag, or free some disk.
    RECORDING_RUNNING = "recording_running"
    DISK_LOW = "disk_low"

    # RobotGateway.restart_mode: sys_manager refused with nothing touched
    # (MAINTENANCE, both sessions up, or MANUAL with an unsaved map). A 409: the
    # next step is switch_mode, not a retry. Unreachable stays 502.
    RESTART_REFUSED = "restart_refused"

    # WebRtcGateway: a create is mid-flight and has no session id yet, so it
    # cannot be preempted. The caller retries; everything else is a 502.
    WHEP_SESSION_PENDING = "whep_session_pending"

    # ParameterGateway: controller_server refuses its own (undotted) parameters
    # while a goal runs. A 409 -- set it again once the robot has stopped.
    CONTROLLER_BUSY = "controller_busy"

    # RobotGateway.preview_route: a newer preview (this console's or another's)
    # preempted this one on the planner, which takes one at a time. A 409: the
    # newer one is the answer, and nothing is broken.
    PREVIEW_SUPERSEDED = "preview_superseded"

    # ParameterGateway: a parameter callback refused the value with a reason
    # (DockingController's v_linear_min above v_linear_max, say). A 400: the
    # combination is the caller's to change; the schema's rules should have
    # said so first, so one of these is also a missing rule.
    PARAMETER_REFUSED = "parameter_refused"

    # LandmarkGateway: the landmark was not (steadily) in view within the
    # detect's budget. A 409: face the robot at it and ask again; a detector
    # that is down or cannot transform stays 502.
    LANDMARK_NOT_DETECTED = "landmark_not_detected"
    # LandmarkGateway: the id is not in the detector's registry. A 400.
    LANDMARK_UNKNOWN = "unknown_landmark"


class FailureMessage(str):
    """Operator-facing prose carrying an optional machine-readable ``code``.

    Only the object carries the code: an f-string built from it is a plain
    ``str``. Read the code first, then reword freely.
    """

    def __new__(cls, message: str, code: Failure) -> "FailureMessage":
        obj = super().__new__(cls, message)
        obj.code = code
        return obj


def fail(code: Failure, message: str) -> FailureMessage:
    """Tag ``message`` with ``code``; it reads as prose everywhere else."""
    return FailureMessage(message, code)


def failure_code(message: str) -> Optional[Failure]:
    """The code ``message`` was tagged with, or ``None`` if it carries none."""
    return getattr(message, "code", None)
