"""Machine-readable discriminators for gateway failures.

A gateway reports trouble as ``(False, message, payload)`` where the message is
prose written for an operator. A few of those failures mean something specific
to the caller: the REST layer answers 400 or 409 instead of its uniform 502, and
the Temporal activity marks the attempt non-retryable instead of retrying it.
Keying that on the prose couples the caller to a sentence that exists to be
reworded, and it scattered the rule: ``unknown voice`` was matched once in the
tts router and again, independently, in the SPEAK activity, so the two could
disagree about the same failure after one reword.

``FailureMessage`` is a ``str`` subclass, so every caller, log line, f-string and
test that treats a gateway message as prose keeps working untouched; ``code``
rides beside it exactly the way ``ConflictError.code`` rides beside its detail,
and for the same stated reason. Read it with :func:`failure_code`, which answers
``None`` for the ordinary untagged failures -- the ones no caller discriminates
on, which is most of them.

Deliberately not every failure. A code is added when some caller acts on the
difference; a code nobody reads is a second contract to keep honest. The webrtc
gateway's camera-busy case is the standing example of the judgement call: it is
indistinguishable from any other pipeline failure in the Go error string, and a
code that guesses is worse than prose that does not.
"""

from enum import Enum
from typing import Optional


class Failure(str, Enum):
    """The gateway failures that some caller answers differently.

    The values are the wire strings the REST layer already publishes as
    ``ConflictError.code``, so naming them here changed no response body.
    """

    # TtsGateway: the one failure that is the caller's to fix (400, and
    # non-retryable for a SPEAK step) rather than the robot's (502).
    #
    # This string is also the one the syncai_tts service publishes, and the two
    # are deliberately identical: the gateway reads `code` off that service's
    # error body and re-tags it with this enum, so renaming either side silently
    # turns a 400 into a 502 and makes a typo'd voice retry three times.
    UNKNOWN_VOICE = "unknown_voice"

    # TtsGateway: more utterances are already waiting than the speech service's
    # queue allows. A 409 the console offers "wait or cancel" for, rather than
    # the 502 that would tell an operator the robot is broken when in fact they
    # pressed Speak nine times. New with the move to the service — while the
    # speaker was a lock inside this process, a ninth caller simply blocked.
    TTS_QUEUE_FULL = "tts_queue_full"

    # RecordingGateway: both are 409s the console offers a different next step
    # for -- stop the running bag, or free some disk.
    RECORDING_RUNNING = "recording_running"
    DISK_LOW = "disk_low"

    # WebRtcGateway: a create is mid-flight and has no session id yet, so it
    # cannot be preempted. The caller retries; everything else is a 502.
    WHEP_SESSION_PENDING = "whep_session_pending"


class FailureMessage(str):
    """Operator-facing prose carrying an optional machine-readable ``code``.

    Note that only the object itself carries the code: an f-string built from it
    is a plain ``str`` again. That is the intended direction of travel -- read
    the code first, then reword freely.
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
