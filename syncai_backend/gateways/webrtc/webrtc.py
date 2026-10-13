import ctypes
import json
import os
import threading
import time
from typing import Dict, FrozenSet, NamedTuple, Optional, Tuple

import structlog

from syncai_backend.gateways.failure import Failure, fail


# Built in the SyncAI-WebRTC-Worker repo with
#
#   go build -buildmode=c-shared -o dist/libsyncai_worker.so ./cmd/lib
#
# and copied in by hand (*.so is gitignored); reached through the bind mount.
_DEFAULT_LIB = os.path.expanduser("~/robot_ws/lib/libsyncai_worker.so")

# setdefault'ed immediately before InitWorker, never after: the worker freezes
# its config under a sync.Once. Set here because the worker's godotenv.Load()
# reads .env relative to the process cwd, which is not reliably the workspace
# root. setdefault, so .env (already loaded by main.py) still overrides.
# TURN_* are deliberately absent: an empty default is worse than none.
_WORKER_ENV_DEFAULTS = {
    "VIDEO_RTP_PORT": "5006",
    "VIDEO_AUDIO_RTP_PORT": "5007",
    "AUDIO_RTP_PORT": "5004",
    "STUN_SERVERS": "stun:stun.l.google.com:19302",
}

# The one export nothing works without; per-kind exports may be missing (see
# _bind_exports).
_INIT_EXPORT = "InitWorker"


class _Slot(NamedTuple):
    """The one session of its kind this backend believes it has.

    ``session_id`` is None while a create is in flight: the slot is claimed
    before the multi-second call so two POSTs cannot both own the device.
    """

    session_id: Optional[str]
    created_at: float


# The three session kinds, one slot each: video is WHEP (robot sends), audio is
# WHIP (browser sends), duplex is both. The signalling is otherwise identical.
_VIDEO = "video"
_AUDIO = "audio"
_DUPLEX = "duplex"
_EXPORTS = {
    _VIDEO: ("CreateVideoSession", "DeleteVideoSession"),
    _AUDIO: ("CreateAudioSession", "DeleteAudioSession"),
    _DUPLEX: ("CreateDuplexSession", "DeleteDuplexSession"),
}

# The devices each kind holds; the whole arbitration rule. A create preempts
# every live session whose devices overlap its own (WHEP and WHIP coexist).
# The worker does not arbitrate: a second session on a busy device fails deep
# in GStreamer with an opaque ALSA/V4L2 error.
_CAMERA = "camera"
_SPEAKER = "speaker"
_DEVICES = {
    _VIDEO: frozenset({_CAMERA}),
    _AUDIO: frozenset({_SPEAKER}),
    _DUPLEX: frozenset({_CAMERA, _SPEAKER}),
}


class WebRtcGateway:
    """WHEP and WHIP signalling over the Go c-shared worker.

    The backend owns the HTTP surface; the media path lives in the dlopen'd
    libsyncai_worker.so, which:

      - installs the Go runtime's signal handlers into this rclpy process;
      - cannot be dlclose'd;
      - takes the whole backend down if it faults.

    Hence the lazy load. Pointing SYNCAI_WEBRTC_LIB at a missing path is the
    only way to disable it.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self._logger = logger

        self.lib_path = os.environ.get("SYNCAI_WEBRTC_LIB", _DEFAULT_LIB)

        # Loaded on first use (see the class docstring).
        self._lib = None

        # Session kinds the loaded .so exports; may be a subset, since the
        # hand-copied library can be older than this file.
        self._kinds: FrozenSet[str] = frozenset()

        # Two locks, neither ever held across a call into the worker: creates
        # block for seconds on ICE gathering, and a DELETE or GET /status must
        # not wait behind one. Go guards its own session map; these guard only
        # bookkeeping. Creates are serialised by the slot claim, not a mutex.
        self._load_lock = threading.Lock()
        self._session_lock = threading.Lock()

        self._slots: Dict[str, Optional[_Slot]] = {kind: None for kind in _EXPORTS}

        # The only record of where we looked if the library is missing.
        self._logger.info("[WebRtcGateway] Using worker library", path=self.lib_path)

    # --- Library ------------------------------------------------------------
    def _load_library(self, path: str):
        """dlopen the worker. A seam: tests override this to inject a fake.

        CDLL, not PyDLL: it releases the GIL around foreign calls, so a
        multi-second create does not freeze the event loop.
        """
        return ctypes.CDLL(path)

    @staticmethod
    def _bind_exports(lib) -> FrozenSet[str]:
        """Declare the C signatures. Returns the session kinds this build has.

        restype is c_void_p, NOT c_char_p: exports malloc their envelope, and
        c_char_p would drop the pointer so FreeString could never free it.

        A missing *session* export is tolerated (backend and worker drift), so
        an older .so still serves the kinds it has. A missing InitWorker
        raises.
        """
        init = getattr(lib, _INIT_EXPORT)
        init.argtypes = []
        init.restype = ctypes.c_void_p

        available = set()
        for kind, names in _EXPORTS.items():
            try:
                exports = [getattr(lib, name) for name in names]
            except AttributeError:
                continue
            for export in exports:
                export.argtypes = [ctypes.c_char_p]
                export.restype = ctypes.c_void_p
            available.add(kind)

        lib.FreeString.argtypes = [ctypes.c_void_p]
        lib.FreeString.restype = None

        return frozenset(available)

    def _ensure_loaded(self) -> Tuple[bool, str]:
        """Load and initialize the worker on first call. Caller must hold _load_lock."""
        if self._lib is not None:
            return True, ""

        if not os.path.isfile(self.lib_path):
            return False, (
                f"webrtc worker library missing: {self.lib_path} — build it in "
                "the SyncAI-WebRTC-Worker repo with `go build -buildmode="
                "c-shared -o dist/libsyncai_worker.so ./cmd/lib` and copy the "
                "result there (see gateways/webrtc/webrtc.py)"
            )

        # Before the load, not after: InitWorker's sync.Once freezes these.
        for key, value in _WORKER_ENV_DEFAULTS.items():
            os.environ.setdefault(key, value)

        try:
            lib = self._load_library(self.lib_path)
            kinds = self._bind_exports(lib)
        except OSError as e:
            # Almost always a missing GStreamer runtime (linked through cgo).
            return False, (
                f"failed to load webrtc worker library: {e} — check that the "
                "GStreamer 1.0 runtime is installed in this container"
            )
        except AttributeError as e:
            return False, f"webrtc worker library is missing an export: {e}"

        self._lib = lib
        self._kinds = kinds

        success, message, _ = self._call(_INIT_EXPORT)
        if not success:
            # The .so stays mapped, but dropping the handle lets a later
            # request retry InitWorker.
            self._lib = None
            self._kinds = frozenset()
            return False, f"webrtc worker failed to initialize: {message}"

        self._logger.info(
            "[WebRtcGateway] Worker library loaded and initialized",
            path=self.lib_path,
            kinds=sorted(kinds),
        )
        missing = sorted(set(_EXPORTS) - kinds)
        if missing:
            # Those routes will answer 502; say the library is behind.
            self._logger.warning(
                "[WebRtcGateway] Worker library predates some session kinds",
                missing=missing,
                path=self.lib_path,
            )
        return True, ""

    def _call(self, name: str, arg: Optional[str] = None) -> Tuple[bool, str, dict]:
        """Invoke an export and decode its envelope, always freeing the result.

        The free is in a ``finally`` so a malformed envelope does not leak.
        Caller must NOT hold _session_lock: session exports block for seconds.
        """
        export = getattr(self._lib, name)
        pointer = export(arg.encode("utf-8")) if arg is not None else export()
        if not pointer:
            return False, f"{name} returned NULL — the worker broke its contract", {}

        try:
            raw = ctypes.string_at(pointer)
        finally:
            self._lib.FreeString(pointer)

        try:
            envelope = json.loads(raw.decode("utf-8", errors="replace"))
        except ValueError:
            return False, f"{name} returned non-JSON: {raw[:200]!r}", {}
        if not isinstance(envelope, dict):
            return False, f"{name} returned a non-object envelope", {}
        if not envelope.get("ok"):
            return False, str(envelope.get("error") or f"{name} failed with no reason"), {}

        return True, "", envelope

    # --- API ----------------------------------------------------------------
    def _require_kind(self, kind: str) -> Tuple[bool, str]:
        """Refuse a kind the loaded library does not export.

        Names the repo and build command: the fix is never in this workspace.
        """
        if kind in self._kinds:
            return True, ""
        return False, (
            f"this worker build has no {kind} session support "
            f"({', '.join(_EXPORTS[kind])} not exported) — rebuild the library "
            "in the SyncAI-WebRTC-Worker repo (`docker compose up build-lib`) "
            "and install it per lib/README.md, then restart the backend; the "
            f"kinds this build does support are: {', '.join(sorted(self._kinds)) or 'none'}"
        )

    def _create_session(self, kind: str, offer_sdp: str) -> Tuple[bool, str, dict]:
        """Exchange an offer for an answer. Returns (success, message, payload).

        Blocks until ICE gathering completes (single-shot, no trickle).
        """
        create_export = _EXPORTS[kind][0]

        with self._load_lock:
            success, message = self._ensure_loaded()
        if not success:
            return False, message, {}

        supported, message = self._require_kind(kind)
        if not supported:
            return False, message, {}

        # Every kind sharing a device with this one, itself included; sorted
        # for stable logs.
        contending = sorted(
            other
            for other, devices in _DEVICES.items()
            if devices & _DEVICES[kind]
        )

        with self._session_lock:
            for other in contending:
                slot = self._slots[other]
                if slot is not None and slot.session_id is None:
                    # A create mid-flight has no id to delete, so it cannot
                    # be preempted. Reported under the requested kind.
                    return (
                        False,
                        fail(
                            Failure.WHEP_SESSION_PENDING,
                            f"{kind} session creation already in progress",
                        ),
                        {},
                    )

            previous = [
                (other, self._slots[other].session_id)
                for other in contending
                if self._slots[other] is not None
            ]
            for other, _ in previous:
                self._slots[other] = None
            self._slots[kind] = _Slot(session_id=None, created_at=time.monotonic())

        try:
            for other, session_id in previous:
                # Preempt rather than refuse: an occupied slot is usually a tab
                # closed without DELETE, and nothing else would ever free it.
                # Across kinds it is necessary, or the device would be busy.
                # The result is discarded (audio answers not-found for an
                # unknown id).
                self._logger.info(
                    "[WebRtcGateway] Preempting the session",
                    kind=other,
                    for_kind=kind,
                    session_id=session_id,
                )
                self._call(_EXPORTS[other][1], session_id)

            success, message, envelope = self._call(create_export, offer_sdp)
        except Exception as e:  # noqa: BLE001
            # A ctypes call can raise anything; a slot left claimed would
            # wedge this kind until restart, so always release it.
            with self._session_lock:
                self._slots[kind] = None
            self._logger.error("Session creation raised", kind=kind, exc_info=True)
            return False, f"{kind} session creation raised: {e}", {}

        with self._session_lock:
            if not success:
                self._slots[kind] = None
                return False, message, {}
            self._slots[kind] = _Slot(
                session_id=envelope.get("session_id"), created_at=time.monotonic()
            )

        return True, "", {
            "session_id": envelope.get("session_id"),
            "answer_sdp": envelope.get("answer_sdp", ""),
        }

    def _delete_session(self, kind: str, session_id: str) -> Tuple[bool, str, None]:
        """Tear a session down."""
        _, delete_export = _EXPORTS[kind]

        with self._load_lock:
            success, message = self._ensure_loaded()
        if not success:
            return False, message, None

        supported, message = self._require_kind(kind)
        if not supported:
            return False, message, None

        # Cleared before the call, so a slow or failing delete cannot leave the
        # slot pointing at a dropped session; only on an id match, so a late
        # DELETE from a preempted tab cannot clear the live one.
        with self._session_lock:
            slot = self._slots[kind]
            if slot is not None and slot.session_id == session_id:
                self._slots[kind] = None

        success, message, _ = self._call(delete_export, session_id)
        return success, message, None

    def create_video_session(self, offer_sdp: str) -> Tuple[bool, str, dict]:
        """WHEP: the robot's camera and microphone out to the browser."""
        return self._create_session(_VIDEO, offer_sdp)

    def delete_video_session(self, session_id: str) -> Tuple[bool, str, None]:
        """Idempotent, matching the worker's export."""
        return self._delete_session(_VIDEO, session_id)

    def create_audio_session(self, offer_sdp: str) -> Tuple[bool, str, dict]:
        """WHIP: the browser's microphone in to the robot's speaker."""
        return self._create_session(_AUDIO, offer_sdp)

    def delete_audio_session(self, session_id: str) -> Tuple[bool, str, None]:
        """Unlike the video export, this answers not-found for an unknown id."""
        return self._delete_session(_AUDIO, session_id)

    def create_duplex_session(self, offer_sdp: str) -> Tuple[bool, str, dict]:
        """Both directions over one peer connection.

        Claims camera and speaker together, preempting any WHEP or WHIP session.
        """
        return self._create_session(_DUPLEX, offer_sdp)

    def delete_duplex_session(self, session_id: str) -> Tuple[bool, str, None]:
        """Idempotent, matching the worker's export."""
        return self._delete_session(_DUPLEX, session_id)

    def status(self) -> Dict:
        """What this backend believes about its session slots.

        Belief, not truth: Go has no callback or stats export, so an abandoned
        or self-closed session reads as active until preempted. The fix is a
        SessionStats export in the worker, not a timeout reaper here (it would
        kill long teleop runs).

        Deliberately does not load the library; polling must not pull the Go
        runtime in.
        """
        with self._session_lock:
            slots = dict(self._slots)

        def describe(slot: Optional[_Slot]) -> Optional[dict]:
            if slot is None or slot.session_id is None:
                return None
            return {
                "session_id": slot.session_id,
                "age_seconds": time.monotonic() - slot.created_at,
            }

        return {
            "library_loaded": self._lib is not None,
            "video": describe(slots[_VIDEO]),
            "audio": describe(slots[_AUDIO]),
            "duplex": describe(slots[_DUPLEX]),
        }


def init_webrtc_gateway(logger: structlog.stdlib.BoundLogger) -> WebRtcGateway:
    return WebRtcGateway(logger=logger)
