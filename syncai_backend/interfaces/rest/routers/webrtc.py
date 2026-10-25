import structlog
from typing import Optional
from fastapi import APIRouter, Body, Response
from pydantic import BaseModel, Field

from syncai_backend.exceptions import BadRequestError, ConflictError, UpstreamError
from syncai_backend.gateways.failure import Failure, failure_code
from syncai_backend.gateways.webrtc.webrtc import WebRtcGateway


# A boundary guard (an offer is a few KB): a stray upload must not reach cgo.
_MAX_SDP_BYTES = 64 * 1024


class SessionStatus(BaseModel):
    session_id: str = Field(..., description="The worker's id for the live session.")
    age_seconds: float = Field(
        ..., description="Seconds since this backend created the session."
    )


class WebRtcStatusResponse(BaseModel):
    library_loaded: bool = Field(
        ...,
        description="Whether the Go worker library has been loaded yet. False on a "
        "boot that has never served a WHEP request.",
    )
    video: Optional[SessionStatus] = Field(
        None,
        description="The single WHEP viewer slot, as this backend believes it to be. "
        "There is no callback from the worker, so an abandoned session reads as "
        "active until the next POST preempts it.",
    )
    audio: Optional[SessionStatus] = Field(
        None,
        description="The single WHIP talker slot, with the same caveat as `video`.",
    )
    duplex: Optional[SessionStatus] = Field(
        None,
        description="The bidirectional slot. It holds the camera and the speaker "
        "together, so it is never populated at the same time as `video` or "
        "`audio` — a create of either kind preempts the other.",
    )


# What the three create routes answer, for OpenAPI: no response_model, the
# body is the answer SDP.
_SDP_ANSWER = {
    201: {
        "content": {"application/sdp": {}},
        "description": "The answer SDP. `Location` carries the session URL.",
    }
}


def init_webrtc_router(
    logger: structlog.stdlib.BoundLogger, webrtc_gw: WebRtcGateway
) -> APIRouter:
    webrtc_router = APIRouter(prefix="", tags=["WebRTC"])

    # Plain `def`: creating a session blocks in the worker until ICE gathering
    # completes (0.1-5 s), which would stall the WebSockets on the event loop.
    # Hence a `bytes` body parameter, not the coroutine Request.body().

    def _decode_offer(raw: bytes) -> str:
        """Turn the request body into an SDP string, or refuse it as a 400.

        Only sanity checks: the offer's semantics are pion's to judge.
        """
        if not raw:
            raise BadRequestError(
                "the request body is empty; POST the SDP offer as the body"
            )
        if len(raw) > _MAX_SDP_BYTES:
            raise BadRequestError(f"offer SDP is too large (> {_MAX_SDP_BYTES} bytes)")
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            raise BadRequestError("offer SDP is not valid UTF-8")
        if not text.lstrip().startswith("v=0"):
            raise BadRequestError("offer SDP must start with 'v=0'")
        if "m=" not in text:
            raise BadRequestError("offer SDP carries no media sections")
        return text

    def _created(kind: str, label: str, payload: dict) -> Response:
        """The 201 every create route answers: the answer SDP, and the session's
        URL in ``Location``.

        Relative on purpose: an absolute one built from the request works from
        one network and not another. ``status_code`` here too: the decorator's
        is only what OpenAPI reads.
        """
        session_id = payload["session_id"]
        logger.info(f"{label} session created", session_id=session_id)
        return Response(
            status_code=201,
            content=payload["answer_sdp"],
            media_type="application/sdp",
            headers={"Location": f"/api/v1/webrtc/{kind}/{session_id}"},
        )

    def _raise_for(message: str) -> None:
        # Only a pending session gets a code (409); everything else is 502 with
        # the worker's sentence. Camera-busy gets no code: the Go error cannot
        # tell it from any other pipeline failure.
        if failure_code(message) is Failure.WHEP_SESSION_PENDING:
            raise ConflictError(message, code=Failure.WHEP_SESSION_PENDING.value)
        raise UpstreamError(message)

    # No response_model: the body is the answer SDP, declared via `responses`.
    @webrtc_router.post(
        "/api/v1/webrtc/whep",
        status_code=201,
        responses=_SDP_ANSWER,
        response_class=Response,
    )
    # Defaulted to empty so _decode_offer answers a 400 sentence, not a 422.
    def whep_create(offer: bytes = Body(b"", media_type="application/sdp")):
        offer_sdp = _decode_offer(offer)

        # The worker expects one video and one audio m-line. Logged, not
        # refused, here and in the other routes: the constraint is the
        # worker's, and enforcing it twice means changing it twice.
        if offer_sdp.count("m=video") != 1 or offer_sdp.count("m=audio") != 1:
            logger.warning(
                "WHEP offer is not the expected two-m-line shape",
                video_lines=offer_sdp.count("m=video"),
                audio_lines=offer_sdp.count("m=audio"),
            )

        success, message, payload = webrtc_gw.create_video_session(offer_sdp=offer_sdp)
        if not success:
            logger.error("WHEP session creation failed", message=message)
            _raise_for(message)

        return _created("whep", "WHEP", payload)

    @webrtc_router.delete(
        "/api/v1/webrtc/whep/{session_id}", status_code=204, response_class=Response
    )
    def whep_delete(session_id: str):
        """Tear a WHEP session down.

        Idempotent like the worker: 204 even for an unknown id (sessions
        self-close on connection failure).
        """
        success, message, _ = webrtc_gw.delete_video_session(session_id=session_id)
        if not success:
            logger.error(
                "WHEP session teardown failed", session_id=session_id, message=message
            )
            _raise_for(message)

        return Response(status_code=204)

    @webrtc_router.post(
        "/api/v1/webrtc/whip",
        status_code=201,
        responses=_SDP_ANSWER,
        response_class=Response,
    )
    def whip_create(offer: bytes = Body(b"", media_type="application/sdp")):
        """Talk to the robot: the browser's microphone out of its speaker.

        The mirror image of the WHEP route. One talker at a time (the speaker
        is one ALSA device); a new POST preempts the old session.
        """
        offer_sdp = _decode_offer(offer)

        # The worker receives exactly one audio track; logged, as for WHEP.
        if offer_sdp.count("m=audio") != 1:
            logger.warning(
                "WHIP offer is not the expected single-audio shape",
                audio_lines=offer_sdp.count("m=audio"),
                video_lines=offer_sdp.count("m=video"),
            )

        success, message, payload = webrtc_gw.create_audio_session(offer_sdp=offer_sdp)
        if not success:
            logger.error("WHIP session creation failed", message=message)
            _raise_for(message)

        return _created("whip", "WHIP", payload)

    @webrtc_router.delete(
        "/api/v1/webrtc/whip/{session_id}", status_code=204, response_class=Response
    )
    def whip_delete(session_id: str):
        """Tear a WHIP session down.

        204 even for an id the worker reports unknown: the gateway has already
        dropped the slot, and a page-unload client cannot act on it.
        """
        success, message, _ = webrtc_gw.delete_audio_session(session_id=session_id)
        if not success:
            # Logged, not raised: a 502 would make a page unload look like a fault.
            logger.info(
                "WHIP session teardown reported no such session",
                session_id=session_id,
                message=message,
            )

        return Response(status_code=204)

    @webrtc_router.post(
        "/api/v1/webrtc/duplex",
        status_code=201,
        responses=_SDP_ANSWER,
        response_class=Response,
    )
    def duplex_create(offer: bytes = Body(b"", media_type="application/sdp")):
        """Both directions over one peer connection: camera out, microphone in.

        **Not** WHEP or WHIP, which each specify one direction. One handshake
        and one port pair, and both directions live or die together. It holds
        the camera and the speaker, so it and WHEP/WHIP preempt each other
        (the gateway arbitrates by device).
        """
        offer_sdp = _decode_offer(offer)

        # One video m-line, one sendrecv audio m-line; logged, as for WHEP. A
        # second audio m-line is rejected by pion and shows up as no microphone.
        if offer_sdp.count("m=video") != 1 or offer_sdp.count("m=audio") != 1:
            logger.warning(
                "Duplex offer is not the expected one-video-one-audio shape",
                video_lines=offer_sdp.count("m=video"),
                audio_lines=offer_sdp.count("m=audio"),
            )

        success, message, payload = webrtc_gw.create_duplex_session(offer_sdp=offer_sdp)
        if not success:
            logger.error("Duplex session creation failed", message=message)
            _raise_for(message)

        return _created("duplex", "Duplex", payload)

    @webrtc_router.delete(
        "/api/v1/webrtc/duplex/{session_id}", status_code=204, response_class=Response
    )
    def duplex_delete(session_id: str):
        """Tear a duplex session down, releasing both devices.

        Idempotent like the WHEP teardown.
        """
        success, message, _ = webrtc_gw.delete_duplex_session(session_id=session_id)
        if not success:
            logger.error(
                "Duplex session teardown failed", session_id=session_id, message=message
            )
            _raise_for(message)

        return Response(status_code=204)

    @webrtc_router.get("/api/v1/webrtc/status", response_model=WebRtcStatusResponse)
    def webrtc_status():
        """What this backend believes about the camera and speaker streams.

        The worker exports no session statistics, so this is the only way to
        tell whether someone is watching. Kept out of /health, which is the
        liveness contract.
        """
        return WebRtcStatusResponse(**webrtc_gw.status())

    return webrtc_router
