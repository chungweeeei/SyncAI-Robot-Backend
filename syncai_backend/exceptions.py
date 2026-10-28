class NotFoundError(Exception):
    pass


class UnauthorizedError(Exception):
    pass


class BadRequestError(Exception):
    """A request that cannot be carried out as written (HTTP 400).

    Optional ``code`` and ``step_id`` ride next to ``detail``. ``step_id``
    names the task-body step the refusal is about, so the console can mark
    that row.
    """

    def __init__(
        self,
        message: str,
        code: "str | None" = None,
        step_id: "str | None" = None,
    ):
        super().__init__(message)
        self.code = code
        self.step_id = step_id


class ConflictError(Exception):
    """A request that contradicts current state (HTTP 409).

    ``code`` is an optional machine-readable discriminator for callers that
    must tell two 409s on one route apart (e.g. activate: "no 3D map" vs
    "stack not ready"), so the UI never matches on prose.
    """

    def __init__(self, message: str, code: "str | None" = None):
        super().__init__(message)
        self.code = code


class PayloadTooLargeError(Exception):
    """A request body over the route's limit (HTTP 413), refused before or
    while it is read, so the limit bounds the disk an upload can take."""

    def __init__(self, message: str, code: "str | None" = None):
        super().__init__(message)
        self.code = code


# 502: a downstream dependency (Temporal, a ROS service) failed.
class UpstreamError(Exception):
    pass
