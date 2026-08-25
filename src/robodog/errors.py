"""Exception hierarchy. Everything raised by robodog derives from RobodogError."""


class RobodogError(Exception):
    """Base class for all robodog errors."""


class BackendError(RobodogError):
    """A backend is unusable (not connected, transport failure, ...)."""


class TransportError(BackendError):
    """The robot could not be reached at all -- no answer, not a refusal.

    A subclass rather than a sibling, so every existing `except BackendError`
    keeps working. It exists because the two cases call for opposite advice: a
    robot that answers "500" is telling you something about its firmware, and a
    robot that never answers is telling you something about the network. Saying
    the first when it was the second sends an operator to reflash a robot whose
    only problem is Wi-Fi.
    """


class KinematicsError(RobodogError):
    """A target is outside the reachable workspace of the leg linkage."""


class SafetyError(RobodogError):
    """Base class for safety-supervisor rejections."""


class NotArmedError(SafetyError):
    """Command rejected because the supervisor is not armed."""


class EStopActiveError(SafetyError):
    """Command rejected because the E-stop latch is active; reset() first."""


class LimitViolationError(SafetyError):
    """Command rejected because it exceeds configured workspace/value limits."""


class RateLimitError(SafetyError):
    """Command rejected because it arrived faster than the configured rate."""


class CapabilityError(RobodogError):
    """The selected backend does not support the requested command/routine."""


class RoutineError(RobodogError):
    """A teach-in routine file is invalid."""


class VisionError(RobodogError):
    """A camera stream or a detector is unusable."""


class AiError(RobodogError):
    """The language model could not be reached, or answered outside the vocabulary."""


class BehaviourError(RobodogError):
    """A behaviour was asked for that does not exist, or with impossible parameters."""
