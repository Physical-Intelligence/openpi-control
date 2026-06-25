"""Typed public exceptions."""


class PiControlError(Exception):
    """Base class for package errors."""


class ConfigurationError(PiControlError, ValueError):
    """A configuration or command is invalid."""


class ConnectionUnavailableError(PiControlError):
    """A requested hardware connection is not ready."""


class NativeProcessError(PiControlError):
    """The owned native process failed."""


class HardwareFaultError(NativeProcessError):
    """The native runtime stopped because hardware reported an actionable fault."""


class ProtocolError(PiControlError):
    """The native process uses an incompatible or malformed protocol."""


class StateTimeoutError(PiControlError, TimeoutError):
    """No fresh state arrived before the deadline."""


class StaleStateError(PiControlError):
    """A state is too old to use safely."""


class CommandRejectedError(PiControlError):
    """The native runtime rejected a command."""


class RoleError(PiControlError):
    """An operation is unavailable for this arm role."""


class AlignmentError(PiControlError):
    """Follower alignment could not complete safely."""
