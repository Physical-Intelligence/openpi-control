"""Internal backend contract for the native controller."""

from __future__ import annotations

from abc import ABC, abstractmethod

from .config import ArmConfig
from .protocol import ArmTopics
from .types import ArmCapabilities, ArmMode, ArmRole, ArmState, InputState, PositionCommand


class ArmBackend(ABC):
    """Internal boundary implemented by the native process backend."""

    def _is_connected(self) -> bool:
        return True

    def configure_pair(self, *, follower_state_topic: str) -> None:
        del follower_state_topic

    @abstractmethod
    def connect(self, config: ArmConfig, role: ArmRole, topics: ArmTopics) -> ArmCapabilities: ...

    @abstractmethod
    def read_state(self, timeout_s: float | None = None) -> ArmState: ...

    @abstractmethod
    def latest_state(self) -> ArmState | None: ...

    @abstractmethod
    def read_inputs(self, timeout_s: float | None = None) -> InputState: ...

    @abstractmethod
    def latest_inputs(self) -> InputState | None: ...

    @abstractmethod
    def command(self, command: PositionCommand, *, live: bool = False) -> None: ...

    @abstractmethod
    def hold(self) -> None: ...

    @abstractmethod
    def pause_live_input(self, paused: bool) -> None: ...

    @abstractmethod
    def set_mode(self, mode: ArmMode) -> None: ...

    @abstractmethod
    def set_force_feedback_gain(self, gain: float) -> None: ...

    @abstractmethod
    def move_to_ready(self) -> None: ...

    @abstractmethod
    def close(self, *, move_to_ready: bool = False) -> None: ...
