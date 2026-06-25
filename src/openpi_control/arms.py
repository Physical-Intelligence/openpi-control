"""Role-specific public arm handles."""

from __future__ import annotations

from threading import RLock

from .backend import ArmBackend
from .config import ArmConfig
from .exceptions import CommandRejectedError, ConfigurationError
from .native import NativeArmBackend
from .protocol import ArmTopics
from .types import ArmCapabilities, ArmMode, ArmRole, ArmState, InputState, PositionCommand


class _Arm:
    def __init__(
        self,
        config: ArmConfig,
        role: ArmRole,
        topics: ArmTopics,
        backend: ArmBackend | None = None,
    ) -> None:
        self.config = config
        self.role = role
        self.topics = topics
        self._backend = backend or NativeArmBackend()
        self._backend.configure_pair(follower_state_topic="")
        self._capabilities: ArmCapabilities | None = None
        self._lifecycle_lock = RLock()
        self._lock = RLock()
        self._dispatch_lock = RLock()
        self._connection_generation = 0

    @property
    def name(self) -> str:
        return self.config.name

    @property
    def capabilities(self) -> ArmCapabilities:
        if self._capabilities is None:
            raise ConfigurationError(f"arm {self.name!r} is not connected")
        return self._capabilities

    @property
    def mode(self) -> ArmMode:
        state = self.latest_state
        return state.mode if state else ArmMode.DISCONNECTED

    @property
    def latest_state(self) -> ArmState | None:
        return self._backend.latest_state()

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._capabilities is not None and self._backend._is_connected()

    def connect(self) -> None:
        with self._lifecycle_lock:
            with self._lock:
                with self._dispatch_lock:
                    if not self.connected:
                        self._capabilities = None
                        try:
                            self._capabilities = self._backend.connect(
                                self.config, self.role, self.topics
                            )
                            self._connection_generation += 1
                        except Exception:
                            try:
                                self._backend.close()
                            except Exception:
                                # Best-effort cleanup must not replace the connect failure.
                                pass
                            raise

    def read_state(self, timeout_s: float | None = 1.0) -> ArmState:
        return self._backend.read_state(timeout_s)

    def close(self, *, move_to_ready: bool = False) -> None:
        with self._lifecycle_lock:
            with self._lock:
                with self._dispatch_lock:
                    try:
                        self._backend.close(move_to_ready=move_to_ready)
                    finally:
                        self._capabilities = None
                        self._connection_generation += 1

    def _capture_connection_generation(self) -> int:
        with self._dispatch_lock:
            if self._capabilities is None or not self._backend._is_connected():
                raise CommandRejectedError(f"arm {self.name!r} is not connected")
            return self._connection_generation

    def _assert_connection_generation(self, expected: int, *, operation: str) -> None:
        with self._dispatch_lock:
            if (
                self._connection_generation != expected
                or self._capabilities is None
                or not self._backend._is_connected()
            ):
                raise CommandRejectedError(f"{self.name} connection changed during {operation}")

    def __enter__(self) -> _Arm:
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class FollowerArm(_Arm):
    def __init__(
        self, config: ArmConfig, topics: ArmTopics, backend: ArmBackend | None = None
    ) -> None:
        super().__init__(config, ArmRole.FOLLOWER, topics, backend)

    def command(self, command: PositionCommand) -> None:
        with self._dispatch_lock:
            self._validate_command(command)
            self._backend.command(command)

    def _validate_command(self, command: PositionCommand) -> None:
        if command.position_rad.size != self.capabilities.dof:
            raise CommandRejectedError(
                f"command DOF {command.position_rad.size} does not match {self.name} DOF "
                f"{self.capabilities.dof}"
            )
        if command.effector is not None and not self.capabilities.has_effector:
            raise CommandRejectedError(f"{self.name} has no configured effector")

    def hold(self) -> None:
        with self._dispatch_lock:
            self._backend.hold()

    def move_to_ready(self) -> None:
        with self._dispatch_lock:
            self._backend.move_to_ready()

    def _hold_for_cleanup(self) -> None:
        self._backend.hold()

    def _pause_live_input(self, paused: bool) -> None:
        self._backend.pause_live_input(paused)


class LeaderArm(_Arm):
    def __init__(
        self, config: ArmConfig, topics: ArmTopics, backend: ArmBackend | None = None
    ) -> None:
        super().__init__(config, ArmRole.LEADER, topics, backend)

    @property
    def latest_inputs(self) -> InputState | None:
        """Most recent operator inputs, or None before the first sample arrives."""
        return self._backend.latest_inputs()

    def read_inputs(self, timeout_s: float | None = 1.0) -> InputState:
        return self._backend.read_inputs(timeout_s)

    def enter_gravity_compensation(self) -> None:
        with self._dispatch_lock:
            self._backend.set_mode(ArmMode.GRAVITY_COMPENSATION)

    def _enable_force_feedback(self, gain: float) -> None:
        with self._dispatch_lock:
            self._backend.set_force_feedback_gain(gain)
            self._backend.set_mode(ArmMode.BILATERAL)

    def _set_force_feedback_gain(self, gain: float) -> None:
        with self._dispatch_lock:
            self._backend.set_force_feedback_gain(gain)

    def _configure_pair(self, follower: FollowerArm) -> None:
        self._backend.configure_pair(follower_state_topic=follower.topics.state)
