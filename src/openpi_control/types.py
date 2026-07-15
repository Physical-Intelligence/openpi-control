"""Immutable, unit-explicit public state and command types."""

from __future__ import annotations

import time
from collections.abc import Iterable
from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np
from numpy.typing import NDArray

from .exceptions import ConfigurationError

FloatArray = NDArray[np.float64]


def readonly_array(values: Iterable[float] | NDArray[np.floating], *, name: str) -> FloatArray:
    array = np.asarray(values, dtype=np.float64).copy()
    if array.ndim != 1:
        raise ConfigurationError(f"{name} must be a one-dimensional array")
    if not np.all(np.isfinite(array)):
        raise ConfigurationError(f"{name} contains non-finite values")
    array.setflags(write=False)
    return array


class ArmRole(StrEnum):
    LEADER = "leader"
    FOLLOWER = "follower"

    @property
    def control_frequency_hz(self) -> int:
        """Canonical control-loop rate for this role."""
        return 200


class ArmMode(StrEnum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    HOLD = "hold"
    DIRECT = "direct"
    GRAVITY_COMPENSATION = "gravity_compensation"
    BILATERAL = "bilateral"
    MOVE_TO_READY = "move_to_ready"
    RECOVERY = "recovery"
    CLOSED = "closed"


@dataclass(frozen=True, slots=True)
class JointState:
    """Joint state. Position is rad, velocity rad/s, effort Nm, current A.

    ``frame_age_ms`` is the age of the hardware feedback frame backing each
    joint's position when the native node published it (-1 = unknown or the
    backend does not track per-joint freshness). It exposes a stale driver
    cache that would otherwise be invisible: the node keeps republishing the
    last cached position when CAN frames stop arriving.
    """

    names: tuple[str, ...]
    position_rad: FloatArray
    velocity_rad_s: FloatArray
    effort_nm: FloatArray
    temperature_c: FloatArray
    current_a: FloatArray
    frame_age_ms: FloatArray | None = None

    def __post_init__(self) -> None:
        fields = ("position_rad", "velocity_rad_s", "effort_nm", "temperature_c", "current_a")
        for name in fields:
            object.__setattr__(self, name, readonly_array(getattr(self, name), name=name))
        expected = len(self.names)
        if expected == 0:
            raise ConfigurationError("joint state must contain at least one joint")
        if self.frame_age_ms is None:
            object.__setattr__(
                self, "frame_age_ms", readonly_array([-1.0] * expected, name="frame_age_ms")
            )
        else:
            object.__setattr__(
                self, "frame_age_ms", readonly_array(self.frame_age_ms, name="frame_age_ms")
            )
        if any(getattr(self, name).size != expected for name in (*fields, "frame_age_ms")):
            raise ConfigurationError("all joint state arrays must match the joint name count")


@dataclass(frozen=True, slots=True)
class EffectorState:
    """Effector state. Position is normalized to [0, 1].

    ``frame_age_ms`` mirrors :attr:`JointState.frame_age_ms` for the gripper
    servo (-1 = unknown).
    """

    position: float
    velocity_s: float = 0.0
    effort_nm: float = 0.0
    temperature_c: float = 0.0
    current_a: float = 0.0
    frame_age_ms: float = -1.0
    connected: bool = True
    activated: bool = True
    faulted: bool = False

    def __post_init__(self) -> None:
        if not 0.0 <= self.position <= 1.0:
            raise ConfigurationError("effector position must be normalized to [0, 1]")


@dataclass(frozen=True, slots=True)
class InputState:
    """Operator inputs on a leader handle: named buttons and analog axes.

    Buttons are booleans. Axes are normalized floats: sticks span [-1, 1] and
    triggers [0, 1]. Inputs are a stream separate from joint state and carry
    their own sequence and timestamps.
    """

    button_names: tuple[str, ...]
    buttons: tuple[bool, ...]
    axis_names: tuple[str, ...]
    axes: FloatArray
    monotonic_timestamp: float
    wall_timestamp: float
    sequence: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "buttons", tuple(bool(value) for value in self.buttons))
        object.__setattr__(self, "axes", readonly_array(self.axes, name="axes"))
        if not self.button_names and not self.axis_names:
            raise ConfigurationError("input state must contain at least one button or axis")
        if len(self.buttons) != len(self.button_names):
            raise ConfigurationError("button values must match the button name count")
        if self.axes.size != len(self.axis_names):
            raise ConfigurationError("axis values must match the axis name count")

    def button(self, name: str) -> bool:
        if name not in self.button_names:
            raise ConfigurationError(
                f"unknown button {name!r}; available: {', '.join(self.button_names) or 'none'}"
            )
        return self.buttons[self.button_names.index(name)]

    def axis(self, name: str) -> float:
        if name not in self.axis_names:
            raise ConfigurationError(
                f"unknown axis {name!r}; available: {', '.join(self.axis_names) or 'none'}"
            )
        return float(self.axes[self.axis_names.index(name)])

    @property
    def age_s(self) -> float:
        return max(0.0, time.monotonic() - self.monotonic_timestamp)

    def is_fresh(self, max_age_s: float) -> bool:
        return self.age_s <= max_age_s


@dataclass(frozen=True, slots=True)
class ArmState:
    name: str
    role: ArmRole
    joints: JointState
    effector: EffectorState | None
    monotonic_timestamp: float
    wall_timestamp: float
    sequence: int
    mode: ArmMode
    diagnostics: ArmDiagnostics | None = None

    @property
    def age_s(self) -> float:
        return max(0.0, time.monotonic() - self.monotonic_timestamp)

    def is_fresh(self, max_age_s: float) -> bool:
        return self.age_s <= max_age_s


def _validate_effector_command(
    effector: float | None, effector_speed: float | None, effector_force: float | None
) -> None:
    for name, value in (
        ("effector", effector),
        ("effector_speed", effector_speed),
        ("effector_force", effector_force),
    ):
        if value is not None and not 0.0 <= value <= 1.0:
            raise ConfigurationError(f"{name} must be normalized to [0, 1]")


@dataclass(frozen=True, slots=True)
class PositionCommand:
    """A direct target with optional normalized gripper controls.

    Effector position uses 0 = closed and 1 = open. Speed and force are in
    [0, 1], where 1 requests the configured maximum.
    """

    position_rad: FloatArray
    effector: float | None = None
    created_monotonic: float = field(default_factory=time.monotonic)
    effector_speed: float | None = None
    effector_force: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "position_rad", readonly_array(self.position_rad, name="position_rad")
        )
        if self.position_rad.size == 0:
            raise ConfigurationError("position command must contain at least one joint")
        _validate_effector_command(self.effector, self.effector_speed, self.effector_force)


@dataclass(frozen=True, slots=True)
class VelocityCommand:
    """A direct joint-velocity target with optional normalized gripper controls.

    Joint velocity is expressed in rad/s. The Franka controller integrates the
    target at its native 1 kHz rate using the Polymetis JointVelocityControl
    law. Effector fields have the same meaning as :class:`PositionCommand`.
    """

    velocity_rad_s: FloatArray
    effector: float | None = None
    created_monotonic: float = field(default_factory=time.monotonic)
    effector_speed: float | None = None
    effector_force: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "velocity_rad_s", readonly_array(self.velocity_rad_s, name="velocity_rad_s")
        )
        if self.velocity_rad_s.size == 0:
            raise ConfigurationError("velocity command must contain at least one joint")
        _validate_effector_command(self.effector, self.effector_speed, self.effector_force)


@dataclass(frozen=True, slots=True)
class ArmDiagnostics:
    """Optional high-rate hardware diagnostics kept separate from joint state."""

    external_joint_torque_nm: FloatArray
    cartesian_wrench: FloatArray
    commanded_position_rad: FloatArray
    joint_contact: tuple[bool, ...]
    joint_collision: tuple[bool, ...]
    robot_mode: int
    control_command_success_rate: float
    hardware_timestamp_s: float
    effector_target: float | None = None
    hardware_faulted: bool = False

    def __post_init__(self) -> None:
        for name in ("external_joint_torque_nm", "cartesian_wrench", "commanded_position_rad"):
            object.__setattr__(self, name, readonly_array(getattr(self, name), name=name))
        if self.external_joint_torque_nm.size != 7 or self.commanded_position_rad.size != 7:
            raise ConfigurationError("Franka joint diagnostics must contain seven values")
        if self.cartesian_wrench.size != 6:
            raise ConfigurationError("Cartesian wrench must contain six values")
        if len(self.joint_contact) != 7 or len(self.joint_collision) != 7:
            raise ConfigurationError("Franka contact diagnostics must contain seven values")
        if not 0.0 <= self.control_command_success_rate <= 1.0:
            raise ConfigurationError("control command success rate must be in [0, 1]")
        if self.effector_target is not None and not 0.0 <= self.effector_target <= 1.0:
            raise ConfigurationError("effector target must be normalized to [0, 1]")


@dataclass(frozen=True, slots=True)
class JointServoReport:
    """One joint's servo parameters as the native node reports them.

    The codec ranges scale every wire command/status; the reported ranges come
    from the motor's own firmware registers (ENCOS range query) and expose
    batch differences (e.g. TOR registers of 30 vs 42 Nm) that make one
    torq_rescale calibration invalid for another arm. Reported ranges are None
    for motor families without a range query.

    torq_rescale, pos_kp and pos_kd are the values the node actually applies
    (after every config layer and runtime update), so consumers such as the
    rollout dump can record the effective calibration without re-deriving it
    from config files. gravity_feed_forward is device-level (the follower
    gravity feed-forward on/off state) and repeats on every joint's report.
    """

    codec_vel_range: tuple[float, float]
    codec_tor_range: tuple[float, float]
    reported_spd_range: tuple[float, float] | None
    reported_tor_range: tuple[float, float] | None
    torq_rescale: float
    pos_kp: float
    pos_kd: float
    gravity_feed_forward: bool


@dataclass(frozen=True, slots=True)
class ArmCapabilities:
    protocol_version: tuple[int, int]
    model: str
    joint_names: tuple[str, ...]
    has_effector: bool
    supports_direct_commands: bool
    supports_live_input: bool
    supports_gravity_compensation: bool
    supports_force_feedback: bool
    supports_move_to_ready: bool
    button_names: tuple[str, ...] = ()
    axis_names: tuple[str, ...] = ()
    max_bilateral_gain: float = 0.3
    supports_effector_activation: bool = False
    supports_recovery: bool = False
    supports_effector_force: bool = False
    supports_velocity_commands: bool = False

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    @property
    def has_inputs(self) -> bool:
        return bool(self.button_names or self.axis_names)
