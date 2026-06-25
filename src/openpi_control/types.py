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
    """Joint state. Position is rad, velocity rad/s, effort Nm, current A."""

    names: tuple[str, ...]
    position_rad: FloatArray
    velocity_rad_s: FloatArray
    effort_nm: FloatArray
    temperature_c: FloatArray
    current_a: FloatArray

    def __post_init__(self) -> None:
        fields = ("position_rad", "velocity_rad_s", "effort_nm", "temperature_c", "current_a")
        for name in fields:
            object.__setattr__(self, name, readonly_array(getattr(self, name), name=name))
        expected = len(self.names)
        if expected == 0:
            raise ConfigurationError("joint state must contain at least one joint")
        if any(getattr(self, name).size != expected for name in fields):
            raise ConfigurationError("all joint state arrays must match the joint name count")


@dataclass(frozen=True, slots=True)
class EffectorState:
    """Effector state. Position is normalized to [0, 1]."""

    position: float
    velocity_s: float = 0.0
    effort_nm: float = 0.0
    temperature_c: float = 0.0
    current_a: float = 0.0

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

    @property
    def age_s(self) -> float:
        return max(0.0, time.monotonic() - self.monotonic_timestamp)

    def is_fresh(self, max_age_s: float) -> bool:
        return self.age_s <= max_age_s


@dataclass(frozen=True, slots=True)
class PositionCommand:
    """A direct target: joint positions in rad and optional normalized effector."""

    position_rad: FloatArray
    effector: float | None = None
    created_monotonic: float = field(default_factory=time.monotonic)

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "position_rad", readonly_array(self.position_rad, name="position_rad")
        )
        if self.position_rad.size == 0:
            raise ConfigurationError("position command must contain at least one joint")
        if self.effector is not None and not 0.0 <= self.effector <= 1.0:
            raise ConfigurationError("effector command must be normalized to [0, 1]")


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

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    @property
    def has_inputs(self) -> bool:
        return bool(self.button_names or self.axis_names)
