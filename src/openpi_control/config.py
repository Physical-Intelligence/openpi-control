"""Connections, safety limits, logical identity, and packaged model resolution."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from importlib.resources import files
from pathlib import Path

from .exceptions import ConfigurationError

_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_.-]*$")

# Native control-loop rates by role, matching robot-test's teleoperation stack:
# leaders sample and publish live commands at 50 Hz; followers track at 200 Hz.
LEADER_CONTROL_FREQUENCY_HZ = 50
FOLLOWER_CONTROL_FREQUENCY_HZ = 200

SUPPORTED_MODELS = (
    "ARX_ENC",
    "ARX_L5",
    "ARX_X5",
    "Yam",
)
SUPPORTED_EFFECTORS = ("E_ARX", "E_ARX_ENC", "E_Yam", "E_Yam_Handle", "E_Yam_Handle_compat")


@dataclass(frozen=True, slots=True)
class SocketCanConnection:
    """An already configured SocketCAN interface."""

    interface: str

    def __post_init__(self) -> None:
        if not self.interface or "/" in self.interface:
            raise ConfigurationError("SocketCAN interface must be a simple interface name")


@dataclass(frozen=True, slots=True)
class SafetyLimits:
    max_bilateral_gain: float = 0.3
    max_joint_velocity_rad_s: float = 0.3
    max_effector_velocity_s: float = 0.5
    minimum_alignment_duration_s: float = 1.0
    max_alignment_duration_s: float = 30.0
    max_alignment_error_rad: float = 0.05
    max_effector_alignment_error: float = 0.05
    # Maximum plausible change between consecutive leader samples during alignment.
    max_leader_drift_rad: float = 0.05
    max_state_age_s: float = 0.25

    def __post_init__(self) -> None:
        for field_name in self.__dataclass_fields__:
            if getattr(self, field_name) <= 0:
                raise ConfigurationError(f"{field_name} must be positive")


@dataclass(frozen=True, slots=True)
class InputLayout:
    """Buttons and analog axes exposed by an arm's operator handle."""

    button_names: tuple[str, ...] = ()
    axis_names: tuple[str, ...] = ()

    @property
    def has_inputs(self) -> bool:
        return bool(self.button_names or self.axis_names)


# Name order follows the native MsgJoystick channel/button layout per handle
# (see native/pi_control/include/pi_topic.hpp).
_HANDLE_INPUT_LAYOUTS = {
    # I2RT YAM teaching handle: two buttons, no joystick.
    "E_Yam_Handle": InputLayout(button_names=("top", "bottom")),
    # Same physical handle; config variant that tolerates older handle firmware.
    "E_Yam_Handle_compat": InputLayout(button_names=("top", "bottom")),
}


@dataclass(frozen=True, slots=True)
class ResolvedArmAssets:
    model_config: Path
    instance_config: Path
    urdf: Path
    effector_model_config: Path | None
    effector_instance_config: Path | None


def resolve_model_assets(
    model: str,
    *,
    effector_model: str | None = None,
    instance_config: Path | None = None,
    effector_instance_config: Path | None = None,
    urdf: Path | None = None,
) -> ResolvedArmAssets:
    """Resolve packaged model files without constructing a hardware connection."""
    if model not in SUPPORTED_MODELS:
        raise ConfigurationError(
            f"unsupported model {model!r}; supported models: {', '.join(SUPPORTED_MODELS)}"
        )
    if effector_model is not None and effector_model not in SUPPORTED_EFFECTORS:
        raise ConfigurationError(
            f"unsupported effector {effector_model!r}; supported effectors: "
            f"{', '.join(SUPPORTED_EFFECTORS)}"
        )
    root = Path(str(files("openpi_control").joinpath("models")))
    arm_dir = root / "arms" / model
    model_config = arm_dir / f"{model}.json"
    instance = instance_config or arm_dir / f"{model}_01.json"
    resolved_urdf = urdf or arm_dir / f"{model}.urdf"
    eff_model: Path | None = None
    eff_instance: Path | None = None
    if effector_model:
        eff_dir = root / "effectors" / effector_model
        eff_model = eff_dir / f"{effector_model}.json"
        eff_instance = effector_instance_config or eff_dir / f"{effector_model}_01.json"
    required = [model_config, Path(instance), Path(resolved_urdf)]
    if eff_model is not None and eff_instance is not None:
        required.extend([eff_model, Path(eff_instance)])
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise ConfigurationError("missing model assets: " + ", ".join(missing))
    return ResolvedArmAssets(
        model_config=model_config,
        instance_config=Path(instance),
        urdf=Path(resolved_urdf),
        effector_model_config=eff_model,
        effector_instance_config=Path(eff_instance) if eff_instance else None,
    )


@dataclass(frozen=True, slots=True)
class ArmConfig:
    """Logical identity plus physical model/calibration selection."""

    name: str
    model: str
    connection: SocketCanConnection
    instance_config: Path | None = None
    effector_model: str | None = None
    effector_instance_config: Path | None = None
    urdf: Path | None = None
    control_frequency_hz: int = 100
    # First contact after the arm has sat idle can exceed a minute of native
    # device init (observed on physical YAM followers), so the default leaves
    # cold starts room to finish.
    connect_timeout_s: float = 120.0
    # Followers only: use synchronized velocity-limited position tracking with
    # model-based gravity feedforward and configured motor-side damping (native
    # slew_pos_gravity planning on the arm device; the attached effector keeps
    # its own planner). Requires the arm model to ship a gravity algo.
    follower_gravity_compensation: bool = False
    # Leaders only: model-based gravity feedforward torque. Defaults on for
    # YAM, off for every other model unless explicitly enabled by the caller.
    leader_gravity_compensation: bool | None = None
    # Opt in to the legacy sustained measured-torque protective stop. When
    # false, over-torque still produces an actionable warning but does not
    # initiate an automatic move-to-ready recovery.
    safety_torque_mode: bool = False

    def __post_init__(self) -> None:
        if not _NAME_RE.fullmatch(self.name):
            raise ConfigurationError(
                "arm name must start with a letter and contain letters, digits, '.', '_', or '-'"
            )
        if self.model not in SUPPORTED_MODELS:
            raise ConfigurationError(
                f"unsupported model {self.model!r}; supported models: {', '.join(SUPPORTED_MODELS)}"
            )
        if self.effector_model is not None and self.effector_model not in SUPPORTED_EFFECTORS:
            raise ConfigurationError(
                f"unsupported effector {self.effector_model!r}; supported effectors: "
                f"{', '.join(SUPPORTED_EFFECTORS)}"
            )
        if self.control_frequency_hz <= 0:
            raise ConfigurationError("control_frequency_hz must be positive")
        if self.connect_timeout_s <= 0:
            raise ConfigurationError("connect_timeout_s must be positive")
        if self.leader_gravity_compensation is None:
            object.__setattr__(self, "leader_gravity_compensation", self.model == "Yam")
        elif not isinstance(self.leader_gravity_compensation, bool):
            raise ConfigurationError("leader_gravity_compensation must be a boolean")
        if not isinstance(self.safety_torque_mode, bool):
            raise ConfigurationError("safety_torque_mode must be a boolean")
        for field_name in ("instance_config", "effector_instance_config", "urdf"):
            value = getattr(self, field_name)
            if value is not None:
                object.__setattr__(self, field_name, Path(value).expanduser().resolve())

    def resolve_assets(self) -> ResolvedArmAssets:
        return resolve_model_assets(
            self.model,
            effector_model=self.effector_model,
            instance_config=self.instance_config,
            effector_instance_config=self.effector_instance_config,
            urdf=self.urdf,
        )

    def input_layout(self) -> InputLayout:
        """Operator inputs published by this arm's handle, if any."""
        assets = self.resolve_assets()
        if assets.effector_model_config is None:
            return InputLayout()
        data = json.loads(assets.effector_model_config.read_text())
        if not data.get("publishes_joystick"):
            return InputLayout()
        known = _HANDLE_INPUT_LAYOUTS.get(self.effector_model or "")
        if known is not None:
            return known
        for joint in data.get("joints", []):
            for servo in joint.get("servos", []):
                buttons = servo.get("joystick_button_num")
                axes = servo.get("joystick_channel_num")
                if buttons is not None or axes is not None:
                    return InputLayout(
                        button_names=tuple(f"button_{i}" for i in range(int(buttons or 0))),
                        axis_names=tuple(f"axis_{i}" for i in range(int(axes or 0))),
                    )
        raise ConfigurationError(
            f"effector {self.effector_model!r} declares operator inputs "
            "but has no known input layout"
        )

    def joint_names(self) -> tuple[str, ...]:
        data = json.loads(self.resolve_assets().model_config.read_text())
        names: list[str] = []
        for index, joint in enumerate(data.get("joints", [])):
            names.append(str(joint.get("joint_name", f"joint_{index + 1}")))
        if not names:
            raise ConfigurationError(f"model {self.model!r} has no joints")
        return tuple(names)

    def is_read_only(self) -> bool:
        """True when the model declares read_only (leader-only, no actuation, e.g. ARX_ENC)."""
        data = json.loads(self.resolve_assets().model_config.read_text())
        return bool(data.get("read_only", False))
