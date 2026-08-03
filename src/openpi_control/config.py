"""Connections, safety limits, logical identity, and packaged model resolution."""

from __future__ import annotations

import ipaddress
import json
import math
import re
from dataclasses import dataclass
from enum import StrEnum
from importlib.resources import files
from pathlib import Path

from .exceptions import ConfigurationError

_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_.-]*$")

SUPPORTED_MODELS = (
    "ARX_ENC",
    "ARX_L5",
    "ARX_X5",
    "FR3",
    "SO101",
    "Trossen_wai_ctrl",
    "Yam",
)
SUPPORTED_EFFECTORS = (
    "E_ARX",
    "E_ARX_ENC",
    "E_SO101",
    "E_Trossen_ctrl",
    "E_Yam",
    "E_Yam_Handle",
    "E_Yam_Handle_compat",
    "Robotiq",
)


@dataclass(frozen=True, slots=True)
class SocketCanConnection:
    """An already configured SocketCAN interface."""

    interface: str

    def __post_init__(self) -> None:
        if not self.interface or "/" in self.interface:
            raise ConfigurationError("SocketCAN interface must be a simple interface name")


@dataclass(frozen=True, slots=True)
class EthernetConnection:
    """A whole-arm Ethernet controller (e.g. Trossen iNerve) at a fixed IPv4 address."""

    ip: str

    def __post_init__(self) -> None:
        try:
            ipaddress.IPv4Address(self.ip)
        except ValueError as err:
            raise ConfigurationError(
                f"invalid controller IPv4 address {self.ip!r} for Ethernet connection"
            ) from err


@dataclass(frozen=True, slots=True)
class SerialConnection:
    """A USB-serial bus-servo chain (e.g. SO-ARM101 FeeTech) at a tty device path."""

    device: str

    def __post_init__(self) -> None:
        if not self.device.startswith("/dev/"):
            raise ConfigurationError(
                f"serial connection device must be a /dev path, got {self.device!r}"
            )


@dataclass(frozen=True, slots=True)
class FR3Connection:
    """A Franka Emika FR3 controller at a fixed IPv4 address."""

    address: str
    reset_pose_rad: tuple[float, ...] = (
        0.0,
        -0.6283185307,
        0.0,
        -2.5132741229,
        0.0,
        1.8849555922,
        0.0,
    )

    def __post_init__(self) -> None:
        try:
            ipaddress.IPv4Address(self.address)
        except ValueError as err:
            raise ConfigurationError(f"invalid FR3 IPv4 address {self.address!r}") from err
        pose = tuple(float(value) for value in self.reset_pose_rad)
        if len(pose) != 7 or any(not math.isfinite(value) for value in pose):
            raise ConfigurationError("FR3 reset_pose_rad must contain seven finite joint positions")
        object.__setattr__(self, "reset_pose_rad", pose)


class RobotiqTransport(StrEnum):
    RTU = "rtu"
    TCP = "tcp"


@dataclass(frozen=True, slots=True)
class RobotiqConnection:
    """Robotiq 2F gripper using true Modbus RTU or Modbus TCP."""

    transport: RobotiqTransport
    endpoint: str
    port: int = 502
    baud_rate: int = 115200
    slave_id: int = 9
    poll_frequency_hz: int = 50
    timeout_s: float = 0.2
    open_position_raw: int = 3
    closed_position_raw: int = 230
    default_speed: float = 1.0
    default_force: float = 1.0

    @classmethod
    def rtu(cls, device: str, **kwargs: object) -> RobotiqConnection:
        return cls(RobotiqTransport.RTU, device, **kwargs)

    @classmethod
    def tcp(cls, address: str, *, port: int = 502, **kwargs: object) -> RobotiqConnection:
        return cls(RobotiqTransport.TCP, address, port=port, **kwargs)

    def __post_init__(self) -> None:
        try:
            transport = RobotiqTransport(self.transport)
        except ValueError as err:
            raise ConfigurationError("Robotiq transport must be 'rtu' or 'tcp'") from err
        object.__setattr__(self, "transport", transport)
        if transport is RobotiqTransport.RTU:
            if not self.endpoint.startswith("/dev/"):
                raise ConfigurationError("Robotiq RTU endpoint must be a /dev path")
        else:
            try:
                ipaddress.IPv4Address(self.endpoint)
            except ValueError as err:
                raise ConfigurationError(
                    f"invalid Robotiq TCP IPv4 address {self.endpoint!r}"
                ) from err
        if not 1 <= self.port <= 65535:
            raise ConfigurationError("Robotiq TCP port must be between 1 and 65535")
        if self.baud_rate <= 0 or not 0 <= self.slave_id <= 247:
            raise ConfigurationError("Robotiq baud_rate and slave_id are invalid")
        if self.poll_frequency_hz <= 0 or self.timeout_s <= 0:
            raise ConfigurationError("Robotiq polling frequency and timeout must be positive")
        if not 0 <= self.open_position_raw < self.closed_position_raw <= 255:
            raise ConfigurationError("Robotiq raw positions must satisfy 0 <= open < closed <= 255")
        for name in ("default_speed", "default_force"):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ConfigurationError(f"Robotiq {name} must be in [0, 1]")


ArmConnection = SocketCanConnection | EthernetConnection | SerialConnection | FR3Connection


def connection_for_interface(interface: str) -> ArmConnection:
    """Build the arm connection for one bus interface string from a runtime config.

    The runtime config stores the interface part of a bus spec ("can:<iface>",
    "eth:<ip>", or "serial:<device>"). The three forms never overlap: an IPv4
    literal selects a whole-arm Ethernet controller, a /dev path selects a
    USB-serial bus, anything else is a SocketCAN interface name.
    """
    if interface.startswith("/dev/"):
        return SerialConnection(interface)
    try:
        ipaddress.IPv4Address(interface)
    except ValueError:
        return SocketCanConnection(interface)
    return EthernetConnection(interface)


@dataclass(frozen=True, slots=True)
class SafetyLimits:
    max_bilateral_gain: float = 0.3
    max_joint_velocity_rad_s: float = 0.3
    # Effector positions are normalized to [0, 1], so 1.0 permits one full
    # gripper stroke per second during pre-teleop alignment.
    max_effector_velocity_s: float = 1.0
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
    urdf: Path | None
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
    resolved_urdf = urdf or (None if model == "FR3" else arm_dir / f"{model}.urdf")
    eff_model: Path | None = None
    eff_instance: Path | None = None
    if effector_model:
        eff_dir = root / "effectors" / effector_model
        eff_model = eff_dir / f"{effector_model}.json"
        eff_instance = effector_instance_config or eff_dir / f"{effector_model}_01.json"
    required = [model_config, Path(instance)]
    if resolved_urdf is not None:
        required.append(Path(resolved_urdf))
    if eff_model is not None and eff_instance is not None:
        required.extend([eff_model, Path(eff_instance)])
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise ConfigurationError("missing model assets: " + ", ".join(missing))
    return ResolvedArmAssets(
        model_config=model_config,
        instance_config=Path(instance),
        urdf=Path(resolved_urdf) if resolved_urdf is not None else None,
        effector_model_config=eff_model,
        effector_instance_config=Path(eff_instance) if eff_instance else None,
    )


@dataclass(frozen=True, slots=True)
class ArmConfig:
    """Logical identity plus physical model/calibration selection."""

    name: str
    model: str
    connection: ArmConnection
    instance_config: Path | None = None
    effector_model: str | None = None
    effector_connection: RobotiqConnection | None = None
    effector_instance_config: Path | None = None
    urdf: Path | None = None
    # First contact after the arm has sat idle can exceed a minute of native
    # device init (observed on physical YAM followers), so the default leaves
    # cold starts room to finish.
    connect_timeout_s: float = 120.0
    # Followers only: model-based gravity feedforward torque (plus configured
    # motor-side damping) sent with every position command, independent of the
    # planning type. None leaves the decision to the follower_gravity_compensation
    # field of the arm's individual config JSON; an explicit True/False overrides
    # it. MIT CAN arms need a URDF-backed gravity algo (the native node fails
    # fast otherwise); controller arms (Trossen) satisfy the flag via the vendor
    # controller's built-in compensation; serial arms reject an explicit True.
    follower_gravity_compensation: bool | None = None
    # Per-joint gravity-delivery calibration override (one value per arm joint).
    # None keeps the model/instance config values; a sequence is passed to the
    # native node and beats both configs (devices.toml [arms]
    # follower_torq_rescale / leader_torq_rescale, per the arm's role).
    torq_rescale: tuple[float, ...] | None = None
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
        if self.model == "FR3":
            if not isinstance(self.connection, FR3Connection):
                raise ConfigurationError("FR3 requires an FR3Connection")
            if self.effector_model != "Robotiq" or self.effector_connection is None:
                raise ConfigurationError("FR3 requires a Robotiq effector and connection")
        elif isinstance(self.connection, FR3Connection):
            raise ConfigurationError("FR3Connection can only be used with the FR3 model")
        if self.effector_model == "Robotiq" and self.model != "FR3":
            raise ConfigurationError("the Robotiq effector is only supported on FR3")
        if self.effector_connection is not None and self.effector_model != "Robotiq":
            raise ConfigurationError("effector_connection is only valid for a Robotiq effector")
        if self.connect_timeout_s <= 0:
            raise ConfigurationError("connect_timeout_s must be positive")
        if self.leader_gravity_compensation is None:
            object.__setattr__(self, "leader_gravity_compensation", self.model == "Yam")
        elif not isinstance(self.leader_gravity_compensation, bool):
            raise ConfigurationError("leader_gravity_compensation must be a boolean")
        if not isinstance(self.safety_torque_mode, bool):
            raise ConfigurationError("safety_torque_mode must be a boolean")
        if self.torq_rescale is not None:
            values = tuple(float(value) for value in self.torq_rescale)
            if not values or any(not math.isfinite(value) or value < 0.0 for value in values):
                raise ConfigurationError(
                    "torq_rescale must be a non-empty sequence of nonnegative finite floats "
                    "(one value per arm joint)"
                )
            object.__setattr__(self, "torq_rescale", values)
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

    def catalog_baudrate(self) -> int:
        """Bus baud rate declared by the model catalog (required for serial buses)."""
        data = json.loads(self.resolve_assets().model_config.read_text())
        baudrate = data.get("catalog", {}).get("baudrate")
        if not isinstance(baudrate, int) or baudrate <= 0:
            raise ConfigurationError(
                f"model {self.model!r} catalog does not declare a positive integer baudrate"
            )
        return baudrate
