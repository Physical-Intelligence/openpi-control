"""Native ZMQ ABI and session-unique topic naming."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

from .exceptions import ProtocolError

PROTOCOL_VERSION = (1, 1)
BASE_PORT = 8500
PORT_RANGE = 1000
MAX_JOINTS = 10

# Native ZmqJointInfo: pos/vel/tor/temp/idc_current[10] + joint_age_ms[10]
# (frame age in ms, -1 = unknown), then msg_id, joint_num, msg_type,
# measured_idc_current (see native/pi_control/include/pi_topic_zmq.hpp).
JOINT_STRUCT = struct.Struct("@60fihhf")
COMMAND_STRUCT = struct.Struct("@hhh10f10i")
STATUS_STRUCT = struct.Struct("@hhh10f10i")
# Native ZmqJoystickInfo: mode, side, channel[5], channel_num, button[5],
# button_num, raw_channel[5] (see native/pi_control/include/pi_topic_zmq.hpp).
INPUT_STRUCT = struct.Struct("@bb2x5fb5bbx5h2x")
MAX_INPUT_CHANNELS = 5

# DROID messages are explicitly little-endian and packed. They intentionally do
# not reuse JOINT_STRUCT, whose native-alignment ABI is retained for ARX/YAM.
DROID_MAGIC = b"DRD1"
DROID_COMMAND_STRUCT = struct.Struct("<4sBBBBQQ7f7f3f")
DROID_STATE_STRUCT = struct.Struct("<4sBBHQQd7f7f7f7f7f6f6fiHHf")
DROID_JOINT_COUNT = 7


class DroidControlMode(IntEnum):
    HOLD = 0
    JOINT_POSITION = 1
    JOINT_VELOCITY = 2


class NativeCommand(IntEnum):
    SHUTDOWN = 1
    MOVE_TO_READY = 13
    PAUSE_LIVE_INPUT = 14
    RESUME_LIVE_INPUT = 15
    MOVE_TO_READY_AND_SHUTDOWN = 17
    ENTER_GRAVITY_COMPENSATION = 30
    ENABLE_FORCE_FEEDBACK = 31
    SET_FORCE_FEEDBACK_GAIN = 32
    HOLD = 33
    HEARTBEAT = 34
    SET_TORQ_RESCALE = 35
    ACTIVATE_EFFECTOR = 36
    RECOVER = 37
    RESUME_DIRECT_COMMANDS = 38


class NativeStatus(IntEnum):
    READY = 1
    ERROR_DETECTED = 20
    RECOVERY_IN_PROGRESS = 21
    SHUTDOWN_AFTER_ERROR = 22
    READY_MOVE_IN_PROGRESS = 23
    HANDSHAKE = 30
    COMMAND_ACK = 31
    MODE = 32
    SERVO_PARAM = 33


CAP_DIRECT = 1 << 0
CAP_LIVE_INPUT = 1 << 1
CAP_GRAVITY_COMP = 1 << 2
CAP_FORCE_FEEDBACK = 1 << 3
CAP_MOVE_TO_READY = 1 << 4
CAP_EFFECTOR_ACTIVATION = 1 << 5
CAP_RECOVERY = 1 << 6
CAP_EFFECTOR_FORCE = 1 << 7
CAP_VELOCITY_COMMAND = 1 << 8


@dataclass(frozen=True, slots=True)
class ArmTopics:
    state: str
    live_command: str
    direct_command: str
    lifecycle_command: str
    status: str
    inputs: str


def topics_for(session_id: str, logical_name: str) -> ArmTopics:
    prefix = f"openpi.{session_id}.{logical_name}"
    return ArmTopics(
        state=f"{prefix}.state",
        live_command=f"{prefix}.live",
        direct_command=f"{prefix}.direct",
        lifecycle_command=f"{prefix}.lifecycle",
        status=f"{prefix}.status",
        inputs=f"{prefix}.inputs",
    )


def _fnv1a_32(value: str) -> int:
    result = 2166136261
    for byte in value.encode():
        result ^= byte
        result = (result * 16777619) & 0xFFFFFFFF
    return result


def _probe_step(value: int) -> int:
    step = value % PORT_RANGE or 1
    if step % 2 == 0:
        step += 1
    while step % 5 == 0:
        step += 2
    return step % PORT_RANGE or 1


def port_candidates(topic: str, count: int = 5) -> tuple[int, ...]:
    first = _fnv1a_32(topic)
    step = _probe_step(_fnv1a_32(topic + "\x1fprobe"))
    return tuple(BASE_PORT + ((first + index * step) % PORT_RANGE) for index in range(count))


def encode_command(
    command: NativeCommand, floats: tuple[float, ...] = (), ints: tuple[int, ...] = ()
) -> bytes:
    if len(floats) > 10 or len(ints) > 10:
        raise ProtocolError("native command supports at most ten float and ten integer parameters")
    return COMMAND_STRUCT.pack(
        int(command),
        len(floats),
        len(ints),
        *(floats + (0.0,) * (10 - len(floats))),
        *(ints + (0,) * (10 - len(ints))),
    )


def decode_inputs(payload: bytes) -> tuple[tuple[float, ...], tuple[bool, ...]]:
    """Decode a native joystick message into normalized axes and button states."""
    if len(payload) != INPUT_STRUCT.size:
        raise ProtocolError(f"invalid input payload size {len(payload)}")
    values = INPUT_STRUCT.unpack(payload)
    channel_count, button_count = values[7], values[13]
    if not 0 <= channel_count <= MAX_INPUT_CHANNELS or not 0 <= button_count <= (
        MAX_INPUT_CHANNELS
    ):
        raise ProtocolError("invalid input channel or button counts")
    axes = tuple(float(value) for value in values[2 : 2 + channel_count])
    buttons = tuple(value != 0 for value in values[8 : 8 + button_count])
    return axes, buttons


def decode_status(payload: bytes) -> tuple[NativeStatus, tuple[float, ...], tuple[int, ...]]:
    if len(payload) != STATUS_STRUCT.size:
        raise ProtocolError(f"invalid status payload size {len(payload)}")
    values = STATUS_STRUCT.unpack(payload)
    key, nf, ni = values[:3]
    if not 0 <= nf <= 10 or not 0 <= ni <= 10:
        raise ProtocolError("invalid status parameter counts")
    try:
        status = NativeStatus(key)
    except ValueError as exc:
        raise ProtocolError(f"unknown native status {key}") from exc
    return status, tuple(values[3 : 3 + nf]), tuple(values[13 : 13 + ni])


@dataclass(frozen=True, slots=True)
class DroidStatePayload:
    sequence: int
    monotonic_ns: int
    hardware_timestamp_s: float
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    effort_nm: tuple[float, ...]
    commanded_position_rad: tuple[float, ...]
    external_joint_torque_nm: tuple[float, ...]
    cartesian_wrench: tuple[float, ...]
    gripper: tuple[float, ...]
    robot_mode: int
    joint_contact_bits: int
    joint_collision_bits: int
    control_command_success_rate: float
    flags: int


def encode_droid_command(
    *,
    sequence: int,
    monotonic_ns: int,
    mode: DroidControlMode,
    position_rad: tuple[float, ...],
    velocity_rad_s: tuple[float, ...] = (0.0,) * DROID_JOINT_COUNT,
    gripper_position: float,
    gripper_speed: float,
    gripper_force: float,
) -> bytes:
    if len(position_rad) != DROID_JOINT_COUNT or len(velocity_rad_s) != DROID_JOINT_COUNT:
        raise ProtocolError("DROID commands require seven joint positions and velocities")
    return DROID_COMMAND_STRUCT.pack(
        DROID_MAGIC,
        *PROTOCOL_VERSION,
        int(mode),
        0,
        sequence,
        monotonic_ns,
        *position_rad,
        *velocity_rad_s,
        gripper_position,
        gripper_speed,
        gripper_force,
    )


def decode_droid_state(payload: bytes) -> DroidStatePayload:
    if len(payload) != DROID_STATE_STRUCT.size:
        raise ProtocolError(f"invalid DROID state payload size {len(payload)}")
    values = DROID_STATE_STRUCT.unpack(payload)
    if values[0] != DROID_MAGIC:
        raise ProtocolError("invalid DROID state magic")
    version = tuple(values[1:3])
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"incompatible DROID protocol {version}; expected {PROTOCOL_VERSION}")
    offset = 3
    flags = values[offset]
    sequence, monotonic_ns, hardware_timestamp_s = values[offset + 1 : offset + 4]
    offset += 4

    def take(count: int) -> tuple[float, ...]:
        nonlocal offset
        result = tuple(float(value) for value in values[offset : offset + count])
        offset += count
        return result

    position = take(7)
    velocity = take(7)
    effort = take(7)
    commanded = take(7)
    external = take(7)
    wrench = take(6)
    gripper = take(6)
    robot_mode = int(values[offset])
    contact_bits = int(values[offset + 1])
    collision_bits = int(values[offset + 2])
    success_rate = float(values[offset + 3])
    return DroidStatePayload(
        sequence=int(sequence),
        monotonic_ns=int(monotonic_ns),
        hardware_timestamp_s=float(hardware_timestamp_s),
        position_rad=position,
        velocity_rad_s=velocity,
        effort_nm=effort,
        commanded_position_rad=commanded,
        external_joint_torque_nm=external,
        cartesian_wrench=wrench,
        gripper=gripper,
        robot_mode=robot_mode,
        joint_contact_bits=contact_bits,
        joint_collision_bits=collision_bits,
        control_command_success_rate=success_rate,
        flags=int(flags),
    )
