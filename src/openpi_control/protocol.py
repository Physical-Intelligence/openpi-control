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

JOINT_STRUCT = struct.Struct("@50fihhf")
COMMAND_STRUCT = struct.Struct("@hhh10f10i")
STATUS_STRUCT = struct.Struct("@hhh10f10i")
# Native ZmqJoystickInfo: mode, side, channel[5], channel_num, button[5],
# button_num, raw_channel[5] (see native/pi_control/include/pi_topic_zmq.hpp).
INPUT_STRUCT = struct.Struct("@bb2x5fb5bbx5h2x")
MAX_INPUT_CHANNELS = 5


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


class NativeStatus(IntEnum):
    READY = 1
    ERROR_DETECTED = 20
    RECOVERY_IN_PROGRESS = 21
    SHUTDOWN_AFTER_ERROR = 22
    READY_MOVE_IN_PROGRESS = 23
    HANDSHAKE = 30
    COMMAND_ACK = 31
    MODE = 32


CAP_DIRECT = 1 << 0
CAP_LIVE_INPUT = 1 << 1
CAP_GRAVITY_COMP = 1 << 2
CAP_FORCE_FEEDBACK = 1 << 3
CAP_MOVE_TO_READY = 1 << 4


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
