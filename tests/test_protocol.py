import struct

import pytest

from openpi_control.exceptions import ProtocolError
from openpi_control.protocol import (
    COMMAND_STRUCT,
    DROID_COMMAND_STRUCT,
    DROID_MAGIC,
    DROID_STATE_STRUCT,
    INPUT_STRUCT,
    JOINT_STRUCT,
    STATUS_STRUCT,
    DroidControlMode,
    NativeCommand,
    decode_droid_state,
    decode_inputs,
    decode_status,
    encode_command,
    encode_droid_command,
    port_candidates,
)


def test_native_abi_sizes_match_natural_cpp_layout() -> None:
    assert JOINT_STRUCT.size == 252
    assert COMMAND_STRUCT.size == 88
    assert STATUS_STRUCT.size == 88
    assert DROID_COMMAND_STRUCT.size == 92
    assert DROID_STATE_STRUCT.size == 232


def test_command_encoding_and_status_decoding() -> None:
    payload = encode_command(NativeCommand.SET_FORCE_FEEDBACK_GAIN, floats=(0.2,), ints=(7,))
    values = COMMAND_STRUCT.unpack(payload)
    assert values[:3] == (NativeCommand.SET_FORCE_FEEDBACK_GAIN, 1, 1)
    assert values[3] == pytest.approx(0.2)
    assert values[13] == 7

    status = STATUS_STRUCT.pack(30, 0, 3, *([0.0] * 10), 1, 0, 31, *([0] * 7))
    key, floats, ints = decode_status(status)
    assert int(key) == 30
    assert floats == ()
    assert ints == (1, 0, 31)


def test_port_hash_is_deterministic_and_spreads_candidates() -> None:
    assert port_candidates("topic") == port_candidates("topic")
    assert len(set(port_candidates("topic", 16))) == 16
    assert port_candidates("topic") != port_candidates("other")


def test_protocol_rejects_bad_payloads() -> None:
    with pytest.raises(ProtocolError):
        decode_status(struct.pack("i", 1))
    with pytest.raises(ProtocolError):
        decode_droid_state(b"bad")


def test_droid_protocol_round_trip() -> None:
    command = encode_droid_command(
        sequence=4,
        monotonic_ns=5,
        mode=DroidControlMode.JOINT_POSITION,
        position_rad=tuple(float(i) for i in range(7)),
        gripper_position=0.25,
        gripper_speed=0.5,
        gripper_force=1.0,
    )
    values = DROID_COMMAND_STRUCT.unpack(command)
    assert values[0] == DROID_MAGIC
    assert values[3] == DroidControlMode.JOINT_POSITION
    assert values[7:14] == pytest.approx(tuple(float(i) for i in range(7)))

    state = DROID_STATE_STRUCT.pack(
        DROID_MAGIC,
        1,
        1,
        3,
        10,
        20,
        1.5,
        *([0.1] * 7),
        *([0.2] * 7),
        *([0.3] * 7),
        *([0.4] * 7),
        *([0.5] * 7),
        *([0.6] * 6),
        0.7,
        0.8,
        0.9,
        10.0,
        1.1,
        0.25,
        2,
        0b001,
        0b100,
        0.99,
    )
    decoded = decode_droid_state(state)
    assert decoded.sequence == 10
    assert decoded.position_rad == pytest.approx([0.1] * 7)
    assert decoded.gripper[0] == pytest.approx(0.7)
    assert decoded.gripper[5] == pytest.approx(0.25)
    assert decoded.joint_contact_bits == 1


def test_input_abi_matches_native_zmq_joystick_info() -> None:
    assert INPUT_STRUCT.size == 44
    payload = INPUT_STRUCT.pack(
        0, 1, 0.5, -1.0, 0.25, 0.0, 0.0, 3, 1, 0, 1, 0, 0, 3, 127, 0, 191, 0, 0
    )
    axes, buttons = decode_inputs(payload)
    assert axes == pytest.approx((0.5, -1.0, 0.25))
    assert buttons == (True, False, True)
    with pytest.raises(ProtocolError):
        decode_inputs(payload[:-1])
    bad_counts = INPUT_STRUCT.pack(0, 0, *([0.0] * 5), 6, *([0] * 5), 0, *([0] * 5))
    with pytest.raises(ProtocolError):
        decode_inputs(bad_counts)
