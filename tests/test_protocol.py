import struct

import pytest

from openpi_control.exceptions import ProtocolError
from openpi_control.protocol import (
    COMMAND_STRUCT,
    INPUT_STRUCT,
    JOINT_STRUCT,
    STATUS_STRUCT,
    NativeCommand,
    decode_inputs,
    decode_status,
    encode_command,
    port_candidates,
)


def test_native_abi_sizes_match_natural_cpp_layout() -> None:
    assert JOINT_STRUCT.size == 212
    assert COMMAND_STRUCT.size == 88
    assert STATUS_STRUCT.size == 88


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
