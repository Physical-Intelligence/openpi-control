"""Property-based tests for the native wire protocol.

The decoders consume bytes straight off ZMQ sockets from a separately-versioned
native process, so the contract is: a correctly-sized payload either decodes or
raises ProtocolError. Any other exception can kill the backend reader thread
with an undiagnosable error.
"""

from __future__ import annotations

import pytest
from hypothesis import given
from hypothesis import strategies as st

from openpi_control.exceptions import ProtocolError
from openpi_control.protocol import (
    BASE_PORT,
    COMMAND_STRUCT,
    INPUT_STRUCT,
    PORT_RANGE,
    STATUS_STRUCT,
    NativeCommand,
    NativeStatus,
    decode_inputs,
    decode_status,
    encode_command,
    port_candidates,
)


@given(st.binary(min_size=STATUS_STRUCT.size, max_size=STATUS_STRUCT.size))
def test_decode_status_raises_only_protocol_error(payload: bytes) -> None:
    try:
        status, floats, ints = decode_status(payload)
    except ProtocolError:
        return
    assert isinstance(status, NativeStatus)
    assert len(floats) <= 10 and len(ints) <= 10


@given(st.binary(max_size=STATUS_STRUCT.size * 2))
def test_decode_status_rejects_wrong_sizes_cleanly(payload: bytes) -> None:
    if len(payload) == STATUS_STRUCT.size:
        return
    with pytest.raises(ProtocolError):
        decode_status(payload)


@given(st.binary(min_size=INPUT_STRUCT.size, max_size=INPUT_STRUCT.size))
def test_decode_inputs_raises_only_protocol_error(payload: bytes) -> None:
    try:
        axes, buttons = decode_inputs(payload)
    except ProtocolError:
        return
    assert len(axes) <= 5 and len(buttons) <= 5
    assert all(isinstance(value, bool) for value in buttons)


@given(
    command=st.sampled_from(list(NativeCommand)),
    floats=st.lists(st.floats(width=32, allow_nan=False), max_size=10),
    ints=st.lists(st.integers(min_value=-(2**31), max_value=2**31 - 1), max_size=10),
)
def test_encode_command_roundtrips(command, floats, ints) -> None:
    payload = encode_command(command, floats=tuple(floats), ints=tuple(ints))
    values = COMMAND_STRUCT.unpack(payload)
    assert values[0] == int(command)
    assert values[1] == len(floats)
    assert values[2] == len(ints)
    assert list(values[3 : 3 + len(floats)]) == pytest.approx(floats)
    assert list(values[13 : 13 + len(ints)]) == ints


@given(
    status=st.sampled_from(list(NativeStatus)),
    ints=st.lists(st.integers(min_value=-(2**31), max_value=2**31 - 1), max_size=10),
)
def test_status_roundtrips_through_decoder(status, ints) -> None:
    payload = STATUS_STRUCT.pack(
        int(status), 0, len(ints), *((0.0,) * 10), *(tuple(ints) + (0,) * (10 - len(ints)))
    )
    decoded_status, floats, decoded_ints = decode_status(payload)
    assert decoded_status is status
    assert floats == ()
    assert list(decoded_ints) == ints


@given(st.text(min_size=1, max_size=80))
def test_port_candidates_are_deterministic_and_in_range(topic: str) -> None:
    first = port_candidates(topic, 16)
    second = port_candidates(topic, 16)
    assert first == second
    assert len(first) == 16
    for port in first:
        assert BASE_PORT <= port < BASE_PORT + PORT_RANGE
