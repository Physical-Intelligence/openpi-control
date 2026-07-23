"""Tests for the servo driver registry and the per-family zeroing routines."""

from __future__ import annotations

import pathlib

import can
import pytest

from openpi_control import servos
from openpi_control.servos import buses, dm_can, dxl_serial, encos_can

_ENCOS_BROADCAST_ID = 0x7FF


class _FakeBus:
    """Minimal python-can bus double: records sends, replays scripted responses."""

    def __init__(self, responses: list[can.Message | None]) -> None:
        self.sent: list[can.Message] = []
        self._responses = responses

    def send(self, message: can.Message) -> None:
        self.sent.append(message)

    def recv(self, timeout: float | None = None) -> can.Message | None:
        if not self._responses:
            return None
        return self._responses.pop(0)


@pytest.fixture(autouse=True)
def _no_settle_sleeps(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(dm_can, "_POST_ZERO_SETTLE_S", 0.0)
    monkeypatch.setattr(encos_can, "_POST_ZERO_SETTLE_S", 0.0)
    monkeypatch.setattr(encos_can, "_INTER_COMMAND_GAP_S", 0.0)
    monkeypatch.setattr(encos_can, "_RESPONSE_TIMEOUT_S", 0.01)
    monkeypatch.setattr(encos_can, "_DISABLE_ACK_TIMEOUT_S", 0.01)


def test_registry_resolves_every_model_json_servo() -> None:
    assert servos.zero_driver("DM J4340") is dm_can
    assert servos.zero_driver("Encos EC-A4310-P2-36") is encos_can
    assert servos.zero_driver("Dynamixel XM430-W210") is dxl_serial
    assert servos.zero_driver("ARX Remote Encoder") is None
    assert servos.zero_driver("CAN Passive Encoder") is None


def test_registry_rejects_unknown_model() -> None:
    with pytest.raises(SystemExit, match="not in the servo registry"):
        servos.zero_driver("Unknown Servo 9000")


def test_every_driver_declares_a_known_port_type() -> None:
    for driver in servos.SERVO_ZERO_DRIVERS.values():
        if driver is not None:
            assert driver.PORT_TYPE in (buses.PORT_TYPE_CAN, buses.PORT_TYPE_SERIAL)


def test_dm_zero_acknowledged() -> None:
    bus = _FakeBus([can.Message(arbitration_id=0x01)])
    assert dm_can.set_zero(bus, 0x01) is None
    assert len(bus.sent) == 1
    assert bus.sent[0].arbitration_id == 0x01
    assert bytes(bus.sent[0].data) == bytes([0xFF] * 7 + [0xFE])


def test_dm_zero_no_ack_is_an_error() -> None:
    bus = _FakeBus([])
    assert dm_can.set_zero(bus, 0x01) == "no acknowledgement"


def test_encos_zero_full_sequence() -> None:
    servo_id = 0x05
    bus = _FakeBus(
        [
            can.Message(arbitration_id=servo_id),  # enable ack
            can.Message(arbitration_id=servo_id),  # set-zero ack
            can.Message(arbitration_id=servo_id),  # disable ack
        ]
    )
    assert encos_can.set_zero(bus, servo_id) is None
    sent_ids = [message.arbitration_id for message in bus.sent]
    assert sent_ids == [servo_id, _ENCOS_BROADCAST_ID, servo_id]


def test_encos_zero_accepts_broadcast_ack() -> None:
    servo_id = 0x05
    bus = _FakeBus(
        [
            can.Message(arbitration_id=servo_id),  # enable ack
            can.Message(arbitration_id=_ENCOS_BROADCAST_ID),  # set-zero ack on broadcast id
        ]
    )
    assert encos_can.set_zero(bus, servo_id) is None


def test_encos_zero_reports_missing_set_zero_ack() -> None:
    servo_id = 0x05
    bus = _FakeBus([can.Message(arbitration_id=servo_id)])  # enable ack only
    assert encos_can.set_zero(bus, servo_id) == "enabled OK but set-zero not acknowledged"


def test_dxl_zero_fails_fast_until_integrated() -> None:
    with pytest.raises(NotImplementedError, match="robot-test pi_control/servos/dxl_serial.py"):
        dxl_serial.set_zero(object(), 1)


def test_serial_bus_session_fails_fast_until_integrated() -> None:
    with pytest.raises(NotImplementedError, match="serial bus sessions"):
        with buses.open_bus(buses.PORT_TYPE_SERIAL, "/dev/ttyUSB_test"):
            pass


def test_check_interface_serial_uses_device_path(tmp_path: pathlib.Path) -> None:
    device = tmp_path / "ttyUSB_test"
    assert buses.check_interface(buses.PORT_TYPE_SERIAL, str(device)) is not None
    device.touch()
    assert buses.check_interface(buses.PORT_TYPE_SERIAL, str(device)) is None


def test_check_interface_can_reports_missing_interface() -> None:
    error = buses.check_interface(buses.PORT_TYPE_CAN, "can_does_not_exist")
    assert error is not None and "does not exist" in error
