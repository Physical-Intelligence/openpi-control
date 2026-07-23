"""DM (Damiao) CAN servo driver — firmware zeroing.

Ported from robot-test ``pi_control/servos/dm_can.py`` (send_command_set_zero):
a special command frame sent to the servo's own CAN id sets the current
position as the firmware zero; any response within the window counts as
acknowledgement.
"""

from __future__ import annotations

import time

import can

from openpi_control.servos import buses

PORT_TYPE = buses.PORT_TYPE_CAN

_ZERO_PAYLOAD = bytes([0xFF] * 7 + [0xFE])
_RESPONSE_TIMEOUT_S = 0.05
# The firmware zero is a config write; the servo misbehaves when the next
# command comes too early (robot-test verified timing).
_POST_ZERO_SETTLE_S = 0.5


def set_zero(bus: can.BusABC, servo_id: int) -> str | None:
    """Set the current position as firmware zero; None on success, error detail otherwise."""
    bus.send(can.Message(arbitration_id=servo_id, data=_ZERO_PAYLOAD, is_extended_id=False))
    if bus.recv(timeout=_RESPONSE_TIMEOUT_S) is None:
        return "no acknowledgement"
    time.sleep(_POST_ZERO_SETTLE_S)
    return None
