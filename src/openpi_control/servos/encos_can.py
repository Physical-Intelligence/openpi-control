"""ENCOS CAN servo driver — firmware zeroing.

Ported from robot-test ``pi_control/servos/encos_can.py``: the servo only
acknowledges the broadcast set-zero after it has been enabled, so the verified
sequence is enable (zero-gain MIT frame) -> set-zero -> settle -> disable. The
broadcast frame carries the target motor id; the servo answers on its own id
(some firmware acks config broadcasts on the broadcast id, so both are
accepted). Config commands need a long response window.
"""

from __future__ import annotations

import time

import can

from openpi_control.servos import buses

PORT_TYPE = buses.PORT_TYPE_CAN

_CAN_ID_BROADCAST = 0x7FF
_CMD_SET_ZERO = 0x03
_CMD_DISABLE = 0x61
_RESPONSE_TIMEOUT_S = 1.0
_DISABLE_ACK_TIMEOUT_S = 0.05
# Zero-gain MIT command (kp=0 kd=0 pos=0 spd=0 tor=0) encoded per the ENCOS
# MIT bit packing. Model-independent: every ENCOS model has symmetric
# pos/vel/tor ranges (0 maps to mid-scale) and kp/kd minimums of 0.
_MIT_IDLE_PAYLOAD = bytes([0x00, 0x00, 0x00, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF])

# Timing ported 1:1 from robot-test (encos_can.py __main__ and pi_ui
# set_servos_to_zero) — the firmware zero is a config write and the servo
# misbehaves when the next command comes too early.
_INTER_COMMAND_GAP_S = 0.1
_POST_ZERO_SETTLE_S = 0.5


def set_zero(bus: can.BusABC, servo_id: int) -> str | None:
    """Set the current position as firmware zero; returns None on success, error detail otherwise."""
    # 1. Enable (wake) with a zero-gain MIT frame — exerts no torque; its ack
    #    also proves the servo is powered and reachable.
    bus.send(can.Message(arbitration_id=servo_id, data=_MIT_IDLE_PAYLOAD, is_extended_id=False))
    if not buses.recv_from(bus, (servo_id,), _RESPONSE_TIMEOUT_S):
        return "no response to enable command (check servo power and CAN wiring)"
    time.sleep(_INTER_COMMAND_GAP_S)

    # 2. Set-zero broadcast carrying the target motor id. The ack normally
    #    comes on the servo's own id, but some firmware answers config
    #    broadcasts on the broadcast id (robot-test's set_new_id waits on it),
    #    so accept either.
    payload = bytes([(servo_id >> 8) & 0xFF, servo_id & 0xFF, 0, _CMD_SET_ZERO])
    bus.send(can.Message(arbitration_id=_CAN_ID_BROADCAST, data=payload, is_extended_id=False))
    zeroed = buses.recv_from(bus, (servo_id, _CAN_ID_BROADCAST), _RESPONSE_TIMEOUT_S)

    # 3. Let the config write settle BEFORE any further frame — disabling too
    #    early corrupts the zero (robot-test waits 0.5 s here).
    time.sleep(_POST_ZERO_SETTLE_S)

    # 4. Return to the disabled state (verified robot-test sequence); drain the ack.
    bus.send(can.Message(arbitration_id=servo_id, data=bytes([_CMD_DISABLE, 0, 0]), is_extended_id=False))
    buses.recv_from(bus, (servo_id,), _DISABLE_ACK_TIMEOUT_S)

    if not zeroed:
        return "enabled OK but set-zero not acknowledged"
    return None
