"""Per-servo-family drivers and the servo model registry.

Mirrors robot-test's ``pi_control/servos`` layout: one driver module per servo
family and bus type (``dm_can``, ``encos_can``, ``dxl_serial``, ...), all
exposing the same function API, plus this registry keyed by the CANONICAL
servo model string.

Naming policy (single source of truth, same as robot-test): the registry keys
are the exact ``servo_model`` strings used in the arm/effector config JSONs
under ``openpi_control/models/``. No translation layer exists — read the
JSON, look the string up here. A new servo model must use the same canonical
string in the model JSON and in this table.

Driver module API (uniform across families):
    PORT_TYPE: str             "can" or "serial" — which bus session the
                               driver needs (see ``buses.open_bus``)
    set_zero(bus, servo_id)    set the current position as the firmware zero;
                               returns None on success or an error string

Adding a servo family: create ``<family>_<bus>.py`` with that API (port the
routine from robot-test ``pi_control/servos``), then register every canonical
model string of the family below. Nothing else changes — the zeroing tool
dispatches through this table.
"""

from __future__ import annotations

import types

from openpi_control.servos import dm_can, dxl_serial, encos_can

# Servo model string -> driver module, or None for read-only encoders whose
# zero is fixed in hardware (reported as skipped by the zeroing tool, never
# as a failure).
SERVO_ZERO_DRIVERS: dict[str, types.ModuleType | None] = {
    "DM J4310": dm_can,
    "DM J4340": dm_can,
    "DM J3507": dm_can,
    "DM S3519": dm_can,
    "Encos EC-A4310-P2-36": encos_can,
    "Encos EC-A6013-H20-100": encos_can,
    "Encos EC-A4315-P2-36": encos_can,
    "Encos EC-A6408-P2-25": encos_can,
    "Encos EC-A10020-P2-24": encos_can,
    "Dynamixel XM430-W210": dxl_serial,
    "Dynamixel XH430-W210": dxl_serial,
    "Dynamixel XC330-T288": dxl_serial,
    "Dynamixel XH430-W350": dxl_serial,
    # Read-only encoders: no motor, zero reference fixed in hardware.
    "ARX Remote Encoder": None,
    "CAN Passive Encoder": None,
}


def zero_driver(servo_model: str) -> types.ModuleType | None:
    """Driver module for ``servo_model``; None means read-only (nothing to zero)."""
    if servo_model not in SERVO_ZERO_DRIVERS:
        raise SystemExit(
            f"servo model {servo_model!r} is not in the servo registry "
            "(openpi_control/servos): add the canonical robot-test model string "
            "and its driver module to SERVO_ZERO_DRIVERS"
        )
    return SERVO_ZERO_DRIVERS[servo_model]
