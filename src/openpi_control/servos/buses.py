"""Bus sessions for servo maintenance tools, keyed by the driver's PORT_TYPE.

Servo drivers declare which transport they need ("can" or "serial"); this
module owns opening/validating that transport so tools never hard-code a bus
type. CAN is implemented (SocketCAN via python-can, with the settle timing
verified in robot-test). Serial is a declared seam: the checks work, but
opening a session raises until the first serial servo family (Dynamixel /
FeeTech) is actually integrated — port the session handling from robot-test
``pi_control/servos/dxl_serial.py`` (plain pyserial at the family's baud rate)
at that point.
"""

from __future__ import annotations

import contextlib
import pathlib
import time
from collections.abc import Iterator

import can

PORT_TYPE_CAN = "can"
PORT_TYPE_SERIAL = "serial"

# robot-test waits 1 s after opening a CAN bus before the first frame;
# adapters (especially SLCAN) drop the first frames when hit too early.
_CAN_OPEN_SETTLE_S = 1.0


def check_interface(port_type: str, interface: str) -> str | None:
    """Return None when ``interface`` exists for ``port_type``, an error message otherwise."""
    if port_type == PORT_TYPE_CAN:
        if pathlib.Path(f"/sys/class/net/{interface}").exists():
            return None
        return (
            f"CAN interface {interface!r} does not exist. "
            "Plug in the adapter and check the names with run/devices.sh."
        )
    if port_type == PORT_TYPE_SERIAL:
        if pathlib.Path(interface).exists():
            return None
        return (
            f"serial device {interface!r} does not exist. Plug in the adapter and "
            "check the names with run/devices.sh (ls /dev/serial/by-id)."
        )
    raise SystemExit(
        f"unknown port type {port_type!r}; supported: {PORT_TYPE_CAN}, {PORT_TYPE_SERIAL}"
    )


@contextlib.contextmanager
def open_bus(port_type: str, interface: str) -> Iterator[can.BusABC]:
    """Open a settled bus session on ``interface`` for drivers of ``port_type``."""
    if port_type == PORT_TYPE_CAN:
        with can.interface.Bus(channel=interface, interface="socketcan") as bus:
            time.sleep(_CAN_OPEN_SETTLE_S)
            yield bus
        return
    if port_type == PORT_TYPE_SERIAL:
        raise NotImplementedError(
            "serial bus sessions are not integrated yet: port the pyserial session "
            "handling from robot-test pi_control/servos/dxl_serial.py when the first "
            "serial servo family is brought up"
        )
    raise SystemExit(
        f"unknown port type {port_type!r}; supported: {PORT_TYPE_CAN}, {PORT_TYPE_SERIAL}"
    )


def recv_from(bus: can.BusABC, expected_ids: tuple[int, ...], timeout_s: float) -> bool:
    """Drain frames from other bus members until one of ``expected_ids`` answers or timeout."""
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        message = bus.recv(timeout=remaining)
        if message is not None and message.arbitration_id in expected_ids:
            return True
