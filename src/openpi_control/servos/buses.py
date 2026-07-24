"""Bus sessions for servo maintenance tools, keyed by the driver's PORT_TYPE.

Servo drivers declare which transport they need ("can", "serial", or
"ethernet"); this module owns opening/validating that transport so tools
never hard-code a bus type. CAN is implemented (SocketCAN via python-can,
with the settle timing verified in robot-test). Ethernet opens a Trossen
controller session (the "bus" is the vendor driver object). Serial opens a
plain pyserial session at the caller-provided baud rate (the model catalog's
``baudrate``); the protocol on top belongs to the servo driver module
(``ft_serial`` for FeeTech, ``dxl_serial`` once Dynamixel is integrated).
"""

from __future__ import annotations

import contextlib
import ipaddress
import pathlib
import time
from collections.abc import Iterator

import can
import serial as pyserial
import trossen_arm

from openpi_control.servos import trossen_eth

PORT_TYPE_CAN = "can"
PORT_TYPE_SERIAL = "serial"
PORT_TYPE_ETHERNET = trossen_eth.PORT_TYPE

_SUPPORTED_PORT_TYPES = f"{PORT_TYPE_CAN}, {PORT_TYPE_SERIAL}, {PORT_TYPE_ETHERNET}"

# robot-test waits 1 s after opening a CAN bus before the first frame;
# adapters (especially SLCAN) drop the first frames when hit too early.
_CAN_OPEN_SETTLE_S = 1.0

# Serial per-read deadline: dominated by one USB latency window (up to 16 ms
# FTDI/CDC), so 100 ms leaves ample margin (robot-test ft_serial timing).
_SERIAL_READ_TIMEOUT_S = 0.1


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
    if port_type == PORT_TYPE_ETHERNET:
        try:
            ipaddress.IPv4Address(interface)
        except ValueError:
            return (
                f"Ethernet controller address {interface!r} is not a valid IPv4 address; "
                "check the bus entry in the config TOML (eth:<ip>)."
            )
        if trossen_eth.reachable(interface):
            return None
        return (
            f"Ethernet controller at {interface} did not answer a discovery probe. Check the "
            "cable and power, and that the host has an address on the controller's subnet."
        )
    raise SystemExit(f"unknown port type {port_type!r}; supported: {_SUPPORTED_PORT_TYPES}")


@contextlib.contextmanager
def open_bus(
    port_type: str, interface: str, *, baudrate: int | None = None
) -> Iterator[can.BusABC | trossen_arm.TrossenArmDriver | pyserial.Serial]:
    """Open a settled bus session on ``interface`` for drivers of ``port_type``.

    Serial sessions require ``baudrate`` (the arm model catalog's ``baudrate``
    field) — the transport is protocol-neutral, so the caller must state the
    bus speed explicitly.
    """
    if port_type == PORT_TYPE_CAN:
        with can.interface.Bus(channel=interface, interface="socketcan") as bus:
            time.sleep(_CAN_OPEN_SETTLE_S)
            yield bus
        return
    if port_type == PORT_TYPE_ETHERNET:
        # The "bus" is the vendor driver session; ``interface`` is the
        # controller's IPv4 address.
        with trossen_eth.open_session(interface) as driver:
            yield driver
        return
    if port_type == PORT_TYPE_SERIAL:
        if baudrate is None or baudrate <= 0:
            raise SystemExit(
                "serial bus sessions need an explicit positive baudrate "
                "(the arm model catalog's 'baudrate' field)"
            )
        with pyserial.Serial(
            port=interface, baudrate=baudrate, timeout=_SERIAL_READ_TIMEOUT_S
        ) as bus:
            yield bus
        return
    raise SystemExit(f"unknown port type {port_type!r}; supported: {_SUPPORTED_PORT_TYPES}")


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
