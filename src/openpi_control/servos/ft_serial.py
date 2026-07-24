"""FeeTech SMS/STS serial servo driver — packet layer, ping/scan, and zeroing.

Ported from robot-test ``pi_control/servos/ft_serial.py``, reduced to what the
zeroing tool and the device wizard need for the STS3215 class (SO-ARM100/101;
the Hiwonder HX-30HM / HX-10HM in SO-ARM101 kits share the exact register
layout and protocol, so no separate servo model exists).

The C++ mirror of this protocol lives in native/pi_control/src/pi_driver_ft.cpp.
"""

from __future__ import annotations

import logging
import struct

import serial

from openpi_control.servos import buses

PORT_TYPE = buses.PORT_TYPE_SERIAL
WHOLE_ARM_ZERO = False

# FeeTech instruction set (subset used here).
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

ID_BROADCAST = 0xFE

# SMS/STS control table (subset).
ADDR_BAUD_RATE = 6
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK_FLAG = 55
ADDR_PRESENT_POSITION = 56

# Writing 128 to Torque Enable calibrates the current position to the encoder
# center (2048) on SMS/STS servos — the standard STS3215 zeroing method (same
# operation as the FeeTech FD software "Calibrate middle" button).
TORQUE_ENABLE_CALIBRATE_MIDDLE = 128

# Baud Rate register (addr 6) value mapping for the STS3215 class.
BAUDRATE_REG_VALUES = {
    1000000: 0,
    500000: 1,
    250000: 2,
    128000: 3,
    115200: 4,
    76800: 5,
    57600: 6,
    38400: 7,
}


def check_sum(packet: bytes) -> int:
    """FeeTech checksum: ~(sum of bytes after the FF FF header) & 0xFF."""
    return (~(sum(packet[2:]) & 0xFF)) & 0xFF


def build_packet(servo_id: int, instruction: int, parameters: bytes) -> bytes:
    """Build an instruction packet: FF FF ID LEN INST [params] CHK."""
    packet = bytearray(b"\xff\xff")
    packet += struct.pack("<BBB", servo_id, 2 + len(parameters), instruction)
    packet += parameters
    packet += struct.pack("<B", check_sum(packet))
    return bytes(packet)


def ft_write(bus: serial.Serial, servo_id: int, address: int, value: int, length: int) -> bool:
    """Write a register (INST_WRITE) and validate the status packet."""
    value_bytes = value.to_bytes(length, byteorder="little", signed=value < 0)
    packet = build_packet(servo_id, INST_WRITE, struct.pack("<B", address) + value_bytes)
    bus.reset_input_buffer()
    bus.write(packet)
    bus.flush()

    response = bus.read(6)
    if len(response) < 6:
        logging.getLogger(__name__).error(
            "ID %d: write timeout waiting for status packet", servo_id
        )
        return False
    if response[:2] != b"\xff\xff" or response[2] != servo_id or response[3] != 0x02:
        logging.getLogger(__name__).error("ID %d: invalid write status packet", servo_id)
        return False
    if response[4] != 0:
        logging.getLogger(__name__).error(
            "ID %d: write failed with error bits 0x%02X", servo_id, response[4]
        )
        return False
    if response[-1] != check_sum(response[:-1]):
        logging.getLogger(__name__).error("ID %d: write status checksum mismatch", servo_id)
        return False
    return True


def ft_read(bus: serial.Serial, servo_id: int, address: int, length: int) -> bytes | None:
    """Read a register window (INST_READ); returns the parameter bytes or None."""
    packet = build_packet(servo_id, INST_READ, struct.pack("<BB", address, length))
    bus.reset_input_buffer()
    bus.write(packet)
    bus.flush()

    expected_length = 6 + length
    response = bus.read(expected_length)
    if len(response) < expected_length:
        logging.getLogger(__name__).error(
            "ID %d: read timeout: got %d bytes, expected %d",
            servo_id,
            len(response),
            expected_length,
        )
        return None
    if response[:2] != b"\xff\xff" or response[2] != servo_id:
        logging.getLogger(__name__).error("ID %d: invalid read status packet", servo_id)
        return None
    if response[-1] != check_sum(response[:-1]):
        logging.getLogger(__name__).error("ID %d: read status checksum mismatch", servo_id)
        return None
    return response[5:-1]


def ping(bus: serial.Serial, servo_id: int) -> bool:
    """Ping one servo (INST_PING) and validate the status packet."""
    packet = build_packet(servo_id, INST_PING, b"")
    bus.reset_input_buffer()
    bus.write(packet)
    bus.flush()

    response = bus.read(6)
    if len(response) < 6:
        return False
    if response[:2] != b"\xff\xff" or response[2] != servo_id or response[3] != 0x02:
        return False
    return response[-1] == check_sum(response[:-1])


def scan(bus: serial.Serial, servo_ids: tuple[int, ...]) -> list[int]:
    """Ping each id in turn; returns the ids that answered."""
    return [servo_id for servo_id in servo_ids if ping(bus, servo_id)]


def torque_enable(bus: serial.Serial, servo_id: int, enable: bool | int) -> bool:
    """Enable/disable torque output (also carries the calibrate-middle value 128)."""
    return ft_write(bus, servo_id, ADDR_TORQUE_ENABLE, int(enable), 1)


def set_eeprom_lock(bus: serial.Serial, servo_id: int, locked: bool) -> bool:
    """Set the EEPROM write Lock Flag (STS3215 ships write-locked)."""
    return ft_write(bus, servo_id, ADDR_LOCK_FLAG, 1 if locked else 0, 1)


def get_baudrate(bus: serial.Serial, servo_id: int) -> int | None:
    """Read the Baud Rate register and return it in bps, or None on failure."""
    response = ft_read(bus, servo_id, ADDR_BAUD_RATE, 1)
    if response is None or len(response) < 1:
        logging.getLogger(__name__).error("ID %d: failed to read Baud Rate register", servo_id)
        return None
    for bps, reg_value in BAUDRATE_REG_VALUES.items():
        if reg_value == response[0]:
            return bps
    logging.getLogger(__name__).error(
        "ID %d: unknown Baud Rate register value %d", servo_id, response[0]
    )
    return None


def set_baudrate(bus: serial.Serial, servo_id: int, new_baudrate: int) -> bool:
    """Change the servo's UART baudrate and follow it on the host side.

    Steps: unlock EEPROM, write the Baud Rate register (takes effect
    immediately, no reboot instruction exists in the FeeTech protocol), switch
    the host serial port to the new baud, re-lock EEPROM, verify with a ping.
    On failure the host baud is restored.
    """
    if new_baudrate not in BAUDRATE_REG_VALUES:
        logging.getLogger(__name__).error(
            "ID %d: baudrate %d is not supported (allowed: %s)",
            servo_id,
            new_baudrate,
            sorted(BAUDRATE_REG_VALUES),
        )
        return False

    old_baudrate = int(bus.baudrate)
    if not set_eeprom_lock(bus, servo_id, locked=False):
        logging.getLogger(__name__).error(
            "ID %d: failed to unlock EEPROM for baudrate change", servo_id
        )
        return False

    if not ft_write(bus, servo_id, ADDR_BAUD_RATE, BAUDRATE_REG_VALUES[new_baudrate], 1):
        logging.getLogger(__name__).error("ID %d: failed to write Baud Rate register", servo_id)
        set_eeprom_lock(bus, servo_id, locked=True)
        return False

    # The servo switches immediately after the write; follow it on the host side.
    bus.baudrate = int(new_baudrate)
    bus.reset_input_buffer()

    if not set_eeprom_lock(bus, servo_id, locked=True):
        logging.getLogger(__name__).error(
            "ID %d: failed to re-lock EEPROM after baudrate change", servo_id
        )

    if not ping(bus, servo_id):
        logging.getLogger(__name__).error(
            "ID %d: no answer at new baudrate %d; restoring host baud %d",
            servo_id,
            new_baudrate,
            old_baudrate,
        )
        bus.baudrate = old_baudrate
        return False
    return True


def set_zero(bus: serial.Serial, servo_id: int) -> str | None:
    """Calibrate the current position to the encoder center (STS3215 zeroing).

    Torque Enable = 128 sets the present position to step 2048; the write is
    wrapped with the EEPROM Lock Flag unlock/re-lock pair the STS3215 firmware
    requires. Returns None on success or an error detail string.
    """
    if not set_eeprom_lock(bus, servo_id, locked=False):
        return "failed to unlock EEPROM for calibrate-middle"
    calibrated = torque_enable(bus, servo_id, TORQUE_ENABLE_CALIBRATE_MIDDLE)
    if not set_eeprom_lock(bus, servo_id, locked=True):
        return "failed to re-lock EEPROM after calibrate-middle"
    if not calibrated:
        return "calibrate-middle command was not acknowledged"
    return None
