"""Software-in-the-loop emulation of DM-series CAN servos (DM J4340/J4310).

Implements just enough of the DM MIT-mode protocol for pi_control_node to
treat a SocketCAN interface (typically a Linux vcan device) as a live arm:

* register writes on the 0x7FF broadcast id are accepted silently,
* enable (0xFC) / disable (0xFD) / set-zero (0xFE) / reset (0xFB) commands are
  acknowledged with a status frame,
* MIT command frames update the servo position (ideal tracking) and are
  answered with a status frame on CAN id ``motor_id + 0x10`` (P16 layout),
* optionally, a passive trigger encoder (YAM teaching handle) answers the
  ``[0xFF, 0x02]`` poll on ``encoder_id + 1`` with the big-endian ``!BhhB``
  payload (device_id, position ticks, velocity ticks/s, button bits), and
  emulates the i2rt firmware/version/EEPROM startup handshake.

Byte layouts mirror ServoDm::can_frame_to_command_dm_servo and
ServoDm::parse_dm_servo_status in native/pi_control/src/pi_servo_dm.cpp, and
ServoCanPassiveEncoder::parse_encoder_status in
native/pi_control/src/pi_servo_can_encoder.cpp.
"""

from __future__ import annotations

import math
import socket
import struct
import threading
import time

# DM parameter ranges (pi_servo_dm.cpp g_servo_dm_param_*). Position and kp
# ranges are common across the J43xx family; velocity/torque scaling differs
# per model, so MIT decoding must use the right model per motor (J4340 arm
# joints: tor +/-28; J4310 gripper: tor +/-10).
POS_MIN, POS_MAX = -12.5, 12.5
VEL_MIN, VEL_MAX = -10.0, 10.0
TOR_MIN, TOR_MAX = -28.0, 28.0
J4310_VEL_MIN, J4310_VEL_MAX = -30.0, 30.0
J4310_TOR_MIN, J4310_TOR_MAX = -10.0, 10.0

_CAN_FRAME = struct.Struct("=IB3x8s")
_RESPONSE_ID_OFFSET = 0x10  # motor N replies on CAN id N + 0x10
_BROADCAST_ID = 0x7FF

# YAM teaching-handle passive encoder (mirrors i2rt PassiveEncoderReader).
_ENCODER_POLL = b"\xff\x02"
_ENCODER_RESTART = b"\xff\x0f"  # i2rt REQ_RESTART: reboots the handle MCU
_ENCODER_SET_REPORT_FREQUENCY = 0x01
_ENCODER_GET_VERSION = 0x03
_ENCODER_SET_ADC_FREQUENCY = 0x04
_ENCODER_READINGS_RESPONSE = 0x86
_ENCODER_GET_EEPROM = 0x07
_ENCODER_ADC_FREQUENCY_LOW_OFFSET = 8
_ENCODER_REPORT_FREQUENCY_LOW_OFFSET = 25
_ENCODER_ADC_FREQUENCY_HIGH_OFFSET = 27
_ENCODER_REPORT_FREQUENCY_HIGH_OFFSET = 28
_ENCODER_RESPONSE = struct.Struct("!BhhB")  # device_id, pos ticks, vel ticks/s, buttons
_ENCODER_TICKS_PER_REV = 4096
_ENCODER_RESPONSE_ID_OFFSET = 1  # firmware "plus_one" receive mode: replies on id + 1


def float_to_uint(value: float, lo: float, hi: float, bits: int) -> int:
    """Mirror of can/math_ops.cpp float_to_uint (truncating, no clamping needed here)."""
    value = min(max(value, lo), hi)
    return int(((1 << bits) - 1) * (value - lo) / (hi - lo))


def uint_to_float(raw: int, lo: float, hi: float, bits: int) -> float:
    return raw * (hi - lo) / float((1 << bits) - 1) + lo


def encode_status(
    motor_id: int,
    error: int,
    pos: float,
    vel: float,
    tor: float,
    temperature: int = 25,
    *,
    vel_lo: float = VEL_MIN,
    vel_hi: float = VEL_MAX,
    tor_lo: float = TOR_MIN,
    tor_hi: float = TOR_MAX,
) -> bytes:
    pos_u = float_to_uint(pos, POS_MIN, POS_MAX, 16)
    vel_u = float_to_uint(vel, vel_lo, vel_hi, 12)
    tor_u = float_to_uint(tor, tor_lo, tor_hi, 12)
    data = bytes(
        [
            ((error & 0x0F) << 4) | (motor_id & 0x0F),
            (pos_u >> 8) & 0xFF,
            pos_u & 0xFF,
            (vel_u >> 4) & 0xFF,
            ((vel_u & 0x0F) << 4) | ((tor_u >> 8) & 0x0F),
            tor_u & 0xFF,
            0,
            temperature & 0xFF,
        ]
    )
    return data


KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0


def decode_mit_command(
    data: bytes,
    *,
    vel_lo: float = VEL_MIN,
    vel_hi: float = VEL_MAX,
    tor_lo: float = TOR_MIN,
    tor_hi: float = TOR_MAX,
) -> tuple[float, float, float, float]:
    """Returns (position, velocity, kp, torque) from an MIT command frame payload."""
    pos_u = (data[0] << 8) | data[1]
    vel_u = (data[2] << 4) | (data[3] >> 4)
    kp_u = ((data[3] & 0x0F) << 8) | data[4]
    tor_u = ((data[6] & 0x0F) << 8) | data[7]
    return (
        uint_to_float(pos_u, POS_MIN, POS_MAX, 16),
        uint_to_float(vel_u, vel_lo, vel_hi, 12),
        uint_to_float(kp_u, KP_MIN, KP_MAX, 12),
        uint_to_float(tor_u, tor_lo, tor_hi, 12),
    )


class FakeDmServoBus:
    """Emulates a bus of DM servos on a SocketCAN interface."""

    def __init__(
        self,
        interface: str,
        motor_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6),
        handle_encoder_id: int | None = None,
        j4310_motors: tuple[int, ...] = (),
        handle_firmware: tuple[int, int, int] | None = (2, 2, 12),
        handle_adc_frequency: int = 255,
        handle_report_frequency: int = 0,
        handle_response_delay_s: float = 0.0,
    ) -> None:
        self.interface = interface
        self.positions: dict[int, float] = {motor_id: 0.0 for motor_id in motor_ids}
        self._temperatures: dict[int, int] = {motor_id: 25 for motor_id in motor_ids}
        self.frames_handled = 0
        # Optional passive trigger encoder (YAM teaching handle). It only ever
        # answers the poll frame; MIT/enable traffic must never reach it.
        self._encoder_id = handle_encoder_id
        self._encoder_device = 0
        self._encoder_firmware = handle_firmware
        self._encoder_adc_frequency = handle_adc_frequency
        self._encoder_report_frequency = handle_report_frequency
        self._encoder_response_delay_s = handle_response_delay_s
        self._encoder_ticks = 0
        self._encoder_buttons = 0
        self._encoder_muted = False
        # Real handles answer polls again ~8.5 s after REQ_RESTART; the fake
        # heals faster so tests stay quick (still asynchronous, never instant).
        self._encoder_restart_heals = False
        self._encoder_restart_delay_s = 0.3
        self._encoder_heal_at: float | None = None
        self._encoder_next_active_report_at = time.monotonic()
        self._encoder_pending_replies: list[tuple[float, bytes]] = []
        self._encoder_outstanding_polls = 0
        self.encoder_polls_handled = 0
        self.encoder_poll_responses_sent = 0
        self.encoder_active_reports_sent = 0
        self.encoder_max_outstanding_polls = 0
        self.encoder_restarts = 0
        self.encoder_version_requests = 0
        self.encoder_eeprom_reads: list[int] = []
        self.encoder_config_writes: list[tuple[int, int]] = []
        self.motor_enables_handled = 0
        self.motor_disables_handled = 0
        self._motor_disables_handled: dict[int, int] = {motor_id: 0 for motor_id in motor_ids}
        self._motor_resets_handled: dict[int, int] = {motor_id: 0 for motor_id in motor_ids}
        self._mit_commands_handled: dict[int, int] = {motor_id: 0 for motor_id in motor_ids}
        self.events: list[str] = []
        # Motors listed here ignore MIT position targets (physically stuck) and report
        # ``position +/- jitter``, alternating sign on every status frame. A jitter above
        # the driver's READY_MOVE_STUCK_POS_DELTA_RAD floor emulates a joint oscillating
        # against an external load (gravity sag), the issue #5 failure mode.
        self._stuck: dict[int, float] = {}
        self._stuck_phase: dict[int, bool] = {}
        self._last_torque: dict[int, float] = {motor_id: 0.0 for motor_id in motor_ids}
        self._last_command: dict[int, tuple[float, float, float, float]] = {}
        self._last_kd: dict[int, float] = {}
        self._torque_mobile: dict[int, float] = {}
        # Torque (Nm) reported in status frames instead of the default 0.0;
        # emulates a mechanically loaded joint for safety-escalation tests.
        self._reported_torque: dict[int, float] = {}
        self._reported_velocity: dict[int, float] = {}
        # Persistent firmware status fault returned by the selected motor.
        self._fault_status: dict[int, int] = {}
        # Startup-only fault latches cleared by the standard DM reset command.
        self._reset_clearable_faults: set[int] = set()
        # Per-motor MIT velocity/torque decode ranges (model-dependent).
        self._mit_ranges: dict[int, tuple[float, float, float, float]] = {
            motor_id: (
                (J4310_VEL_MIN, J4310_VEL_MAX, J4310_TOR_MIN, J4310_TOR_MAX)
                if motor_id in j4310_motors
                else (VEL_MIN, VEL_MAX, TOR_MIN, TOR_MAX)
            )
            for motor_id in motor_ids
        }
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._socket.bind((interface,))
        self._socket.settimeout(0.005)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="fake-dm-servo-bus", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._socket.close()

    def position(self, motor_id: int) -> float:
        with self._lock:
            return self.positions[motor_id]

    def set_position(self, motor_id: int, position: float) -> None:
        with self._lock:
            self.positions[motor_id] = position

    def set_temperature(self, motor_id: int, temperature_c: int) -> None:
        with self._lock:
            self._temperatures[motor_id] = temperature_c

    def set_stuck(self, motor_id: int, position: float, jitter: float = 0.0) -> None:
        """Makes the motor ignore position targets and report ``position +/- jitter``."""
        with self._lock:
            self.positions[motor_id] = position
            self._stuck[motor_id] = jitter
            self._stuck_phase[motor_id] = False

    def clear_stuck(self, motor_id: int) -> None:
        """Releases a stuck motor so it tracks position targets again."""
        with self._lock:
            self._stuck.pop(motor_id, None)
            self._stuck_phase.pop(motor_id, None)

    def set_torque_mobile(self, motor_id: int, rad_per_nm_per_frame: float = 0.045) -> None:
        """Makes zero-kp torque commands move the motor (viscous mobility model).

        Real DM servos accelerate under torque-only MIT frames; kp-gated
        teleporting cannot represent that, so torque-mode consumers (the
        monopi-style gripper spring) opt their motor into ``pos += torque *
        gain`` per command frame. Stuck motors stay stuck (a blocked gripper).
        """
        with self._lock:
            self._torque_mobile[motor_id] = rad_per_nm_per_frame

    def last_torque(self, motor_id: int) -> float:
        """Torque feedforward (Nm) carried by the most recent MIT command frame."""
        with self._lock:
            return self._last_torque[motor_id]

    def last_command(self, motor_id: int) -> tuple[float, float, float, float] | None:
        """Most recent MIT command as (position, velocity, kp, torque), or None."""
        with self._lock:
            return self._last_command.get(motor_id)

    def last_kd(self, motor_id: int) -> float | None:
        """Derivative gain carried by the most recent MIT command frame."""
        with self._lock:
            return self._last_kd.get(motor_id)

    def mit_command_count(self, motor_id: int) -> int:
        with self._lock:
            return self._mit_commands_handled[motor_id]

    def motor_disable_count(self, motor_id: int) -> int:
        with self._lock:
            return self._motor_disables_handled[motor_id]

    def motor_reset_count(self, motor_id: int) -> int:
        with self._lock:
            return self._motor_resets_handled[motor_id]

    def set_reported_torque(self, motor_id: int, torque_nm: float) -> None:
        """Makes the motor report ``torque_nm`` in every status frame (0 resets)."""
        with self._lock:
            if torque_nm == 0.0:
                self._reported_torque.pop(motor_id, None)
            else:
                self._reported_torque[motor_id] = torque_nm

    def set_reported_velocity(self, motor_id: int, velocity_rad_s: float) -> None:
        """Makes the motor report ``velocity_rad_s`` in every status frame (0 resets)."""
        with self._lock:
            if velocity_rad_s == 0.0:
                self._reported_velocity.pop(motor_id, None)
            else:
                self._reported_velocity[motor_id] = velocity_rad_s

    def set_fault_status(self, motor_id: int, status_code: int | None) -> None:
        """Makes a motor report a persistent DM status fault, or clears it with None."""
        with self._lock:
            self._reset_clearable_faults.discard(motor_id)
            if status_code is None:
                self._fault_status.pop(motor_id, None)
            else:
                self._fault_status[motor_id] = status_code & 0x0F

    def set_reset_clearable_fault_status(self, motor_id: int, status_code: int) -> None:
        """Makes a motor report a startup fault until it receives a standard reset."""
        with self._lock:
            self._fault_status[motor_id] = status_code & 0x0F
            self._reset_clearable_faults.add(motor_id)

    def set_trigger_rad(self, position_rad: float) -> None:
        """Sets the handle trigger displacement reported to the next poll."""
        with self._lock:
            self._encoder_ticks = round(position_rad * _ENCODER_TICKS_PER_REV / math.tau)

    def set_encoder_mute(self, muted: bool, *, restart_heals: bool = False) -> None:
        """Makes the handle encoder ignore polls (the intermittent firmware wedge).

        With ``restart_heals`` the wedge clears ``_encoder_restart_delay_s``
        after a REQ_RESTART frame arrives, emulating the firmware reboot.
        """
        with self._lock:
            self._encoder_muted = muted
            self._encoder_restart_heals = restart_heals
            self._encoder_heal_at = None

    def set_encoder_response_delay(self, delay_s: float) -> None:
        """Delays future poll replies without delaying configuration replies."""
        with self._lock:
            self._encoder_response_delay_s = delay_s

    def encoder_outstanding_polls(self) -> int:
        with self._lock:
            return self._encoder_outstanding_polls

    def encoder_frequencies(self) -> tuple[int, int]:
        with self._lock:
            return self._encoder_adc_frequency, self._encoder_report_frequency

    def set_buttons(self, top: bool, bottom: bool) -> None:
        """Sets the handle button states (digital_inputs bit 0 = top, bit 1 = bottom)."""
        with self._lock:
            self._encoder_buttons = (1 if top else 0) | (2 if bottom else 0)

    def _send_status(self, motor_id: int, error: int) -> None:
        with self._lock:
            pos = self.positions[motor_id]
            vel = self._reported_velocity.get(motor_id, 0.0)
            tor = self._reported_torque.get(motor_id, 0.0)
            temperature = self._temperatures[motor_id]
            error = self._fault_status.get(motor_id, error)
            vel_lo, vel_hi, tor_lo, tor_hi = self._mit_ranges[motor_id]
            if motor_id in self._stuck:
                phase = self._stuck_phase[motor_id]
                self._stuck_phase[motor_id] = not phase
                pos += self._stuck[motor_id] * (1.0 if phase else -1.0)
        payload = encode_status(
            motor_id,
            error,
            pos,
            vel,
            tor,
            temperature,
            vel_lo=vel_lo,
            vel_hi=vel_hi,
            tor_lo=tor_lo,
            tor_hi=tor_hi,
        )
        frame = _CAN_FRAME.pack(motor_id + _RESPONSE_ID_OFFSET, len(payload), payload)
        self._socket.send(frame)

    def _run(self) -> None:
        while not self._stop.is_set():
            self._flush_encoder_reports()
            try:
                raw = self._socket.recv(16)
            except TimeoutError:
                continue
            except OSError:
                return
            can_id, dlc, data = _CAN_FRAME.unpack(raw)
            can_id &= 0x1FFFFFFF
            self._handle_frame(can_id, data[:dlc])

    def _encoder_report_payload(self) -> bytes:
        return _ENCODER_RESPONSE.pack(
            self._encoder_device, self._encoder_ticks, 0, self._encoder_buttons
        )

    def _send_encoder_report(self, payload: bytes, *, poll_response: bool) -> None:
        if self._encoder_id is None:
            return
        frame = _CAN_FRAME.pack(
            self._encoder_id + _ENCODER_RESPONSE_ID_OFFSET, len(payload), payload
        )
        try:
            self._socket.send(frame)
        except OSError:
            return
        with self._lock:
            if poll_response:
                self.encoder_poll_responses_sent += 1
                self._encoder_outstanding_polls -= 1
            else:
                self.encoder_active_reports_sent += 1

    def _flush_encoder_reports(self) -> None:
        now = time.monotonic()
        due: list[bytes] = []
        active_payload: bytes | None = None
        with self._lock:
            remaining: list[tuple[float, bytes]] = []
            for due_at, payload in self._encoder_pending_replies:
                if due_at <= now:
                    due.append(payload)
                else:
                    remaining.append((due_at, payload))
            self._encoder_pending_replies = remaining

            if (
                self._encoder_id is not None
                and not self._encoder_muted
                and self._encoder_report_frequency > 0
                and now >= self._encoder_next_active_report_at
            ):
                active_payload = self._encoder_report_payload()
                self._encoder_next_active_report_at = now + 1.0 / self._encoder_report_frequency

        for payload in due:
            self._send_encoder_report(payload, poll_response=True)
        if active_payload is not None:
            self._send_encoder_report(active_payload, poll_response=False)

    def _send_encoder_config_reply(self, payload: bytes) -> None:
        if self._encoder_id is None:
            return
        frame = _CAN_FRAME.pack(self._encoder_id, len(payload), payload)
        self._socket.send(frame)

    @staticmethod
    def _decode_encoder_frequency(data: bytes) -> int:
        if len(data) == 3:
            return data[2]
        return (data[2] << 8) | data[3]

    def _encoder_eeprom_byte(self, offset: int) -> int | None:
        if offset == _ENCODER_ADC_FREQUENCY_LOW_OFFSET:
            return self._encoder_adc_frequency & 0xFF
        if offset == _ENCODER_ADC_FREQUENCY_HIGH_OFFSET:
            return 0xFF if self._encoder_adc_frequency <= 0xFF else self._encoder_adc_frequency >> 8
        if offset == _ENCODER_REPORT_FREQUENCY_LOW_OFFSET:
            return self._encoder_report_frequency & 0xFF
        if offset == _ENCODER_REPORT_FREQUENCY_HIGH_OFFSET:
            return (
                0xFF
                if self._encoder_report_frequency <= 0xFF
                else self._encoder_report_frequency >> 8
            )
        return None

    def _handle_frame(self, can_id: int, data: bytes) -> None:
        is_encoder_poll = self._encoder_id is not None and can_id == self._encoder_id
        if is_encoder_poll and data[:2] == bytes([0xFF, _ENCODER_GET_VERSION]):
            with self._lock:
                self.encoder_version_requests += 1
                firmware = self._encoder_firmware
                if firmware is not None:
                    self.events.append("encoder-version")
            if firmware is not None:
                self._send_encoder_config_reply(
                    bytes([self._encoder_device, _ENCODER_GET_VERSION | 0x80, *firmware])
                )
            return
        if is_encoder_poll and len(data) >= 3 and data[1] == _ENCODER_GET_EEPROM:
            with self._lock:
                offset = data[2]
                value = self._encoder_eeprom_byte(offset)
                self.encoder_eeprom_reads.append(offset)
                self.events.append(f"encoder-eeprom-read-{offset}")
            if value is not None:
                self._send_encoder_config_reply(
                    bytes([self._encoder_device, _ENCODER_READINGS_RESPONSE, 0, value, 0])
                )
            return
        if (
            is_encoder_poll
            and len(data) >= 3
            and data[1] in (_ENCODER_SET_ADC_FREQUENCY, _ENCODER_SET_REPORT_FREQUENCY)
        ):
            value = self._decode_encoder_frequency(data)
            with self._lock:
                if data[1] == _ENCODER_SET_ADC_FREQUENCY:
                    self._encoder_adc_frequency = value
                    event = "encoder-write-adc"
                else:
                    self._encoder_report_frequency = value
                    self._encoder_next_active_report_at = time.monotonic()
                    event = "encoder-write-report"
                self.encoder_config_writes.append((data[1], value))
                self.events.append(event)
            return
        if is_encoder_poll and data[:2] == _ENCODER_RESTART:
            with self._lock:
                self.encoder_restarts += 1
                self.events.append("encoder-restart")
                if self._encoder_muted and self._encoder_restart_heals:
                    self._encoder_heal_at = time.monotonic() + self._encoder_restart_delay_s
            return
        if is_encoder_poll and data[:2] == _ENCODER_POLL:
            with self._lock:
                if self._encoder_muted:
                    if (
                        self._encoder_heal_at is not None
                        and time.monotonic() >= self._encoder_heal_at
                    ):
                        self._encoder_muted = False
                        self._encoder_heal_at = None
                    else:
                        return
                # Real handle firmware reports device_id=0 in the payload
                # regardless of the encoder's CAN id (observed on hardware).
                payload = self._encoder_report_payload()
                self.encoder_polls_handled += 1
                self._encoder_outstanding_polls += 1
                self.encoder_max_outstanding_polls = max(
                    self.encoder_max_outstanding_polls, self._encoder_outstanding_polls
                )
                self._encoder_pending_replies.append(
                    (time.monotonic() + self._encoder_response_delay_s, payload)
                )
            return

        if can_id == _BROADCAST_ID or can_id not in self.positions or len(data) < 8:
            # Register writes (mode setup etc.) need no acknowledgement.
            return
        self.frames_handled += 1
        motor_id = can_id

        if data[:7] == b"\xff" * 7:
            command = data[7]
            if command == 0xFC:  # enable
                with self._lock:
                    self.motor_enables_handled += 1
                    self.events.append("motor-enable")
                self._send_status(motor_id, error=1)
            elif command == 0xFD:  # disable: error nibble must be 0 to match normal_code
                with self._lock:
                    self.motor_disables_handled += 1
                    self._motor_disables_handled[motor_id] += 1
                    self.events.append("motor-disable")
                self._send_status(motor_id, error=0)
            elif command == 0xFE:  # set zero position
                with self._lock:
                    self.positions[motor_id] = 0.0
                self._send_status(motor_id, error=1)
            elif command == 0xFB:  # reset
                with self._lock:
                    self._motor_resets_handled[motor_id] += 1
                    if motor_id in self._reset_clearable_faults:
                        self._fault_status.pop(motor_id, None)
                        self._reset_clearable_faults.discard(motor_id)
                self._send_status(motor_id, error=1)
            return

        # MIT command frame: track the target instantly and report back. Real DM
        # servos only act on the position field when kp > 0 -- zero-gain frames are
        # how the driver polls status (and how torque/damping modes command), so
        # they must not move the servo. Stuck motors keep their position regardless.
        vel_lo, vel_hi, tor_lo, tor_hi = self._mit_ranges[motor_id]
        target_pos, target_vel, kp, target_tor = decode_mit_command(
            data, vel_lo=vel_lo, vel_hi=vel_hi, tor_lo=tor_lo, tor_hi=tor_hi
        )
        kd_u = (data[5] << 4) | (data[6] >> 4)
        kd = uint_to_float(kd_u, KD_MIN, KD_MAX, 12)
        with self._lock:
            self._mit_commands_handled[motor_id] += 1
            self._last_torque[motor_id] = target_tor
            self._last_command[motor_id] = (target_pos, target_vel, kp, target_tor)
            self._last_kd[motor_id] = kd
            if motor_id not in self._stuck:
                if kp > 0.0:
                    self.positions[motor_id] = target_pos
                elif motor_id in self._torque_mobile:
                    self.positions[motor_id] += target_tor * self._torque_mobile[motor_id]
        self._send_status(motor_id, error=1)


def main() -> None:
    import argparse
    import signal

    parser = argparse.ArgumentParser(description="Fake DM servo bus for SIL testing")
    parser.add_argument("interface", help="SocketCAN interface, e.g. vcan0")
    parser.add_argument("--motors", type=int, default=6, help="number of servo ids (1..N)")
    args = parser.parse_args()

    bus = FakeDmServoBus(args.interface, tuple(range(1, args.motors + 1)))
    bus.start()
    signal.sigwait({signal.SIGINT, signal.SIGTERM})
    bus.stop()


if __name__ == "__main__":
    main()
