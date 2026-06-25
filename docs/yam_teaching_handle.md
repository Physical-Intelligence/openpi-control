# YAM teaching handle: native support specification

Status: **implemented in `pi_control_node`** (`ServoCanPassiveEncoder`,
servo model `"CAN Passive Encoder"`). The Python API surface (`InputState`,
`LeaderArm.read_inputs`, the `E_Yam_Handle` input layout) was already complete;
this document captures the handle protocol from the i2rt reference
implementation (`i2rt/motor_drivers/dm_driver.py`, `PassiveEncoderReader`) and
now describes the shipped behavior.

## Hardware

The YAM leader's teaching handle is a passive device on the same CAN bus as
the arm motors. It carries:

- a trigger encoder (used as the leader's effector/gripper source), and
- two discrete buttons (top and bottom).

There is no joystick and no actively driven gripper on the leader.

## CAN protocol (request/response poll)

- Required startup configuration (mirrors i2rt
  `PassiveJointEncoder.validate_encoders`):
  - Query firmware with `[0xFF, 0x03]` on the request CAN id and require
    version `>=2.2.12`.
  - Read ADC frequency from EEPROM offsets `27` (high) and `8` (low), and
    report frequency from offsets `28` (high) and `25` (low), using
    `[device_id, 0x07, offset]`.
  - Write ADC frequency `255` with `[0xFF, 0x04, 0xFF]` and passive report
    frequency `0` with `[0xFF, 0x01, 0x00]`, but only when the corresponding
    EEPROM value is mismatched.
  - Re-read corrected values and drain queued frames before starting normal
    reception or enabling any arm motors. Missing, old, or uncorrectable
    firmware is a hard startup failure.
- Request: send a 2-byte frame `[0xFF, 0x02]` to the encoder's CAN id
  (arbitration id = encoder id; the response arrives on the same id, or id+1
  depending on firmware receive mode — i2rt default is `plus_one`). YAM
  handles ship with the fixed encoder id `0x50E` (request) / `0x50F`
  (response) — i2rt `encoder_manager.py` `REQ`/`REPORT`.
- Response payload, big-endian `!B h h B` (6 bytes):

  | field          | type | meaning                                        |
  |----------------|------|------------------------------------------------|
  | device_id      | u8   | logical device id — observed 0 on hardware regardless of CAN id; do not validate (i2rt ignores it) |
  | position       | i16  | encoder ticks; radians = ticks * 2π / 4096     |
  | velocity       | i16  | ticks/s; rad/s = ticks * 2π / 4096             |
  | digital_inputs | u8   | bit 0 = button 0 (top), bit 1 = button 1 (bottom) |

- Trigger normalization (i2rt convention): clip position to ±`range_rad`
  (0.7 rad) and map `|position| / range_rad` to `[0, 1]`. The i2rt consumer
  then inverts (`1.0 - position`) to make the value open-positive, and divides
  by a per-handle `trigger_closed_position` calibration scalar before
  clipping. In OpenPI both live natively: the inversion is `open_at_min` on the
  effector model, and the travel saturation is the instance config's
  `pos_max` — physical triggers only reach ~0.9 of the encoder's 0.7 rad
  range, so `E_Yam_Handle_01.json` ships `pos_max: 0.63` (= 0.9 × 0.7) and a
  full squeeze clips to exactly 0.0 (closed). Saturating at the source keeps
  every consumer consistent: the native live-forwarding path (engaged
  teleop), DAgger takeover commands, and recorded actions all see the same
  value. openpi-crust's `trigger_to_gripper` divisor
  (`leader_trigger_closed_positions`) therefore stays at 1.0; it remains
  available as an additional per-cell knob for handles whose travel drifts
  from the shipped calibration.

## Native implementation

`ServoCanPassiveEncoder` (`native/pi_control/src/pi_servo_can_encoder.cpp`)
implements the servo model `"CAN Passive Encoder"`:

- **Startup handshake.** `DriverArx::open()` validates every registered
  passive encoder on the open CAN socket before it starts the reception
  thread. Firmware must be `>=2.2.12`; ADC/report frequencies must read back
  as `255`/`0`. Only mismatched EEPROM values are written. Any validation or
  correction failure aborts startup before the first motor-enable frame.
- **Polling.** `read_hardware_values()` snapshots the driver cache and
  advances a bounded poll state machine. At most one poll is outstanding,
  normal requests are capped at i2rt's 250 Hz cadence (4 ms minimum spacing),
  and a request that misses its 10 ms response deadline triggers a 500 ms
  retry backoff. This remains correct when generic effector code reads the
  cached joint more than once per control loop. `DriverArx` routes responses
  by an explicitly registered CAN id (`response_can_id`, default `id + 1`
  for the i2rt `plus_one` firmware receive mode) ahead of the DM/ENCOS
  heuristics.
- **Read-only.** The servo never sends enable/MIT/zero frames; `move()` and
  `apply_torque()` are no-ops, and `start_hardware()` proves presence purely
  by polling (fail-fast `NO_RESPONSE` when the handle is absent).
- **Trigger frame conventions.** The cached position keeps the encoder's sign;
  the servo reads the displacement magnitude (`|ticks| * 2π/4096`), so both
  squeeze directions land on the `[pos_min, pos_max]` joint range
  (`[0, 0.63]` rad in `E_Yam_Handle_01.json` — per-handle travel calibration
  lives there). `open_at_min: true` on the effector model flips the published
  normalized value so released reads 1.0 (open) and full squeeze 0.0.
- **Buttons.** Bits 0/1 of `digital_inputs` publish through
  `MsgJoystick.button_[0..1]` on `--topic_joystick` (`ZmqJoystickInfo`,
  44-byte ABI decoded by
  `openpi_control.protocol.decode_inputs`), decoding as `("top", "bottom")`.
  Buttons publish only when a fresh encoder response reaches the cache; a
  repeated cached read never duplicates the previous button publication.
- **Loss detection.** Silence is measured by wall clock, independent of
  control/read frequency. The cached trigger is held while silent, a warning
  is logged at 2 s, one firmware restart (`[0xFF, 0x0F]`) is sent at 3 s, and
  `SAFE_MODE_SIG` is raised at 20 s if the encoder never returns.

SIL coverage in `tests/sil/test_native_sil.py` runs native leader sessions
against `tests/sil/fake_dm_servo.py`, covering startup correction/order,
already-correct and unsupported firmware, delayed replies, trigger sweep,
button edges, mute/restart recovery, and continuous polling.
