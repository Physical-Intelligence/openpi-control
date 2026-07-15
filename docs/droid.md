# DROID Franka and Robotiq control

`openpi-control` owns DROID hardware through the same `ArmSession` and
`FollowerArm` API as the other supported arms. One `pi_control_node` process
owns both the Franka FCI connection and the optional Robotiq serial port.

## Runtime architecture

```text
ArmSession / FollowerArm
  -> NativeArmBackend (ZMQ, lifecycle heartbeat, process ownership)
    -> pi_control_node --device_type franka
      -> DeviceFranka (DeviceArm)
        -> DriverFranka: libfranka 1 kHz torque callback
        -> DeviceRobotiq (DeviceEffector)
          -> RobotiqTransport: independent 50 Hz Modbus worker
```

The Python command frequency does not set the Franka servo frequency. Policy
targets may arrive at 15 Hz while libfranka continues at 1 kHz. Position
targets hold; velocity targets expire to a measured-position hold after 250 ms.
There is no Python-side interpolation, smoothing, or rate limit.

## Dependencies

The DROID control computer reports robot-server protocol version 6. Use
libfranka `>=0.10.0,<0.13.3`; `0.13.2` is the validated version. Newer releases
speak a newer protocol and are rejected by this controller. Install libmodbus
for a physical Robotiq gripper.

```bash
sudo apt-get install libmodbus-dev
cmake -S . -B build \
  -DOPENPI_CONTROL_WITH_FRANKA=ON \
  -DOPENPI_CONTROL_WITH_MODBUS=ON \
  -DFranka_DIR=/path/to/libfranka-0.13.2/lib/cmake/Franka
cmake --build build -j
```

`OPENPI_CONTROL_MOCK_FRANKA=ON` and `OPENPI_CONTROL_MOCK_ROBOTIQ=ON` build
synthetic device paths without either hardware library. Real and corresponding
mock options should not be enabled together.

The user running the node needs access to the Robotiq serial device, normally
through the `dialout` group, and permission to use real-time scheduling when
`FrankaRealtimeConfig.ENFORCE` is selected. Both entrypoints default to
`IGNORE`, matching the non-PREEMPT_RT DROID laptop deployment.

## Python usage

```python
from openpi_control import (
    ArmConfig,
    ArmSession,
    FrankaConnection,
    PositionCommand,
    RobotiqConnection,
)

config = ArmConfig(
    name="droid",
    model="Franka",
    connection=FrankaConnection("172.16.0.2"),
    effector_model="Robotiq",
    effector_connection=RobotiqConnection("/dev/ttyUSB0"),
    control_frequency_hz=100,
)

with ArmSession() as session:
    robot = session.add_follower(config)
    session.connect()             # Holds the measured pose; does not go home.
    robot.activate_effector()     # Robotiq reset/calibration sequence.
    robot.move_to_ready()         # Blocking DROID reset pose.
    robot.resume()                # Explicitly release any prior hold latch.
    robot.command(
        PositionCommand(
            [0.0, -0.63, 0.0, -2.51, 0.0, 1.88, 0.0],
            effector=0.0,         # 0 closed, 1 open.
            effector_speed=1.0,
            effector_force=1.0,
        )
    )
```

Use `FrankaConnection(..., read_only=True)` for state-only acceptance tests.
That mode never starts the torque callback and advertises no direct-command,
reset, or recovery capability.

## Controller behavior

The native callback matches the DROID Polymetis behavior:

- Position mode uses hybrid joint impedance
  `(J' Kx J + Kq)(q_des-q) + (J' Kxd J + Kqd)(-dq) + coriolis`.
- Velocity mode initializes its position reference from measured joints when
  entered and applies `q_des += dq_des / 1000` every callback.
- The last target remains active until a new command, hold, reset, fault, or
  heartbeat timeout.
- DROID Cartesian, joint-position, and joint-velocity soft limits contribute
  safety torque before the final per-joint torque clamp.
- libfranka collision behavior and command-rate limiting remain enabled.

The reset pose and safety envelope live in `FrankaConnection`, and are passed
to native code when the node starts. The controller does not fetch dynamics or
kinematics from packaged JSON or URDF; libfranka remains their source of truth.

## Robotiq behavior

The default calibration matches the DROID setup: raw `3` is open and raw
`230` is closed. Public state and commands use `0=closed, 1=open`. Modbus runs
outside the Franka callback. A positive force request is always encoded as a
nonzero force byte, and close is reasserted after contact so firmware automatic
re-grasp remains active instead of accepting the first contact width as done.

## Safety and lifecycle

Connecting is passive at the application level: the arm starts holding its
measured position and does not move to the reset pose. `move_to_ready()` is an
explicit blocking operation. `recover()` invokes Franka automatic error
recovery without first moving home. `activate_effector()` affects only the
Robotiq device.

Command handoff uses a latest-value mailbox; the callback uses `try_lock` and
retains its previous snapshot for one cycle if a writer is active. State
publication is also best-effort under `try_lock`, so no Python or transport
thread can block the libfranka callback on an OS mutex.

The existing native heartbeat watchdog is reused. If the Python owner dies,
the node enters hold and stops active gripper motion. Process shutdown joins
the Robotiq worker and libfranka controller thread before exiting.

Hardware acceptance should proceed in this order:

1. Connect with `read_only=True` and verify seven joint states.
2. Connect the Robotiq alone, activate it, and verify open/closed calibration.
3. Connect in control mode and verify measured-position hold.
4. Run a blocking reset with an operator on the E-stop.
5. Send 15 Hz targets and verify the 1 kHz callback remains healthy.
6. Kill the Python owner and verify arm hold and gripper stop.
