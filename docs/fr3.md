# FR3 and Robotiq control

`openpi-control` owns one Franka Emika FR3 and one Robotiq 2F gripper through
the same `ArmSession` and `FollowerArm` API used by the other follower arms.
One `pi_control_node` process owns both hardware connections.

## Requirements

- libfranka is pinned to **0.21.3** and built from source by
  `scripts/build_deps.sh`.
- The FR3 must run Robot System version **5.9.0 or newer** (robot server
  protocol 10).
- The host must be able to reach the FR3 controller over Ethernet.
- The gripper must use either true Modbus RTU over a serial device or true
  Modbus TCP. RTU-over-TCP gateways are not treated as serial devices.

A real-time kernel is not required. The driver always constructs libfranka
with `franka::RealtimeConfig::kIgnore`; the torque callback still runs at the
1 kHz rate owned by libfranka.

## Python configuration

For serial RTU:

```python
from openpi_control import ArmConfig, FR3Connection, RobotiqConnection

config = ArmConfig(
    "follower",
    "FR3",
    FR3Connection("192.168.1.10"),
    effector_model="Robotiq",
    effector_connection=RobotiqConnection.rtu(
        "/dev/serial/by-id/usb-robotiq",
        baud_rate=115200,
        slave_id=9,
    ),
)
```

For Modbus TCP, only the gripper connection changes:

```python
effector_connection=RobotiqConnection.tcp("192.168.1.11", port=502)
```

The public gripper convention is `0.0 = fully closed` and `1.0 = fully open`.
Raw Robotiq register calibration defaults to 3 (open) and 230 (closed).

Connecting is passive: the arm holds its measured pose and the gripper is not
activated. `move_to_ready()` performs internal FR3 error recovery, moves to the
configured seven-joint reset pose, activates the gripper, and waits for it to
open fully. There are no
FR3-specific activation, recovery, velocity-command, read-only, or synthetic
backend APIs.

```python
from openpi_control import ArmSession, PositionCommand

with ArmSession() as session:
    follower = session.add_follower(config)
    session.connect()
    follower.move_to_ready()
    follower.command(
        PositionCommand(
            [0.0, -0.6283185307, 0.0, -2.5132741229, 0.0, 1.8849555922, 0.0],
            1.0,
        )
    )
```

## Native controller

Policy targets may arrive at a lower frequency while libfranka continues its
1 kHz torque callback. Position targets remain active until replaced or held.
The controller retains the ported, tuned hybrid joint/Cartesian impedance
gains:

- Cartesian stiffness: `400, 400, 400, 15, 15, 15`
- Cartesian damping: `37, 37, 37, 2, 2, 2`
- Joint stiffness: `40, 30, 50, 25, 35, 25, 10`
- Joint damping: `4, 6, 5, 5, 3, 2, 1`

Joint, velocity, Cartesian, torque, collision, owner-liveness, and command
shape checks remain active. libfranka supplies the robot dynamics and
kinematics, so the FR3 configuration does not require a packaged URDF.

The dependency builder produces static `libfranka.a` and `libmodbus.a`
archives. Both are linked directly into the packaged `pi_control_node`, just
like the existing pinned native dependencies.

## Hardware acceptance

Perform the first actuating checks with the workspace clear and an operator at
the E-stop:

1. Connect and verify seven joint states plus one gripper state without motion.
2. Verify the RTU or TCP gripper endpoint and the open/closed convention.
3. Call `move_to_ready()` and verify the blocking reset motion and activation.
4. Send low-rate position targets and verify the 1 kHz callback remains healthy.
5. Terminate the Python owner and verify that the arm holds and gripper motion stops.
