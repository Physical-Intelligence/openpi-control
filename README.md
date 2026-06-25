# openpi-control

`openpi-control` is a native C++ robot-control library. Each arm
runs one `pi_control_node` process that communicates with the main
Python process via ZeroMQ.

| Arm | Follower effector | Leader effector |
| --- | --- | --- |
| `Yam` | `E_Yam` | `E_Yam_Handle` |
| `ARX_L5` | `E_ARX` | `E_ARX` |
| `ARX_X5` | `E_ARX` | `E_ARX` |

```python
from openpi_control import ArmConfig, ArmSession, PositionCommand, SocketCanConnection

with ArmSession() as session:
    follower = session.add_follower(
        ArmConfig(
            "right_follower",
            "Yam",
            SocketCanConnection("can_follower_r"),
            effector_model="E_Yam",
            follower_gravity_compensation=True,
        )
    )
    session.connect()
    follower.command(PositionCommand([0, 0, 0, 0, 0, 0], 1.0))
```

## Building from source

Building requires CMake, a C++17 compiler, Eigen, Boost, ZeroMQ,
cppzmq, and Pinocchio. It has only been tested on Ubuntu 22.04.

```bash
sudo ./scripts/install_build_deps_ubuntu.sh
./scripts/build_deps.sh
uv build --wheel
```

The wheel is written to `dist/`.
