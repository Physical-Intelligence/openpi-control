import pytest

from openpi_control import (
    ArmConfig,
    ArmMode,
    ArmRole,
    ArmState,
    EffectorState,
    FrankaConnection,
    JointState,
    PositionCommand,
    RobotiqConnection,
    SocketCanConnection,
    VelocityCommand,
)
from openpi_control.native import NativeArmBackend
from openpi_control.protocol import (
    DROID_COMMAND_STRUCT,
    DROID_MAGIC,
    DROID_STATE_STRUCT,
    JOINT_STRUCT,
    STATUS_STRUCT,
    DroidControlMode,
    NativeStatus,
)


def test_native_arm_only_command_preserves_configured_effector() -> None:
    backend = NativeArmBackend()
    backend._config = ArmConfig(
        "follower", "Yam", SocketCanConnection("test"), effector_model="E_Yam"
    )
    backend._state = ArmState(
        "follower",
        ArmRole.FOLLOWER,
        JointState(("1", "2", "3", "4", "5", "6"), [0] * 6, [0] * 6, [0] * 6, [20] * 6, [0] * 6),
        EffectorState(0.75),
        1.0,
        1.0,
        1,
        ArmMode.HOLD,
    )

    values = JOINT_STRUCT.unpack(backend._encode_joint_command(PositionCommand([0.1] * 6)))

    assert values[:7] == pytest.approx([0.1] * 6 + [0.75])
    assert values[61] == 7
    backend.close()


def test_native_ready_status_is_tracked() -> None:
    backend = NativeArmBackend()
    backend._config = ArmConfig("follower", "Yam", SocketCanConnection("test"))
    backend._role = ArmRole.FOLLOWER
    payload = STATUS_STRUCT.pack(int(NativeStatus.READY), 0, 0, *([0.0] * 10), *([0] * 10))

    backend._consume_status(payload)

    assert backend._ready
    backend.close()


def test_native_droid_command_carries_effector_controls() -> None:
    backend = NativeArmBackend()
    backend._config = ArmConfig(
        "droid",
        "Franka",
        FrankaConnection("172.16.0.2"),
        effector_model="Robotiq",
        effector_connection=RobotiqConnection("/dev/ttyUSB0"),
    )
    payload = backend._encode_joint_command(
        PositionCommand([0.1] * 7, effector=0.0, effector_speed=0.5, effector_force=0.8)
    )
    values = DROID_COMMAND_STRUCT.unpack(payload)
    assert values[0] == DROID_MAGIC
    assert values[7:14] == pytest.approx([0.1] * 7)
    assert values[-3:] == pytest.approx((0.0, 0.5, 0.8))
    backend.close()


def test_native_droid_velocity_command_uses_velocity_mode() -> None:
    backend = NativeArmBackend()
    backend._config = ArmConfig(
        "droid",
        "Franka",
        FrankaConnection("172.16.0.2"),
        effector_model="Robotiq",
        effector_connection=RobotiqConnection("/dev/ttyUSB0"),
    )
    payload = backend._encode_joint_command(
        VelocityCommand([0.1] * 7, effector=0.0, effector_speed=0.5, effector_force=0.8)
    )
    values = DROID_COMMAND_STRUCT.unpack(payload)
    assert values[3] == DroidControlMode.JOINT_VELOCITY
    assert values[14:21] == pytest.approx([0.1] * 7)
    backend.close()


def test_native_droid_state_maps_diagnostics_and_gripper_target() -> None:
    backend = NativeArmBackend()
    backend._config = ArmConfig(
        "droid",
        "Franka",
        FrankaConnection("172.16.0.2"),
        effector_model="Robotiq",
        effector_connection=RobotiqConnection("/dev/ttyUSB0"),
    )
    backend._role = ArmRole.FOLLOWER
    backend._consume_state(
        DROID_STATE_STRUCT.pack(
            DROID_MAGIC,
            1,
            1,
            15,
            10,
            20,
            1.5,
            *([0.1] * 7),
            *([0.2] * 7),
            *([0.3] * 7),
            *([0.4] * 7),
            *([0.5] * 7),
            *([0.6] * 6),
            0.7,
            0.8,
            0.0,
            0.0,
            1.2,
            0.25,
            2,
            1,
            4,
            0.99,
        )
    )
    state = backend.latest_state()
    assert state is not None and state.diagnostics is not None and state.effector is not None
    assert state.effector.position == pytest.approx(0.7)
    assert state.effector.connected
    assert state.effector.activated
    assert state.effector.faulted
    assert state.diagnostics.effector_target == pytest.approx(0.25)
    assert state.diagnostics.hardware_faulted
    assert state.diagnostics.joint_contact[0]
    assert state.diagnostics.joint_collision[2]
    backend.close()
