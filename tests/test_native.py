import pytest

from openpi_control import (
    ArmConfig,
    ArmMode,
    ArmRole,
    ArmState,
    EffectorState,
    JointState,
    PositionCommand,
    SocketCanConnection,
)
from openpi_control.native import NativeArmBackend
from openpi_control.protocol import JOINT_STRUCT, STATUS_STRUCT, NativeStatus


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
    assert values[51] == 7
    backend.close()


def test_native_ready_status_is_tracked() -> None:
    backend = NativeArmBackend()
    backend._config = ArmConfig("follower", "Yam", SocketCanConnection("test"))
    backend._role = ArmRole.FOLLOWER
    payload = STATUS_STRUCT.pack(int(NativeStatus.READY), 0, 0, *([0.0] * 10), *([0] * 10))

    backend._consume_status(payload)

    assert backend._ready
    backend.close()
