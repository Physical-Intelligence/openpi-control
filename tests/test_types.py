import numpy as np
import pytest

from openpi_control import (
    ArmMode,
    ArmRole,
    ArmState,
    ConfigurationError,
    EffectorState,
    InputState,
    JointState,
    PositionCommand,
)


def test_state_arrays_are_copied_and_read_only() -> None:
    source = np.array([1.0, 2.0])
    joints = JointState(
        names=("a", "b"),
        position_rad=source,
        velocity_rad_s=[0, 0],
        effort_nm=[0, 0],
        temperature_c=[20, 20],
        current_a=[0, 0],
    )
    source[0] = 99
    assert joints.position_rad.tolist() == [1.0, 2.0]
    with pytest.raises(ValueError):
        joints.position_rad[0] = 2


def test_state_freshness_uses_monotonic_time() -> None:
    joints = JointState(("a",), [0], [0], [0], [0], [0])
    state = ArmState("a", ArmRole.FOLLOWER, joints, None, 0, 0, 1, ArmMode.HOLD)
    assert not state.is_fresh(0.01)


def test_joint_state_frame_age_defaults_to_unknown_and_validates_size() -> None:
    joints = JointState(("a", "b"), [0, 0], [0, 0], [0, 0], [20, 20], [0, 0])
    assert joints.frame_age_ms is not None
    assert joints.frame_age_ms.tolist() == [-1.0, -1.0]

    explicit = JointState(
        ("a", "b"), [0, 0], [0, 0], [0, 0], [20, 20], [0, 0], frame_age_ms=[3.0, 250.0]
    )
    assert explicit.frame_age_ms is not None
    assert explicit.frame_age_ms.tolist() == [3.0, 250.0]

    with pytest.raises(ConfigurationError):
        JointState(("a", "b"), [0, 0], [0, 0], [0, 0], [20, 20], [0, 0], frame_age_ms=[3.0])

    assert EffectorState(0.5).frame_age_ms == -1.0


def test_command_and_effector_validation() -> None:
    with pytest.raises(ConfigurationError):
        PositionCommand([float("nan")])
    with pytest.raises(ConfigurationError):
        EffectorState(1.01)


def test_input_state_named_access_and_validation() -> None:
    inputs = InputState(
        button_names=("stick", "upper", "lower"),
        buttons=(False, True, False),
        axis_names=("stick_x", "stick_y", "trigger"),
        axes=[0.5, -1.0, 0.0],
        monotonic_timestamp=0.0,
        wall_timestamp=0.0,
        sequence=1,
    )
    assert inputs.button("upper") and not inputs.button("lower")
    assert inputs.axis("stick_x") == 0.5
    assert not inputs.is_fresh(0.01)
    with pytest.raises(ValueError):
        inputs.axes[0] = 2
    with pytest.raises(ConfigurationError):
        inputs.button("missing")
    with pytest.raises(ConfigurationError):
        inputs.axis("missing")
    buttons_only = InputState(("top", "bottom"), (True, False), (), [], 0.0, 0.0, 1)
    assert buttons_only.button("top") and not buttons_only.button("bottom")
    assert buttons_only.axes.size == 0
    with pytest.raises(ConfigurationError):
        InputState((), (), (), [], 0.0, 0.0, 1)
    with pytest.raises(ConfigurationError):
        InputState(("a",), (), (), [], 0.0, 0.0, 1)
    with pytest.raises(ConfigurationError):
        InputState(("a",), (True,), ("x",), [], 0.0, 0.0, 1)
