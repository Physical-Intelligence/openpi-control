import json
from pathlib import Path

import pytest

from openpi_control import ArmConfig, ConfigurationError, InputLayout, SocketCanConnection
from openpi_control.config import SUPPORTED_MODELS
from openpi_control.protocol import topics_for


@pytest.mark.parametrize("model", SUPPORTED_MODELS)
def test_physical_model_catalog_is_complete(model: str) -> None:
    config = ArmConfig("arm", model, SocketCanConnection("test"))
    assets = config.resolve_assets()
    assert assets.model_config.is_file()
    assert assets.instance_config.is_file()
    assert assets.urdf.is_file()
    assert len(config.joint_names()) == 6
    assert json.loads(assets.model_config.read_text())["algo_type"] == "Pinocchio"


def test_logical_identity_is_separate_from_instance_config(tmp_path: Path) -> None:
    canonical = (
        ArmConfig("source", "Yam", SocketCanConnection("test")).resolve_assets().instance_config
    )
    copied = tmp_path / "calibration.json"
    copied.write_bytes(canonical.read_bytes())
    config = ArmConfig("left_follower", "Yam", SocketCanConnection("test"), instance_config=copied)
    assert config.name == "left_follower"
    assert config.resolve_assets().instance_config == copied


def test_names_and_topics_are_isolated() -> None:
    with pytest.raises(ConfigurationError):
        ArmConfig("not valid", "Yam", SocketCanConnection("test"))
    assert topics_for("session", "left").state != topics_for("session", "right").state
    assert topics_for("one", "left").status != topics_for("two", "left").status


def test_input_layout_is_derived_from_handle_configuration() -> None:
    yam = ArmConfig("leader", "Yam", SocketCanConnection("test"), effector_model="E_Yam")
    assert yam.input_layout() == InputLayout()
    handle = ArmConfig("leader", "Yam", SocketCanConnection("test"), effector_model="E_Yam_Handle")
    assert handle.input_layout().button_names == ("top", "bottom")
    assert handle.input_layout().axis_names == ()
    assert not ArmConfig("leader", "Yam", SocketCanConnection("test")).input_layout().has_inputs
    assert topics_for("session", "leader").inputs == "openpi.session.leader.inputs"


def test_yam_gripper_uses_fast_motor_side_force_bound() -> None:
    config = ArmConfig("follower", "Yam", SocketCanConnection("test"), effector_model="E_Yam")
    assets = config.resolve_assets()
    assert assets.effector_model_config is not None
    assert assets.effector_instance_config is not None
    model = json.loads(assets.effector_model_config.read_text())
    instance = json.loads(assets.effector_instance_config.read_text())
    joint = model["joints"][0]
    servo = joint["servos"][0]

    assert joint["vel_max"] == 3.2
    assert joint["grip_torque_limit"] == 1.11
    assert "grip_closing_position_error" not in joint
    assert "pos_max_safety_margin" not in joint
    assert joint["normalized_pos_min"] == 0.0
    assert joint["normalized_pos_max"] == 4.5
    assert servo["pos_kp"] == 2.5
    assert servo["pos_kd"] == 0.1
    assert instance["joints"][0]["servos"][0]["pos_min"] == 0.0
    assert instance["joints"][0]["servos"][0]["pos_max"] == 5.4
    assert instance["joints"][0]["servos"][0]["position_wrap_period"] == pytest.approx(
        2.0 * 3.141592653589793
    )
    assert instance["control_mode"] == "position"
    assert instance["dist_to_torque_const"] == 6.67
    assert "grip_spring_offset" not in instance


def test_yam_uses_monopi_style_follower_tracking_constants() -> None:
    config = ArmConfig("follower", "Yam", SocketCanConnection("test"))
    model = json.loads(config.resolve_assets().model_config.read_text())

    assert [joint["follow_vel_max"] for joint in model["joints"]] == [2.5, 2.6, 2.8, 6.0, 6.0, 6.0]
    assert [joint["follow_viscous_damping"] for joint in model["joints"]] == pytest.approx(
        [0.7777778, 0.7777778, 0.7777778, 0.0, 0.0, 0.0]
    )
    assert [joint["vel_max"] for joint in model["joints"]] == [2.0] * 6


def test_arx_x5_uses_yam_style_follower_velocity_limits() -> None:
    config = ArmConfig("follower", "ARX_X5", SocketCanConnection("test"))
    model = json.loads(config.resolve_assets().model_config.read_text())

    assert [joint["follow_vel_max"] for joint in model["joints"]] == [2.5, 2.6, 2.8, 6.0, 6.0, 6.0]
    assert [joint["vel_max"] for joint in model["joints"]] == [2.0] * 6


def test_connect_timeout_is_validated_and_generous_by_default() -> None:
    assert ArmConfig("arm", "Yam", SocketCanConnection("test")).connect_timeout_s == 120.0
    with pytest.raises(ConfigurationError):
        ArmConfig("arm", "Yam", SocketCanConnection("test"), connect_timeout_s=0)


def test_safety_torque_mode_is_opt_in() -> None:
    assert ArmConfig("arm", "Yam", SocketCanConnection("test")).safety_torque_mode is False
    assert (
        ArmConfig(
            "arm", "Yam", SocketCanConnection("test"), safety_torque_mode=True
        ).safety_torque_mode
        is True
    )
    with pytest.raises(ConfigurationError, match="safety_torque_mode must be a boolean"):
        ArmConfig("arm", "Yam", SocketCanConnection("test"), safety_torque_mode="false")  # type: ignore[arg-type]


def test_leader_gravity_compensation_defaults_on_for_yam_only() -> None:
    arx = ArmConfig("arm", "ARX_X5", SocketCanConnection("test"))
    arx_l5 = ArmConfig("arm", "ARX_L5", SocketCanConnection("test"))
    assert arx.leader_gravity_compensation is False
    assert arx_l5.leader_gravity_compensation is False
    assert ArmConfig("arm", "Yam", SocketCanConnection("test")).leader_gravity_compensation is True
    assert (
        ArmConfig(
            "arm",
            "ARX_X5",
            SocketCanConnection("test"),
            leader_gravity_compensation=True,
        ).leader_gravity_compensation
        is True
    )
    with pytest.raises(ConfigurationError, match="leader_gravity_compensation must be a boolean"):
        ArmConfig(  # type: ignore[arg-type]
            "arm",
            "Yam",
            SocketCanConnection("test"),
            leader_gravity_compensation="false",
        )
