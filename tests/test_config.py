import json
from pathlib import Path

import pytest

from openpi_control import (
    ArmConfig,
    ConfigurationError,
    EthernetConnection,
    FR3Connection,
    InputLayout,
    RobotiqConnection,
    SafetyLimits,
    SerialConnection,
    SocketCanConnection,
    connection_for_interface,
)
from openpi_control.config import SUPPORTED_MODELS
from openpi_control.protocol import topics_for


@pytest.mark.parametrize("model", SUPPORTED_MODELS)
def test_physical_model_catalog_is_complete(model: str) -> None:
    config = (
        ArmConfig(
            "arm",
            "FR3",
            FR3Connection("192.168.1.10"),
            effector_model="Robotiq",
            effector_connection=RobotiqConnection.tcp("192.168.1.11"),
        )
        if model == "FR3"
        else ArmConfig("arm", model, SocketCanConnection("test"))
    )
    assets = config.resolve_assets()
    assert assets.model_config.is_file()
    assert assets.instance_config.is_file()
    if model == "FR3":
        assert assets.urdf is None
    else:
        assert assets.urdf is not None and assets.urdf.is_file()
    # SO101 (SO-ARM100/101) is the catalog's only 5-DOF arm; everything else is 6-DOF.
    assert len(config.joint_names()) == (7 if model == "FR3" else 5 if model == "SO101" else 6)
    # The native node is always launched with --algo_type Pinocchio, which overrides
    # any config value except "Algo" (config keeps priority for "Algo"). Arm configs
    # must therefore never declare "Algo": robot-test 1.1.1 configs say "KDL"
    # (overridden to Pinocchio), openpi-tuned configs say "Pinocchio" directly.
    expected_algos = ("None",) if model == "FR3" else ("Pinocchio", "KDL")
    assert json.loads(assets.model_config.read_text())["algo_type"] in expected_algos


def test_fr3_requires_typed_arm_and_gripper_connections() -> None:
    gripper = RobotiqConnection.rtu("/dev/serial/by-id/usb-robotiq")
    config = ArmConfig(
        "follower",
        "FR3",
        FR3Connection("192.168.1.10"),
        effector_model="Robotiq",
        effector_connection=gripper,
    )
    assert config.joint_names() == tuple(f"fr3_joint{i}" for i in range(1, 8))
    assert gripper.endpoint.startswith("/dev/serial/by-id/")
    with pytest.raises(ConfigurationError, match="FR3 requires"):
        ArmConfig("follower", "FR3", SocketCanConnection("can0"))


def test_robotiq_supports_true_rtu_and_tcp_endpoints() -> None:
    rtu = RobotiqConnection.rtu("/dev/ttyUSB0", baud_rate=115200)
    tcp = RobotiqConnection.tcp("192.168.1.11", port=502)
    assert rtu.transport.value == "rtu"
    assert tcp.transport.value == "tcp"
    with pytest.raises(ConfigurationError, match="/dev path"):
        RobotiqConnection.rtu("192.168.1.11")


def test_connection_for_interface_dispatches_on_the_interface_form() -> None:
    assert connection_for_interface("can_left") == SocketCanConnection("can_left")
    assert connection_for_interface("192.168.1.11") == EthernetConnection("192.168.1.11")
    assert connection_for_interface("/dev/serial/by-id/usb-test-if00") == SerialConnection(
        "/dev/serial/by-id/usb-test-if00"
    )


def test_serial_connection_requires_a_dev_path() -> None:
    with pytest.raises(ConfigurationError, match="/dev path"):
        SerialConnection("ttyUSB0")


def test_default_alignment_allows_a_full_effector_stroke_in_one_second() -> None:
    limits = SafetyLimits()

    assert limits.max_effector_velocity_s == 1.0
    assert limits.max_effector_velocity_s * limits.minimum_alignment_duration_s >= 1.0


def test_so101_catalog_declares_serial_port_type_and_baudrate() -> None:
    config = ArmConfig("arm", "SO101", SocketCanConnection("test"), effector_model="E_SO101")
    catalog = json.loads(config.resolve_assets().model_config.read_text())["catalog"]
    assert catalog["port_type"] == "Serial"
    assert config.catalog_baudrate() == 1000000


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
    compat = ArmConfig(
        "leader", "Yam", SocketCanConnection("test"), effector_model="E_Yam_Handle_compat"
    )
    assert compat.input_layout().button_names == ("top", "bottom")
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


def test_arx_gripper_uses_training_normalized_range() -> None:
    # The model contract (state x0.375, action x4.5) assumes normalized 1.0 == 4.5 rad
    # motor angle. Without normalized_pos_min/max the C++ node falls back to the raw
    # servo range (read /5.077, write x4.877 with the old safety margin), feeding the
    # policy a gripper state ~11% low and executing commands ~8% hot (fixed in 00.00.47).
    config = ArmConfig("follower", "ARX_X5", SocketCanConnection("test"), effector_model="E_ARX")
    assets = config.resolve_assets()
    assert assets.effector_model_config is not None
    model = json.loads(assets.effector_model_config.read_text())
    joint = model["joints"][0]

    assert joint["normalized_pos_min"] == 0.0
    assert joint["normalized_pos_max"] == 4.5
    # Mutually exclusive with the normalized range (validated by the native node).
    assert "pos_max_safety_margin" not in joint
    # The raw servo limit stays as the safety bound.
    assert joint["servos"][0]["pos_max"] == 5.077


def test_yam_uses_reference_follower_tracking_constants() -> None:
    config = ArmConfig("follower", "Yam", SocketCanConnection("test"))
    model = json.loads(config.resolve_assets().model_config.read_text())

    assert [joint["follow_vel_max"] for joint in model["joints"]] == [2.5, 2.6, 2.8, 6.0, 6.0, 6.0]
    assert [joint["follow_viscous_damping"] for joint in model["joints"]] == pytest.approx(
        [0.7777778, 0.7777778, 0.7777778, 0.0, 0.0, 0.0]
    )
    # Reference-controller planning envelope (00.00.80); follow_vel_max above
    # is the operative follower speed limit.
    assert [joint["vel_max"] for joint in model["joints"]] == [20.0] * 6
    assert [joint["torq_max"] for joint in model["joints"]] == [27, 27, 27, 7, 7, 7]


@pytest.mark.parametrize("model_name", ["ARX_X5", "ARX_L5"])
def test_arx_uses_reference_follower_tracking_limits(model_name: str) -> None:
    # ARX_X5 and ARX_L5 mirror the Yam velocity limits (X5 restored in
    # 00.00.36, L5 in 00.00.57): the 0.3 rad/s robot-test cap made follower
    # tracking far too sluggish for policy execution.
    config = ArmConfig("follower", model_name, SocketCanConnection("test"))
    model = json.loads(config.resolve_assets().model_config.read_text())

    assert [joint["follow_vel_max"] for joint in model["joints"]] == [2.5, 2.6, 2.8, 6.0, 6.0, 6.0]
    # Reference-controller planning envelope and base-joint torque limits
    # (00.00.80): X5 base joints are ENCOS A4310 (datasheet peak 36 Nm), L5
    # base joints are DM J4340 (27 Nm inside the +-28 Nm MIT codec range);
    # wrists stay +-7 Nm.
    assert [joint["vel_max"] for joint in model["joints"]] == [20.0] * 6
    base_torque = 36 if model_name == "ARX_X5" else 27
    assert [joint["torq_max"] for joint in model["joints"]] == [base_torque] * 3 + [7] * 3


@pytest.mark.parametrize("model_name", ["ARX_X5", "ARX_L5"])
def test_arx_follower_position_limits_match_reference_urdf(model_name: str) -> None:
    # Reference URDF joint bounds (00.00.83/85): the operational envelope
    # derived from successful-episode data; commands outside are clipped
    # natively.
    config = ArmConfig("follower", model_name, SocketCanConnection("test"))
    model = json.loads(config.resolve_assets().model_config.read_text())

    expected = [(-2.1, 3.1), (0.0, 3.63), (0.0, 3.2), (-1.45, 1.35), (-1.58, 1.58), (-2.05, 2.05)]
    actual = [
        (joint["servos"][0]["pos_min"], joint["servos"][0]["pos_max"])
        for joint in model["joints"]
    ]
    assert actual == expected


@pytest.mark.parametrize("model_name", ["ARX_X5", "ARX_L5"])
def test_arx_torque_rescale_matches_controller_profile(model_name: str) -> None:
    config = ArmConfig("follower", model_name, SocketCanConnection("test"))
    model = json.loads(config.resolve_assets().model_config.read_text())

    expected = [1.0] * 3 + [10.0 / 7.6] * 3 if model_name == "ARX_X5" else [1.4] * 6
    assert [joint["torq_rescale"] for joint in model["joints"]] == pytest.approx(expected)


@pytest.mark.parametrize("model_name", ["ARX_X5", "ARX_L5"])
def test_arx_default_gains_are_the_gravity_assisted_baseline(model_name: str) -> None:
    # 00.00.94: the model config ships the gravity-assisted baseline gains as
    # the default (position gains correct tracking error; gravity feed-forward
    # holds the arm). The stiff profile lives in <model>_high_gain_01.json.
    config = ArmConfig("follower", model_name, SocketCanConnection("test"))
    model = json.loads(config.resolve_assets().model_config.read_text())

    gains = [
        (joint["servos"][0]["pos_kp"], joint["servos"][0]["pos_kd"])
        for joint in model["joints"]
    ]
    if model_name == "ARX_X5":
        assert gains == [(40, 1.2), (40, 1.2), (32, 1.0), (10, 0.8), (10, 0.8), (10, 1)]
    else:
        assert gains == [(150, 5), (150, 5), (150, 5), (30, 0.8), (25, 0.8), (10, 1)]


@pytest.mark.parametrize("model_name", ["ARX_X5", "ARX_L5"])
def test_arx_bundles_high_gain_calibration_variant(model_name: str) -> None:
    # 00.00.94: the stiff vendor-style gains are a selectable instance-config
    # variant (one-line devices.toml toggle, no wheel rebuild).
    config = ArmConfig("follower", model_name, SocketCanConnection("test"))
    arm_dir = config.resolve_assets().instance_config.parent
    variant = json.loads((arm_dir / f"{model_name}_high_gain_01.json").read_text())

    gains = [
        (joint["servos"][0]["pos_kp"], joint["servos"][0]["pos_kd"])
        for joint in variant["joints"]
    ]
    if model_name == "ARX_X5":
        assert gains == [(150, 12), (150, 12), (150, 12), (30, 0.8), (25, 0.8), (10, 1)]
    else:
        assert gains == [(150, 12), (150, 12), (150, 12), (30, 0.8), (30, 0.8), (30, 0.8)]


def test_arm_config_torq_rescale_normalizes_to_floats() -> None:
    config = ArmConfig(
        "follower",
        "ARX_X5",
        SocketCanConnection("test"),
        torq_rescale=[0.8, 0.8, 0.8, 1.5, 1.5, 1.5],
    )
    assert config.torq_rescale == (0.8, 0.8, 0.8, 1.5, 1.5, 1.5)


@pytest.mark.parametrize("bad", [[], [-0.1], [float("nan")], [float("inf")]])
def test_arm_config_rejects_non_physical_torq_rescale(bad: list[float]) -> None:
    with pytest.raises(ConfigurationError, match="torq_rescale"):
        ArmConfig("follower", "ARX_X5", SocketCanConnection("test"), torq_rescale=bad)


def test_arx_gripper_uses_reference_torque_spring() -> None:
    # Reference gripper torque-spring values: X5 stiffness 6.67 Nm/rad with
    # 0.2 rad spring offset, saturated at +-1.11 Nm; the bundled
    # E_ARX_l5_01.json variant carries the softer L5 spring (4.44 / 0.1).
    config = ArmConfig("follower", "ARX_X5", SocketCanConnection("test"), effector_model="E_ARX")
    assets = config.resolve_assets()
    assert assets.effector_model_config is not None
    assert assets.effector_instance_config is not None
    model = json.loads(assets.effector_model_config.read_text())
    instance = json.loads(assets.effector_instance_config.read_text())

    assert model["joints"][0]["grip_torque_limit"] == 1.11
    assert instance["control_mode"] == "torque"
    assert instance["dist_to_torque_const"] == 6.67
    assert instance["grip_spring_offset"] == 0.2

    l5_variant = assets.effector_instance_config.parent / "E_ARX_l5_01.json"
    l5 = json.loads(l5_variant.read_text())
    assert l5["dist_to_torque_const"] == 4.44
    assert l5["grip_spring_offset"] == 0.1


def test_arx_model_configs_define_servo_position_limits() -> None:
    # robot-test 1.1.1 places pos_min/pos_max in the model-config servo blocks, so
    # position limits no longer depend on the instance config being loadable.
    for model_name in ("ARX_X5", "ARX_ENC"):
        config = ArmConfig("arm", model_name, SocketCanConnection("test"))
        model = json.loads(config.resolve_assets().model_config.read_text())
        for joint in model["joints"]:
            for servo in joint["servos"]:
                assert servo["pos_min"] < servo["pos_max"]


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
