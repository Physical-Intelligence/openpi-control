"""Software-in-the-loop tests: the real pi_control_node against fake CAN servos.

These run only when OPENPI_SIL_VCAN names a configured Linux vcan interface
(see .github/workflows/ci.yml). The node binary is resolved through the
normal packaged/OPENPI_CONTROL_NODE lookup, so CI can interpose valgrind or a
sanitizer build via the env override.
"""

from __future__ import annotations

import os
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))

VCAN = os.environ.get("OPENPI_SIL_VCAN", "")
# Second bus for leader/follower pair tests (the leader and follower are
# separate physical CAN buses on the real cell).
VCAN2 = os.environ.get("OPENPI_SIL_VCAN2", "")

pytestmark = pytest.mark.skipif(
    sys.platform != "linux" or not VCAN or not (Path("/sys/class/net") / VCAN).exists(),
    reason="SIL tests need Linux and OPENPI_SIL_VCAN pointing at a configured vcan interface",
)

needs_second_vcan = pytest.mark.skipif(
    not VCAN2 or not (Path("/sys/class/net") / VCAN2).exists(),
    reason="pair tests need OPENPI_SIL_VCAN2 pointing at a second configured vcan interface",
)


@pytest.fixture
def fake_bus():
    from fake_dm_servo import FakeDmServoBus

    bus = FakeDmServoBus(VCAN)
    bus.start()
    yield bus
    bus.stop()


@pytest.fixture
def fake_bus_with_gripper():
    from fake_dm_servo import FakeDmServoBus

    bus = FakeDmServoBus(VCAN, motor_ids=(1, 2, 3, 4, 5, 6, 7), j4310_motors=(7,))
    bus.set_torque_mobile(7)
    bus.start()
    yield bus
    bus.stop()


@pytest.fixture
def fake_bus_with_handle():
    from fake_dm_servo import FakeDmServoBus

    # 0x50E is the fixed encoder id YAM teaching handles ship with (i2rt
    # encoder_manager: REQ=0x50E, REPORT=0x50F).
    bus = FakeDmServoBus(VCAN, motor_ids=(1, 2, 3, 4, 5, 6), handle_encoder_id=0x50E)
    bus.start()
    yield bus
    bus.stop()


@pytest.fixture
def session_factory():
    from openpi_control import ArmConfig, ArmSession, SocketCanConnection

    def make(*, effector_model: str | None = None, safety_torque_mode: bool = False):
        session = ArmSession()
        follower = session.add_follower(
            ArmConfig(
                "sil-follower",
                "Yam",
                SocketCanConnection(VCAN),
                effector_model=effector_model,
                control_frequency_hz=100,
                safety_torque_mode=safety_torque_mode,
            )
        )
        return session, follower

    return make


def wait_for(predicate, timeout_s: float, what: str):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError(f"timed out after {timeout_s}s waiting for {what}")


def assert_gripper_thermal_stop_is_inert(bus) -> None:
    wait_for(lambda: bus.motor_disable_count(7) == 1, timeout_s=3.0, what="one gripper disable")
    command = bus.last_command(7)
    assert command is not None
    _position, velocity, kp, torque = command
    assert velocity == pytest.approx(0.0, abs=0.02)
    assert kp == pytest.approx(0.0, abs=0.02)
    assert bus.last_kd(7) == pytest.approx(0.0, abs=0.02)
    assert torque == pytest.approx(0.0, abs=0.02)
    command_count = bus.mit_command_count(7)
    time.sleep(0.25)
    assert bus.mit_command_count(7) == command_count
    assert bus.motor_disable_count(7) == 1


def make_handle_leader(*, connect_timeout_s: float = 20.0):
    from openpi_control import ArmConfig, ArmSession, SocketCanConnection

    session = ArmSession()
    leader = session.add_leader(
        ArmConfig(
            "sil-leader",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam_Handle",
            control_frequency_hz=100,
            connect_timeout_s=connect_timeout_s,
        )
    )
    return session, leader


def test_full_session_against_fake_servos(fake_bus, session_factory):
    from openpi_control import PositionCommand

    session, follower = session_factory()
    with session:
        session.connect()

        state = follower.read_state(timeout_s=10.0)
        assert len(state.joints.names) == 6
        # The fake servos sit at their zero position (within DM quantization).
        for position in state.joints.position_rad:
            assert abs(position) < 0.01

        # Direct position command must propagate to the (fake) hardware. All
        # targets sit inside the per-joint limits (Yam_01.json mirrors the URDF
        # ranges; joints 2/3 only travel positive).
        target = [0.5, 0.25, 0.3, 0.0, 0.1, -0.1]
        follower.command(PositionCommand(target))
        wait_for(
            lambda: all(
                abs(follower.read_state(timeout_s=2.0).joints.position_rad[i] - target[i]) < 0.02
                for i in range(6)
            ),
            timeout_s=20.0,
            what="follower joints to reach the commanded target",
        )
        assert fake_bus.frames_handled > 0


def test_plain_follower_tracking_is_velocity_bounded(fake_bus, session_factory) -> None:
    # Direct tracking (planning "None", no gravity comp) must still traverse a
    # large command step at the synchronized follow_vel_max slew rate instead of
    # handing the raw target to the PD loop in one stiff jump. Yam joint 0 has
    # follow_vel_max=2.5 rad/s, so a 0.5 rad step takes ~0.2 s; the fake servo
    # tracks MIT commands instantly, so the reported position IS the slewed
    # command trajectory.
    from openpi_control import PositionCommand

    session, follower = session_factory()
    with session:
        session.connect()
        state = follower.read_state(timeout_s=10.0)
        assert abs(state.joints.position_rad[0]) < 0.01

        target = 0.5
        follower.command(PositionCommand([target, 0.0, 0.0, 0.0, 0.0, 0.0]))

        start_time: float | None = None
        arrival_time: float | None = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            position = follower.read_state(timeout_s=2.0).joints.position_rad[0]
            now = time.monotonic()
            if start_time is None and position > 0.05:
                start_time = now
            if position > target - 0.05:
                arrival_time = now
                break
        assert start_time is not None, "joint 0 never started moving"
        assert arrival_time is not None, "joint 0 never reached the commanded target"

        # 0.4 rad between the two thresholds at 2.5 rad/s takes 0.16 s. Allow a
        # generous lower margin for sampling jitter; an unslewed jump crosses
        # both thresholds within one 100 Hz control tick and fails this bound.
        traverse_s = arrival_time - start_time
        assert traverse_s > 0.08, f"crossed 0.4 rad in {traverse_s * 1000:.0f} ms - no slew limit"
        assert traverse_s < 2.0, f"took {traverse_s:.2f} s for 0.4 rad - far below follow_vel_max"


def test_node_survives_command_burst(fake_bus, session_factory):
    from openpi_control import PositionCommand

    session, follower = session_factory()
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

        # Bursts exercise the RX-thread/main-loop handoff far faster than the
        # teleop path would; under TSan this is what shakes out cache races.
        for step in range(200):
            offset = 0.001 * (step % 50)
            follower.command(PositionCommand([offset] * 6))
            if step % 20 == 0:
                follower.read_state(timeout_s=2.0)

        state = follower.read_state(timeout_s=2.0)
        assert len(state.joints.names) == 6


def test_killed_node_surfaces_as_native_process_error(fake_bus, session_factory):
    from openpi_control.exceptions import NativeProcessError, StateTimeoutError

    session, follower = session_factory()
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

        process = follower._backend._process  # noqa: SLF001 - chaos test needs the pid
        assert process is not None
        os.kill(process.pid, signal.SIGKILL)

        with pytest.raises(NativeProcessError):
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                try:
                    follower.read_state(timeout_s=1.0)
                except StateTimeoutError:
                    pass  # death detection may lag one read
                time.sleep(0.1)


def test_native_node_gracefully_stops_when_python_parent_dies(fake_bus, tmp_path):
    """Exercise the real inherited-FD lifeline across an abrupt Python exit."""
    native_pid_file = tmp_path / "native-pid"
    client_script = """
import sys
import time
from pathlib import Path

from openpi_control import ArmConfig, ArmSession, SocketCanConnection

session = ArmSession()
follower = session.add_follower(
    ArmConfig(
        "sil-parent-liveness-follower",
        "Yam",
        SocketCanConnection(sys.argv[1]),
        control_frequency_hz=100,
    )
)
session.connect()
follower.read_state(timeout_s=10.0)
Path(sys.argv[2]).write_text(str(follower._backend._process.pid))
while True:
    time.sleep(60)
"""
    client = subprocess.Popen(
        [sys.executable, "-c", client_script, VCAN, str(native_pid_file)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    native_pid: int | None = None
    native_pidfd: int | None = None
    try:
        def client_is_connected() -> bool:
            if native_pid_file.exists():
                return True
            if client.poll() is not None:
                output = client.stdout.read() if client.stdout is not None else ""
                raise AssertionError(f"client exited before connecting:\n{output}")
            return False

        wait_for(client_is_connected, timeout_s=20.0, what="client and native node to connect")
        native_pid = int(native_pid_file.read_text())
        native_pidfd = os.pidfd_open(native_pid)

        # The native node must not share the terminal-facing Python process's
        # session, while its inherited pipe must still tie its lifetime to Python.
        assert os.getsid(native_pid) == native_pid
        disable_counts = {
            motor_id: fake_bus.motor_disable_count(motor_id) for motor_id in range(1, 7)
        }

        os.kill(client.pid, signal.SIGKILL)
        client.wait(timeout=5.0)

        poller = select.poll()
        poller.register(native_pidfd, select.POLLIN)
        assert poller.poll(10_000), "native node survived its Python parent's death"
        wait_for(
            lambda: all(
                fake_bus.motor_disable_count(motor_id) > disable_counts[motor_id]
                for motor_id in range(1, 7)
            ),
            timeout_s=3.0,
            what="graceful native shutdown to disable every motor",
        )
    finally:
        if client.poll() is None:
            client.kill()
            client.wait(timeout=5.0)
        if client.stdout is not None:
            client.stdout.close()
        if native_pidfd is not None:
            try:
                signal.pidfd_send_signal(native_pidfd, signal.SIGKILL)
            except ProcessLookupError:
                pass
            os.close(native_pidfd)
        elif native_pid is not None:
            try:
                os.kill(native_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def test_connect_is_passive_and_explicit_homing_still_works(fake_bus, session_factory):
    # Park the fake arm away from home before the node starts: bring-up must
    # hold this pose, not drive the joints to zero.
    for motor_id in list(fake_bus.positions):
        fake_bus.set_position(motor_id, 0.4)

    session, follower = session_factory()
    with session:
        session.connect()

        state = follower.read_state(timeout_s=10.0)
        positions = state.joints.position_rad
        for position in positions:
            assert abs(abs(position) - 0.4) < 0.02, f"startup moved a joint: {positions}"

        # Explicit homing must still execute even though startup homing is
        # disabled (the flag is scoped to the startup phase).
        follower.move_to_ready()
        def joints_at_home() -> bool:
            current = follower.read_state(timeout_s=2.0).joints.position_rad
            return all(abs(p) < 0.02 for p in current)

        wait_for(
            joints_at_home,
            timeout_s=30.0,
            what="explicit move_to_ready to home the joints",
        )


def test_gripper_polarity_and_command_round_trip(fake_bus_with_gripper):
    # Pins the E_Yam gripper's normalized convention end to end: 0 = closed (zero
    # travel), 1 = open (full travel) -- the on-disk i2rt convention. The servo is
    # configured with dir_invert=-1, a joint-frame sign (relative = -absolute), which
    # must NOT flip the published value: treating it as "open at min" published the
    # physical YAM gripper inverted (physically closed read 1.0 -- caught on camera
    # during calibration-movement review). Commands must use the same mapping so the
    # round trip command(read()) holds position instead of mirroring across
    # mid-travel.
    from openpi_control import ArmConfig, ArmSession, PositionCommand, SocketCanConnection

    # Park the gripper beyond the semantic full-open endpoint but still inside the
    # raw recovery range. dir_invert=-1 puts the valid travel in negative absolute
    # radians: -5.0 here is relative 5.0 of the raw [0, 5.4] range.
    fake_bus_with_gripper.set_position(7, -5.0)

    session = ArmSession()
    follower = session.add_follower(
        ArmConfig(
            "sil-follower",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam",
            control_frequency_hz=100,
        )
    )
    with session:
        session.connect()

        state = follower.read_state(timeout_s=10.0)
        assert state.effector is not None
        # Readings past the calibrated 4.5-rad endpoint clip to open instead of
        # expanding the semantic command/observation range.
        assert state.effector.position == pytest.approx(1.0, abs=0.02)

        fake_bus_with_gripper.set_position(7, -4.5)
        wait_for(
            lambda: follower.read_state(timeout_s=2.0).effector.position > 0.98,
            timeout_s=2.0,
            what="calibrated 4.5-rad endpoint to report fully open",
        )
        fake_bus_with_gripper.set_position(7, -5.4)
        wait_for(
            lambda: follower.read_state(timeout_s=2.0).effector.position > 0.98,
            timeout_s=2.0,
            what="raw 5.4-rad endpoint to clip to fully open",
        )

        follower.command(PositionCommand(state.joints.position_rad, 1.0))
        wait_for(
            lambda: abs(fake_bus_with_gripper.position(7)) < 4.6,
            timeout_s=3.0,
            what="gripper above semantic open endpoint to move inward",
        )
        assert abs(fake_bus_with_gripper.position(7)) == pytest.approx(4.5, abs=0.08)
        command = fake_bus_with_gripper.last_command(7)
        assert command is not None
        assert command[0] == pytest.approx(-4.5, abs=0.03)

        target = 0.3
        follower.command(PositionCommand(state.joints.position_rad, target))
        # Commands and observations use the same calibrated semantic range [0, 4.5].
        wait_for(
            lambda: abs(follower.read_state(timeout_s=2.0).effector.position - target) < 0.05,
            timeout_s=10.0,
            what="gripper to settle at the commanded normalized position",
        )
        # The commanded 0.3 must be on the closed side physically: travel shrank.
        assert abs(fake_bus_with_gripper.position(7)) < 2.5


def test_gripper_coil_overtemperature_fails_with_actionable_error(fake_bus_with_gripper):
    from openpi_control import ArmConfig, ArmSession, HardwareFaultError, SocketCanConnection

    fake_bus_with_gripper.set_fault_status(7, 0xC)
    session = ArmSession()
    follower = session.add_follower(
        ArmConfig(
            "sil-follower",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam",
            control_frequency_hz=100,
        )
    )

    with session:
        with pytest.raises(HardwareFaultError) as exc_info:
            session.connect()

        message = str(exc_info.value)
        assert "DM servo id=7" in message
        assert "status 0xC (motor coil overtemperature)" in message
        assert "allow the motor to cool" in message
        assert "raw_range=[0.000, 5.400] rad" in message
        assert "normalized_range=[0.000, 4.500] rad" in message
        assert "zero_output_rc=" in message
        assert "disable_rc=" in message
        assert_gripper_thermal_stop_is_inert(fake_bus_with_gripper)
        log = "".join(follower._backend._log_lines)  # noqa: SLF001
        assert "No response while enabling servo id=7" not in log


def test_startup_recovers_multiple_stale_communication_loss_latches(fake_bus, session_factory):
    fake_bus.set_reset_clearable_fault_status(1, 0xD)
    fake_bus.set_reset_clearable_fault_status(2, 0xD)

    session, follower = session_factory()
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

    assert fake_bus.motor_reset_count(1) == 3
    assert fake_bus.motor_reset_count(2) == 3


def test_startup_fails_after_one_reset_for_persistent_communication_loss(fake_bus, session_factory):
    from openpi_control import HardwareFaultError

    fake_bus.set_fault_status(1, 0xD)
    session, _follower = session_factory()
    with session:
        with pytest.raises(HardwareFaultError, match=r"status 0xD \(communication loss\)"):
            session.connect()

    assert fake_bus.motor_reset_count(1) == 3


@pytest.mark.parametrize(
    ("status_code", "description"),
    [(0xB, "MOSFET overtemperature"), (0xC, "motor coil overtemperature")],
)
def test_live_gripper_thermal_fault_preempts_state_timeout_and_parks(
    fake_bus_with_gripper, status_code: int, description: str
):
    from openpi_control import (
        ArmConfig,
        ArmSession,
        HardwareFaultError,
        PositionCommand,
        SocketCanConnection,
    )

    session = ArmSession()
    follower = session.add_follower(
        ArmConfig(
            "sil-follower",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam",
            control_frequency_hz=100,
        )
    )

    with session:
        session.connect()
        state = follower.read_state(timeout_s=10.0)
        fake_bus_with_gripper.set_stuck(7, -2.7)
        follower.command(PositionCommand(state.joints.position_rad, 1.0))
        time.sleep(0.2)
        fake_bus_with_gripper.set_fault_status(7, status_code)

        with pytest.raises(HardwareFaultError, match=description) as exc_info:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                follower.read_state(timeout_s=1.0)

        message = str(exc_info.value)
        assert "requested_target=" in message
        assert "applied_target=" in message
        assert "limiter_active=1" in message
        assert "measured_position=" in message
        assert "effort=" in message
        assert "current=" in message
        assert "temperature=" in message
        assert_gripper_thermal_stop_is_inert(fake_bus_with_gripper)


def test_first_effector_temperature_sample_above_force_stop_parks(fake_bus_with_gripper):
    from openpi_control import ArmConfig, ArmSession, HardwareFaultError, SocketCanConnection

    session = ArmSession()
    follower = session.add_follower(
        ArmConfig(
            "sil-follower",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam",
            control_frequency_hz=100,
        )
    )

    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)
        fake_bus_with_gripper.set_temperature(7, 94)

        with pytest.raises(HardwareFaultError, match="first sample above 93 C") as exc_info:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                follower.read_state(timeout_s=1.0)

        assert "temperature=94 C" in str(exc_info.value)
        assert_gripper_thermal_stop_is_inert(fake_bus_with_gripper)


def test_nonthermal_gripper_hardware_fault_does_not_use_thermal_disable(fake_bus_with_gripper):
    from openpi_control import ArmConfig, ArmSession, HardwareFaultError, SocketCanConnection

    session = ArmSession()
    follower = session.add_follower(
        ArmConfig(
            "sil-follower",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam",
            control_frequency_hz=100,
        )
    )

    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)
        fake_bus_with_gripper.set_fault_status(7, 0xE)

        with pytest.raises(HardwareFaultError, match="overload"):
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                follower.read_state(timeout_s=1.0)

        assert fake_bus_with_gripper.motor_disable_count(7) == 0


def test_gripper_motor_side_control_is_fast_and_force_bounded(fake_bus_with_gripper):
    # Keep contact dynamics in the motor-side position loop while limiting
    # both opening and closing demand to 1.11 Nm. Low kp widens the safe moving
    # window for fast free travel.
    from openpi_control import ArmConfig, ArmSession, PositionCommand, SocketCanConnection

    bus = fake_bus_with_gripper
    bus.set_position(7, -5.0)  # start open (relative 5.0 of [0, 5.4])

    grip_limit_nm = 1.11
    kp_config = 2.5
    max_error_rad = grip_limit_nm / kp_config

    session = ArmSession()
    follower = session.add_follower(
        ArmConfig(
            "sil-follower",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam",
            control_frequency_hz=100,
        )
    )
    with session:
        session.connect()
        state = follower.read_state(timeout_s=10.0)
        assert state.effector is not None

        # The wide object: the gripper physically cannot close past mid travel
        # (relative 2.7 = absolute -2.7 under dir_invert=-1).
        blocked_rel = 2.7
        bus.set_stuck(7, -blocked_rel)

        def assert_bounded(target: float, expected_window: float) -> None:
            follower.command(PositionCommand(state.joints.position_rad, target))
            time.sleep(0.25)
            command = bus.last_command(7)
            assert command is not None
            target_pos_abs, _vel, kp, torque_ff = command
            applied_error = abs(target_pos_abs - (-blocked_rel))
            demand_nm = kp * applied_error + abs(torque_ff)
            assert applied_error == pytest.approx(expected_window, abs=0.03)
            assert kp == pytest.approx(kp_config, abs=0.1)
            assert bus.last_kd(7) == pytest.approx(0.1, abs=0.01)
            assert abs(torque_ff) < 0.05
            assert demand_nm <= grip_limit_nm * 1.1

        assert_bounded(0.0, max_error_rad)
        assert_bounded(1.0, max_error_rad)

        # The node must still be healthy (no emergency recovery from the block).
        state = follower.read_state(timeout_s=2.0)
        assert state.effector.position == pytest.approx(blocked_rel / 4.5, abs=0.05)
        log = "".join(follower._backend._log_lines)  # noqa: SLF001
        assert "[RECOVERY]" not in log, "a blocked gripper must not trigger emergency recovery"

        # Thermal derating applies equally to both signs and reaches a zero
        # error window at 90 C.
        bus.set_temperature(7, 75)
        assert_bounded(0.0, max_error_rad * 0.5)
        assert_bounded(1.0, max_error_rad * 0.5)
        bus.set_temperature(7, 90)
        assert_bounded(0.0, 0.0)
        assert_bounded(1.0, 0.0)
        bus.set_temperature(7, 25)

        # Object removed: both directions complete quickly.
        bus.clear_stuck(7)
        started = time.monotonic()
        wait_for(
            lambda: follower.read_state(timeout_s=2.0).effector.position > 0.9,
            timeout_s=2.0,
            what="gripper to open under bounded position control",
        )
        assert time.monotonic() - started < 1.5

        follower.command(PositionCommand(state.joints.position_rad, 0.0))
        started = time.monotonic()
        wait_for(
            lambda: follower.read_state(timeout_s=2.0).effector.position < 0.05,
            timeout_s=2.0,
            what="gripper to close under bounded position control",
        )
        assert time.monotonic() - started < 1.5


def test_follower_gravity_compensation_feeds_torque(fake_bus):
    # With follower_gravity_compensation enabled, the arm device switches to
    # slew_pos_gravity planning: every follower position command carries a
    # model-based (pinocchio RNEA over Yam.urdf) gravity feedforward torque plus
    # MonoPi-style viscous damping on joints 1-3. Park the fake arm bent so the
    # pitch joints are gravity-loaded, then assert the MIT command frames carry
    # torque on the loaded elbow and that position tracking still works through
    # the synchronized velocity slew limiter.
    from openpi_control import ArmConfig, ArmSession, PositionCommand, SocketCanConnection

    for motor_id in (2, 3):
        fake_bus.set_position(motor_id, 1.0)

    session = ArmSession()
    follower = session.add_follower(
        ArmConfig(
            "sil-follower",
            "Yam",
            SocketCanConnection(VCAN),
            control_frequency_hz=100,
            follower_gravity_compensation=True,
        )
    )
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

        wait_for(
            lambda: abs(fake_bus.last_torque(3)) > 0.2,
            timeout_s=10.0,
            what="gravity feedforward torque on the loaded elbow joint",
        )
        assert abs(fake_bus.last_torque(1)) < 0.2, "base yaw joint must carry no gravity torque"

        joint1_baseline = fake_bus.last_torque(1)
        joint4_baseline = fake_bus.last_torque(4)
        fake_bus.set_reported_velocity(1, 1.0)
        fake_bus.set_reported_velocity(4, 1.0)
        wait_for(
            lambda: abs((fake_bus.last_torque(1) - joint1_baseline) + 0.7777778) < 0.1,
            timeout_s=5.0,
            what="joint-1 viscous damping to oppose measured velocity",
        )
        assert abs(fake_bus.last_torque(4) - joint4_baseline) < 0.1
        fake_bus.set_reported_velocity(1, 0.0)
        fake_bus.set_reported_velocity(4, 0.0)

        # Position commands still track through the synchronized slew limiter.
        target = [0.1, 0.9, 1.1, 0.0, 0.1, -0.1]
        follower.command(PositionCommand(target))
        wait_for(
            lambda: all(
                abs(follower.read_state(timeout_s=2.0).joints.position_rad[i] - target[i]) < 0.05
                for i in range(6)
            ),
            timeout_s=20.0,
            what="follower joints to reach the commanded target under gravity planning",
        )


def test_handle_encoder_startup_corrects_configuration_before_motor_enable():
    from fake_dm_servo import FakeDmServoBus

    bus = FakeDmServoBus(
        VCAN,
        motor_ids=(1, 2, 3, 4, 5, 6),
        handle_encoder_id=0x50E,
        handle_adc_frequency=100,
        handle_report_frequency=50,
    )
    bus.start()
    session, leader = make_handle_leader()
    try:
        with session:
            session.connect()
            leader.read_state(timeout_s=10.0)

            assert bus.encoder_frequencies() == (255, 0)
            assert bus.encoder_config_writes == [(0x04, 255), (0x01, 0)]
            first_enable = bus.events.index("motor-enable")
            assert bus.events.index("encoder-write-adc") < first_enable
            assert bus.events.index("encoder-write-report") < first_enable
            for offset in (27, 8, 28, 25):
                assert bus.encoder_eeprom_reads.count(offset) >= 2, (
                    f"corrected EEPROM offset {offset} was not re-read for verification"
                )
    finally:
        bus.stop()


def test_handle_encoder_startup_skips_writes_when_already_configured(fake_bus_with_handle):
    session, leader = make_handle_leader()
    with session:
        session.connect()
        leader.read_state(timeout_s=10.0)

        assert fake_bus_with_handle.encoder_version_requests >= 1
        assert fake_bus_with_handle.encoder_config_writes == []
        assert fake_bus_with_handle.motor_enables_handled > 0


@pytest.mark.parametrize(
    ("firmware", "expected_log"),
    [
        (None, "no valid firmware response"),
        ((2, 2, 11), "firmware 2.2.11 is unsupported"),
    ],
)
def test_handle_encoder_missing_or_old_firmware_fails_before_motor_enable(firmware, expected_log):
    from fake_dm_servo import FakeDmServoBus

    from openpi_control.exceptions import NativeProcessError

    bus = FakeDmServoBus(
        VCAN,
        motor_ids=(1, 2, 3, 4, 5, 6),
        handle_encoder_id=0x50E,
        handle_firmware=firmware,
    )
    bus.start()
    session, leader = make_handle_leader(connect_timeout_s=10.0)
    try:
        with pytest.raises(NativeProcessError) as excinfo:
            session.connect()
        assert expected_log in str(excinfo.value)
        assert bus.motor_enables_handled == 0
        assert "motor-enable" not in bus.events
        assert leader.latest_state is None
    finally:
        try:
            session.close()
        finally:
            bus.stop()


def test_handle_encoder_delayed_replies_keep_one_poll_and_one_button_publication():
    from fake_dm_servo import FakeDmServoBus

    bus = FakeDmServoBus(
        VCAN,
        motor_ids=(1, 2, 3, 4, 5, 6),
        handle_encoder_id=0x50E,
        handle_response_delay_s=0.1,
    )
    bus.start()
    session, leader = make_handle_leader()
    try:
        with session:
            session.connect()
            wait_for(
                lambda: bus.encoder_polls_handled >= 4,
                timeout_s=10.0,
                what="several delayed encoder polls",
            )
            assert bus.encoder_max_outstanding_polls == 1

            # Let the latest delayed response reach the Python subscriber,
            # then mute only future polls while leaving the next queued reply
            # intact. That one fresh response must create exactly one input
            # publication; cached double-reads must create none.
            wait_for(
                lambda: bus.encoder_outstanding_polls() == 0,
                timeout_s=5.0,
                what="delayed encoder reply to settle",
            )
            time.sleep(0.1)
            assert leader.latest_inputs is not None
            baseline_sequence = leader.latest_inputs.sequence
            baseline_responses = bus.encoder_poll_responses_sent
            wait_for(
                lambda: bus.encoder_outstanding_polls() == 1,
                timeout_s=5.0,
                what="one queued delayed encoder reply",
            )
            bus.set_encoder_mute(True)
            wait_for(
                lambda: bus.encoder_poll_responses_sent == baseline_responses + 1,
                timeout_s=2.0,
                what="the queued delayed encoder reply",
            )
            wait_for(
                lambda: (
                    leader.latest_inputs is not None
                    and leader.latest_inputs.sequence == baseline_sequence + 1
                ),
                timeout_s=2.0,
                what="one button publication for the delayed reply",
            )
            time.sleep(0.3)
            assert leader.latest_inputs is not None
            assert leader.latest_inputs.sequence == baseline_sequence + 1
            assert bus.encoder_max_outstanding_polls == 1
    finally:
        bus.stop()


def test_leader_handle_trigger_and_buttons(fake_bus_with_handle):
    # First native leader-session coverage: the YAM teaching handle is a passive
    # CAN encoder (servo model "CAN Passive Encoder") that the node polls with
    # [0xFF, 0x02] and that answers on id+1 with trigger ticks + button bits.
    # Pins the i2rt trigger convention end to end: released = effector 1.0
    # (open), a full squeeze beyond the 0.63 rad calibration = 0.0, and the two buttons
    # surface through read_inputs() as ("top", "bottom").
    from openpi_control import ArmConfig, ArmSession, SocketCanConnection

    bus = fake_bus_with_handle
    session = ArmSession()
    leader = session.add_leader(
        ArmConfig(
            "sil-leader",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam_Handle",
            control_frequency_hz=100,
        )
    )
    with session:
        session.connect()

        state = leader.read_state(timeout_s=10.0)
        assert state.effector is not None
        # Trigger at rest (0 ticks) must read fully open.
        assert state.effector.position == pytest.approx(1.0, abs=0.02)

        # Physical full squeeze (~0.66 rad, beyond the calibrated 0.63 travel)
        # must saturate at 0.0 (closed) -- the instance config's pos_max is the
        # i2rt trigger_closed_position calibration.
        bus.set_trigger_rad(0.66)
        wait_for(
            lambda: leader.read_state(timeout_s=2.0).effector.position < 0.02,
            timeout_s=10.0,
            what="squeezed trigger to saturate at 0.0 on the effector joint",
        )

        # Half of the calibrated travel lands mid-range (linear, not binary).
        bus.set_trigger_rad(0.315)
        wait_for(
            lambda: abs(leader.read_state(timeout_s=2.0).effector.position - 0.5) < 0.05,
            timeout_s=10.0,
            what="half-squeezed trigger to read ~0.5",
        )

        # Buttons publish through MsgJoystick and decode as top/bottom.
        inputs = leader.read_inputs(timeout_s=10.0)
        assert inputs.button_names == ("top", "bottom")
        assert inputs.buttons == (False, False)

        bus.set_buttons(top=True, bottom=False)
        wait_for(
            lambda: leader.read_inputs(timeout_s=2.0).buttons == (True, False),
            timeout_s=10.0,
            what="top button press to reach read_inputs",
        )
        bus.set_buttons(top=False, bottom=True)
        wait_for(
            lambda: leader.read_inputs(timeout_s=2.0).buttons == (False, True),
            timeout_s=10.0,
            what="bottom button press to reach read_inputs",
        )

        # Polling must be continuous, not just a startup probe. An absolute
        # count would encode a loop rate, which valgrind/TSan runs don't honor
        # (the 50 Hz+ requirement is validated on hardware: ~195 Hz measured).
        polls_before = bus.encoder_polls_handled
        wait_for(
            lambda: bus.encoder_polls_handled > polls_before + 10,
            timeout_s=10.0,
            what="continuous encoder polling at the control rate",
        )


def test_handle_encoder_transient_mute_holds_trigger(fake_bus_with_handle):
    # The YAM handle firmware intermittently stops answering polls for over a
    # second and then recovers by itself (seen on both yambot handles on the
    # wire, 2026-06-10, while the DM servos on the same bus kept responding).
    # A transient mute must NOT trigger emergency recovery: the trigger holds
    # its last value and the stream resumes when the encoder comes back. The
    # old ~1 s silence budget turned every such transient into a recovery that
    # killed the teleop session.
    from openpi_control import ArmConfig, ArmSession, SocketCanConnection

    bus = fake_bus_with_handle
    bus.set_trigger_rad(0.315)  # mid travel, so the held value is distinctive

    session = ArmSession()
    leader = session.add_leader(
        ArmConfig(
            "sil-leader",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam_Handle",
            control_frequency_hz=100,
        )
    )
    with session:
        session.connect()
        wait_for(
            lambda: abs(leader.read_state(timeout_s=2.0).effector.position - 0.5) < 0.05,
            timeout_s=10.0,
            what="mid-travel trigger before the mute",
        )

        # Two seconds of wall-clock silence is below the 3 s restart threshold
        # and far below the 20 s SAFE_MODE_SIG threshold on every loop rate.
        bus.set_encoder_mute(True)
        time.sleep(2.0)
        held = leader.read_state(timeout_s=2.0).effector.position
        assert abs(held - 0.5) < 0.05, f"muted trigger must hold its last value, read {held:.3f}"
        log = "".join(leader._backend._log_lines)  # noqa: SLF001
        assert "[RECOVERY]" not in log, "a transient encoder mute must not trigger recovery"

        bus.set_encoder_mute(False)
        bus.set_trigger_rad(0.66)
        wait_for(
            lambda: leader.read_state(timeout_s=2.0).effector.position < 0.02,
            timeout_s=10.0,
            what="trigger stream to resume after the mute",
        )
        assert "[RECOVERY]" not in "".join(leader._backend._log_lines)  # noqa: SLF001


def test_handle_encoder_wedge_recovers_via_firmware_restart(fake_bus_with_handle):
    # A wedged handle that stays silent past ~3 s gets the i2rt REQ_RESTART
    # frame from the node; the real handle reboots and answers again ~8.5 s
    # later. The whole episode must ride through without emergency recovery
    # and without any test-side intervention.
    from openpi_control import ArmConfig, ArmSession, SocketCanConnection

    bus = fake_bus_with_handle
    bus.set_trigger_rad(0.315)

    session = ArmSession()
    leader = session.add_leader(
        ArmConfig(
            "sil-leader",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam_Handle",
            control_frequency_hz=100,
        )
    )
    with session:
        session.connect()
        wait_for(
            lambda: abs(leader.read_state(timeout_s=2.0).effector.position - 0.5) < 0.05,
            timeout_s=10.0,
            what="mid-travel trigger before the wedge",
        )

        bus.set_encoder_mute(True, restart_heals=True)
        # The node must notice (~3 s of stale reads), send REQ_RESTART, and the
        # "rebooted" encoder must come back -- all without touching the fake
        # again. Generous budget for valgrind's slower loop rate.
        wait_for(lambda: bus.encoder_restarts >= 1, timeout_s=60.0, what="firmware restart request")
        bus.set_trigger_rad(0.66)
        wait_for(
            lambda: leader.read_state(timeout_s=2.0).effector.position < 0.02,
            timeout_s=30.0,
            what="trigger stream to resume after the restart",
        )
        log = "".join(leader._backend._log_lines)  # noqa: SLF001
        assert "sending firmware restart" in log
        assert "[RECOVERY]" not in log, "a restart-recoverable wedge must not trigger recovery"


def test_leader_float_applies_gravity_torque(fake_bus_with_handle):
    # A connected leader floats on model gravity torque (i2rt behavior): the
    # session puts leaders into gravity-compensation mode at connect, and the
    # node must feed pinocchio gravity torques to the loaded joints. Before the
    # fix, enabled_gravity_compensation_ was never set and "float" sent zero
    # torque -- a limp arm that droops the moment the operator lets go.
    from openpi_control import ArmConfig, ArmSession, SocketCanConnection

    bus = fake_bus_with_handle
    for motor_id in (2, 3):
        bus.set_position(motor_id, 1.0)

    session = ArmSession()
    leader = session.add_leader(
        ArmConfig(
            "sil-leader",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam_Handle",
            control_frequency_hz=100,
        )
    )
    with session:
        session.connect()
        leader.read_state(timeout_s=10.0)

        wait_for(
            lambda: abs(bus.last_torque(3)) > 0.2,
            timeout_s=10.0,
            what="float-mode gravity torque on the loaded elbow joint",
        )
        assert abs(bus.last_torque(1)) < 0.2, "base yaw joint must carry no gravity torque"


@needs_second_vcan
def test_teleop_pair_engage_mirrors_joints_gripper_and_keeps_gravity():
    # End-to-end native teleop: leader (with teaching handle) on one bus,
    # follower (with gripper) on a second bus, engaged through TeleopPair.
    # Covers the full chain Kyle drives in the morning session:
    #   - engage aligns the follower to the leader pose,
    #   - native live forwarding mirrors operator joint motion,
    #   - the trigger drives the follower gripper closed/open (with the
    #     pos_max travel calibration saturating full squeeze at 0.0),
    #   - bilateral mode keeps gravity feedforward on the leader (pre-fix the
    #     engaged leader lost its float and the operator carried the arm),
    #   - disengage freezes the follower and returns the leader to float.
    # Leader motors are marked "stuck" to emulate an operator-held arm: they
    # report the held pose and ignore the bilateral position spring.
    from fake_dm_servo import FakeDmServoBus

    from openpi_control import ArmConfig, ArmSession, SocketCanConnection

    # Bent pose with the gravity load on joints 2/3 (the configuration the
    # follower gravity test already proves yields > 0.2 Nm at the elbow).
    held_pose = {1: 0.3, 2: 1.0, 3: 1.0, 4: 0.3, 5: 0.3, 6: 0.3}
    leader_bus = FakeDmServoBus(VCAN, motor_ids=(1, 2, 3, 4, 5, 6), handle_encoder_id=0x50E)
    follower_bus = FakeDmServoBus(VCAN2, motor_ids=(1, 2, 3, 4, 5, 6, 7), j4310_motors=(7,))
    leader_bus.start()
    follower_bus.start()
    try:
        for motor_id, position in held_pose.items():
            leader_bus.set_stuck(motor_id, position)

        session = ArmSession()
        leader = session.add_leader(
            ArmConfig(
                "sil-leader",
                "Yam",
                SocketCanConnection(VCAN),
                effector_model="E_Yam_Handle",
                control_frequency_hz=100,
            )
        )
        follower = session.add_follower(
            ArmConfig(
                "sil-follower",
                "Yam",
                SocketCanConnection(VCAN2),
                effector_model="E_Yam",
                control_frequency_hz=100,
                follower_gravity_compensation=True,
            )
        )
        pair = session.add_pair(leader, follower)
        with session:
            session.connect()

            leader_state = leader.read_state(timeout_s=10.0)
            assert leader_state.effector is not None

            pair.engage()
            assert pair.engaged

            def follower_tracks_leader(tolerance: float = 0.05) -> bool:
                lead = leader.read_state(timeout_s=2.0).joints.position_rad
                follow = follower.read_state(timeout_s=2.0).joints.position_rad
                return all(abs(lead[i] - follow[i]) < tolerance for i in range(6))

            wait_for(
                follower_tracks_leader,
                timeout_s=20.0,
                what="follower to align with the held leader pose after engage",
            )

            # Bilateral must keep the leader's gravity float: the MIT frames to
            # the loaded elbow carry the pinocchio feedforward, not just the
            # follower-tracking position spring.
            wait_for(
                lambda: abs(leader_bus.last_torque(3)) > 0.2,
                timeout_s=10.0,
                what="gravity feedforward on the engaged leader's elbow",
            )
            wait_for(
                lambda: all(
                    leader_bus.last_kd(motor_id) is not None
                    and abs(leader_bus.last_kd(motor_id)) < 0.01
                    for motor_id in range(1, 7)
                ),
                timeout_s=10.0,
                what="zero bilateral damping on every engaged leader joint",
            )

            # Operator moves the leader: the follower mirrors through the
            # native live-forwarding path.
            for motor_id, position in held_pose.items():
                leader_bus.set_stuck(motor_id, position - 0.25)
            wait_for(
                follower_tracks_leader,
                timeout_s=20.0,
                what="follower to mirror the operator's leader motion",
            )
            follower_positions = follower.read_state(timeout_s=2.0).joints.position_rad
            assert any(abs(p) > 0.2 for p in follower_positions), "follower never left home"

            # Trigger forwarding: full squeeze (saturated by the travel
            # calibration) closes the follower gripper; release reopens it.
            leader_bus.set_trigger_rad(0.66)
            wait_for(
                lambda: follower.read_state(timeout_s=2.0).effector.position < 0.05,
                timeout_s=10.0,
                what="follower gripper to close on a full trigger squeeze",
            )
            leader_bus.set_trigger_rad(0.0)
            wait_for(
                lambda: follower.read_state(timeout_s=2.0).effector.position > 0.9,
                timeout_s=10.0,
                what="follower gripper to reopen on trigger release",
            )

            # Re-baseline right before disengage: the follower has been
            # live-tracking through the gripper phase and may have converged
            # further since the post-move capture.
            follower_positions = follower.read_state(timeout_s=2.0).joints.position_rad

            pair.disengage()
            assert not pair.engaged

            def dump_pair_state(label: str) -> None:
                lead = leader.read_state(timeout_s=2.0).joints.position_rad
                follow = follower.read_state(timeout_s=2.0).joints.position_rad
                print(f"{label}: leader={[round(p, 3) for p in lead]} "
                      f"follower={[round(p, 3) for p in follow]} "
                      f"baseline={[round(p, 3) for p in follower_positions]}")
                for arm in (leader, follower):
                    log_tail = "".join(arm._backend._log_lines)[-1500:]  # noqa: SLF001
                    print(f"--- {arm.name} node log tail ---\n{log_tail}")

            # The follower must freeze where disengage left it...
            time.sleep(0.5)
            follower_frozen = follower.read_state(timeout_s=2.0).joints.position_rad
            if not all(abs(follower_frozen[i] - follower_positions[i]) < 0.05 for i in range(6)):
                dump_pair_state("follower moved during/after disengage with a still leader")
                raise AssertionError("follower must hold its pose through disengage itself")

            # ...and keep ignoring further leader motion.
            for motor_id, position in held_pose.items():
                leader_bus.set_stuck(motor_id, position)
            time.sleep(1.0)
            follower_after = follower.read_state(timeout_s=2.0).joints.position_rad
            if not all(abs(follower_after[i] - follower_positions[i]) < 0.05 for i in range(6)):
                dump_pair_state("follower tracked the leader after disengage")
                raise AssertionError("follower must ignore leader motion after disengage")
    finally:
        leader_bus.stop()
        follower_bus.stop()


def test_ready_move_with_oscillating_joint_completes_within_budget(fake_bus, session_factory):
    # Issue #5 failure mode: a joint oscillating against an external load (gravity sag
    # fighting the position hold) twitches past the stuck-detection motion floor every
    # few iterations, resetting both the stuck counter and the arrival confirmation --
    # pre-fix, the ready move looped forever at full control-loop rate. The
    # distance-derived iteration budget must complete it best-effort and name the joint.
    for motor_id in list(fake_bus.positions):
        fake_bus.set_position(motor_id, 0.4)
    fake_bus.set_stuck(4, position=0.4, jitter=0.02)

    session, follower = session_factory()
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

        start = time.monotonic()
        follower.move_to_ready()  # pre-fix: ProtocolError after the 60s client deadline
        elapsed = time.monotonic() - start
        # Budget floor is 1500 iterations at 100 Hz (~15s) plus settle/restore overhead.
        assert elapsed < 45.0, f"budgeted ready move took {elapsed:.1f}s"

        positions = follower.read_state(timeout_s=2.0).joints.position_rad
        assert abs(positions[3] - 0.4) < 0.05, f"oscillating joint should stay parked: {positions}"
        for i in (0, 1, 2, 4, 5):
            assert abs(positions[i]) < 0.02, f"healthy joint {i} should be home: {positions}"

        log = "".join(follower._backend._log_lines)  # noqa: SLF001 - asserting on node diagnostics
        assert "exceeded its iteration budget" in log
        assert "servo id=4" in log


def test_ready_move_with_stuck_joint_latches_and_completes_fast(fake_bus, session_factory):
    # A motionless stuck joint must keep taking the fast exit: the stuck counter crosses
    # its threshold after ~2s, latches, and the ready move completes with the joint
    # reported -- long before the iteration budget would fire.
    for motor_id in list(fake_bus.positions):
        fake_bus.set_position(motor_id, 0.4)
    fake_bus.set_stuck(2, position=0.4)

    session, follower = session_factory()
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

        start = time.monotonic()
        follower.move_to_ready()
        elapsed = time.monotonic() - start
        assert elapsed < 12.0, f"stuck-joint ready move took {elapsed:.1f}s (not the latch path?)"

        positions = follower.read_state(timeout_s=2.0).joints.position_rad
        assert abs(positions[1] - 0.4) < 0.05, f"stuck joint should stay parked: {positions}"

        log = "".join(follower._backend._log_lines)  # noqa: SLF001 - asserting on node diagnostics
        assert "stuck joint(s)" in log
        assert "first stuck id=2" in log


def test_malformed_wire_messages_do_not_kill_the_node(fake_bus, session_factory):
    # joint_num comes straight off the wire and controls indexes into fixed
    # float[10] payload arrays. A corrupt or buggy publisher must not be able to
    # crash the control node mid-teleop (motors torque off, arm drops).
    from openpi_control.protocol import JOINT_STRUCT

    session, follower = session_factory()
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

        direct_pub = follower._backend._direct_pub  # noqa: SLF001 - raw-wire chaos test
        assert direct_pub is not None
        joint_info_type = 1  # MsgType::JOINT_INFO
        for msg_id in range(1, 21):
            payload = JOINT_STRUCT.pack(*([0.0] * 60), msg_id, 32767, joint_info_type, 0.0)
            direct_pub.send(payload)
            time.sleep(0.02)

        # The node is still alive and the reject path actually ran.
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            follower.read_state(timeout_s=2.0)
            time.sleep(0.1)
        process = follower._backend._process  # noqa: SLF001
        assert process is not None and process.poll() is None, "node died on a malformed message"
        log = "".join(follower._backend._log_lines)  # noqa: SLF001
        assert "Invalid joint count" in log, "malformed JOINT_INFO never reached the parser"


def test_out_of_range_normalized_gripper_command_clamps(fake_bus_with_gripper):
    # The normalized->radian mapping used to return 0 rad (FULL CLOSE for
    # E_Yam, whose open end is pos_max) for any input outside [0, 1] -- so a
    # producer emitting 1.0001 from float rounding slammed the gripper to the
    # exact opposite of its intent. Out-of-range finite commands must clamp
    # (1.5 -> open), and NaN must hold instead of propagating into the CAN
    # float->uint encode (UB).
    from openpi_control import ArmConfig, ArmSession, SocketCanConnection
    from openpi_control.protocol import JOINT_STRUCT

    bus = fake_bus_with_gripper
    bus.set_position(7, -2.7)  # mid travel (relative 2.7 of [0, 5.4])

    session = ArmSession()
    follower = session.add_follower(
        ArmConfig(
            "sil-follower",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam",
            control_frequency_hz=100,
        )
    )
    with session:
        session.connect()
        state = follower.read_state(timeout_s=10.0)
        assert state.effector is not None
        assert state.effector.position == pytest.approx(2.7 / 4.5, abs=0.05)

        direct_pub = follower._backend._direct_pub  # noqa: SLF001 - raw-wire test
        assert direct_pub is not None
        joints = list(state.joints.position_rad)

        def send_gripper(normalized: float, msg_id: int) -> None:
            positions = joints + [normalized] + [0.0] * 3
            payload = JOINT_STRUCT.pack(
                *(positions + [0.0] * 50),
                msg_id,
                7,
                1,
                0.0,  # 7 joints, MsgType::JOINT_INFO
            )
            direct_pub.send(payload)

        # 1.5 must clamp to 1.0 = fully open (pre-fix: full close).
        msg_id = 1
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            send_gripper(1.5, msg_id)
            msg_id += 1
            if follower.read_state(timeout_s=2.0).effector.position > 0.9:
                break
            time.sleep(0.05)
        opened = follower.read_state(timeout_s=2.0).effector.position
        assert opened > 0.9, f"out-of-range command should clamp to open, gripper at {opened:.2f}"

        # NaN must hold position, and must not kill the node.
        for _ in range(20):
            send_gripper(float("nan"), msg_id)
            msg_id += 1
            time.sleep(0.02)
        time.sleep(0.5)
        held = follower.read_state(timeout_s=2.0).effector.position
        assert held > 0.85, f"NaN command should hold the gripper, moved to {held:.2f}"
        process = follower._backend._process  # noqa: SLF001
        assert process is not None and process.poll() is None


def test_over_torque_warns_once_and_continues_by_default(fake_bus, session_factory):
    from openpi_control import PositionCommand

    session, follower = session_factory()
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

        fake_bus.set_reported_torque(2, 12.0)  # joint 2 torq_max is 10 Nm
        warning = "Torque safe mode disabled: sustained measured torque exceeds limit"

        def warning_count() -> int:
            return "".join(follower._backend._log_lines).count(warning)  # noqa: SLF001

        wait_for(lambda: warning_count() == 1, timeout_s=90.0, what="warning-only over-torque")
        time.sleep(0.5)
        assert warning_count() == 1

        backend = follower._backend  # noqa: SLF001
        process = backend._process
        assert process is not None and process.poll() is None
        log = "".join(backend._log_lines)
        warning_lines = [line for line in backend._log_lines if warning in line]
        assert len(warning_lines) == 1
        assert "launch with --safety_torque_mode" in warning_lines[0]
        assert "servo ID 2" in warning_lines[0]
        assert "current=" in warning_lines[0]
        assert "torq_max=10.000 Nm" in warning_lines[0]
        assert "Protective stop: Torque limit exceeded" not in log

        target = [0.2, 0.25, 0.3, 0.0, 0.1, -0.1]
        follower.command(PositionCommand(target))
        wait_for(
            lambda: abs(follower.read_state(timeout_s=2.0).joints.position_rad[0] - target[0])
            < 0.02,
            timeout_s=20.0,
            what="commands to continue after warning-only over-torque",
        )

        fake_bus.set_reported_torque(2, 0.0)
        healthy_start = fake_bus.mit_command_count(2)
        wait_for(
            lambda: fake_bus.mit_command_count(2) >= healthy_start + 120,
            timeout_s=60.0,
            what="over-torque warning hysteresis to re-arm",
        )
        fake_bus.set_reported_torque(2, 12.0)
        wait_for(lambda: warning_count() == 2, timeout_s=90.0, what="re-armed over-torque warning")
        assert process.poll() is None
        fake_bus.set_reported_torque(2, 0.0)


def test_attached_effector_over_torque_warns_and_continues_by_default(
    fake_bus_with_gripper, session_factory
):
    session, follower = session_factory(effector_model="E_Yam")
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

        fake_bus_with_gripper.set_reported_torque(7, 8.0)  # E_Yam torq_max is 7 Nm

        def effector_warning_logged() -> bool:
            log = "".join(follower._backend._log_lines)  # noqa: SLF001
            return (
                "Torque safe mode disabled: sustained measured torque exceeds limit" in log
                and "servo ID 7" in log
                and "torq_max=7.000 Nm" in log
            )

        wait_for(
            effector_warning_logged,
            timeout_s=90.0,
            what="attached effector over-torque warning",
        )
        process = follower._backend._process  # noqa: SLF001
        assert process is not None and process.poll() is None
        follower.read_state(timeout_s=5.0)
        fake_bus_with_gripper.set_reported_torque(7, 0.0)


def test_over_torque_on_middle_joint_escalates_to_protective_stop(fake_bus, session_factory):
    # With safety_torque_mode explicitly enabled, sustained over-torque remains
    # a protective-stop trigger: SafeMode returns SAFE_MODE_TOR so the main
    # loop enters emergency recovery. Pre-fix, DeviceArm::read_hardware_values
    # let the next healthy joint's SUCCESS overwrite the code, so only a fault
    # on the LAST joint ever escalated. Joint 2 (not last) must escalate too.
    session, follower = session_factory(safety_torque_mode=True)
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

        fake_bus.set_reported_torque(2, 12.0)  # joint 2 torq_max is 10 Nm

        def protective_stop_logged() -> bool:
            log = "".join(follower._backend._log_lines)  # noqa: SLF001
            return "Protective stop: Torque limit exceeded" in log

        # 300 cycles at 100 Hz is ~3 s on bare metal; valgrind runs the loop
        # several times slower, so the budget is generous.
        wait_for(protective_stop_logged, timeout_s=90.0, what="over-torque protective stop")
        fake_bus.set_reported_torque(2, 0.0)


@pytest.mark.skip(
    reason="quarantined 2026-07-23: the client heartbeat thread cannot hold its 1 Hz "
    "cadence on current hosted runners -- it sends exactly once and then a bounded "
    "1 s wait (Event.wait and plain time.sleep alike) oversleeps for the rest of the "
    "test, so the node never arms its dead-client watchdog. Not a code regression: "
    "the untouched last-green commit 418218a fails identically on the same runner "
    "image (20260714.228.1) that passed it on 2026-07-20. Evidence in PR #5/#6 "
    "debug runs. Needs investigation on a live runner (strace of the stalled thread)."
)
def test_dead_client_watchdog_idles_the_node(fake_bus, session_factory):
    # kill -9 of the operator's process used to leave the native nodes
    # teleoperating unsupervised indefinitely (the leader node keeps feeding
    # the follower's live stream; nothing watches safety). The client now
    # sends 1 Hz heartbeats and the node, once it has seen one, drops to its
    # safe idle after 5 s of silence: leaders fall back to gravity float,
    # followers pause leader listening and hold. Emulate abrupt client death
    # by stopping only the heartbeat thread -- the rest of the client stays
    # alive so the test can still observe the node.
    session, follower = session_factory()
    with session:
        session.connect()
        follower.read_state(timeout_s=10.0)

        backend = follower._backend  # noqa: SLF001 - simulating client death
        backend._heartbeat_stop.set()
        assert backend._heartbeat is not None
        backend._heartbeat.join(timeout=5.0)

        def watchdog_tripped() -> bool:
            return "client heartbeat lost" in "".join(backend._log_lines)

        # 5 s of required silence plus generous valgrind margin.
        wait_for(watchdog_tripped, timeout_s=60.0, what="dead-client watchdog to trip")

        # The node idles but stays alive and keeps publishing state.
        follower.read_state(timeout_s=5.0)
        process = backend._process
        assert process is not None and process.poll() is None


def test_leader_recovery_goes_silent_instead_of_steering_the_follower(fake_bus_with_handle):
    # A leader that faults mid-session (e.g. sustained over-torque) drives
    # ITSELF home via emergency recovery. Its state topic doubles as the
    # paired follower's live-command stream, so pre-fix the follower
    # faithfully replayed the leader's self-homing sweep across the workspace
    # with whatever it was gripping. The leader must stop publishing
    # observations the moment recovery starts.
    from openpi_control import ArmConfig, ArmSession, SocketCanConnection
    from openpi_control.exceptions import NativeProcessError, StateTimeoutError

    bus = fake_bus_with_handle
    for motor_id in (2, 3):
        bus.set_position(motor_id, 1.0)  # away from home: recovery has real distance to cover

    session = ArmSession()
    leader = session.add_leader(
        ArmConfig(
            "sil-leader",
            "Yam",
            SocketCanConnection(VCAN),
            effector_model="E_Yam_Handle",
            control_frequency_hz=100,
            safety_torque_mode=True,
        )
    )
    with session:
        session.connect()
        leader.read_state(timeout_s=10.0)

        bus.set_reported_torque(2, 12.0)

        def gate_logged() -> bool:
            return "leader observation publishing gated" in "".join(
                leader._backend._log_lines  # noqa: SLF001
            )

        wait_for(gate_logged, timeout_s=90.0, what="recovery publish gate")
        bus.set_reported_torque(2, 0.0)

        # From the gate onward no new observation arrives: at most the SUB's
        # small high-water mark of in-flight messages drains, then the stream
        # is silent until the node parks and exits after recovery completes.
        with pytest.raises((StateTimeoutError, NativeProcessError)):
            for _ in range(3):
                leader.read_state(timeout_s=2.0)
