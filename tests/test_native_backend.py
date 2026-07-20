"""NativeArmBackend tests against the fake native node (no hardware needed).

The spawn tests exercise the real process lifecycle — handshake, acks, state
flow, crash detection, shutdown — by pointing OPENPI_CONTROL_NODE at
tests/fake_native_node.py. They need Linux (NativeArmBackend refuses other
platforms) plus the loopback interface as a stand-in SocketCAN device; the
message-consumption tests below them run everywhere.
"""

from __future__ import annotations

import logging
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import pytest
import zmq

from openpi_control import (
    ArmConfig,
    ArmMode,
    ArmRole,
    ArmSession,
    InputLayout,
    PositionCommand,
    SafetyLimits,
    SocketCanConnection,
)
from openpi_control.exceptions import (
    CommandRejectedError,
    HardwareFaultError,
    NativeProcessError,
    ProtocolError,
)
from openpi_control.native import (
    NativeArmBackend,
    _native_process_failure,
    _Publisher,
    _Subscriber,
)
from openpi_control.protocol import (
    COMMAND_STRUCT,
    INPUT_STRUCT,
    JOINT_STRUCT,
    PROTOCOL_VERSION,
    STATUS_STRUCT,
    NativeCommand,
    NativeStatus,
    port_candidates,
    topics_for,
)

REPO_ROOT = Path(__file__).parents[1]

spawn_tests = pytest.mark.skipif(
    sys.platform != "linux" or not Path("/sys/class/net/lo").exists(),
    reason="NativeArmBackend spawning requires Linux",
)


def test_native_hardware_fault_is_actionable_and_concise():
    error = _native_process_failure(
        255,
        [
            "\x1b[33m[WARNING] noisy retry 1/10\x1b[0m\n",
            "\x1b[31m[ERROR] HARDWARE FAULT: DM servo id=7 reported status 0xC "
            "(motor coil overtemperature). Action: allow the motor to cool.\x1b[0m\n",
            "\x1b[31m[ERROR] Device start() failed: error code=-8\x1b[0m\n",
        ],
    )

    assert isinstance(error, HardwareFaultError)
    assert str(error) == (
        "HARDWARE FAULT: DM servo id=7 reported status 0xC "
        "(motor coil overtemperature). Action: allow the motor to cool."
    )
    assert "noisy retry" not in str(error)
    assert "\x1b" not in str(error)


def test_native_process_failure_summarizes_only_terminal_errors():
    error = _native_process_failure(
        3,
        ["startup detail\n", "[ERROR] first failure\n", "[ERROR] terminal failure\n"],
    )

    assert type(error) is NativeProcessError
    assert str(error) == "pi_control_node exited with 3: first failure | terminal failure"


def test_native_process_failure_preserves_root_error_before_cleanup_errors():
    error = _native_process_failure(
        3,
        [
            "[ERROR] unsupported firmware\n",
            "[ERROR] cleanup failure 1\n",
            "[ERROR] cleanup failure 2\n",
            "[ERROR] cleanup failure 3\n",
            "[ERROR] cleanup failure 4\n",
        ],
    )

    assert type(error) is NativeProcessError
    assert str(error) == (
        "pi_control_node exited with 3: unsupported firmware | "
        "cleanup failure 3 | cleanup failure 4"
    )


def test_live_hardware_fault_preempts_state_timeout():
    backend = make_consuming_backend()
    backend._running = True
    backend._hardware_fault_message = (  # noqa: SLF001 - inject native log outcome
        "HARDWARE FAULT: DM servo id=7 reported status 0xC (motor coil overtemperature)."
    )

    with pytest.raises(HardwareFaultError, match="motor coil overtemperature"):
        backend.read_state(timeout_s=0.0)


@pytest.fixture
def fake_node(tmp_path, monkeypatch):
    """Routes OPENPI_CONTROL_NODE at the protocol-faithful fake node."""

    def configure(behavior: str = "normal") -> tuple[Path, Path]:
        wrapper = tmp_path / "fake_node.sh"
        ready_marker = tmp_path / "ready.marker"
        state_release = tmp_path / "state-release.marker"
        wrapper.write_text(
            "#!/bin/sh\n"
            f'exec "{sys.executable}" "{REPO_ROOT / "tests" / "fake_native_node.py"}" "$@"\n'
        )
        wrapper.chmod(0o755)
        monkeypatch.setenv("OPENPI_CONTROL_NODE", str(wrapper))
        monkeypatch.setenv("FAKE_NODE_BEHAVIOR", behavior)
        monkeypatch.setenv("FAKE_NODE_READY_MARKER", str(ready_marker))
        monkeypatch.setenv("FAKE_NODE_STATE_RELEASE", str(state_release))
        return ready_marker, state_release

    return configure


def make_backend_config() -> ArmConfig:
    return ArmConfig("native-test", "Yam", SocketCanConnection("lo"), control_frequency_hz=100)


@pytest.mark.parametrize("safety_torque_mode", [False, True])
def test_native_backend_forwards_safety_torque_mode_only_when_enabled(
    monkeypatch, safety_torque_mode: bool
) -> None:
    captured_args: list[str] = []
    backend = NativeArmBackend()
    config = ArmConfig(
        "native-test",
        "Yam",
        SocketCanConnection("lo"),
        control_frequency_hz=100,
        safety_torque_mode=safety_torque_mode,
    )

    def capture_spawn(args, **_kwargs):
        captured_args.extend(args)
        raise RuntimeError("captured native arguments")

    monkeypatch.setattr("openpi_control.native.platform.system", lambda: "Linux")
    monkeypatch.setattr("openpi_control.native.validate_connection", lambda connection: None)
    monkeypatch.setattr("openpi_control.native.native_executable", lambda: Path(sys.executable))
    monkeypatch.setattr("openpi_control.native.subprocess.Popen", capture_spawn)

    try:
        with pytest.raises(RuntimeError, match="captured native arguments"):
            backend.connect(
                config,
                ArmRole.FOLLOWER,
                topics_for(f"torque-{safety_torque_mode}", "native-test"),
            )
    finally:
        backend.close()

    assert ("--safety_torque_mode" in captured_args) is safety_torque_mode


@pytest.mark.parametrize(("model", "expected_present"), [("ARX_X5", False), ("Yam", True)])
def test_native_backend_forwards_leader_gravity_compensation_default(
    monkeypatch, model: str, expected_present: bool
) -> None:
    captured_args: list[str] = []
    backend = NativeArmBackend()
    config = ArmConfig(
        "native-test",
        model,
        SocketCanConnection("lo"),
        control_frequency_hz=100,
    )

    def capture_spawn(args, **_kwargs):
        captured_args.extend(args)
        raise RuntimeError("captured native arguments")

    monkeypatch.setattr("openpi_control.native.platform.system", lambda: "Linux")
    monkeypatch.setattr("openpi_control.native.validate_connection", lambda connection: None)
    monkeypatch.setattr("openpi_control.native.native_executable", lambda: Path(sys.executable))
    monkeypatch.setattr("openpi_control.native.subprocess.Popen", capture_spawn)

    try:
        with pytest.raises(RuntimeError, match="captured native arguments"):
            backend.connect(
                config,
                ArmRole.LEADER,
                topics_for(f"leader-gc-{model}", "native-test"),
            )
    finally:
        backend.close()

    assert ("--leader_gravity_compensation" in captured_args) is expected_present


def test_publisher_closes_socket_when_bind_candidates_are_exhausted():
    topic = f"t-publisher-bind-failure-{time.time_ns()}"
    blocker_context = zmq.Context()
    publisher_context = zmq.Context()
    blockers: list[zmq.Socket] = []
    publisher_socket = publisher_context.socket(zmq.PUB)

    class RecordingContext:
        def socket(self, socket_type):
            assert socket_type == zmq.PUB
            return publisher_socket

    try:
        for port in port_candidates(topic, 16):
            blocker = blocker_context.socket(zmq.PUB)
            blocker.setsockopt(zmq.LINGER, 0)
            try:
                blocker.bind(f"tcp://127.0.0.1:{port}")
            except zmq.ZMQError:
                blocker.close(0)
            else:
                blockers.append(blocker)

        with pytest.raises(NativeProcessError, match="unable to bind publisher"):
            _Publisher(RecordingContext(), topic)

        assert publisher_socket.closed
    finally:
        publisher_socket.close(0)
        publisher_context.term()
        for blocker in blockers:
            blocker.close(0)
        blocker_context.term()


def test_publisher_closes_socket_when_option_setup_fails():
    class FailingSocket:
        def __init__(self) -> None:
            self.option_calls = 0
            self.closed = False

        def setsockopt(self, option, value):
            del option, value
            self.option_calls += 1
            if self.option_calls == 2:
                raise zmq.ZMQError(zmq.EINVAL)

        def close(self, linger=0):
            del linger
            self.closed = True

    class RecordingContext:
        def __init__(self) -> None:
            self.socket_instance = FailingSocket()

        def socket(self, socket_type):
            assert socket_type == zmq.PUB
            return self.socket_instance

    context = RecordingContext()

    with pytest.raises(zmq.ZMQError):
        _Publisher(context, "t-publisher-option-failure")

    assert context.socket_instance.closed


def test_subscriber_closes_socket_when_connect_fails():
    class FailingSocket:
        def __init__(self) -> None:
            self.closed = False

        def setsockopt(self, option, value):
            del option, value

        def connect(self, endpoint):
            del endpoint
            raise zmq.ZMQError(zmq.EINVAL)

        def close(self, linger=0):
            del linger
            self.closed = True

    class RecordingContext:
        def __init__(self) -> None:
            self.socket_instance = FailingSocket()

        def socket(self, socket_type):
            assert socket_type == zmq.SUB
            return self.socket_instance

    context = RecordingContext()

    with pytest.raises(zmq.ZMQError):
        _Subscriber(context, "t-subscriber-connect-failure")

    assert context.socket_instance.closed


def test_concurrent_connects_cannot_both_prepare_backend(monkeypatch):
    backend = NativeArmBackend()
    config = make_backend_config()
    topics = topics_for("t-concurrent-connect", "native-test")
    first_prepared = threading.Event()
    second_prepared = threading.Event()
    second_call_started = threading.Event()
    release_first = threading.Event()
    first_errors: list[BaseException] = []
    second_errors: list[BaseException] = []

    monkeypatch.setattr("openpi_control.native.platform.system", lambda: "Linux")
    monkeypatch.setattr("openpi_control.native.validate_connection", lambda connection: None)
    monkeypatch.setattr("openpi_control.native.native_executable", lambda: Path(sys.executable))

    def connect_prepared(*args):
        del args
        if threading.current_thread().name == "first-connect":
            first_prepared.set()
            assert release_first.wait(timeout=2.0)
        else:
            second_prepared.set()
        with backend._condition:
            backend._running = True
        return object()

    backend._connect_prepared = connect_prepared
    first = threading.Thread(
        target=lambda: capture_error(
            lambda: backend.connect(config, ArmRole.FOLLOWER, topics), first_errors
        ),
        name="first-connect",
    )

    def connect_again() -> None:
        second_call_started.set()
        capture_error(lambda: backend.connect(config, ArmRole.FOLLOWER, topics), second_errors)

    second = threading.Thread(target=connect_again, name="second-connect")
    try:
        first.start()
        assert first_prepared.wait(timeout=1.0)
        second.start()
        assert second_call_started.wait(timeout=1.0)
        assert not second_prepared.wait(timeout=0.2)

        release_first.set()
        first.join(timeout=1.0)
        second.join(timeout=1.0)
    finally:
        release_first.set()
        first.join(timeout=1.0)
        second.join(timeout=1.0)
        backend.close()

    assert not first.is_alive()
    assert not second.is_alive()
    assert not first_errors
    assert len(second_errors) == 1
    assert isinstance(second_errors[0], NativeProcessError)


def test_close_waits_for_inflight_connect(monkeypatch):
    backend = NativeArmBackend()
    config = make_backend_config()
    topics = topics_for("t-close-during-connect", "native-test")
    connect_prepared = threading.Event()
    close_started = threading.Event()
    close_returned = threading.Event()
    release_connect = threading.Event()
    connect_errors: list[BaseException] = []
    close_errors: list[BaseException] = []

    monkeypatch.setattr("openpi_control.native.platform.system", lambda: "Linux")
    monkeypatch.setattr("openpi_control.native.validate_connection", lambda connection: None)
    monkeypatch.setattr("openpi_control.native.native_executable", lambda: Path(sys.executable))

    def finish_connect(*args):
        del args
        connect_prepared.set()
        assert release_connect.wait(timeout=2.0)
        with backend._condition:
            backend._running = True
        return object()

    backend._connect_prepared = finish_connect
    connect = threading.Thread(
        target=lambda: capture_error(
            lambda: backend.connect(config, ArmRole.FOLLOWER, topics), connect_errors
        )
    )

    def close_backend() -> None:
        close_started.set()
        capture_error(backend.close, close_errors)
        close_returned.set()

    close = threading.Thread(target=close_backend)
    try:
        connect.start()
        assert connect_prepared.wait(timeout=1.0)
        close.start()
        assert close_started.wait(timeout=1.0)
        assert not close_returned.wait(timeout=0.2)

        release_connect.set()
        connect.join(timeout=1.0)
        close.join(timeout=1.0)

        assert not connect.is_alive()
        assert not close.is_alive()
        assert not connect_errors
        assert not close_errors
        assert not backend._running
    finally:
        release_connect.set()
        connect.join(timeout=1.0)
        close.join(timeout=1.0)
        with backend._condition:
            backend._running = False
            backend._closed = False
        backend.close()


@spawn_tests
def test_full_lifecycle_against_fake_node(fake_node):
    fake_node("normal")
    backend = NativeArmBackend()
    try:
        capabilities = backend.connect(
            make_backend_config(), ArmRole.FOLLOWER, topics_for("t1", "native-test")
        )
        assert capabilities.protocol_version == PROTOCOL_VERSION
        assert capabilities.supports_direct_commands
        assert capabilities.supports_move_to_ready

        state = backend.read_state(timeout_s=5.0)
        assert len(state.joints.names) == 6

        backend.command(PositionCommand([0.25] * 6))
        deadline_state = backend.read_state(timeout_s=5.0)
        for _ in range(200):
            deadline_state = backend.read_state(timeout_s=5.0)
            if abs(deadline_state.joints.position_rad[0] - 0.25) < 1e-6:
                break
        assert abs(deadline_state.joints.position_rad[0] - 0.25) < 1e-6

        backend.hold()  # lifecycle round-trip: ack with result 0
    finally:
        backend.close()
    assert backend._process is not None and backend._process.returncode == 0


@spawn_tests
def test_backend_can_reconnect_after_close(fake_node):
    fake_node("normal")
    backend = NativeArmBackend()
    first_process = None
    second_process = None
    try:
        topics = topics_for("t-reconnect", "native-test")
        backend.connect(make_backend_config(), ArmRole.FOLLOWER, topics)
        first_process = backend._process
        backend.close()
        assert first_process is not None and first_process.returncode == 0

        backend.connect(make_backend_config(), ArmRole.FOLLOWER, topics)
        second_process = backend._process
        assert second_process is not None and second_process is not first_process
        backend.close()

        assert second_process.returncode == 0
    finally:
        for process in (first_process, second_process, backend._process):
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2)


@spawn_tests
def test_reconnect_after_process_exit_retires_previous_resources(fake_node):
    fake_node("normal")
    backend = NativeArmBackend()
    old_process = None
    replacement_process = None
    old_stop = None
    old_threads: tuple[threading.Thread, ...] = ()
    old_sockets = ()
    try:
        topics = topics_for("t-exited-reconnect", "native-test")
        backend.connect(make_backend_config(), ArmRole.FOLLOWER, topics)
        old_process = backend._process
        old_stop = backend._heartbeat_stop
        old_threads = tuple(
            thread
            for thread in (backend._reader, backend._log_reader, backend._heartbeat)
            if thread is not None
        )
        old_sockets = tuple(
            socket
            for socket in (
                backend._state_sub,
                backend._status_sub,
                backend._inputs_sub,
                backend._direct_pub,
                backend._lifecycle_pub,
            )
            if socket is not None
        )

        assert old_process is not None
        old_process.kill()
        old_process.wait(timeout=2)
        deadline = time.monotonic() + 2.0
        while backend._running and time.monotonic() < deadline:
            time.sleep(0.005)
        assert not backend._running

        backend.connect(make_backend_config(), ArmRole.FOLLOWER, topics)
        replacement_process = backend._process

        assert old_stop.is_set()
        assert all(not thread.is_alive() for thread in old_threads)
        assert all(socket.socket.closed for socket in old_sockets)
        assert replacement_process is not None and replacement_process is not old_process
    finally:
        if old_stop is not None:
            old_stop.set()
        for socket in old_sockets:
            socket.close()
        for thread in old_threads:
            thread.join(timeout=1)
        backend.close()
        for process in (old_process, replacement_process, backend._process):
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2)


@spawn_tests
def test_session_reconnects_arm_after_native_process_exit(fake_node):
    fake_node("normal")
    backend = NativeArmBackend()
    session = ArmSession(session_id="t-session-exited-reconnect")
    follower = session.add_follower(make_backend_config(), backend=backend)
    old_process = None
    replacement_process = None
    try:
        session.connect()
        old_process = backend._process
        assert old_process is not None

        old_process.kill()
        old_process.wait(timeout=2)
        deadline = time.monotonic() + 2.0
        while backend._running and time.monotonic() < deadline:
            time.sleep(0.005)
        assert not backend._running

        session.connect()
        replacement_process = backend._process

        assert replacement_process is not None and replacement_process is not old_process
        assert replacement_process.poll() is None
        assert follower.connected
    finally:
        session.close()
        for process in (old_process, replacement_process, backend._process):
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2)


@spawn_tests
def test_session_disengages_pair_before_reconnecting_dead_arm(fake_node):
    fake_node("normal")
    leader_backend = NativeArmBackend()
    follower_backend = NativeArmBackend()
    session = ArmSession(session_id="t-pair-exited-reconnect")
    leader = session.add_leader(
        ArmConfig("native-leader", "Yam", SocketCanConnection("lo"), control_frequency_hz=100),
        backend=leader_backend,
    )
    follower = session.add_follower(
        ArmConfig("native-follower", "Yam", SocketCanConnection("lo"), control_frequency_hz=100),
        backend=follower_backend,
    )
    pair = session.add_pair(
        leader,
        follower,
        safety_limits=SafetyLimits(minimum_alignment_duration_s=0.01),
    )
    old_follower_process = None
    replacement_process = None
    try:
        session.connect()
        pair.engage()
        assert pair.engaged
        assert leader.mode is ArmMode.BILATERAL

        old_follower_process = follower_backend._process
        assert old_follower_process is not None
        old_follower_process.kill()
        old_follower_process.wait(timeout=2)
        deadline = time.monotonic() + 2.0
        while follower_backend._running and time.monotonic() < deadline:
            time.sleep(0.005)
        assert not follower_backend._running

        session.connect()
        replacement_process = follower_backend._process

        assert replacement_process is not None and replacement_process is not old_follower_process
        assert not pair.engaged
        assert leader.mode is ArmMode.GRAVITY_COMPENSATION
    finally:
        session.close()
        for process in (
            old_follower_process,
            replacement_process,
            leader_backend._process,
            follower_backend._process,
        ):
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2)


@spawn_tests
def test_move_to_ready_waits_for_correlated_ready_and_post_completion_state(fake_node):
    ready_marker, state_release = fake_node("stale-ready-after-ack")
    backend = NativeArmBackend()
    move_errors: list[BaseException] = []
    move_thread: threading.Thread | None = None
    try:
        backend.connect(
            make_backend_config(), ArmRole.FOLLOWER, topics_for("t-stale", "native-test")
        )
        backend.command(PositionCommand([0.4] * 6))
        for _ in range(200):
            state = backend.read_state(timeout_s=5.0)
            if abs(state.joints.position_rad[0] - 0.4) < 1e-6:
                break
        assert abs(state.joints.position_rad[0] - 0.4) < 1e-6

        move_thread = threading.Thread(
            target=lambda: capture_error(backend.move_to_ready, move_errors),
        )
        move_thread.start()
        deadline = time.monotonic() + 5.0
        while not ready_marker.exists() and time.monotonic() < deadline:
            time.sleep(0.005)

        assert ready_marker.exists()
        move_thread.join(timeout=0.5)
        assert move_thread.is_alive()

        state_release.touch()
        move_thread.join(timeout=5.0)

        assert not move_thread.is_alive()
        assert not move_errors
        assert backend.latest_state() is not None
        assert backend.latest_state().joints.position_rad == pytest.approx([0.0] * 6)
    finally:
        state_release.touch()
        if move_thread is not None and move_thread.is_alive():
            move_thread.join(timeout=1.0)
        backend.close()


@spawn_tests
def test_incompatible_protocol_version_fails_connect(fake_node):
    fake_node("bad-version")
    backend = NativeArmBackend()
    try:
        with pytest.raises(NativeProcessError, match="protocol"):
            backend.connect(
                make_backend_config(), ArmRole.FOLLOWER, topics_for("t2", "native-test")
            )
    finally:
        backend.close()


@spawn_tests
def test_backend_can_reconnect_after_failed_connect(fake_node):
    fake_node("bad-version")
    backend = NativeArmBackend()
    failed_process = None
    replacement_process = None
    try:
        topics = topics_for("t-failed-reconnect", "native-test")
        with pytest.raises(NativeProcessError, match="protocol"):
            backend.connect(make_backend_config(), ArmRole.FOLLOWER, topics)
        failed_process = backend._process

        fake_node("normal")
        capabilities = backend.connect(make_backend_config(), ArmRole.FOLLOWER, topics)
        replacement_process = backend._process

        assert capabilities.protocol_version == PROTOCOL_VERSION
        assert failed_process is not None and failed_process.poll() is not None
        assert replacement_process is not None and replacement_process is not failed_process
    finally:
        backend.close()
        for process in (failed_process, replacement_process, backend._process):
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2)


@spawn_tests
def test_process_exit_before_handshake_fails_connect(fake_node):
    fake_node("exit-early")
    backend = NativeArmBackend()
    try:
        with pytest.raises(NativeProcessError, match="exited with 3"):
            backend.connect(
                make_backend_config(), ArmRole.FOLLOWER, topics_for("t3", "native-test")
            )
    finally:
        backend.close()


@spawn_tests
def test_rejected_lifecycle_command_raises(fake_node):
    fake_node("reject-commands")
    backend = NativeArmBackend()
    try:
        # connect() itself sends no lifecycle commands, so it succeeds.
        backend.connect(make_backend_config(), ArmRole.FOLLOWER, topics_for("t4", "native-test"))
        with pytest.raises(CommandRejectedError, match="code 7"):
            backend.hold()
    finally:
        backend.close()


# ---------------------------------------------------------------------------
# Message-consumption unit tests (no process spawn; run on all platforms).
# ---------------------------------------------------------------------------


def make_consuming_backend(role: ArmRole = ArmRole.FOLLOWER) -> NativeArmBackend:
    backend = NativeArmBackend()
    backend._config = ArmConfig("native-test", "Yam", SocketCanConnection("test"))
    backend._role = role
    return backend


def pack_state(
    positions: list[float],
    sequence: int = 1,
    joint_count: int = 6,
    *,
    velocities: list[float] | None = None,
    efforts: list[float] | None = None,
    temperatures: list[float] | None = None,
    currents: list[float] | None = None,
    frame_ages: list[float] | None = None,
) -> bytes:
    padded = positions + [0.0] * (10 - len(positions))
    arrays = []
    for values in (velocities, efforts, temperatures, currents):
        values = values or []
        arrays.append(values + [0.0] * (10 - len(values)))
    # Default to fresh frames (small constant age) unless a test overrides it.
    ages = frame_ages if frame_ages is not None else [2.0] * 10
    ages = ages + [0.0] * (10 - len(ages))
    return JOINT_STRUCT.pack(
        *padded,
        *arrays[0],
        *arrays[1],
        *arrays[2],
        *arrays[3],
        *ages,
        sequence,
        joint_count,
        1,
        0.0,
    )


def pack_status(status: NativeStatus, ints: tuple[int, ...] = ()) -> bytes:
    return STATUS_STRUCT.pack(
        int(status), 0, len(ints), *((0.0,) * 10), *(ints + (0,) * (10 - len(ints)))
    )


def pack_inputs(buttons: tuple[int, ...] = (1,)) -> bytes:
    return INPUT_STRUCT.pack(
        0,
        0,
        *((0.0,) * 5),
        0,
        *(buttons + (0,) * (5 - len(buttons))),
        len(buttons),
        *((0,) * 5),
    )


def capture_error(callback, errors: list[BaseException]) -> None:
    try:
        callback()
    except BaseException as exc:  # noqa: BLE001 - captured for a test thread
        errors.append(exc)


class ObservedCondition:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self.wait_started = threading.Event()

    def __enter__(self):
        self._condition.acquire()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self._condition.release()

    def wait(self, timeout=None):
        self.wait_started.set()
        return self._condition.wait(timeout)

    def notify_all(self) -> None:
        self._condition.notify_all()


def test_consume_state_rejects_wrong_payload_size():
    backend = make_consuming_backend()
    with pytest.raises(ProtocolError, match="payload size"):
        backend._consume_state(b"\x00" * 7)
    backend.close()


def test_consume_state_rejects_too_few_joints():
    backend = make_consuming_backend()
    with pytest.raises(ProtocolError, match="expected at least 6"):
        backend._consume_state(pack_state([0.0] * 6, joint_count=3))
    backend.close()


def test_consume_state_extracts_effector_when_extra_joint_present():
    backend = make_consuming_backend()
    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.77]
    backend._consume_state(
        pack_state(
            positions,
            joint_count=7,
            velocities=[0.0] * 6 + [0.12],
            efforts=[0.0] * 6 + [1.1],
            temperatures=[25.0] * 6 + [72.0],
            currents=[0.0] * 6 + [2.3],
            frame_ages=[5.0] * 6 + [321.0],
        )
    )
    assert backend._state is not None
    assert backend._state.effector is not None
    assert backend._state.effector.position == pytest.approx(0.77)
    assert backend._state.effector.velocity_s == pytest.approx(0.12)
    assert backend._state.effector.effort_nm == pytest.approx(1.1)
    assert backend._state.effector.temperature_c == pytest.approx(72.0)
    assert backend._state.effector.current_a == pytest.approx(2.3)
    assert backend._state.effector.frame_age_ms == pytest.approx(321.0)
    assert backend._state.joints.frame_age_ms == pytest.approx([5.0] * 6)
    backend.close()


def test_subscriber_recv_latest_returns_newest_and_counts_discards():
    context = zmq.Context()
    publisher = _Publisher(context, "t-recv-latest")
    subscriber = _Subscriber(context, "t-recv-latest")
    try:
        # PUB/SUB slow joiner: keep probing until the subscription propagates.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            publisher.send(b"probe")
            payload, _ = subscriber.recv_latest()
            if payload is not None:
                break
            time.sleep(0.01)
        else:
            pytest.fail("subscriber never received the probe message")

        for index in range(20):
            publisher.send(bytes([index]))
            time.sleep(0.001)  # let the io thread flush past the SNDHWM of 2

        latest: bytes | None = None
        discarded_total = 0
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and latest != bytes([19]):
            payload, discarded = subscriber.recv_latest()
            discarded_total += discarded
            if payload is not None:
                latest = payload
            time.sleep(0.01)

        assert latest == bytes([19])
        assert discarded_total >= 1
    finally:
        publisher.close()
        subscriber.close()
        context.term()


class QueueSubscriber:
    """Reader-loop test double mirroring the _Subscriber drain interface."""

    def __init__(self, payloads: list[bytes]) -> None:
        self.queue = deque(payloads)

    def recv(self) -> bytes | None:
        return self.queue.popleft() if self.queue else None

    def recv_latest(self) -> tuple[bytes | None, int]:
        latest: bytes | None = None
        discarded = 0
        while self.queue:
            if latest is not None:
                discarded += 1
            latest = self.queue.popleft()
        return latest, discarded

    def close(self) -> None:
        self.queue.clear()


def test_reader_loop_consumes_state_backlog_as_single_newest_sample(caplog):
    backend = make_consuming_backend()
    backlog = [pack_state([0.001 * index] * 6, sequence=index + 1) for index in range(60)]
    backend._state_sub = QueueSubscriber(backlog)
    backend._running = True
    reader = threading.Thread(target=backend._reader_loop)
    backend._reader = reader

    with caplog.at_level(logging.WARNING, logger="openpi_control.native"):
        try:
            reader.start()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                with backend._condition:
                    if backend._state is not None:
                        break
                time.sleep(0.005)
        finally:
            with backend._condition:
                backend._running = False
            reader.join(timeout=2.0)
            backend.close()

    assert not reader.is_alive()
    assert backend._state is not None
    # Only the newest sample is observed; the 59 stale ones are discarded
    # instead of being replayed one per poll (which would lag the state feed).
    assert backend._state.joints.position_rad == pytest.approx([0.001 * 59] * 6)
    assert backend._state_generation == 1
    assert any("59 stale state message(s)" in record.getMessage() for record in caplog.records)


def test_read_state_returns_next_packet_when_sequence_repeats():
    backend = make_consuming_backend()
    condition = ObservedCondition()
    backend._condition = condition
    backend._consume_state(pack_state([0.0] * 6, sequence=7))
    backend._running = True
    states = []
    errors: list[BaseException] = []
    reader = threading.Thread(
        target=lambda: capture_error(lambda: states.append(backend.read_state(0.5)), errors)
    )

    try:
        reader.start()
        assert condition.wait_started.wait(timeout=1.0)
        backend._consume_state(pack_state([0.2] * 6, sequence=7))
        reader.join(timeout=1.0)
    finally:
        backend.close()
        reader.join(timeout=1.0)

    assert not reader.is_alive()
    assert not errors
    assert len(states) == 1
    assert states[0].joints.position_rad == pytest.approx([0.2] * 6)


def test_read_inputs_rejects_cached_sample_after_close():
    backend = make_consuming_backend(ArmRole.LEADER)
    backend._input_layout = InputLayout(button_names=("button",))
    backend._consume_inputs(pack_inputs())
    backend.close()

    with pytest.raises(NativeProcessError, match="not running"):
        backend.read_inputs()


def test_blocking_input_reader_rejects_a_reconnected_backend_generation():
    backend = make_consuming_backend(ArmRole.LEADER)
    condition = ObservedCondition()
    backend._condition = condition
    backend._input_layout = InputLayout(button_names=("button",))
    backend._reader = threading.Thread()
    backend._running = True
    inputs = []
    errors: list[BaseException] = []
    reader = threading.Thread(
        target=lambda: capture_error(lambda: inputs.append(backend.read_inputs(None)), errors)
    )

    try:
        reader.start()
        assert condition.wait_started.wait(timeout=1.0)
        with condition:
            backend._closed = True
            backend._running = False
            condition.notify_all()
            backend._inputs = None
            backend._reader = threading.Thread()
            backend._reader_error = None
            backend._closed = False
            backend._running = True
            backend._consume_inputs(pack_inputs())
        reader.join(timeout=1.0)
    finally:
        backend.close()
        reader.join(timeout=1.0)

    assert not reader.is_alive()
    assert not inputs
    assert len(errors) == 1
    assert isinstance(errors[0], NativeProcessError)


def test_consume_status_handshake_version_mismatch_raises():
    backend = make_consuming_backend()
    with pytest.raises(ProtocolError, match="incompatible native protocol"):
        backend._consume_status(pack_status(NativeStatus.HANDSHAKE, (9, 9, 0)))
    backend.close()


def test_consume_status_mode_update_replaces_state_mode():
    backend = make_consuming_backend()
    backend._consume_state(pack_state([0.0] * 6))
    modes = list(ArmMode)
    backend._consume_status(pack_status(NativeStatus.MODE, (modes.index(ArmMode.RECOVERY),)))
    assert backend._state is not None and backend._state.mode is ArmMode.RECOVERY
    # An out-of-range mode index is ignored rather than crashing the reader.
    backend._consume_status(pack_status(NativeStatus.MODE, (250,)))
    assert backend._state.mode is ArmMode.RECOVERY
    backend.close()


def test_state_packet_cannot_overwrite_a_concurrent_mode_transition():
    class CoordinatedCondition:
        def __init__(self) -> None:
            self._lock = threading.Lock()
            self.state_publish_started = threading.Event()
            self.release_state_publish = threading.Event()

        def __enter__(self):
            if threading.current_thread().name == "delayed-state-publish":
                self.state_publish_started.set()
                assert self.release_state_publish.wait(timeout=1.0)
            self._lock.acquire()
            return self

        def __exit__(self, exc_type, exc_value, traceback) -> None:
            self._lock.release()

        def notify_all(self) -> None:
            pass

    backend = make_consuming_backend()
    backend._consume_state(pack_state([0.0] * 6, sequence=1))
    condition = CoordinatedCondition()
    backend._condition = condition
    errors: list[BaseException] = []
    state_thread = threading.Thread(
        target=lambda: capture_error(
            lambda: backend._consume_state(pack_state([0.1] * 6, sequence=2)), errors
        ),
        name="delayed-state-publish",
    )

    try:
        state_thread.start()
        assert condition.state_publish_started.wait(timeout=1.0)
        backend._replace_mode(ArmMode.RECOVERY)
        condition.release_state_publish.set()
        state_thread.join(timeout=1.0)

        assert not state_thread.is_alive()
        assert not errors
        assert backend._state is not None
        assert backend._state.mode is ArmMode.RECOVERY
    finally:
        condition.release_state_publish.set()
        state_thread.join(timeout=1.0)
        backend.close()


def test_concurrent_mode_transitions_are_serialized():
    backend = make_consuming_backend(ArmRole.LEADER)
    backend._consume_state(pack_state([0.0] * 6))
    first_send_started = threading.Event()
    release_first_send = threading.Event()
    second_call_started = threading.Event()
    second_send_started = threading.Event()
    commands = []
    errors: list[BaseException] = []

    def send_mode(command, *args, **kwargs):
        del args, kwargs
        commands.append(command)
        if len(commands) == 1:
            first_send_started.set()
            assert release_first_send.wait(timeout=2.0)
        else:
            second_send_started.set()

    def set_bilateral() -> None:
        second_call_started.set()
        capture_error(lambda: backend.set_mode(ArmMode.BILATERAL), errors)

    backend._send_lifecycle = send_mode
    first = threading.Thread(
        target=lambda: capture_error(
            lambda: backend.set_mode(ArmMode.GRAVITY_COMPENSATION), errors
        )
    )
    second = threading.Thread(target=set_bilateral)

    try:
        first.start()
        assert first_send_started.wait(timeout=1.0)
        second.start()
        assert second_call_started.wait(timeout=1.0)
        assert not second_send_started.wait(timeout=0.1)

        release_first_send.set()
        first.join(timeout=1.0)
        assert not first.is_alive()
        assert second_send_started.wait(timeout=1.0)
        second.join(timeout=1.0)
    finally:
        release_first_send.set()
        first.join(timeout=1.0)
        second.join(timeout=1.0)
        backend.close()

    assert not second.is_alive()
    assert not errors
    assert backend._state is not None
    assert backend._state.mode is ArmMode.BILATERAL


def test_mode_transition_rejects_a_reconnected_backend_generation():
    backend = make_consuming_backend(ArmRole.LEADER)
    backend._reader = threading.Thread()
    backend._running = True
    backend._consume_state(pack_state([0.0] * 6))
    backend._send_lifecycle = lambda *args, **kwargs: None
    original_replace_mode = backend._replace_mode
    replace_started = threading.Event()
    release_replace = threading.Event()
    errors: list[BaseException] = []

    def delayed_replace(mode, *args, **kwargs):
        replace_started.set()
        assert release_replace.wait(timeout=2.0)
        return original_replace_mode(mode, *args, **kwargs)

    backend._replace_mode = delayed_replace
    transition = threading.Thread(
        target=lambda: capture_error(lambda: backend.set_mode(ArmMode.BILATERAL), errors)
    )

    try:
        transition.start()
        assert replace_started.wait(timeout=1.0)
        with backend._condition:
            backend._closed = True
            backend._running = False
            backend._condition.notify_all()
            backend._state = None
            backend._reader = threading.Thread()
            backend._reader_error = None
            backend._closed = False
            backend._running = True
            backend._consume_state(pack_state([0.2] * 6))
        release_replace.set()
        transition.join(timeout=1.0)
    finally:
        release_replace.set()
        transition.join(timeout=1.0)
        backend.close()

    assert not transition.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], NativeProcessError)
    assert backend._state is not None
    assert backend._state.mode is ArmMode.GRAVITY_COMPENSATION


def test_late_duplicate_ack_does_not_accumulate_after_command_completes():
    class RecordingPublisher:
        def __init__(self) -> None:
            self.sent = threading.Event()

        def send(self, payload: bytes) -> None:
            del payload
            self.sent.set()

        def close(self) -> None:
            pass

    backend = make_consuming_backend()
    publisher = RecordingPublisher()
    backend._lifecycle_pub = publisher
    backend._running = True
    errors: list[BaseException] = []
    command = threading.Thread(target=lambda: capture_error(backend.hold, errors))

    try:
        command.start()
        assert publisher.sent.wait(timeout=1.0)
        backend._consume_status(pack_status(NativeStatus.COMMAND_ACK, (1, 0)))
        command.join(timeout=1.0)
        assert not command.is_alive()

        backend._consume_status(pack_status(NativeStatus.COMMAND_ACK, (1, 0)))
        assert backend._acks == {}
    finally:
        backend._consume_status(pack_status(NativeStatus.COMMAND_ACK, (1, 0)))
        command.join(timeout=1.0)
        backend.close()

    assert not errors


def test_concurrent_lifecycle_commands_are_serialized_until_ack():
    class RecordingPublisher:
        def __init__(self) -> None:
            self.sent: list[tuple[NativeCommand, int]] = []
            self.pause_sent = threading.Event()
            self.resume_sent = threading.Event()

        def send(self, payload: bytes) -> None:
            values = COMMAND_STRUCT.unpack(payload)
            command = NativeCommand(values[0])
            request_id = values[13]
            self.sent.append((command, request_id))
            if command is NativeCommand.PAUSE_LIVE_INPUT:
                self.pause_sent.set()
            elif command is NativeCommand.RESUME_LIVE_INPUT:
                self.resume_sent.set()

        def close(self) -> None:
            pass

    backend = make_consuming_backend()
    publisher = RecordingPublisher()
    backend._lifecycle_pub = publisher
    backend._running = True
    second_call_started = threading.Event()
    errors: list[BaseException] = []
    pause = threading.Thread(
        target=lambda: capture_error(lambda: backend.pause_live_input(True), errors)
    )

    def resume_live_input() -> None:
        second_call_started.set()
        capture_error(lambda: backend.pause_live_input(False), errors)

    resume = threading.Thread(target=resume_live_input)

    try:
        pause.start()
        assert publisher.pause_sent.wait(timeout=1.0)
        resume.start()
        assert second_call_started.wait(timeout=1.0)
        assert not publisher.resume_sent.wait(timeout=0.1)

        backend._consume_status(pack_status(NativeStatus.COMMAND_ACK, (1, 0)))
        pause.join(timeout=1.0)
        assert not pause.is_alive()
        assert publisher.resume_sent.wait(timeout=1.0)
        backend._consume_status(pack_status(NativeStatus.COMMAND_ACK, (2, 0)))
        resume.join(timeout=1.0)
    finally:
        backend._consume_status(pack_status(NativeStatus.COMMAND_ACK, (1, 0)))
        backend._consume_status(pack_status(NativeStatus.COMMAND_ACK, (2, 0)))
        pause.join(timeout=1.0)
        resume.join(timeout=1.0)
        backend.close()

    assert not resume.is_alive()
    assert not errors
    assert publisher.sent == [
        (NativeCommand.PAUSE_LIVE_INPUT, 1),
        (NativeCommand.RESUME_LIVE_INPUT, 2),
    ]


def test_failed_native_connect_preserves_error_when_cleanup_fails(monkeypatch):
    backend = NativeArmBackend()
    config = make_backend_config()
    topics = topics_for("t-failed-native-connect-cleanup", "native-test")
    original_close = backend.close
    close_attempts = 0

    monkeypatch.setattr("openpi_control.native.platform.system", lambda: "Linux")
    monkeypatch.setattr("openpi_control.native.validate_connection", lambda connection: None)
    monkeypatch.setattr("openpi_control.native.native_executable", lambda: Path(sys.executable))
    monkeypatch.setattr(backend, "_prepare_connect", lambda: None)

    def fail_connect(*args):
        del args
        raise NativeProcessError("native connect failed")

    def fail_first_close(*, move_to_ready=False):
        nonlocal close_attempts
        close_attempts += 1
        if close_attempts == 1:
            raise RuntimeError("native cleanup failed")
        original_close(move_to_ready=move_to_ready)

    monkeypatch.setattr(backend, "_connect_prepared", fail_connect)
    monkeypatch.setattr(backend, "close", fail_first_close)

    try:
        with pytest.raises(NativeProcessError, match="native connect failed"):
            backend.connect(config, ArmRole.FOLLOWER, topics)
        assert close_attempts == 1
    finally:
        backend.close()
def test_ready_during_move_requires_matching_request_id_and_new_state():
    backend = make_consuming_backend()
    backend._consume_state(pack_state([0.4] * 6, sequence=1))
    backend._pending_ready_request_id = 42
    backend._ready = False

    backend._consume_status(pack_status(NativeStatus.READY))
    backend._consume_status(pack_status(NativeStatus.READY, (41,)))
    assert not backend._ready

    backend._consume_status(pack_status(NativeStatus.READY, (42,)))
    backend._consume_status(pack_status(NativeStatus.READY))
    assert not backend._ready
    assert backend._pending_ready_request_id == 42

    backend._consume_state(pack_state([0.0] * 6, sequence=2))
    assert backend._ready
    assert backend._pending_ready_request_id is None
    backend.close()


def test_concurrent_move_to_ready_calls_are_serialized():
    backend = make_consuming_backend()
    backend._consume_state(pack_state([0.4] * 6, sequence=1))
    backend._running = True
    first_send_started = threading.Event()
    release_first_send = threading.Event()
    second_call_started = threading.Event()
    second_send_started = threading.Event()
    request_ids: list[int] = []
    errors: list[BaseException] = []

    def complete_move(*args, request_id=None, **kwargs):
        del args, kwargs
        assert request_id is not None
        request_ids.append(request_id)
        if len(request_ids) == 1:
            first_send_started.set()
            assert release_first_send.wait(timeout=2.0)
        else:
            second_send_started.set()
        backend._consume_status(pack_status(NativeStatus.READY, (request_id,)))
        backend._consume_state(pack_state([0.0] * 6, sequence=request_id + 1))

    def second_move() -> None:
        second_call_started.set()
        capture_error(backend.move_to_ready, errors)

    backend._send_lifecycle = complete_move
    first = threading.Thread(target=lambda: capture_error(backend.move_to_ready, errors))
    second = threading.Thread(target=second_move)

    try:
        first.start()
        assert first_send_started.wait(timeout=1.0)
        second.start()
        assert second_call_started.wait(timeout=1.0)
        assert not second_send_started.wait(timeout=0.1)

        release_first_send.set()
        first.join(timeout=1.0)
        assert not first.is_alive()
        assert second_send_started.wait(timeout=1.0)
        second.join(timeout=1.0)
    finally:
        release_first_send.set()
        first.join(timeout=1.0)
        second.join(timeout=1.0)
        backend.close()

    assert not second.is_alive()
    assert not errors
    assert request_ids == [1, 2]


def test_move_to_ready_rejects_a_reconnected_backend_generation():
    backend = make_consuming_backend()
    condition = ObservedCondition()
    backend._condition = condition
    backend._reader = threading.Thread()
    backend._running = True
    backend._send_lifecycle = lambda *args, **kwargs: None
    errors: list[BaseException] = []
    move = threading.Thread(target=lambda: capture_error(backend.move_to_ready, errors))

    try:
        move.start()
        assert condition.wait_started.wait(timeout=1.0)
        with condition:
            backend._closed = True
            backend._running = False
            condition.notify_all()
            backend._pending_ready_request_id = None
            backend._pending_ready_state_generation = None
            backend._reader = threading.Thread()
            backend._reader_error = None
            backend._closed = False
            backend._running = True
            backend._ready = True
            condition.notify_all()
        move.join(timeout=1.0)
    finally:
        backend.close()
        move.join(timeout=1.0)

    assert not move.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], NativeProcessError)


def test_concurrent_close_waits_for_inflight_cleanup():
    class FakeProcess:
        returncode: int | None = None

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            del timeout
            self.returncode = 0
            return 0

        def terminate(self):
            self.returncode = 0

        def kill(self):
            self.returncode = -9

    backend = NativeArmBackend()
    backend._process = FakeProcess()
    first_close_blocked = threading.Event()
    release_first_close = threading.Event()
    second_close_started = threading.Event()
    second_close_returned = threading.Event()

    def block_lifecycle_send(*args, **kwargs):
        del args, kwargs
        first_close_blocked.set()
        assert release_first_close.wait(timeout=2.0)

    backend._send_lifecycle = block_lifecycle_send
    first = threading.Thread(target=backend.close)

    def close_again():
        second_close_started.set()
        backend.close()
        second_close_returned.set()

    second = threading.Thread(target=close_again)
    try:
        first.start()
        assert first_close_blocked.wait(timeout=1.0)
        second.start()
        assert second_close_started.wait(timeout=1.0)
        assert not second_close_returned.wait(timeout=0.1)
    finally:
        release_first_close.set()
        first.join(timeout=2.0)
        second.join(timeout=2.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert second_close_returned.is_set()


def test_close_waits_for_inflight_direct_command():
    class CoordinatedSocketCondition:
        def __init__(self, release_send: threading.Event) -> None:
            self._lock = threading.Lock()
            self._release_send = release_send

        def __enter__(self):
            if threading.current_thread().name == "close-during-command" and self._lock.locked():
                self._release_send.set()
            self._lock.acquire()
            return self

        def __exit__(self, exc_type, exc_value, traceback) -> None:
            self._lock.release()

        def notify_all(self) -> None:
            pass

    class BlockingPublisher:
        def __init__(self) -> None:
            self.send_started = threading.Event()
            self.release_send = threading.Event()
            self.sending = False
            self.closed_during_send = False

        def send(self, payload):
            del payload
            self.sending = True
            self.send_started.set()
            assert self.release_send.wait(timeout=2.0)
            self.sending = False

        def close(self):
            self.closed_during_send = self.sending
            self.release_send.set()

    backend = make_consuming_backend()
    publisher = BlockingPublisher()
    backend._direct_pub = publisher
    backend._running = True
    backend._condition = CoordinatedSocketCondition(publisher.release_send)
    command_errors: list[BaseException] = []
    close_errors: list[BaseException] = []

    def capture(callback, errors):
        try:
            callback()
        except BaseException as exc:  # noqa: BLE001 - capture worker failures
            errors.append(exc)

    command_thread = threading.Thread(
        target=lambda: capture(lambda: backend.command(PositionCommand([0.1] * 6)), command_errors)
    )
    close_thread = threading.Thread(
        target=lambda: capture(backend.close, close_errors), name="close-during-command"
    )

    try:
        command_thread.start()
        assert publisher.send_started.wait(timeout=1.0)
        close_thread.start()
        command_thread.join(timeout=2.0)
        close_thread.join(timeout=2.0)
    finally:
        publisher.release_send.set()
        command_thread.join(timeout=2.0)
        close_thread.join(timeout=2.0)

    assert not command_thread.is_alive()
    assert not close_thread.is_alive()
    assert not command_errors
    assert not close_errors
    assert not publisher.closed_during_send


def test_close_rejects_direct_commands_before_shutdown_finishes():
    class FakeProcess:
        returncode: int | None = None

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            del timeout
            self.returncode = 0
            return 0

        def terminate(self):
            self.returncode = 0

        def kill(self):
            self.returncode = -9

    class RecordingPublisher:
        def __init__(self) -> None:
            self.send_calls = 0

        def send(self, payload):
            del payload
            self.send_calls += 1

        def close(self):
            pass

    backend = make_consuming_backend()
    publisher = RecordingPublisher()
    close_started = threading.Event()
    release_close = threading.Event()
    close_errors: list[BaseException] = []
    backend._process = FakeProcess()
    backend._direct_pub = publisher
    backend._running = True

    def block_shutdown(*args, **kwargs):
        del args, kwargs
        close_started.set()
        assert release_close.wait(timeout=2.0)

    backend._send_lifecycle = block_shutdown
    close = threading.Thread(target=lambda: capture_error(backend.close, close_errors))

    try:
        close.start()
        assert close_started.wait(timeout=1.0)
        with pytest.raises(CommandRejectedError, match="not connected"):
            backend.command(PositionCommand([0.1] * 6))
    finally:
        release_close.set()
        close.join(timeout=2.0)

    assert not close.is_alive()
    assert not close_errors
    assert publisher.send_calls == 0


def test_close_wakes_a_blocking_state_reader():
    class ObservedCondition(threading.Condition):
        def __init__(self) -> None:
            super().__init__()
            self.wait_started = threading.Event()

        def wait(self, timeout=None):
            self.wait_started.set()
            return super().wait(timeout)

    backend = make_consuming_backend()
    condition = ObservedCondition()
    backend._condition = condition
    backend._running = True
    errors: list[BaseException] = []
    reader_returned = threading.Event()

    def read_state() -> None:
        _capture_error(lambda: backend.read_state(None), errors)
        reader_returned.set()

    reader = threading.Thread(target=read_state)

    try:
        reader.start()
        assert condition.wait_started.wait(timeout=1.0)
        backend.close()
        assert reader_returned.wait(timeout=0.5)
    finally:
        with condition:
            condition.notify_all()
        reader.join(timeout=1.0)

    assert not reader.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], NativeProcessError)


def test_close_waits_for_active_subscriber_receive_before_closing_it():
    class BlockingSubscriber:
        def __init__(self) -> None:
            self.receive_started = threading.Event()
            self.release_receive = threading.Event()
            self.closed = threading.Event()
            self.receiving = False
            self.closed_during_receive = False

        def recv_latest(self):
            self.receiving = True
            self.receive_started.set()
            assert self.release_receive.wait(timeout=3.0)
            self.receiving = False
            return None, 0

        def close(self):
            self.closed_during_receive = self.receiving
            self.closed.set()

    backend = make_consuming_backend()
    subscriber = BlockingSubscriber()
    backend._state_sub = subscriber
    backend._running = True
    reader = threading.Thread(target=backend._reader_loop)
    backend._reader = reader
    close_errors: list[BaseException] = []
    close = threading.Thread(target=lambda: capture_error(backend.close, close_errors))

    try:
        reader.start()
        assert subscriber.receive_started.wait(timeout=1.0)
        close.start()

        assert not subscriber.closed.wait(timeout=1.5)
    finally:
        subscriber.release_receive.set()
        reader.join(timeout=2.0)
        close.join(timeout=2.0)

    assert not reader.is_alive()
    assert not close.is_alive()
    assert not close_errors
    assert subscriber.closed.is_set()
    assert not subscriber.closed_during_receive


def test_stale_reader_failure_cannot_stop_a_reconnected_backend():
    class BlockingFailingSubscriber:
        def __init__(self) -> None:
            self.receive_started = threading.Event()
            self.release_receive = threading.Event()

        def recv_latest(self):
            self.receive_started.set()
            assert self.release_receive.wait(timeout=2.0)
            raise RuntimeError("retired reader failed")

    backend = make_consuming_backend()
    subscriber = BlockingFailingSubscriber()
    backend._state_sub = subscriber
    backend._running = True
    retired_reader = threading.Thread(target=backend._reader_loop)
    backend._reader = retired_reader

    try:
        retired_reader.start()
        assert subscriber.receive_started.wait(timeout=1.0)

        replacement_reader = threading.Thread()
        backend._reader = replacement_reader
        backend._reader_error = None
        backend._running = True
        subscriber.release_receive.set()
        retired_reader.join(timeout=1.0)

        assert not retired_reader.is_alive()
        assert backend._running
        assert backend._reader_error is None
    finally:
        subscriber.release_receive.set()
        backend._running = False
        retired_reader.join(timeout=1.0)
        backend._state_sub = None
        backend.close()


def test_stale_heartbeat_cannot_send_on_a_reconnected_backend():
    class BlockingStopEvent(threading.Event):
        def __init__(self) -> None:
            super().__init__()
            self.check_started = threading.Event()
            self.release_check = threading.Event()

        def is_set(self):
            self.check_started.set()
            assert self.release_check.wait(timeout=2.0)
            return super().is_set()

    class RecordingPublisher:
        def __init__(self, name: str, stop: threading.Event, sends: list[str]) -> None:
            self.name = name
            self.stop = stop
            self.sends = sends

        def send(self, payload):
            del payload
            self.sends.append(self.name)
            self.stop.set()

    backend = make_consuming_backend()
    stop = BlockingStopEvent()
    sends: list[str] = []
    backend._heartbeat_stop = stop
    backend._lifecycle_pub = RecordingPublisher("retired", stop, sends)
    backend._running = True
    retired_heartbeat = threading.Thread(target=backend._heartbeat_loop)
    backend._heartbeat = retired_heartbeat

    try:
        retired_heartbeat.start()
        assert stop.check_started.wait(timeout=1.0)

        stop.set()
        backend._running = False
        backend._heartbeat = threading.Thread()
        backend._lifecycle_pub = RecordingPublisher("replacement", stop, sends)
        stop.clear()
        backend._running = True
        stop.release_check.set()
        retired_heartbeat.join(timeout=1.0)

        assert not retired_heartbeat.is_alive()
        assert sends == []
    finally:
        stop.set()
        stop.release_check.set()
        backend._running = False
        retired_heartbeat.join(timeout=1.0)
        backend._lifecycle_pub = None
        backend.close()


def test_stale_log_reader_cannot_append_after_reconnect():
    class BlockingStdout:
        def __init__(self) -> None:
            self.read_started = threading.Event()
            self.release_read = threading.Event()
            self.finished = False

        def __iter__(self):
            return self

        def __next__(self):
            if self.finished:
                raise StopIteration
            self.read_started.set()
            assert self.release_read.wait(timeout=2.0)
            self.finished = True
            return "retired process output\n"

    class Process:
        def __init__(self, stdout) -> None:
            self.stdout = stdout

    backend = make_consuming_backend()
    stdout = BlockingStdout()
    backend._process = Process(stdout)
    retired_log_reader = threading.Thread(target=backend._drain_process_log)
    backend._log_reader = retired_log_reader

    try:
        retired_log_reader.start()
        assert stdout.read_started.wait(timeout=1.0)

        backend._process = None
        backend._log_reader = threading.Thread()
        backend._log_lines.clear()
        stdout.release_read.set()
        retired_log_reader.join(timeout=1.0)

        assert not retired_log_reader.is_alive()
        assert not backend._log_lines
    finally:
        stdout.release_read.set()
        retired_log_reader.join(timeout=1.0)
        backend._process = None
        backend.close()


def test_close_retires_owned_process_stdout():
    class RecordingStdout:
        def __init__(self) -> None:
            self.closed = False

        def close(self) -> None:
            self.closed = True

    class ExitedProcess:
        def __init__(self, stdout) -> None:
            self.stdout = stdout

        def poll(self):
            return 0

    backend = make_consuming_backend()
    stdout = RecordingStdout()
    backend._process = ExitedProcess(stdout)

    backend.close()

    assert stdout.closed


def test_close_retires_later_resources_after_process_terminate_error():
    class FailingProcess:
        def __init__(self) -> None:
            self.wait_calls = 0

        def poll(self):
            return None

        def terminate(self) -> None:
            raise RuntimeError("native termination failed")

        def wait(self, timeout):
            del timeout
            self.wait_calls += 1
            return 0

    class RecordingSocket:
        def __init__(self) -> None:
            self.closed = False

        def close(self) -> None:
            self.closed = True

    class RecordingContext:
        def __init__(self) -> None:
            self.terminated = False

        def term(self) -> None:
            self.terminated = True

    backend = make_consuming_backend()
    backend._context.term()
    process = FailingProcess()
    socket = RecordingSocket()
    context = RecordingContext()
    backend._process = process
    backend._state_sub = socket
    backend._context = context
    backend._running = True

    def fail_shutdown(*args, **kwargs):
        del args, kwargs
        raise RuntimeError("native shutdown failed")

    backend._send_lifecycle = fail_shutdown

    with pytest.raises(RuntimeError, match="native termination failed"):
        backend.close()

    assert process.wait_calls == 1
    assert not backend._running
    assert socket.closed
    assert context.terminated


def test_close_retires_later_resources_after_process_wait_error():
    class FailingProcess:
        def poll(self):
            return None

        def terminate(self) -> None:
            pass

        def wait(self, timeout):
            del timeout
            raise RuntimeError("native wait failed")

    class RecordingSocket:
        def __init__(self) -> None:
            self.closed = False

        def close(self) -> None:
            self.closed = True

    class RecordingContext:
        def __init__(self) -> None:
            self.terminated = False

        def term(self) -> None:
            self.terminated = True

    backend = make_consuming_backend()
    backend._context.term()
    socket = RecordingSocket()
    context = RecordingContext()
    backend._process = FailingProcess()
    backend._state_sub = socket
    backend._context = context
    backend._running = True

    def fail_shutdown(*args, **kwargs):
        del args, kwargs
        raise RuntimeError("native shutdown failed")

    backend._send_lifecycle = fail_shutdown

    with pytest.raises(RuntimeError, match="native wait failed"):
        backend.close()

    assert not backend._running
    assert socket.closed
    assert context.terminated


@pytest.mark.parametrize("failure", ["kill", "post-kill-wait"])
def test_close_retires_later_resources_after_forced_process_stop_error(failure):
    class FailingProcess:
        def __init__(self) -> None:
            self.wait_calls = 0

        def poll(self):
            return None

        def terminate(self) -> None:
            pass

        def wait(self, timeout):
            self.wait_calls += 1
            if self.wait_calls == 1:
                raise subprocess.TimeoutExpired("native process", timeout)
            if failure == "post-kill-wait":
                raise RuntimeError("native post-kill wait failed")
            return 0

        def kill(self) -> None:
            if failure == "kill":
                raise RuntimeError("native kill failed")

    class RecordingSocket:
        def __init__(self) -> None:
            self.closed = False

        def close(self) -> None:
            self.closed = True

    class RecordingContext:
        def __init__(self) -> None:
            self.terminated = False

        def term(self) -> None:
            self.terminated = True

    backend = make_consuming_backend()
    backend._context.term()
    process = FailingProcess()
    socket = RecordingSocket()
    context = RecordingContext()
    backend._process = process
    backend._state_sub = socket
    backend._context = context
    backend._running = True

    def fail_shutdown(*args, **kwargs):
        del args, kwargs
        raise RuntimeError("native shutdown failed")

    backend._send_lifecycle = fail_shutdown
    expected_error = "native kill failed" if failure == "kill" else "native post-kill wait failed"

    with pytest.raises(RuntimeError, match=expected_error):
        backend.close()

    assert process.wait_calls == 2
    assert not backend._running
    assert socket.closed
    assert context.terminated


def test_close_retires_later_resources_after_socket_close_error():
    class FailingSocket:
        def __init__(self) -> None:
            self.close_attempts = 0

        def close(self) -> None:
            self.close_attempts += 1
            raise RuntimeError("subscriber close failed")

    class RecordingSocket:
        def __init__(self) -> None:
            self.closed = False

        def close(self) -> None:
            self.closed = True

    class RecordingContext:
        def __init__(self) -> None:
            self.terminated = False

        def term(self) -> None:
            self.terminated = True

    backend = make_consuming_backend()
    backend._context.term()
    failing_socket = FailingSocket()
    later_socket = RecordingSocket()
    context = RecordingContext()
    backend._state_sub = failing_socket
    backend._status_sub = later_socket
    backend._context = context

    with pytest.raises(RuntimeError, match="subscriber close failed"):
        backend.close()

    assert failing_socket.close_attempts == 1
    assert later_socket.closed
    assert context.terminated


def test_close_retries_incomplete_context_termination():
    class TransientContext:
        def __init__(self) -> None:
            self.term_calls = 0

        def term(self) -> None:
            self.term_calls += 1
            if self.term_calls == 1:
                raise RuntimeError("context termination interrupted")

    backend = make_consuming_backend()
    backend._context.term()
    context = TransientContext()
    backend._context = context

    with pytest.raises(RuntimeError, match="termination interrupted"):
        backend.close()

    backend.close()
    backend.close()
    assert context.term_calls == 2


def test_commands_after_close_do_not_touch_closed_publishers():
    class Publisher:
        def __init__(self) -> None:
            self.closed = False
            self.send_calls = 0

        def send(self, payload):
            del payload
            self.send_calls += 1
            assert not self.closed

        def close(self):
            self.closed = True

    backend = make_consuming_backend()
    direct_pub = Publisher()
    lifecycle_pub = Publisher()
    backend._direct_pub = direct_pub
    backend._lifecycle_pub = lifecycle_pub
    backend._running = True

    backend.close()

    with pytest.raises(CommandRejectedError, match="not connected"):
        backend.command(PositionCommand([0.1] * 6))
    with pytest.raises(NativeProcessError, match="stopped"):
        backend.hold()
    assert direct_pub.send_calls == 0
    assert lifecycle_pub.send_calls == 0


def _capture_error(callback, errors: list[BaseException]) -> None:
    try:
        callback()
    except BaseException as exc:  # noqa: BLE001 - test helper captures thread failures
        errors.append(exc)
