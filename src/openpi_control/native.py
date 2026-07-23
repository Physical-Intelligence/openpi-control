"""Owned native process lifecycle and ZMQ transport."""

from __future__ import annotations

import logging
import os
import platform
import re
import shutil
import stat
import subprocess
import threading
import time
from collections import deque
from collections.abc import Iterable
from importlib.resources import files
from pathlib import Path
from typing import TextIO

import zmq

from . import log_paths
from .backend import ArmBackend
from .config import ArmConfig, InputLayout, ResolvedArmAssets, SocketCanConnection
from .exceptions import (
    CommandRejectedError,
    ConfigurationError,
    ConnectionUnavailableError,
    HardwareFaultError,
    NativeProcessError,
    ProtocolError,
    StateTimeoutError,
)
from .protocol import (
    CAP_DIRECT,
    CAP_FORCE_FEEDBACK,
    CAP_GRAVITY_COMP,
    CAP_LIVE_INPUT,
    CAP_MOVE_TO_READY,
    JOINT_STRUCT,
    PROTOCOL_VERSION,
    ArmTopics,
    NativeCommand,
    NativeStatus,
    decode_inputs,
    decode_status,
    encode_command,
    port_candidates,
)
from .types import (
    ArmCapabilities,
    ArmMode,
    ArmRole,
    ArmState,
    EffectorState,
    InputState,
    JointState,
    PositionCommand,
)
from .urdf_inertial import prepare_merged_urdf

_ANSI_ESCAPE = re.compile(r"\x1b(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")

# Status messages (acks, READY, MODE) must all be processed in order, so the
# reader drains the whole queue every poll; the cap only guards against a
# runaway publisher starving the other subscribers.
MAX_STATUS_MESSAGES_PER_POLL = 256
# Discarding a handful of state/input samples per poll is normal (the native
# node publishes faster than the reader polls). A backlog beyond this many
# messages means the consumer stalled and the policy would have been fed stale
# state, so it is worth a warning.
STREAM_BACKLOG_WARN_MESSAGES = 50


def _hardware_fault_message(log_line: str) -> str | None:
    cleaned = _ANSI_ESCAPE.sub("", log_line).strip()
    marker_index = cleaned.find("HARDWARE FAULT:")
    return cleaned[marker_index:] if marker_index >= 0 else None


def _native_process_failure(
    return_code: int | None, log_lines: Iterable[str]
) -> NativeProcessError:
    cleaned = [cleaned for line in log_lines if (cleaned := _ANSI_ESCAPE.sub("", line).strip())]
    for line in reversed(cleaned):
        if (message := _hardware_fault_message(line)) is not None:
            return HardwareFaultError(message)

    error_lines = [line.removeprefix("[ERROR] ").strip() for line in cleaned if "[ERROR]" in line]
    if len(error_lines) > 3:
        summary_lines = [error_lines[0], *error_lines[-2:]]
    else:
        summary_lines = error_lines or cleaned[-3:]
    summary = " | ".join(summary_lines)
    prefix = f"pi_control_node exited with {return_code}"
    return NativeProcessError(f"{prefix}: {summary}" if summary else prefix)


def native_executable() -> Path:
    override = os.environ.get("OPENPI_CONTROL_NODE")
    if override:
        return Path(override).expanduser().resolve()
    packaged = Path(str(files("openpi_control").joinpath("bin", "pi_control_node")))
    if packaged.is_file():
        return packaged
    found = shutil.which("pi_control_node")
    return Path(found) if found else packaged


def validate_connection(connection: SocketCanConnection) -> None:
    path = Path("/sys/class/net") / connection.interface
    if not path.exists():
        raise ConnectionUnavailableError(
            f"SocketCAN interface {connection.interface!r} does not exist; "
            "configure it outside openpi-control"
        )
    operstate = path / "operstate"
    if operstate.is_file() and operstate.read_text().strip() not in {"up", "unknown"}:
        raise ConnectionUnavailableError(f"SocketCAN interface {connection.interface!r} is not up")


class _Publisher:
    def __init__(self, context: zmq.Context, topic: str) -> None:
        self.topic = topic.encode()
        self.socket = context.socket(zmq.PUB)
        try:
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.SNDHWM, 2)
            last_error: Exception | None = None
            for port in port_candidates(topic, 16):
                try:
                    self.socket.bind(f"tcp://127.0.0.1:{port}")
                    return
                except zmq.ZMQError as exc:
                    last_error = exc
            raise NativeProcessError(f"unable to bind publisher for {topic}: {last_error}")
        except BaseException:
            try:
                self.socket.close(0)
            except BaseException:
                pass
            raise

    def send(self, payload: bytes) -> None:
        self.socket.send_multipart([self.topic, payload])

    def close(self) -> None:
        self.socket.close(0)


class _Subscriber:
    def __init__(self, context: zmq.Context, topic: str) -> None:
        self.topic = topic.encode()
        self.socket = context.socket(zmq.SUB)
        try:
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.RCVHWM, 2)
            self.socket.setsockopt(zmq.SUBSCRIBE, self.topic)
            # Probe the publisher's full bind-fallback window (the native node
            # tries 16 candidate ports); connecting to unused ports is harmless.
            for port in port_candidates(topic, 16):
                self.socket.connect(f"tcp://127.0.0.1:{port}")
        except BaseException:
            try:
                self.socket.close(0)
            except BaseException:
                pass
            raise

    def recv(self) -> bytes | None:
        try:
            topic, payload = self.socket.recv_multipart(zmq.NOBLOCK)
        except zmq.Again:
            return None
        if topic != self.topic:
            return None
        return payload

    def recv_latest(self) -> tuple[bytes | None, int]:
        """Drain every queued message and return (newest payload, discarded older count).

        Last-value-wins streams (state, inputs) must always be consumed to the
        newest sample: reading one message per poll replays any backlog one
        stale sample per cycle, so a transient consumer stall becomes a
        permanent multi-second state lag (TCP socket buffers hold well over
        10 s of 100 Hz traffic despite the HWM of 2). The native node guards
        its own SUB sockets the same way (drain_sub_to_latest in
        pi_topic_zmq.cpp).
        """
        latest: bytes | None = None
        discarded = 0
        while True:
            try:
                topic, payload = self.socket.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                return latest, discarded
            if topic != self.topic:
                continue
            if latest is not None:
                discarded += 1
            latest = payload

    def close(self) -> None:
        self.socket.close(0)


class NativeArmBackend(ArmBackend):
    """Local process owner for `pi_control_node`."""

    def __init__(self) -> None:
        self._context = zmq.Context()
        self._process: subprocess.Popen[str] | None = None
        self._parent_liveness_write_fd: int | None = None
        self._config: ArmConfig | None = None
        self._joint_names: tuple[str, ...] = ()
        self._role: ArmRole | None = None
        self._topics: ArmTopics | None = None
        self._state_sub: _Subscriber | None = None
        self._status_sub: _Subscriber | None = None
        self._inputs_sub: _Subscriber | None = None
        self._direct_pub: _Publisher | None = None
        self._lifecycle_pub: _Publisher | None = None
        self._state: ArmState | None = None
        self._state_generation = 0
        self._inputs: InputState | None = None
        self._inputs_sequence = 0
        self._input_layout = InputLayout()
        self._capabilities: ArmCapabilities | None = None
        self._condition = threading.Condition()
        self._connect_lock = threading.RLock()
        self._close_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._move_to_ready_lock = threading.Lock()
        self._mode_lock = threading.Lock()
        self._acks: dict[int, int] = {}
        self._pending_ack_request_id: int | None = None
        self._request_id = 0
        self._pending_ready_request_id: int | None = None
        self._pending_ready_state_generation: int | None = None
        self._running = False
        self._reader: threading.Thread | None = None
        self._log_reader: threading.Thread | None = None
        self._heartbeat: threading.Thread | None = None
        self._heartbeat_stop = threading.Event()
        self._log_lines: deque[str] = deque(maxlen=200)
        self._log_tee: TextIO | None = None
        self._reader_error: Exception | None = None
        self._hardware_fault_message: str | None = None
        self._paired_follower_state_topic = ""
        self._ready = False
        self._closed = False
        self._resources_closed = False

    def configure_pair(self, *, follower_state_topic: str) -> None:
        self._paired_follower_state_topic = follower_state_topic

    def _is_connected(self) -> bool:
        with self._condition:
            return self._running

    def _prepare_connect(self) -> None:
        with self._condition:
            if self._running or (self._process is not None and self._process.poll() is None):
                raise NativeProcessError("native backend is already connected")

        self.close()
        with self._condition:
            self._process = None
            self._parent_liveness_write_fd = None
            self._state_sub = None
            self._status_sub = None
            self._inputs_sub = None
            self._direct_pub = None
            self._lifecycle_pub = None
            self._state = None
            self._inputs = None
            self._inputs_sequence = 0
            self._capabilities = None
            self._acks.clear()
            self._pending_ack_request_id = None
            self._request_id = 0
            self._pending_ready_request_id = None
            self._reader = None
            self._log_reader = None
            self._heartbeat = None
            self._heartbeat_stop = threading.Event()
            self._log_lines.clear()
            self._log_tee = None
            self._reader_error = None
            self._hardware_fault_message = None
            self._ready = False
            self._closed = False
            self._resources_closed = False

        if self._context.closed:
            self._context = zmq.Context()

    def connect(self, config: ArmConfig, role: ArmRole, topics: ArmTopics) -> ArmCapabilities:
        with self._connect_lock:
            if platform.system() != "Linux":
                raise ConnectionUnavailableError(
                    "physical native control is supported on Linux only"
                )
            validate_connection(config.connection)
            if role is ArmRole.FOLLOWER and config.is_read_only():
                raise ConfigurationError(
                    f"model {config.model!r} is read-only (leader-only, no actuation) "
                    "and cannot be used as a follower"
                )
            executable = native_executable()
            if not executable.is_file() or not executable.stat().st_mode & stat.S_IXUSR:
                raise NativeProcessError(
                    f"native executable not found or not executable: {executable}; "
                    "reinstall openpi-control"
                )
            assets = config.resolve_assets()
            self._prepare_connect()
            try:
                return self._connect_prepared(config, role, topics, executable, assets)
            except BaseException:
                try:
                    self.close()
                except BaseException:
                    # Preserve the bring-up failure even if teardown also fails.
                    pass
                raise

    def _connect_prepared(
        self,
        config: ArmConfig,
        role: ArmRole,
        topics: ArmTopics,
        executable: Path,
        assets: ResolvedArmAssets,
    ) -> ArmCapabilities:
        self._config, self._role, self._topics = config, role, topics
        # Resolved once: joint_names() re-resolves the packaged model assets
        # (several stat() calls plus a JSON read). The reader thread needs the
        # names for every 100-200 Hz state message WHILE HOLDING the condition
        # lock, so per-message resolution turns the lock into a convoy that can
        # starve the other threads (observed: the 1 Hz heartbeat sender never
        # got the lock again, so the node's dead-client watchdog never armed).
        self._joint_names = ()
        self._joint_names = self._resolved_joint_names()
        self._input_layout = config.input_layout() if role is ArmRole.LEADER else InputLayout()
        # The gravity model must see the effector inertia exactly once: hand the
        # node a merged URDF whose end link inertial is replaced with the effector
        # mass model (zero mass when no effector). A caller-supplied URDF is
        # trusted as-is and skips merging.
        if config.urdf is None:
            urdf_path = prepare_merged_urdf(
                assets, model=config.model, effector_model=config.effector_model
            )
        else:
            urdf_path = assets.urdf
        self._state_sub = _Subscriber(self._context, topics.state)
        self._status_sub = _Subscriber(self._context, topics.status)
        self._lifecycle_pub = _Publisher(self._context, topics.lifecycle_command)
        if self._input_layout.has_inputs:
            self._inputs_sub = _Subscriber(self._context, topics.inputs)
        if role is ArmRole.FOLLOWER:
            self._direct_pub = _Publisher(self._context, topics.direct_command)
        connection_args = ["--control_port", config.connection.interface]
        args = [
            str(executable),
            "--role",
            role.value,
            "--device_type",
            "arms",
            "--device_model",
            config.model,
            "--device_id",
            "01",
            "--logical_name",
            config.name,
            "--control_frequency",
            str(config.control_frequency_hz),
            "--info_level",
            os.environ.get("OPENPI_CONTROL_INFO_LEVEL", "0"),
            "--topic_type",
            "ZMQ",
            # Model configs (format 1.1.1, shared with robot-test) declare "KDL",
            # which this node does not ship; force the Pinocchio implementation.
            # Effector configs declaring "Algo" keep priority over this flag.
            "--algo_type",
            "Pinocchio",
            "--topic_state",
            topics.state,
            "--topic_live_command",
            topics.live_command,
            "--topic_direct_command",
            topics.direct_command,
            "--topic_lifecycle",
            topics.lifecycle_command,
            "--topic_status",
            topics.status,
            "--arm_model_config",
            str(assets.model_config),
            "--arm_instance_config",
            str(assets.instance_config),
            "--urdf_path",
            str(urdf_path),
            "--force_feedback",
            "-1",
            # Connect is passive: the arm holds its current pose instead of
            # driving to home during bring-up. Homing stays an explicit
            # move_to_ready() action (which still executes with this flag).
            "--dont_go_to_home_pos",
            *connection_args,
        ]
        if self._paired_follower_state_topic:
            args.extend(["--paired_follower_state_topic", self._paired_follower_state_topic])
        if role is ArmRole.FOLLOWER and config.follower_gravity_compensation:
            # Arm-device-scoped MonoPi-style synchronized slew tracking plus
            # gravity/damping feedforward. The attached effector keeps the
            # planner from its own model config.
            args.extend(["--arm_planning_type", "slew_pos_gravity"])
        if role is ArmRole.LEADER and config.leader_gravity_compensation:
            args.append("--leader_gravity_compensation")
        if config.safety_torque_mode:
            args.append("--safety_torque_mode")
        if self._input_layout.has_inputs:
            args.extend(["--topic_joystick", topics.inputs])
        if assets.effector_model_config and assets.effector_instance_config:
            args.extend(
                [
                    "--effector_model",
                    config.effector_model or "",
                    "--effector_model_config",
                    str(assets.effector_model_config),
                    "--effector_instance_config",
                    str(assets.effector_instance_config),
                ]
            )
        # Tee the node's stdout to a persistent per-arm log file so post-mortem
        # debugging survives process exit (the in-memory ring buffer holds only
        # the last 200 lines). Earlier runs are preserved via timestamp rename.
        tee_path = log_paths.log_dir() / (
            f"pi_control_node__{role.value}__{config.name}__{config.model}.log"
        )
        log_paths.rotate_existing(tee_path)
        self._log_tee = tee_path.open("w", encoding="utf-8")
        parent_liveness_read_fd, parent_liveness_write_fd = os.pipe()
        self._parent_liveness_write_fd = parent_liveness_write_fd
        args.extend(["--parent_liveness_fd", str(parent_liveness_read_fd)])
        try:
            self._process = subprocess.Popen(
                args,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                pass_fds=(parent_liveness_read_fd,),
                # Terminal-generated signals target the foreground process
                # group. Give the supervised node its own session so Python
                # alone handles Ctrl+C and chooses the shutdown behavior.
                start_new_session=True,
            )
        finally:
            os.close(parent_liveness_read_fd)
        self._log_reader = threading.Thread(target=self._drain_process_log, daemon=True)
        self._log_reader.start()
        self._running = True
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()
        self._heartbeat = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat.start()
        deadline = time.monotonic() + config.connect_timeout_s
        with self._condition:
            while self._capabilities is None or not self._ready or self._state is None:
                self._raise_if_hardware_fault()
                if self._process.poll() is not None:
                    raise self._stopped_process_error("native process stopped during connect")
                if self._reader_error is not None:
                    raise NativeProcessError(f"native protocol reader failed: {self._reader_error}")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    missing = ", ".join(
                        name
                        for name, present in (
                            ("handshake", self._capabilities is not None),
                            ("ready", self._ready),
                            ("state", self._state is not None),
                        )
                        if not present
                    )
                    raise ProtocolError(
                        f"timed out after {config.connect_timeout_s:g}s waiting for native "
                        f"{missing}; node log tail: {''.join(self._log_lines)[-2000:] or '<empty>'}"
                    )
                self._condition.wait(min(remaining, 0.1))
            return self._capabilities

    def _drain_process_log(self) -> None:
        log_reader = threading.current_thread()
        with self._condition:
            if self._log_reader is not log_reader:
                return
            process = self._process
            if process is None or process.stdout is None:
                return
            stdout = process.stdout
        for line in stdout:
            message = _hardware_fault_message(line)
            with self._condition:
                if self._log_reader is not log_reader:
                    return
                self._log_lines.append(line)
                tee = self._log_tee
                if message is not None:
                    self._hardware_fault_message = message
                    self._condition.notify_all()
            # Tee I/O stays OUTSIDE the condition lock: the node floods stdout
            # during startup, and a flushed disk write per line while holding
            # the lock convoys every other lock user (observed: the 1 Hz
            # heartbeat sender starved through the whole startup window, so
            # the node never armed its dead-client watchdog). The tee handle
            # is only ever written by this thread.
            if tee is not None and not tee.closed:
                try:
                    tee.write(line)
                    tee.flush()
                except OSError:
                    # Disk-full or revoked mount must not take down the
                    # control path; the ring buffer still captures the tail.
                    with self._condition:
                        self._log_tee = None

    def _raise_if_hardware_fault(self) -> None:
        if self._hardware_fault_message is not None:
            raise HardwareFaultError(self._hardware_fault_message)

    def _stopped_process_error(self, fallback: str) -> NativeProcessError:
        if self._log_reader is not None and self._log_reader.is_alive():
            self._log_reader.join(timeout=0.1)
        if self._process is not None:
            return_code = self._process.poll()
            if return_code is not None:
                return _native_process_failure(return_code, self._log_lines)
        return NativeProcessError(fallback)

    def _reader_loop(self) -> None:
        reader = threading.current_thread()
        with self._condition:
            if self._reader is not reader:
                return
            state_sub = self._state_sub
            status_sub = self._status_sub
            inputs_sub = self._inputs_sub
            process = self._process

        while True:
            try:
                with self._condition:
                    if not self._running or self._reader is not reader:
                        return
                # Status messages (acks, READY, MODE) all carry meaning, so the
                # entire queue is processed in order. State and inputs are
                # last-value-wins streams and are drained to the newest sample
                # every poll -- see _Subscriber.recv_latest for why.
                if status_sub:
                    for _ in range(MAX_STATUS_MESSAGES_PER_POLL):
                        payload = status_sub.recv()
                        if payload is None:
                            break
                        with self._condition:
                            if not self._running or self._reader is not reader:
                                return
                            self._consume_status(payload)
                for subscriber, consume, stream in (
                    (state_sub, self._consume_state, "state"),
                    (inputs_sub, self._consume_inputs, "inputs"),
                ):
                    if not subscriber:
                        continue
                    payload, discarded = subscriber.recv_latest()
                    if discarded >= STREAM_BACKLOG_WARN_MESSAGES:
                        self._warn_stream_backlog(stream, discarded)
                    if payload is not None:
                        with self._condition:
                            if not self._running or self._reader is not reader:
                                return
                            consume(payload)
                if process and process.poll() is not None:
                    with self._condition:
                        if self._reader is not reader:
                            return
                        self._running = False
                        self._condition.notify_all()
                    return
                time.sleep(0.001)
            except Exception as exc:
                with self._condition:
                    if self._reader is not reader:
                        return
                    self._reader_error = exc
                    self._running = False
                    self._condition.notify_all()
                return

    def _warn_stream_backlog(self, stream: str, discarded: int) -> None:
        name = self._config.name if self._config else "unknown"
        hz = self._config.control_frequency_hz if self._config else 0
        stalled_s = discarded / hz if hz > 0 else 0.0
        logging.getLogger(__name__).warning(
            "%s: drained %d stale %s message(s) (~%.1f s of backlog); "
            "the consumer stalled and the discarded samples were never observed",
            name,
            discarded,
            stream,
            stalled_s,
        )

    def _resolved_joint_names(self) -> tuple[str, ...]:
        """Joint names, resolved from the model assets once and cached.

        joint_names() re-resolves the packaged model assets on every call
        (several stat() calls plus a JSON read); the consumers below need the
        names for every 100-200 Hz state message while holding the condition
        lock, so per-message resolution turns the lock into a convoy.
        """
        if not self._joint_names and self._config is not None:
            self._joint_names = self._config.joint_names()
        return self._joint_names

    def _consume_state(self, payload: bytes) -> None:
        if len(payload) != JOINT_STRUCT.size:
            raise ProtocolError(f"invalid joint payload size {len(payload)}")
        assert self._config is not None and self._role is not None
        values = JOINT_STRUCT.unpack(payload)
        joint_count = int(values[61])
        names = self._resolved_joint_names()
        arm_dof = len(names)
        if joint_count < arm_dof:
            raise ProtocolError(
                f"native state has {joint_count} joints; expected at least {arm_dof}"
            )
        positions = values[0:10][:arm_dof]
        velocities = values[10:20][:arm_dof]
        efforts = values[20:30][:arm_dof]
        temperatures = values[30:40][:arm_dof]
        currents = values[40:50][:arm_dof]
        frame_ages = values[50:60][:arm_dof]
        effector = (
            EffectorState(
                position=float(values[arm_dof]),
                velocity_s=float(values[10 + arm_dof]),
                effort_nm=float(values[20 + arm_dof]),
                temperature_c=float(values[30 + arm_dof]),
                current_a=float(values[40 + arm_dof]),
                frame_age_ms=float(values[50 + arm_dof]),
            )
            if joint_count > arm_dof
            else None
        )
        with self._condition:
            previous_mode = (
                self._state.mode
                if self._state
                else (
                    ArmMode.GRAVITY_COMPENSATION if self._role is ArmRole.LEADER else ArmMode.HOLD
                )
            )
            self._state = ArmState(
                name=self._config.name,
                role=self._role,
                joints=JointState(
                    names=names,
                    position_rad=positions,
                    velocity_rad_s=velocities,
                    effort_nm=efforts,
                    temperature_c=temperatures,
                    current_a=currents,
                    frame_age_ms=frame_ages,
                ),
                effector=effector,
                monotonic_timestamp=time.monotonic(),
                wall_timestamp=time.time(),
                sequence=int(values[60]),
                mode=previous_mode,
            )
            self._state_generation += 1
            if (
                self._pending_ready_request_id is not None
                and self._pending_ready_state_generation is not None
                and self._state_generation > self._pending_ready_state_generation
            ):
                self._pending_ready_request_id = None
                self._pending_ready_state_generation = None
                self._ready = True
            self._condition.notify_all()

    def _consume_inputs(self, payload: bytes) -> None:
        axes, buttons = decode_inputs(payload)
        layout = self._input_layout
        if len(buttons) < len(layout.button_names) or len(axes) < len(layout.axis_names):
            raise ProtocolError(
                f"native inputs have {len(buttons)} buttons and {len(axes)} axes; "
                f"expected at least {len(layout.button_names)} and {len(layout.axis_names)}"
            )
        with self._condition:
            self._inputs_sequence += 1
            self._inputs = InputState(
                button_names=layout.button_names,
                buttons=buttons[: len(layout.button_names)],
                axis_names=layout.axis_names,
                axes=axes[: len(layout.axis_names)],
                monotonic_timestamp=time.monotonic(),
                wall_timestamp=time.time(),
                sequence=self._inputs_sequence,
            )
            self._condition.notify_all()

    def _consume_status(self, payload: bytes) -> None:
        assert self._config is not None and self._role is not None
        status, floats, ints = decode_status(payload)
        with self._condition:
            if status is NativeStatus.HANDSHAKE:
                if len(ints) < 3 or tuple(ints[:2]) != PROTOCOL_VERSION:
                    raise ProtocolError(
                        f"incompatible native protocol {tuple(ints[:2])}; "
                        f"expected {PROTOCOL_VERSION}"
                    )
                flags = ints[2]
                self._capabilities = ArmCapabilities(
                    protocol_version=PROTOCOL_VERSION,
                    model=self._config.model,
                    joint_names=self._resolved_joint_names(),
                    has_effector=self._config.effector_model is not None,
                    supports_direct_commands=bool(flags & CAP_DIRECT),
                    supports_live_input=bool(flags & CAP_LIVE_INPUT),
                    supports_gravity_compensation=bool(flags & CAP_GRAVITY_COMP),
                    supports_force_feedback=bool(flags & CAP_FORCE_FEEDBACK),
                    supports_move_to_ready=bool(flags & CAP_MOVE_TO_READY),
                    button_names=self._input_layout.button_names,
                    axis_names=self._input_layout.axis_names,
                )
            elif (
                status is NativeStatus.COMMAND_ACK
                and len(ints) >= 2
                and ints[0] == self._pending_ack_request_id
            ):
                self._acks[ints[0]] = ints[1]
            elif status is NativeStatus.READY:
                if self._pending_ready_request_id is None:
                    self._ready = True
                elif (
                    self._pending_ready_state_generation is None
                    and ints
                    and ints[0] == self._pending_ready_request_id
                ):
                    # State and status use independent ZMQ topics, so consume a
                    # state sample after the correlated completion before returning.
                    self._pending_ready_state_generation = self._state_generation
            elif status is NativeStatus.MODE and self._state and ints:
                mode_values = list(ArmMode)
                if 0 <= ints[0] < len(mode_values):
                    self._state = ArmState(
                        name=self._state.name,
                        role=self._state.role,
                        joints=self._state.joints,
                        effector=self._state.effector,
                        monotonic_timestamp=self._state.monotonic_timestamp,
                        wall_timestamp=self._state.wall_timestamp,
                        sequence=self._state.sequence,
                        mode=mode_values[ints[0]],
                    )
            self._condition.notify_all()

    def _heartbeat_loop(self) -> None:
        """Fire-and-forget client-liveness heartbeats at 1 Hz.

        The node arms its dead-client watchdog after the first heartbeat and
        drops to a safe idle (leader: gravity float; follower: pause + hold)
        after 5 s of silence -- so an abrupt client death (kill -9, host crash)
        no longer leaves the pair teleoperating unsupervised. Sends share the
        lifecycle lock with _send_lifecycle: ZMQ sockets are not thread-safe.
        """
        payload = encode_command(NativeCommand.HEARTBEAT)
        heartbeat = threading.current_thread()
        with self._condition:
            if self._heartbeat is not heartbeat:
                return
            stop = self._heartbeat_stop
            pub = self._lifecycle_pub

        while not stop.is_set():
            # The lifecycle lock (not the backend condition) serializes the
            # socket against _send_lifecycle: it is the only mutual exclusion
            # the ZMQ socket needs, and it is nearly uncontended. Waiting on
            # the busy backend condition here let a loaded host starve the
            # 1 Hz cadence for tens of seconds, so the node could go a whole
            # session without seeing a heartbeat and never arm its dead-client
            # watchdog. The liveness flags are read without the condition:
            # attribute reads are atomic, and the worst case -- one extra
            # heartbeat racing teardown -- ends in the send raising on the
            # closed socket, which exits the loop.
            with self._lifecycle_lock:
                if self._heartbeat is not heartbeat or pub is None or not self._running:
                    return
                try:
                    pub.send(payload)
                    self._debug_heartbeats_sent = getattr(self, "_debug_heartbeats_sent", 0) + 1
                except Exception:  # noqa: BLE001 - socket closing under us is a normal exit
                    return
            if stop.wait(1.0):
                return

    def _send_lifecycle(
        self,
        command: NativeCommand,
        floats: tuple[float, ...] = (),
        timeout_s: float = 3.0,
        *,
        request_id: int | None = None,
    ) -> None:
        with self._lifecycle_lock:
            with self._condition:
                if self._lifecycle_pub is None:
                    raise CommandRejectedError("arm is not connected")
                if request_id is None:
                    self._request_id += 1
                    request_id = self._request_id
                self._pending_ack_request_id = request_id
            try:
                payload = encode_command(command, floats=floats, ints=(request_id,))
                deadline = time.monotonic() + timeout_s
                with self._condition:
                    while request_id not in self._acks:
                        self._raise_if_hardware_fault()
                        if not self._running:
                            raise self._stopped_process_error(
                                "native process stopped before command acknowledgement"
                            )
                        self._lifecycle_pub.send(payload)
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise ProtocolError(
                                f"timed out waiting for {command.name} acknowledgement"
                            )
                        self._condition.wait(min(remaining, 0.2))
                    result = self._acks.pop(request_id)
            finally:
                with self._condition:
                    if self._pending_ack_request_id == request_id:
                        self._pending_ack_request_id = None
                    self._acks.pop(request_id, None)
            if result != 0:
                raise CommandRejectedError(
                    f"native command {command.name} rejected with code {result}"
                )

    def read_state(self, timeout_s: float | None = None) -> ArmState:
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        with self._condition:
            previous_generation = self._state_generation
            while self._state is None or self._state_generation == previous_generation:
                self._raise_if_hardware_fault()
                if not self._running:
                    if self._reader_error is not None:
                        raise NativeProcessError(
                            f"native protocol reader failed: {self._reader_error}"
                        )
                    raise self._stopped_process_error("native process is not running")
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    raise StateTimeoutError("timed out waiting for native arm state")
                self._condition.wait(remaining)
            return self._state

    def latest_state(self) -> ArmState | None:
        with self._condition:
            return self._state

    def read_inputs(self, timeout_s: float | None = None) -> InputState:
        if not self._input_layout.has_inputs:
            raise ConfigurationError("arm has no operator inputs")
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        with self._condition:
            reader = self._reader
            self._raise_if_hardware_fault()
            if self._closed or not self._running:
                if self._reader_error is not None:
                    raise NativeProcessError(
                        f"native protocol reader failed: {self._reader_error}"
                    )
                raise NativeProcessError("native process is not running")
            while self._inputs is None:
                self._raise_if_hardware_fault()
                if self._reader is not reader:
                    raise NativeProcessError("native connection changed while waiting for inputs")
                if self._closed or not self._running:
                    if self._reader_error is not None:
                        raise NativeProcessError(
                            f"native protocol reader failed: {self._reader_error}"
                        )
                    raise self._stopped_process_error("native process is not running")
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    raise StateTimeoutError("timed out waiting for native input state")
                self._condition.wait(remaining)
            if self._reader is not reader:
                raise NativeProcessError("native connection changed while waiting for inputs")
            return self._inputs

    def latest_inputs(self) -> InputState | None:
        with self._condition:
            return self._inputs

    def command(self, command: PositionCommand, *, live: bool = False) -> None:
        if live:
            raise CommandRejectedError("native live input is published by the leader process")
        with self._condition:
            self._raise_if_hardware_fault()
            if self._role is not ArmRole.FOLLOWER or self._direct_pub is None:
                raise CommandRejectedError("only followers accept direct position commands")
            if self._closed or not self._running:
                raise CommandRejectedError("arm is not connected")
            self._direct_pub.send(self._encode_joint_command(command))

    def _encode_joint_command(self, command: PositionCommand) -> bytes:
        positions = list(map(float, command.position_rad))
        effector = command.effector
        if self._config is not None and self._config.effector_model is not None:
            if effector is None:
                if self._state is None or self._state.effector is None:
                    raise CommandRejectedError(
                        "cannot preserve effector position before receiving native state"
                    )
                effector = self._state.effector.position
            positions.append(float(effector))
        elif effector is not None:
            positions.append(float(effector))
        total = len(positions)
        if total > 10:
            raise CommandRejectedError("native protocol supports at most ten total joints")
        joint_count = len(positions)
        arrays = (
            positions
            + [0.0] * (10 - joint_count)
            + [0.0] * 10
            + [0.0] * 10
            + [0.0] * 10
            + [0.0] * 10
            # joint_age_ms slots: meaningless for a target command (-1 = unknown).
            + [-1.0] * 10
        )
        return JOINT_STRUCT.pack(
            *arrays, int(time.monotonic_ns() & 0x7FFFFFFF), joint_count, 1, 0.0
        )

    def hold(self) -> None:
        self._send_lifecycle(NativeCommand.HOLD)

    def pause_live_input(self, paused: bool) -> None:
        self._send_lifecycle(
            NativeCommand.PAUSE_LIVE_INPUT if paused else NativeCommand.RESUME_LIVE_INPUT
        )

    def set_mode(self, mode: ArmMode) -> None:
        with self._mode_lock:
            with self._condition:
                reader = self._reader
            if mode is ArmMode.GRAVITY_COMPENSATION:
                self._send_lifecycle(NativeCommand.ENTER_GRAVITY_COMPENSATION)
            elif mode is ArmMode.BILATERAL:
                self._send_lifecycle(NativeCommand.ENABLE_FORCE_FEEDBACK)
            elif mode is ArmMode.HOLD:
                self.hold()
            else:
                raise CommandRejectedError(f"native runtime cannot directly enter {mode}")
            self._replace_mode(mode, expected_reader=reader)

    def set_force_feedback_gain(self, gain: float) -> None:
        self._send_lifecycle(NativeCommand.SET_FORCE_FEEDBACK_GAIN, floats=(gain,))

    def move_to_ready(self) -> None:
        with self._move_to_ready_lock:
            with self._condition:
                reader = self._reader
                self._request_id += 1
                request_id = self._request_id
                self._pending_ready_request_id = request_id
                self._pending_ready_state_generation = None
                self._ready = False
            try:
                self._send_lifecycle(
                    NativeCommand.MOVE_TO_READY, timeout_s=60.0, request_id=request_id
                )
                deadline = time.monotonic() + 60.0
                with self._condition:
                    while not self._ready:
                        self._raise_if_hardware_fault()
                        if self._reader is not reader:
                            raise NativeProcessError(
                                "native connection changed during move-to-ready"
                            )
                        if not self._running:
                            raise self._stopped_process_error(
                                "native process stopped during move-to-ready"
                            )
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise ProtocolError(
                                "timed out waiting for native move-to-ready completion"
                            )
                        self._condition.wait(min(remaining, 0.2))
                    if self._reader is not reader:
                        raise NativeProcessError("native connection changed during move-to-ready")
            finally:
                with self._condition:
                    if self._pending_ready_request_id == request_id:
                        self._pending_ready_request_id = None
                        self._pending_ready_state_generation = None

    def _replace_mode(
        self, mode: ArmMode, *, expected_reader: threading.Thread | None = None
    ) -> None:
        with self._condition:
            if expected_reader is not None and self._reader is not expected_reader:
                raise NativeProcessError("native connection changed during mode transition")
            if self._state is None:
                return
            self._state = ArmState(
                name=self._state.name,
                role=self._state.role,
                joints=self._state.joints,
                effector=self._state.effector,
                monotonic_timestamp=self._state.monotonic_timestamp,
                wall_timestamp=self._state.wall_timestamp,
                sequence=self._state.sequence,
                mode=mode,
            )
            self._condition.notify_all()

    def close(self, *, move_to_ready: bool = False) -> None:
        with self._connect_lock:
            with self._close_lock:
                self._close_owned_resources(move_to_ready=move_to_ready)

    def _close_owned_resources(self, *, move_to_ready: bool) -> None:
        with self._condition:
            if self._resources_closed:
                return
            self._closed = True
            self._condition.notify_all()
        cleanup_errors: list[Exception] = []
        if self._process and self._process.poll() is None:
            try:
                command = (
                    NativeCommand.MOVE_TO_READY_AND_SHUTDOWN
                    if move_to_ready
                    else NativeCommand.SHUTDOWN
                )
                self._send_lifecycle(command, timeout_s=60.0 if move_to_ready else 3.0)
            except Exception:
                try:
                    self._process.terminate()
                except Exception as exc:  # noqa: BLE001 - finish retiring owned resources
                    cleanup_errors.append(exc)
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    self._process.kill()
                except Exception as exc:  # noqa: BLE001 - finish retiring owned resources
                    cleanup_errors.append(exc)
                try:
                    self._process.wait(timeout=2)
                except Exception as exc:  # noqa: BLE001 - finish retiring owned resources
                    cleanup_errors.append(exc)
            except Exception as exc:  # noqa: BLE001 - finish retiring owned resources
                cleanup_errors.append(exc)
        if self._parent_liveness_write_fd is not None:
            try:
                os.close(self._parent_liveness_write_fd)
            except OSError as exc:
                cleanup_errors.append(exc)
            finally:
                self._parent_liveness_write_fd = None
        with self._condition:
            self._running = False
            self._condition.notify_all()
        self._heartbeat_stop.set()
        if self._heartbeat and self._heartbeat.is_alive():
            self._heartbeat.join(timeout=2)
        if self._reader and self._reader.is_alive():
            # Subscriber sockets belong to the reader thread. Its receives are
            # nonblocking, so wait for the current cycle before closing them.
            self._reader.join()
        if self._log_reader and self._log_reader.is_alive():
            self._log_reader.join(timeout=1)
        with self._condition:
            if self._log_tee is not None:
                try:
                    self._log_tee.close()
                except OSError as exc:
                    cleanup_errors.append(exc)
                self._log_tee = None
        process_stdout = getattr(self._process, "stdout", None)
        if process_stdout:
            try:
                process_stdout.close()
            except Exception as exc:  # noqa: BLE001 - retire every owned resource
                cleanup_errors.append(exc)
        with self._condition:
            for socket in (
                self._state_sub,
                self._status_sub,
                self._inputs_sub,
                self._direct_pub,
                self._lifecycle_pub,
            ):
                if socket:
                    try:
                        socket.close()
                    except Exception as exc:  # noqa: BLE001 - retire every owned resource
                        cleanup_errors.append(exc)
            try:
                self._context.term()
            except Exception as exc:  # noqa: BLE001 - surface after all cleanup attempts
                cleanup_errors.append(exc)
        if cleanup_errors:
            raise cleanup_errors[0]
        with self._condition:
            self._resources_closed = True
