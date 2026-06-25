"""Protocol-faithful stand-in for pi_control_node, used by test_native_backend.

Speaks the real ZMQ ABI from openpi_control.protocol: binds PUB sockets for the
state/status topics, subscribes to the lifecycle/direct topics, performs the
HANDSHAKE/READY startup, acknowledges lifecycle commands, and echoes direct
joint commands into the published state. Behavior knobs for failure-path tests
come from the FAKE_NODE_BEHAVIOR environment variable:

* ``normal`` (default) — full healthy lifecycle.
* ``bad-version`` — handshake advertises protocol version (9, 9).
* ``exit-early`` — exits with code 3 before any handshake.
* ``reject-commands`` — acknowledges every lifecycle command with code 7.
* ``stale-ready-after-ack`` — sends an uncorrelated READY after the move ACK,
  then sends the request-correlated completion before the ready-position state.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import zmq

from openpi_control.protocol import (
    CAP_DIRECT,
    CAP_FORCE_FEEDBACK,
    CAP_GRAVITY_COMP,
    CAP_LIVE_INPUT,
    CAP_MOVE_TO_READY,
    COMMAND_STRUCT,
    JOINT_STRUCT,
    PROTOCOL_VERSION,
    STATUS_STRUCT,
    NativeCommand,
    NativeStatus,
    port_candidates,
)

ALL_CAPS = CAP_DIRECT | CAP_LIVE_INPUT | CAP_GRAVITY_COMP | CAP_FORCE_FEEDBACK | CAP_MOVE_TO_READY


def bind_pub(context: zmq.Context, topic: str) -> zmq.Socket:
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    for port in port_candidates(topic, 16):
        try:
            socket.bind(f"tcp://127.0.0.1:{port}")
            return socket
        except zmq.ZMQError:
            continue
    raise SystemExit(f"fake node: no free port for {topic}")


def connect_sub(context: zmq.Context, topic: str) -> zmq.Socket:
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SUBSCRIBE, topic.encode())
    for port in port_candidates(topic):
        socket.connect(f"tcp://127.0.0.1:{port}")
    return socket


def recv_payload(socket: zmq.Socket, topic: str) -> bytes | None:
    try:
        received_topic, payload = socket.recv_multipart(zmq.NOBLOCK)
    except zmq.Again:
        return None
    return payload if received_topic == topic.encode() else None


def pack_status(status: NativeStatus, ints: tuple[int, ...] = ()) -> bytes:
    return STATUS_STRUCT.pack(
        int(status), 0, len(ints), *((0.0,) * 10), *(ints + (0,) * (10 - len(ints)))
    )


def pack_state(positions: list[float], sequence: int, joint_count: int) -> bytes:
    padded = positions + [0.0] * (10 - len(positions))
    zeros = [0.0] * 10
    temps = [25.0] * 10
    return JOINT_STRUCT.pack(*padded, *zeros, *zeros, *temps, *zeros, sequence, joint_count, 1, 0.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic_state", required=True)
    parser.add_argument("--topic_status", required=True)
    parser.add_argument("--topic_lifecycle", required=True)
    parser.add_argument("--topic_direct_command", required=True)
    args, _ = parser.parse_known_args()

    behavior = os.environ.get("FAKE_NODE_BEHAVIOR", "normal")
    if behavior == "exit-early":
        print("fake node: exiting early as requested")
        return 3

    version = (9, 9) if behavior == "bad-version" else PROTOCOL_VERSION
    ack_code = 7 if behavior == "reject-commands" else 0

    context = zmq.Context()
    state_pub = bind_pub(context, args.topic_state)
    status_pub = bind_pub(context, args.topic_status)
    lifecycle_sub = connect_sub(context, args.topic_lifecycle)
    direct_sub = connect_sub(context, args.topic_direct_command)

    positions = [0.0] * 6
    sequence = 0
    last_handshake = 0.0

    while True:
        now = time.monotonic()
        # PUB/SUB joins are asynchronous; repeat the handshake until shutdown
        # so a slow-joining subscriber cannot miss it.
        if now - last_handshake > 0.05:
            status_pub.send_multipart(
                [
                    args.topic_status.encode(),
                    pack_status(NativeStatus.HANDSHAKE, (*version, ALL_CAPS)),
                ]
            )
            status_pub.send_multipart([args.topic_status.encode(), pack_status(NativeStatus.READY)])
            last_handshake = now

        if (payload := recv_payload(direct_sub, args.topic_direct_command)) is not None:
            if len(payload) == JOINT_STRUCT.size:
                values = JOINT_STRUCT.unpack(payload)
                positions = list(values[0:6])

        if (payload := recv_payload(lifecycle_sub, args.topic_lifecycle)) is not None:
            if len(payload) == COMMAND_STRUCT.size:
                values = COMMAND_STRUCT.unpack(payload)
                command, request_id = values[0], values[13]
                status_pub.send_multipart(
                    [
                        args.topic_status.encode(),
                        pack_status(NativeStatus.COMMAND_ACK, (request_id, ack_code, command)),
                    ]
                )
                if command == NativeCommand.MOVE_TO_READY and ack_code == 0:
                    if behavior == "stale-ready-after-ack":
                        status_pub.send_multipart(
                            [args.topic_status.encode(), pack_status(NativeStatus.READY)]
                        )
                        time.sleep(0.25)
                        status_pub.send_multipart(
                            [
                                args.topic_status.encode(),
                                pack_status(NativeStatus.READY, (request_id,)),
                            ]
                        )
                        ready_marker = Path(os.environ["FAKE_NODE_READY_MARKER"])
                        state_release = Path(os.environ["FAKE_NODE_STATE_RELEASE"])
                        ready_marker.touch()
                        deadline = time.monotonic() + 10.0
                        while not state_release.exists():
                            if time.monotonic() >= deadline:
                                raise SystemExit(
                                    "fake node: timed out waiting to release ready state"
                                )
                            time.sleep(0.005)
                    positions = [0.0] * 6
                    sequence += 1
                    state_pub.send_multipart(
                        [
                            args.topic_state.encode(),
                            pack_state(positions, sequence, joint_count=6),
                        ]
                    )
                    if behavior != "stale-ready-after-ack":
                        status_pub.send_multipart(
                            [
                                args.topic_status.encode(),
                                pack_status(NativeStatus.READY, (request_id,)),
                            ]
                        )
                if command in (NativeCommand.SHUTDOWN, NativeCommand.MOVE_TO_READY_AND_SHUTDOWN):
                    print("fake node: shutdown acknowledged")
                    return 0

        sequence += 1
        state_pub.send_multipart(
            [args.topic_state.encode(), pack_state(positions, sequence, joint_count=6)]
        )
        time.sleep(0.005)


if __name__ == "__main__":
    sys.exit(main())
