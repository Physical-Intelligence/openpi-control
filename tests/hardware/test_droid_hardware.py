"""Opt-in DROID hardware acceptance checks.

These tests never run from the normal suite. Read-only checks require
OPENPI_DROID_HARDWARE=1; motion additionally requires OPENPI_DROID_ALLOW_MOTION=1.
"""

import os
import time

import pytest

from openpi_control import (
    ArmConfig,
    ArmSession,
    FrankaConnection,
    PositionCommand,
    RobotiqConnection,
)

pytestmark = pytest.mark.skipif(
    os.environ.get("OPENPI_DROID_HARDWARE") != "1",
    reason="DROID hardware acceptance is explicitly opt-in",
)


def _config(*, read_only: bool) -> ArmConfig:
    serial_device = os.environ.get("OPENPI_ROBOTIQ_DEVICE")
    return ArmConfig(
        "droid_acceptance",
        "Franka",
        FrankaConnection(
            os.environ.get("OPENPI_FRANKA_ADDRESS", "172.16.0.2"), read_only=read_only
        ),
        effector_model="Robotiq" if serial_device else None,
        effector_connection=RobotiqConnection(serial_device) if serial_device else None,
    )


def test_read_only_franka_state() -> None:
    with ArmSession() as session:
        robot = session.add_follower(_config(read_only=True))
        session.connect()
        state = robot.read_state(timeout_s=2)
        assert state.joints.position_rad.shape == (7,)
        assert state.diagnostics is not None
        assert not robot.capabilities.supports_direct_commands


@pytest.mark.skipif(
    os.environ.get("OPENPI_DROID_ALLOW_MOTION") != "1",
    reason="DROID motion acceptance is explicitly opt-in",
)
def test_reset_and_fifteen_hertz_target_hold() -> None:
    with ArmSession() as session:
        robot = session.add_follower(_config(read_only=False))
        session.connect()
        robot.move_to_ready()
        start = time.monotonic()
        sent = 0
        while time.monotonic() - start < 1:
            state = robot.latest_state
            assert state is not None
            robot.command(PositionCommand(state.joints.position_rad))
            sent += 1
            time.sleep(1 / 15)
        assert 13 <= sent <= 17
