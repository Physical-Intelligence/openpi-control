"""Unit tests for the role-specific public arm handles."""

from openpi_control.arms import FollowerArm
from openpi_control.config import ArmConfig, SocketCanConnection
from openpi_control.protocol import topics_for
from openpi_control.types import ArmMode


class _RecordingBackend:
    """Minimal ArmBackend stand-in that records the dispatched calls."""

    def __init__(self) -> None:
        self.modes: list[ArmMode] = []
        self.holds = 0
        self.float_thresholds: list[float | None] = []
        self.torq_rescales: list[tuple[float, ...]] = []

    def configure_pair(self, *, follower_state_topic: str) -> None:
        del follower_state_topic

    def set_mode(self, mode: ArmMode) -> None:
        self.modes.append(mode)

    def enter_gravity_float(self, drift_abort_rad: float | None = None) -> None:
        self.float_thresholds.append(drift_abort_rad)

    def set_torq_rescale(self, values: tuple[float, ...]) -> None:
        self.torq_rescales.append(values)

    def hold(self) -> None:
        self.holds += 1


def test_follower_arm_exposes_the_calibration_gravity_float() -> None:
    # gravity_tune floats a follower (gravity feed-forward only, no position
    # PD) between position moves; the native node accepts
    # ENTER_GRAVITY_COMPENSATION on followers since 00.00.91, watches the
    # runaway drift in its own control loop (a client round trip is too slow),
    # and HOLD re-engages position control at the current pose.
    config = ArmConfig("follower", "ARX_X5", SocketCanConnection("test"))
    backend = _RecordingBackend()
    arm = FollowerArm(config, topics_for("session", "follower"), backend=backend)

    arm.enter_gravity_compensation()
    arm.enter_gravity_compensation(drift_abort_rad=0.2)
    arm.hold()

    assert backend.float_thresholds == [None, 0.2]
    assert backend.holds == 1


def test_follower_arm_updates_torq_rescale_at_runtime() -> None:
    # gravity_tune switches calibration candidates without a node restart: the
    # native node applies the new per-joint values within one control tick
    # while the arm keeps holding.
    config = ArmConfig("follower", "ARX_X5", SocketCanConnection("test"))
    backend = _RecordingBackend()
    arm = FollowerArm(config, topics_for("session", "follower"), backend=backend)

    arm.set_torq_rescale([0.8, 0.8, 0.8, 1.5, 1.5, 1.5])

    assert backend.torq_rescales == [(0.8, 0.8, 0.8, 1.5, 1.5, 1.5)]
