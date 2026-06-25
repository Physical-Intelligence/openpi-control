"""Leader/follower pairing and safe bilateral engagement."""

from __future__ import annotations

import threading
import time

import numpy as np

from .arms import FollowerArm, LeaderArm
from .config import SafetyLimits
from .exceptions import AlignmentError, CommandRejectedError, ConfigurationError, StaleStateError
from .types import ArmMode, PositionCommand

_ALIGNMENT_PERIOD_S = 0.01


class TeleopPair:
    def __init__(
        self,
        leader: LeaderArm,
        follower: FollowerArm,
        *,
        gain: float = 0.2,
        safety_limits: SafetyLimits | None = None,
    ) -> None:
        self.leader = leader
        self.follower = follower
        self.safety_limits = safety_limits or SafetyLimits()
        self._gain = gain
        self._engaged = False
        self._align_cancel = threading.Event()
        self._state_lock = threading.Lock()
        self._cleanup_lock = threading.Lock()
        self._engage_in_progress = False
        self._engagement_generation = 0
        self._engagement_arm_generations: tuple[int, int] | None = None
        self._needs_rollback = False
        self._engagement_blocked = False
        self.set_force_feedback_gain(gain)

    @property
    def engaged(self) -> bool:
        with self._state_lock:
            return self._engaged

    @property
    def gain(self) -> float:
        with self._state_lock:
            return self._gain

    def set_force_feedback_gain(self, gain: float) -> None:
        if not 0.0 <= gain <= self.safety_limits.max_bilateral_gain:
            raise ConfigurationError(
                f"bilateral gain must be in [0, {self.safety_limits.max_bilateral_gain}]"
            )
        try:
            with self._cleanup_lock:
                with self._state_lock:
                    engaged = self._engaged
                if engaged:
                    with self.leader._dispatch_lock:
                        self._assert_active_engagement()
                        self.leader._set_force_feedback_gain(gain)
                        with self.follower._dispatch_lock:
                            self._assert_active_engagement()
                            with self._state_lock:
                                self._gain = gain
                else:
                    with self._state_lock:
                        self._gain = gain
        except CommandRejectedError:
            self._rollback()
            raise

    def _validate_pair(self) -> tuple[object, object]:
        leader_state = self.leader.read_state()
        follower_state = self.follower.read_state()
        if not leader_state.is_fresh(self.safety_limits.max_state_age_s):
            raise StaleStateError(f"leader {self.leader.name} state is stale")
        if not follower_state.is_fresh(self.safety_limits.max_state_age_s):
            raise StaleStateError(f"follower {self.follower.name} state is stale")
        if leader_state.joints.names != follower_state.joints.names:
            raise AlignmentError("leader and follower joint order differs")
        if (leader_state.effector is None) != (follower_state.effector is None):
            raise AlignmentError("leader and follower effector capabilities differ")
        return leader_state, follower_state

    def _align(self) -> None:
        self.follower._pause_live_input(True)
        self._raise_if_alignment_cancelled()
        self.leader.enter_gravity_compensation()
        self._raise_if_alignment_cancelled()
        previous_leader, follower_state = self._validate_pair()
        self._raise_if_alignment_cancelled()
        command_position = np.asarray(follower_state.joints.position_rad, dtype=np.float64)
        command_effector = follower_state.effector.position if follower_state.effector else None
        max_joint_step = self.safety_limits.max_joint_velocity_rad_s * _ALIGNMENT_PERIOD_S
        max_effector_step = self.safety_limits.max_effector_velocity_s * _ALIGNMENT_PERIOD_S
        started = time.monotonic()
        next_step = started

        while True:
            self._raise_if_alignment_cancelled()

            now = time.monotonic()
            elapsed = now - started
            if elapsed >= self.safety_limits.max_alignment_duration_s:
                raise AlignmentError(
                    f"alignment timed out after {self.safety_limits.max_alignment_duration_s:g}s"
                )

            leader_state = self.leader.latest_state
            follower_state = self.follower.latest_state
            self._raise_if_alignment_cancelled()
            if leader_state is None or not leader_state.is_fresh(
                self.safety_limits.max_state_age_s
            ):
                raise StaleStateError(f"leader {self.leader.name} state became stale")
            if follower_state is None or not follower_state.is_fresh(
                self.safety_limits.max_state_age_s
            ):
                raise StaleStateError(f"follower {self.follower.name} state became stale")
            if leader_state.joints.names != follower_state.joints.names:
                raise AlignmentError("leader and follower joint order differs")
            if (leader_state.effector is None) != (follower_state.effector is None):
                raise AlignmentError("leader and follower effector capabilities differ")

            leader_step = np.max(
                np.abs(leader_state.joints.position_rad - previous_leader.joints.position_rad)
            )
            if leader_step > self.safety_limits.max_leader_drift_rad:
                raise AlignmentError(
                    "leader position changed discontinuously during alignment: "
                    f"{leader_step:.3f} rad exceeds "
                    f"{self.safety_limits.max_leader_drift_rad:.3f} rad"
                )
            previous_leader = leader_state

            joint_error = np.max(
                np.abs(follower_state.joints.position_rad - leader_state.joints.position_rad)
            )
            effector_error = 0.0
            if leader_state.effector and follower_state.effector:
                effector_error = abs(
                    follower_state.effector.position - leader_state.effector.position
                )
            if (
                elapsed >= self.safety_limits.minimum_alignment_duration_s
                and joint_error <= self.safety_limits.max_alignment_error_rad
                and effector_error <= self.safety_limits.max_effector_alignment_error
            ):
                return

            command_position = command_position + np.clip(
                leader_state.joints.position_rad - command_position,
                -max_joint_step,
                max_joint_step,
            )
            if leader_state.effector is not None:
                assert command_effector is not None
                command_effector += float(
                    np.clip(
                        leader_state.effector.position - command_effector,
                        -max_effector_step,
                        max_effector_step,
                    )
                )
            self.follower.command(PositionCommand(command_position, command_effector))
            self._raise_if_alignment_cancelled()

            next_step += _ALIGNMENT_PERIOD_S
            remaining = next_step - time.monotonic()
            if remaining > 0:
                self._align_cancel.wait(remaining)

    def _raise_if_alignment_cancelled(self) -> None:
        if self._align_cancel.is_set():
            with self._state_lock:
                self._needs_rollback = True
            raise AlignmentError("alignment cancelled")
        self._assert_engagement_arm_generations()

    def _enable(self) -> None:
        self._raise_if_alignment_cancelled()
        with self._cleanup_lock:
            with self.leader._lifecycle_lock:
                with self.follower._lifecycle_lock:
                    self._raise_if_alignment_cancelled()
                    with self._state_lock:
                        gain = self._gain
                    self.leader._enable_force_feedback(gain)
                    self._assert_engagement_arm_generations()
                    self.follower._pause_live_input(False)
                    self._assert_engagement_arm_generations()
                    self._mark_engaged()

    def _begin_engage(self) -> bool:
        with self._cleanup_lock:
            with self._state_lock:
                if self._engagement_blocked:
                    raise ConfigurationError("cannot engage while the owning session is closing")
                if not self.leader.connected or not self.follower.connected:
                    raise ConfigurationError("both arms must be connected to engage")
                engaged = self._engaged
                already_admitted = engaged or self._engage_in_progress
                if not already_admitted:
                    self._engagement_arm_generations = (
                        self.leader._capture_connection_generation(),
                        self.follower._capture_connection_generation(),
                    )
                    self._engage_in_progress = True
                    self._engagement_generation += 1
                    self._needs_rollback = True
                    self._align_cancel.clear()
                    return True
            if engaged:
                self._assert_active_engagement()
            else:
                self._assert_engagement_arm_generations()
            return False

    def _assert_engagement_arm_generations(self) -> None:
        with self._state_lock:
            generations = self._engagement_arm_generations
        if generations is None:
            raise ConfigurationError("engagement has no admitted arm generation")
        leader_generation, follower_generation = generations
        self.leader._assert_connection_generation(leader_generation, operation="engagement")
        self.follower._assert_connection_generation(follower_generation, operation="engagement")

    def _assert_active_engagement(self) -> None:
        with self.leader._dispatch_lock:
            self._assert_engagement_arm_generations()
            if self.leader.mode is not ArmMode.BILATERAL:
                raise CommandRejectedError(
                    f"leader {self.leader.name} is no longer in bilateral mode"
                )

    def _mark_engaged(self) -> None:
        with self._state_lock:
            generations = self._engagement_arm_generations
        if generations is None:
            raise ConfigurationError("engagement has no admitted arm generation")
        leader_generation, follower_generation = generations
        with self.leader._dispatch_lock:
            with self.follower._dispatch_lock:
                self.leader._assert_connection_generation(
                    leader_generation, operation="engagement"
                )
                self.follower._assert_connection_generation(
                    follower_generation, operation="engagement"
                )
                with self._state_lock:
                    self._engaged = True

    def _connections_are_current(self) -> bool:
        with self._state_lock:
            engaged = self._engaged
            requires_admitted_generations = engaged or self._engage_in_progress
        if not self.leader.connected or not self.follower.connected:
            return False
        if not requires_admitted_generations:
            return True
        try:
            if engaged:
                self._assert_active_engagement()
            else:
                self._assert_engagement_arm_generations()
        except (CommandRejectedError, ConfigurationError):
            return False
        return True

    def _block_engagement(self) -> None:
        with self._state_lock:
            self._engagement_blocked = True
            self._align_cancel.set()
        # Wait for any mirror/enable transition that passed the gate before it
        # closed. New transitions observe _engagement_blocked and reject.
        with self._cleanup_lock:
            pass

    def _allow_engagement(self) -> None:
        with self._state_lock:
            self._engagement_blocked = False

    def _finish_engage(self) -> None:
        with self._state_lock:
            self._engage_in_progress = False

    def engage(self) -> None:
        try:
            if not self._begin_engage():
                return
        except Exception:
            self.disengage()
            raise
        try:
            self._align()
            self._enable()
        except Exception:
            self._rollback()
            raise
        finally:
            self._finish_engage()

    def cancel_engage(self) -> None:
        """Cancel an in-progress alignment without changing an engaged pair."""
        self._align_cancel.set()

    def _rollback(self, *, expected_generation: int | None = None) -> None:
        """Best-effort return to the safe disengaged state.

        Every leg runs even if an earlier one raises (a follower whose native
        process died mid-teleop must not leave the leader latched in bilateral
        mode), and nothing propagates: this runs inside engage()'s exception
        path, where a raise here would mask the original failure.
        """
        with self._cleanup_lock:
            with self._state_lock:
                if (
                    expected_generation is not None
                    and expected_generation != self._engagement_generation
                ):
                    return
                if not self._needs_rollback:
                    return
                self._needs_rollback = False
                self._engaged = False
                arm_generations = self._engagement_arm_generations
            cleanup_failed = False
            if arm_generations is not None:
                leader_generation, follower_generation = arm_generations
                with self.follower._lifecycle_lock:
                    if (
                        self.follower._connection_generation == follower_generation
                        and self.follower._capabilities is not None
                        and self.follower._backend._is_connected()
                    ):
                        try:
                            self.follower._pause_live_input(True)
                        except Exception as exc:  # noqa: BLE001 - teardown must reach the leader
                            cleanup_failed = True
                            print(
                                f"teleop rollback: follower {self.follower.name} "
                                f"pause failed: {exc}",
                                flush=True,
                            )
                        try:
                            self.follower._hold_for_cleanup()
                        except Exception as exc:  # noqa: BLE001 - teardown must reach the leader
                            cleanup_failed = True
                            print(
                                f"teleop rollback: follower {self.follower.name} "
                                f"hold failed: {exc}",
                                flush=True,
                            )
                with self.leader._lifecycle_lock:
                    if (
                        self.leader._connection_generation == leader_generation
                        and self.leader._capabilities is not None
                        and self.leader._backend._is_connected()
                    ):
                        try:
                            self.leader.enter_gravity_compensation()
                        except Exception as exc:  # noqa: BLE001
                            cleanup_failed = True
                            print(
                                f"teleop rollback: leader {self.leader.name} cleanup failed: {exc}",
                                flush=True,
                            )
            if cleanup_failed:
                with self._state_lock:
                    self._needs_rollback = True

    def disengage(self) -> None:
        with self._state_lock:
            engagement_generation = self._engagement_generation
        self.cancel_engage()
        self._rollback(expected_generation=engagement_generation)
        time.sleep(0)
