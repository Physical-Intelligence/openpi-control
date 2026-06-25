"""Multi-arm ownership and atomic pair engagement."""

from __future__ import annotations

import threading
import uuid
from collections.abc import Iterator

from .arms import FollowerArm, LeaderArm
from .backend import ArmBackend
from .config import ArmConfig, SafetyLimits
from .exceptions import ConfigurationError
from .protocol import ArmTopics, topics_for
from .teleop import TeleopPair


class ArmSession:
    def __init__(self, *, session_id: str | None = None) -> None:
        self.session_id = session_id or uuid.uuid4().hex
        self._leaders: dict[str, LeaderArm] = {}
        self._followers: dict[str, FollowerArm] = {}
        self._pairs: list[TeleopPair] = []
        self._operation_lock = threading.RLock()

    @property
    def leaders(self) -> tuple[LeaderArm, ...]:
        with self._operation_lock:
            return tuple(self._leaders.values())

    @property
    def followers(self) -> tuple[FollowerArm, ...]:
        with self._operation_lock:
            return tuple(self._followers.values())

    @property
    def pairs(self) -> tuple[TeleopPair, ...]:
        with self._operation_lock:
            return tuple(self._pairs)

    def _check_name(self, name: str) -> None:
        if name in self._leaders or name in self._followers:
            raise ConfigurationError(f"logical arm name {name!r} is already in this session")

    def add_leader(self, config: ArmConfig, *, backend: ArmBackend | None = None) -> LeaderArm:
        with self._operation_lock:
            self._check_name(config.name)
            arm = LeaderArm(config, topics_for(self.session_id, config.name), backend)
            self._leaders[config.name] = arm
            return arm

    def add_follower(self, config: ArmConfig, *, backend: ArmBackend | None = None) -> FollowerArm:
        with self._operation_lock:
            self._check_name(config.name)
            arm = FollowerArm(config, topics_for(self.session_id, config.name), backend)
            self._followers[config.name] = arm
            return arm

    def add_pair(
        self,
        leader: LeaderArm | ArmConfig,
        follower: FollowerArm | ArmConfig,
        *,
        gain: float = 0.2,
        safety_limits: SafetyLimits | None = None,
    ) -> TeleopPair:
        with self._operation_lock:
            config_names = [
                arm.name for arm in (leader, follower) if isinstance(arm, ArmConfig)
            ]
            if len(config_names) != len(set(config_names)):
                raise ConfigurationError(
                    f"logical arm name {config_names[0]!r} is already used by this pair"
                )
            for name in config_names:
                self._check_name(name)
            if isinstance(leader, LeaderArm) and self._leaders.get(leader.name) is not leader:
                raise ConfigurationError("leader and follower arms must belong to this session")
            if (
                isinstance(follower, FollowerArm)
                and self._followers.get(follower.name) is not follower
            ):
                raise ConfigurationError("leader and follower arms must belong to this session")
            if isinstance(leader, ArmConfig):
                leader = self.add_leader(leader)
            if isinstance(follower, ArmConfig):
                follower = self.add_follower(follower)
            if leader.connected or follower.connected:
                raise ConfigurationError("pairs must be configured before session.connect()")
            if any(pair.leader is leader or pair.follower is follower for pair in self._pairs):
                raise ConfigurationError("an arm can belong to only one teleoperation pair")
            pair = TeleopPair(leader, follower, gain=gain, safety_limits=safety_limits)
            leader._configure_pair(follower)
            follower.topics = ArmTopics(
                state=follower.topics.state,
                live_command=leader.topics.live_command,
                direct_command=follower.topics.direct_command,
                lifecycle_command=follower.topics.lifecycle_command,
                status=follower.topics.status,
                inputs=follower.topics.inputs,
            )
            self._pairs.append(pair)
            return pair

    def connect(self) -> None:
        with self._operation_lock:
            for pair in self._pairs:
                if not pair._connections_are_current():
                    pair._block_engagement()
                    pair.disengage()
            newly_connected: list[LeaderArm | FollowerArm] = []
            try:
                for follower in self.followers:
                    if follower.connected:
                        continue
                    follower.connect()
                    newly_connected.append(follower)
                    follower._pause_live_input(True)
                for leader in self.leaders:
                    if leader.connected:
                        continue
                    leader.connect()
                    newly_connected.append(leader)
                    leader.enter_gravity_compensation()
                for pair in self._pairs:
                    pair._allow_engagement()
            except Exception:
                for arm in reversed(newly_connected):
                    try:
                        arm.close()
                    except Exception:
                        # Preserve the triggering setup failure and keep retiring
                        # the remaining arms; one failed close must not orphan them.
                        pass
                raise

    def disengage_all(self) -> None:
        for pair in self._pairs:
            pair.disengage()

    def close(self) -> None:
        with self._operation_lock:
            for pair in self._pairs:
                pair._block_engagement()
            # Teardown must always reach arm.close(): leaving a native node running
            # because disengage raised orphans an energized arm holding its ZMQ
            # ports (which then poisons port probing for the next session). Every
            # arm gets a close attempt; the first error resurfaces afterwards.
            errors: list[Exception] = []
            try:
                self.disengage_all()
            except Exception as exc:  # noqa: BLE001 - arms still need closing
                errors.append(exc)
            for arm in self._arms_reverse():
                try:
                    arm.close()
                except Exception as exc:  # noqa: BLE001 - keep closing the rest
                    errors.append(exc)
            if errors:
                raise errors[0]

    def _arms_reverse(self) -> Iterator[LeaderArm | FollowerArm]:
        yield from reversed(self.leaders)
        yield from reversed(self.followers)

    def __enter__(self) -> ArmSession:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
