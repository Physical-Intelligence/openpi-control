"""Shared persistent log directory policy."""

from __future__ import annotations

from pathlib import Path

import pytest

from openpi_control import log_paths


def test_log_dir_honors_env_override_and_creates_directories(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(log_paths.OPENPI_LOG_DIR_ENV, str(tmp_path / "custom" / "logs"))

    root = log_paths.log_dir()
    runtime = log_paths.runtime_log_dir()

    assert root == tmp_path / "custom" / "logs"
    assert root.is_dir()
    assert runtime == root / "runtime"
    assert runtime.is_dir()


def test_rotate_existing_preserves_previous_content(tmp_path: Path) -> None:
    target = tmp_path / "node.log"
    target.write_text("previous run")

    log_paths.rotate_existing(target)

    assert not target.exists()
    rotated = list(tmp_path.glob("node__*.log"))
    assert len(rotated) == 1
    assert rotated[0].read_text() == "previous run"


def test_rotate_existing_is_a_noop_without_a_file(tmp_path: Path) -> None:
    log_paths.rotate_existing(tmp_path / "absent.log")
    assert list(tmp_path.iterdir()) == []


def test_rotate_existing_uniquifies_same_second_restarts(tmp_path: Path) -> None:
    target = tmp_path / "node.log"
    for content in ("first", "second"):
        target.write_text(content)
        log_paths.rotate_existing(target)

    rotated_contents = sorted(path.read_text() for path in tmp_path.glob("node__*.log"))
    assert rotated_contents == ["first", "second"]
