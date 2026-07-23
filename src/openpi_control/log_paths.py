"""Persistent log directory policy shared by openpi-control and the AI layer.

Every long-lived diagnostic artifact (native node output tees, CLI run logs,
crash captures) lands under a single per-user directory so post-mortem debugging
after a hardware session never depends on scrollback. The location can be
overridden with the OPENPI_LOG_DIR environment variable.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

OPENPI_LOG_DIR_ENV = "OPENPI_LOG_DIR"
DEFAULT_LOG_DIR = "~/openpi-data/logs"


def log_dir() -> Path:
    """Return the persistent log directory, creating it if needed."""
    root = Path(os.environ.get(OPENPI_LOG_DIR_ENV, DEFAULT_LOG_DIR)).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    return root


def runtime_log_dir() -> Path:
    """Return the CLI run-log subdirectory, creating it if needed."""
    directory = log_dir() / "runtime"
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def rotate_existing(path: Path) -> None:
    """Preserve an existing log file by renaming it with its mtime timestamp.

    A fresh run always writes to the canonical name; the previous run's file is
    kept alongside as ``<stem>__YYYYmmdd-HHMMSS<suffix>``.
    """
    if not path.exists():
        return
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(path.stat().st_mtime))
    rotated = path.with_name(f"{path.stem}__{stamp}{path.suffix}")
    if rotated.exists():
        # Same-second restart: fold into the existing rotated file name with a
        # uniquifying counter rather than silently overwriting history.
        counter = 1
        while rotated.exists():
            rotated = path.with_name(f"{path.stem}__{stamp}-{counter}{path.suffix}")
            counter += 1
    path.rename(rotated)
