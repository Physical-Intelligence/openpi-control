"""Persistent run logs and crash capture for the runtime CLI tools.

Attaches a rotating file handler under the shared OpenPI log directory
(``openpi_control.log_paths``) and captures unhandled exceptions plus native
crashes (via ``faulthandler``) into the same file, so every doctor/preview/
record/evaluate run leaves a diagnosable trace on disk. Console output is
unchanged.
"""

from __future__ import annotations

import faulthandler
import logging
import logging.handlers
import sys
from pathlib import Path
from types import TracebackType

from openpi_control import log_paths

_MAX_BYTES = 1_000_000
_BACKUP_COUNT = 3

# Keep module-level references so repeated setup calls (e.g. in tests) do not
# stack handlers, and so the faulthandler stream outlives this function.
_state: dict[str, object] = {}


def setup_run_logging(command: str) -> Path:
    """Attach persistent logging and crash capture for a CLI run.

    Args:
        command: CLI command name (doctor/preview/record/evaluate); selects the
            log file ``<log_dir>/runtime/<command>.log``.

    Returns:
        Path of the run log file.
    """
    log_path = log_paths.runtime_log_dir() / f"{command}.log"

    root = logging.getLogger()
    previous = _state.get("handler")
    if isinstance(previous, logging.Handler):
        root.removeHandler(previous)
        previous.close()

    handler = logging.handlers.RotatingFileHandler(
        log_path, maxBytes=_MAX_BYTES, backupCount=_BACKUP_COUNT, encoding="utf-8"
    )
    handler.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")
    )
    handler.setLevel(logging.DEBUG)
    root.addHandler(handler)
    if root.level > logging.INFO or root.level == logging.NOTSET:
        root.setLevel(logging.INFO)
    _state["handler"] = handler

    # Native crashes (segfault in the ZMQ/CAN stack, fatal signals) dump all
    # thread stacks into the run log. The stream must stay open for the whole
    # process lifetime, hence the module-level reference.
    crash_stream = log_path.open("a", encoding="utf-8")
    if faulthandler.is_enabled():
        faulthandler.disable()
    faulthandler.enable(file=crash_stream)
    _state["crash_stream"] = crash_stream

    def _log_unhandled(
        exc_type: type[BaseException],
        exc_value: BaseException,
        exc_traceback: TracebackType | None,
    ) -> None:
        if not issubclass(exc_type, KeyboardInterrupt):
            logging.getLogger("openpi_control.crash").critical(
                "unhandled exception", exc_info=(exc_type, exc_value, exc_traceback)
            )
        sys.__excepthook__(exc_type, exc_value, exc_traceback)

    sys.excepthook = _log_unhandled

    logging.getLogger("openpi_control").info("run started: %s (log: %s)", command, log_path)
    return log_path
