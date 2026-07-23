import logging
import pathlib

import pytest

from openpi_control import log_paths, runlog


@pytest.fixture(autouse=True)
def isolated_log_dir(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch) -> pathlib.Path:
    monkeypatch.setenv(log_paths.OPENPI_LOG_DIR_ENV, str(tmp_path / "logs"))
    return tmp_path / "logs"


def test_setup_creates_run_log_and_captures_records(isolated_log_dir: pathlib.Path) -> None:
    log_path = runlog.setup_run_logging("doctor")

    assert log_path == isolated_log_dir / "runtime" / "doctor.log"
    logging.getLogger("openpi_control.test").info("marker line for the run log")
    content = log_path.read_text()
    assert "run started: doctor" in content
    assert "marker line for the run log" in content


def test_repeated_setup_does_not_stack_handlers(isolated_log_dir: pathlib.Path) -> None:
    first = runlog.setup_run_logging("evaluate")
    second = runlog.setup_run_logging("evaluate")

    assert first == second
    handlers = [
        handler
        for handler in logging.getLogger().handlers
        if isinstance(handler, logging.handlers.RotatingFileHandler)
        and pathlib.Path(handler.baseFilename) == second
    ]
    assert len(handlers) == 1
