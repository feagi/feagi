"""
Tests for FEAGI engine detached spawn configuration.

These tests focus on platform-specific subprocess options used by
FeagiEngine daemon mode startup.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import feagi.engine.manager as engine_manager
from feagi.engine.manager import FeagiEngine


def test_build_detached_spawn_options_windows(monkeypatch):
    """Windows detached options use process-group and detached flags."""
    monkeypatch.setattr(engine_manager.os, "name", "nt", raising=False)
    monkeypatch.setattr(
        engine_manager.subprocess,
        "CREATE_NEW_PROCESS_GROUP",
        0x00000200,
        raising=False,
    )
    monkeypatch.setattr(
        engine_manager.subprocess,
        "DETACHED_PROCESS",
        0x00000008,
        raising=False,
    )

    options = engine_manager._build_detached_spawn_options()

    assert options["creationflags"] == (0x00000200 | 0x00000008)
    assert options["stdin"] is engine_manager.subprocess.DEVNULL
    assert "start_new_session" not in options


def test_build_detached_spawn_options_posix(monkeypatch):
    """POSIX detached options keep start_new_session behavior."""
    monkeypatch.setattr(engine_manager.os, "name", "posix", raising=False)

    options = engine_manager._build_detached_spawn_options()

    assert options == {"start_new_session": True}


def test_start_daemon_windows_uses_detached_spawn_options(monkeypatch, tmp_path):
    """FeagiEngine.start uses Windows detached kwargs in daemon mode."""
    popen_calls: list[dict[str, object]] = []

    class DummyProcess:
        """Minimal subprocess process stub for startup tests."""

        pid = 12345

        @staticmethod
        def poll():
            """Return running state (None means still running)."""
            return None

    def fake_popen(*args, **kwargs):
        """Capture subprocess kwargs and return a running process stub."""
        popen_calls.append({"args": args, "kwargs": kwargs})
        return DummyProcess()

    def fake_get_feagi_paths():
        """Provide temporary log directory creation for daemon startup."""
        return SimpleNamespace(
            create_log_run_dir=lambda component, retention: Path(tmp_path)
        )

    monkeypatch.setenv("FEAGI_DAEMON_MODE", "1")
    monkeypatch.setattr(engine_manager.os, "name", "nt", raising=False)
    monkeypatch.setattr(
        engine_manager.subprocess,
        "CREATE_NEW_PROCESS_GROUP",
        0x00000200,
        raising=False,
    )
    monkeypatch.setattr(
        engine_manager.subprocess,
        "DETACHED_PROCESS",
        0x00000008,
        raising=False,
    )
    monkeypatch.setattr(engine_manager.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(engine_manager.os, "open", lambda *_args, **_kwargs: 3)

    import feagi.paths

    monkeypatch.setattr(feagi.paths, "get_feagi_paths", fake_get_feagi_paths)

    engine = FeagiEngine(feagi_path="feagi", working_dir=str(tmp_path))
    started = engine.start(wait_for_ready=False)

    assert started is True
    assert len(popen_calls) == 1
    kwargs = popen_calls[0]["kwargs"]
    assert kwargs["creationflags"] == (0x00000200 | 0x00000008)
    assert kwargs["stdin"] is engine_manager.subprocess.DEVNULL
    assert "start_new_session" not in kwargs
