"""Tests for selecting genome and connectome startup artifacts."""

from pathlib import Path

import pytest

from feagi.engine import FeagiEngine


def test_engine_loads_connectome_artifact_from_absolute_path(
    tmp_path: Path,
) -> None:
    """A valid connectome path becomes the selected startup artifact."""
    artifact_path = tmp_path / "trained.connectome"
    artifact_path.write_bytes(b"connectome")
    engine = FeagiEngine(feagi_path="feagi")

    result = engine.load_connectome(str(artifact_path))

    assert result is engine
    assert engine.connectome_path == artifact_path
    assert engine.genome_path is None


def test_engine_rejects_invalid_connectome_extension(
    tmp_path: Path,
) -> None:
    """Connectome startup selection rejects unrelated file formats."""
    artifact_path = tmp_path / "trained.bin"
    artifact_path.write_bytes(b"connectome")
    engine = FeagiEngine(feagi_path="feagi")

    with pytest.raises(ValueError, match=r"\.connectome extension"):
        engine.load_connectome(str(artifact_path))


def test_engine_brain_artifact_selection_is_mutually_exclusive(
    tmp_path: Path,
) -> None:
    """Selecting a connectome clears an earlier genome selection."""
    genome_path = tmp_path / "brain.genome"
    connectome_path = tmp_path / "brain.connectome"
    genome_path.write_text("{}")
    connectome_path.write_bytes(b"connectome")
    engine = FeagiEngine(feagi_path="feagi")

    engine.load_genome(str(genome_path))
    engine.load_connectome(str(connectome_path))

    assert engine.genome_path is None
    assert engine.connectome_path == connectome_path


def test_engine_passes_connectome_to_native_executable(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    """Engine startup forwards the selected connectome through native CLI."""
    artifact_path = tmp_path / "trained.connectome"
    artifact_path.write_bytes(b"connectome")
    captured_command: list[str] = []

    class DummyProcess:
        """Represent a running native FEAGI process."""

        pid = 123

        @staticmethod
        def poll() -> None:
            """Report that the process is still running."""
            return None

    def capture_popen(command: list[str], **_kwargs: object) -> DummyProcess:
        """Capture the native command without creating a subprocess."""
        captured_command.extend(command)
        return DummyProcess()

    monkeypatch.delenv("FEAGI_DAEMON_MODE", raising=False)
    monkeypatch.setattr("feagi.engine.manager.subprocess.Popen", capture_popen)
    engine = FeagiEngine(feagi_path="feagi", working_dir=str(tmp_path))
    engine.load_connectome(str(artifact_path))

    assert engine.start(wait_for_ready=False)
    assert captured_command == [
        "feagi",
        "--connectome",
        str(artifact_path),
    ]
    engine.process = None
