"""Unit tests for the [mujoco_control_server] section of the FEAGI config file.

Covers the loader (``load_mujoco_control_server_config``), the central default table,
and the one-time migration performed by ``ensure_default_config``. Everything runs
against tmp_path-isolated files, so the user's real
``~/.feagi/config/feagi_configuration.toml`` is never touched.
"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
import toml

from feagi.config import (
    DEFAULT_CONFIG_CONTENT,
    REQUIRED_CONFIG_DEFAULTS,
    MujocoControlServerConfig,
    ensure_default_config,
    load_mujoco_control_server_config,
)

_KEYS = (
    "live_measure_timeout_s",
    "physics_command_timeout_s",
    "scene_object_apply_timeout_s",
)

_VALID = """
[mujoco_control_server]
live_measure_timeout_s = 1.5
physics_command_timeout_s = 2.5
scene_object_apply_timeout_s = 7
"""


def _write(path: Path, content: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    return path


def _section(**overrides: object) -> str:
    values: dict[str, object] = {
        "live_measure_timeout_s": 1.0,
        "physics_command_timeout_s": 2.0,
        "scene_object_apply_timeout_s": 3.0,
    }
    values.update(overrides)
    return "[mujoco_control_server]\n" + "\n".join(f"{k} = {v}" for k, v in values.items()) + "\n"


def _migrate(config_path: Path) -> Path:
    """Run ensure_default_config against ``config_path`` instead of the user's file."""
    mock_paths = MagicMock()
    mock_paths.get_default_config.return_value = config_path
    mock_paths.ensure_config_dir.side_effect = lambda: config_path.parent.mkdir(
        parents=True, exist_ok=True
    )
    with patch("feagi.paths.get_feagi_paths", return_value=mock_paths):
        return ensure_default_config()


# --- defaults ---------------------------------------------------------------------------


def test_default_config_template_includes_the_section():
    section = toml.loads(DEFAULT_CONFIG_CONTENT)["mujoco_control_server"]
    assert section == REQUIRED_CONFIG_DEFAULTS["mujoco_control_server"]
    assert set(section) == set(_KEYS)
    assert all(value > 0 for value in section.values())


def test_required_defaults_hold_the_three_timeouts():
    assert REQUIRED_CONFIG_DEFAULTS["mujoco_control_server"] == {
        "live_measure_timeout_s": 2.0,
        "physics_command_timeout_s": 3.0,
        "scene_object_apply_timeout_s": 10.0,
    }


# --- loader -----------------------------------------------------------------------------


def test_loader_returns_a_frozen_validated_dataclass(tmp_path):
    cfg = load_mujoco_control_server_config(_write(tmp_path / "c.toml", _VALID))
    assert cfg == MujocoControlServerConfig(1.5, 2.5, 7.0)
    assert isinstance(cfg.scene_object_apply_timeout_s, float)  # int 7 widened
    with pytest.raises(Exception):
        cfg.live_measure_timeout_s = 99.0  # type: ignore[misc]


def test_loader_missing_section_raises(tmp_path):
    path = _write(tmp_path / "c.toml", "[api]\nport = 8000\n")
    with pytest.raises(KeyError, match=r"\[mujoco_control_server\]"):
        load_mujoco_control_server_config(path)


def test_loader_missing_key_raises_naming_it(tmp_path):
    body = "[mujoco_control_server]\nlive_measure_timeout_s = 1.0\nphysics_command_timeout_s = 2.0\n"
    with pytest.raises(KeyError, match="scene_object_apply_timeout_s"):
        load_mujoco_control_server_config(_write(tmp_path / "c.toml", body))


def test_loader_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_mujoco_control_server_config(tmp_path / "absent.toml")


@pytest.mark.parametrize("key", _KEYS)
@pytest.mark.parametrize("bad", ["0", "0.0", "-1.0", "nan", "inf", "-inf"])
def test_loader_rejects_non_positive_or_non_finite(tmp_path, key, bad):
    path = _write(tmp_path / "c.toml", _section(**{key: bad}))
    with pytest.raises(ValueError, match=key):
        load_mujoco_control_server_config(path)


@pytest.mark.parametrize("key", _KEYS)
def test_loader_rejects_bool_and_string(tmp_path, key):
    for bad, needle in (("true", "bool"), ('"2"', "str")):
        path = _write(tmp_path / "c.toml", _section(**{key: bad}))
        with pytest.raises(TypeError, match=needle):
            load_mujoco_control_server_config(path)


# --- migration --------------------------------------------------------------------------


def test_ensure_default_config_adds_the_section_to_a_file_that_lacks_it(tmp_path):
    path = _write(
        tmp_path / "feagi_configuration.toml",
        '[api]\nhost = "127.0.0.1"\nport = 8000\n\n[recovery]\nhealth_poll_interval_s = 9.0\n',
    )
    assert _migrate(path) == path
    migrated = toml.load(path)
    assert migrated["mujoco_control_server"] == REQUIRED_CONFIG_DEFAULTS["mujoco_control_server"]
    assert migrated["api"]["port"] == 8000  # existing values preserved
    assert migrated["recovery"]["health_poll_interval_s"] == 9.0
    # The migrated file now loads through the typed loader.
    assert load_mujoco_control_server_config(path) == MujocoControlServerConfig(2.0, 3.0, 10.0)


def test_ensure_default_config_keeps_user_values_and_fills_missing_keys(tmp_path):
    path = _write(
        tmp_path / "feagi_configuration.toml",
        "[mujoco_control_server]\nlive_measure_timeout_s = 0.75\n",
    )
    _migrate(path)
    section = toml.load(path)["mujoco_control_server"]
    assert section["live_measure_timeout_s"] == 0.75
    assert section["physics_command_timeout_s"] == 3.0
    assert section["scene_object_apply_timeout_s"] == 10.0


def test_loader_without_a_path_migrates_through_ensure_default_config(tmp_path):
    path = _write(tmp_path / "feagi_configuration.toml", "[api]\nport = 8000\n")
    mock_paths = MagicMock()
    mock_paths.get_default_config.return_value = path
    with patch("feagi.paths.get_feagi_paths", return_value=mock_paths):
        cfg = load_mujoco_control_server_config()
    assert cfg == MujocoControlServerConfig(2.0, 3.0, 10.0)


def test_a_fresh_install_gets_the_section_from_the_template(tmp_path):
    path = tmp_path / "new" / "feagi_configuration.toml"
    _migrate(path)
    assert load_mujoco_control_server_config(path) == MujocoControlServerConfig(2.0, 3.0, 10.0)
