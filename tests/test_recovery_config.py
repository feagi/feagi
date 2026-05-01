"""Unit tests for the [recovery] section of the FEAGI config file.

These tests cover both the loader (``load_recovery_config``) and the one-time
migration that ``ensure_default_config`` performs via
``_merge_required_config_defaults``. They run entirely against tmp_path-isolated
config files so the user's real ``~/.feagi/config/feagi_configuration.toml`` is
untouched.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import toml

from feagi.config import (
    DEFAULT_CONFIG_CONTENT,
    REQUIRED_CONFIG_DEFAULTS,
    RecoveryConfig,
    _merge_required_config_defaults,
    load_recovery_config,
)


def _write_config(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


VALID_RECOVERY = """
[recovery]
health_poll_interval_s = 2.0
health_fetch_timeout_ms = 1500
reconnect_cooldown_ms = 1000
reconnect_max_failures = 5
"""


def test_default_config_template_includes_recovery_section():
    """The shipped default template must already contain [recovery]."""
    parsed = toml.loads(DEFAULT_CONFIG_CONTENT)
    assert "recovery" in parsed
    section = parsed["recovery"]
    assert section["health_poll_interval_s"] > 0
    assert section["health_fetch_timeout_ms"] > 0
    assert section["reconnect_cooldown_ms"] > 0
    assert section["reconnect_max_failures"] > 0
    assert (
        section["health_fetch_timeout_ms"]
        < section["health_poll_interval_s"] * 1000
    )


def test_required_config_defaults_includes_recovery_section():
    """Migration table must include all four recovery keys."""
    assert "recovery" in REQUIRED_CONFIG_DEFAULTS
    expected_keys = {
        "health_poll_interval_s",
        "health_fetch_timeout_ms",
        "reconnect_cooldown_ms",
        "reconnect_max_failures",
    }
    assert set(REQUIRED_CONFIG_DEFAULTS["recovery"].keys()) == expected_keys


def test_load_recovery_config_returns_validated_dataclass(tmp_path):
    """A well-formed config produces a frozen, typed RecoveryConfig."""
    cfg_path = tmp_path / "feagi_configuration.toml"
    _write_config(cfg_path, VALID_RECOVERY)

    cfg = load_recovery_config(cfg_path)

    assert isinstance(cfg, RecoveryConfig)
    assert cfg.health_poll_interval_s == 2.0
    assert cfg.health_fetch_timeout_ms == 1500
    assert cfg.reconnect_cooldown_ms == 1000
    assert cfg.reconnect_max_failures == 5

    with pytest.raises(Exception):
        cfg.health_poll_interval_s = 99.0  # type: ignore[misc]


def test_load_recovery_config_widens_int_to_float(tmp_path):
    """An integer poll interval is coerced to float (TOML decodes ``2`` as int)."""
    cfg_path = tmp_path / "feagi_configuration.toml"
    _write_config(
        cfg_path,
        """
[recovery]
health_poll_interval_s = 2
health_fetch_timeout_ms = 500
reconnect_cooldown_ms = 250
reconnect_max_failures = 3
""",
    )

    cfg = load_recovery_config(cfg_path)
    assert isinstance(cfg.health_poll_interval_s, float)
    assert cfg.health_poll_interval_s == 2.0


def test_load_recovery_config_missing_section_raises(tmp_path):
    cfg_path = tmp_path / "feagi_configuration.toml"
    _write_config(cfg_path, "[api]\nport = 8000\n")

    with pytest.raises(KeyError, match=r"\[recovery\]"):
        load_recovery_config(cfg_path)


def test_load_recovery_config_missing_key_raises(tmp_path):
    cfg_path = tmp_path / "feagi_configuration.toml"
    _write_config(
        cfg_path,
        """
[recovery]
health_poll_interval_s = 1.0
reconnect_cooldown_ms = 500
reconnect_max_failures = 3
""",
    )

    with pytest.raises(KeyError, match="health_fetch_timeout_ms"):
        load_recovery_config(cfg_path)


@pytest.mark.parametrize(
    "key,bad_value",
    [
        ("health_poll_interval_s", 0.0),
        ("health_poll_interval_s", -1.0),
        ("health_fetch_timeout_ms", 0),
        ("health_fetch_timeout_ms", -10),
        ("reconnect_cooldown_ms", 0),
        ("reconnect_cooldown_ms", -1),
        ("reconnect_max_failures", 0),
        ("reconnect_max_failures", -5),
    ],
)
def test_load_recovery_config_rejects_non_positive_values(
    tmp_path, key, bad_value
):
    cfg_path = tmp_path / "feagi_configuration.toml"
    base = {
        "health_poll_interval_s": 2.0,
        "health_fetch_timeout_ms": 500,
        "reconnect_cooldown_ms": 250,
        "reconnect_max_failures": 3,
    }
    base[key] = bad_value
    body = "[recovery]\n" + "\n".join(f"{k} = {v}" for k, v in base.items()) + "\n"
    _write_config(cfg_path, body)

    with pytest.raises(ValueError, match=key):
        load_recovery_config(cfg_path)


def test_load_recovery_config_rejects_fetch_timeout_at_or_above_poll_interval(
    tmp_path,
):
    cfg_path = tmp_path / "feagi_configuration.toml"
    _write_config(
        cfg_path,
        """
[recovery]
health_poll_interval_s = 1.0
health_fetch_timeout_ms = 1000
reconnect_cooldown_ms = 100
reconnect_max_failures = 3
""",
    )

    with pytest.raises(ValueError, match="health_fetch_timeout_ms"):
        load_recovery_config(cfg_path)


def test_load_recovery_config_rejects_bool_int_smuggling(tmp_path):
    """``True`` decodes as int(1) in some TOML libs; we reject it."""
    cfg_path = tmp_path / "feagi_configuration.toml"
    _write_config(
        cfg_path,
        """
[recovery]
health_poll_interval_s = 2.0
health_fetch_timeout_ms = 500
reconnect_cooldown_ms = 250
reconnect_max_failures = true
""",
    )

    with pytest.raises(TypeError, match="bool"):
        load_recovery_config(cfg_path)


def test_load_recovery_config_missing_path_raises(tmp_path):
    missing = tmp_path / "does_not_exist.toml"
    with pytest.raises(FileNotFoundError):
        load_recovery_config(missing)


def test_merge_required_config_defaults_adds_recovery_to_legacy_config(tmp_path):
    """Existing configs without [recovery] get migrated, existing keys preserved."""
    legacy = {
        "api": {"host": "127.0.0.1", "port": 8000},
        "timeouts": {"service_startup": 7.0},
    }
    modified = _merge_required_config_defaults(legacy)
    assert modified is True
    assert "recovery" in legacy
    assert legacy["recovery"] == REQUIRED_CONFIG_DEFAULTS["recovery"]
    assert legacy["timeouts"]["service_startup"] == 7.0


def test_merge_required_config_defaults_preserves_partial_recovery_section(
    tmp_path,
):
    """A user-edited [recovery] section keeps its values; only missing keys are added."""
    partial = {
        "recovery": {"health_poll_interval_s": 10.0},
    }
    modified = _merge_required_config_defaults(partial)
    assert modified is True
    assert partial["recovery"]["health_poll_interval_s"] == 10.0
    assert (
        partial["recovery"]["reconnect_max_failures"]
        == REQUIRED_CONFIG_DEFAULTS["recovery"]["reconnect_max_failures"]
    )


def test_merge_required_config_defaults_idempotent_when_full(tmp_path):
    full = {
        "timeouts": {"service_startup": 3.0},
        "recovery": dict(REQUIRED_CONFIG_DEFAULTS["recovery"]),
    }
    modified = _merge_required_config_defaults(full)
    assert modified is False
