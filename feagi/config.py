"""
FEAGI Configuration Management

Handles default configuration generation and management for FEAGI SDK.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional
import logging

import toml

logger = logging.getLogger("feagi.config")

# Defaults for required sections when migrating existing configs (add-only, no overwrite).
# When a section already exists in the user's config, only missing keys inside it are
# added; existing keys are never overwritten.
REQUIRED_CONFIG_DEFAULTS: dict[str, Any] = {
    "timeouts": {"service_startup": 3.0},
    "recovery": {
        "health_poll_interval_s": 2.0,
        "health_fetch_timeout_ms": 1500,
        "reconnect_cooldown_ms": 1000,
        "reconnect_max_failures": 5,
    },
}


DEFAULT_CONFIG_CONTENT = """# FEAGI Configuration File
# This file is auto-generated. Modify as needed for your deployment.
#
# For more information, see: https://github.com/feagi/feagi/tree/main/docs

[api]
host = "127.0.0.1"  # Localhost only - secure by default, no firewall prompts. Override with FEAGI_API_HOST for network deployments
port = 8000

[websocket]
host = "127.0.0.1"  # Localhost only - secure by default, no firewall prompts. Override with FEAGI_WS_HOST for network deployments
enabled = true
visualization_port = 8080
sensory_port = 5558
motor_port = 5564

[burst_engine]
# Burst duration in seconds (0.01 = 10ms bursts)
burst_duration = 0.01

# Maximum bursts to execute (0 = unlimited)
max_bursts = 0

# Enable GPU acceleration if available
gpu_enabled = false

[performance]
# Number of worker threads for parallel processing
worker_threads = 4

# Enable performance profiling
profiling_enabled = false

[logging]
# Log level: debug, info, warning, error
level = "info"

# Enable file logging
file_logging = true

[timeouts]
# Timeout settings in seconds
service_startup = 3.0

[connectome]
# Maximum neuron space allocation
neuron_space = 1000000

# Maximum synapse space allocation
synapse_space = 10000000

[recovery]
# FeagiHealthMonitor / reconnect policy used by SDK controllers.
# The decision logic itself lives in the feagi-agent Rust crate so behavior
# is identical across the Python, Rust, and Java SDKs. These knobs only
# control *when* the controller polls FEAGI and how aggressively it
# reconnects. CLI flags on individual controllers may override these
# values; if no override is supplied, these values are used as-is. There
# are no implicit runtime fallbacks.
health_poll_interval_s   = 2.0    # How often the controller polls FEAGI /health (seconds)
health_fetch_timeout_ms  = 1500   # HTTP timeout for each health fetch (must be < poll interval)
reconnect_cooldown_ms    = 1000   # Minimum interval between consecutive reconnect attempts
reconnect_max_failures   = 5      # Give up after this many consecutive failed reconnects
"""


def generate_default_config(output_path: Optional[Path] = None, force: bool = False) -> Path:
    """
    Generate a default FEAGI configuration file.
    
    Args:
        output_path: Path to write config file (default: ~/.feagi/config/feagi_configuration.toml)
        force: If True, overwrite existing file
    
    Returns:
        Path to created configuration file
    
    Raises:
        FileExistsError: If file exists and force=False
    
    Example:
        from feagi.config import generate_default_config
        
        # Generate in default location
        config_path = generate_default_config()
        
        # Generate in custom location
        config_path = generate_default_config(Path("./my_config.toml"))
        
        # Overwrite existing
        config_path = generate_default_config(force=True)
    """
    from feagi.paths import get_feagi_paths
    
    if output_path is None:
        paths = get_feagi_paths()
        paths.ensure_config_dir()
        output_path = paths.get_default_config()
    else:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
    
    if output_path.exists() and not force:
        raise FileExistsError(
            f"Config file already exists: {output_path}\n"
            f"Use force=True to overwrite."
        )
    
    output_path.write_text(DEFAULT_CONFIG_CONTENT)
    logger.info(f"Generated default config: {output_path}")
    
    return output_path


def _merge_required_config_defaults(config: dict[str, Any]) -> bool:
    """
    Add required default sections/keys to config if missing (add-only, no overwrite).
    Returns True if config was modified.
    """
    modified = False
    for section, defaults in REQUIRED_CONFIG_DEFAULTS.items():
        if not isinstance(defaults, dict):
            continue
        if section not in config or not isinstance(config.get(section), dict):
            config[section] = dict(defaults)
            modified = True
        else:
            for key, value in defaults.items():
                if key not in config[section]:
                    config[section][key] = value
                    modified = True
    return modified


def ensure_default_config() -> Path:
    """
    Ensure default configuration file exists, creating it if necessary.
    If the file exists but is missing required sections (e.g. [timeouts]),
    they are added with default values (one-time migration).
    
    Returns:
        Path to configuration file (existing or newly created)
    
    Example:
        from feagi.config import ensure_default_config
        
        # Get or create default config
        config_path = ensure_default_config()
        print(f"Using config: {config_path}")
    """
    from feagi.paths import get_feagi_paths
    
    paths = get_feagi_paths()
    config_path = paths.get_default_config()
    
    if not config_path.exists():
        paths.ensure_config_dir()
        config_path.write_text(DEFAULT_CONFIG_CONTENT)
        logger.info(f"Created default config: {config_path}")
        return config_path
    
    try:
        data = toml.load(config_path)
        if _merge_required_config_defaults(data):
            config_path.write_text(toml.dumps(data))
            logger.info(f"Migrated config: added missing required sections to {config_path}")
    except Exception as exc:
        logger.warning("Could not migrate existing config: %s", exc)
    
    return config_path


@dataclass(frozen=True)
class RecoveryConfig:
    """
    Validated recovery / reconnect configuration consumed by SDK controllers.

    All four fields must be strictly positive. The dataclass is frozen so callers
    cannot mutate it after `load_recovery_config()` has returned, which keeps the
    config-first contract intact across the rest of the controller's lifetime.
    """

    health_poll_interval_s: float
    health_fetch_timeout_ms: int
    reconnect_cooldown_ms: int
    reconnect_max_failures: int


_RECOVERY_SECTION = "recovery"
_RECOVERY_REQUIRED_KEYS = (
    "health_poll_interval_s",
    "health_fetch_timeout_ms",
    "reconnect_cooldown_ms",
    "reconnect_max_failures",
)


def _coerce_recovery_value(key: str, raw: Any) -> Any:
    """Coerce TOML-decoded values to the type expected by RecoveryConfig.

    TOML may decode integers (e.g. ``2``) where a float is expected for
    ``health_poll_interval_s``. We only widen ints to floats for the float field;
    we never narrow floats to ints, since that would silently round user input.
    """
    if key == "health_poll_interval_s":
        if isinstance(raw, bool):  # bool is a subclass of int in Python
            raise TypeError(f"[recovery].{key} must be a number, got bool")
        if isinstance(raw, int):
            return float(raw)
        if isinstance(raw, float):
            return raw
        raise TypeError(
            f"[recovery].{key} must be a number, got {type(raw).__name__}"
        )

    if isinstance(raw, bool):
        raise TypeError(f"[recovery].{key} must be an integer, got bool")
    if isinstance(raw, int):
        return raw
    raise TypeError(
        f"[recovery].{key} must be an integer, got {type(raw).__name__}"
    )


def load_recovery_config(config_path: Optional[Path] = None) -> RecoveryConfig:
    """
    Load and validate the ``[recovery]`` section from the FEAGI config file.

    Resolution order:
        1. ``config_path`` argument if provided.
        2. ``ensure_default_config()`` (creates the file with defaults if missing
           and migrates older files to include the [recovery] section).

    The returned config is fully validated:
        - All four keys must be present.
        - All values must be strictly positive.
        - ``health_fetch_timeout_ms`` must be smaller than
          ``health_poll_interval_s * 1000`` so a fetch cannot outlive a tick.

    Args:
        config_path: Optional path to a TOML config file. If ``None``, the
            canonical user config is used (``ensure_default_config()``).

    Returns:
        A validated ``RecoveryConfig`` instance.

    Raises:
        FileNotFoundError: If ``config_path`` is provided and does not exist.
        KeyError: If the ``[recovery]`` section or one of its required keys
            is missing.
        ValueError: If a value is non-positive or violates the cross-field
            constraint.
        TypeError: If a value has the wrong TOML type.
    """
    if config_path is None:
        config_path = ensure_default_config()
    else:
        config_path = Path(config_path)
        if not config_path.exists():
            raise FileNotFoundError(
                f"FEAGI config file not found: {config_path}"
            )

    data = toml.load(config_path)
    section = data.get(_RECOVERY_SECTION)
    if not isinstance(section, dict):
        raise KeyError(
            f"Missing [recovery] section in {config_path}. "
            "Re-run ensure_default_config() to migrate older configs."
        )

    missing = [k for k in _RECOVERY_REQUIRED_KEYS if k not in section]
    if missing:
        raise KeyError(
            f"[recovery] section in {config_path} is missing keys: {missing}"
        )

    coerced: dict[str, Any] = {
        key: _coerce_recovery_value(key, section[key])
        for key in _RECOVERY_REQUIRED_KEYS
    }

    for key, value in coerced.items():
        if value <= 0:
            raise ValueError(
                f"[recovery].{key} must be > 0 (got {value}) in {config_path}"
            )

    poll_ms = coerced["health_poll_interval_s"] * 1000.0
    fetch_ms = coerced["health_fetch_timeout_ms"]
    if fetch_ms >= poll_ms:
        raise ValueError(
            "[recovery].health_fetch_timeout_ms "
            f"({fetch_ms} ms) must be smaller than "
            f"health_poll_interval_s * 1000 ({poll_ms} ms) in {config_path}"
        )

    return RecoveryConfig(**coerced)


def init_feagi_environment() -> dict:
    """
    Initialize FEAGI environment with all necessary directories and default config.
    
    Creates all standard directories and ensures default config exists.
    
    Returns:
        Dictionary with paths to key directories and files:
        {
            "config_dir": Path,
            "config_file": Path,
            "logs_dir": Path,
            "cache_dir": Path,
            "genomes_dir": Path,
            "connectomes_dir": Path
        }
    
    Example:
        from feagi.config import init_feagi_environment
        
        # Initialize everything
        env = init_feagi_environment()
        print(f"Config: {env['config_file']}")
        print(f"Genomes: {env['genomes_dir']}")
    """
    from feagi.paths import get_feagi_paths
    
    paths = get_feagi_paths()
    
    # Ensure all directories exist
    paths.ensure_all()
    
    # Ensure default config exists
    config_file = ensure_default_config()
    
    logger.info("FEAGI environment initialized")
    logger.info(f"  Config: {config_file}")
    logger.info(f"  Genomes: {paths.genomes_dir}")
    logger.info(f"  Connectomes: {paths.connectomes_dir}")
    
    return {
        "config_dir": paths.config_dir,
        "config_file": config_file,
        "logs_dir": paths.logs_dir,
        "cache_dir": paths.cache_dir,
        "genomes_dir": paths.genomes_dir,
        "connectomes_dir": paths.connectomes_dir,
    }
