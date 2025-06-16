# -*- coding: utf-8 -*-
#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
TOML Configuration Loader for FEAGI 2.0

This module provides TOML-based configuration loading with:
- Environment variable overrides
- Command-line parameter overrides
- Fail-fast port conflict detection
- Rust/RTOS compatibility
- Cloud-native deployment support

@cursor:critical-path This is core configuration infrastructure
"""

import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Union

try:
    import tomllib  # Python 3.11+
except ImportError:
    try:
        import tomli as tomllib  # fallback for older Python versions
    except ImportError as e:
        raise ImportError(
            "TOML support required. Install with: pip install tomli"
        ) from e

logger = logging.getLogger(__name__)

# Global cache for configuration to prevent repeated loading and log spam
_CONFIG_CACHE: Optional[Dict[str, Any]] = None
_CONFIG_CACHE_KEY: Optional[str] = None


def _get_cache_key(
    config_path: Optional[Path] = None, cli_args: Optional[Dict[str, Any]] = None
) -> str:
    """Generate a cache key based on config path and CLI arguments."""
    path_str = str(config_path) if config_path else "default"
    cli_str = str(sorted(cli_args.items())) if cli_args else "none"
    return f"{path_str}:{cli_str}"


@dataclass
class PortConfiguration:
    """
    Port configuration loaded from TOML with validation.

    All port numbers are required and must be explicitly configured.
    No default values are provided to enforce configuration-driven architecture.
    """

    # Required port configurations
    zmq_req_rep_port: int
    zmq_pub_sub_port: int
    zmq_push_pull_port: int
    zmq_sensory_port: int
    zmq_visualization_port: int
    zmq_rest_port: int
    zmq_motor_port: int

    def get_all_ports(self) -> Dict[str, int]:
        """Return all configured ports as a dictionary."""
        return {
            "zmq_req_rep": self.zmq_req_rep_port,
            "zmq_pub_sub": self.zmq_pub_sub_port,
            "zmq_push_pull": self.zmq_push_pull_port,
            "zmq_sensory": self.zmq_sensory_port,
            "zmq_visualization": self.zmq_visualization_port,
            "zmq_rest": self.zmq_rest_port,
            "zmq_motor": self.zmq_motor_port,
        }


@dataclass
class HostConfiguration:
    """
    Host configuration with required validation.

    All hosts must be explicitly configured - no defaults provided.
    """

    api_host: str
    zmq_host: str

    def __post_init__(self):
        """Validate required host configurations."""
        if not self.api_host or self.api_host == "":
            raise ValueError(
                "API host is required. Set via FEAGI_API_HOST environment variable "
                "or api.host in configuration file."
            )
        if not self.zmq_host or self.zmq_host == "":
            raise ValueError(
                "ZMQ host is required. Set via FEAGI_ZMQ_HOST environment variable "
                "or zmq.host in configuration file."
            )


@dataclass
class TimeoutConfiguration:
    """
    System timeout configurations loaded from TOML.

    All timeouts are configurable to support different deployment environments.
    """

    # System timeouts (seconds)
    graceful_shutdown: float = 8.0
    service_startup: float = 3.0
    thread_join: float = 2.0
    process_join: float = 2.0
    service_stop: float = 5.0
    visualization_shutdown: float = 5.0
    api_service_shutdown: float = 10.0
    fq_sampler_shutdown: float = 2.0

    # ZMQ timeouts (milliseconds)
    socket_connect_timeout: int = 1000
    socket_receive_timeout: int = 5000
    socket_send_timeout: int = 5000
    client_heartbeat_timeout: int = 30000
    inactive_client_timeout: int = 60000
    polling_timeout: int = 100


@dataclass
class GenomeConfiguration:
    """
    Genome loading and validation configurations loaded from TOML.

    Controls how FEAGI handles invalid genome files.
    """

    auto_recovery_on_validation_failure: bool = True  # Default: allow auto-recovery


class FeagiConfigurationError(Exception):
    """Custom exception for FEAGI configuration errors."""

    pass


def find_config_file() -> Path:
    """
    Find the FEAGI configuration file.

    Search order:
    1. Environment variable: FEAGI_CONFIG_PATH
    2. Current working directory: ./feagi_configuration.toml
    3. Script directory: <script_dir>/feagi_configuration.toml
    4. Parent directory: ../feagi_configuration.toml

    Returns:
        Path to the configuration file

    Raises:
        FeagiConfigurationError: If no config file is found
    """
    # Check environment variable first
    env_path = os.environ.get("FEAGI_CONFIG_PATH")
    if env_path:
        config_path = Path(env_path)
        if config_path.exists():
            return config_path
        else:
            raise FeagiConfigurationError(
                f"Config file specified by FEAGI_CONFIG_PATH not found: {env_path}"
            )

    # Search in common locations
    search_paths = [
        Path.cwd() / "feagi_configuration.toml",  # Current directory
        Path(__file__).parent.parent.parent / "feagi_configuration.toml",  # feagi_core/
        Path(__file__).parent.parent.parent.parent
        / "feagi_configuration.toml",  # Project root
    ]

    for path in search_paths:
        if path.exists():
            logger.info(f"Found FEAGI configuration at: {path}")
            return path

    raise FeagiConfigurationError(
        "FEAGI configuration file 'feagi_configuration.toml' not found in any of these locations:\n"
        + "\n".join(f"  - {path}" for path in search_paths)
        + "\n\nSet FEAGI_CONFIG_PATH environment variable to specify custom location."
    )


def apply_environment_overrides(config: Dict[str, Any]) -> Dict[str, Any]:
    """
    Apply environment variable overrides to configuration.

    Environment variable mapping:
    - FEAGI_API_HOST -> api.host
    - FEAGI_API_PORT -> api.port
    - FEAGI_ZMQ_HOST -> zmq.host
    - FEAGI_DATA_DIR -> system.data_dir
    - etc.

    Args:
        config: Base configuration dictionary

    Returns:
        Configuration with environment overrides applied
    """
    env_mappings = {
        # API settings
        "FEAGI_API_HOST": ("api", "host"),
        "FEAGI_API_PORT": ("api", "port"),
        "FEAGI_API_WORKERS": ("api", "workers"),
        "FEAGI_API_RELOAD": ("api", "reload"),
        # ZMQ settings
        "FEAGI_ZMQ_HOST": ("zmq", "host"),
        # System settings
        "FEAGI_DATA_DIR": ("system", "data_dir"),
        "FEAGI_MAX_CORES": ("system", "max_cores"),
        "FEAGI_LOG_LEVEL": ("system", "log_level"),
        # Database settings
        "MONGODB_HOST": ("database", "mongodb_host"),
        "MONGODB_PORT": ("database", "mongodb_port"),
        "INFLUXDB_URL": ("database", "influxdb_url"),
    }

    for env_var, (section, key) in env_mappings.items():
        value = os.environ.get(env_var)
        if value is not None:
            if section not in config:
                config[section] = {}

            # Type conversion
            if key in ["port", "workers", "max_cores", "mongodb_port"]:
                try:
                    value = int(value)
                except ValueError:
                    logger.warning(f"Invalid integer value for {env_var}: {value}")
                    continue
            elif key in ["reload", "debug"]:
                value = value.lower() in ("true", "1", "yes", "on")

            config[section][key] = value
            logger.info(
                f"Applied environment override: {env_var} -> {section}.{key} = {value}"
            )

    return config


def apply_cli_overrides(
    config: Dict[str, Any], cli_args: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Apply command-line argument overrides to configuration.

    Args:
        config: Base configuration dictionary
        cli_args: Command-line arguments (optional)

    Returns:
        Configuration with CLI overrides applied
    """
    if not cli_args:
        return config

    # Map CLI arguments to config paths
    cli_mappings = {
        "zmq_req_port": ("ports", "zmq_req_rep_port"),
        "zmq_pub_port": ("ports", "zmq_pub_sub_port"),
        "zmq_push_port": ("ports", "zmq_push_pull_port"),
        "zmq_sensory_port": ("ports", "zmq_sensory_port"),
        "zmq_visualization_port": ("ports", "zmq_visualization_port"),
        "zmq_rest_port": ("ports", "zmq_rest_port"),
        "zmq_motor_port": ("ports", "zmq_motor_port"),
        "api_port": ("api", "port"),
        "api_host": ("api", "host"),
        "zmq_host": ("zmq", "host"),
        "debug": ("system", "debug"),
        "log_level": ("system", "log_level"),
        "profile": ("system", "profile"),
        "embedded": ("system", "embedded"),
        "debug_api": ("debug", "api"),
        "debug_npu": ("debug", "npu"),
        "debug_zmq_outbound": ("debug", "zmq_outbound"),
        "debug_zmq_inbound": ("debug", "zmq_inbound"),
    }

    for cli_arg, (section, key) in cli_mappings.items():
        if cli_arg in cli_args and cli_args[cli_arg] is not None:
            if section not in config:
                config[section] = {}
            config[section][key] = cli_args[cli_arg]
            logger.info(
                f"Applied CLI override: --{cli_arg} -> {section}.{key} = {cli_args[cli_arg]}"
            )

    return config


def load_toml_configuration(
    config_path: Optional[Union[str, Path]] = None,
    cli_args: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Load FEAGI configuration from TOML file with overrides.

    Loading order (later overrides earlier):
    1. TOML file defaults
    2. Environment variables
    3. Command-line arguments

    Args:
        config_path: Optional path to config file
        cli_args: Optional command-line arguments

    Returns:
        Complete configuration dictionary

    Raises:
        FeagiConfigurationError: If configuration loading fails
    """
    try:
        # Find and load config file
        if config_path:
            config_file = Path(config_path)
        else:
            config_file = find_config_file()

        logger.info(f"Loading FEAGI configuration from: {config_file}")

        with open(config_file, "rb") as f:
            config = tomllib.load(f)

        # Apply overrides in order
        config = apply_environment_overrides(config)
        config = apply_cli_overrides(config, cli_args)

        # Validate port configuration
        # port_config = get_port_config(config)  # Unused variable removed

        logger.info("FEAGI configuration loaded successfully")
        return config

    except FileNotFoundError as e:
        raise FeagiConfigurationError(f"Configuration file not found: {e}") from e
    except Exception as e:
        # Handle both tomllib.TOMLDecodeError and any other errors
        if "TOML" in str(type(e)):
            raise FeagiConfigurationError(f"Invalid TOML syntax: {e}") from e
        raise FeagiConfigurationError(f"Failed to load configuration: {e}") from e


def get_port_config(config: Dict[str, Any]) -> PortConfiguration:
    """
    Extract and validate port configuration from loaded TOML config.

    Args:
        config: Configuration dictionary loaded from TOML

    Returns:
        PortConfiguration with all required ports

    Raises:
        ValueError: If any required port is missing or invalid
        KeyError: If ports section is missing from configuration
    """
    if "ports" not in config:
        raise KeyError("Missing 'ports' section in configuration")

    ports = config["ports"]

    # Validate all required ports are present
    required_ports = [
        "zmq_req_rep_port",
        "zmq_pub_sub_port",
        "zmq_push_pull_port",
        "zmq_sensory_port",
        "zmq_visualization_port",
        "zmq_rest_port",
        "zmq_motor_port",
    ]

    for port_name in required_ports:
        if port_name not in ports:
            raise ValueError(f"Missing required port configuration: {port_name}")
        port_value = ports[port_name]
        if not isinstance(port_value, int) or port_value <= 0:
            raise ValueError(
                f"Invalid port value for {port_name}: {port_value} (must be positive integer)"
            )

    return PortConfiguration(
        zmq_req_rep_port=ports["zmq_req_rep_port"],
        zmq_pub_sub_port=ports["zmq_pub_sub_port"],
        zmq_push_pull_port=ports["zmq_push_pull_port"],
        zmq_sensory_port=ports["zmq_sensory_port"],
        zmq_visualization_port=ports["zmq_visualization_port"],
        zmq_rest_port=ports["zmq_rest_port"],
        zmq_motor_port=ports["zmq_motor_port"],
    )


def get_host_config(config: Dict[str, Any]) -> HostConfiguration:
    """
    Extract and validate host configuration from loaded TOML config.

    Args:
        config: Configuration dictionary loaded from TOML

    Returns:
        HostConfiguration with validated hosts

    Raises:
        ValueError: If required hosts are missing or empty
    """
    api_host = config.get("api", {}).get("host", "")
    zmq_host = config.get("zmq", {}).get("host", "")

    # Allow environment variable overrides
    api_host = os.environ.get("FEAGI_API_HOST", api_host)
    zmq_host = os.environ.get("FEAGI_ZMQ_HOST", zmq_host)

    return HostConfiguration(api_host=api_host, zmq_host=zmq_host)


def get_timeout_config(config: Dict[str, Any]) -> TimeoutConfiguration:
    """
    Extract timeout configuration from loaded TOML config.

    Args:
        config: Configuration dictionary loaded from TOML

    Returns:
        TimeoutConfiguration with all timeout values
    """
    timeout_config = config.get("timeouts", {})
    zmq_config = config.get("zmq", {})

    return TimeoutConfiguration(
        # System timeouts
        graceful_shutdown=timeout_config.get("graceful_shutdown", 8.0),
        service_startup=timeout_config.get("service_startup", 3.0),
        thread_join=timeout_config.get("thread_join", 2.0),
        process_join=timeout_config.get("process_join", 2.0),
        service_stop=timeout_config.get("service_stop", 5.0),
        visualization_shutdown=timeout_config.get("visualization_shutdown", 5.0),
        api_service_shutdown=timeout_config.get("api_service_shutdown", 10.0),
        fq_sampler_shutdown=timeout_config.get("fq_sampler_shutdown", 2.0),
        # ZMQ timeouts
        socket_connect_timeout=zmq_config.get("socket_connect_timeout", 1000),
        socket_receive_timeout=zmq_config.get("socket_receive_timeout", 5000),
        socket_send_timeout=zmq_config.get("socket_send_timeout", 5000),
        client_heartbeat_timeout=zmq_config.get("client_heartbeat_timeout", 30000),
        inactive_client_timeout=zmq_config.get("inactive_client_timeout", 60000),
        polling_timeout=zmq_config.get("polling_timeout", 100),
    )


def get_genome_config(config: Dict[str, Any]) -> GenomeConfiguration:
    """
    Extract genome configuration from loaded TOML config.

    Args:
        config: Configuration dictionary loaded from TOML

    Returns:
        GenomeConfiguration with genome validation settings
    """
    genome_config = config.get("genome", {})

    return GenomeConfiguration(
        auto_recovery_on_validation_failure=genome_config.get(
            "auto_recovery_on_validation_failure", True
        )
    )


def validate_configuration(config: Dict[str, Any]) -> None:
    """
    Validate the complete configuration for consistency and correctness.

    Args:
        config: Configuration dictionary to validate

    Raises:
        FeagiConfigurationError: If validation fails
    """
    required_sections = ["system", "api", "ports", "zmq"]

    for section in required_sections:
        if section not in config:
            raise FeagiConfigurationError(
                f"Missing required configuration section: [{section}]"
            )

    # Validate port ranges
    port_config = get_port_config(config)
    for port_name, port_value in port_config.get_all_ports().items():
        if not (1024 <= port_value <= 65535):
            raise FeagiConfigurationError(
                f"Port {port_name} = {port_value} is outside valid range (1024-65535). "
                f"Edit feagi_configuration.toml to fix this."
            )

    # Validate API port with environment variable override support
    api_port = config.get("api", {}).get("port", 8080)
    # Check for environment variable override
    api_port = int(os.environ.get("FEAGI_API_PORT", api_port))

    # Validate port range
    if not (1024 <= api_port <= 65535):
        raise FeagiConfigurationError(
            f"API port = {api_port} is outside valid range (1024-65535). "
            f"Edit feagi_configuration.toml or set FEAGI_API_PORT environment variable to fix this."
        )

    logger.info("Configuration validation passed")


# Convenience function for common usage
def load_feagi_config(cli_args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Load and validate FEAGI configuration with all overrides applied.

    Uses global caching to prevent repeated file loading and log spam.

    Args:
        cli_args: Optional command-line arguments

    Returns:
        Complete, validated configuration dictionary
    """
    global _CONFIG_CACHE, _CONFIG_CACHE_KEY

    # Generate cache key for this configuration request
    cache_key = _get_cache_key(cli_args=cli_args)

    # Return cached configuration if available and matching
    if _CONFIG_CACHE is not None and _CONFIG_CACHE_KEY == cache_key:
        return _CONFIG_CACHE.copy()  # Return a copy to prevent modification

    # Load fresh configuration
    config = load_toml_configuration(cli_args=cli_args)
    validate_configuration(config)

    # Cache the result
    _CONFIG_CACHE = config.copy()
    _CONFIG_CACHE_KEY = cache_key

    return config
