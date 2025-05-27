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

import os
import sys
import logging
from pathlib import Path
from typing import Dict, Any, Optional, Union
from dataclasses import dataclass

try:
    import tomllib  # Python 3.11+
except ImportError:
    try:
        import tomli as tomllib  # fallback for older Python versions
    except ImportError:
        raise ImportError(
            "TOML support required. Install with: pip install tomli"
        )

logger = logging.getLogger(__name__)


@dataclass
class PortConfiguration:
    """Port configuration with validation."""
    zmq_req_rep_port: int = 5555
    zmq_pub_sub_port: int = 5556  
    zmq_push_pull_port: int = 5557
    zmq_sensory_port: int = 5558
    zmq_control_port: int = 5559
    zmq_visualization_port: int = 5562
    zmq_rest_port: int = 5563
    zmq_motor_port: int = 5564

    def get_all_ports(self) -> Dict[str, int]:
        """Get all port numbers as a dictionary."""
        return {
            'zmq_req_rep_port': self.zmq_req_rep_port,
            'zmq_pub_sub_port': self.zmq_pub_sub_port,
            'zmq_push_pull_port': self.zmq_push_pull_port,
            'zmq_sensory_port': self.zmq_sensory_port,
            'zmq_control_port': self.zmq_control_port,
            'zmq_visualization_port': self.zmq_visualization_port,
            'zmq_rest_port': self.zmq_rest_port,
            'zmq_motor_port': self.zmq_motor_port,
        }

    def validate_unique_ports(self) -> None:
        """Validate that all ports are unique."""
        ports = list(self.get_all_ports().values())
        if len(ports) != len(set(ports)):
            duplicates = [port for port in set(ports) if ports.count(port) > 1]
            raise ValueError(
                f"Port configuration contains duplicate ports: {duplicates}. "
                f"Edit feagi_configuration.toml to fix conflicts."
            )


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
    env_path = os.environ.get('FEAGI_CONFIG_PATH')
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
        Path(__file__).parent.parent.parent.parent / "feagi_configuration.toml",  # Project root
    ]
    
    for path in search_paths:
        if path.exists():
            logger.info(f"Found FEAGI configuration at: {path}")
            return path
    
    raise FeagiConfigurationError(
        f"FEAGI configuration file 'feagi_configuration.toml' not found in any of these locations:\n"
        + "\n".join(f"  - {path}" for path in search_paths) +
        f"\n\nSet FEAGI_CONFIG_PATH environment variable to specify custom location."
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
        'FEAGI_API_HOST': ('api', 'host'),
        'FEAGI_API_PORT': ('api', 'port'),
        'FEAGI_API_WORKERS': ('api', 'workers'),
        'FEAGI_API_RELOAD': ('api', 'reload'),
        
        # ZMQ settings
        'FEAGI_ZMQ_HOST': ('zmq', 'host'),
        
        # System settings
        'FEAGI_DATA_DIR': ('system', 'data_dir'),
        'FEAGI_MAX_CORES': ('system', 'max_cores'),
        'FEAGI_LOG_LEVEL': ('system', 'log_level'),
        
        # Database settings
        'MONGODB_HOST': ('database', 'mongodb_host'),
        'MONGODB_PORT': ('database', 'mongodb_port'),
        'INFLUXDB_URL': ('database', 'influxdb_url'),
    }
    
    for env_var, (section, key) in env_mappings.items():
        value = os.environ.get(env_var)
        if value is not None:
            if section not in config:
                config[section] = {}
            
            # Type conversion
            if key in ['port', 'workers', 'max_cores', 'mongodb_port']:
                try:
                    value = int(value)
                except ValueError:
                    logger.warning(f"Invalid integer value for {env_var}: {value}")
                    continue
            elif key in ['reload', 'debug']:
                value = value.lower() in ('true', '1', 'yes', 'on')
            
            config[section][key] = value
            logger.info(f"Applied environment override: {env_var} -> {section}.{key} = {value}")
    
    return config


def apply_cli_overrides(config: Dict[str, Any], cli_args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
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
        'zmq_req_port': ('ports', 'zmq_req_rep_port'),
        'zmq_pub_port': ('ports', 'zmq_pub_sub_port'),
        'zmq_push_port': ('ports', 'zmq_push_pull_port'),
        'zmq_sensory_port': ('ports', 'zmq_sensory_port'),
        'zmq_control_port': ('ports', 'zmq_control_port'),
        'zmq_visualization_port': ('ports', 'zmq_visualization_port'),
        'zmq_rest_port': ('ports', 'zmq_rest_port'),
        'zmq_motor_port': ('ports', 'zmq_motor_port'),
        'api_port': ('api', 'port'),
        'api_host': ('api', 'host'),
        'zmq_host': ('zmq', 'host'),
        'debug': ('system', 'debug'),
        'log_level': ('system', 'log_level'),
        'profile': ('system', 'profile'),
        'embedded': ('system', 'embedded'),
    }
    
    for cli_arg, (section, key) in cli_mappings.items():
        if cli_arg in cli_args and cli_args[cli_arg] is not None:
            if section not in config:
                config[section] = {}
            config[section][key] = cli_args[cli_arg]
            logger.info(f"Applied CLI override: --{cli_arg} -> {section}.{key} = {cli_args[cli_arg]}")
    
    return config


def load_toml_configuration(
    config_path: Optional[Union[str, Path]] = None,
    cli_args: Optional[Dict[str, Any]] = None
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
        
        with open(config_file, 'rb') as f:
            config = tomllib.load(f)
        
        # Apply overrides in order
        config = apply_environment_overrides(config)
        config = apply_cli_overrides(config, cli_args)
        
        # Validate port configuration
        port_config = PortConfiguration(**config.get('ports', {}))
        port_config.validate_unique_ports()
        
        # Update config with validated ports
        config['ports'] = port_config.get_all_ports()
        
        logger.info("FEAGI configuration loaded successfully")
        return config
        
    except FileNotFoundError as e:
        raise FeagiConfigurationError(f"Configuration file not found: {e}")
    except tomllib.TOMLDecodeError as e:
        raise FeagiConfigurationError(f"Invalid TOML syntax: {e}")
    except Exception as e:
        raise FeagiConfigurationError(f"Failed to load configuration: {e}")


def get_port_config(config: Dict[str, Any]) -> PortConfiguration:
    """
    Extract port configuration from the full config.
    
    Args:
        config: Full configuration dictionary
        
    Returns:
        PortConfiguration object
    """
    return PortConfiguration(**config.get('ports', {}))


def validate_configuration(config: Dict[str, Any]) -> None:
    """
    Validate the complete configuration for consistency and correctness.
    
    Args:
        config: Configuration dictionary to validate
        
    Raises:
        FeagiConfigurationError: If validation fails
    """
    required_sections = ['system', 'api', 'ports', 'zmq']
    
    for section in required_sections:
        if section not in config:
            raise FeagiConfigurationError(f"Missing required configuration section: [{section}]")
    
    # Validate port ranges
    port_config = get_port_config(config)
    for port_name, port_value in port_config.get_all_ports().items():
        if not (1024 <= port_value <= 65535):
            raise FeagiConfigurationError(
                f"Port {port_name} = {port_value} is outside valid range (1024-65535). "
                f"Edit feagi_configuration.toml to fix this."
            )
    
    # Validate API port separately
    api_port = config.get('api', {}).get('port', 8000)
    if not (1024 <= api_port <= 65535):
        raise FeagiConfigurationError(
            f"API port = {api_port} is outside valid range (1024-65535). "
            f"Edit feagi_configuration.toml to fix this."
        )
    
    logger.info("Configuration validation passed")


# Convenience function for common usage
def load_feagi_config(cli_args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Load and validate FEAGI configuration with all overrides applied.
    
    Args:
        cli_args: Optional command-line arguments
        
    Returns:
        Complete, validated configuration dictionary
    """
    config = load_toml_configuration(cli_args=cli_args)
    validate_configuration(config)
    return config 