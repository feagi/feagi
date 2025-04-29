"""
Configuration loading utilities for FEAGI.

This module provides functions for loading configuration from various sources.
"""

import os
import logging
from typing import Dict, Any, Optional
import yaml
import copy

logger = logging.getLogger("feagi.config")

DEFAULT_CONFIG = {
    "npu": {
        "backend": "auto",
        "threads": 4,
        "burst_size": 100,
    },
    "logging": {
        "level": "INFO",
        "file": None,
    },
    "api": {
        "host": "127.0.0.1",
        "port": 8000,
    },
}


class ConfigLoader:
    """
    Configuration loader for FEAGI.
    
    This class handles loading configuration from various sources and provides
    a unified interface for accessing configuration values.
    """
    
    def __init__(self):
        """Initialize an empty configuration."""
        self._config: Dict[str, Any] = {}
    
    def load_default(self) -> Dict[str, Any]:
        """
        Load the default configuration.
        
        Returns:
            Default configuration dictionary.
        """
        self._config = copy.deepcopy(DEFAULT_CONFIG)
        return self._config
    
    def load_file(self, path: str) -> Dict[str, Any]:
        """
        Load configuration from a YAML file.
        
        Args:
            path: Path to the configuration file.
            
        Returns:
            Updated configuration dictionary.
            
        Raises:
            FileNotFoundError: If the configuration file does not exist.
            yaml.YAMLError: If the configuration file is invalid YAML.
        """
        path = os.path.expanduser(path)
        logger.info(f"Loading configuration from {path}")
        
        with open(path, "r") as f:
            file_config = yaml.safe_load(f)
        
        if not file_config:
            logger.warning(f"Configuration file {path} is empty or invalid")
            return self._config
        
        # Deep merge with existing config
        self._deep_update(self._config, file_config)
        return self._config
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        Get a configuration value.
        
        Keys are dot-separated, e.g., "npu.backend".
        
        Args:
            key: Configuration key.
            default: Default value if the key is not found.
            
        Returns:
            Configuration value, or default if not found.
        """
        parts = key.split(".")
        current = self._config
        
        for part in parts:
            if not isinstance(current, dict) or part not in current:
                return default
            current = current[part]
            
        return current
    
    def set(self, key: str, value: Any) -> None:
        """
        Set a configuration value.
        
        Keys are dot-separated, e.g., "npu.backend".
        
        Args:
            key: Configuration key.
            value: Configuration value.
        """
        parts = key.split(".")
        current = self._config
        
        # Navigate to the parent of the key
        for part in parts[:-1]:
            if part not in current:
                current[part] = {}
            elif not isinstance(current[part], dict):
                current[part] = {}
            current = current[part]
            
        # Set the value
        current[parts[-1]] = value
    
    @staticmethod
    def _deep_update(target: Dict[str, Any], source: Dict[str, Any]) -> None:
        """
        Deep update a dictionary with another dictionary.
        
        Args:
            target: Target dictionary to update.
            source: Source dictionary with new values.
        """
        for key, value in source.items():
            if key in target and isinstance(target[key], dict) and isinstance(value, dict):
                ConfigLoader._deep_update(target[key], value)
            else:
                target[key] = value 