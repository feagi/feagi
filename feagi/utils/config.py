"""
Configuration utilities for FEAGI.

This module provides configuration management for FEAGI components,
allowing settings to be loaded from files or environment variables.
"""

import logging
import os
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)

class FeagiConfig:
    """
    Configuration manager for FEAGI.
    
    This class manages configuration settings for FEAGI components,
    with support for default values and dot notation access.
    """
    
    def __init__(self, config_dict: Optional[Dict[str, Any]] = None):
        """
        Initialize configuration with optional dictionary.
        
        Args:
            config_dict: Optional configuration dictionary
        """
        self._config = config_dict or {}
        self._defaults = {
            "connectome.max_neurons": 10000000,
            "connectome.max_synapses_per_neuron": 1000,
        }
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        Get a configuration value with dot notation.
        
        Args:
            key: Configuration key with dot notation (e.g., 'connectome.max_neurons')
            default: Default value if not found
            
        Returns:
            Configuration value or default
        """
        # Check if key is in config
        if key in self._config:
            return self._config[key]
        
        # Try hierarchical access with dots
        parts = key.split('.')
        config = self._config
        for part in parts:
            if isinstance(config, dict) and part in config:
                config = config[part]
            else:
                # Not found in config, try defaults
                return self._defaults.get(key, default)
        
        return config
    
    def set(self, key: str, value: Any) -> None:
        """
        Set a configuration value.
        
        Args:
            key: Configuration key with dot notation
            value: Value to set
        """
        self._config[key] = value
        
    def __repr__(self) -> str:
        """String representation of the config."""
        return f"FeagiConfig({self._config})" 