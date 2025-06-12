"""
Configuration module for FEAGI.

This module handles loading, validation, and access to configuration settings.
"""

from feagi.config.loader import ConfigLoader
from feagi.config.validation import validate_config

# Global configuration instance
config = ConfigLoader().load_default()
validate_config(config)

__all__ = ["config", "ConfigLoader", "validate_config"]
