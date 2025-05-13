"""
Configuration validation for FEAGI.

This module provides functions for validating configuration settings.
"""

from feagi.utils.logger import setup_logger
logger = setup_logger("feagi.config")
from typing import Dict, Any, Tuple, List



def validate_config(config: Dict[str, Any]) -> bool:
    """
    Validate the configuration.
    
    This function checks that the configuration contains valid settings.
    
    Args:
        config: Configuration dictionary to validate.
        
    Returns:
        True if the configuration is valid, False otherwise.
    """
    # Handle empty config
    if not config:
        return True
        
    try:
        # Validate NPU backend
        if "npu" in config:
            if "backend" in config["npu"]:
                backend = config["npu"]["backend"]
                if backend not in ["auto", "cpu", "cuda", "webgpu", "metal"]:
                    logger.warning(f"Invalid NPU backend: {backend}")
                    return False
        
        return True
    except Exception as e:
        logger.error(f"Configuration validation failed: {e}")
        return False 