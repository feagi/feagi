"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Configuration validation for FEAGI.

This module provides functions for validating configuration settings.
"""

from feagi.utils.logger import setup_logger

logger = setup_logger("feagi.config")
from typing import Any, Dict


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
