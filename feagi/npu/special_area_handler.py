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
Special Area Handler for FEAGI Neural Processing Unit.

Handles detection and management of special cortical areas that have specific behaviors
during neural simulation, such as power areas that inject neurons into the FCL.

@cursor:critical-path Special area detection and processing is critical for burst engine timing
@cursor:ffi-safe Uses static typing and minimal dynamic behavior for Rust compatibility
"""

import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set

from feagi.utils.logger import setup_logger

logger = setup_logger()

# Type aliases for clarity
CorticalId = str
NeuronId = int


@dataclass
class SpecialAreaConfig:
    """Configuration for a special cortical area."""

    area_id: CorticalId
    area_type: str  # "power", "modulator", etc.
    injection_timing: str  # "pre_burst", "during_burst", "post_burst"
    injection_probability: float  # 0.0 to 1.0 for probabilistic injection
    target_neurons: Optional[Set[NeuronId]] = (
        None  # Specific neurons to inject, None for all
    )
    enabled: bool = True


class SpecialAreaHandler:
    """
    Handler for core power area injection.

    Simplified for 100kHz performance: Directly accesses the core power area
    at cortical_idx=1 (___pwr) which is guaranteed to exist in every genome.
    No detection or caching needed - direct access for maximum reliability.
    """

    def __init__(
        self, connectome_manager: Any, config: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize the special area handler.

        Args:
            connectome_manager: The connectome manager instance
            config: Optional configuration parameters (unused but kept for compatibility)
        """
        self.connectome_manager = connectome_manager

        # Statistics only
        self.injection_count = 0
        self.last_injection_time = 0.0

        logger.info(
            "Special Area Handler initialized for core power area (cortical_idx=1)",
            status="[FAST]",
        )

    def get_power_area_neurons(self) -> List[NeuronId]:
        """
        Get neurons from core power area (cortical_idx=1) for injection.

        Direct access method optimized for 100kHz performance.
        No lookup overhead - directly accesses guaranteed core area.

        Returns:
            List of neuron IDs from the core power area (___pwr)
        """
        try:
            # Fast path: Get core power area neurons (cortical_idx=1) directly
            power_neurons = self.connectome_manager.get_neurons_by_cortical_idx(1)

            # PERFORMANCE: Only log at startup or critical errors - remove expensive debug operations
            if not power_neurons:
                logger.debug("[POWER DETECTION] No neurons found in core power area (cortical_idx=1)")
            elif len(power_neurons) != 1:
                # Only log critical corruption occasionally to avoid performance impact
                if not hasattr(self, '_corruption_logged') and len(power_neurons) > 5:
                    logger.error(f"🚨 Power area corruption: expected 1 neuron, found {len(power_neurons)}")
                    self._corruption_logged = True
            else:
                logger.debug(f"[POWER DETECTION] ✅ Found correct power neurons: {len(power_neurons)}")

            return power_neurons if power_neurons else []
        except KeyError as e:
            # cortical_idx=1 (___pwr area) doesn't exist - likely neurogenesis failed
            # Use DEBUG level to avoid log spam, only warn once per minute
            if not hasattr(self, "_last_pwr_warning_time"):
                self._last_pwr_warning_time = 0

            current_time = time.perf_counter()
            if (
                current_time - self._last_pwr_warning_time > 60.0
            ):  # Only warn once per minute
                logger.warning(
                    "[POWER DETECTION] Core power area (___pwr) not found - neurogenesis may have failed"
                )
                self._last_pwr_warning_time = current_time
            else:
                logger.debug(
                    f"[POWER DETECTION] Core power area (___pwr) not found: {e}"
                )

            return []
        except Exception as e:
            # Other errors - reduce log spam by using debug level
            logger.debug(
                f"[POWER DETECTION] Error accessing core power area (cortical_idx=1): {e}"
            )
            return []

    def get_all_power_neurons(self) -> Dict[CorticalId, List[NeuronId]]:
        """
        Get all power area neurons in dictionary format.

        BACKWARD COMPATIBILITY: This method maintains the old interface
        that returns a dictionary, but now only contains the single core power area.

        Returns:
            Dictionary mapping cortical_id to list of neuron IDs
            For core power area: {"___pwr": [neuron_ids]}
        """
        try:
            power_neurons = self.get_power_area_neurons()
            if power_neurons:
                return {"___pwr": power_neurons}
            else:
                return {}
        except Exception as e:
            logger.error(f"Error getting all power neurons: {e}")
            return {}

    def get_special_config(self, cortical_id: CorticalId) -> Optional[Any]:
        """
        Get configuration for a special area.

        SIMPLIFIED: For core power area, returns a simple config object.
        For other areas, returns None (not supported in simplified approach).

        Args:
            cortical_id: The cortical area ID

        Returns:
            Configuration object for the area, or None if not a special area
        """
        if cortical_id == "___pwr":
            # Return a simple config object for the core power area
            from types import SimpleNamespace

            return SimpleNamespace(
                enabled=True,
                injection_timing="pre_burst",
                injection_probability=1.0,
                cortical_id="___pwr",
                area_type="power",
            )
        else:
            # Not a special area in the simplified approach
            logger.debug(
                f"No special config for area {cortical_id} (not a core power area)"
            )
            return None

    def get_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about power area injection.

        Returns:
            Dictionary with statistics and performance metrics
        """
        return {
            "injection_count": self.injection_count,
            "last_injection_time": self.last_injection_time,
            "core_power_area": "cortical_idx=1 (___pwr)",
        }

    def record_injection(self) -> None:
        """
        Record that an injection has occurred.

        Updates internal statistics for monitoring injection frequency.
        """
        self.injection_count += 1
        self.last_injection_time = time.perf_counter()


# Example usage for testing
def example_usage():
    """Example usage of the SpecialAreaHandler."""
    # This would be used with a real connectome manager
    # handler = SpecialAreaHandler(connectome_manager)
    # handler.detect_special_areas()
    # power_areas = handler.get_power_areas()
    # power_neurons = handler.get_all_power_neurons()
    pass


if __name__ == "__main__":
    example_usage()
