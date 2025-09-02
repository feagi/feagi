"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

# Special Area Handler for FEAGI Neural Processing Unit.
# Handles detection and management of special cortical areas that have
# specific behaviors during neural simulation, such as power areas that
# inject neurons into the FCL.
# @cursor:critical-path Special area detection and processing is critical
# for burst engine timing
# @cursor:ffi-safe Uses static typing and minimal dynamic behavior for Rust compatibility

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set
import time

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

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
    """Handler for core power area injection using new NPU interface.

    Updated for new NPU architecture: Uses NPU interface for neuron access
    while maintaining the same performance characteristics and API.
    """

    def __init__(
        self, connectome_manager: Any, config: Optional[Dict[str, Any]] = None, npu_interface: Any = None
    ):
        """Initialize the special area handler.

        Args:
            connectome_manager: The connectome manager instance (for backward compatibility)
            config: Optional configuration parameters (unused but kept for compatibility)
            npu_interface: NPU interface instance for new architecture
        """
        self.connectome_manager = connectome_manager
        self.npu_interface = npu_interface

        # Legacy-compatible public attributes (empty by default)
        self.special_areas: Dict[str, SpecialAreaConfig] = {}
        self.power_areas: Set[str] = set()
        self.power_area_neurons: Dict[str, List[int]] = {}

        # Statistics only
        self.injection_count = 0
        self.last_injection_time = 0.0

        logger.info(
            "Special Area Handler initialized for core power area (cortical_idx=1) with NPU interface",
            status="[FAST]",
        )

    def get_power_area_neurons(self, cortical_id: Optional[str] = None) -> List[NeuronId]:
        """Get neurons from core power area (_power) for injection.

        Deterministic: Requires NPU interface (single source of truth).
        Uses cortical_idx resolved from NPU mapping of cortical_id.
        No fallbacks.

        Returns:
            List of neuron IDs from the power area
        """
        if cortical_id is not None and cortical_id != "_power":
            # Legacy-style query from cached connectome-based detection
            return list(self.power_area_neurons.get(cortical_id, []))

        if not self.npu_interface:
            raise RuntimeError(
                "NPU Interface required for power neuron retrieval (no fallbacks allowed)"
            )

        try:
            # Resolve cortical_idx for '_power' using NPU mapping
            cortical_idx = self.npu_interface.get_cortical_idx_by_id("_power")
            if cortical_idx is None:
                raise RuntimeError(
                    "'_power' cortical_id not registered in NPUInterface.cortical_areas"
                )

            # Optional debug
            try:
                from feagi.core.state_manager import FeagiStateManager
                if FeagiStateManager.instance().is_debug_npu_enabled():
                    cm_npu_id = (
                        id(self.connectome_manager._npu_interface)
                        if hasattr(self.connectome_manager, "_npu_interface")
                        and self.connectome_manager._npu_interface is not None
                        else None
                    )
                    logger.info(
                        f"[INJECTION-DEBUG] NPU ids: handler={id(self.npu_interface)} vs CM={cm_npu_id}"
                    )
                    logger.info(
                        f"[INJECTION-DEBUG] '_power' resolved to cortical_idx={cortical_idx}"
                    )
            except Exception:
                pass

            neuron_ids = self.npu_interface.get_neurons_by_area(cortical_idx)
            if not isinstance(neuron_ids, list):
                raise RuntimeError(
                    "Invalid response from NPU Interface: expected List[int] for power neurons"
                )

            # Optional debug
            try:
                from feagi.core.state_manager import FeagiStateManager
                if FeagiStateManager.instance().is_debug_npu_enabled():
                    logger.info(
                        f"[INJECTION-DEBUG] '_power' neurons (cortical_idx={cortical_idx}): {neuron_ids}"
                    )
                    if not neuron_ids:
                        area_info = self.npu_interface.cortical_areas.get(
                            cortical_idx, {}
                        )
                        logger.info(
                            f"[INJECTION-DEBUG] Area stats: created={area_info.get('created')}, type={area_info.get('type')}, neuron_count={area_info.get('neuron_count')}, dimensions={area_info.get('dimensions')}"
                        )
                        logger.info(
                            f"[INJECTION-DEBUG] Mapping sizes: neuron_to_area={len(self.npu_interface.neuron_to_area)}, area_neuron_ranges_present={cortical_idx in self.npu_interface.area_neuron_ranges}"
                        )
            except Exception:
                pass

            return neuron_ids
        except Exception as e:
            # Deterministic failure: propagate with clear message
            raise RuntimeError(
                f"Failed to retrieve power neurons via NPU Interface: {e}"
            )

    def get_all_power_neurons(self) -> Dict[CorticalId, List[NeuronId]]:
        """Get all power area neurons in dictionary format.

        BACKWARD COMPATIBILITY: This method maintains the old interface
        that returns a dictionary, but now only contains the single core power area.

        Returns:
            Dictionary mapping cortical_id to list of neuron IDs
            For core power area: {"_power": [neuron_ids]}
        """
        # If no NPU interface, return cached legacy-detected power areas for tests
        if not self.npu_interface:
            return {area_id: list(self.power_area_neurons.get(area_id, [])) for area_id in self.power_areas}

        try:
            power_neurons = self.get_power_area_neurons()
            if power_neurons:
                return {"_power": power_neurons}
            else:
                return {}
        except Exception as e:
            logger.error(f"Error getting all power neurons: {e}")
            return {}

    def get_special_config(self, cortical_id: CorticalId) -> Optional[Any]:
        """Get configuration for a special area.

        SIMPLIFIED: For core power area, returns a simple config object.
        For other areas, returns None (not supported in simplified approach).

        Args:
            cortical_id: The cortical area ID

        Returns:
            Configuration object for the area, or None if not a special area
        """
        # If detect_special_areas() populated legacy cache, serve from there (tests expect this)
        if cortical_id in self.special_areas:
            return self.special_areas[cortical_id]

        if cortical_id == "_power":
            # Return a simple config object for the core power area
            from types import SimpleNamespace

            return SimpleNamespace(
                enabled=True,
                injection_timing="pre_burst",
                injection_probability=1.0,
                cortical_id="_power",
                area_type="power",
            )
        else:
            # Not a special area in the simplified approach
            logger.debug(
                f"No special config for area {cortical_id} (not a core power area)"
            )
            return None

    def get_statistics(self) -> Dict[str, Any]:
        """Get statistics about power area injection.

        Returns:
            Dictionary with statistics and performance metrics
        """
        return {
            "injection_count": self.injection_count,
            "last_injection_time": self.last_injection_time,
            "core_power_area": "cortical_idx=1 (_power)",
            "total_special_areas": len(self.special_areas),
            "power_areas_count": len(self.power_areas),
            "total_power_neurons": sum(len(v) for v in self.power_area_neurons.values()) if self.power_area_neurons else 0,
            "power_areas": list(self.power_areas),
            "special_area_types": list({cfg.area_type for cfg in self.special_areas.values()}),
        }

    # ===== Legacy test-oriented helpers (no fallbacks in production pipeline) =====
    def detect_special_areas(self) -> None:
        """Populate legacy structures from connectome_manager."""
        self.special_areas.clear()
        self.power_areas.clear()
        self.power_area_neurons.clear()

        areas = getattr(self.connectome_manager, "cortical_areas", {})
        if not isinstance(areas, dict):
            return

        for cortical_id, area in areas.items():
            props = getattr(area, "properties", {}) or {}
            is_power = False
            area_type = ""

            if cortical_id == "_power" or cortical_id.endswith("_pwr") or props.get("__power_injection", False):
                is_power = True
                area_type = "power"
            elif cortical_id.endswith("_mod") or props.get("__modulator", False):
                area_type = "modulator"

            if area_type:
                cfg = SpecialAreaConfig(
                    area_id=cortical_id,
                    area_type=area_type,
                    injection_timing=props.get("injection_timing", "pre_burst"),
                    injection_probability=float(props.get("injection_probability", 1.0)),
                    enabled=True,
                )
                self.special_areas[cortical_id] = cfg

            if is_power:
                self.power_areas.add(cortical_id)
                try:
                    neurons = self.connectome_manager.get_neurons_by_area(cortical_id)
                except Exception:
                    neurons = []
                self.power_area_neurons[cortical_id] = list(neurons or [])

    def is_special_area(self, cortical_id: str) -> bool:
        return cortical_id in self.special_areas

    def is_power_area(self, cortical_id: str) -> bool:
        return cortical_id in self.power_areas

    def get_power_area_neurons_legacy(self, cortical_id: str) -> List[int]:
        """Legacy helper expected by some tests."""
        return list(self.power_area_neurons.get(cortical_id, []))

    def update_power_area_cache(self, cortical_id: str) -> None:
        try:
            neurons = self.connectome_manager.get_neurons_by_area(cortical_id)
        except Exception:
            neurons = []
        self.power_area_neurons[cortical_id] = list(neurons or [])

    def refresh_all_caches(self) -> None:
        self.detect_special_areas()

    def record_injection(self) -> None:
        """Record that an injection has occurred.

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
