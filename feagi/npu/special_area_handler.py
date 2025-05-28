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
from typing import Dict, List, Optional, Set, Any, Union
from dataclasses import dataclass
from collections import defaultdict

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
    target_neurons: Optional[Set[NeuronId]] = None  # Specific neurons to inject, None for all
    enabled: bool = True


class SpecialAreaHandler:
    """
    Handler for special cortical areas with unique neural behaviors.
    
    Special areas are identified by specific naming patterns and have behaviors
    that differ from standard cortical areas. For example:
    - "___pwr": Power areas that inject all their neurons into the FCL on every burst
    - "___mod": Modulator areas that affect other areas' firing patterns
    - "___mem": Enhanced memory areas with extended temporal windows
    
    This handler detects these areas and manages their special processing.
    """
    
    def __init__(self, connectome_manager: Any, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the special area handler.
        
        Args:
            connectome_manager: The connectome manager instance
            config: Optional configuration parameters
        """
        self.connectome_manager = connectome_manager
        self.config = config or {}
        
        # Cache of special areas and their configurations
        self.special_areas: Dict[CorticalId, SpecialAreaConfig] = {}
        
        # Power areas cache for fast lookup during bursts
        self.power_areas: Set[CorticalId] = set()
        self.power_area_neurons: Dict[CorticalId, List[NeuronId]] = {}
        
        # Statistics
        self.injection_count = 0
        self.last_injection_time = 0.0
        
        # Performance optimization: batch injection threshold
        self.batch_threshold = self.config.get('batch_injection_threshold', 100)
        
        logger.info("Special Area Handler initialized", status="[FAST]")
    
    def detect_special_areas(self) -> None:
        """
        Detect and catalog special areas based on naming patterns.
        
        Scans all cortical areas in the connectome and identifies those with
        special naming patterns that require unique processing.
        """
        if not hasattr(self.connectome_manager, 'cortical_areas'):
            logger.warning("No cortical areas available for special area detection")
            return
        
        # Clear existing cache
        self.special_areas.clear()
        self.power_areas.clear()
        self.power_area_neurons.clear()
        
        special_count = 0
        
        for cortical_id, area in self.connectome_manager.cortical_areas.items():
            special_type = self._identify_special_type(cortical_id, area)
            
            if special_type:
                config = self._create_special_config(cortical_id, special_type, area)
                self.special_areas[cortical_id] = config
                
                # Cache power areas for fast access during bursts
                if special_type == "power":
                    self.power_areas.add(cortical_id)
                    # Pre-cache neuron list for performance
                    try:
                        neurons = self.connectome_manager.get_neurons_by_area(cortical_id)
                        self.power_area_neurons[cortical_id] = neurons
                        logger.info(f"Detected power area '{cortical_id}' with {len(neurons)} neurons", status="[FAST]")
                    except Exception as e:
                        logger.error(f"Error getting neurons for power area {cortical_id}: {e}")
                        
                special_count += 1
                logger.debug(f"Detected special area: {cortical_id} (type: {special_type})")
        
        logger.info(f"Detected {special_count} special areas ({len(self.power_areas)} power areas)", status="[SEARCH]")
    
    def _identify_special_type(self, cortical_id: CorticalId, area: Any) -> Optional[str]:
        """
        Identify the special type of a cortical area based on naming patterns.
        
        Args:
            cortical_id: The cortical area ID
            area: The cortical area object
            
        Returns:
            Special type string or None if not special
        """
        # Check naming patterns
        if cortical_id.endswith("_pwr") or cortical_id == "___pwr":
            return "power"
        elif cortical_id.endswith("_mod") or cortical_id == "___mod":
            return "modulator"
        elif cortical_id.endswith("_mem") or cortical_id == "___mem":
            return "enhanced_memory"
        
        # Check area properties for special markers
        if hasattr(area, 'properties') and area.properties:
            if area.properties.get('__power_injection', False):
                return "power"
            if area.properties.get('__modulator', False):
                return "modulator"
        
        return None
    
    def _create_special_config(self, cortical_id: CorticalId, special_type: str, area: Any) -> SpecialAreaConfig:
        """
        Create configuration for a special area.
        
        Args:
            cortical_id: The cortical area ID
            special_type: The type of special behavior
            area: The cortical area object
            
        Returns:
            Configuration object for the special area
        """
        # Default configurations based on type
        if special_type == "power":
            injection_timing = "pre_burst"  # Inject before regular burst processing
            injection_probability = 1.0     # Always inject
        elif special_type == "modulator":
            injection_timing = "during_burst"
            injection_probability = 0.5     # Probabilistic modulation
        else:
            injection_timing = "post_burst"
            injection_probability = 1.0
        
        # Override with area-specific properties if available
        if hasattr(area, 'properties') and area.properties:
            injection_timing = area.properties.get('injection_timing', injection_timing)
            injection_probability = area.properties.get('injection_probability', injection_probability)
        
        return SpecialAreaConfig(
            area_id=cortical_id,
            area_type=special_type,
            injection_timing=injection_timing,
            injection_probability=injection_probability,
            enabled=True
        )
    
    def get_power_areas(self) -> Set[CorticalId]:
        """
        Get all power areas for fast access during burst processing.
        
        Returns:
            Set of cortical IDs that are power areas
        """
        return self.power_areas.copy()
    
    def get_power_area_neurons(self, cortical_id: CorticalId) -> List[NeuronId]:
        """
        Get neurons for a specific power area.
        
        Args:
            cortical_id: The power area ID
            
        Returns:
            List of neuron IDs in the power area
        """
        if cortical_id not in self.power_areas:
            return []
        
        # Use cached neurons if available
        if cortical_id in self.power_area_neurons:
            return self.power_area_neurons[cortical_id].copy()
        
        # Fallback to connectome manager
        try:
            neurons = self.connectome_manager.get_neurons_by_area(cortical_id)
            # Update cache
            self.power_area_neurons[cortical_id] = neurons
            return neurons
        except Exception as e:
            logger.error(f"Error getting neurons for power area {cortical_id}: {e}")
            return []
    
    def get_all_power_neurons(self) -> Dict[CorticalId, List[NeuronId]]:
        """
        Get all neurons from all power areas for batch injection.
        
        Returns:
            Dictionary mapping power area IDs to their neuron lists
        """
        result = {}
        for cortical_id in self.power_areas:
            neurons = self.get_power_area_neurons(cortical_id)
            if neurons:
                result[cortical_id] = neurons
        return result
    
    def is_special_area(self, cortical_id: CorticalId) -> bool:
        """
        Check if a cortical area is a special area.
        
        Args:
            cortical_id: The cortical area ID to check
            
        Returns:
            True if the area is special, False otherwise
        """
        return cortical_id in self.special_areas
    
    def is_power_area(self, cortical_id: CorticalId) -> bool:
        """
        Check if a cortical area is a power area.
        
        Args:
            cortical_id: The cortical area ID to check
            
        Returns:
            True if the area is a power area, False otherwise
        """
        return cortical_id in self.power_areas
    
    def get_special_config(self, cortical_id: CorticalId) -> Optional[SpecialAreaConfig]:
        """
        Get the configuration for a special area.
        
        Args:
            cortical_id: The cortical area ID
            
        Returns:
            Configuration object or None if not special
        """
        return self.special_areas.get(cortical_id)
    
    def update_power_area_cache(self, cortical_id: CorticalId) -> None:
        """
        Update the cached neuron list for a power area.
        
        This should be called when neurons are added/removed from power areas.
        
        Args:
            cortical_id: The power area ID to update
        """
        if cortical_id not in self.power_areas:
            return
        
        try:
            neurons = self.connectome_manager.get_neurons_by_area(cortical_id)
            self.power_area_neurons[cortical_id] = neurons
            logger.debug(f"Updated power area cache for {cortical_id}: {len(neurons)} neurons")
        except Exception as e:
            logger.error(f"Error updating power area cache for {cortical_id}: {e}")
    
    def refresh_all_caches(self) -> None:
        """
        Refresh all cached data for special areas.
        
        This should be called when the connectome structure changes significantly.
        """
        logger.info("Refreshing special area caches", status="[PROC]")
        self.detect_special_areas()
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about special area processing.
        
        Returns:
            Dictionary with statistics and performance metrics
        """
        return {
            "total_special_areas": len(self.special_areas),
            "power_areas_count": len(self.power_areas),
            "total_power_neurons": sum(len(neurons) for neurons in self.power_area_neurons.values()),
            "injection_count": self.injection_count,
            "last_injection_time": self.last_injection_time,
            "power_areas": list(self.power_areas),
            "special_area_types": [config.area_type for config in self.special_areas.values()]
        }
    
    def record_injection(self) -> None:
        """Record that an injection occurred for statistics."""
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