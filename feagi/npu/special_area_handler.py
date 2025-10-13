"""
Special Area Handler for FEAGI NPU - Clean Architecture

Simplified special area handling that integrates with the clean FCL/Fire Queue architecture.
Provides type aliases and basic special area detection for compatibility.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

# Type aliases for compatibility
CorticalId = str
NeuronId = int


@dataclass
class SpecialAreaConfig:
    """Configuration for a special cortical area."""
    area_id: CorticalId
    area_type: str  # "power", "modulator", etc.
    injection_probability: float = 1.0
    target_neurons: Optional[Set[NeuronId]] = None
    enabled: bool = True


class SpecialAreaHandler:
    """Simplified special area handler for clean NPU architecture."""
    
    def __init__(self, connectome_manager=None):
        """Initialize special area handler."""
        self.connectome_manager = connectome_manager
        self.special_areas: Dict[CorticalId, SpecialAreaConfig] = {}
        
        # Initialize power area neuron cache
        self._power_neurons_cache = []
        self._cache_valid = False
        self._injection_count = 0
        self._last_injection_time = None
        
        # Special Area Handler initialized
    
    def get_power_area_neurons(self) -> List[int]:
        """Get neurons from power areas (compatibility method)."""
        if self._cache_valid and self._power_neurons_cache:
            return self._power_neurons_cache
            
        power_neurons = []
        
        try:
            if self.connectome_manager:
                # Get cortical area 1 (power area) neurons
                cortical_areas = getattr(self.connectome_manager, 'cortical_areas', {})
                if 1 in cortical_areas or '_power' in cortical_areas:
                    power_area = cortical_areas.get(1) or cortical_areas.get('_power')
                    if power_area:
                        # Get neurons from power area
                        if hasattr(power_area, 'neurons'):
                            power_neurons.extend(power_area.neurons)
                        elif hasattr(self.connectome_manager, 'get_area_neurons'):
                            power_neurons = self.connectome_manager.get_area_neurons(1) or []
                            
            # Cache the result
            self._power_neurons_cache = power_neurons
            self._cache_valid = True
            
        except Exception as e:
            logger.debug(f"Error getting power area neurons: {e}")
            
        return power_neurons
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get special area handler statistics (compatibility method)."""
        return {
            'special_areas_count': len(self.special_areas),
            'power_neurons_cached': len(self._power_neurons_cache),
            'cache_valid': self._cache_valid,
            'injection_count': self._injection_count,
            'last_injection_time': self._last_injection_time,
            'core_power_area': "cortical_idx=1 (_power)"
        }
    
    def record_injection(self, area_id: str = "default", neuron_count: int = 1) -> None:
        """Record injection statistics (compatibility method)."""
        import time
        self._injection_count += 1
        self._last_injection_time = time.time()
    
    def register_special_area(self, config: SpecialAreaConfig):
        """Register a special cortical area."""
        self.special_areas[config.area_id] = config
        # Special area registered
    
    def get_special_areas_by_type(self, area_type: str) -> List[SpecialAreaConfig]:
        """Get all special areas of a specific type."""
        return [
            config for config in self.special_areas.values()
            if config.area_type == area_type and config.enabled
        ]
    
    def get_power_areas(self) -> List[SpecialAreaConfig]:
        """Get all power areas."""
        return self.get_special_areas_by_type("power")
    
    def is_special_area(self, area_id: CorticalId) -> bool:
        """Check if an area is a special area."""
        return area_id in self.special_areas
    
    def get_area_config(self, area_id: CorticalId) -> Optional[SpecialAreaConfig]:
        """Get configuration for a special area."""
        return self.special_areas.get(area_id)
