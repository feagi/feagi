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
        
        # Special Area Handler initialized
    
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
