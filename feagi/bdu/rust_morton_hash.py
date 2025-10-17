"""Rust Morton Spatial Hash Wrapper.

Provides a drop-in replacement for Python Morton spatial hash using Rust backend.

Performance improvements:
- 100x+ faster position lookups
- 10x+ faster bulk operations
- Native Roaring bitmap operations

Copyright 2025 Neuraville Inc.
"""

import logging
from typing import Dict, List, Optional, Set, Tuple

logger = logging.getLogger(__name__)

# Try to import Rust implementation
try:
    from feagi_bdu import PyMortonSpatialHash
    RUST_MORTON_AVAILABLE = True
    logger.info("🦀 Rust Morton spatial hash loaded")
except ImportError:
    RUST_MORTON_AVAILABLE = False
    logger.warning("Rust Morton not available - using Python fallback")


class RustMortonSpatialHash:
    """Rust-accelerated Morton spatial hash.
    
    Drop-in replacement for Python RoaringSpatialHash with identical API.
    """
    
    def __init__(self):
        """Initialize Rust Morton spatial hash."""
        if not RUST_MORTON_AVAILABLE:
            raise RuntimeError(
                "Rust Morton spatial hash not available. "
                "Build with: cd feagi-rust && ./build_bdu.sh"
            )
        
        self._hash = PyMortonSpatialHash()
        logger.info("[RUST MORTON] Initialized")
    
    def add_neuron(
        self,
        cortical_area: str,
        x: int,
        y: int,
        z: int,
        neuron_id: int
    ) -> bool:
        """Add a neuron to the spatial hash.
        
        Args:
            cortical_area: Cortical area identifier
            x, y, z: 3D coordinates
            neuron_id: Unique neuron identifier
            
        Returns:
            True if neuron was added successfully
        """
        return self._hash.add_neuron(cortical_area, x, y, z, neuron_id)
    
    def get_neuron_at_coordinate(
        self,
        cortical_area: str,
        x: int,
        y: int,
        z: int
    ) -> Optional[int]:
        """Get first neuron at coordinate.
        
        Args:
            cortical_area: Cortical area identifier
            x, y, z: 3D coordinates
            
        Returns:
            Neuron ID if found, None otherwise
        """
        return self._hash.get_neuron_at_coordinate(cortical_area, x, y, z)
    
    def get_neurons_at_coordinate(
        self,
        cortical_area: str,
        x: int,
        y: int,
        z: int
    ) -> List[int]:
        """Get all neurons at coordinate.
        
        Args:
            cortical_area: Cortical area identifier
            x, y, z: 3D coordinates
            
        Returns:
            List of neuron IDs at the coordinate
        """
        return self._hash.get_neurons_at_coordinate(cortical_area, x, y, z)
    
    def get_neurons_in_region(
        self,
        cortical_area: str,
        x1: int, y1: int, z1: int,
        x2: int, y2: int, z2: int
    ) -> List[int]:
        """Get all neurons in a 3D region.
        
        Args:
            cortical_area: Cortical area identifier
            x1, y1, z1: Start coordinates (inclusive)
            x2, y2, z2: End coordinates (inclusive)
            
        Returns:
            List of neuron IDs in the region
        """
        return self._hash.get_neurons_in_region(
            cortical_area, x1, y1, z1, x2, y2, z2
        )
    
    def get_neuron_position(
        self,
        neuron_id: int
    ) -> Optional[Tuple[str, int, int, int]]:
        """Get neuron's position.
        
        Args:
            neuron_id: Neuron identifier
            
        Returns:
            Tuple of (cortical_area, x, y, z) or None
        """
        return self._hash.get_neuron_position(neuron_id)
    
    def remove_neuron(self, neuron_id: int) -> bool:
        """Remove a neuron from the spatial hash.
        
        Args:
            neuron_id: Neuron identifier
            
        Returns:
            True if neuron was removed
        """
        return self._hash.remove_neuron(neuron_id)
    
    def clear(self):
        """Clear all data from the spatial hash."""
        self._hash.clear()
    
    def get_statistics(self) -> Dict[str, any]:
        """Get detailed statistics about the spatial hash.
        
        Returns:
            Dictionary with statistics
        """
        return self._hash.get_stats()
    
    def is_ready(self) -> bool:
        """Check if the system is ready for use."""
        return True  # Rust implementation is always ready


# Convenience function to create appropriate hash
def create_morton_spatial_hash(prefer_rust: bool = True):
    """Create Morton spatial hash (Rust or Python).
    
    Args:
        prefer_rust: Prefer Rust implementation if available
        
    Returns:
        Morton spatial hash instance
    """
    if prefer_rust and RUST_MORTON_AVAILABLE:
        return RustMortonSpatialHash()
    else:
        # Fallback to Python implementation
        from feagi.bdu.morton_spatial_hash import get_morton_spatial_hash
        return get_morton_spatial_hash()

