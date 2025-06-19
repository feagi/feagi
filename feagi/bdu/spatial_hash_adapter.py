"""
Spatial Hash Adapter for Morton Encoding Backend

This module provides backward compatibility for the existing GlobalSpatialHash
interface while using the new Morton encoding + Roaring bitmap implementation
underneath. This allows for seamless migration without breaking existing code.

Copyright 2025 Neuraville Inc.
Licensed under the Apache License, Version 2.0
"""

import logging
from typing import Dict, List, Tuple, Optional, Any, Set
from pathlib import Path
import threading
import hashlib

from .morton_spatial_hash import (
    RoaringSpatialHash, 
    MortonSpatialHashState, 
    get_morton_spatial_hash,
    reset_morton_spatial_hash
)

logger = logging.getLogger(__name__)


class SpatialHashConfig:
    """Configuration class for spatial hash (backward compatibility)."""
    
    def __init__(self, max_dimension: int = 256, genome_based_sizing: bool = True, 
                 enable_simd: bool = True, hash_prime: int = 73856093, 
                 cache_size: int = 1000000, enable_caching: bool = True):
        self.max_dimension = max_dimension
        self.genome_based_sizing = genome_based_sizing
        self.enable_simd = enable_simd  # Legacy parameter - ignored in Morton backend
        self.hash_prime = hash_prime    # Legacy parameter - ignored in Morton backend  
        self.cache_size = cache_size    # Legacy parameter - ignored in Morton backend
        self.enable_caching = enable_caching
        self.cache_dir = Path("cache/spatial_hash")


class SpatialHashState:
    """State enumeration for backward compatibility."""
    UNINITIALIZED = "uninitialized"
    ANALYZING_GENOME = "analyzing_genome"
    BUILDING_CACHE = "building_cache"
    LOADING_CACHE = "loading_cache"
    READY = "ready"
    EXPANDING_CACHE = "expanding_cache"
    ERROR = "error"


class GlobalSpatialHashAdapter:
    """
    Adapter class that provides the old GlobalSpatialHash interface
    while using the new Morton encoding + Roaring bitmap backend.
    
    This ensures backward compatibility during the migration.
    """
    
    def __init__(self, config: Optional[SpatialHashConfig] = None):
        """Initialize the adapter with the new Morton backend."""
        self.config = config or SpatialHashConfig()
        self._morton_hash = get_morton_spatial_hash()
        self._lock = threading.RLock()
        
        # Legacy state tracking
        self._current_max_dims = (1, 1, 1)
        self._coordinate_count = 0
        self._hash_collisions = 0
        
        logger.info("[SPATIAL HASH ADAPTER] Initialized with Morton encoding backend")
    
    def _map_state(self, morton_state: MortonSpatialHashState) -> str:
        """Map Morton spatial hash state to legacy state."""
        mapping = {
            MortonSpatialHashState.UNINITIALIZED: SpatialHashState.UNINITIALIZED,
            MortonSpatialHashState.BUILDING: SpatialHashState.BUILDING_CACHE,
            MortonSpatialHashState.READY: SpatialHashState.READY,
            MortonSpatialHashState.EXPANDING: SpatialHashState.EXPANDING_CACHE,
            MortonSpatialHashState.ERROR: SpatialHashState.ERROR,
        }
        return mapping.get(morton_state, SpatialHashState.UNINITIALIZED)
    
    def get_state(self) -> str:
        """Get current spatial hash state (legacy interface)."""
        morton_state = self._morton_hash.get_state()
        return self._map_state(morton_state)
    
    def is_ready(self) -> bool:
        """Check if spatial hash is ready (legacy interface)."""
        return self._morton_hash.is_ready()
    
    def wait_for_ready(self, timeout_seconds: float = 60.0) -> bool:
        """
        Wait for spatial hash to become ready (legacy interface).
        
        Args:
            timeout_seconds: Maximum time to wait
            
        Returns:
            True if ready within timeout, False otherwise
        """
        import time
        start_time = time.time()
        
        while time.time() - start_time < timeout_seconds:
            if self.is_ready():
                return True
            time.sleep(0.1)
        
        return False
    
    def initialize_for_dimensions(self, max_dims: Tuple[int, int, int]) -> None:
        """
        Initialize spatial hash for specific dimensions (legacy interface).
        
        Args:
            max_dims: Maximum dimensions (compatibility only - not used in Morton)
        """
        self._current_max_dims = max_dims
        self._morton_hash._set_state(MortonSpatialHashState.READY)
        
        logger.info(f"[SPATIAL HASH ADAPTER] Initialized for dimensions: {max_dims}")
        logger.info(f"[SPATIAL HASH ADAPTER] Using Morton encoding (dimensions are advisory only)")
    
    def add_coordinate(self, cortical_area: str, x: int, y: int, z: int, neuron_id: int) -> bool:
        """
        Add a coordinate to the spatial hash (legacy interface).
        
        Args:
            cortical_area: Cortical area identifier
            x, y, z: 3D coordinates
            neuron_id: Neuron identifier
            
        Returns:
            True if added successfully
        """
        success = self._morton_hash.add_neuron(cortical_area, x, y, z, neuron_id)
        if success:
            self._coordinate_count += 1
        return success
    
    def get_neuron_id(self, cortical_area: str, x: int, y: int, z: int) -> Optional[int]:
        """
        Get neuron ID at coordinate (legacy interface).
        
        Args:
            cortical_area: Cortical area identifier
            x, y, z: 3D coordinates
            
        Returns:
            Neuron ID if found, None otherwise
        """
        return self._morton_hash.get_neuron_at_coordinate(cortical_area, x, y, z)
    
    def expand_cache_for_new_area(self, position: Tuple[int, int, int], 
                                 dimensions: Tuple[int, int, int]) -> bool:
        """
        Expand cache for new cortical area (legacy interface).
        
        Note: With Morton encoding, this is essentially a no-op since
        the system automatically handles any coordinate range.
        
        Args:
            position: Position of new area (ignored in Morton)
            dimensions: Dimensions of new area (ignored in Morton)
            
        Returns:
            Always True (Morton encoding handles expansion automatically)
        """
        logger.info(f"[SPATIAL HASH ADAPTER] Expanding cache for area: pos={position}, dims={dimensions}")
        logger.info(f"[SPATIAL HASH ADAPTER] Morton encoding handles expansion automatically")
        return True
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get spatial hash statistics (legacy interface)."""
        morton_stats = self._morton_hash.get_statistics()
        
        # Convert to legacy format
        legacy_stats = {
            "state": self.get_state(),
            "coordinate_count": morton_stats["total_coordinates"],
            "hash_collisions": 0,  # Morton encoding doesn't have hash collisions
            "current_dimensions": self._current_max_dims,
            "memory_efficient": True,
            "backend": "morton_roaring_bitmap",
            "cortical_areas": morton_stats["cortical_areas"]
        }
        
        return legacy_stats
    
    def clear(self):
        """Clear all spatial hash data (legacy interface)."""
        self._morton_hash.clear()
        self._coordinate_count = 0
        self._hash_collisions = 0
        logger.info("[SPATIAL HASH ADAPTER] Cleared all data")
    
    def save_cache(self, cache_key: str) -> bool:
        """Save cache to file (legacy interface)."""
        return self._morton_hash.save_to_cache(cache_key)
    
    def load_cache(self, cache_key: str) -> bool:
        """Load cache from file (legacy interface)."""
        return self._morton_hash.load_from_cache(cache_key)
    
    def batch_coordinate_lookup(self, candidate_positions: Set[Tuple[int, int, int]], 
                               neuron_positions: List[Tuple[int, int, int]]) -> List[Tuple[int, int]]:
        """
        Fast batch lookup for matching candidate positions to neuron positions.
        
        Args:
            candidate_positions: Set of (x, y, z) coordinates to search for
            neuron_positions: List of (x, y, z) coordinates where neurons exist
            
        Returns:
            List of (candidate_idx, neuron_idx) pairs for matching coordinates
        """
        try:
            # Convert to lists for indexing
            candidate_list = list(candidate_positions)
            matches = []
            
            # Create fast lookup from position to neuron index
            pos_to_neuron_idx = {}
            for neuron_idx, pos in enumerate(neuron_positions):
                if len(pos) >= 3:  # Ensure we have x, y, z
                    coord = (int(pos[0]), int(pos[1]), int(pos[2]))
                    pos_to_neuron_idx[coord] = neuron_idx
            
            # Find matches
            for candidate_idx, candidate_pos in enumerate(candidate_list):
                if len(candidate_pos) >= 3:  # Ensure we have x, y, z
                    coord = (int(candidate_pos[0]), int(candidate_pos[1]), int(candidate_pos[2]))
                    if coord in pos_to_neuron_idx:
                        neuron_idx = pos_to_neuron_idx[coord]
                        matches.append((candidate_idx, neuron_idx))
            
            return matches
            
        except Exception as e:
            logger.warning(f"[SPATIAL HASH ADAPTER] Error in batch_coordinate_lookup: {e}")
            return []


# Global instance for backward compatibility
_global_spatial_hash_instance: Optional[GlobalSpatialHashAdapter] = None
_global_instance_lock = threading.Lock()


class GlobalSpatialHash:
    """
    Global singleton spatial hash (legacy interface).
    
    This maintains the exact same interface as the original GlobalSpatialHash
    but uses the new Morton encoding + Roaring bitmap backend.
    """
    
    _instance: Optional[GlobalSpatialHashAdapter] = None
    _lock = threading.Lock()
    
    def __new__(cls):
        """Singleton pattern implementation."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = GlobalSpatialHashAdapter()
            return cls._instance
    
    def __init__(self):
        """Initialize (singleton pattern handles actual initialization)."""
        pass
    
    def __getattr__(self, name):
        """Delegate all attribute access to the adapter instance."""
        if self._instance is None:
            with self._lock:
                if self._instance is None:
                    self._instance = GlobalSpatialHashAdapter()
        return getattr(self._instance, name)
    
    @classmethod
    def reset_instance(cls):
        """Reset the global instance (for testing)."""
        with cls._lock:
            cls._instance = None
        reset_morton_spatial_hash()


def analyze_genome_coordinate_space(genome_data: Dict[str, Any]) -> Tuple[int, int, int]:
    """
    Analyze genome coordinate space (legacy interface).
    
    Note: With Morton encoding, this analysis is primarily for logging purposes
    since Morton encoding can handle any coordinate range efficiently.
    
    Args:
        genome_data: Genome data to analyze
        
    Returns:
        Tuple of maximum dimensions (for compatibility)
    """
    logger.info("[SPATIAL HASH ADAPTER] Analyzing genome coordinate space for Morton encoding")
    
    max_dimensions = [1, 1, 1]
    areas_analyzed = 0
    
    try:
        blueprint = genome_data.get("blueprint", {})
        
        if not blueprint:
            logger.warning("[SPATIAL HASH ADAPTER] No blueprint found in genome data")
            return tuple(max_dimensions)
        
        # Parse FEAGI 2.0 flat format
        cortical_areas = {}
        
        for key, value in blueprint.items():
            if key.endswith("-cx-___bbx-i"):
                area_id = key.split("-")[1]
                if area_id not in cortical_areas:
                    cortical_areas[area_id] = {}
                cortical_areas[area_id]["x"] = value
            elif key.endswith("-cx-___bby-i"):
                area_id = key.split("-")[1]
                if area_id not in cortical_areas:
                    cortical_areas[area_id] = {}
                cortical_areas[area_id]["y"] = value
            elif key.endswith("-cx-___bbz-i"):
                area_id = key.split("-")[1]
                if area_id not in cortical_areas:
                    cortical_areas[area_id] = {}
                cortical_areas[area_id]["z"] = value
        
        # Find maximum dimensions
        for area_id, dims in cortical_areas.items():
            if "x" in dims and "y" in dims and "z" in dims:
                max_dimensions[0] = max(max_dimensions[0], dims["x"])
                max_dimensions[1] = max(max_dimensions[1], dims["y"])
                max_dimensions[2] = max(max_dimensions[2], dims["z"])
                areas_analyzed += 1
        
        # Apply buffer factor (for compatibility)
        buffer_factor = 1.1
        max_dimensions = [int(dim * buffer_factor) for dim in max_dimensions]
        
        logger.info(f"[SPATIAL HASH ADAPTER] Analyzed {areas_analyzed} cortical areas")
        logger.info(f"[SPATIAL HASH ADAPTER] Max dimensions: {max_dimensions[0]}x{max_dimensions[1]}x{max_dimensions[2]}")
        logger.info(f"[SPATIAL HASH ADAPTER] Morton encoding will handle these coordinates efficiently")
        
        return tuple(max_dimensions)
        
    except Exception as e:
        logger.error(f"[SPATIAL HASH ADAPTER] Error analyzing genome: {e}")
        return tuple(max_dimensions) 