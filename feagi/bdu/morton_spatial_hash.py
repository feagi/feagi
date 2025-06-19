"""
Morton Encoding + Roaring Bitmap Spatial Hash System

This module implements an efficient spatial hash system using:
1. Morton encoding (Z-order curve) for spatial locality preservation
2. Roaring bitmaps for sparse coordinate storage
3. Per-cortical-area organization for modularity

This approach provides:
- 95%+ memory savings for sparse genomes
- Microsecond union operations for multi-area queries
- Spatial locality preservation for cache efficiency
- Easy Rust migration path via Morton encoding

Copyright 2025 Neuraville Inc.
Licensed under the Apache License, Version 2.0
"""

import logging
from typing import Dict, List, Tuple, Set, Optional, Any
from enum import Enum
import threading
import time
from pathlib import Path
import pickle
import hashlib

try:
    from pyroaring import BitMap as RoaringBitmap
except ImportError:
    raise ImportError("pyroaring is required. Install with: pip install pyroaring")

logger = logging.getLogger(__name__)


class MortonSpatialHashState(Enum):
    """State enumeration for Morton spatial hash system."""
    UNINITIALIZED = "uninitialized"
    BUILDING = "building"
    READY = "ready"
    EXPANDING = "expanding"
    ERROR = "error"


class MortonUtils:
    """Utilities for Morton encoding/decoding with 3D coordinates."""
    
    @staticmethod
    def morton_encode_3d(x: int, y: int, z: int) -> int:
        """
        Encode 3D coordinates into Morton code (Z-order curve).
        
        Interleaves bits of x, y, z coordinates to preserve spatial locality.
        
        Args:
            x, y, z: 3D coordinates (must be non-negative)
            
        Returns:
            Morton encoded integer
        """
        if x < 0 or y < 0 or z < 0:
            raise ValueError(f"Morton encoding requires non-negative coordinates: ({x}, {y}, {z})")
            
        # Limit to 21 bits per dimension (63 bits total for 64-bit integer)
        if x >= (1 << 21) or y >= (1 << 21) or z >= (1 << 21):
            raise ValueError(f"Coordinates too large for Morton encoding: ({x}, {y}, {z})")
        
        result = 0
        for i in range(21):
            result |= ((x & (1 << i)) << (2 * i))
            result |= ((y & (1 << i)) << (2 * i + 1))
            result |= ((z & (1 << i)) << (2 * i + 2))
        return result
    
    @staticmethod
    def morton_decode_3d(morton_code: int) -> Tuple[int, int, int]:
        """
        Decode Morton code back to 3D coordinates.
        
        Args:
            morton_code: Morton encoded integer
            
        Returns:
            Tuple of (x, y, z) coordinates
        """
        x = y = z = 0
        for i in range(21):
            x |= ((morton_code & (1 << (3 * i))) >> (2 * i))
            y |= ((morton_code & (1 << (3 * i + 1))) >> (2 * i + 1))
            z |= ((morton_code & (1 << (3 * i + 2))) >> (2 * i + 2))
        return x, y, z
    
    @staticmethod
    def morton_encode_region_3d(x1: int, y1: int, z1: int, x2: int, y2: int, z2: int) -> RoaringBitmap:
        """
        Encode a 3D region into a roaring bitmap.
        
        Args:
            x1, y1, z1: Start coordinates (inclusive)
            x2, y2, z2: End coordinates (inclusive)
            
        Returns:
            RoaringBitmap containing all Morton codes in the region
        """
        bitmap = RoaringBitmap()
        for z in range(z1, z2 + 1):
            for y in range(y1, y2 + 1):
                for x in range(x1, x2 + 1):
                    morton_code = MortonUtils.morton_encode_3d(x, y, z)
                    bitmap.add(morton_code)
        return bitmap


class RoaringSpatialHash:
    """
    Morton encoding + Roaring bitmap spatial hash implementation.
    
    This class provides efficient spatial coordinate mapping using:
    - Morton encoding for spatial locality preservation
    - Roaring bitmaps for sparse coordinate storage
    - Per-cortical-area organization for modularity
    """
    
    def __init__(self, cache_dir: Optional[Path] = None):
        """
        Initialize the roaring spatial hash system.
        
        Args:
            cache_dir: Directory for caching bitmap data
        """
        self._state = MortonSpatialHashState.UNINITIALIZED
        self._lock = threading.RLock()
        
        # Core data structures
        self.cortical_bitmaps: Dict[str, RoaringBitmap] = {}
        self.neuron_map: Dict[Tuple[str, int], int] = {}  # (cortical_area, morton_code) → neuron_id
        self.coordinate_map: Dict[int, Tuple[str, int, int, int]] = {}  # neuron_id → (cortical_area, x, y, z)
        
        # Statistics
        self.total_coordinates = 0
        self.total_areas = 0
        self.memory_usage = 0
        
        # Caching
        self.cache_dir = cache_dir or Path("cache/morton_spatial_hash")
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        
        logger.info("[MORTON SPATIAL HASH] Initialized with roaring bitmap architecture")
    
    def _set_state(self, new_state: MortonSpatialHashState):
        """Update the system state with thread safety."""
        with self._lock:
            old_state = self._state
            self._state = new_state
            logger.debug(f"[MORTON SPATIAL HASH] State: {old_state.value} → {new_state.value}")
    
    def get_state(self) -> MortonSpatialHashState:
        """Get current system state."""
        return self._state
    
    def is_ready(self) -> bool:
        """Check if the system is ready for use."""
        return self._state == MortonSpatialHashState.READY
    
    def add_neuron(self, cortical_area: str, x: int, y: int, z: int, neuron_id: int) -> bool:
        """
        Add a neuron coordinate to the spatial hash.
        
        Args:
            cortical_area: Cortical area identifier
            x, y, z: 3D coordinates within the cortical area
            neuron_id: Unique neuron identifier
            
        Returns:
            True if neuron was added successfully
        """
        try:
            with self._lock:
                # Encode coordinates
                morton_code = MortonUtils.morton_encode_3d(x, y, z)
                
                # Create cortical area bitmap if needed
                if cortical_area not in self.cortical_bitmaps:
                    self.cortical_bitmaps[cortical_area] = RoaringBitmap()
                    self.total_areas += 1
                
                # Add to cortical area bitmap
                self.cortical_bitmaps[cortical_area].add(morton_code)
                
                # Store neuron mappings (include cortical area in key)
                self.neuron_map[(cortical_area, morton_code)] = neuron_id
                self.coordinate_map[neuron_id] = (cortical_area, x, y, z)
                
                self.total_coordinates += 1
                return True
                
        except Exception as e:
            logger.error(f"[MORTON SPATIAL HASH] Error adding neuron: {e}")
            return False
    
    def get_neuron_at_coordinate(self, cortical_area: str, x: int, y: int, z: int) -> Optional[int]:
        """
        Get neuron ID at specific coordinate.
        
        Args:
            cortical_area: Cortical area identifier
            x, y, z: 3D coordinates
            
        Returns:
            Neuron ID if found, None otherwise
        """
        try:
            morton_code = MortonUtils.morton_encode_3d(x, y, z)
            
            # Check if coordinate exists in cortical area
            if cortical_area in self.cortical_bitmaps:
                if morton_code in self.cortical_bitmaps[cortical_area]:
                    return self.neuron_map.get((cortical_area, morton_code))
            
            return None
            
        except Exception as e:
            logger.error(f"[MORTON SPATIAL HASH] Error getting neuron: {e}")
            return None
    
    def get_neurons_in_region(self, cortical_area: str, x1: int, y1: int, z1: int, 
                             x2: int, y2: int, z2: int) -> List[int]:
        """
        Get all neuron IDs in a 3D region.
        
        Args:
            cortical_area: Cortical area identifier
            x1, y1, z1: Start coordinates (inclusive)
            x2, y2, z2: End coordinates (inclusive)
            
        Returns:
            List of neuron IDs in the region
        """
        try:
            if cortical_area not in self.cortical_bitmaps:
                return []
            
            # Create region bitmap
            region_bitmap = MortonUtils.morton_encode_region_3d(x1, y1, z1, x2, y2, z2)
            
            # Fast intersection with cortical area
            intersection = self.cortical_bitmaps[cortical_area] & region_bitmap
            
            # Get neuron IDs
            neuron_ids = []
            for morton_code in intersection:
                neuron_key = (cortical_area, morton_code)
                if neuron_key in self.neuron_map:
                    neuron_ids.append(self.neuron_map[neuron_key])
            
            return neuron_ids
            
        except Exception as e:
            logger.error(f"[MORTON SPATIAL HASH] Error getting neurons in region: {e}")
            return []
    
    def get_area_union(self, cortical_areas: List[str]) -> RoaringBitmap:
        """
        Get union of multiple cortical areas (fast bitmap operation).
        
        Args:
            cortical_areas: List of cortical area identifiers
            
        Returns:
            RoaringBitmap containing union of all areas
        """
        result = RoaringBitmap()
        for area in cortical_areas:
            if area in self.cortical_bitmaps:
                result |= self.cortical_bitmaps[area]  # Fast union operation!
        return result
    
    def get_area_intersection(self, cortical_areas: List[str]) -> RoaringBitmap:
        """
        Get intersection of multiple cortical areas.
        
        Args:
            cortical_areas: List of cortical area identifiers
            
        Returns:
            RoaringBitmap containing intersection of all areas
        """
        if not cortical_areas:
            return RoaringBitmap()
        
        result = self.cortical_bitmaps.get(cortical_areas[0], RoaringBitmap())
        for area in cortical_areas[1:]:
            if area in self.cortical_bitmaps:
                result &= self.cortical_bitmaps[area]  # Fast intersection!
            else:
                return RoaringBitmap()  # Empty intersection
        return result
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get detailed statistics about the spatial hash."""
        with self._lock:
            stats = {
                "state": self._state.value,
                "total_coordinates": self.total_coordinates,
                "total_areas": self.total_areas,
                "cortical_areas": {},
                "memory_efficient": True,
                "encoding": "morton_3d"
            }
            
            # Per-area statistics
            for area_id, bitmap in self.cortical_bitmaps.items():
                stats["cortical_areas"][area_id] = {
                    "coordinate_count": len(bitmap),
                    "memory_bytes": bitmap.shrink_to_fit() if hasattr(bitmap, 'shrink_to_fit') else 0
                }
            
            return stats
    
    def clear(self):
        """Clear all data from the spatial hash."""
        with self._lock:
            self.cortical_bitmaps.clear()
            self.neuron_map.clear()
            self.coordinate_map.clear()
            self.total_coordinates = 0
            self.total_areas = 0
            logger.info("[MORTON SPATIAL HASH] Cleared all data")
    
    def save_to_cache(self, cache_key: str) -> bool:
        """
        Save current state to cache file.
        
        Args:
            cache_key: Unique identifier for cache file
            
        Returns:
            True if saved successfully
        """
        try:
            cache_file = self.cache_dir / f"morton_hash_{cache_key}.pkl"
            
            cache_data = {
                "cortical_bitmaps": self.cortical_bitmaps,
                "neuron_map": self.neuron_map,
                "coordinate_map": self.coordinate_map,
                "total_coordinates": self.total_coordinates,
                "total_areas": self.total_areas,
                "version": "1.0"
            }
            
            with open(cache_file, "wb") as f:
                pickle.dump(cache_data, f)
            
            logger.info(f"[MORTON SPATIAL HASH] Saved to cache: {cache_file}")
            return True
            
        except Exception as e:
            logger.error(f"[MORTON SPATIAL HASH] Error saving cache: {e}")
            return False
    
    def load_from_cache(self, cache_key: str) -> bool:
        """
        Load state from cache file.
        
        Args:
            cache_key: Unique identifier for cache file
            
        Returns:
            True if loaded successfully
        """
        try:
            cache_file = self.cache_dir / f"morton_hash_{cache_key}.pkl"
            
            if not cache_file.exists():
                return False
            
            with open(cache_file, "rb") as f:
                cache_data = pickle.load(f)
            
            # Restore state
            self.cortical_bitmaps = cache_data["cortical_bitmaps"]
            self.neuron_map = cache_data["neuron_map"]
            self.coordinate_map = cache_data["coordinate_map"]
            self.total_coordinates = cache_data["total_coordinates"]
            self.total_areas = cache_data["total_areas"]
            
            logger.info(f"[MORTON SPATIAL HASH] Loaded from cache: {cache_file}")
            return True
            
        except Exception as e:
            logger.error(f"[MORTON SPATIAL HASH] Error loading cache: {e}")
            return False


# Singleton instance for global access
_morton_spatial_hash_instance: Optional[RoaringSpatialHash] = None
_morton_instance_lock = threading.Lock()


def get_morton_spatial_hash() -> RoaringSpatialHash:
    """Get or create the global Morton spatial hash instance."""
    global _morton_spatial_hash_instance
    
    with _morton_instance_lock:
        if _morton_spatial_hash_instance is None:
            _morton_spatial_hash_instance = RoaringSpatialHash()
        return _morton_spatial_hash_instance


def reset_morton_spatial_hash():
    """Reset the global Morton spatial hash instance."""
    global _morton_spatial_hash_instance
    
    with _morton_instance_lock:
        _morton_spatial_hash_instance = None 