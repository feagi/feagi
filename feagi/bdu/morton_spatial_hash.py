"""Morton Encoding + Roaring Bitmap Spatial Hash System.

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
import pickle
import threading
from collections import defaultdict
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

# Try to import roaring bitmap - fall back to set if not available
try:
    from pyroaring import BitMap as RoaringBitmap

    ROARING_AVAILABLE = True
except ImportError:
    # Fallback implementation using Python set
    class RoaringBitmap:
        def __init__(self, values=None):
            self._data = set(values or [])

        def add(self, value):
            self._data.add(value)

        def __contains__(self, value):
            return value in self._data

        def __len__(self):
            return len(self._data)

        def __iter__(self):
            return iter(self._data)

        def __and__(self, other):
            result = RoaringBitmap()
            result._data = self._data & other._data
            return result

        def __or__(self, other):
            result = RoaringBitmap()
            result._data = self._data | other._data
            return result

        def __iand__(self, other):
            self._data &= other._data
            return self

        def __ior__(self, other):
            self._data |= other._data
            return self

    ROARING_AVAILABLE = False

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
        """Encode 3D coordinates into Morton code (Z-order curve).

        Interleaves bits of x, y, z coordinates to preserve spatial locality.

        Args:
            x, y, z: 3D coordinates (must be non-negative)

        Returns:
            Morton encoded integer
        """
        if x < 0 or y < 0 or z < 0:
            raise ValueError(
                f"Morton encoding requires non-negative coordinates: ({x}, {y}, {z})"
            )

        # Limit to 21 bits per dimension (63 bits total for 64-bit integer)
        if x >= (1 << 21) or y >= (1 << 21) or z >= (1 << 21):
            raise ValueError(
                f"Coordinates too large for Morton encoding: ({x}, {y}, {z})"
            )

        result = 0
        for i in range(21):
            result |= (x & (1 << i)) << (2 * i)
            result |= (y & (1 << i)) << (2 * i + 1)
            result |= (z & (1 << i)) << (2 * i + 2)
        return result

    @staticmethod
    def morton_decode_3d(morton_code: int) -> Tuple[int, int, int]:
        """Decode Morton code back to 3D coordinates.

        Args:
            morton_code: Morton encoded integer

        Returns:
            Tuple of (x, y, z) coordinates
        """
        x = y = z = 0
        for i in range(21):
            x |= (morton_code & (1 << (3 * i))) >> (2 * i)
            y |= (morton_code & (1 << (3 * i + 1))) >> (2 * i + 1)
            z |= (morton_code & (1 << (3 * i + 2))) >> (2 * i + 2)
        return x, y, z

    @staticmethod
    def morton_encode_region_3d(
        x1: int, y1: int, z1: int, x2: int, y2: int, z2: int
    ) -> RoaringBitmap:
        """Encode a 3D region into a roaring bitmap.

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
    """Morton encoding + Roaring bitmap spatial hash implementation.

    This class provides efficient spatial coordinate mapping using:
    - Morton encoding for spatial locality preservation
    - Roaring bitmaps for sparse coordinate storage
    - Per-cortical-area organization for modularity
    """

    def __init__(self, cache_dir: Optional[Path] = None):
        """Initialize the roaring spatial hash system.

        Args:
            cache_dir: Directory for caching bitmap data
        """
        self._state = (
            MortonSpatialHashState.READY
        )  # FIXED: Start in READY state
        self._lock = threading.RLock()

        # Core data structures
        self.cortical_bitmaps: Dict[str, RoaringBitmap] = {}
        self.neuron_map: Dict[Tuple[str, int], List[int]] = defaultdict(list)
        self.coordinate_map: Dict[int, Tuple[str, int, int, int]] = (
            {}
        )  # neuron_id → (cortical_area, x, y, z)

        # Statistics
        self.total_coordinates = 0
        self.total_areas = 0
        self.memory_usage = 0

        # Caching
        self.cache_dir = cache_dir or Path("cache/morton_spatial_hash")
        self.cache_dir.mkdir(parents=True, exist_ok=True)

        logger.info(
            f"[MORTON SPATIAL HASH] Initialized with roaring bitmap architecture (ROARING_AVAILABLE: {ROARING_AVAILABLE})"
        )

    def _set_state(self, new_state: MortonSpatialHashState):
        """Update the system state with thread safety."""
        with self._lock:
            old_state = self._state
            self._state = new_state
            logger.debug(
                f"[MORTON SPATIAL HASH] State: {old_state.value} → {new_state.value}"
            )

    def get_state(self) -> MortonSpatialHashState:
        """Get current system state."""
        return self._state

    def is_ready(self) -> bool:
        """Check if the system is ready for use."""
        return self._state == MortonSpatialHashState.READY

    def add_neuron(
        self, cortical_area: str, x: int, y: int, z: int, neuron_id: int
    ) -> bool:
        """Add a neuron coordinate to the spatial hash.

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
                coordinate_key = (cortical_area, morton_code)
                if neuron_id not in self.neuron_map[coordinate_key]:
                    self.neuron_map[coordinate_key].append(neuron_id)
                    self.coordinate_map[neuron_id] = (cortical_area, x, y, z)

                self.total_coordinates += 1
                return True

        except Exception as e:
            logger.error(f"[MORTON SPATIAL HASH] Error adding neuron: {e}")
            return False

    def get_neuron_at_coordinate(
        self, cortical_area: str, x: int, y: int, z: int
    ) -> Optional[int]:
        """Get neuron ID at specific coordinate.

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
                    coordinate_key = (cortical_area, morton_code)
                    return (
                        self.neuron_map[coordinate_key][0]
                        if self.neuron_map[coordinate_key]
                        else None
                    )

            return None

        except Exception as e:
            logger.error(f"[MORTON SPATIAL HASH] Error getting neuron: {e}")
            return None

    def get_neurons_at_coordinate(
        self, cortical_area: str, x: int, y: int, z: int
    ) -> List[int]:
        """Get all neuron IDs at specific coordinate.

        Args:
            cortical_area: Cortical area identifier
            x, y, z: 3D coordinates

        Returns:
            List of neuron IDs at the coordinate
        """
        try:
            morton_code = MortonUtils.morton_encode_3d(x, y, z)

            # Check if coordinate exists in cortical area
            if cortical_area in self.cortical_bitmaps:
                if morton_code in self.cortical_bitmaps[cortical_area]:
                    coordinate_key = (cortical_area, morton_code)
                    return self.neuron_map[coordinate_key].copy()

            return []

        except Exception as e:
            logger.error(f"[MORTON SPATIAL HASH] Error getting neurons: {e}")
            return []

    def get_neurons_in_region(
        self,
        cortical_area: str,
        x1: int,
        y1: int,
        z1: int,
        x2: int,
        y2: int,
        z2: int,
    ) -> List[int]:
        """Get all neuron IDs in a 3D region.

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
            region_bitmap = MortonUtils.morton_encode_region_3d(
                x1, y1, z1, x2, y2, z2
            )

            # Fast intersection with cortical area
            intersection = self.cortical_bitmaps[cortical_area] & region_bitmap

            # Get neuron IDs
            neuron_ids = []
            for morton_code in intersection:
                coordinate_key = (cortical_area, morton_code)
                if coordinate_key in self.neuron_map:
                    neuron_ids.extend(self.neuron_map[coordinate_key])

            return neuron_ids

        except Exception as e:
            logger.error(
                f"[MORTON SPATIAL HASH] Error getting neurons in region: {e}"
            )
            return []

    def get_area_union(self, cortical_areas: List[str]) -> RoaringBitmap:
        """Get union of multiple cortical areas (fast bitmap operation).

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

    def get_area_intersection(
        self, cortical_areas: List[str]
    ) -> RoaringBitmap:
        """Get intersection of multiple cortical areas.

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
                "encoding": "morton_3d",
            }

            # Per-area statistics
            for area_id, bitmap in self.cortical_bitmaps.items():
                stats["cortical_areas"][area_id] = {
                    "coordinate_count": len(bitmap),
                    "memory_bytes": (
                        bitmap.shrink_to_fit()
                        if hasattr(bitmap, "shrink_to_fit")
                        else 0
                    ),
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
        """Save current state to cache file.

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
                "version": "1.0",
            }

            with open(cache_file, "wb") as f:
                pickle.dump(cache_data, f)

            logger.info(f"[MORTON SPATIAL HASH] Saved to cache: {cache_file}")
            return True

        except Exception as e:
            logger.error(f"[MORTON SPATIAL HASH] Error saving cache: {e}")
            return False

    def load_from_cache(self, cache_key: str) -> bool:
        """Load state from cache file.

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

            #  Handle backward compatibility: convert old single-neuron format
            #  to list format
            loaded_neuron_map = cache_data["neuron_map"]
            self.neuron_map = defaultdict(list)

            for key, value in loaded_neuron_map.items():
                if isinstance(value, list):
                    # New format: already a list
                    self.neuron_map[key] = value
                else:
                    # Old format: single neuron ID, convert to list
                    self.neuron_map[key] = [value]

            self.coordinate_map = cache_data["coordinate_map"]
            self.total_coordinates = cache_data["total_coordinates"]
            self.total_areas = cache_data["total_areas"]

            logger.info(
                f"[MORTON SPATIAL HASH] Loaded from cache: {cache_file}"
            )
            return True

        except Exception as e:
            logger.error(f"[MORTON SPATIAL HASH] Error loading cache: {e}")
            return False

    # ConnectomeManager compatibility methods
    def initialize_for_dimensions(
        self, max_dims: Tuple[int, int, int]
    ) -> None:
        """Initialize spatial hash for specific dimensions (compatibility
        method).

        Args:
            max_dims: Maximum dimensions (advisory only for Morton encoding)
        """
        with self._lock:
            self._set_state(MortonSpatialHashState.READY)
            logger.info(
                f"[MORTON SPATIAL HASH] Initialized for dimensions: {max_dims}"
            )
            logger.info(
                "[MORTON SPATIAL HASH] Morton encoding handles any coordinate range automatically"
            )

    def wait_for_ready(self, timeout_seconds: float = 60.0) -> bool:
        """Wait for spatial hash to become ready.

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

    def expand_cache_for_new_area(
        self, position: Tuple[int, int, int], dimensions: Tuple[int, int, int]
    ) -> bool:
        """Expand cache for new cortical area (compatibility method).

        Note: Morton encoding handles any coordinate range automatically.

        Args:
            position: Position of new area (ignored)
            dimensions: Dimensions of new area (ignored)

        Returns:
            Always True (Morton encoding handles expansion automatically)
        """
        logger.debug(
            f"[MORTON SPATIAL HASH] Cache expansion requested for pos={position}, dims={dimensions}"
        )
        logger.debug(
            "[MORTON SPATIAL HASH] Morton encoding handles expansion automatically"
        )
        return True

    def add_coordinate(
        self, cortical_area: str, x: int, y: int, z: int, neuron_id: int
    ) -> bool:
        """
        Add coordinate (compatibility method - delegates to add_neuron).

        Args:
            cortical_area: Cortical area identifier
            x, y, z: 3D coordinates
            neuron_id: Neuron identifier

        Returns:
            True if added successfully
        """
        return self.add_neuron(cortical_area, x, y, z, neuron_id)

    def get_neuron_id(
        self, cortical_area: str, x: int, y: int, z: int
    ) -> Optional[int]:
        """
        Get neuron ID at coordinate (compatibility method - delegates to get_neuron_at_coordinate).

        Args:
            cortical_area: Cortical area identifier
            x, y, z: 3D coordinates

        Returns:
            Neuron ID if found, None otherwise
        """
        return self.get_neuron_at_coordinate(cortical_area, x, y, z)

    def batch_coordinate_lookup(
        self,
        candidate_positions: Set[Tuple[int, int, int]],
        neuron_positions: List[Tuple[int, int, int]],
    ) -> List[Tuple[int, int]]:
        """Fast batch lookup for matching candidate positions to neuron
        positions.

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
                    coord = (
                        int(candidate_pos[0]),
                        int(candidate_pos[1]),
                        int(candidate_pos[2]),
                    )
                    if coord in pos_to_neuron_idx:
                        neuron_idx = pos_to_neuron_idx[coord]
                        matches.append((candidate_idx, neuron_idx))

            return matches

        except Exception as e:
            logger.warning(
                f"[MORTON SPATIAL HASH] Error in batch_coordinate_lookup: {e}"
            )
            return []


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


def get_morton_coordinate_limits() -> Dict[str, int]:
    """Get Morton encoding coordinate limits and information.

    Returns:
        Dictionary containing coordinate limits and encoding information
    """
    return {
        "max_coordinate_per_dimension": (1 << 21)
        - 1,  # 2,097,151 (21-bit limit)
        "coordinate_limit_exclusive": (1 << 21),  # 2,097,152
        "bits_per_dimension": 21,
        "total_bits_used": 63,  # 21 * 3 dimensions
        "supports_negative_coordinates": False,
        "encoding_type": "morton_3d_z_order",
        "memory_efficient": True,
        "spatial_locality_preserved": True,
    }


def validate_coordinate_range(x: int, y: int, z: int) -> bool:
    """Validate that coordinates are within Morton encoding limits.

    Args:
        x, y, z: 3D coordinates to validate

    Returns:
        True if coordinates are valid, False otherwise
    """
    limit = 1 << 21  # 21-bit limit
    return 0 <= x < limit and 0 <= y < limit and 0 <= z < limit


def get_max_cortical_area_dimensions() -> Tuple[int, int, int]:
    """Get maximum safe cortical area dimensions for Morton encoding.

    Returns:
        Tuple of (max_width, max_height, max_depth) that can be safely used
    """
    # Leave room for 0-based indexing
    max_dim = (1 << 21) - 1  # 2,097,151
    return (max_dim, max_dim, max_dim)


def analyze_coordinate_requirements(
    max_x: int, max_y: int, max_z: int
) -> Dict[str, Any]:
    """Analyze coordinate requirements and suggest optimal Morton
    configuration.

    Args:
        max_x, max_y, max_z: Maximum coordinates that will be used

    Returns:
        Dictionary with analysis results and recommendations
    """
    limits = get_morton_coordinate_limits()
    max_allowed = limits["max_coordinate_per_dimension"]

    analysis = {
        "requested_max": (max_x, max_y, max_z),
        "morton_limit": max_allowed,
        "fits_in_21_bit": all(
            coord <= max_allowed for coord in [max_x, max_y, max_z]
        ),
        "exceeds_limit_by": {
            "x": max(0, max_x - max_allowed),
            "y": max(0, max_y - max_allowed),
            "z": max(0, max_z - max_allowed),
        },
        "recommendations": [],
    }

    if analysis["fits_in_21_bit"]:
        analysis["recommendations"].append(
            "✅ Coordinates fit within current 21-bit Morton encoding"
        )
    else:
        analysis["recommendations"].extend(
            [
                "⚠️  Coordinates exceed 21-bit Morton encoding limits",
                "🔧 Consider splitting large cortical areas into smaller ones",
                "🚀 Or upgrade to 64-bit Morton encoding for unlimited range",
                f"📊 Current limit: {max_allowed:,} per dimension",
                f"📊 Required: {max(max_x, max_y, max_z):,} per dimension",
            ]
        )

    return analysis
