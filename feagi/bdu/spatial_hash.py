"""
FEAGI Spatial Hash System - Morton Encoding Backend

This module provides efficient spatial indexing for neuron locations using:
- Morton encoding (Z-order curve) for spatial locality preservation  
- Roaring bitmaps for sparse coordinate storage with 95%+ memory efficiency
- Per-cortical-area organization for modular spatial domains
- Thread-safe operations with proper locking mechanisms

Performance Benefits:
- 95-99% memory savings for sparse genomes
- O(log N) region queries vs O(N) traditional
- Microsecond multi-area union/intersection operations
- No hash collisions - direct coordinate encoding

Copyright 2025 Neuraville Inc.
Licensed under the Apache License, Version 2.0
"""

import logging
from typing import Dict, List, Tuple, Set, Optional, Any
from pathlib import Path

from .morton_spatial_hash import (
    RoaringSpatialHash,
    MortonSpatialHashState, 
    get_morton_spatial_hash,
    reset_morton_spatial_hash
)

logger = logging.getLogger(__name__)


class SpatialHashConfig:
    """Configuration for Morton spatial hash system."""
    
    def __init__(self, max_dimension: int = 256, genome_based_sizing: bool = True, 
                 enable_caching: bool = True):
        """
        Initialize spatial hash configuration.
        
        Args:
            max_dimension: Maximum coordinate dimension (advisory only for Morton)
            genome_based_sizing: Whether to size based on genome analysis
            enable_caching: Whether to enable persistent caching
        """
        self.max_dimension = max_dimension
        self.genome_based_sizing = genome_based_sizing
        self.enable_caching = enable_caching
        self.cache_dir = Path("cache/morton_spatial_hash")


class SpatialHashState:
    """State constants for spatial hash system."""
    UNINITIALIZED = "uninitialized"
    BUILDING = "building"
    READY = "ready"
    EXPANDING = "expanding"
    ERROR = "error"


class GlobalSpatialHash:
    """
    Global singleton spatial hash using Morton encoding.
    
    This class provides the main interface to FEAGI's spatial hash system,
    using Morton encoding + Roaring bitmaps for optimal performance.
    """
    
    _instance: Optional[RoaringSpatialHash] = None
    _lock = None
    
    def __new__(cls):
        """Singleton pattern - returns the global Morton spatial hash instance."""
        return get_morton_spatial_hash()
    
    def __init__(self):
        """Initialize (singleton pattern handles actual initialization)."""
        pass
    
    @classmethod
    def reset_instance(cls):
        """Reset the global instance (for testing)."""
        reset_morton_spatial_hash()


# Main interface functions
def get_spatial_hash() -> RoaringSpatialHash:
    """
    Get the singleton spatial hash instance.
    
    Returns:
        RoaringSpatialHash: The global Morton spatial hash instance
    """
    return get_morton_spatial_hash()


def initialize_spatial_hash(config: Optional[SpatialHashConfig] = None) -> RoaringSpatialHash:
    """
    Initialize spatial hash with optional configuration.
    
    Args:
        config: Optional SpatialHashConfig (cache settings only)
        
    Returns:
        RoaringSpatialHash: The initialized spatial hash instance
    """
    spatial_hash = get_morton_spatial_hash()
    
    if config and config.cache_dir:
        # Update cache directory if specified
        spatial_hash.cache_dir = config.cache_dir
        spatial_hash.cache_dir.mkdir(parents=True, exist_ok=True)
    
    logger.info("[SPATIAL HASH] Morton encoding system initialized")
    return spatial_hash


def get_cache_info() -> Dict[str, Any]:
    """
    Get cache information from the spatial hash.
    
    Returns:
        Dict: Cache information and statistics
    """
    spatial_hash = get_morton_spatial_hash()
    return spatial_hash.get_statistics()


def clear_spatial_hash_cache() -> bool:
    """
    Clear the spatial hash cache.
    
    Returns:
        bool: True if cache was cleared successfully
    """
    spatial_hash = get_morton_spatial_hash()
    spatial_hash.clear()
    logger.info("[SPATIAL HASH] Cache cleared")
    return True


def rebuild_spatial_hash_cache() -> bool:
    """
    Rebuild the spatial hash cache.
    
    Note: Morton encoding doesn't need rebuilding, but we maintain
    the interface for compatibility.
    
    Returns:
        bool: Always True (no-op for Morton backend)
    """
    logger.info("[SPATIAL HASH] Cache rebuild requested (no-op for Morton system)")
    return True


def analyze_genome_coordinate_space(genome_data: Dict[str, Any]) -> Tuple[int, int, int]:
    """
    Analyze genome coordinate space to determine optimal dimensions.
    
    Args:
        genome_data: Genome data containing cortical area definitions
        
    Returns:
        Tuple of (max_x, max_y, max_z) dimensions found in genome
    """
    try:
        max_x = max_y = max_z = 0
        
        # Look for cortical area dimensions in blueprint
        blueprint = genome_data.get("blueprint", {})
        
        for key, value in blueprint.items():
            if isinstance(key, str) and isinstance(value, (int, float)):
                # Parse cortical area dimension keys
                if key.endswith("-bbx-i"):  # bounding box x
                    max_x = max(max_x, int(value))
                elif key.endswith("-bby-i"):  # bounding box y  
                    max_y = max(max_y, int(value))
                elif key.endswith("-bbz-i"):  # bounding box z
                    max_z = max(max_z, int(value))
        
        # Ensure minimum dimensions
        max_x = max(max_x, 8)
        max_y = max(max_y, 8) 
        max_z = max(max_z, 8)
        
        logger.info(f"[SPATIAL HASH] Analyzed genome dimensions: ({max_x}, {max_y}, {max_z})")
        return (max_x, max_y, max_z)
        
    except Exception as e:
        logger.warning(f"[SPATIAL HASH] Error analyzing genome: {e}")
        return (256, 256, 256)  # Safe fallback


# Re-export key classes for backward compatibility
__all__ = [
    "GlobalSpatialHash",
    "SpatialHashConfig", 
    "SpatialHashState",
    "get_spatial_hash",
    "initialize_spatial_hash",
    "get_cache_info",
    "clear_spatial_hash_cache", 
    "rebuild_spatial_hash_cache",
    "analyze_genome_coordinate_space"
] 