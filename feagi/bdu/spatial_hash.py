"""
Global Spatial Hash System for FEAGI - Morton Encoding Backend

This module provides the legacy spatial hash interface while using the new
Morton encoding + Roaring bitmap implementation for superior memory efficiency.

NEW ARCHITECTURE (Morton + Roaring Bitmaps):
- 95%+ memory savings through sparse coordinate storage
- Spatial locality preservation via Morton encoding (Z-order curve)
- Microsecond multi-area union/intersection operations
- No hash collisions - direct coordinate encoding
- Automatic dynamic expansion without pre-allocated grids

BACKWARD COMPATIBILITY:
- Maintains exact same interface as original spatial hash
- Seamless migration without code changes required
- All existing functionality preserved

Copyright 2025 Neuraville Inc.
Licensed under the Apache License, Version 2.0
"""

# Import the adapter classes that provide backward compatibility
from .spatial_hash_adapter import (
    GlobalSpatialHash,
    SpatialHashConfig,
    SpatialHashState,
    analyze_genome_coordinate_space
)

# Re-export for backward compatibility
__all__ = [
    "GlobalSpatialHash",
    "SpatialHashConfig", 
    "SpatialHashState",
    "analyze_genome_coordinate_space",
    "get_spatial_hash",
    "initialize_spatial_hash",
    "get_cache_info",
    "clear_spatial_hash_cache",
    "rebuild_spatial_hash_cache"
]


def get_spatial_hash() -> GlobalSpatialHash:
    """
    Get the singleton spatial hash instance.
    
    Returns:
        GlobalSpatialHash: The singleton spatial hash instance
    """
    return GlobalSpatialHash()


def initialize_spatial_hash(config=None) -> GlobalSpatialHash:
    """
    Initialize spatial hash with optional configuration.
    
    Args:
        config: Optional SpatialHashConfig (ignored in Morton backend)
        
    Returns:
        GlobalSpatialHash: The initialized spatial hash instance
    """
    hash_instance = GlobalSpatialHash()
    
    # Morton encoding doesn't need pre-initialization,
    # but we maintain the interface for compatibility
    return hash_instance


def get_cache_info():
    """
    Get cache information from the spatial hash.
    
    Returns:
        Dict: Cache information and statistics
    """
    hash_instance = GlobalSpatialHash()
    return hash_instance.get_statistics()


def clear_spatial_hash_cache() -> bool:
    """
    Clear the spatial hash cache.
    
    Returns:
        bool: True if cache was cleared successfully
    """
    hash_instance = GlobalSpatialHash()
    hash_instance.clear()
    return True


def rebuild_spatial_hash_cache() -> bool:
    """
    Rebuild the spatial hash cache.
    
    Note: Morton encoding doesn't need rebuilding, but we maintain
    the interface for compatibility.
    
    Returns:
        bool: Always True (no-op for Morton backend)
    """
    # Morton encoding doesn't need cache rebuilding
    return True 