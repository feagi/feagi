"""Metrics and performance utilities for the BDU.

This module provides functions for collecting performance statistics
and metrics about the connectome structure.
"""

import time
from typing import Dict, Any, List, Tuple, Callable, Optional
import logging

logger = logging.getLogger(__name__)

class PerformanceTimer:
    """A utility class for timing operations and collecting performance metrics."""
    
    def __init__(self, name: str):
        """Initialize a new timer with a given name.
        
        Args:
            name: A descriptive name for this timer
        """
        self.name = name
        self.start_time = None
        self.history = []
        
    def start(self) -> None:
        """Start the timer."""
        self.start_time = time.time()
    
    def stop(self) -> float:
        """Stop the timer and record the elapsed time.
        
        Returns:
            The elapsed time in seconds
        """
        if self.start_time is None:
            raise ValueError("Timer was not started")
        
        elapsed = time.time() - self.start_time
        self.history.append(elapsed)
        self.start_time = None
        return elapsed
    
    def get_stats(self) -> Dict[str, float]:
        """Get statistics about recorded times.
        
        Returns:
            Dictionary with mean, min, max, and total times
        """
        if not self.history:
            return {"count": 0, "mean": 0, "min": 0, "max": 0, "total": 0}
        
        return {
            "count": len(self.history),
            "mean": sum(self.history) / len(self.history),
            "min": min(self.history),
            "max": max(self.history),
            "total": sum(self.history)
        }
    
    def reset(self) -> None:
        """Reset the timer's history."""
        self.history = []
        self.start_time = None


def timing_decorator(func: Callable) -> Callable:
    """Decorator to time a function's execution.
    
    Args:
        func: The function to time
        
    Returns:
        A wrapped function that logs timing information
    """
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        elapsed = time.time() - start_time
        logger.debug(f"{func.__name__} took {elapsed:.6f} seconds")
        return result
    return wrapper


class ConnectomeMetrics:
    """Collects and calculates metrics about the connectome structure."""
    
    @staticmethod
    def neuron_density(neuron_count: int, area_dimensions: Tuple[int, int, int]) -> float:
        """Calculate neuron density within a cortical area.
        
        Args:
            neuron_count: Number of neurons in the area
            area_dimensions: Dimensions of the area (width, height, depth)
            
        Returns:
            Neurons per unit volume
        """
        volume = area_dimensions[0] * area_dimensions[1] * area_dimensions[2]
        if volume == 0:
            return 0
        return neuron_count / volume
    
    @staticmethod
    def connectivity_density(synapse_count: int, neuron_count: int) -> float:
        """Calculate average number of synapses per neuron.
        
        Args:
            synapse_count: Total number of synapses
            neuron_count: Total number of neurons
            
        Returns:
            Average synapses per neuron
        """
        if neuron_count == 0:
            return 0
        return synapse_count / neuron_count
    
    @staticmethod
    def area_connectivity_matrix(connectome, area_ids: List[str]) -> Dict[Tuple[str, str], int]:
        """Calculate connectivity between areas.
        
        Args:
            connectome: The connectome manager object
            area_ids: List of area IDs to analyze
            
        Returns:
            Dictionary mapping (source_area, target_area) tuples to connection counts
        """
        matrix = {}
        
        for source_id in area_ids:
            for target_id in area_ids:
                matrix[(source_id, target_id)] = 0
        
        # This is a placeholder - actual implementation would depend on
        # how the connectome stores connections between areas
        return matrix 