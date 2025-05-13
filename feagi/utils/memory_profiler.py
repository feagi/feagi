"""Memory profiling utilities for FEAGI.

This module provides tools for profiling memory usage in FEAGI, which helps
identify memory-intensive operations that would benefit from Rust's ownership model.
"""
import os
import sys
import time
from feagi.utils.logger import setup_logger
logger = setup_logger()
import functools
import tracemalloc
from typing import Dict, List, Optional, Any, Callable, Union, Tuple
import psutil
import numpy as np

logger = logging.getLogger("feagi.memory_profiler")

# Enable tracemalloc by default for memory tracking
tracemalloc.start()

class MemorySnapshot:
    """A memory snapshot that can be compared with others."""
    
    def __init__(self, snapshot=None, label: str = ""):
        """
        Initialize a memory snapshot.
        
        Args:
            snapshot: Optional tracemalloc snapshot
            label: Label for this snapshot
        """
        self.snapshot = snapshot or tracemalloc.take_snapshot()
        self.label = label
        self.timestamp = time.time()
        
        # Get process memory info
        self.process = psutil.Process(os.getpid())
        self.process_memory = self.process.memory_info().rss
    
    def compare_to(self, other: 'MemorySnapshot') -> Dict[str, Any]:
        """
        Compare this snapshot to another snapshot.
        
        Args:
            other: Another MemorySnapshot
            
        Returns:
            Dictionary with comparison statistics
        """
        if not isinstance(other, MemorySnapshot):
            raise TypeError("Can only compare with another MemorySnapshot")
        
        # Compare tracemalloc snapshots
        stats = self.snapshot.compare_to(other.snapshot, 'lineno')
        
        # Calculate process memory diff
        memory_diff = self.process_memory - other.process_memory
        time_diff = self.timestamp - other.timestamp
        
        return {
            "current_memory": self.process_memory,
            "previous_memory": other.process_memory,
            "memory_diff": memory_diff,
            "memory_diff_mb": memory_diff / (1024 * 1024),
            "time_diff": time_diff,
            "top_stats": stats,
        }
    
    def print_stats(self, limit: int = 10) -> None:
        """
        Print statistics from this snapshot.
        
        Args:
            limit: Number of top memory blocks to display
        """
        logger.info(f"--- Memory Snapshot: {self.label} ---")
        logger.info(f"Process memory: {self.process_memory / (1024 * 1024):.2f} MB")
        logger.info(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.timestamp))}")
        top_stats = self.snapshot.statistics('lineno')
        logger.info(f"Top {limit} memory blocks:")
        for stat in top_stats[:limit]:
            logger.info(f"{stat.size / 1024:.1f} KB - {stat.count} objects - {stat.traceback.format()[0]}")


class MemoryProfiler:
    """Memory profiler for tracking memory usage in FEAGI."""
    
    def __init__(self):
        """Initialize the memory profiler."""
        self.snapshots: Dict[str, MemorySnapshot] = {}
        self.baseline = self._take_snapshot("baseline")
    
    def _take_snapshot(self, label: str) -> MemorySnapshot:
        """
        Take a memory snapshot.
        
        Args:
            label: Label for the snapshot
            
        Returns:
            MemorySnapshot
        """
        snapshot = MemorySnapshot(label=label)
        self.snapshots[label] = snapshot
        return snapshot
    
    def snapshot(self, label: str) -> MemorySnapshot:
        """
        Take a labeled memory snapshot.
        
        Args:
            label: Label for the snapshot
            
        Returns:
            MemorySnapshot
        """
        return self._take_snapshot(label)
    
    def compare(self, start_label: str, end_label: str) -> Dict[str, Any]:
        """
        Compare two snapshots.
        
        Args:
            start_label: Label of the first snapshot
            end_label: Label of the second snapshot
            
        Returns:
            Dictionary with comparison statistics
        """
        if start_label not in self.snapshots:
            raise ValueError(f"Snapshot with label '{start_label}' not found")
        if end_label not in self.snapshots:
            raise ValueError(f"Snapshot with label '{end_label}' not found")
        
        start = self.snapshots[start_label]
        end = self.snapshots[end_label]
        
        return end.compare_to(start)
    
    def compare_to_baseline(self, label: str) -> Dict[str, Any]:
        """
        Compare a snapshot to the baseline.
        
        Args:
            label: Label of the snapshot to compare
            
        Returns:
            Dictionary with comparison statistics
        """
        if label not in self.snapshots:
            raise ValueError(f"Snapshot with label '{label}' not found")
        
        return self.snapshots[label].compare_to(self.baseline)
    
    def print_snapshot(self, label: str, limit: int = 10) -> None:
        """
        Print statistics for a snapshot.
        
        Args:
            label: Label of the snapshot
            limit: Number of top memory blocks to display
        """
        if label not in self.snapshots:
            raise ValueError(f"Snapshot with label '{label}' not found")
        
        self.snapshots[label].print_stats(limit)
    
    def print_comparison(self, start_label: str, end_label: str, limit: int = 10) -> None:
        """
        Print comparison between two snapshots.
        
        Args:
            start_label: Label of the first snapshot
            end_label: Label of the second snapshot
            limit: Number of top memory blocks to display
        """
        comparison = self.compare(start_label, end_label)
        logger.info(f"--- Memory Comparison: {start_label} → {end_label} ---")
        logger.info(f"Memory change: {comparison['memory_diff'] / (1024 * 1024):.2f} MB")
        logger.info(f"Time difference: {comparison['time_diff']:.2f} seconds")
        logger.info(f"Top {limit} memory changes:")
        for stat in comparison['top_stats'][:limit]:
            logger.info(f"{stat.size / 1024:.1f} KB - {stat.count} objects - {stat.traceback.format()[0]}")


def profile_memory(label: Optional[str] = None) -> Callable:
    """
    Decorator to profile memory usage of a function.
    
    Args:
        label: Optional label prefix for the snapshots
        
    Returns:
        Decorated function
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            # Create a profiler
            profiler = MemoryProfiler()
            
            # Generate a label
            func_label = label or func.__name__
            
            # Take a snapshot before the function call
            profiler.snapshot(f"{func_label}_start")
            
            # Call the function
            result = func(*args, **kwargs)
            
            # Take a snapshot after the function call
            profiler.snapshot(f"{func_label}_end")
            
            # Print the comparison
            profiler.print_comparison(f"{func_label}_start", f"{func_label}_end")
            
            return result
        return wrapper
    return decorator


# Create a global memory profiler instance
memory_profiler = MemoryProfiler()


# Example usage of the memory profiler
def example_usage():
    """Example of using the memory profiler."""
    # Take a snapshot before an operation
    memory_profiler.snapshot("before_operation")
    
    # Perform a memory-intensive operation
    big_array = np.zeros((1000, 1000))
    
    # Take a snapshot after the operation
    memory_profiler.snapshot("after_operation")
    
    # Print the comparison
    memory_profiler.print_comparison("before_operation", "after_operation")


# Example of using the decorator
@profile_memory("create_array")
def create_large_array(size: Tuple[int, ...]):
    """Create a large array and return it."""
    return np.zeros(size) 