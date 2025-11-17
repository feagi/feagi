"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import logging
import time
from typing import Callable, Dict, List, Tuple

import numpy as np

logger = logging.getLogger(__name__)
"""Performance metrics and profiling utilities for the BDU.

This module provides utilities for measuring and analyzing the performance
of neural processing operations.
"""


class PerformanceTimer:
    """A utility class for timing operations and collecting performance
    metrics."""

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
            "total": sum(self.history),
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
    def neuron_density(
        neuron_count: int, area_dimensions: Tuple[int, int, int]
    ) -> float:
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

