"""
Rate limiting utilities for FEAGI API.

This module provides rate limiting mechanisms for API requests.
"""

from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import time
from typing import Dict, Optional, List, Any, Union


class RateLimiter:
    """
    Simple rate limiter implementation.
    
    This class provides mechanisms to limit the rate of operations
    based on various criteria.
    """
    
    def __init__(self):
        """Initialize a new RateLimiter instance."""
        # Store of last operation times by key
        self.last_ops = {}
        
        # Store of operation counts within window by key
        self.op_counts = {}
        
        # Store of window start times by key
        self.window_starts = {}
    
    def check_rate(
        self, 
        key: str, 
        max_rate: float, 
        window: float = 1.0
    ) -> bool:
        """
        Check if an operation is allowed under the rate limit.
        
        Args:
            key: Identifier for the rate limit
            max_rate: Maximum operations per second
            window: Time window to check (in seconds)
            
        Returns:
            True if operation is allowed, False if it would exceed rate limit
        """
        current_time = time.time()
        
        # Initialize window if not exists
        if key not in self.window_starts:
            self.window_starts[key] = current_time
            self.op_counts[key] = 0
        
        # Check if window has expired
        if current_time - self.window_starts[key] > window:
            # Reset window
            self.window_starts[key] = current_time
            self.op_counts[key] = 0
        
        # Check if operation would exceed rate
        if self.op_counts[key] >= max_rate * window:
            return False
        
        # Increment count and allow operation
        self.op_counts[key] += 1
        return True
    
    def throttle(
        self, 
        key: str, 
        min_interval: float
    ) -> float:
        """
        Check if operation should be throttled and get wait time.
        
        Args:
            key: Identifier for the throttled operation
            min_interval: Minimum interval between operations (in seconds)
            
        Returns:
            Time to wait (in seconds) before operation should proceed
            (0 if operation can proceed immediately)
        """
        current_time = time.time()
        
        # Get last operation time
        last_time = self.last_ops.get(key, 0)
        
        # Calculate time since last operation
        elapsed = current_time - last_time
        
        # Update last operation time
        self.last_ops[key] = current_time
        
        # If enough time has elapsed, no wait needed
        if elapsed >= min_interval:
            return 0
        
        # Otherwise, return wait time
        return min_interval - elapsed
    
    async def wait_if_needed(
        self, 
        key: str, 
        min_interval: float
    ) -> None:
        """
        Wait if necessary to maintain the minimum interval.
        
        Args:
            key: Identifier for the throttled operation
            min_interval: Minimum interval between operations (in seconds)
        """
        import asyncio
        
        wait_time = self.throttle(key, min_interval)
        if wait_time > 0:
            await asyncio.sleep(wait_time) 