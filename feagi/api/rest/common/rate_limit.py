"""Rate limiting utilities for FEAGI REST API."""

import time
from typing import Dict, Tuple, Optional
import logging

logger = logging.getLogger(__name__)

class RateLimiter:
    """Rate limiter for API endpoints."""
    
    def __init__(self, requests_per_minute: int = 60):
        """
        Initialize a rate limiter.
        
        Args:
            requests_per_minute: Maximum number of requests allowed per minute.
        """
        self.requests_per_minute = requests_per_minute
        self.window_size = 60  # seconds
        self.clients: Dict[str, Tuple[float, int]] = {}  # client_id -> (window_start, count)
        
    def check_limit(self, client_id: str) -> bool:
        """
        Check if a client has exceeded the rate limit.
        
        Args:
            client_id: ID of the client.
            
        Returns:
            True if the client has not exceeded the rate limit, False otherwise.
        """
        current_time = time.time()
        
        # Get client data or initialize
        window_start, count = self.clients.get(client_id, (current_time, 0))
        
        # Check if window has expired
        if current_time - window_start > self.window_size:
            # Reset window
            window_start = current_time
            count = 0
        
        # Check if limit exceeded
        if count >= self.requests_per_minute:
            logger.warning(f"Rate limit exceeded for client {client_id}")
            return False
        
        # Update client data
        self.clients[client_id] = (window_start, count + 1)
        return True
        
    def get_remaining(self, client_id: str) -> Tuple[int, int]:
        """
        Get the remaining requests and seconds in the current window.
        
        Args:
            client_id: ID of the client.
            
        Returns:
            Tuple of (remaining requests, seconds remaining in window).
        """
        current_time = time.time()
        
        # Get client data or initialize
        window_start, count = self.clients.get(client_id, (current_time, 0))
        
        # Check if window has expired
        if current_time - window_start > self.window_size:
            return self.requests_per_minute, self.window_size
        
        # Calculate remaining
        remaining_requests = max(0, self.requests_per_minute - count)
        remaining_seconds = max(0, self.window_size - (current_time - window_start))
        
        return remaining_requests, int(remaining_seconds) 