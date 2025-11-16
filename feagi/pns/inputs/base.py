"""
Base Input Class

Abstract base class for all FEAGI inputs (sensory data sources).
"""

from abc import ABC, abstractmethod
from typing import Optional, Any


class BaseInput(ABC):
    """
    Abstract base class for all FEAGI inputs.
    
    Inputs represent any data source that feeds into FEAGI:
    - Vision (cameras)
    - Audio (microphones)
    - Text (language input)
    - Numeric (sensors, game state, market data)
    - GPIO (digital/analog pins)
    
    Subclasses must implement:
    - _register_with_cache(): Register with Rust IOCache
    - _write_to_cache(): Write current value to cache
    
    Example:
        class MyInput(BaseInput):
            def _register_with_cache(self, cache, group_id):
                cache.sensor_my_type_try_register(group_id, ...)
            
            def _write_to_cache(self, cache):
                cache.sensor_my_type_try_write(self.group_id, ...)
    """
    
    def __init__(self, group_id: Optional[int] = None):
        """
        Initialize base input.
        
        Args:
            group_id: Cortical group ID (assigned during registration)
        """
        self.group_id = group_id
        self._registered = False
    
    @abstractmethod
    def _register_with_cache(self, cache: Any, group_id: int):
        """
        Register this input with Rust IOCache.
        
        Called internally by brain_input when sensor is registered.
        Maps to appropriate sensor_*_try_register() method in Rust.
        
        Args:
            cache: Rust IOCache instance
            group_id: Assigned cortical group ID
        """
        pass
    
    @abstractmethod
    def _write_to_cache(self, cache: Any):
        """
        Write current input value to Rust IOCache.
        
        Called internally by brain_input.send().
        Maps to appropriate sensor_*_try_write() method in Rust.
        
        Args:
            cache: Rust IOCache instance
        """
        pass
    
    def _mark_registered(self, group_id: int):
        """Mark this input as registered (called internally)"""
        self.group_id = group_id
        self._registered = True
    
    def is_registered(self) -> bool:
        """Check if this input is registered"""
        return self._registered

