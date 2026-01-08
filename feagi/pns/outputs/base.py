"""
Base Output Class

Abstract base class for all FEAGI outputs (motor/action targets).
"""

from abc import ABC, abstractmethod
from typing import Optional, Any


class BaseOutput(ABC):
    """
    Abstract base class for all FEAGI outputs.
    
    Outputs represent any data target that receives from FEAGI:
    - Motors (servo, rotary, stepper)
    - Audio (speakers)
    - Text (language generation)
    - Numeric (control signals, trading actions)
    - GPIO (digital/analog outputs)
    - LED (visual indicators)
    
    Subclasses must implement:
    - _register_with_cache(): Register with Rust IOCache
    - _read_from_cache(): Read current value from cache
    
    Example:
        class MyOutput(BaseOutput):
            def _register_with_cache(self, cache, group_id):
                cache.motor_my_type_try_register(group_id, ...)
            
            def _read_from_cache(self, cache):
                value = cache.motor_my_type_try_read(self.group_id, ...)
                self._current_value = value
    """
    
    def __init__(self, group_id: Optional[int] = None):
        """
        Initialize base output.
        
        Args:
            group_id: Cortical group ID (assigned during registration)
        """
        self.group_id = group_id
        self._registered = False
    
    @abstractmethod
    def _register_with_cache(self, cache: Any, group_id: int, channel_index: int = None, decoder_registered: bool = False):
        """
        Register this output with Rust IOCache.
        
        Called internally by brain_output when actuator is registered.
        Maps to appropriate motor_*_try_register() method in Rust.
        
        Args:
            cache: Rust IOCache instance
            group_id: Assigned cortical group ID
            channel_index: Optional channel index (for motors, all use group=0, differentiated by channel)
            decoder_registered: Whether the motor decoder has already been registered
        """
        pass
    
    @abstractmethod
    def _read_from_cache(self, cache: Any):
        """
        Read current output value from Rust IOCache.
        
        Called internally by brain_output.receive().
        Maps to appropriate motor_*_try_read() method in Rust.
        
        Args:
            cache: Rust IOCache instance
        """
        pass
    
    def _mark_registered(self, group_id: int):
        """Mark this output as registered (called internally)"""
        self.group_id = group_id
        self._registered = True
    
    def is_registered(self) -> bool:
        """Check if this output is registered"""
        return self._registered

