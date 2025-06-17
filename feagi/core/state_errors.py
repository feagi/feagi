"""
Rust-compatible error handling for FEAGI state management.

This module provides Result-style error handling that can be directly
converted to Rust's std::result::Result<T, E> when migrating.
"""

from enum import IntEnum
from typing import Optional, Generic, TypeVar, Union, Callable, Any
import logging

logger = logging.getLogger(__name__)

class StateError(IntEnum):
    """
    Rust-compatible error codes using fixed integer values.
    
    These map directly to Rust enum discriminants and avoid
    dynamic string allocation in error handling.
    """
    SUCCESS = 0
    INVALID_TRANSITION = 1
    PREREQUISITE_NOT_MET = 2
    SYSTEM_NOT_READY = 3
    STORAGE_FAILURE = 4
    VALIDATION_FAILED = 5
    CONCURRENT_MODIFICATION = 6
    RESOURCE_EXHAUSTED = 7
    TIMEOUT = 8
    PERMISSION_DENIED = 9

T = TypeVar('T')
E = TypeVar('E')

class Result(Generic[T]):
    """
    Rust-style Result type for error handling without exceptions.
    
    This provides a zero-cost abstraction for error handling that
    can be directly converted to Rust's Result<T, E> type.
    
    Examples:
        >>> result = Result.ok(42)
        >>> if result.is_ok:
        ...     value = result.unwrap()
        
        >>> error_result = Result.err(StateError.INVALID_TRANSITION)
        >>> if error_result.is_err:
        ...     error = error_result.unwrap_err()
    """
    
    def __init__(self, value: Optional[T] = None, error: Optional[StateError] = None, _is_ok: bool = None):
        """
        Initialize Result with either a value or an error.
        
        Args:
            value: Success value (mutually exclusive with error)
            error: Error code (mutually exclusive with value)
            _is_ok: Internal flag to distinguish None value from no value
        """
        if error is not None:
            if value is not None:
                raise ValueError("Result cannot have both value and error")
            self._value = None
            self._error = error
            self._is_ok = False
        elif _is_ok is not False:
            self._value = value
            self._error = None
            self._is_ok = True
        else:
            raise ValueError("Result must have either value or error")
    
    @property
    def is_ok(self) -> bool:
        """Return True if Result contains a value."""
        return self._is_ok
    
    @property
    def is_err(self) -> bool:
        """Return True if Result contains an error."""
        return not self._is_ok
    
    def unwrap(self) -> T:
        """
        Extract the value, panicking if Result contains an error.
        
        Returns:
            The contained value
            
        Raises:
            RuntimeError: If Result contains an error
        """
        if not self._is_ok:
            raise RuntimeError(f"Called unwrap on error: {self._error.name}")
        return self._value
    
    def unwrap_err(self) -> StateError:
        """
        Extract the error, panicking if Result contains a value.
        
        Returns:
            The contained error
            
        Raises:
            RuntimeError: If Result contains a value
        """
        if self._is_ok:
            raise RuntimeError("Called unwrap_err on success value")
        return self._error
    
    def unwrap_or(self, default: T) -> T:
        """
        Extract the value or return a default.
        
        Args:
            default: Value to return if Result contains an error
            
        Returns:
            The contained value or the default
        """
        return self._value if self._is_ok else default
    
    def unwrap_or_else(self, func: Callable[[StateError], T]) -> T:
        """
        Extract the value or compute a default from the error.
        
        Args:
            func: Function to compute default from error
            
        Returns:
            The contained value or computed default
        """
        return self._value if self._is_ok else func(self._error)
    
    def expect(self, message: str) -> T:
        """
        Extract the value, panicking with a custom message if error.
        
        Args:
            message: Custom panic message
            
        Returns:
            The contained value
            
        Raises:
            RuntimeError: If Result contains an error
        """
        if not self._is_ok:
            raise RuntimeError(f"{message}: {self._error.name}")
        return self._value
    
    def map(self, func: Callable[[T], Any]) -> 'Result[Any]':
        """
        Transform the contained value if present.
        
        Args:
            func: Function to transform the value
            
        Returns:
            New Result with transformed value or original error
        """
        if self._is_ok:
            try:
                return Result.ok(func(self._value))
            except Exception as e:
                logger.error(f"Error in Result.map: {e}")
                return Result.err(StateError.VALIDATION_FAILED)
        else:
            return Result.err(self._error)
    
    def and_then(self, func: Callable[[T], 'Result[Any]']) -> 'Result[Any]':
        """
        Chain operations that return Results (flatMap).
        
        Args:
            func: Function that takes value and returns Result
            
        Returns:
            Result from func or original error
        """
        if self._is_ok:
            return func(self._value)
        else:
            return Result.err(self._error)
    
    @staticmethod
    def ok(value: T) -> 'Result[T]':
        """
        Create a successful Result.
        
        Args:
            value: Success value
            
        Returns:
            Result containing the value
        """
        return Result(value=value, _is_ok=True)
    
    @staticmethod
    def err(error: StateError) -> 'Result[T]':
        """
        Create an error Result.
        
        Args:
            error: Error code
            
        Returns:
            Result containing the error
        """
        return Result(error=error, _is_ok=False)

def validate_state_transition(from_state: int, to_state: int, 
                            valid_transitions: dict) -> Result[None]:
    """
    Validate a state transition using a lookup table.
    
    This provides constant-time validation suitable for real-time systems.
    
    Args:
        from_state: Current state value
        to_state: Desired state value
        valid_transitions: Dictionary of valid (from, to) transitions
        
    Returns:
        Result indicating success or failure
    """
    if (from_state, to_state) in valid_transitions:
        return Result.ok(None)
    else:
        logger.warning(f"Invalid state transition: {from_state} -> {to_state}")
        return Result.err(StateError.INVALID_TRANSITION)

def combine_results(*results: Result[Any]) -> Result[list]:
    """
    Combine multiple Results into a single Result.
    
    Args:
        *results: Variable number of Results to combine
        
    Returns:
        Result containing list of all values, or first error encountered
    """
    values = []
    for result in results:
        if result.is_err:
            return Result.err(result.unwrap_err())
        values.append(result.unwrap())
    
    return Result.ok(values) 