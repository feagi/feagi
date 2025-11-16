"""
Numeric Outputs

Generic numeric streams for control signals, trading actions, etc.
"""

from typing import Optional
from feagi.pns.outputs.base import BaseOutput


class NumericStream(BaseOutput):
    """
    Generic numeric stream output.
    
    For game actions, trading signals, control commands, or any numeric output.
    
    Args:
        dimensions: Number of numeric values
    
    Example:
        # Trading bot - 3 signals (buy, sell, hold)
        trading_signal = NumericStream.register(dimensions=3)
        
        while True:
            brain_output.receive()
            signals = trading_signal.get_values()  # [0.1, 0.8, 0.1]
            
            if signals[0] > 0.7:  # Buy signal
                execute_trade("buy")
            elif signals[1] > 0.7:  # Sell signal
                execute_trade("sell")
    """
    
    def __init__(self, dimensions: int):
        super().__init__()
        self.dimensions = dimensions
        
        # Current values (from FEAGI)
        self._current_values: Optional[list] = [0.0] * dimensions
    
    @classmethod
    def register(cls, dimensions: int = 1) -> 'NumericStream':
        """
        Register a new numeric stream output.
        
        Args:
            dimensions: Number of values in stream
        
        Returns:
            NumericStream instance
        """
        from feagi.pns import brain_output
        
        stream = cls(dimensions)
        brain_output.register_output(stream)
        return stream
    
    def get_values(self) -> list:
        """
        Get current numeric values from FEAGI.
        
        Returns:
            List of numeric values
        """
        return self._current_values
    
    def _register_with_cache(self, cache, group_id: int):
        """Register with Rust IOCache"""
        # TODO: Implement generic numeric output registration
        pass
    
    def _read_from_cache(self, cache):
        """Read current values from Rust IOCache"""
        # TODO: Implement generic numeric output read
        pass

