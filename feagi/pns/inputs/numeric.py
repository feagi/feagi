"""
Numeric Inputs

Generic numeric streams and numeric sensors (infrared, proximity, etc).
"""

from typing import Literal, Optional
from feagi.pns.inputs.base import BaseInput

# Type hints
NumericEncoding = Literal["absolute", "incremental", "inverted_absolute", "inverted_incremental"]


class NumericStream(BaseInput):
    """
    Generic numeric stream input.
    
    For game state, market data, sensor arrays, or any numeric data.
    Maps to Rust `sensor_miscellaneous_*` methods.
    
    Args:
        dimensions: Number of numeric values
        encoding: Encoding mode
        min_value: Minimum value (for normalization)
        max_value: Maximum value (for normalization)
    
    Example:
        # Game AI - 10D state vector
        game_state = NumericStream.register(
            dimensions=10,
            encoding="absolute"
        )
        
        while True:
            state = [player_x, player_y, enemy_x, ...]  # 10 values
            game_state.set_values(state)
            brain_input.send()
    """
    
    def __init__(
        self,
        dimensions: int,
        encoding: NumericEncoding = "absolute",
        min_value: float = 0.0,
        max_value: float = 1.0
    ):
        super().__init__()
        self.dimensions = dimensions
        self.encoding = encoding
        self.min_value = min_value
        self.max_value = max_value
        
        # Current values
        self._current_values: Optional[list] = None
    
    @classmethod
    def register(
        cls,
        dimensions: int = 1,
        encoding: NumericEncoding = "absolute",
        min_value: float = 0.0,
        max_value: float = 1.0
    ) -> 'NumericStream':
        """
        Register a new numeric stream input.
        
        Args:
            dimensions: Number of values in stream
            encoding: Encoding mode
            min_value: Minimum value
            max_value: Maximum value
        
        Returns:
            NumericStream instance
        """
        from feagi.pns import brain_input
        
        stream = cls(dimensions, encoding, min_value, max_value)
        brain_input.register_input(stream)
        return stream
    
    def set_values(self, values: list):
        """
        Set current numeric values.
        
        Args:
            values: List of numeric values (length must match dimensions)
        """
        if len(values) != self.dimensions:
            raise ValueError(
                f"Expected {self.dimensions} values, got {len(values)}"
            )
        self._current_values = values
    
    def _register_with_cache(self, cache, group_id: int):
        """Register with Rust IOCache"""
        # TODO: Implement generic numeric registration
        # For now, just store group_id
        pass
    
    def _write_to_cache(self, cache):
        """Write current values to Rust IOCache"""
        if self._current_values is None:
            return
        
        # TODO: Implement generic numeric write
        pass


class Infrared(BaseInput):
    """
    Infrared distance sensor input.
    
    Maps to Rust `sensor_infrared_*` methods.
    
    Args:
        encoding: Encoding mode
        inverted: Whether to invert the reading
        min_distance: Minimum distance (cm)
        max_distance: Maximum distance (cm)
    
    Example:
        infrared = Infrared.register(encoding="absolute")
        
        while True:
            distance = read_ir_sensor()  # Your hardware API
            infrared.set_distance(distance)
            brain_input.send()
    """
    
    def __init__(
        self,
        encoding: Literal["absolute", "incremental"] = "absolute",
        inverted: bool = False,
        min_distance: float = 0.0,
        max_distance: float = 400.0
    ):
        super().__init__()
        self.encoding = encoding
        self.inverted = inverted
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.channel = 0
        
        # Current distance
        self._current_distance: Optional[float] = None
    
    @classmethod
    def register(
        cls,
        encoding: Literal["absolute", "incremental"] = "absolute",
        inverted: bool = False,
        min_distance: float = 0.0,
        max_distance: float = 400.0
    ) -> 'Infrared':
        """
        Register a new infrared sensor input.
        
        Args:
            encoding: "absolute" or "incremental"
            inverted: Whether to invert the reading
            min_distance: Minimum distance (cm)
            max_distance: Maximum distance (cm)
        
        Returns:
            Infrared instance
        """
        from feagi.pns import brain_input
        
        sensor = cls(encoding, inverted, min_distance, max_distance)
        brain_input.register_input(sensor)
        return sensor
    
    def set_distance(self, distance: float):
        """
        Set current distance reading.
        
        Args:
            distance: Distance in cm
        """
        self._current_distance = distance
    
    def _register_with_cache(self, cache, group_id: int):
        """Register with Rust IOCache"""
        # Build method name based on encoding and inverted
        invert_str = "inverted_" if self.inverted else ""
        method_name = f"sensor_infrared_{invert_str}{self.encoding}_linear_try_register"
        
        # Get method
        if not hasattr(cache, method_name):
            raise AttributeError(f"Rust IOCache missing method: {method_name}")
        
        register_method = getattr(cache, method_name)
        
        # Register
        register_method(
            group=group_id,
            number_of_channels=1,
            z_neuron_depth=10  # Default depth
        )
    
    def _write_to_cache(self, cache):
        """Write current distance to Rust IOCache"""
        if self._current_distance is None:
            return
        
        # Normalize to [0, 1]
        normalized = (self._current_distance - self.min_distance) / (self.max_distance - self.min_distance)
        normalized = max(0.0, min(1.0, normalized))  # Clamp
        
        # Build method name
        invert_str = "inverted_" if self.inverted else ""
        method_name = f"sensor_infrared_{invert_str}{self.encoding}_linear_try_write"
        
        # Get method
        write_method = getattr(cache, method_name)
        
        # Write
        write_method(
            group=self.group_id,
            channel=self.channel,
            data=normalized
        )

