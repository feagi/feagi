"""
Motor Outputs

Servo motors, rotary motors (DC), stepper motors, etc.
"""

from typing import Tuple, Literal
from feagi.pns.outputs.base import BaseOutput

# Type hints
MotorEncoding = Literal["absolute", "incremental"]


class ServoMotor(BaseOutput):
    """
    Servo motor output (positional servo).
    
    Maps to Rust `motor_positional_servo_*` methods.
    
    Args:
        range: (min_angle, max_angle) in degrees
        encoding: Encoding mode ("absolute" or "incremental")
    
    Example:
        from feagi.pns.outputs import ServoMotor
        from feagi.pns import brain_output
        
        # Register servo
        servo = ServoMotor.register(range=(0, 180))
        
        # Configure and connect
        brain_output.configure(feagi_host="localhost")
        brain_output.connect()
        
        # Main loop
        while True:
            brain_output.receive()
            angle = servo.get_angle()
            set_servo_hardware(angle)  # Your hardware API
    
    Note:
        The servo reads values as SignedPercentage (-1.0 to 1.0) from FEAGI
        and automatically maps them to your specified angle range.
    """
    
    def __init__(
        self,
        range: Tuple[float, float] = (0, 180),
        encoding: MotorEncoding = "absolute"
    ):
        super().__init__()
        self.min_angle, self.max_angle = range
        self.encoding = encoding
        self.channel = 0
        
        # Current angle (from FEAGI)
        self._current_angle: float = (self.min_angle + self.max_angle) / 2
    
    @classmethod
    def register(
        cls,
        range: Tuple[float, float] = (0, 180),
        encoding: MotorEncoding = "absolute"
    ) -> 'ServoMotor':
        """
        Register a new servo motor output.
        
        Args:
            range: (min_angle, max_angle) in degrees
            encoding: "absolute" or "incremental"
        
        Returns:
            ServoMotor instance
        """
        from feagi.pns import brain_output
        
        servo = cls(range, encoding)
        brain_output.register_output(servo)
        return servo
    
    def get_angle(self) -> float:
        """
        Get current servo angle from FEAGI.
        
        Returns:
            Angle in degrees
        """
        return self._current_angle
    
    def _register_with_cache(self, cache, group_id: int):
        """Register with Rust IOCache"""
        # Build method name (servo is called "positional_servo" in Rust)
        method_name = f"motor_positional_servo_{self.encoding}_linear_try_register"
        
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
    
    def _read_from_cache(self, cache):
        """Read current angle from Rust IOCache"""
        # Build method name (servo is called "positional_servo" in Rust)
        method_name = f"motor_positional_servo_{self.encoding}_linear_try_read_postprocessed_cached_value"
        
        # Get method
        if not hasattr(cache, method_name):
            return  # Method not available yet
        
        read_method = getattr(cache, method_name)
        
        # Read
        try:
            import feagi_rust_py_libs as frpl
            
            # Read percentage value
            percentage = read_method(
                group=self.group_id,
                channel=self.channel
            )
            
            # Convert SignedPercentage to float
            if hasattr(percentage, 'value'):
                value = percentage.value()
            else:
                value = float(percentage)
            
            # Map from signed percentage (-1.0 to 1.0) to angle range
            # Normalize to 0.0 to 1.0
            normalized = (value + 1.0) / 2.0
            
            # Map to angle range
            self._current_angle = self.min_angle + (normalized * (self.max_angle - self.min_angle))
        except Exception:
            # If reading fails, keep current value
            pass


class RotaryMotor(BaseOutput):
    """
    Rotary motor (DC motor) output.
    
    Maps to Rust `motor_rotary_motor_*` methods.
    
    Args:
        encoding: Encoding mode
        bidirectional: Whether motor can reverse
    
    Example:
        motor_left = RotaryMotor.register(bidirectional=True)
        motor_right = RotaryMotor.register(bidirectional=True)
        
        while True:
            brain_output.receive()
            left_speed = motor_left.get_speed()   # -1.0 to 1.0
            right_speed = motor_right.get_speed()
            set_motor_speeds(left_speed, right_speed)
    """
    
    def __init__(
        self,
        encoding: MotorEncoding = "absolute",
        bidirectional: bool = True
    ):
        super().__init__()
        self.encoding = encoding
        self.bidirectional = bidirectional
        self.channel = 0
        
        # Current speed (from FEAGI)
        # Range: -1.0 to 1.0 if bidirectional, 0.0 to 1.0 if not
        self._current_speed: float = 0.0
    
    @classmethod
    def register(
        cls,
        encoding: MotorEncoding = "absolute",
        bidirectional: bool = True
    ) -> 'RotaryMotor':
        """
        Register a new rotary motor output.
        
        Args:
            encoding: "absolute" or "incremental"
            bidirectional: Whether motor can reverse
        
        Returns:
            RotaryMotor instance
        """
        from feagi.pns import brain_output
        
        motor = cls(encoding, bidirectional)
        brain_output.register_output(motor)
        return motor
    
    def get_speed(self) -> float:
        """
        Get current motor speed from FEAGI.
        
        Returns:
            Speed: -1.0 to 1.0 (bidirectional) or 0.0 to 1.0 (unidirectional)
        """
        return self._current_speed
    
    def _register_with_cache(self, cache, group_id: int):
        """Register with Rust IOCache"""
        # Build method name
        method_name = f"motor_rotary_motor_{self.encoding}_linear_try_register"
        
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
    
    def _read_from_cache(self, cache):
        """Read current speed from Rust IOCache"""
        # Build method name
        method_name = f"motor_rotary_motor_{self.encoding}_linear_try_read_postprocessed_cached_value"
        
        # Get method
        if not hasattr(cache, method_name):
            return  # Method not available yet
        
        read_method = getattr(cache, method_name)
        
        # Read
        try:
            import feagi_rust_py_libs as frpl
            
            # Read percentage value
            percentage = read_method(
                group=self.group_id,
                channel=self.channel
            )
            
            # Convert SignedPercentage to float
            if hasattr(percentage, 'value'):
                value = percentage.value()
            else:
                value = float(percentage)
            
            # Map to speed range
            if self.bidirectional:
                self._current_speed = value  # Already -1.0 to 1.0
            else:
                self._current_speed = (value + 1.0) / 2.0  # Map to 0.0 to 1.0
        except Exception:
            # If reading fails, keep current value
            pass

