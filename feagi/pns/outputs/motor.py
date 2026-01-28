"""
Motor Outputs

Servo motors, rotary motors (DC), stepper motors, etc.
"""

from typing import Tuple, Literal, Optional
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
        encoding: MotorEncoding = "absolute",
        gain: float = 1.0,
        group_id: int = 0,
        channel_index: Optional[int] = None,
    ):
        super().__init__(group_id)
        self.min_angle, self.max_angle = range
        self.encoding = encoding
        self.channel = 0
        self.gain = gain  # Amplification factor for motor commands
        self.preferred_group_id = group_id
        self.preferred_channel_index = channel_index
        
        # Current angle (from FEAGI)
        self._current_angle: float = (self.min_angle + self.max_angle) / 2
    
    @classmethod
    def register(
        cls,
        range: Tuple[float, float] = (0, 180),
        encoding: MotorEncoding = "absolute",
        gain: float = 1.0,
        group_id: int = 0,
        channel_index: Optional[int] = None,
    ) -> 'ServoMotor':
        """
        Register a new servo motor output.
        
        Args:
            range: (min_angle, max_angle) in degrees
            encoding: "absolute" or "incremental"
            gain: Amplification factor for motor commands (default: 1.0)
                  Use >1.0 to amplify weak signals, <1.0 to dampen strong signals
            group_id: Cortical unit index (motor group) for this servo
            channel_index: Optional channel override within the motor group
        
        Returns:
            ServoMotor instance
        """
        from feagi.pns import brain_output
        
        servo = cls(range, encoding, gain, group_id, channel_index)
        brain_output.register_output(servo)
        return servo
    
    def get_angle(self) -> float:
        """
        Get current servo angle from FEAGI.
        
        Returns:
            Angle in degrees
        """
        return self._current_angle
    
    def _register_with_cache(self, cache, group_id: int, channel_index: int = None, decoder_registered: bool = False):
        """Register callback with Rust MotorDeviceCache (decoder must be registered first)"""
        try:
            import feagi_rust_py_libs as frpl
            
            # Use provided channel_index if given, otherwise use self.channel
            if channel_index is not None:
                self.channel = channel_index
            
            # Only register callback if decoder has been registered
            if decoder_registered:
                # Register callback to receive motor commands from FEAGI
                cache.register_callback(
                    motor_unit=frpl.data_structures.genomic.MotorCorticalType.PositionalServo,
                    group=group_id,
                    channel=self.channel,
                    callback=self._on_motor_command
                )
            
            self.group_id = group_id
        except Exception as e:
            raise RuntimeError(f"Failed to register ServoMotor: {e}") from e
    
    def _on_motor_command(self, value: float):
        """Callback invoked when FEAGI sends a motor command"""
        # Value is SignedPercentage (-1.0 to 1.0) from FEAGI
        # Store raw value for debugging
        raw_value = value
        
        # Apply gain to amplify/dampen the signal
        value = value * self.gain
        # Clamp to [-1.0, 1.0] after gain application
        value = max(-1.0, min(1.0, value))
        
        # Map to angle range: -1.0 -> min_angle, 0.0 -> center, 1.0 -> max_angle
        center = (self.min_angle + self.max_angle) / 2.0
        half_range = (self.max_angle - self.min_angle) / 2.0
        self._current_angle = center + (value * half_range)
        
        # Debug logging (only for first motor, every 10th update to avoid spam)
        if self.channel == 0 and abs(raw_value) > 0.001:
            if not hasattr(self, '_debug_counter'):
                self._debug_counter = 0
            self._debug_counter += 1
            if self._debug_counter % 10 == 0:
                print(f"🎯 Motor[0]: raw={raw_value:.4f}, gain={self.gain}, scaled={value:.4f}, angle={self._current_angle:.4f} rad (range=[{self.min_angle:.4f}, {self.max_angle:.4f}] rad)", flush=True)
    
    def _read_from_cache(self, cache):
        """No longer needed - callbacks handle updates"""
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
        bidirectional: bool = True,
        group_id: int = 0,
        channel_index: Optional[int] = None,
    ):
        super().__init__(group_id)
        self.encoding = encoding
        self.bidirectional = bidirectional
        self.channel = 0
        self.preferred_group_id = group_id
        self.preferred_channel_index = channel_index
        
        # Current speed (from FEAGI)
        # Range: -1.0 to 1.0 if bidirectional, 0.0 to 1.0 if not
        self._current_speed: float = 0.0
    
    @classmethod
    def register(
        cls,
        encoding: MotorEncoding = "absolute",
        bidirectional: bool = True,
        group_id: int = 0,
        channel_index: Optional[int] = None,
    ) -> 'RotaryMotor':
        """
        Register a new rotary motor output.
        
        Args:
            encoding: "absolute" or "incremental"
            bidirectional: Whether motor can reverse
            group_id: Cortical unit index (motor group) for this motor
            channel_index: Optional channel override within the motor group
        
        Returns:
            RotaryMotor instance
        """
        from feagi.pns import brain_output
        
        motor = cls(encoding, bidirectional, group_id, channel_index)
        brain_output.register_output(motor)
        return motor
    
    def get_speed(self) -> float:
        """
        Get current motor speed from FEAGI.
        
        Returns:
            Speed: -1.0 to 1.0 (bidirectional) or 0.0 to 1.0 (unidirectional)
        """
        return self._current_speed
    
    def _register_with_cache(self, cache, group_id: int, channel_index: int = None, decoder_registered: bool = False):
        """Register callback with Rust MotorDeviceCache (decoder must be registered first)"""
        try:
            import feagi_rust_py_libs as frpl
            
            # Use provided channel_index if given, otherwise use self.channel
            if channel_index is not None:
                self.channel = channel_index
            
            # Only register callback if decoder has been registered
            if decoder_registered:
                # Register callback to receive motor commands from FEAGI
                cache.register_callback(
                    motor_unit=frpl.data_structures.genomic.MotorCorticalType.RotaryMotor,
                    group=group_id,
                    channel=self.channel,
                    callback=self._on_motor_command
                )
            
            self.group_id = group_id
        except Exception as e:
            raise RuntimeError(f"Failed to register RotaryMotor: {e}") from e
    
    def _on_motor_command(self, value: float):
        """Callback invoked when FEAGI sends a motor command"""
        # Value is signed percentage (-1.0 to 1.0)
        if self.bidirectional:
            self._current_speed = value  # Already -1.0 to 1.0
        else:
            self._current_speed = (value + 1.0) / 2.0  # Map to 0.0 to 1.0
    
    def _read_from_cache(self, cache):
        """No longer needed - callbacks handle updates"""
        pass

