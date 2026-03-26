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
        The servo reads values from FEAGI: Percentage (0-100%) for PositionalServo
        or SignedPercentage (-1.0 to 1.0) for RotaryMotor, and maps to your angle range.
    """
    
    def __init__(
        self,
        range: Tuple[float, float] = (0, 180),
        encoding: MotorEncoding = "absolute",
        gain: float = 1.0,
        unit_id: int = 0,
        channel_index: Optional[int] = None,
    ):
        super().__init__(unit_id)
        self.min_angle, self.max_angle = range
        self.encoding = encoding
        self.channel = 0
        self.gain = gain  # Amplification factor for motor commands
        self.preferred_group_id = unit_id
        self.preferred_channel_index = channel_index

        # Current angle (from FEAGI)
        self._current_angle: float = (self.min_angle + self.max_angle) / 2
        # Contract-level semantic controls used by both sim and real adapters.
        self.control_semantics: str = "normalized_position"
        self.absolute_command_scale: float = 1.0
        self.incremental_command_scale: float = 1.0
        # Incremental mode uses normalized delta-per-update.
        self.incremental_step_ratio: float = 0.05
        # Diagnostics state for controller-side logging
        self._last_rx_raw_value: Optional[float] = None
        self._last_rx_value: Optional[float] = None
        self._last_rx_mode: Optional[str] = None
        # Monotonic command sequence used by controllers to detect new events.
        self._rx_command_seq: int = 0
    
    @classmethod
    def register(
        cls,
        range: Tuple[float, float] = (0, 180),
        encoding: MotorEncoding = "absolute",
        gain: float = 1.0,
        unit_id: int = 0,
        channel_index: Optional[int] = None,
    ) -> 'ServoMotor':
        """
        Register a new servo motor output.
        
        Args:
            range: (min_angle, max_angle) in degrees
            encoding: "absolute" or "incremental"
            gain: Amplification factor for motor commands (default: 1.0).
                Use >1.0 to amplify weak signals,
                <1.0 to dampen strong signals.
            unit_id: Cortical unit index (motor group) for this servo
            channel_index: Optional channel override within the motor group
        
        Returns:
            ServoMotor instance
        """
        from feagi.pns import brain_output
        
        servo = cls(range, encoding, gain, unit_id, channel_index)
        brain_output.register_output(servo)
        return servo
    
    def get_angle(self) -> float:
        """
        Get current servo angle from FEAGI.
        
        Returns:
            Angle in degrees
        """
        return self._current_angle
    
    def _register_with_cache(
        self,
        cache,
        group_id: int,
        channel_index: int = None,
        decoder_registered: bool = False,
    ):
        """Register callback with Rust MotorDeviceCache."""
        try:
            import feagi_rust_py_libs as frpl

            # Use provided channel_index if given, otherwise use self.channel
            if channel_index is not None:
                self.channel = channel_index

            # Only register callback if decoder has been registered
            if decoder_registered:
                # Register callback to receive motor commands from FEAGI
                cache.register_callback(
                    motor_unit=(
                        (
                            frpl.data_structures.genomic
                            .MotorCorticalType.PositionalServo
                        )
                    ),
                    group=group_id,
                    channel=self.channel,
                    callback=self._on_motor_command,
                )

            self.group_id = group_id
        except Exception as e:
            raise RuntimeError(f"Failed to register ServoMotor: {e}") from e

    def _on_motor_command(
        self,
        value: float,
        command_mode: Optional[str] = None,
    ):
        """Callback invoked when FEAGI sends a motor command.

        Value may be:
        - Percentage (0.0 to 1.0): PositionalServo uses unsigned 0-100%
        - SignedPercentage (-1.0 to 1.0): RotaryMotor uses signed
        """
        import logging
        logger = logging.getLogger(__name__)
        
        # Normalize to float: support PyPercentage (0-1) or PySignedPercentage (-1 to 1)
        raw_value = value
        is_percentage = False
        if not isinstance(value, (int, float)):
            if hasattr(value, "get_as_0_1"):
                raw_value = float(value.get_as_0_1())
                is_percentage = True
            elif hasattr(value, "get_as_m1_1"):
                raw_value = float(value.get_as_m1_1())
                is_percentage = False
            else:
                raw_value = float(value)
        value = raw_value

        # Apply gain
        value = value * self.gain
        if is_percentage:
            value = max(0.0, min(1.0, value))
        else:
            value = max(-1.0, min(1.0, value))

        center = (self.min_angle + self.max_angle) / 2.0
        half_range = (self.max_angle - self.min_angle) / 2.0
        
        # For PositionalServo: The Rust decoder outputs different value ranges:
        # - Incremental: 0.0-1.0 with 0.5 as neutral (from incremental cortical area)
        # - Absolute: 0.0-1.0 as position (from absolute cortical area)
        # Use command_mode if provided, otherwise use self.encoding
        effective_mode = command_mode if command_mode is not None else self.encoding
        # Persist latest RX for controller-side diagnostics.
        self._last_rx_raw_value = float(raw_value)
        self._last_rx_value = float(value)
        self._last_rx_mode = effective_mode
        self._rx_command_seq += 1

        # Apply contract-defined scaling only for incremental updates.
        # Absolute mode is treated as a direct fixed-position target.
        if effective_mode == "incremental":
            scale = max(0.0, float(self.incremental_command_scale))
            if is_percentage:
                value = 0.5 + ((value - 0.5) * scale)
                value = max(0.0, min(1.0, value))
            else:
                value = max(-1.0, min(1.0, value * scale))
        
        old_angle = self._current_angle
        
        if effective_mode == "incremental":
            step = half_range * self.incremental_step_ratio
            if is_percentage:
                delta = (value - 0.5) * 2.0 * step
            else:
                delta = value * step
            next_angle = self._current_angle + delta
            self._current_angle = max(
                self.min_angle,
                min(self.max_angle, next_angle),
            )
        else:
            if is_percentage:
                self._current_angle = self.min_angle + (self.max_angle - self.min_angle) * value
            else:
                self._current_angle = center + (value * half_range)

        # DETAILED LOGGING for debugging
        print(f"[SERVO-DEBUG] Channel {self.channel} received value {raw_value:.4f}", flush=True)
        logger.info(
            f"[SERVO] Ch={self.channel}, raw={raw_value:.4f}, value={value:.4f}, "
            f"mode={effective_mode}, old_angle={old_angle:.2f}, new_angle={self._current_angle:.2f}, "
            f"delta={self._current_angle - old_angle:.4f}"
        )

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
        unit_id: int = 0,
        channel_index: Optional[int] = None,
    ):
        super().__init__(unit_id)
        self.encoding = encoding
        self.bidirectional = bidirectional
        self.channel = 0
        self.preferred_group_id = unit_id
        self.preferred_channel_index = channel_index
        # Contract-level semantic controls used by both sim and real adapters.
        self.control_semantics: str = "normalized_velocity"
        self.absolute_command_scale: float = 1.0
        self.incremental_command_scale: float = 1.0
        
        # Current speed (from FEAGI)
        # Range: -1.0 to 1.0 if bidirectional, 0.0 to 1.0 if not
        self._current_speed: float = 0.0
        # Diagnostics state for controller-side logging
        self._last_rx_raw_value: Optional[float] = None
        self._last_rx_value: Optional[float] = None
        self._last_rx_mode: Optional[str] = None
        # Monotonic command sequence used by controllers to detect new events.
        self._rx_command_seq: int = 0
    
    @classmethod
    def register(
        cls,
        encoding: MotorEncoding = "absolute",
        bidirectional: bool = True,
        unit_id: int = 0,
        channel_index: Optional[int] = None,
    ) -> 'RotaryMotor':
        """
        Register a new rotary motor output.
        
        Args:
            encoding: "absolute" or "incremental"
            bidirectional: Whether motor can reverse
            unit_id: Cortical unit index (motor group) for this motor
            channel_index: Optional channel override within the motor group
        
        Returns:
            RotaryMotor instance
        """
        from feagi.pns import brain_output
        
        motor = cls(encoding, bidirectional, unit_id, channel_index)
        brain_output.register_output(motor)
        return motor
    
    def get_speed(self) -> float:
        """
        Get current motor speed from FEAGI.
        
        Returns:
            Speed: -1.0 to 1.0 (bidirectional) or 0.0 to 1.0 (unidirectional)
        """
        return self._current_speed
    
    def _register_with_cache(
        self,
        cache,
        group_id: int,
        channel_index: int = None,
        decoder_registered: bool = False,
    ):
        """Register callback with Rust MotorDeviceCache."""
        try:
            import feagi_rust_py_libs as frpl

            # Use provided channel_index if given, otherwise use self.channel
            if channel_index is not None:
                self.channel = channel_index

            # Only register callback if decoder has been registered
            if decoder_registered:
                # Register callback to receive motor commands from FEAGI
                cache.register_callback(
                    motor_unit=(
                        (
                            frpl.data_structures.genomic
                            .MotorCorticalType.RotaryMotor
                        )
                    ),
                    group=group_id,
                    channel=self.channel,
                    callback=self._on_motor_command,
                )

            self.group_id = group_id
        except Exception as e:
            raise RuntimeError(f"Failed to register RotaryMotor: {e}") from e

    def _on_motor_command(
        self,
        value: float,
        command_mode: Optional[str] = None,
    ):
        """Callback invoked when FEAGI sends a motor command"""
        raw_value = value
        if not isinstance(value, (int, float)):
            if hasattr(value, "get_as_m1_1"):
                raw_value = float(value.get_as_m1_1())
            elif hasattr(value, "get_as_0_1"):
                raw_value = (float(value.get_as_0_1()) * 2.0) - 1.0
            else:
                raw_value = float(value)
        raw_value = float(raw_value)
        effective_mode = command_mode if command_mode is not None else self.encoding
        scale = (
            max(0.0, float(self.incremental_command_scale))
            if effective_mode == "incremental"
            else max(0.0, float(self.absolute_command_scale))
        )
        scaled_value = max(-1.0, min(1.0, raw_value * scale))
        self._last_rx_raw_value = raw_value
        self._last_rx_value = scaled_value
        self._last_rx_mode = effective_mode
        self._rx_command_seq += 1

        # Value is signed percentage (-1.0 to 1.0)
        if self.bidirectional:
            self._current_speed = scaled_value  # Already -1.0 to 1.0
        else:
            self._current_speed = (scaled_value + 1.0) / 2.0  # Map to 0.0 to 1.0
    
    def _read_from_cache(self, cache):
        """No longer needed - callbacks handle updates"""
        pass

