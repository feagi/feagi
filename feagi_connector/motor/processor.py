"""
Motor Command Processing

Handles processing of motor commands from FEAGI and maps them to device actions.
"""

import logging
import struct
from typing import Dict, Any, Optional, Callable

logger = logging.getLogger(__name__)


class MotorProcessor:
    """Process motor commands from FEAGI and execute device actions."""
    
    def __init__(self):
        """Initialize motor processor."""
        self.device_handlers = {}
        self.last_motor_data = None
        
        # Register default device handlers
        self._register_default_handlers()
        
        logger.info("🎮 Motor processor initialized")
    
    def _register_default_handlers(self):
        """Register default device handlers."""
        self.device_handlers.update({
            "motor": self._handle_motor_device,
            "servo": self._handle_servo_device,
            "led": self._handle_led_device,
            "buzzer": self._handle_buzzer_device
        })
    
    def register_device_handler(self, device_type: str, handler: Callable):
        """Register a custom device handler."""
        self.device_handlers[device_type] = handler
        logger.info(f"🔧 Registered custom handler for {device_type}")
    
    async def process_motor_commands(self, motor_data: Dict[str, Any], capabilities_manager):
        """Process motor commands and execute device actions."""
        try:
            self.last_motor_data = motor_data
            output_caps = capabilities_manager.get_output_capabilities()
            
            for cortical_id, neuron_data in motor_data.items():
                if not neuron_data:
                    continue
                
                # Map cortical ID to device
                device_mapping = capabilities_manager.map_cortical_id_to_device(
                    cortical_id, output_caps
                )
                
                if device_mapping:
                    device_type, device_id, config = device_mapping
                    await self._process_device_commands(
                        device_type, device_id, config, neuron_data
                    )
                else:
                    logger.debug(f"🤷 No device mapping for cortical area: {cortical_id}")
                    
        except Exception as e:
            logger.error(f"❌ Error processing motor commands: {e}")
    
    async def _process_device_commands(self, device_type: str, device_id: str, config: Dict, neuron_data: Dict):
        """Process commands for a specific device."""
        try:
            handler = self.device_handlers.get(device_type)
            if handler:
                await handler(device_id, config, neuron_data)
            else:
                logger.warning(f"⚠️  No handler for device type: {device_type}")
                
        except Exception as e:
            logger.error(f"❌ Error processing {device_type} device {device_id}: {e}")
    
    async def _handle_motor_device(self, device_id: str, config: Dict, neuron_data: Dict):
        """Handle motor device commands."""
        try:
            # Calculate average activation as motor power
            if neuron_data:
                total_activation = sum(neuron_data.values())
                avg_activation = total_activation / len(neuron_data)
                
                # Scale to motor power range
                max_power = config.get("max_power", 100)
                motor_power = avg_activation * max_power
                
                # Apply direction based on neuron positions
                direction = self._determine_motor_direction(neuron_data)
                final_power = motor_power * direction
                
                logger.info(
                    f"🛞 Motor {device_id} ({config.get('custom_name', device_id)}): "
                    f"power={final_power:.1f}, neurons={len(neuron_data)}"
                )
                
                # Here you would send the command to actual hardware
                # For example: await self.send_motor_command(device_id, final_power)
                
        except Exception as e:
            logger.error(f"❌ Motor device error: {e}")
    
    async def _handle_servo_device(self, device_id: str, config: Dict, neuron_data: Dict):
        """Handle servo device commands."""
        try:
            if neuron_data:
                # Use neuron position to determine servo angle
                total_weighted_position = 0
                total_activation = 0
                
                for (x, y, z), activation in neuron_data.items():
                    # Use x-coordinate as position indicator
                    total_weighted_position += x * activation
                    total_activation += activation
                
                if total_activation > 0:
                    avg_position = total_weighted_position / total_activation
                    
                    # Map to servo angle range
                    min_angle = config.get("min_angle", 0)
                    max_angle = config.get("max_angle", 180)
                    servo_angle = min_angle + (avg_position / 10.0) * (max_angle - min_angle)
                    
                    # Clamp to valid range
                    servo_angle = max(min_angle, min(max_angle, servo_angle))
                    
                    logger.info(
                        f"🔄 Servo {device_id} ({config.get('custom_name', device_id)}): "
                        f"angle={servo_angle:.1f}°, neurons={len(neuron_data)}"
                    )
                    
                    # Here you would send the command to actual hardware
                    # For example: await self.send_servo_command(device_id, servo_angle)
                    
        except Exception as e:
            logger.error(f"❌ Servo device error: {e}")
    
    async def _handle_led_device(self, device_id: str, config: Dict, neuron_data: Dict):
        """Handle LED device commands."""
        try:
            if neuron_data:
                # Calculate brightness from neuron activations
                max_activation = max(neuron_data.values())
                brightness = int(max_activation * 255)  # 0-255 brightness
                
                logger.info(
                    f"💡 LED {device_id} ({config.get('custom_name', device_id)}): "
                    f"brightness={brightness}/255, neurons={len(neuron_data)}"
                )
                
                # Here you would send the command to actual hardware
                # For example: await self.send_led_command(device_id, brightness)
                
        except Exception as e:
            logger.error(f"❌ LED device error: {e}")
    
    async def _handle_buzzer_device(self, device_id: str, config: Dict, neuron_data: Dict):
        """Handle buzzer device commands."""
        try:
            if neuron_data:
                # Use activation level to determine buzzer frequency/volume
                max_activation = max(neuron_data.values())
                
                if max_activation > 0.5:  # Threshold for activation
                    frequency = int(200 + max_activation * 800)  # 200-1000 Hz
                    duration = max_activation * 0.5  # 0-0.5 seconds
                    
                    logger.info(
                        f"🔊 Buzzer {device_id} ({config.get('custom_name', device_id)}): "
                        f"freq={frequency}Hz, duration={duration:.2f}s"
                    )
                    
                    # Here you would send the command to actual hardware
                    # For example: await self.send_buzzer_command(device_id, frequency, duration)
                    
        except Exception as e:
            logger.error(f"❌ Buzzer device error: {e}")
    
    def _determine_motor_direction(self, neuron_data: Dict) -> float:
        """Determine motor direction based on neuron spatial distribution."""
        if not neuron_data:
            return 1.0
        
        # Simple heuristic: use y-coordinate to determine direction
        # Positive y = forward, negative y = backward
        forward_activation = 0
        backward_activation = 0
        
        for (x, y, z), activation in neuron_data.items():
            if y > 0:
                forward_activation += activation
            elif y < 0:
                backward_activation += activation
        
        if forward_activation > backward_activation:
            return 1.0  # Forward
        elif backward_activation > forward_activation:
            return -1.0  # Backward
        else:
            return 1.0  # Default forward
    
    def get_last_motor_data(self) -> Optional[Dict]:
        """Get the last processed motor data."""
        return self.last_motor_data 