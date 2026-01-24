"""
ESP32 Serial Controller

A Python bridge that connects ESP32 controllers to FEAGI via serial communication.

This module implements a serial-to-ZMQ bridge that:
1. Reads JSON messages from ESP32 via serial port
2. Converts to FEAGI neuron_id_potential_pairs format
3. Sends to FEAGI via ZMQ
4. Receives motor commands from FEAGI
5. Converts to JSON and sends to ESP32 via serial

Usage:
    from feagi.agent.esp32 import Esp32SerialController
    
    controller = Esp32SerialController(
        # agent_id must be a base64 AgentDescriptor (48-byte payload)
        agent_id="<agent_descriptor_b64>",
        serial_port="/dev/ttyUSB0",
        baud_rate=115200,
        feagi_host="localhost"
    )
    controller.run()
"""

import json
import logging
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import serial
import serial.tools.list_ports

from feagi.pns import AgentType, FeagiAgentClient

logger = logging.getLogger("feagi.agent.esp32")


class Esp32SerialController:
    """
    ESP32 Serial-to-ZMQ Bridge Controller
    
    Bridges communication between ESP32 firmware (via serial) and FEAGI (via ZMQ).
    
    Attributes:
        agent_id: Unique identifier for this controller
        serial_port: Serial port path (e.g., "/dev/ttyUSB0" or "COM3")
        baud_rate: Serial baud rate (default: 115200)
        feagi_host: FEAGI server hostname or IP
        feagi_registration_port: FEAGI registration port (default: 30001)
        feagi_sensory_port: FEAGI sensory input port (default: 5555)
        feagi_motor_port: FEAGI motor output port (default: 5564)
        heartbeat_interval: Heartbeat interval in seconds (default: 5.0)
    
    Example:
        ```python
        controller = Esp32SerialController(
            # agent_id must be a base64 AgentDescriptor (48-byte payload)
            agent_id="<agent_descriptor_b64>",
            serial_port="/dev/ttyUSB0",
            feagi_host="localhost"
        )
        controller.run()  # Blocks until Ctrl+C
        ```
    """
    
    def __init__(
        self,
        agent_id: str,
        serial_port: Optional[str] = None,
        baud_rate: int = 115200,
        feagi_host: str = "localhost",
        feagi_registration_port: int = 30001,
        feagi_sensory_port: int = 5555,
        feagi_motor_port: int = 5564,
        heartbeat_interval: float = 5.0,
        auto_detect_port: bool = True,
    ):
        """
        Initialize ESP32 Serial Controller
        
        Args:
            agent_id: Unique identifier for this controller instance
            serial_port: Serial port path. If None and auto_detect_port=True, will
                auto-detect.
            baud_rate: Serial communication baud rate (default: 115200)
            feagi_host: FEAGI server hostname or IP address
            feagi_registration_port: FEAGI registration port
            feagi_sensory_port: FEAGI sensory data input port (ZMQ PUSH)
            feagi_motor_port: FEAGI motor command output port (ZMQ SUB)
            heartbeat_interval: Heartbeat interval in seconds (0 to disable)
            auto_detect_port: If True and serial_port is None, auto-detect ESP32
                port.
        """
        self.agent_id = agent_id
        self.baud_rate = baud_rate
        self.feagi_host = feagi_host
        self.feagi_registration_port = feagi_registration_port
        self.feagi_sensory_port = feagi_sensory_port
        self.feagi_motor_port = feagi_motor_port
        self.heartbeat_interval = heartbeat_interval
        
        # Serial port
        if serial_port is None and auto_detect_port:
            serial_port = self._detect_esp32_port()
            if serial_port is None:
                raise RuntimeError(
                    "No ESP32 serial port found. Please specify serial_port manually."
                )
        
        if serial_port is None:
            raise ValueError(
                "serial_port must be provided or auto_detect_port must be True"
            )
        
        self.serial_port = serial_port
        self.serial_connection: Optional[serial.Serial] = None
        
        # FEAGI client
        self.feagi_client: Optional[FeagiAgentClient] = None
        
        # Control flags
        self.running = False
        self._stop_event = threading.Event()
        
        # Statistics
        self.stats = {
            "sensory_messages_sent": 0,
            "motor_messages_received": 0,
            "serial_read_errors": 0,
            "serial_write_errors": 0,
            "feagi_errors": 0,
        }
        
        logger.info(f"ESP32 Serial Controller initialized: {agent_id}")
        logger.info(f"  Serial port: {serial_port} @ {baud_rate} baud")
        logger.info(
            "  FEAGI: %s:%s (sensory), %s (motor)",
            feagi_host,
            feagi_sensory_port,
            feagi_motor_port,
        )
    
    @staticmethod
    def _detect_esp32_port() -> Optional[str]:
        """
        Auto-detect ESP32 serial port
        
        Returns:
            Serial port path if found, None otherwise
        """
        logger.info("Auto-detecting ESP32 serial port...")
        
        # Common ESP32 USB identifiers
        esp32_keywords = [
            "ESP32",
            "CH340",  # Common ESP32 USB-to-serial chip
            "CP210",  # Another common USB-to-serial chip
            "FTDI",   # FTDI USB-to-serial
            "Silicon Labs",  # CP210x manufacturer
        ]
        
        ports = serial.tools.list_ports.comports()
        for port in ports:
            port_description = (port.description or "").upper()
            port_manufacturer = (port.manufacturer or "").upper()
            
            # Check if description or manufacturer contains ESP32 keywords
            for keyword in esp32_keywords:
                if (
                    keyword.upper() in port_description
                    or keyword.upper() in port_manufacturer
                ):
                    logger.info(
                        "  ✓ Found potential ESP32: %s (%s)",
                        port.device,
                        port.description,
                    )
                    return port.device
            
            # Also check common Linux serial port patterns
            if port.device.startswith(("/dev/ttyUSB", "/dev/ttyACM")):
                # If no other criteria match, this might be an ESP32
                logger.info("  ⚠ Found serial port (may be ESP32): %s", port.device)
                return port.device
        
        logger.warning("  ✗ No ESP32 serial port detected")
        return None
    
    def connect_serial(self) -> None:
        """Connect to ESP32 via serial port"""
        if self.serial_connection is not None and self.serial_connection.is_open:
            logger.warning("Serial port already connected")
            return
        
        try:
            logger.info(
                "Connecting to serial port: %s @ %s baud",
                self.serial_port,
                self.baud_rate,
            )
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,  # 1 second timeout for reads
                write_timeout=1.0,
            )
            
            # Give the serial port a moment to initialize
            time.sleep(2.0)
            
            # Flush any existing data
            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()
            
            logger.info("✓ Serial port connected successfully")
            
        except serial.SerialException as e:
            logger.error(f"Failed to connect to serial port: {e}")
            raise RuntimeError(f"Serial connection failed: {e}") from e
    
    def connect_feagi(self) -> None:
        """Connect to FEAGI via ZMQ"""
        if self.feagi_client is not None:
            logger.warning("FEAGI client already connected")
            return
        
        try:
            logger.info(f"Connecting to FEAGI at {self.feagi_host}...")
            
            # Create FEAGI client (BOTH type: sends sensory, receives motor)
            self.feagi_client = FeagiAgentClient(self.agent_id, AgentType.BOTH)
            
            # Configure client
            self.feagi_client.configure(
                feagi_host=self.feagi_host,
                registration_port=self.feagi_registration_port,
                sensory_port=self.feagi_sensory_port,
                motor_port=self.feagi_motor_port,
                heartbeat_interval=self.heartbeat_interval,
            )
            
            # Connect (registers with FEAGI)
            self.feagi_client.connect()
            
            logger.info("✓ FEAGI client connected and registered")
            
        except Exception as e:
            logger.error(f"Failed to connect to FEAGI: {e}")
            raise RuntimeError(f"FEAGI connection failed: {e}") from e
    
    def disconnect_serial(self) -> None:
        """Disconnect from serial port"""
        if self.serial_connection is not None and self.serial_connection.is_open:
            self.serial_connection.close()
            logger.info("Serial port disconnected")
    
    def disconnect_feagi(self) -> None:
        """Disconnect from FEAGI"""
        if self.feagi_client is not None:
            self.feagi_client.disconnect()
            logger.info("FEAGI client disconnected")
    
    def _read_serial_json(self) -> Optional[Dict[str, Any]]:
        """
        Read a complete JSON message from ESP32 serial port
        
        Returns:
            Parsed JSON dictionary if successful, None otherwise
        """
        if self.serial_connection is None or not self.serial_connection.is_open:
            return None
        
        try:
            # Read until newline (ESP32 sends JSON messages ending with \n)
            line = self.serial_connection.readline()
            
            if not line:
                return None
            
            # Decode bytes to string
            try:
                message_str = line.decode('utf-8').strip()
            except UnicodeDecodeError:
                logger.warning("Received invalid UTF-8 data from serial")
                self.stats["serial_read_errors"] += 1
                return None
            
            if not message_str:
                return None
            
            # Parse JSON
            try:
                message = json.loads(message_str)
                return message
            except json.JSONDecodeError as e:
                logger.warning(f"Failed to parse JSON from ESP32: {e}")
                logger.debug(f"  Raw message: {message_str[:100]}...")
                self.stats["serial_read_errors"] += 1
                return None
                
        except serial.SerialException as e:
            logger.error(f"Serial read error: {e}")
            self.stats["serial_read_errors"] += 1
            return None
    
    def _parse_sensory_data(self, message: Dict[str, Any]) -> List[Tuple[int, float]]:
        """
        Parse ESP32 sensory JSON message to FEAGI neuron_id_potential_pairs format
        
        ESP32 format: {"np":[[id,pot],...],"id":"esp32","f":N}
        FEAGI format: List[Tuple[int, float]]  # [(neuron_id, potential), ...]
        
        Args:
            message: Parsed JSON message from ESP32
            
        Returns:
            List of (neuron_id, potential) tuples
        """
        neuron_pairs: List[Tuple[int, float]] = []
        
        # Handle both shortened and full format
        pairs_key = message.get("np") or message.get("neuron_id_potential_pairs")
        
        if pairs_key is None:
            logger.debug("No neuron pairs found in message")
            return neuron_pairs
        
        # Parse pairs array
        if isinstance(pairs_key, list):
            for pair in pairs_key:
                if isinstance(pair, list) and len(pair) >= 2:
                    try:
                        neuron_id = int(pair[0])
                        potential = float(pair[1])
                        neuron_pairs.append((neuron_id, potential))
                    except (ValueError, TypeError) as e:
                        logger.warning(f"Invalid neuron pair format: {pair} - {e}")
                        continue
        
        return neuron_pairs
    
    def _format_motor_command(self, motor_data: Dict[str, Any]) -> Optional[str]:
        """
        Format FEAGI motor command to ESP32 JSON format
        
        FEAGI format: {"motor": {motor_index: power, ...}} from decode_motor_xyzp
        ESP32 format: {"motor_commands":[{"neuron_id":N,"value":V},...]}
        
        Note: FEAGI sends motor indices (0, 1, 2, ...) but ESP32 firmware
        expects neuron IDs from cortical mappings. This bridge maps motor indices
        directly to neuron IDs by assuming sequential mapping:
        - Motor index 0 → First configured GPIO output neuron ID
        - Motor index 1 → Second configured GPIO output neuron ID
        etc.
        
        For proper mapping, the ESP32 firmware configuration should ensure
        GPIO outputs are mapped to sequential neuron IDs starting from a known base.
        
        Args:
            motor_data: Motor command dictionary from FEAGI (format: {"motor": {...}})
            
        Returns:
            JSON string for ESP32, or None if no commands
        """
        motor_commands: List[Dict[str, Any]] = []
        
        # FEAGI client returns {"motor": {motor_index: power, ...}}
        if "motor" in motor_data:
            motors = motor_data["motor"]
            if isinstance(motors, dict):
                # Convert motor indices to neuron IDs
                # For now, assume motor index directly maps to neuron ID
                # TODO: Implement proper mapping from GPIO config if needed
                for motor_idx_str, power in motors.items():
                    try:
                        motor_idx = int(motor_idx_str)
                        neuron_id = motor_idx  # Simple mapping: motor index = neuron ID
                        power_float = float(power)
                        
                        motor_commands.append({
                            "neuron_id": neuron_id,
                            "value": power_float
                        })
                    except (ValueError, TypeError):
                        logger.warning(
                            "Invalid motor command: %s=%s",
                            motor_idx_str,
                            power,
                        )
                        continue
        
        if not motor_commands:
            return None
        
        # Format as JSON for ESP32
        message = {"motor_commands": motor_commands}
        return json.dumps(message) + "\n"
    
    def _write_serial_json(self, json_str: str) -> bool:
        """
        Write JSON string to ESP32 serial port
        
        Args:
            json_str: JSON string to send
            
        Returns:
            True if successful, False otherwise
        """
        if self.serial_connection is None or not self.serial_connection.is_open:
            return False
        
        try:
            data_bytes = json_str.encode('utf-8')
            bytes_written = self.serial_connection.write(data_bytes)
            self.serial_connection.flush()  # Ensure data is sent immediately
            
            if bytes_written != len(data_bytes):
                logger.warning(
                    "Partial write: %s/%s bytes",
                    bytes_written,
                    len(data_bytes),
                )
                return False
            
            return True
            
        except serial.SerialException as e:
            logger.error(f"Serial write error: {e}")
            self.stats["serial_write_errors"] += 1
            return False
    
    def _sensory_loop(self) -> None:
        """Main loop: Read from ESP32 serial and send to FEAGI"""
        logger.info("Sensory loop started")
        
        while not self._stop_event.is_set():
            try:
                # Read JSON message from ESP32
                message = self._read_serial_json()
                
                if message is None:
                    # No data available, continue
                    time.sleep(0.01)  # Small delay to avoid busy-waiting
                    continue
                
                # Parse to neuron pairs
                neuron_pairs = self._parse_sensory_data(message)
                
                if not neuron_pairs:
                    continue  # No valid data to send
                
                # Send to FEAGI
                try:
                    self.feagi_client.send_sensory_data(neuron_pairs)
                    self.stats["sensory_messages_sent"] += 1
                    
                    # Log periodically
                    if self.stats["sensory_messages_sent"] % 100 == 0:
                        logger.debug(
                            "Sent %s sensory messages (%s neurons in last message)",
                            self.stats["sensory_messages_sent"],
                            len(neuron_pairs),
                        )
                        
                except Exception as e:
                    logger.error(f"Failed to send sensory data to FEAGI: {e}")
                    self.stats["feagi_errors"] += 1
                    
            except Exception as e:
                logger.error(f"Error in sensory loop: {e}")
                time.sleep(0.1)  # Brief pause before retry
        
        logger.info("Sensory loop stopped")
    
    def _motor_loop(self) -> None:
        """Main loop: Receive from FEAGI and send to ESP32 serial"""
        logger.info("Motor loop started")
        
        while not self._stop_event.is_set():
            try:
                # Receive motor commands from FEAGI (non-blocking)
                motor_data = self.feagi_client.receive_motor_data()
                
                if motor_data is None:
                    # No motor commands available
                    time.sleep(0.01)  # Small delay to avoid busy-waiting
                    continue
                
                # Format for ESP32
                json_str = self._format_motor_command(motor_data)
                
                if json_str is None:
                    continue  # No valid commands to send
                
                # Send to ESP32 via serial
                if self._write_serial_json(json_str):
                    self.stats["motor_messages_received"] += 1
                    
                    # Log periodically
                    if self.stats["motor_messages_received"] % 100 == 0:
                        logger.debug(
                            "Received %s motor messages",
                            self.stats["motor_messages_received"],
                        )
                else:
                    logger.warning("Failed to write motor command to serial")
                    
            except Exception as e:
                logger.error(f"Error in motor loop: {e}")
                time.sleep(0.1)  # Brief pause before retry
        
        logger.info("Motor loop stopped")
    
    def run(self) -> None:
        """
        Start the controller main loop
        
        This will:
        1. Connect to serial port
        2. Connect to FEAGI
        3. Start sensory and motor loops in separate threads
        4. Block until stopped (Ctrl+C)
        """
        logger.info("=" * 60)
        logger.info("ESP32 Serial Controller - Starting")
        logger.info("=" * 60)
        
        try:
            # Connect to serial port
            self.connect_serial()
            
            # Connect to FEAGI
            self.connect_feagi()
            
            # Set running flag
            self.running = True
            self._stop_event.clear()
            
            # Start worker threads
            sensory_thread = threading.Thread(
                target=self._sensory_loop,
                name="ESP32-Sensory",
                daemon=True
            )
            motor_thread = threading.Thread(
                target=self._motor_loop,
                name="ESP32-Motor",
                daemon=True
            )
            
            sensory_thread.start()
            motor_thread.start()
            
            logger.info("✓ Controller running (Press Ctrl+C to stop)")
            logger.info("")
            
            # Main loop - wait for stop event
            try:
                while self.running and not self._stop_event.is_set():
                    time.sleep(1.0)
                    
                    # Print statistics periodically
                    total_events = sum(self.stats.values())
                    if total_events > 0 and total_events % 1000 == 0:
                        total_errors = (
                            self.stats["serial_read_errors"]
                            + self.stats["serial_write_errors"]
                            + self.stats["feagi_errors"]
                        )
                        logger.info(
                            f"Stats: Sensory={self.stats['sensory_messages_sent']}, "
                            f"Motor={self.stats['motor_messages_received']}, "
                            f"Errors={total_errors}"
                        )
                        
            except KeyboardInterrupt:
                logger.info("\nReceived interrupt signal, stopping...")
                self.stop()
                
        except Exception as e:
            logger.error(f"Controller error: {e}")
            raise
        finally:
            # Cleanup
            self.stop()
    
    def stop(self) -> None:
        """Stop the controller gracefully"""
        logger.info("Stopping ESP32 controller...")
        
        self.running = False
        self._stop_event.set()
        
        # Wait a moment for threads to finish
        time.sleep(0.5)
        
        # Disconnect
        self.disconnect_feagi()
        self.disconnect_serial()
        
        # Print final statistics
        logger.info("")
        logger.info("=" * 60)
        logger.info("Controller Statistics:")
        logger.info(f"  Sensory messages sent: {self.stats['sensory_messages_sent']}")
        logger.info(
            "  Motor messages received: %s",
            self.stats["motor_messages_received"],
        )
        logger.info(f"  Serial read errors: {self.stats['serial_read_errors']}")
        logger.info(f"  Serial write errors: {self.stats['serial_write_errors']}")
        logger.info(f"  FEAGI errors: {self.stats['feagi_errors']}")
        logger.info("=" * 60)
        logger.info("Controller stopped")


def main():
    """Command-line entry point for ESP32 controller"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="ESP32 Serial-to-FEAGI Bridge Controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-detect ESP32 port
  python -m feagi.agent.esp32 --agent-id esp32-01
  
  # Specify serial port
  python -m feagi.agent.esp32 --agent-id esp32-01 --serial-port /dev/ttyUSB0
  
  # Custom FEAGI host
  python -m feagi.agent.esp32 --agent-id esp32-01 --feagi-host 192.168.1.100
        """
    )
    
    parser.add_argument(
        "--agent-id",
        required=True,
        help="Unique identifier for this controller instance"
    )
    parser.add_argument(
        "--serial-port",
        default=None,
        help=(
            "Serial port path (e.g., /dev/ttyUSB0 or COM3). "
            "Auto-detects if not specified."
        ),
    )
    parser.add_argument(
        "--baud-rate",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--feagi-host",
        default="localhost",
        help="FEAGI server hostname or IP (default: localhost)"
    )
    parser.add_argument(
        "--feagi-registration-port",
        type=int,
        default=30001,
        help="FEAGI registration port (default: 30001)"
    )
    parser.add_argument(
        "--feagi-sensory-port",
        type=int,
        default=5555,
        help="FEAGI sensory input port (default: 5555)"
    )
    parser.add_argument(
        "--feagi-motor-port",
        type=int,
        default=5564,
        help="FEAGI motor output port (default: 5564)",
    )
    parser.add_argument(
        "--heartbeat-interval",
        type=float,
        default=5.0,
        help="Heartbeat interval in seconds (default: 5.0, set to 0 to disable)"
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)"
    )
    
    args = parser.parse_args()
    
    # Configure logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create and run controller
    controller = Esp32SerialController(
        agent_id=args.agent_id,
        serial_port=args.serial_port,
        baud_rate=args.baud_rate,
        feagi_host=args.feagi_host,
        feagi_registration_port=args.feagi_registration_port,
        feagi_sensory_port=args.feagi_sensory_port,
        feagi_motor_port=args.feagi_motor_port,
        heartbeat_interval=args.heartbeat_interval,
        auto_detect_port=(args.serial_port is None),
    )
    
    controller.run()


if __name__ == "__main__":
    main()

