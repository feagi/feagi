"""
Bluetooth Robot Agent

Simplified base class for Bluetooth-enabled robots.
Handles all transport complexity - developers only implement robot protocol.
"""

import asyncio
import logging
import os
from abc import abstractmethod
from typing import Dict, Any, Optional
from .base import BaseAgent

logger = logging.getLogger(__name__)


class BluetoothRobot(BaseAgent):
    """
    Base class for Bluetooth robots - SIMPLE API!
    
    Developers only need to define 3 things:
    1. BLUETOOTH_CONFIG - Connection UUIDs
    2. parse_sensors() - Robot data → FEAGI format
    3. format_motors() - FEAGI commands → Robot format
    
    Everything else (platform detection, BLE connection, FEAGI integration) 
    is handled automatically!
    
    Example:
        class MyRobot(BluetoothRobot):
            BLUETOOTH_CONFIG = {
                "device_name": "MyRobot",
                "service_uuid": "6e400001-...",
                "rx_uuid": "6e400002-...",
                "tx_uuid": "6e400003-..."
            }
            
            def parse_sensors(self, raw_bytes: bytes) -> dict:
                # Parse robot's BLE data
                return {'gyro': {'0': [x, y, z]}}
            
            def format_motors(self, feagi_output: dict) -> bytes:
                # Format FEAGI commands for robot
                return b'M...'
        
        # That's it! Run it:
        robot = MyRobot("my-robot-001")
        asyncio.run(robot.run())
    """
    
    # Subclasses MUST override this with their Bluetooth config
    BLUETOOTH_CONFIG: Dict[str, str] = {}
    
    def __init__(
        self,
        agent_id: str,
        feagi_host: str = "localhost",
        feagi_port: int = 3000,
        platform: Optional[str] = None
    ):
        """
        Initialize Bluetooth robot agent.
        
        Args:
            agent_id: Unique identifier for this robot
            feagi_host: FEAGI server host (default: localhost)
            feagi_port: FEAGI server port (default: 3000)
            platform: Override platform detection ("desktop" or "cloud")
        """
        super().__init__(agent_id, feagi_host)
        self.feagi_port = feagi_port
        self.platform = platform or self._detect_platform()
        self.transport = None
        self.logger = logger
        
        if not self.BLUETOOTH_CONFIG:
            raise ValueError(
                f"{self.__class__.__name__} must define BLUETOOTH_CONFIG"
            )
        
        logger.info(f"Initializing {self.__class__.__name__} on {self.platform} platform")
    
    def _detect_platform(self) -> str:
        """
        Auto-detect if running on desktop or cloud.
        
        Returns:
            "desktop" or "cloud"
        """
        # Check environment variable
        platform_env = os.getenv("FEAGI_PLATFORM", "").lower()
        if platform_env in ("desktop", "cloud"):
            return platform_env
        
        # Check if running in feagi-desktop (has TAURI env vars)
        if os.getenv("TAURI_PLATFORM") or os.getenv("FEAGI_DESKTOP"):
            return "desktop"
        
        # Check if WebSocket port is listening (cloud indicator)
        # Default to desktop for local development
        return "desktop"
    
    def initialize_hardware(self):
        """
        Initialize Bluetooth transport.
        
        Automatically chooses the right transport based on platform.
        """
        logger.info(f"Initializing Bluetooth transport for {self.platform}")
        
        if self.platform == "desktop":
            self._init_desktop_transport()
        else:
            self._init_cloud_transport()
    
    def _init_desktop_transport(self):
        """Initialize direct BLE connection for desktop"""
        from feagi.transports import BluetoothTransport
        
        self.transport = BluetoothTransport(self.BLUETOOTH_CONFIG)
        logger.info("Desktop Bluetooth transport initialized")
    
    def _init_cloud_transport(self):
        """Initialize WebSocket relay for cloud/NRS"""
        from feagi.transports import WebSocketTransport
        
        ws_config = {
            "host": "0.0.0.0",
            "port": 9052,
            "embodiment_id": self.agent_id
        }
        self.transport = WebSocketTransport(ws_config)
        logger.info("Cloud WebSocket transport initialized")
    
    async def run(self):
        """
        Main robot loop - handles everything automatically!
        
        1. Connects to Bluetooth (desktop) or WebSocket (cloud)
        2. Connects to FEAGI
        3. Runs sensor/motor loop
        4. Handles errors and reconnection
        """
        self.running = True
        self.initialize_hardware()
        
        try:
            # Connect to robot transport
            logger.info("Connecting to robot...")
            await self.transport.connect()
            logger.info("✓ Robot connected")
            
            # Connect to FEAGI
            logger.info(f"Connecting to FEAGI at {self.feagi_host}:{self.feagi_port}...")
            await self.connect()
            logger.info("✓ FEAGI connected")
            
            logger.info(f"{self.__class__.__name__} ready! Starting main loop...")
            
            # Main loop
            while self.running:
                try:
                    # Receive raw data from robot
                    raw_data = await asyncio.wait_for(
                        self.transport.receive(),
                        timeout=1.0
                    )
                    
                    # Parse robot-specific protocol (subclass implements)
                    sensor_data = self.parse_sensors(raw_data)
                    
                    if sensor_data:
                        # Send to FEAGI
                        await self.client.send_sensory_data(sensor_data)
                    
                    # Get motor commands from FEAGI
                    motor_data = await self.client.receive_motor_data()
                    
                    if motor_data:
                        # Format for robot (subclass implements)
                        robot_command = self.format_motors(motor_data)
                        
                        if robot_command:
                            # Send to robot
                            await self.transport.send(robot_command)
                
                except asyncio.TimeoutError:
                    # No data from robot - that's okay, continue
                    continue
                except Exception as e:
                    logger.error(f"Error in main loop: {e}")
                    # Continue running unless critical error
                    await asyncio.sleep(0.1)
        
        except KeyboardInterrupt:
            logger.info("Shutting down (Ctrl+C)")
        except Exception as e:
            logger.error(f"Fatal error: {e}")
            raise
        finally:
            await self.stop()
    
    async def stop(self):
        """
        Stop robot gracefully.
        
        Disconnects from Bluetooth/WebSocket and FEAGI.
        """
        logger.info("Stopping robot...")
        self.running = False
        
        if self.transport:
            await self.transport.disconnect()
        
        if self.client:
            await self.client.disconnect()
        
        logger.info("Robot stopped")
    
    # Abstract methods that subclasses MUST implement
    
    @abstractmethod
    def parse_sensors(self, raw_bytes: bytes) -> Dict[str, Any]:
        """
        Parse robot's raw Bluetooth data into FEAGI format.
        
        This is where you implement your robot's protocol parsing.
        
        Args:
            raw_bytes: Raw data received from robot via Bluetooth
        
        Returns:
            Dictionary with FEAGI sensor data.
            Example: {
                'gyro': {'0': [x, y, z]},
                'proximity': {'0': distance},
                'infrared': {'0': value, '1': value}
            }
        
        Example:
            def parse_sensors(self, raw_bytes):
                # Robot sends: "Gx,y,z#"
                data_str = raw_bytes.decode('utf-8').strip('#')
                if data_str.startswith('G'):
                    x, y, z = map(float, data_str[1:].split(','))
                    return {'gyro': {'0': [x, y, z]}}
                return {}
        """
        pass
    
    @abstractmethod
    def format_motors(self, feagi_output: Dict[str, Any]) -> bytes:
        """
        Format FEAGI motor commands into robot's protocol.
        
        This is where you implement your robot's command formatting.
        
        Args:
            feagi_output: Motor commands from FEAGI.
            Example: {
                'motor': {'0': 50, '1': -30},
                'servo': {'0': 45, '1': 90}
            }
        
        Returns:
            Raw bytes to send to robot via Bluetooth.
            Return empty bytes (b'') if no command to send.
        
        Example:
            def format_motors(self, feagi_output):
                if 'motor' in feagi_output:
                    left = feagi_output['motor'].get('0', 0)
                    right = feagi_output['motor'].get('1', 0)
                    return f"M{left},{right}#".encode()
                return b''
        """
        pass
    
    # BaseAgent required methods (with sensible defaults)
    
    def map_sensors(self, hardware_data: Any) -> Dict[str, bytes]:
        """
        BaseAgent compatibility method.
        
        For BluetoothRobot, sensor mapping is done in parse_sensors().
        """
        return {}
    
    def map_motors(self, feagi_output: Dict[str, Any]) -> Any:
        """
        BaseAgent compatibility method.
        
        For BluetoothRobot, motor mapping is done in format_motors().
        """
        return None

