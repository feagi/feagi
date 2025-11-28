"""
Bluetooth Robot Agent

Simplified base class for Bluetooth-enabled robots.
Handles all transport complexity - developers only implement robot protocol.
"""

import asyncio
import logging
import os
import socket
import sys
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
        platform: Optional[str] = None,
        capabilities: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize Bluetooth robot agent.
        
        Args:
            agent_id: Unique identifier for this robot
            feagi_host: FEAGI server host (default: localhost)
            feagi_port: FEAGI server port (default: 3000)
            platform: Override platform detection ("desktop" or "cloud")
            capabilities: Agent capabilities (sensors, motors, etc.)
        """
        super().__init__(agent_id, feagi_host, capabilities)
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
        
        Detection strategy (in order):
        1. FEAGI_PLATFORM env var (explicit override)
        2. FEAGI_BLE_RELAY_ENABLED=true → cloud (platform manages BLE)
        3. Check if BLE relay port is listening → cloud
        4. Default → desktop (Python manages BLE directly)
        
        Returns:
            "desktop" - Python owns BLE connection (direct bleak)
            "cloud" - Platform owns BLE (WebSocket relay)
        """
        # 1. Check explicit platform override
        platform_env = os.getenv("FEAGI_PLATFORM", "").lower()
        if platform_env in ("desktop", "cloud"):
            logger.info(f"Platform explicitly set via FEAGI_PLATFORM: {platform_env}")
            return platform_env
        
        # 2. Check if BLE relay is enabled by platform (Tauri, Electron, etc.)
        if os.getenv("FEAGI_BLE_RELAY_ENABLED", "").lower() in ("true", "1", "yes"):
            logger.info("BLE relay enabled by platform - using cloud mode")
            return "cloud"
        
        # 3. Check if BLE relay port is listening
        if os.getenv("FEAGI_BLE_RELAY_PORT"):
            try:
                relay_port = int(os.getenv("FEAGI_BLE_RELAY_PORT"))
                if self._is_port_listening(relay_port):
                    logger.info(f"BLE relay detected on port {relay_port} - using cloud mode")
                    return "cloud"
            except ValueError:
                logger.warning(f"Invalid FEAGI_BLE_RELAY_PORT: {os.getenv('FEAGI_BLE_RELAY_PORT')}")
        
        # 4. Default to desktop (Python manages BLE directly)
        logger.info("No BLE relay detected - using desktop mode (Python direct BLE)")
        return "desktop"
    
    def _is_port_listening(self, port: int) -> bool:
        """
        Check if a port is listening (relay server is running).
        
        Args:
            port: Port number to check
            
        Returns:
            True if port is listening, False otherwise
        """
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.1)
                result = s.connect_ex(('127.0.0.1', port))
                return result == 0
        except Exception as e:
            logger.debug(f"Port check failed for {port}: {e}")
            return False
    
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
        """
        Initialize WebSocket connection to platform BLE relay.
        
        Configuration comes ONLY from environment variables (set by platform).
        Controllers do NOT access FEAGI's config file.
        
        Required environment variables:
        - FEAGI_BLE_RELAY_PORT: Relay WebSocket port (set by Tauri/platform)
        - FEAGI_BLE_RELAY_HOST: Relay host (optional, defaults to 127.0.0.1)
        
        NO FALLBACKS - explicit configuration required.
        
        Note: This uses WebSocketClientTransport (connects TO relay as client),
        not WebSocketTransport (starts server FOR browsers).
        """
        from feagi.transports import WebSocketClientTransport
        
        # Get relay config from environment (set by Tauri/platform)
        relay_host = os.getenv("FEAGI_BLE_RELAY_HOST", "127.0.0.1")
        relay_port_str = os.getenv("FEAGI_BLE_RELAY_PORT")
        
        if not relay_port_str:
            raise RuntimeError(
                "BLE relay mode selected but FEAGI_BLE_RELAY_PORT not set.\n"
                "Platform (Tauri/Electron) must set this environment variable when launching controller.\n"
                "Example: FEAGI_BLE_RELAY_PORT=49152"
            )
        
        try:
            relay_port = int(relay_port_str)
        except ValueError as e:
            raise RuntimeError(
                f"Invalid FEAGI_BLE_RELAY_PORT: must be integer, got '{relay_port_str}'"
            ) from e
        
        ws_config = {
            "host": relay_host,
            "port": relay_port,
            "embodiment_id": self.agent_id
        }
        
        logger.info(
            f"BLE relay client transport initialized: "
            f"ws://{ws_config['host']}:{ws_config['port']}"
        )
        
        self.transport = WebSocketClientTransport(ws_config)
    
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
            print("\n🔌 Connecting to BLE relay...")
            sys.stdout.flush()
            await self.transport.connect()
            print("✅ BLE relay connected")
            sys.stdout.flush()
            
            # Connect to FEAGI
            print(f"🧠 Connecting to FEAGI at {self.feagi_host}:{self.feagi_port}...")
            sys.stdout.flush()
            await self.connect()
            print("✅ FEAGI connected - Agent registered")
            sys.stdout.flush()
            
            print(f"\n🚀 {self.__class__.__name__} ready! Starting main loop...\n")
            sys.stdout.flush()
            
            # Main loop
            while self.running:
                try:
                    # Receive raw data from robot (with timeout)
                    try:
                        raw_data = await asyncio.wait_for(
                            self.transport.receive(),
                            timeout=0.1
                        )
                    except asyncio.TimeoutError:
                        raw_data = None
                    
                    # Parse robot-specific protocol (subclass implements)
                    if raw_data:
                        sensor_data = self.parse_sensors(raw_data)
                        if sensor_data:
                            # Send to FEAGI (synchronous!)
                            self.client.send_sensory_data(sensor_data)
                    
                    # Get motor commands from FEAGI (synchronous!)
                    motor_data = self.client.receive_motor_data()
                    
                    if motor_data:
                        logger.info(f"📥 Received motor data: {motor_data}")
                        # Format for robot (subclass implements)
                        robot_command = self.format_motors(motor_data)
                        logger.info(f"📤 Formatted command: {robot_command}")
                        
                        if robot_command:
                            # Send to robot
                            await self.transport.send(robot_command)
                            logger.info(f"✅ Sent {len(robot_command)} bytes to robot")
                
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
            self.client.disconnect()  # Synchronous, not async!
        
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

