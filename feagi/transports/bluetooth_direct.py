"""
Bluetooth Transport (Direct BLE using bleak)

For desktop/local use - connects directly to Bluetooth devices.
"""

import asyncio
import logging
from typing import Optional
from .base import BaseTransport

try:
    from bleak import BleakClient, BleakScanner
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False
    BleakClient = None
    BleakScanner = None

logger = logging.getLogger(__name__)


class BluetoothTransport(BaseTransport):
    """
    Direct Bluetooth Low Energy transport using bleak.
    
    Works on Windows, macOS, and Linux (with BlueZ).
    Connects directly to BLE devices using platform-native APIs.
    
    Example:
        config = {
            "device_name": "Bittle",
            "service_uuid": "6e400001-b5a3-f393-e0a9-e50e24dcca9e",
            "rx_uuid": "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
            "tx_uuid": "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
        }
        transport = BluetoothTransport(config)
        await transport.connect()
        await transport.send(b"Hello Robot")
        data = await transport.receive()
    """
    
    def __init__(self, config: dict):
        """
        Initialize Bluetooth transport.
        
        Args:
            config: Dictionary with keys:
                - device_name: Name of BLE device to connect to
                - service_uuid: BLE service UUID
                - rx_uuid: RX characteristic UUID (write to robot)
                - tx_uuid: TX characteristic UUID (notifications from robot)
                - timeout: Optional scan timeout (default: 10s)
        """
        if not BLEAK_AVAILABLE:
            raise ImportError(
                "bleak is required for Bluetooth transport. "
                "Install with: pip install 'feagi[bluetooth]'"
            )
        
        super().__init__(config)
        self.device_name = config["device_name"]
        self.service_uuid = config["service_uuid"]
        self.rx_uuid = config["rx_uuid"]
        self.tx_uuid = config["tx_uuid"]
        self.scan_timeout = config.get("timeout", 10.0)
        
        self.client: Optional[BleakClient] = None
        self.rx_queue = asyncio.Queue()
        self._notification_task: Optional[asyncio.Task] = None
    
    async def connect(self) -> None:
        """
        Scan for and connect to the Bluetooth device.
        
        Raises:
            ConnectionError: If device not found or connection fails
        """
        logger.info(f"Scanning for Bluetooth device: {self.device_name}")
        
        # Scan for devices
        devices = await BleakScanner.discover(timeout=self.scan_timeout)
        
        # Find target device
        target_device = None
        for device in devices:
            if device.name and self.device_name.lower() in device.name.lower():
                target_device = device
                break
        
        if not target_device:
            raise ConnectionError(
                f"Bluetooth device '{self.device_name}' not found. "
                f"Scanned {len(devices)} devices."
            )
        
        logger.info(f"Found device: {target_device.name} ({target_device.address})")
        
        # Connect to device
        self.client = BleakClient(target_device.address)
        await self.client.connect()
        
        if not self.client.is_connected:
            raise ConnectionError(f"Failed to connect to {target_device.name}")
        
        logger.info(f"Connected to {target_device.name}")
        
        # Start notifications from TX characteristic
        await self.client.start_notify(self.tx_uuid, self._notification_handler)
        
        self.connected = True
        logger.info("Bluetooth transport ready")
    
    async def disconnect(self) -> None:
        """Close Bluetooth connection"""
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(self.tx_uuid)
            except Exception as e:
                logger.warning(f"Error stopping notifications: {e}")
            
            await self.client.disconnect()
            logger.info("Bluetooth disconnected")
        
        self.connected = False
        self.client = None
    
    async def send(self, data: bytes) -> None:
        """
        Send data to robot via RX characteristic.
        
        Args:
            data: Raw bytes to send
            
        Raises:
            ConnectionError: If not connected
        """
        if not self.connected or not self.client:
            raise ConnectionError("Not connected to Bluetooth device")
        
        await self.client.write_gatt_char(self.rx_uuid, data, response=False)
        logger.debug(f"Sent {len(data)} bytes via Bluetooth")
    
    async def receive(self) -> bytes:
        """
        Receive data from robot (from notification queue).
        
        Returns:
            Raw bytes received from robot
            
        Raises:
            ConnectionError: If not connected
        """
        if not self.connected:
            raise ConnectionError("Not connected to Bluetooth device")
        
        # Wait for data from notification queue
        data = await self.rx_queue.get()
        logger.debug(f"Received {len(data)} bytes via Bluetooth")
        return data
    
    def _notification_handler(self, characteristic, data: bytearray) -> None:
        """
        Handle incoming BLE notifications.
        
        Called by bleak when data arrives from TX characteristic.
        
        Args:
            characteristic: BLE characteristic (unused)
            data: Raw data from robot
        """
        # Put data in queue for receive() to consume
        self.rx_queue.put_nowait(bytes(data))
        
        # Also call callback if set
        if self.on_data_callback:
            self.on_data_callback(bytes(data))

