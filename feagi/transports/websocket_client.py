"""
WebSocket Client Transport for BLE Relay

Connects to platform-managed BLE relay (Tauri, Electron, etc.)
Python controller acts as WebSocket CLIENT, not server.

Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
"""

import asyncio
import logging
from typing import Optional
from .base import BaseTransport

try:
    import websockets
    from websockets.client import connect
except ImportError:
    raise ImportError(
        "websockets package is required for WebSocket client transport. "
        "Install with: pip install websockets>=12.0"
    )

logger = logging.getLogger(__name__)


class WebSocketClientTransport(BaseTransport):
    """
    WebSocket client transport for connecting to BLE relay.
    
    Used when a platform (Tauri, Electron) manages the BLE connection
    and relays data via WebSocket.
    
    Architecture:
        Robot BLE ↔ Platform (Tauri) ↔ WebSocket relay ↔ Python (this) ↔ FEAGI
    """
    
    def __init__(self, config: dict):
        """
        Initialize WebSocket client transport.
        
        Args:
            config: Configuration dict with keys:
                - host: Relay host (e.g., "127.0.0.1")
                - port: Relay port (set by platform)
                - embodiment_id: Optional embodiment identifier
        """
        self.host = config.get("host")
        self.port = config.get("port")
        self.embodiment_id = config.get("embodiment_id", "unknown")
        
        if not self.host or not self.port:
            raise ValueError(
                "WebSocket client transport requires 'host' and 'port' in config. "
                f"Got: host={self.host}, port={self.port}"
            )
        
        self.websocket = None
        self.connected = False
        self.uri = f"ws://{self.host}:{self.port}"
        
        logger.info(f"WebSocket client transport initialized for {self.uri}")
    
    async def connect(self) -> None:
        """
        Connect to the WebSocket relay server.
        
        Raises:
            ConnectionError: If connection fails
        """
        logger.info(f"Connecting to BLE relay at {self.uri}...")
        
        try:
            # Connect to relay server (with timeout)
            self.websocket = await asyncio.wait_for(
                connect(self.uri),
                timeout=5.0
            )
            self.connected = True
            logger.info(f"✅ Connected to BLE relay at {self.uri}")
            
        except asyncio.TimeoutError:
            raise ConnectionError(
                f"Timeout connecting to BLE relay at {self.uri}. "
                "Make sure the platform has started the relay server."
            )
        except Exception as e:
            raise ConnectionError(
                f"Failed to connect to BLE relay at {self.uri}: {e}"
            ) from e
    
    async def disconnect(self) -> None:
        """Disconnect from the WebSocket relay"""
        if self.websocket:
            await self.websocket.close()
            self.connected = False
            logger.info("Disconnected from BLE relay")
    
    async def send(self, data: bytes) -> None:
        """
        Send data to robot via relay.
        
        Args:
            data: Bytes to send to robot
            
        Raises:
            ConnectionError: If not connected
        """
        if not self.connected or not self.websocket:
            raise ConnectionError("Not connected to BLE relay")
        
        try:
            await self.websocket.send(data)
            logger.debug(f"📤 Sent {len(data)} bytes to relay")
        except Exception as e:
            self.connected = False
            raise ConnectionError(f"Failed to send data to relay: {e}") from e
    
    async def receive(self) -> Optional[bytes]:
        """
        Receive data from robot via relay.
        
        Returns:
            Bytes received from robot, or None if no data
            
        Raises:
            ConnectionError: If not connected or connection lost
        """
        if not self.connected or not self.websocket:
            raise ConnectionError("Not connected to BLE relay")
        
        try:
            # Non-blocking receive with short timeout
            data = await asyncio.wait_for(
                self.websocket.recv(),
                timeout=0.01  # 10ms timeout for non-blocking behavior
            )
            
            if isinstance(data, bytes):
                logger.debug(f"📥 Received {len(data)} bytes from relay")
                return data
            elif isinstance(data, str):
                # Convert string to bytes if needed
                return data.encode('utf-8')
            
            return None
            
        except asyncio.TimeoutError:
            # No data available (normal for non-blocking)
            return None
        except websockets.exceptions.ConnectionClosed:
            self.connected = False
            raise ConnectionError("BLE relay connection closed")
        except Exception as e:
            self.connected = False
            raise ConnectionError(f"Error receiving from relay: {e}") from e
    
    def is_connected(self) -> bool:
        """Check if connected to relay"""
        return self.connected and self.websocket is not None

