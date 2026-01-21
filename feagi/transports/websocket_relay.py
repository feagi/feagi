"""
WebSocket Transport (Relay from Browser)

For cloud/NRS use - receives BLE data from browser via WebSocket.
"""

import asyncio
import logging
from typing import Optional
from .base import BaseTransport

try:
    import websockets
    from websockets.server import serve
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    websockets = None
    serve = None

logger = logging.getLogger(__name__)


class WebSocketTransport(BaseTransport):
    """
    WebSocket relay transport for cloud/NRS deployments.
    
    Browser connects to robot via Web Bluetooth API, then relays
    data to this Python controller via WebSocket.
    
    Example:
        config = {
            "host": "127.0.0.1",  # Localhost only by default, use "0.0.0.0" for network deployments
            "port": 9052,
            "embodiment_id": "em-bittle123"
        }
        transport = WebSocketTransport(config)
        await transport.connect()  # Starts WebSocket server
        data = await transport.receive()  # From browser
        await transport.send(b"Command")  # To browser (then robot)
    """
    
    def __init__(self, config: dict):
        """
        Initialize WebSocket transport.
        
        Args:
            config: Dictionary with keys:
                - host: Host to bind to (default: "127.0.0.1" - localhost only, secure by default)
                - port: Port to listen on (default: 9052)
                - embodiment_id: Optional ID for logging
        """
        if not WEBSOCKETS_AVAILABLE:
            raise ImportError(
                "websockets is required for WebSocket transport. "
                "Install with: pip install 'feagi[bluetooth]'"
            )
        
        super().__init__(config)
        self.host = config.get("host", "127.0.0.1")
        self.port = config.get("port", 9052)
        self.embodiment_id = config.get("embodiment_id", "unknown")
        
        self.websocket: Optional[websockets.WebSocketServerProtocol] = None
        self.server: Optional[websockets.WebSocketServer] = None
        self.rx_queue = asyncio.Queue()
        self.tx_queue = asyncio.Queue()
        self._server_task: Optional[asyncio.Task] = None
        self._client_task: Optional[asyncio.Task] = None
    
    async def connect(self) -> None:
        """
        Start WebSocket server and wait for browser to connect.
        
        Raises:
            ConnectionError: If server fails to start
        """
        logger.info(f"Starting WebSocket server on ws://{self.host}:{self.port}")
        
        # Start WebSocket server
        self.server = await serve(self._handle_client, self.host, self.port)
        
        logger.info(f"WebSocket server started, waiting for browser connection...")
        self.connected = True
    
    async def disconnect(self) -> None:
        """Close WebSocket server and client connection"""
        if self.websocket:
            await self.websocket.close()
            self.websocket = None
        
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            self.server = None
        
        if self._client_task:
            self._client_task.cancel()
            self._client_task = None
        
        self.connected = False
        logger.info("WebSocket transport closed")
    
    async def send(self, data: bytes) -> None:
        """
        Send data to browser (which forwards to robot via BLE).
        
        Args:
            data: Raw bytes to send
            
        Raises:
            ConnectionError: If browser not connected
        """
        if not self.websocket:
            raise ConnectionError("Browser not connected to WebSocket")
        
        await self.websocket.send(data)
        logger.debug(f"Sent {len(data)} bytes to browser via WebSocket")
    
    async def receive(self) -> bytes:
        """
        Receive data from browser (relayed from robot via BLE).
        
        Returns:
            Raw bytes received from robot
            
        Raises:
            ConnectionError: If browser not connected
        """
        if not self.websocket and not self.connected:
            raise ConnectionError("Browser not connected to WebSocket")
        
        # Wait for data from browser
        data = await self.rx_queue.get()
        logger.debug(f"Received {len(data)} bytes from browser via WebSocket")
        return data
    
    async def _handle_client(self, websocket):
        """
        Handle incoming WebSocket connection from browser.
        
        Args:
            websocket: WebSocket connection
        """
        logger.info(f"Browser connected from {websocket.remote_address}")
        self.websocket = websocket
        
        try:
            # Listen for messages from browser
            async for message in websocket:
                # Browser sends raw BLE bytes
                if isinstance(message, bytes):
                    data = message
                elif isinstance(message, str):
                    # Handle text messages (capabilities, ping, etc.)
                    import json
                    try:
                        msg_data = json.loads(message)
                        if msg_data.get("type") == "ping":
                            await websocket.send(json.dumps({"type": "pong"}))
                            continue
                        elif msg_data.get("type") == "capabilities":
                            logger.info(f"Received capabilities: {msg_data}")
                            continue
                        # Otherwise treat as embodiment data
                        embodiment_data = msg_data.get(self.embodiment_id, {})
                        data_str = embodiment_data.get("data", "")
                        data = data_str.encode() if isinstance(data_str, str) else data_str
                    except json.JSONDecodeError:
                        data = message.encode()
                else:
                    logger.warning(f"Unknown message type: {type(message)}")
                    continue
                
                # Put data in queue for receive() to consume
                self.rx_queue.put_nowait(data)
                
                # Also call callback if set
                if self.on_data_callback:
                    self.on_data_callback(data)
        
        except websockets.exceptions.ConnectionClosed:
            logger.info("Browser disconnected")
        finally:
            self.websocket = None

