"""
ZeroMQ Sensorimotor Stream Implementation for FEAGI API

This module implements the specialized streaming pattern for sensorimotor data.
It provides:
- Efficient binary serialization for high-performance data exchange
- Optimized transfer of sensory inputs and motor outputs
- Direct streaming between FEAGI and peripherals
"""

import asyncio
import logging
import time
import uuid
import zlib
import struct
from typing import Dict, Any, List, Optional, Set, Tuple, Union, Callable

import zmq
import zmq.asyncio
import numpy as np

from ...core.service import CoreApiService
from ..serialization import serialize_message, deserialize_message
from ...utils.rate_limit import RateLimiter

logger = logging.getLogger(__name__)


class SensorimotorStream:
    """
    ZeroMQ Sensorimotor Stream implementation.
    
    This specialized stream efficiently transfers sensorimotor data between
    FEAGI and peripherals, using binary serialization for high performance.
    """
    
    def __init__(
        self, 
        core_api: CoreApiService,
        host: str = "*", 
        port: int = 5558,
        compression_level: int = 0,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize a new Sensorimotor Stream.
        
        Args:
            core_api: The CoreApiService instance to delegate calls to
            host: Host address to bind to
            port: Port for data streaming
            compression_level: Compression level (0-9, 0=none)
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.compression_level = compression_level
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        
        # Router socket for bidirectional communication
        self.socket = self.context.socket(zmq.ROUTER)
        self.socket.bind(f"tcp://{host}:{port}")
        
        # Connected clients and their configurations
        self.clients = {}
        
        # Rate limiter for throttling high-frequency data
        self.rate_limiter = RateLimiter()
        
        # Periodic task references
        self.periodic_tasks = {}

    async def start(self) -> None:
        """Start the sensorimotor stream server."""
        logger.info(f"Starting Sensorimotor Stream server on {self.host}:{self.port}")
        self.running = True
        
        # Store the current event loop for this method
        self._event_loop = asyncio.get_event_loop()
        
        # Start request handler in the current loop
        self.periodic_tasks["request_handler"] = self._event_loop.create_task(
            self._handle_requests()
        )

    async def stop(self) -> None:
        """Stop the sensorimotor stream server."""
        logger.info("Stopping Sensorimotor Stream server")
        self.running = False
        
        # Cancel all periodic tasks
        for task_name, task in self.periodic_tasks.items():
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    logger.debug(f"Cancelled periodic task: {task_name}")
        
        # Close the socket
        self.socket.close()

    async def _handle_requests(self) -> None:
        """Main loop for handling client requests."""
        while self.running:
            try:
                # Wait for a message (client_id, empty delimiter, message parts)
                multipart = await self.socket.recv_multipart()
                
                if len(multipart) < 3:
                    logger.error(f"Received malformed message: {multipart}")
                    continue
                
                client_id = multipart[0].decode()
                command = multipart[2].decode()
                
                logger.debug(f"Received command {command} from client {client_id}")
                
                # Process based on command
                if command == "register":
                    await self._handle_registration(client_id, multipart[3:])
                elif command == "unregister":
                    await self._handle_unregistration(client_id)
                elif command == "sensory_data":
                    await self._handle_sensory_data(client_id, multipart[3:])
                elif command == "motor_query":
                    await self._handle_motor_query(client_id, multipart[3:])
                else:
                    logger.warning(f"Unknown command: {command}")
                    await self._send_error(client_id, f"Unknown command: {command}")
            
            except asyncio.CancelledError:
                logger.debug("Request handler cancelled")
                break
            except Exception as e:
                logger.error(f"Error handling request: {e}")
                await asyncio.sleep(1)  # Avoid tight loop on errors

    async def _handle_registration(self, client_id: str, data_parts: List[bytes]) -> None:
        """
        Handle client registration.
        
        Args:
            client_id: Client identifier
            data_parts: Additional data parts
        """
        if len(data_parts) < 1:
            await self._send_error(client_id, "Missing configuration data")
            return
            
        try:
            # Parse configuration
            config = deserialize_message(data_parts[0], "application/json")
            
            # Store client configuration
            self.clients[client_id] = {
                "config": config,
                "last_active": time.time(),
                "data_format": config.get("data_format", "binary")
            }
            
            logger.info(f"Registered sensorimotor client: {client_id}")
            
            # Send acknowledgement
            await self._send_response(client_id, "register_ack", {
                "status": "ok",
                "client_id": client_id,
                "server_time": time.time()
            })
            
        except Exception as e:
            logger.error(f"Error during client registration: {e}")
            await self._send_error(client_id, f"Registration error: {str(e)}")

    async def _handle_unregistration(self, client_id: str) -> None:
        """
        Handle client unregistration.
        
        Args:
            client_id: Client identifier
        """
        if client_id in self.clients:
            del self.clients[client_id]
            logger.info(f"Unregistered sensorimotor client: {client_id}")
            
        # Send acknowledgement
        await self._send_response(client_id, "unregister_ack", {
            "status": "ok"
        })

    async def _handle_sensory_data(self, client_id: str, data_parts: List[bytes]) -> None:
        """
        Handle incoming sensory data from a client.
        
        Args:
            client_id: Client identifier
            data_parts: Additional data parts containing sensory data
        """
        if len(data_parts) < 2:
            await self._send_error(client_id, "Missing sensory data parts")
            return
            
        try:
            # Parse data type and sensory data
            data_type = data_parts[0].decode()
            data = data_parts[1]
            
            if client_id in self.clients:
                # Update last active timestamp
                self.clients[client_id]["last_active"] = time.time()
            
            # Process the sensory data through the core API
            # This would typically feed into the brain's sensory cortical areas
            result = await self.core_api.process_sensory_data(client_id, data_type, data)
            
            # Send acknowledgement
            await self._send_response(client_id, "sensory_ack", {
                "status": "ok",
                "timestamp": time.time()
            })
            
        except Exception as e:
            logger.error(f"Error processing sensory data: {e}")
            await self._send_error(client_id, f"Sensory data error: {str(e)}")

    async def _handle_motor_query(self, client_id: str, data_parts: List[bytes]) -> None:
        """
        Handle motor data query from a client.
        
        Args:
            client_id: Client identifier
            data_parts: Additional data parts containing query parameters
        """
        if len(data_parts) < 1:
            await self._send_error(client_id, "Missing motor query parameters")
            return
            
        try:
            # Parse query parameters
            params = deserialize_message(data_parts[0], "application/json")
            
            if client_id in self.clients:
                # Update last active timestamp
                self.clients[client_id]["last_active"] = time.time()
            
            # Fetch motor data from the core API
            # This would typically come from the brain's motor cortical areas
            motor_areas = params.get("motor_areas", [])
            motor_data = await self.core_api.get_motor_data(client_id, motor_areas)
            
            # Send motor data response
            motor_data_bytes = serialize_message(motor_data, "application/octet-stream")
            await self._send_response(client_id, "motor_data", motor_data_bytes, binary=True)
            
        except Exception as e:
            logger.error(f"Error handling motor query: {e}")
            await self._send_error(client_id, f"Motor query error: {str(e)}")

    async def _send_response(
        self, 
        client_id: str, 
        response_type: str, 
        data: Any, 
        binary: bool = False
    ) -> None:
        """
        Send a response to a client.
        
        Args:
            client_id: Client identifier
            response_type: Type of response
            data: Response data
            binary: Whether data is binary
        """
        try:
            if binary:
                # For binary data, assume data is already serialized
                await self.socket.send_multipart([
                    client_id.encode(),
                    b"",
                    response_type.encode(),
                    data
                ])
            else:
                # For other data, serialize as JSON
                json_data = serialize_message(data, "application/json")
                await self.socket.send_multipart([
                    client_id.encode(),
                    b"",
                    response_type.encode(),
                    json_data
                ])
                
        except Exception as e:
            logger.error(f"Error sending response to client {client_id}: {e}")

    async def _send_error(self, client_id: str, error_message: str) -> None:
        """
        Send an error response to a client.
        
        Args:
            client_id: Client identifier
            error_message: Error message
        """
        await self._send_response(client_id, "error", {
            "error": error_message,
            "timestamp": time.time()
        })

    async def broadcast_motor_data(
        self, 
        motor_data: Dict[str, Any], 
        target_clients: Optional[List[str]] = None
    ) -> None:
        """
        Broadcast motor data to clients.
        
        Args:
            motor_data: Motor data to broadcast
            target_clients: Specific clients to target, or None for all
        """
        clients_to_send = target_clients or list(self.clients.keys())
        
        for client_id in clients_to_send:
            if client_id in self.clients:
                try:
                    # Serialize and compress if needed
                    data_format = self.clients[client_id].get("data_format", "binary")
                    
                    if data_format == "binary":
                        # Binary format with optional compression
                        data_bytes = self._encode_data(motor_data, self.compression_level)
                    else:
                        # Default to JSON
                        data_bytes = serialize_message(motor_data, "application/json")
                    
                    # Send to client
                    await self._send_response(client_id, "motor_update", data_bytes, binary=True)
                    
                except Exception as e:
                    logger.error(f"Error broadcasting motor data to client {client_id}: {e}")
    
    def _encode_data(self, data: Any, compression_level: int = 0) -> bytes:
        """
        Encode data to binary format with optional compression.
        
        Args:
            data: Data to encode
            compression_level: Compression level (0-9, 0=none)
            
        Returns:
            Binary encoded data
        """
        # Serialize based on data type
        if isinstance(data, dict):
            # Handle dictionary of numpy arrays or other values
            encoded_parts = []
            
            # Encode dictionary structure
            header = {}
            for key, value in data.items():
                if isinstance(value, np.ndarray):
                    # Store array metadata
                    header[key] = {
                        "type": "ndarray",
                        "shape": value.shape,
                        "dtype": str(value.dtype),
                        "offset": len(encoded_parts)
                    }
                    # Add array data
                    encoded_parts.append(value.tobytes())
                else:
                    # Store regular value directly
                    header[key] = {
                        "type": "value",
                        "value": value
                    }
            
            # Encode header as JSON
            header_bytes = json.dumps(header).encode()
            
            # Combine header and data parts
            header_size = struct.pack(">I", len(header_bytes))
            binary_data = header_size + header_bytes
            
            for part in encoded_parts:
                part_size = struct.pack(">I", len(part))
                binary_data += part_size + part
                
        elif isinstance(data, np.ndarray):
            # Handle single numpy array
            array_data = data.tobytes()
            shape_str = json.dumps(data.shape).encode()
            dtype_str = str(data.dtype).encode()
            
            shape_size = struct.pack(">H", len(shape_str))
            dtype_size = struct.pack(">H", len(dtype_str))
            
            binary_data = shape_size + shape_str + dtype_size + dtype_str + array_data
            
        elif isinstance(data, bytes):
            # Already binary
            binary_data = data
            
        else:
            # Fallback to JSON for other types
            binary_data = json.dumps(data).encode()
        
        # Apply compression if requested
        if compression_level > 0:
            compressed_data = zlib.compress(binary_data, compression_level)
            # Add compression marker and level
            marker = struct.pack(">BB", 1, compression_level)
            return marker + compressed_data
        else:
            # Add uncompressed marker
            marker = struct.pack(">BB", 0, 0)
            return marker + binary_data
            

class SensorimotorClient:
    """
    ZeroMQ Sensorimotor Client implementation.
    
    This client connects to a SensorimotorStream server to exchange
    sensory and motor data.
    """
    
    def __init__(
        self, 
        host: str = "localhost", 
        port: int = 5558,
        context: Optional[zmq.asyncio.Context] = None,
        timeout: float = 5.0
    ):
        """
        Initialize a new Sensorimotor Client.
        
        Args:
            host: Server host address to connect to
            port: Server port to connect to
            context: Optional existing ZMQ context to use
            timeout: Request timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.context = context or zmq.asyncio.Context.instance()
        
        # Dealer socket for bidirectional communication
        self.socket = self.context.socket(zmq.DEALER)
        self.client_id = str(uuid.uuid4())
        self.socket.setsockopt(zmq.IDENTITY, self.client_id.encode())
        self.socket.connect(f"tcp://{host}:{port}")
        
        # Client configuration
        self.config = {
            "data_format": "binary"
        }
        
        # Running state
        self.running = False
        
        # Data format
        self.data_format = "binary"
        
        # Callbacks for different message types
        self.callbacks = {}
        
        # Motor data cache
        self.motor_data_cache = {}
        self.last_motor_update = 0

    async def start(self) -> None:
        """Start the sensorimotor client."""
        logger.info(f"Starting Sensorimotor client to {self.host}:{self.port}")
        self.running = True
        
        # Register with the server
        await self._register()
        
        # Start response handler
        asyncio.create_task(self._handle_responses())

    async def stop(self) -> None:
        """Stop the sensorimotor client."""
        logger.info("Stopping Sensorimotor client")
        
        # Unregister from server
        try:
            await self._unregister()
        except:
            pass
            
        self.running = False
        self.socket.close()

    async def _register(self) -> bool:
        """
        Register with the sensorimotor server.
        
        Returns:
            True if registration was successful
        """
        json_data = serialize_message(self.config, "application/json")
        await self.socket.send_multipart([
            b"",
            b"register",
            json_data
        ])
        
        # Wait for acknowledgement
        try:
            self.socket.setsockopt(zmq.RCVTIMEO, int(self.timeout * 1000))
            response = await self.socket.recv_multipart()
            
            if len(response) >= 2 and response[0].decode() == "register_ack":
                logger.info("Registered with sensorimotor server")
                return True
            else:
                logger.error(f"Unexpected registration response: {response}")
                return False
                
        except zmq.error.Again:
            logger.error("Registration timed out")
            return False

    async def _unregister(self) -> None:
        """Unregister from the sensorimotor server."""
        await self.socket.send_multipart([
            b"",
            b"unregister"
        ])

    async def _handle_responses(self) -> None:
        """Handle responses from the server."""
        while self.running:
            try:
                multipart = await self.socket.recv_multipart()
                
                if len(multipart) < 2:
                    logger.error(f"Received malformed response: {multipart}")
                    continue
                
                response_type = multipart[0].decode()
                data = multipart[1]
                
                logger.debug(f"Received {response_type} response")
                
                # Process based on response type
                if response_type == "motor_data" or response_type == "motor_update":
                    await self._handle_motor_data(data)
                elif response_type == "error":
                    logger.error(f"Error from server: {data.decode()}")
                
                # Call registered callback if any
                if response_type in self.callbacks:
                    try:
                        await self.callbacks[response_type](data)
                    except Exception as e:
                        logger.error(f"Error in callback for {response_type}: {e}")
            
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error handling response: {e}")
                await asyncio.sleep(1)

    async def _handle_motor_data(self, data: bytes) -> None:
        """
        Handle motor data from the server.
        
        Args:
            data: Binary motor data
        """
        # Decode the data
        try:
            motor_data = self._decode_data(data)
            
            # Update cache
            self.motor_data_cache = motor_data
            self.last_motor_update = time.time()
            
        except Exception as e:
            logger.error(f"Error decoding motor data: {e}")

    async def send_sensory_data(
        self, 
        data_type: str, 
        data: Union[np.ndarray, bytes, Dict[str, Any]]
    ) -> bool:
        """
        Send sensory data to the server.
        
        Args:
            data_type: Type of sensory data (e.g., "vision", "touch")
            data: Sensory data to send
            
        Returns:
            True if data was sent successfully
        """
        try:
            # Encode the data
            if isinstance(data, (np.ndarray, dict)):
                binary_data = self._encode_data(data)
            elif isinstance(data, bytes):
                binary_data = data
            else:
                binary_data = str(data).encode()
            
            # Send to server
            await self.socket.send_multipart([
                b"",
                b"sensory_data",
                data_type.encode(),
                binary_data
            ])
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending sensory data: {e}")
            return False

    async def get_motor_data(
        self, 
        motor_areas: Optional[List[str]] = None
    ) -> Dict[str, Any]:
        """
        Get motor data from the server.
        
        Args:
            motor_areas: List of motor areas to query, or None for all
            
        Returns:
            Dictionary of motor data
        """
        # Send motor query
        params = {"motor_areas": motor_areas or []}
        json_data = serialize_message(params, "application/json")
        
        await self.socket.send_multipart([
            b"",
            b"motor_query",
            json_data
        ])
        
        # Use cached data while waiting for response
        await asyncio.sleep(0.01)  # Small delay to allow response to be processed
        
        return self.motor_data_cache

    def register_callback(self, response_type: str, callback: Callable) -> None:
        """
        Register a callback for a specific response type.
        
        Args:
            response_type: Type of response to register for
            callback: Async function to call when response is received
        """
        self.callbacks[response_type] = callback

    def _encode_data(self, data: Any) -> bytes:
        """
        Encode data to binary format.
        
        Args:
            data: Data to encode
            
        Returns:
            Binary encoded data
        """
        # Import here to avoid circular references
        import json
        
        if isinstance(data, np.ndarray):
            # Encode numpy array
            array_data = data.tobytes()
            shape_str = json.dumps(data.shape).encode()
            dtype_str = str(data.dtype).encode()
            
            shape_size = struct.pack(">H", len(shape_str))
            dtype_size = struct.pack(">H", len(dtype_str))
            
            return shape_size + shape_str + dtype_size + dtype_str + array_data
            
        elif isinstance(data, dict):
            # Convert dictionary to JSON
            return json.dumps(data).encode()
            
        else:
            # Default to string representation
            return str(data).encode()

    def _decode_data(self, binary_data: bytes) -> Any:
        """
        Decode binary data.
        
        Args:
            binary_data: Binary data to decode
            
        Returns:
            Decoded data
        """
        # Import here to avoid circular references
        import json
        
        # Check for compression
        if len(binary_data) >= 2:
            is_compressed, comp_level = struct.unpack(">BB", binary_data[:2])
            data = binary_data[2:]
            
            if is_compressed:
                data = zlib.decompress(data)
        else:
            # Too short, return as is
            return binary_data
            
        try:
            # Try to decode as JSON
            return json.loads(data.decode())
        except:
            # Not JSON, try other formats
            try:
                # Check if it's a numpy array
                if len(data) >= 4:
                    shape_size = struct.unpack(">H", data[:2])[0]
                    if 2 + shape_size + 2 <= len(data):
                        shape_str = data[2:2+shape_size]
                        shape = json.loads(shape_str.decode())
                        
                        dtype_size_pos = 2 + shape_size
                        dtype_size = struct.unpack(">H", data[dtype_size_pos:dtype_size_pos+2])[0]
                        
                        if dtype_size_pos + 2 + dtype_size <= len(data):
                            dtype_str = data[dtype_size_pos+2:dtype_size_pos+2+dtype_size].decode()
                            array_data = data[dtype_size_pos+2+dtype_size:]
                            
                            return np.frombuffer(array_data, dtype=dtype_str).reshape(shape)
            except:
                pass
                
            # Return as binary
            return data 