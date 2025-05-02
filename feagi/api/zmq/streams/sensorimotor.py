"""Sensorimotor data stream implementation for FEAGI ZeroMQ interface."""

import logging
import threading
import struct
import time
import numpy as np
from typing import Dict, Any, Optional, Callable, List, Tuple, Union

import zmq

logger = logging.getLogger(__name__)

class SensorimotorStream:
    """
    Sensorimotor data stream for FEAGI ZeroMQ interface.
    
    This class provides optimized binary serialization for sensorimotor data,
    used for high-performance streaming between FEAGI and peripherals.
    """
    
    def __init__(
        self,
        context: zmq.Context,
        is_server: bool = True,
        host: str = "127.0.0.1",
        push_port: int = 5557,
        pull_port: int = 5558,
        compression_level: int = 0,
    ):
        """
        Initialize the sensorimotor stream.
        
        Args:
            context: ZeroMQ context.
            is_server: Whether this is a server or client.
            host: Host address to bind/connect to.
            push_port: Port for pushing data.
            pull_port: Port for pulling data.
            compression_level: Compression level (0-9, 0 = no compression).
        """
        self.context = context
        self.is_server = is_server
        self.host = host
        self.push_port = push_port
        self.pull_port = pull_port
        self.compression_level = compression_level
        
        self.push_socket = None
        self.pull_socket = None
        self.running = False
        self.pull_thread = None
        
        # Callbacks
        self.data_callbacks: List[Callable[[bytes, str], None]] = []
        
    def start(self):
        """Start the sensorimotor stream."""
        if self.running:
            logger.warning("Sensorimotor stream already running")
            return
            
        # Setup sockets
        self._setup_push_socket()
        self._setup_pull_socket()
        
        # Start pull thread
        self._start_pull_thread()
        
        self.running = True
        logger.info(f"{'Server' if self.is_server else 'Client'} sensorimotor stream started")
        
    def stop(self):
        """Stop the sensorimotor stream."""
        self.running = False
        
        if self.pull_thread:
            self.pull_thread.join(timeout=2.0)
            
        if self.push_socket:
            self.push_socket.close()
            
        if self.pull_socket:
            self.pull_socket.close()
            
        logger.info(f"{'Server' if self.is_server else 'Client'} sensorimotor stream stopped")
        
    def _setup_push_socket(self):
        """Set up the push socket."""
        self.push_socket = self.context.socket(zmq.PUSH if self.is_server else zmq.PULL)
        
        if self.is_server:
            self.push_socket.bind(f"tcp://{self.host}:{self.push_port}")
            logger.info(f"PUSH socket bound to tcp://{self.host}:{self.push_port}")
        else:
            self.push_socket.connect(f"tcp://{self.host}:{self.push_port}")
            logger.info(f"PULL socket connected to tcp://{self.host}:{self.push_port}")
            
    def _setup_pull_socket(self):
        """Set up the pull socket."""
        self.pull_socket = self.context.socket(zmq.PULL if self.is_server else zmq.PUSH)
        
        if self.is_server:
            self.pull_socket.bind(f"tcp://{self.host}:{self.pull_port}")
            logger.info(f"PULL socket bound to tcp://{self.host}:{self.pull_port}")
        else:
            self.pull_socket.connect(f"tcp://{self.host}:{self.pull_port}")
            logger.info(f"PUSH socket connected to tcp://{self.host}:{self.pull_port}")
            
    def _start_pull_thread(self):
        """Start a thread to handle pull socket."""
        self.pull_thread = threading.Thread(target=self._pull_handler)
        self.pull_thread.daemon = True
        self.pull_thread.start()
        
    def _pull_handler(self):
        """Handle pull socket messages."""
        poller = zmq.Poller()
        poller.register(self.pull_socket if self.is_server else self.push_socket, zmq.POLLIN)
        
        socket = self.pull_socket if self.is_server else self.push_socket
        
        while self.running:
            try:
                # Wait for a message with timeout
                socks = dict(poller.poll(1000))
                if socket in socks and socks[socket] == zmq.POLLIN:
                    # Receive multipart message
                    multipart = socket.recv_multipart()
                    
                    # Process message
                    if len(multipart) >= 2:
                        data_type = multipart[0].decode()
                        data = multipart[1]
                        
                        # Call callbacks
                        self._call_data_callbacks(data, data_type)
                    else:
                        logger.warning(f"Received invalid multipart message: {multipart}")
                    
            except zmq.ZMQError as e:
                logger.error(f"ZMQ error in pull handler: {e}")
            except Exception as e:
                logger.exception(f"Error in pull handler: {e}")
                
    def _call_data_callbacks(self, data: bytes, data_type: str):
        """
        Call registered callbacks for data.
        
        Args:
            data: Binary data.
            data_type: Type of data.
        """
        for callback in self.data_callbacks:
            try:
                callback(data, data_type)
            except Exception as e:
                logger.exception(f"Error in data callback: {e}")
                
    def register_data_callback(self, callback: Callable[[bytes, str], None]):
        """
        Register a callback for data.
        
        Args:
            callback: Callback function that takes the data and type as arguments.
        """
        self.data_callbacks.append(callback)
        
    def unregister_data_callback(self, callback: Callable[[bytes, str], None]):
        """
        Unregister a callback for data.
        
        Args:
            callback: Callback function to unregister.
        """
        if callback in self.data_callbacks:
            self.data_callbacks.remove(callback)
            
    def send_data(self, data: Union[bytes, np.ndarray], data_type: str) -> bool:
        """
        Send data.
        
        Args:
            data: Data to send (either binary or numpy array).
            data_type: Type of data.
            
        Returns:
            True if successful, False otherwise.
        """
        if not (self.push_socket if self.is_server else self.pull_socket):
            logger.error("Socket not initialized")
            return False
            
        try:
            # Convert numpy array to binary if needed
            binary_data = self._encode_data(data)
            
            # Compress data if enabled
            if self.compression_level > 0:
                import zlib
                binary_data = zlib.compress(binary_data, self.compression_level)
                
            # Send multipart message
            socket = self.push_socket if self.is_server else self.pull_socket
            socket.send_multipart([
                data_type.encode(),
                binary_data
            ])
            
            return True
        except zmq.ZMQError as e:
            logger.error(f"ZMQ error in send_data: {e}")
            return False
        except Exception as e:
            logger.exception(f"Error in send_data: {e}")
            return False
            
    def _encode_data(self, data: Union[bytes, np.ndarray]) -> bytes:
        """
        Encode data to binary.
        
        Args:
            data: Data to encode.
            
        Returns:
            Binary data.
        """
        if isinstance(data, bytes):
            return data
        elif isinstance(data, np.ndarray):
            # Encode numpy array with shape and dtype
            shape = data.shape
            dtype = str(data.dtype)
            
            # Create header with shape and dtype
            header = struct.pack(">I", len(shape))  # Number of dimensions
            for dim in shape:
                header += struct.pack(">I", dim)  # Size of each dimension
            
            # Add dtype length and dtype string
            dtype_bytes = dtype.encode()
            header += struct.pack(">I", len(dtype_bytes))
            header += dtype_bytes
            
            # Flatten array and convert to bytes
            data_bytes = data.tobytes()
            
            return header + data_bytes
        else:
            raise TypeError(f"Unsupported data type: {type(data)}")
            
    def decode_data(self, binary_data: bytes) -> Union[bytes, np.ndarray]:
        """
        Decode binary data.
        
        Args:
            binary_data: Binary data to decode.
            
        Returns:
            Decoded data.
        """
        try:
            # Check if data is compressed
            if self.compression_level > 0:
                import zlib
                binary_data = zlib.decompress(binary_data)
                
            # Check if data is a numpy array
            if len(binary_data) > 4:
                # Try to parse header
                offset = 0
                dims = struct.unpack(">I", binary_data[offset:offset+4])[0]
                offset += 4
                
                # If valid header, decode numpy array
                if dims <= 10:  # Sanity check
                    shape = []
                    for i in range(dims):
                        shape.append(struct.unpack(">I", binary_data[offset:offset+4])[0])
                        offset += 4
                        
                    dtype_len = struct.unpack(">I", binary_data[offset:offset+4])[0]
                    offset += 4
                    
                    dtype = binary_data[offset:offset+dtype_len].decode()
                    offset += dtype_len
                    
                    # Reconstruct numpy array
                    array_data = binary_data[offset:]
                    return np.frombuffer(array_data, dtype=dtype).reshape(shape)
            
            # If not a numpy array or parsing failed, return raw bytes
            return binary_data
        except Exception as e:
            logger.exception(f"Error decoding data: {e}")
            return binary_data 