"""
Mock implementation of zmq.asyncio for testing within FEAGI.
This helps bypass the need for the actual ZMQ asyncio module during tests.
"""

import zmq

# Fake Context class
class Context(zmq.Context):
    """Mock asyncio Context."""
    
    @classmethod
    def instance(cls):
        """Get a singleton instance."""
        return zmq.Context.instance()
    
    def socket(self, socket_type):
        """Return a socket with asyncio support."""
        socket = super().socket(socket_type)
        return Socket(socket)

# Fake Socket wrapper
class Socket:
    """Mock asyncio Socket wrapper."""
    
    def __init__(self, socket):
        self._socket = socket
    
    def bind(self, address):
        """Bind the socket to an address."""
        return self._socket.bind(address)
    
    def connect(self, address):
        """Connect the socket to an address."""
        return self._socket.connect(address)
    
    def close(self, linger=None):
        """Close the socket."""
        if linger is not None:
            self._socket.setsockopt(zmq.LINGER, linger)
        return self._socket.close()
    
    async def send(self, data, flags=0, copy=True, track=False):
        """Async send operation."""
        return self._socket.send(data, flags=flags, copy=copy, track=track)
    
    async def recv(self, flags=0, copy=True, track=False):
        """Async receive operation."""
        return self._socket.recv(flags=flags, copy=copy, track=track)
    
    async def send_multipart(self, msg_parts, flags=0, copy=True, track=False):
        """Async send multipart operation."""
        return self._socket.send_multipart(msg_parts, flags=flags, copy=copy, track=track)
    
    async def recv_multipart(self, flags=0, copy=True, track=False):
        """Async receive multipart operation."""
        return self._socket.recv_multipart(flags=flags, copy=copy, track=track)

class Poller:
    """Mock asyncio Poller."""
    
    def __init__(self):
        self._poller = zmq.Poller()
        
    def register(self, socket, flags):
        """Register a socket with the poller."""
        return self._poller.register(socket, flags)
        
    def unregister(self, socket):
        """Unregister a socket from the poller."""
        return self._poller.unregister(socket)
        
    async def poll(self, timeout=None):
        """Poll for events."""
        return self._poller.poll(timeout) 