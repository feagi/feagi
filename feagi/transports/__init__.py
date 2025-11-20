"""
FEAGI Transport Layer

Provides different transport mechanisms for connecting agents to robots and FEAGI.
"""

from .base import BaseTransport

__all__ = ["BaseTransport"]

# Optional imports based on available dependencies
try:
    from .bluetooth_direct import BluetoothTransport
    __all__.append("BluetoothTransport")
except ImportError:
    BluetoothTransport = None

try:
    from .websocket_relay import WebSocketTransport
    __all__.append("WebSocketTransport")
except ImportError:
    WebSocketTransport = None

