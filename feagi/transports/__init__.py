"""
FEAGI Transport Layer

Provides different transport mechanisms for connecting agents to robots and FEAGI.
"""

from .base import BaseTransport

__all__ = ["BaseTransport"]



def _missing_transport(name: str, extra: str, error: ImportError):
    """Placeholder for a transport whose optional dependency is missing.

    Raises a clear error when used, naming the extra to install, instead of
    leaving ``None`` to be called.
    """
    missing = getattr(error, "name", None) or str(error)
    message = f"{name} needs feagi-core[{extra}] (missing: {missing})"

    class _Missing:
        def __init__(self, *args, **kwargs):
            raise ImportError(message)

    _Missing.__name__ = name
    _Missing.__qualname__ = name
    return _Missing


# Optional imports based on available dependencies
try:
    from .bluetooth_direct import BluetoothTransport
    __all__.append("BluetoothTransport")
except ImportError as _err:
    BluetoothTransport = _missing_transport("BluetoothTransport", "bluetooth", _err)

try:
    from .websocket_relay import WebSocketTransport
    __all__.append("WebSocketTransport")
except ImportError as _err:
    WebSocketTransport = _missing_transport("WebSocketTransport", "bluetooth", _err)

try:
    from .websocket_client import WebSocketClientTransport
    __all__.append("WebSocketClientTransport")
except ImportError as _err:
    WebSocketClientTransport = _missing_transport(
        "WebSocketClientTransport", "bluetooth", _err
    )
