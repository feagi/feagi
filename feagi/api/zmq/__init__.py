"""Deprecated ZMQ module - all functionality moved to Rust PNS.

This module only exists for backward compatibility. All ZMQ functionality
has been migrated to Rust and is now available through:
    
    feagi_rust.PyPNS - Rust Peripheral Nervous System (PNS)

The Rust PNS handles:
- Agent registration (via ZMQ REST stream)
- Sensory data ingestion (via SHM + Rust burst engine)
- Motor data output (via ZMQ PUB + SHM)
- Visualization data output (via ZMQ PUB + SHM)
- Heartbeat tracking

No Python ZMQ server or client should be used going forward.
"""

import warnings


class ZmqClient:
    """Deprecated stub class for ZmqClient.
    
    This class exists only for backward compatibility with code that imports it.
    All actual ZMQ functionality has been migrated to Rust PNS.
    """
    
    def __init__(self, *args, **kwargs):
        warnings.warn(
            "ZmqClient is deprecated. All ZMQ functionality has been migrated to "
            "Rust PNS (feagi_rust.PyPNS).",
            DeprecationWarning,
            stacklevel=2
        )
        # Store args for compatibility but do nothing
        self._deprecated = True


def create_zmq_client(*args, **kwargs):
    """Deprecated: Use Rust PNS (feagi_rust.PyPNS) instead."""
    raise DeprecationWarning(
        "Python ZMQ client is deprecated. "
        "All ZMQ functionality has been migrated to Rust PNS (feagi_rust.PyPNS)."
    )


def create_zmq_server(*args, **kwargs):
    """Deprecated: Use Rust PNS (feagi_rust.PyPNS) instead."""
    raise DeprecationWarning(
        "Python ZMQ server is deprecated. "
        "All ZMQ functionality has been migrated to Rust PNS (feagi_rust.PyPNS). "
        "Use ProcessManager to initialize the Rust PNS automatically."
    )


__all__ = [
    "ZmqClient",
    "create_zmq_client",
    "create_zmq_server",
]
