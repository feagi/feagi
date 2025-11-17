"""FEAGI API Module.

This module provides the main API interface for FEAGI, including both REST and
ZMQ endpoints.
"""

import os

# Check if embedded mode is enabled
EMBEDDED_MODE = os.environ.get("FEAGI_EMBEDDED_MODE", "0") == "1"

if not EMBEDDED_MODE:
    from feagi.api.rest import create_rest_app
else:
    # In embedded mode, provide a stub function
    def create_rest_app(*args, **kwargs):
        raise RuntimeError("REST API is disabled in embedded mode")


# Core API exports
from feagi.api.core.services import CoreAPIService

# Gateway exports
from feagi.api.gateway import APIGateway, get_api_gateway

# REST API exports
from feagi.api.rest import create_rest_app


# ═══════════════════════════════════════════════════════════════════════
# DEPRECATED ZMQ STUBS - All functionality migrated to Rust PNS
# ═══════════════════════════════════════════════════════════════════════
# All ZMQ functionality has been migrated to Rust and is now available via:
#     feagi_rust.PyPNS - Rust Peripheral Nervous System
#
# These stubs exist only for backward compatibility with legacy code.
# ═══════════════════════════════════════════════════════════════════════

import warnings


class ZmqClient:
    """Deprecated stub - ZMQ functionality migrated to Rust PNS (feagi_rust.PyPNS)."""
    
    def __init__(self, *args, **kwargs):
        warnings.warn(
            "ZmqClient is deprecated. All ZMQ functionality has been migrated to "
            "Rust PNS (feagi_rust.PyPNS).",
            DeprecationWarning,
            stacklevel=2
        )
        self._deprecated = True


def create_zmq_client(*args, **kwargs):
    """Deprecated: Use Rust PNS (feagi_rust.PyPNS) instead."""
    raise DeprecationWarning(
        "create_zmq_client() is deprecated. "
        "All ZMQ functionality has been migrated to Rust PNS (feagi_rust.PyPNS)."
    )


def create_zmq_server(*args, **kwargs):
    """Deprecated: Use Rust PNS (feagi_rust.PyPNS) instead."""
    raise DeprecationWarning(
        "create_zmq_server() is deprecated. "
        "All ZMQ functionality has been migrated to Rust PNS (feagi_rust.PyPNS). "
        "Use ProcessManager to initialize the Rust PNS automatically."
    )


__all__ = [
    "CoreAPIService",
    "APIGateway",
    "get_api_gateway",
    "create_rest_app",
    "ZmqClient",  # Deprecated stub
    "create_zmq_server",  # Deprecated stub
    "create_zmq_client",  # Deprecated stub
]
