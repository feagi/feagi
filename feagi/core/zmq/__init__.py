"""Core ZMQ implementation for FEAGI.

This module contains the internal ZMQ implementation details.
The public API is exposed through the top-level feagi.zmq module.
"""
from .server import ZMQServer

__all__ = ["ZMQServer"]
