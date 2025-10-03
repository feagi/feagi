"""
FEAGI Connector State Management

Defines connection states and exceptions for FEAGI agent connections.
"""

from .connection import ConnectionState, FeagiAgentError

__all__ = ['ConnectionState', 'FeagiAgentError'] 