"""
Base classes and utilities for FEAGI protocols.

This module provides the foundational classes for the FEAGI communication 
protocols, enabling version management and protocol negotiation.
"""

import enum
import importlib
import os
import pkgutil
import sys
from typing import Dict, Any, List, Optional, Type, Union, Set, Tuple

from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)


class ProtocolID(enum.IntEnum):
    """Enumeration of supported protocol types."""
    FCP = 1  # FEAGI Control Protocol
    FSMP = 2  # FEAGI Sensorimotor Protocol
    FVP = 3  # FEAGI Visualization Protocol


class VersionedProtocol:
    """
    Base class for versioned protocols.
    
    All protocol implementations should inherit from this class and set:
    - PROTOCOL_ID: The ID of the protocol
    - VERSION: The version number
    
    And implement these methods:
    - encode: Convert data structure to binary
    - decode: Convert binary to data structure
    """
    
    PROTOCOL_ID: ProtocolID = None
    VERSION: int = None
    
    @classmethod
    def encode(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode data structure to binary format.
        
        Args:
            data: The data to encode
            
        Returns:
            Encoded binary data
        """
        raise NotImplementedError("encode method must be implemented by subclass")
    
    @classmethod
    def decode(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode binary data to data structure.
        
        Args:
            data: The binary data to decode
            
        Returns:
            Decoded data structure
        """
        raise NotImplementedError("decode method must be implemented by subclass")


class ProtocolRegistry:
    """
    Registry for protocol implementations.
    
    This class maintains a registry of all protocol implementations, 
    allowing lookup by protocol ID and version.
    """
    
    def __init__(self):
        """Initialize the registry."""
        # Registry format: {protocol_id: {version: implementation}}
        self._registry: Dict[ProtocolID, Dict[int, Type[VersionedProtocol]]] = {}
    
    def register(self, protocol_cls: Type[VersionedProtocol]) -> None:
        """
        Register a protocol implementation.
        
        Args:
            protocol_cls: The protocol class to register
        """
        if not issubclass(protocol_cls, VersionedProtocol):
            raise TypeError(f"Protocol class must inherit from VersionedProtocol, got {protocol_cls}")
            
        if protocol_cls.PROTOCOL_ID is None or protocol_cls.VERSION is None:
            raise ValueError(f"Protocol class {protocol_cls.__name__} missing PROTOCOL_ID or VERSION")
            
        # Ensure the protocol ID exists in the registry
        if protocol_cls.PROTOCOL_ID not in self._registry:
            self._registry[protocol_cls.PROTOCOL_ID] = {}
            
        # Register the implementation
        self._registry[protocol_cls.PROTOCOL_ID][protocol_cls.VERSION] = protocol_cls
        logger.debug(f"Registered protocol {protocol_cls.PROTOCOL_ID.name} v{protocol_cls.VERSION}")
    
    def get_protocol(self, protocol_id: ProtocolID, version: int) -> Optional[Type[VersionedProtocol]]:
        """
        Get a specific protocol implementation.
        
        Args:
            protocol_id: Protocol ID
            version: Protocol version
            
        Returns:
            Protocol implementation, or None if not found
        """
        if protocol_id not in self._registry:
            return None
            
        return self._registry[protocol_id].get(version)
    
    def get_latest_version(self, protocol_id: ProtocolID) -> Optional[int]:
        """
        Get the latest version of a protocol.
        
        Args:
            protocol_id: Protocol ID
            
        Returns:
            Latest version, or None if no versions exist
        """
        if protocol_id not in self._registry or not self._registry[protocol_id]:
            return None
            
        return max(self._registry[protocol_id].keys())
    
    def get_compatible_version(self, protocol_id: ProtocolID, 
                            supported_versions: List[int]) -> Optional[int]:
        """
        Find a compatible protocol version.
        
        Args:
            protocol_id: Protocol ID
            supported_versions: List of versions supported by the client
            
        Returns:
            Highest compatible version, or None if none found
        """
        if protocol_id not in self._registry:
            return None
            
        # Find all available versions for this protocol
        available_versions = set(self._registry[protocol_id].keys())
        
        # Find common versions
        common_versions = available_versions.intersection(set(supported_versions))
        
        if not common_versions:
            return None
            
        return max(common_versions)
    
    def list_protocols(self) -> Dict[str, List[int]]:
        """
        List all registered protocols and their versions.
        
        Returns:
            Dictionary mapping protocol names to lists of versions
        """
        result = {}
        
        for protocol_id, versions in self._registry.items():
            protocol_name = protocol_id.name
            result[protocol_name] = sorted(versions.keys())
            
        return result


class ProtocolManager:
    """
    Manager for protocol operations.
    
    This class provides a high-level interface for encoding and decoding
    messages using the appropriate protocol.
    """
    
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        """Singleton pattern implementation."""
        if cls._instance is None:
            cls._instance = super(ProtocolManager, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        """Initialize the protocol manager."""
        if not hasattr(self, 'initialized'):
            self.registry = ProtocolRegistry()
            self._load_protocol_implementations()
            self.initialized = True
    
    def _load_protocol_implementations(self) -> None:
        """Load all protocol implementations from submodules."""
        from feagi.api.protocols import fcp, fsmp, fvp
        
        # Call register_protocols function if available
        for module in [fcp, fsmp, fvp]:
            if hasattr(module, 'register_protocols'):
                module.register_protocols(self.registry)
    
    def encode_message(self, data: Dict[str, Any], protocol_id: ProtocolID, 
                     version: int) -> bytes:
        """
        Encode a message using a specific protocol.
        
        Args:
            data: Data to encode
            protocol_id: Protocol ID
            version: Protocol version
            
        Returns:
            Binary data
            
        Raises:
            ValueError: If the protocol is not supported
        """
        protocol_cls = self.registry.get_protocol(protocol_id, version)
        
        if protocol_cls is None:
            raise ValueError(f"Protocol {protocol_id.name} v{version} is not supported")
            
        # Encode the message
        encoded = protocol_cls.encode(data)
        
        # Prepend protocol header (protocol ID and version)
        header = bytes([protocol_id.value, version])
        
        return header + encoded
    
    def decode_message(self, data: bytes) -> Tuple[Dict[str, Any], ProtocolID, int]:
        """
        Decode a message with protocol header.
        
        Args:
            data: Binary data to decode
            
        Returns:
            Tuple of (decoded message, protocol ID, version)
            
        Raises:
            ValueError: If the protocol is not supported or header is invalid
        """
        if len(data) < 2:
            raise ValueError("Invalid message: too short to contain header")
            
        # Extract protocol header
        protocol_value, version = data[0], data[1]
        message_data = data[2:]
        
        try:
            protocol_id = ProtocolID(protocol_value)
        except ValueError:
            raise ValueError(f"Invalid protocol ID: {protocol_value}")
            
        # Get the protocol implementation
        protocol_cls = self.registry.get_protocol(protocol_id, version)
        
        if protocol_cls is None:
            raise ValueError(f"Protocol {protocol_id.name} v{version} is not supported")
            
        # Decode the message
        decoded = protocol_cls.decode(message_data)
        
        return decoded, protocol_id, version
    
    def get_compatible_version(self, protocol_id: ProtocolID, 
                            supported_versions: List[int]) -> Optional[int]:
        """
        Find a compatible protocol version.
        
        Args:
            protocol_id: Protocol ID
            supported_versions: List of versions supported by the client
            
        Returns:
            Highest compatible version, or None if none found
        """
        return self.registry.get_compatible_version(protocol_id, supported_versions)
    
    def get_latest_version(self, protocol_id: ProtocolID) -> Optional[int]:
        """
        Get the latest version of a protocol.
        
        Args:
            protocol_id: Protocol ID
            
        Returns:
            Latest version, or None if no versions exist
        """
        return self.registry.get_latest_version(protocol_id) 