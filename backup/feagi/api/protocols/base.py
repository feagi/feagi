#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
Base classes for FEAGI binary protocols.

This module provides the foundation for all FEAGI binary protocols with version
support. It defines abstract base classes and common utilities used by the
specific protocol implementations.
"""

from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, ClassVar, Dict, List, Optional, Tuple, Type


class ProtocolID(Enum):
    """Protocol identifiers used in binary messages."""

    FCP = 0x01  # FEAGI Control Protocol
    FVP = 0x02  # FEAGI Visualization Protocol
    FSMP = 0x03  # FEAGI Sensorimotor Protocol


class VersionedProtocol(ABC):
    """Base class for all versioned protocols."""

    # Class variables to be defined in subclasses
    PROTOCOL_ID: ClassVar[ProtocolID]
    VERSION: ClassVar[int]

    @classmethod
    @abstractmethod
    def encode(cls, data: Any) -> bytes:
        """
        Encode internal data structure to binary protocol format.

        Args:
            data: The internal data structure to encode

        Returns:
            Binary protocol data (excluding protocol ID and version header)
        """
        pass

    @classmethod
    @abstractmethod
    def decode(cls, data: bytes) -> Any:
        """
        Decode binary protocol data to internal data structure.

        Args:
            data: Binary protocol data (excluding protocol ID and version header)

        Returns:
            Decoded internal data structure
        """
        pass


class ProtocolRegistry:
    """Registry for protocol implementations by protocol type and version."""

    def __init__(self):
        self._registry: Dict[ProtocolID, Dict[int, Type[VersionedProtocol]]] = {
            protocol_id: {} for protocol_id in ProtocolID
        }

    def register(self, protocol_class: Type[VersionedProtocol]) -> None:
        """
        Register a protocol implementation.

        Args:
            protocol_class: Protocol class to register
        """
        protocol_id = protocol_class.PROTOCOL_ID
        version = protocol_class.VERSION

        if version in self._registry[protocol_id]:
            raise ValueError(
                f"Protocol {protocol_id.name} version {version} already registered"
            )

        self._registry[protocol_id][version] = protocol_class

    def get_protocol(
        self, protocol_id: ProtocolID, version: int
    ) -> Type[VersionedProtocol]:
        """
        Get a protocol implementation by ID and version.

        Args:
            protocol_id: Protocol identifier
            version: Protocol version

        Returns:
            Protocol implementation class

        Raises:
            ValueError: If the protocol version is not registered
        """
        if version not in self._registry[protocol_id]:
            raise ValueError(
                f"Protocol {protocol_id.name} version {version} not registered"
            )

        return self._registry[protocol_id][version]

    def get_latest_version(self, protocol_id: ProtocolID) -> int:
        """
        Get the latest version number for a protocol.

        Args:
            protocol_id: Protocol identifier

        Returns:
            Latest version number

        Raises:
            ValueError: If no versions are registered for the protocol
        """
        if not self._registry[protocol_id]:
            raise ValueError(f"No versions registered for protocol {protocol_id.name}")

        return max(self._registry[protocol_id].keys())

    def get_compatible_version(
        self, protocol_id: ProtocolID, client_versions: List[int]
    ) -> Optional[int]:
        """
        Find the highest compatible version between server and client.

        Args:
            protocol_id: Protocol identifier
            client_versions: List of versions supported by client

        Returns:
            Highest compatible version, or None if none are compatible
        """
        available_versions = set(self._registry[protocol_id].keys())
        compatible_versions = available_versions.intersection(set(client_versions))

        if not compatible_versions:
            return None

        return max(compatible_versions)

    def list_protocols(self) -> Dict[str, List[int]]:
        """
        List all registered protocols and their versions.

        Returns:
            Dictionary of protocol names to lists of versions
        """
        return {
            protocol_id.name: sorted(versions.keys())
            for protocol_id, versions in self._registry.items()
            if versions  # Only include protocols with registered versions
        }


class ProtocolManager:
    """Manager for protocol encoding and decoding."""

    def __init__(self):
        self.registry = ProtocolRegistry()
        self._load_protocol_implementations()

    def _load_protocol_implementations(self) -> None:
        """Load all protocol implementations."""
        # This will be called by implementations to register themselves
        # with the registry. Here we just import the modules.

        # These will be implemented later
        # from feagi.api.protocols import fcp, fvp, fsmp

    def encode_message(self, data: Any, protocol_id: ProtocolID, version: int) -> bytes:
        """
        Encode a message with protocol headers.

        Args:
            data: Internal data to encode
            protocol_id: Protocol identifier
            version: Protocol version

        Returns:
            Complete binary message with headers
        """
        protocol_class = self.registry.get_protocol(protocol_id, version)
        payload = protocol_class.encode(data)

        # Add protocol ID and version header
        header = bytes([protocol_id.value, version])
        return header + payload

    def decode_message(self, data: bytes) -> Tuple[Any, ProtocolID, int]:
        """
        Decode a message with headers.

        Args:
            data: Binary message data with headers

        Returns:
            Tuple of (decoded data, protocol ID, version)

        Raises:
            ValueError: If the message format is invalid or unsupported
        """
        if len(data) < 2:
            raise ValueError("Message too short to contain header")

        try:
            protocol_id = ProtocolID(data[0])
        except ValueError:
            raise ValueError(f"Unknown protocol ID: {data[0]}")

        version = data[1]
        payload = data[2:]

        protocol_class = self.registry.get_protocol(protocol_id, version)
        decoded_data = protocol_class.decode(payload)

        return decoded_data, protocol_id, version
