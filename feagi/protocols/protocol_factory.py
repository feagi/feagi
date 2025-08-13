"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""Protocol factory for FEAGI."""

from feagi.utils.logger import setup_logger

logger = setup_logger()
from typing import Dict, Optional, Type


# Protocol base class definition
class Protocol:
    """Base protocol interface."""

    name: str
    content_type: str

    @classmethod
    def serialize(cls, data):
        """Serialize data to bytes."""
        raise NotImplementedError

    @classmethod
    def deserialize(cls, data):
        """Deserialize bytes to data."""
        raise NotImplementedError


# Protocol registry
_protocols: Dict[str, Type[Protocol]] = {}


def register_protocol(protocol_class: Type[Protocol]):
    """
    Register a protocol implementation.

    Args:
        protocol_class: Protocol class to register.
    """
    if not hasattr(protocol_class, "name") or not protocol_class.name:
        raise ValueError("Protocol class must define a name attribute")

    _protocols[protocol_class.name] = protocol_class
    logger.debug(f"Registered protocol: {protocol_class.name}")

    return protocol_class


def create_protocol(protocol_name: str) -> Optional[Type[Protocol]]:
    """
    Create a protocol by name.

    Args:
        protocol_name: Name of the protocol to create.

    Returns:
        Protocol class or None if not found.
    """
    if protocol_name not in _protocols:
        logger.warning(f"Protocol not found: {protocol_name}")
        return None

    return _protocols[protocol_name]


def get_protocol_by_content_type(
    content_type: str,
) -> Optional[Type[Protocol]]:
    """
    Get a protocol by content type.

    Args:
        content_type: Content type to match.

    Returns:
        Protocol class or None if not found.
    """
    for protocol_class in _protocols.values():
        if (
            hasattr(protocol_class, "content_type")
            and protocol_class.content_type == content_type
        ):
            return protocol_class

    logger.warning(f"No protocol found for content type: {content_type}")
    return None


def get_available_protocols() -> Dict[str, str]:
    """
    Get a dictionary of available protocols.

    Returns:
        Dictionary mapping protocol names to content types.
    """
    return {
        protocol.name: protocol.content_type
        for protocol in _protocols.values()
        if hasattr(protocol, "content_type")
    }
