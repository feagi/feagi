"""Protocol specifications and serialization for FEAGI."""

from feagi.protocols.json_protocol import JSONProtocol
from feagi.protocols.binary_protocol import BinaryProtocol
from feagi.protocols.protocol_factory import create_protocol

__all__ = [
    "JSONProtocol",
    "BinaryProtocol",
    "create_protocol"
] 