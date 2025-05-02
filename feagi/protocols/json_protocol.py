"""JSON protocol implementation for FEAGI."""

import json
import logging
from typing import Any, Dict, Optional

from feagi.protocols.protocol_factory import Protocol, register_protocol

logger = logging.getLogger(__name__)

@register_protocol
class JSONProtocol(Protocol):
    """
    JSON protocol implementation.
    
    This protocol serializes data as JSON and is used for REST and ZMQ interfaces
    where human-readable messages are desired.
    """
    
    name = "json"
    content_type = "application/json"
    
    @classmethod
    def serialize(cls, data: Any) -> bytes:
        """
        Serialize data to JSON bytes.
        
        Args:
            data: Data to serialize.
            
        Returns:
            JSON bytes.
        """
        try:
            return json.dumps(data).encode()
        except Exception as e:
            logger.exception(f"Error serializing data to JSON: {e}")
            return json.dumps({"error": str(e)}).encode()
            
    @classmethod
    def deserialize(cls, data: bytes) -> Any:
        """
        Deserialize JSON bytes to data.
        
        Args:
            data: JSON bytes to deserialize.
            
        Returns:
            Deserialized data.
        """
        try:
            return json.loads(data.decode())
        except Exception as e:
            logger.exception(f"Error deserializing JSON data: {e}")
            return {"error": str(e)}
            
    @classmethod
    def validate(cls, data: Any) -> bool:
        """
        Validate that data can be serialized as JSON.
        
        Args:
            data: Data to validate.
            
        Returns:
            True if data can be serialized as JSON, False otherwise.
        """
        try:
            json.dumps(data)
            return True
        except Exception:
            return False 