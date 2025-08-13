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

"""JSON protocol implementation for FEAGI."""

import json

from feagi.utils.logger import setup_logger

logger = setup_logger()
from typing import Any

from feagi.protocols.protocol_factory import Protocol, register_protocol


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
