"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Serialization utilities for FEAGI ZeroMQ interfaces.

This module provides serialization and deserialization functions for
different content types used in ZeroMQ communication.
"""

import json

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)
from typing import Any


def serialize_message(
    data: Any, content_type: str = "application/json"
) -> bytes:
    """Serialize a message according to the specified content type.

    Args:
        data: Data to serialize
        content_type: Content type for serialization

    Returns:
        Serialized data as bytes

    Raises:
        ValueError: If content_type is not supported
    """
    if content_type == "application/json":
        return json.dumps(data).encode()
    elif content_type == "application/octet-stream":
        # Binary data should already be bytes-like
        if isinstance(data, bytes):
            return data
        elif isinstance(data, (dict, list)):
            # Fallback to JSON for complex structures
            return json.dumps(data).encode()
        else:
            # Convert to string and encode
            return str(data).encode()
    elif content_type == "text/plain":
        return str(data).encode()
    else:
        raise ValueError(f"Unsupported content type: {content_type}")


def deserialize_message(
    data: bytes, content_type: str = "application/json"
) -> Any:
    """Deserialize a message according to the specified content type.

    Args:
        data: Serialized data as bytes
        content_type: Content type for deserialization

    Returns:
        Deserialized data

    Raises:
        ValueError: If content_type is not supported
    """
    if content_type == "application/json":
        return json.loads(data.decode())
    elif content_type == "application/octet-stream":
        # Try to interpret as JSON first
        try:
            return json.loads(data.decode())
        except json.JSONDecodeError:
            # Otherwise return as raw bytes
            return data
    elif content_type == "text/plain":
        return data.decode()
    else:
        raise ValueError(f"Unsupported content type: {content_type}")
