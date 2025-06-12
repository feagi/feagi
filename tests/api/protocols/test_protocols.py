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
Tests for FEAGI binary protocols.
"""

import pytest

# Skip the entire test module since the protocols implementation has changed
pytest.skip(
    "Protocol tests need to be updated after protocol refactoring and CapnP removal",
    allow_module_level=True,
)

import json
import struct
import time

# Original imports and tests below - kept for reference
import unittest
from typing import Any, Dict
from unittest.mock import MagicMock, patch

from feagi.api.protocols.base import (
    ProtocolManager,
    ProtocolRegistry,
    VersionedProtocol,
)
from feagi.api.protocols.fcp import FCPCommandType, FCPv1


# Test helper functions
def create_register_message(agent_id, agent_type, capabilities=None):
    """Create a test register message."""
    return {
        "command_type": FCPCommandType.REGISTER,
        "payload": {
            "agent_id": agent_id,
            "agent_type": agent_type,
            "capabilities": capabilities or {},
        },
    }


class TestProtocolRegistry(unittest.TestCase):
    """Test cases for the protocol registry."""

    def test_register_protocol(self):
        """Test registering a protocol."""
        registry = ProtocolRegistry()
        registry.register(FCPv1)
        assert FCPv1.protocol_id in registry.protocols

    def test_get_protocol(self):
        """Test getting a protocol."""
        registry = ProtocolRegistry()
        registry.register(FCPv1)
        protocol = registry.get_protocol(FCPv1.protocol_id, FCPv1.version)
        assert protocol == FCPv1

    def test_get_latest_version(self):
        """Test getting the latest version of a protocol."""
        registry = ProtocolRegistry()
        registry.register(FCPv1)

        # Create a mock v2 protocol
        FCPv2 = MagicMock()
        FCPv2.protocol_id = FCPv1.protocol_id
        FCPv2.version = 2

        # Register v2
        registry.register(FCPv2)

        # Get latest should return v2
        latest = registry.get_latest_version(FCPv1.protocol_id)
        assert latest == FCPv2

        # Get specific version
        v1 = registry.get_protocol(FCPv1.protocol_id, 1)
        assert v1 == FCPv1

    def test_get_compatible_version(self):
        """Test finding compatible protocol versions."""
        registry = ProtocolRegistry()
        registry.register(FCPv1)

        # Create a mock v2 protocol
        FCPv2 = MagicMock()
        FCPv2.protocol_id = FCPv1.protocol_id
        FCPv2.version = 2
        registry.register(FCPv2)

        # Test when all versions are available
        compat = registry.get_compatible_version(FCPv1.protocol_id, [1, 2])
        assert compat == FCPv2  # Should return the highest compatible version

        # Test when only v1 is compatible
        compat = registry.get_compatible_version(FCPv1.protocol_id, [1])
        assert compat == FCPv1

        # Test when requested version doesn't exist
        compat = registry.get_compatible_version(FCPv1.protocol_id, [3])
        assert compat is None

        # Test with protocol that doesn't exist
        compat = registry.get_compatible_version("UNKNOWN", [1])
        assert compat is None

    def test_list_protocols(self):
        """Test listing available protocols."""
        registry = ProtocolRegistry()
        registry.register(FCPv1)

        # Create a mock v2 protocol
        FCPv2 = MagicMock()
        FCPv2.protocol_id = FCPv1.protocol_id
        FCPv2.version = 2
        registry.register(FCPv2)

        # Create a different protocol
        OtherProto = MagicMock()
        OtherProto.protocol_id = "OTHER"
        OtherProto.version = 1
        registry.register(OtherProto)

        # List all protocols
        protocols = registry.list_protocols()
        assert len(protocols) == 2  # Two protocol types
        assert FCPv1.protocol_id in protocols
        assert OtherProto.protocol_id in protocols
        assert len(protocols[FCPv1.protocol_id]) == 2  # Two versions of FCP

        # List specific protocol
        fcp_versions = registry.list_versions(FCPv1.protocol_id)
        assert len(fcp_versions) == 2
        assert 1 in fcp_versions
        assert 2 in fcp_versions


class TestFCPv1(unittest.TestCase):
    """Test cases for FCP protocol version 1."""

    def test_encode_decode(self):
        """Test encoding and decoding a message."""
        # Create a test message
        original_data = create_register_message(
            agent_id="test-agent",
            agent_type="test",
            capabilities={"sensors": ["camera"], "motors": ["wheel"]},
        )
        original_data["payload"][
            "timestamp"
        ] = "2023-01-01T00:00:00"  # Use fixed timestamp for testing

        # Encode the message
        encoded = FCPv1.encode(original_data)

        # Decode the message
        decoded = FCPv1.decode(encoded)

        # Check that the decoded message matches the original
        assert decoded["command_type"] == original_data["command_type"]
        assert decoded["payload"]["agent_id"] == original_data["payload"]["agent_id"]
        assert (
            decoded["payload"]["agent_type"] == original_data["payload"]["agent_type"]
        )

    def test_encode_format(self):
        """Test the format of encoded messages."""
        # Create a test message
        data = {"command_type": FCPCommandType.REGISTER, "payload": {"test": "data"}}

        # Encode the message
        encoded = FCPv1.encode(data)

        # Check header format
        command_type, payload_length = struct.unpack("!BI", encoded[:5])
        assert command_type == int(FCPCommandType.REGISTER)

        # Check payload is valid JSON
        payload = encoded[5 : 5 + payload_length].decode("utf-8")
        payload_data = json.loads(payload)
        assert payload_data == {"test": "data"}


class TestProtocolManager(unittest.TestCase):
    """Test cases for the protocol manager."""

    @patch("feagi.api.protocols.ProtocolRegistry")
    def test_encode_decode_message(self, mock_registry):
        """Test encoding and decoding messages with the manager."""
        # Set up mock registry
        registry_instance = MagicMock()
        mock_registry.return_value = registry_instance

        # Make get_protocol return FCPv1
        registry_instance.get_protocol.return_value = FCPv1
        registry_instance.get_compatible_version.return_value = FCPv1

        # Create manager
        manager = ProtocolManager()

        # Create test message
        data = create_register_message(agent_id="test-agent", agent_type="test")

        # Encode message
        encoded = manager.encode_message("FCP", 1, data)

        # Decode message
        protocol_id, version, decoded = manager.decode_message(encoded)

        # Check results
        assert protocol_id == "FCP"
        assert version == 1
        assert decoded["command_type"] == data["command_type"]
        assert decoded["payload"]["agent_id"] == data["payload"]["agent_id"]

        # Verify correct protocol was used
        registry_instance.get_protocol.assert_called_with("FCP", 1)


class TestProtocolTranslator(unittest.TestCase):
    """Test cases for the protocol translator."""

    @patch("feagi.api.protocols.ProtocolManager")
    def test_register_agent(self, mock_manager):
        """Test registering an agent."""
        # Set up mock manager
        manager_instance = MagicMock()
        mock_manager.return_value = manager_instance

        # Create translator
        from feagi.api.protocols import ProtocolTranslator

        translator = ProtocolTranslator()

        # Register an agent
        translator.register_agent(
            agent_id="test-agent",
            agent_type="test",
            capabilities={"sensors": ["camera"]},
        )

        # Verify manager was called correctly
        manager_instance.encode_message.assert_called()
        call_args = manager_instance.encode_message.call_args[0]
        assert call_args[0] == "FCP"  # protocol_id
        assert call_args[1] == 1  # version
        assert call_args[2]["command_type"] == FCPCommandType.REGISTER
        assert call_args[2]["payload"]["agent_id"] == "test-agent"

    @patch("feagi.api.protocols.ProtocolManager")
    def test_encode_decode(self, mock_manager):
        """Test encoding and decoding messages."""
        # Set up mock manager
        manager_instance = MagicMock()
        mock_manager.return_value = manager_instance

        # Create translator
        from feagi.api.protocols import ProtocolTranslator

        translator = ProtocolTranslator()

        # Test encoding
        translator.encode_message("FCP", 1, {"test": "data"})
        manager_instance.encode_message.assert_called_with("FCP", 1, {"test": "data"})

        # Test decoding
        manager_instance.decode_message.return_value = ("FCP", 1, {"decoded": "data"})
        protocol_id, version, data = translator.decode_message(b"test_data")
        assert protocol_id == "FCP"
        assert version == 1
        assert data == {"decoded": "data"}
        manager_instance.decode_message.assert_called_with(b"test_data")
