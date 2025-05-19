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
from unittest.mock import patch
import json
import struct

from feagi.api.protocols.base import ProtocolID, VersionedProtocol, ProtocolRegistry, ProtocolManager
from feagi.api.protocols.fcp import FCPv1, FCPCommandType, create_register_message
from feagi.api.protocols.translator import ProtocolTranslator


class TestProtocolRegistry:
    """Tests for the ProtocolRegistry class."""

    def test_register_protocol(self):
        """Test registering a protocol."""
        registry = ProtocolRegistry()
        registry.register(FCPv1)
        
        # Check that the protocol was registered
        assert registry._registry[ProtocolID.FCP][1] == FCPv1
    
    def test_get_protocol(self):
        """Test getting a protocol."""
        registry = ProtocolRegistry()
        registry.register(FCPv1)
        
        # Get the protocol
        protocol = registry.get_protocol(ProtocolID.FCP, 1)
        assert protocol == FCPv1
    
    def test_get_latest_version(self):
        """Test getting the latest version of a protocol."""
        registry = ProtocolRegistry()
        registry.register(FCPv1)
        
        # Define a mock version 2 protocol
        class FCPv2(VersionedProtocol):
            PROTOCOL_ID = ProtocolID.FCP
            VERSION = 2
            
            @classmethod
            def encode(cls, data):
                return b""
                
            @classmethod
            def decode(cls, data):
                return {}
        
        registry.register(FCPv2)
        
        # Get the latest version
        latest = registry.get_latest_version(ProtocolID.FCP)
        assert latest == 2
    
    def test_get_compatible_version(self):
        """Test finding compatible protocol versions."""
        registry = ProtocolRegistry()
        registry.register(FCPv1)
        
        # Define a mock version 2 protocol
        class FCPv2(VersionedProtocol):
            PROTOCOL_ID = ProtocolID.FCP
            VERSION = 2
            
            @classmethod
            def encode(cls, data):
                return b""
                
            @classmethod
            def decode(cls, data):
                return {}
        
        registry.register(FCPv2)
        
        # Client supports both versions
        compatible = registry.get_compatible_version(ProtocolID.FCP, [1, 2])
        assert compatible == 2
        
        # Client supports only version 1
        compatible = registry.get_compatible_version(ProtocolID.FCP, [1])
        assert compatible == 1
        
        # Client supports only version 3 (not available)
        compatible = registry.get_compatible_version(ProtocolID.FCP, [3])
        assert compatible is None
    
    def test_list_protocols(self):
        """Test listing available protocols."""
        registry = ProtocolRegistry()
        registry.register(FCPv1)
        
        # Define a mock FVP protocol
        class FVPv1(VersionedProtocol):
            PROTOCOL_ID = ProtocolID.FVP
            VERSION = 1
            
            @classmethod
            def encode(cls, data):
                return b""
                
            @classmethod
            def decode(cls, data):
                return {}
        
        registry.register(FVPv1)
        
        # List protocols
        protocols = registry.list_protocols()
        assert "FCP" in protocols
        assert "FVP" in protocols
        assert protocols["FCP"] == [1]
        assert protocols["FVP"] == [1]


class TestFCPv1:
    """Tests for the FCPv1 protocol."""
    
    def test_encode_decode(self):
        """Test encoding and decoding a message."""
        # Create a test message
        original_data = create_register_message(
            agent_id="test-agent",
            agent_type="test",
            capabilities={"sensors": ["camera"], "motors": ["wheel"]}
        )
        original_data["payload"]["timestamp"] = "2023-01-01T00:00:00"  # Use fixed timestamp for testing
        
        # Encode the message
        encoded = FCPv1.encode(original_data)
        
        # Decode the message
        decoded = FCPv1.decode(encoded)
        
        # Check that the decoded message matches the original
        assert decoded["command_type"] == original_data["command_type"]
        assert decoded["payload"]["agent_id"] == original_data["payload"]["agent_id"]
        assert decoded["payload"]["agent_type"] == original_data["payload"]["agent_type"]
        assert decoded["payload"]["capabilities"] == original_data["payload"]["capabilities"]
    
    def test_encode_format(self):
        """Test the format of encoded messages."""
        # Create a test message
        data = {
            "command_type": FCPCommandType.REGISTER,
            "payload": {"test": "data"}
        }
        
        # Encode the message
        encoded = FCPv1.encode(data)
        
        # Check header format
        command_type, payload_length = struct.unpack("!BI", encoded[:5])
        assert command_type == FCPCommandType.REGISTER
        
        # Check payload content
        payload = json.loads(encoded[5:].decode("utf-8"))
        assert payload["test"] == "data"


class TestProtocolManager:
    """Tests for the ProtocolManager class."""

    @patch('feagi.api.protocols.fcp.register_protocols')
    def test_encode_decode_message(self, mock_register):
        """Test encoding and decoding a message with protocol ID and version."""
        manager = ProtocolManager()
        
        # Manually register protocol for testing
        registry = ProtocolRegistry()
        registry.register(FCPv1)
        manager.registry = registry
        
        # Create test data
        original_data = create_register_message(
            agent_id="test-agent",
            agent_type="test",
            capabilities={}
        )
        
        # Encode with protocol header
        encoded = manager.encode_message(original_data, ProtocolID.FCP, 1)
        
        # Check protocol header
        assert encoded[0] == ProtocolID.FCP.value
        assert encoded[1] == 1
        
        # Decode full message
        decoded, protocol_id, version = manager.decode_message(encoded)
        
        # Check decoding results
        assert protocol_id == ProtocolID.FCP
        assert version == 1
        assert decoded["command_type"] == original_data["command_type"]
        assert decoded["payload"]["agent_id"] == original_data["payload"]["agent_id"]


class TestProtocolTranslator:
    """Tests for the ProtocolTranslator class."""
    
    @patch('feagi.api.protocols.base.ProtocolManager._load_protocol_implementations')
    def test_register_agent(self, mock_load):
        """Test registering an agent with protocol version negotiation."""
        translator = ProtocolTranslator()
        
        # Manually register protocol for testing
        registry = ProtocolRegistry()
        registry.register(FCPv1)
        
        # Define a mock FCP v2 protocol
        class FCPv2(VersionedProtocol):
            PROTOCOL_ID = ProtocolID.FCP
            VERSION = 2
            
            @classmethod
            def encode(cls, data):
                return b""
                
            @classmethod
            def decode(cls, data):
                return {}
        
        registry.register(FCPv2)
        translator.protocol_manager.registry = registry
        
        # Agent supports both FCP v1 and v2
        agent_versions = {"FCP": [1, 2]}
        compatible = translator.register_agent("agent1", agent_versions)
        
        # Should select highest compatible version
        assert compatible["FCP"] == 2
        
        # Agent supports only FCP v1
        agent_versions = {"FCP": [1]}
        compatible = translator.register_agent("agent2", agent_versions)
        assert compatible["FCP"] == 1
        
        # Agent sends single version (not a list)
        agent_versions = {"FCP": 1}
        compatible = translator.register_agent("agent3", agent_versions)
        assert compatible["FCP"] == 1
    
    @patch('feagi.api.protocols.base.ProtocolManager._load_protocol_implementations')
    def test_encode_decode(self, mock_load):
        """Test encoding and decoding messages for a specific agent."""
        translator = ProtocolTranslator()
        
        # Manually register protocol for testing
        registry = ProtocolRegistry()
        registry.register(FCPv1)
        translator.protocol_manager.registry = registry
        
        # Register an agent
        agent_versions = {"FCP": 1}
        translator.register_agent("agent1", agent_versions)
        
        # Create test data
        original_data = create_register_message(
            agent_id="agent1",
            agent_type="test",
            capabilities={}
        )
        
        # Encode for the agent
        encoded = translator.encode("agent1", original_data, "FCP")
        
        # Decode (should work regardless of agent)
        decoded, protocol_name, version = translator.decode(encoded)
        
        # Check decoding results
        assert protocol_name == "FCP"
        assert version == 1
        assert decoded["command_type"] == original_data["command_type"]
        assert decoded["payload"]["agent_id"] == original_data["payload"]["agent_id"]
        assert decoded["payload"]["agent_type"] == original_data["payload"]["agent_type"] 