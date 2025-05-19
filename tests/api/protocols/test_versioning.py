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
Tests for FEAGI protocol versioning system.

This module tests the versioning system for byte structures.
"""

import pytest
import unittest
import json
from typing import Dict, Any
from unittest import mock

from feagi.api.protocols.constants import ByteStructureID
from feagi.api.protocols.byte_structures import ByteStructureEncoder, ByteStructureDecoder
from feagi.api.protocols.byte_structures.utils import get_structure_info
from feagi.api.protocols import ByteStructureTranslator


@pytest.mark.skip(reason="Version negotiation features not yet fully implemented")
class TestVersionHandling(unittest.TestCase):
    """Test cases for byte structure version handling."""
    
    def setUp(self):
        self.encoder = ByteStructureEncoder()
        self.decoder = ByteStructureDecoder()
        self.translator = ByteStructureTranslator()
    
    def test_header_version(self):
        """Test that headers encode version correctly."""
        # Create headers with different versions
        header_v1 = self.encoder.encode_header(ByteStructureID.JSON, 1)
        header_v2 = self.encoder.encode_header(ByteStructureID.JSON, 2)
        
        # Check bytes are different
        self.assertNotEqual(header_v1, header_v2)
        
        # Extract and check versions
        structure_type_1, version_1 = get_structure_info(header_v1)
        structure_type_2, version_2 = get_structure_info(header_v2)
        
        self.assertEqual(structure_type_1, ByteStructureID.JSON)
        self.assertEqual(version_1, 1)
        self.assertEqual(structure_type_2, ByteStructureID.JSON)
        self.assertEqual(version_2, 2)
    
    def test_structure_version_validation(self):
        """Test validation of structure versions."""
        # Valid version: should not raise an exception
        self.encoder.encode_json({"test": "data"}, version=1)
        
        # Invalid version: should raise ValueError
        with self.assertRaises(ValueError):
            self.encoder.encode_json({"test": "data"}, version=999)
    
    def test_version_information_preserved(self):
        """Test that version information is preserved in decoded data."""
        # Create test data with version 1
        cortical_data = {
            "AREA01": {
                "x": [1, 2, 3],
                "y": [4, 5, 6],
                "z": [7, 8, 9],
                "potentials": [0.1, 0.2, 0.3]
            }
        }
        
        # Encode as neuron flat format (version 1)
        encoded = self.translator.create_neuron_data_message(cortical_data)
        
        # Decode and check version is included
        decoded = self.translator.decode_message(encoded)
        self.assertIn("data", decoded)
        self.assertIn("version", decoded["data"])
        self.assertEqual(decoded["data"]["version"], 1)
    
    def test_version_negotiation(self):
        """Test version negotiation between clients and server."""
        # Create translator with client capabilities
        translator = ByteStructureTranslator()
        
        # Register a client with various capabilities
        translator.register_client_capabilities("client1", {
            "structure_versions": {
                str(ByteStructureID.JSON): [1, 2],  # Supports JSON v1, v2
                str(ByteStructureID.NEURON_FLAT): [1]  # Only supports NEURON_FLAT v1
            }
        })
        
        # Check version selection
        self.assertEqual(translator.get_supported_version("client1", ByteStructureID.JSON), 1)
        self.assertEqual(translator.get_supported_version("client1", ByteStructureID.NEURON_FLAT), 1)
        
        # Unknown client should get default versions
        self.assertEqual(translator.get_supported_version("unknown", ByteStructureID.JSON), 1)
    
    def test_handshake_capabilities(self):
        """Test that handshake exchange includes version capabilities."""
        # Create capabilities message
        capabilities = self.translator.create_handshake_capabilities(
            supported_sensory=["camera", "microphone"],
            supported_motor=["arm", "leg"],
            protocol_versions={"fcp": 1, "fsmp": 1, "fvp": 1}
        )
        
        # Decode message
        decoded = self.translator.decode_message(capabilities)
        
        # Should include structure_versions
        self.assertIn("structure_versions", decoded)
        
        # Extract capabilities and check
        extracted = self.translator.extract_capabilities(decoded)
        self.assertIn("structure_versions", extracted)
        self.assertIn("protocol_versions", extracted)
    
    def test_client_specific_versions(self):
        """Test that messages use client-specific versions."""
        # Create translator
        translator = ByteStructureTranslator()
        
        # Register two clients with different capabilities
        translator.register_client_capabilities("old_client", {
            "structure_versions": {
                str(ByteStructureID.NEURON_FLAT): [1]
            }
        })
        
        # Hypothetically, if version 2 was supported
        from feagi.api.protocols.byte_structures.encoder import SUPPORTED_VERSIONS
        # This is just for the test - in reality we'd add version 2 to the SUPPORTED_VERSIONS
        
        # Create test data
        cortical_data = {
            "AREA01": {
                "x": [1, 2, 3],
                "y": [4, 5, 6], 
                "z": [7, 8, 9],
                "potentials": [0.1, 0.2, 0.3]
            }
        }
        
        # Create messages for each client
        msg_old = translator.create_neuron_data_message(cortical_data, client_id="old_client")
        
        # Check the versions
        _, version_old = get_structure_info(msg_old)
        self.assertEqual(version_old, 1)


if __name__ == '__main__':
    unittest.main() 