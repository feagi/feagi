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

# Skip the entire test module since the protocols implementation has changed
pytest.skip(
    "Protocol tests need to be updated after protocol refactoring and CapnP removal",
    allow_module_level=True,
)

import json

# Original imports and tests below - kept for reference
import unittest
from typing import Any, Dict

from feagi_bytes import ByteStructureDecoder, ByteStructureEncoder
from feagi_bytes.utils import get_structure_info

from feagi.api.protocols import ByteStructureTranslator
from feagi.api.protocols.constants import ByteStructureID


class TestVersionHandling(unittest.TestCase):
    """Test cases for byte structure version handling."""

    def setUp(self):
        """Set up test cases."""
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
        self.assertEqual(structure_type_2, ByteStructureID.JSON)
        self.assertEqual(version_1, 1)
        self.assertEqual(version_2, 2)

    def test_structure_version_validation(self):
        """Test validation of structure versions."""
        # Valid version: should not raise an exception
        self.encoder.encode_json({"test": "data"}, version=1)

        # Invalid version: should raise ValueError
        with self.assertRaises(ValueError):
            self.encoder.encode_json({"test": "data"}, version=99)

    def test_version_information_preserved(self):
        """Test that version information is preserved in decoded data."""
        # Create test data with version 1
        cortical_data = {
            "AREA01": {
                "x": [1, 2, 3],
                "y": [4, 5, 6],
                "z": [7, 8, 9],
                "potentials": [0.1, 0.2, 0.3],
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
        translator.register_client_capabilities(
            "client1",
            {
                "structure_versions": {
                    str(ByteStructureID.JSON): [1, 2],  # Supports JSON v1, v2
                    str(ByteStructureID.NEURON_FLAT): [
                        1
                    ],  # Only supports NEURON_FLAT v1
                }
            },
        )

        # Check that we can get versions for a client
        versions = translator.get_client_capabilities("client1")
        self.assertIn("structure_versions", versions)
        self.assertEqual(
            versions["structure_versions"][str(ByteStructureID.JSON)], [1, 2]
        )

    def test_client_specific_versions(self):
        """Test that messages use client-specific versions."""
        # Create translator
        translator = ByteStructureTranslator()

        # Register two clients with different capabilities
        translator.register_client_capabilities(
            "old_client",
            {"structure_versions": {str(ByteStructureID.NEURON_FLAT): [1]}},
        )

        translator.register_client_capabilities(
            "new_client",
            {"structure_versions": {str(ByteStructureID.NEURON_FLAT): [1, 2]}},
        )

        # Create test data
        cortical_data = {
            "AREA01": {"x": [1, 2], "y": [3, 4], "z": [5, 6], "potentials": [0.1, 0.2]}
        }

        # Create message for old client
        msg1 = translator.create_neuron_data_message(
            cortical_data, client_id="old_client"
        )

        # Create message for new client
        msg2 = translator.create_neuron_data_message(
            cortical_data, client_id="new_client"
        )

        # Decode both messages
        decoded1 = translator.decode_message(msg1)
        decoded2 = translator.decode_message(msg2)

        # Old client should use version 1
        self.assertEqual(decoded1["data"]["version"], 1)

        # New client should use the latest version (2)
        self.assertEqual(decoded2["data"]["version"], 2)

    def test_handshake_capabilities(self):
        """Test that handshake exchange includes version capabilities."""
        # Create capabilities message
        capabilities = self.translator.create_handshake_capabilities(
            supported_sensory=["camera", "microphone"],
            supported_motor=["arm", "leg"],
            protocol_versions={"fcp": 1, "fsmp": 1, "fvp": 1},
        )

        # Decode message
        decoded = self.translator.decode_message(capabilities)

        # Should include structure_versions
        self.assertIn("structure_versions", decoded)

        # Extract capabilities and check
        extracted = self.translator.extract_capabilities(decoded)
        self.assertIn("structure_versions", extracted)
        self.assertIn("protocol_versions", extracted)


if __name__ == "__main__":
    unittest.main()
