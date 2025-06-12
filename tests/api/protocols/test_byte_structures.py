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
Tests for FEAGI byte structures implementation.

This module contains unit tests for the byte structure encoder and decoder.
"""

import json
import unittest

import numpy as np
import pytest
from feagi_bytes import ByteStructureDecoder, ByteStructureEncoder
from feagi_bytes.utils import is_compressed, validate_cortical_id

from feagi.api.protocols import ByteStructureTranslator
from feagi.api.protocols.constants import ByteStructureID


class TestByteStructureEncoder(unittest.TestCase):
    """Test cases for ByteStructureEncoder."""

    def setUp(self):
        self.encoder = ByteStructureEncoder()
        self.decoder = ByteStructureDecoder()

    def test_encode_header(self):
        """Test encoding of byte structure header."""
        header = self.encoder.encode_header(ByteStructureID.JSON, 1)
        self.assertEqual(len(header), 2)
        self.assertEqual(header[0], ByteStructureID.JSON)
        self.assertEqual(header[1], 1)

    def test_encode_decode_json(self):
        """Test encoding and decoding of JSON structure."""
        test_data = {"name": "test", "value": 42}
        encoded = self.encoder.encode_json(test_data)

        # Check header
        self.assertEqual(encoded[0], ByteStructureID.JSON)

        # Decode and verify
        decoded = self.decoder.decode_json(encoded)
        self.assertEqual(decoded, test_data)

    def test_encode_decode_raw_image(self):
        """Test encoding and decoding of raw image structure."""
        # Create a small test image (2x2 BGR)
        test_image = np.zeros((2, 2, 3), dtype=np.uint8)
        test_image[0, 0] = [255, 0, 0]  # Blue in BGR
        test_image[0, 1] = [0, 255, 0]  # Green in BGR
        test_image[1, 0] = [0, 0, 255]  # Red in BGR
        test_image[1, 1] = [255, 255, 255]  # White in BGR

        encoded = self.encoder.encode_raw_image(test_image)

        # Check header
        self.assertEqual(encoded[0], ByteStructureID.RAW_IMAGE)

        # Decode and verify
        decoded = self.decoder.decode_raw_image(encoded)
        self.assertEqual(decoded.shape, test_image.shape)
        np.testing.assert_array_equal(decoded, test_image)

    def test_encode_decode_multi_holder(self):
        """Test encoding and decoding of multi-holder structure."""
        # Create two test structures
        json_data = self.encoder.encode_json({"test": "data"})

        test_image = np.zeros((2, 2, 3), dtype=np.uint8)
        test_image[0, 0] = [255, 0, 0]  # Blue in BGR
        image_data = self.encoder.encode_raw_image(test_image)

        # Combine into multi-holder
        encoded = self.encoder.encode_multi_holder([json_data, image_data])

        # Check header
        self.assertEqual(encoded[0], ByteStructureID.MULTI_HOLDER)

        # Decode and verify
        decoded = self.decoder.decode_multi_holder(encoded)
        self.assertEqual(len(decoded), 2)

        # Check individual structures
        self.assertEqual(decoded[0][0], ByteStructureID.JSON)
        self.assertEqual(decoded[1][0], ByteStructureID.RAW_IMAGE)

        # Verify contents by decoding each structure
        json_result = self.decoder.decode_json(decoded[0])
        self.assertEqual(json_result, {"test": "data"})

        image_result = self.decoder.decode_raw_image(decoded[1])
        np.testing.assert_array_equal(image_result[0, 0], [255, 0, 0])

    def test_encode_decode_neuron_flat(self):
        """Test encoding and decoding of neuron flat structure."""
        # Test data
        cortical_ids = ["AREA01", "AREA01", "AREA01"]
        x_coords = [1, 2, 3]
        y_coords = [4, 5, 6]
        z_coords = [7, 8, 9]
        potentials = [0.1, 0.5, 0.9]

        # Encode
        encoded = self.encoder.encode_neuron_flat(
            cortical_ids=cortical_ids,
            x_coords=x_coords,
            y_coords=y_coords,
            z_coords=z_coords,
            potentials=potentials,
        )

        # Check header
        self.assertEqual(encoded[0], ByteStructureID.NEURON_FLAT)

        # Decode and verify
        decoded = self.decoder.decode_neuron_flat(encoded)
        self.assertEqual(len(decoded["cortical_ids"]), 3)
        self.assertEqual(decoded["cortical_ids"][0], "AREA01")
        self.assertEqual(decoded["x"], x_coords)
        self.assertEqual(decoded["y"], y_coords)
        self.assertEqual(decoded["z"], z_coords)
        self.assertEqual(decoded["potentials"], potentials)

    def test_encode_decode_neuron_categories(self):
        """Test encoding and decoding of neuron categories structure."""
        # Test data
        cortical_data = {
            "AREA01": {"x": [1, 2], "y": [3, 4], "z": [5, 6], "potentials": [0.1, 0.2]},
            "AREA02": {
                "x": [7, 8, 9],
                "y": [10, 11, 12],
                "z": [13, 14, 15],
                "potentials": [0.3, 0.4, 0.5],
            },
        }

        # Encode
        encoded = self.encoder.encode_neuron_categories(cortical_data)

        # Check header
        self.assertEqual(encoded[0], ByteStructureID.NEURON_CATEGORIES)

        # Decode and verify
        decoded = self.decoder.decode_neuron_categories(encoded)
        self.assertEqual(len(decoded), 2)
        self.assertIn("AREA01", decoded)
        self.assertIn("AREA02", decoded)

        # Check AREA01 data
        area1 = decoded["AREA01"]
        self.assertEqual(len(area1["x"]), 2)
        self.assertEqual(area1["x"][0], 1)
        self.assertEqual(area1["potentials"][1], 0.2)

        # Check AREA02 data
        area2 = decoded["AREA02"]
        self.assertEqual(len(area2["x"]), 3)
        self.assertEqual(area2["z"][2], 15)
        self.assertEqual(area2["potentials"][0], 0.3)

    def test_compression(self):
        """Test compression and decompression."""
        # Create sample data
        test_data = b"A" * 1000

        # Compress
        compressed = self.encoder.compress(test_data)

        # Verify compression happened
        self.assertLess(len(compressed), len(test_data))

        # Decompress and verify
        decompressed = self.encoder.decompress(compressed)
        self.assertEqual(decompressed, test_data)


class TestByteStructureTranslator(unittest.TestCase):
    """Test cases for ByteStructureTranslator."""

    def setUp(self):
        self.translator = ByteStructureTranslator()

    def test_create_handshake_hello(self):
        """Test creating handshake hello message."""
        message = self.translator.create_handshake_hello("test_agent", "test_type")

        # Check it's a valid byte structure
        self.assertEqual(message[0], ByteStructureID.JSON)

        # Decode and verify contents
        decoded = self.translator.decode_message(message)
        self.assertEqual(decoded["agent_id"], "test_agent")
        self.assertEqual(decoded["agent_type"], "test_type")
        self.assertEqual(decoded["message_type"], "hello")

    def test_neuron_data_message_flat(self):
        """Test creating neuron data message in flat format."""
        cortical_data = {
            "AREA01": {
                "x": [1, 2, 3],
                "y": [4, 5, 6],
                "z": [7, 8, 9],
                "potentials": [0.1, 0.2, 0.3],
            }
        }

        message = self.translator.create_neuron_data_message(cortical_data)

        # Should use flat format for single area
        self.assertEqual(message[0], ByteStructureID.NEURON_FLAT)

        # Decode and verify
        decoded = self.translator.decode_message(message)
        self.assertEqual(decoded["message_type"], "neuron_data")
        self.assertEqual(len(decoded["data"]["cortical_ids"]), 3)
        self.assertEqual(decoded["data"]["x"][1], 2)

    def test_neuron_data_message_categories(self):
        """Test creating neuron data message in categories format."""
        cortical_data = {
            "AREA01": {"x": [1, 2], "y": [3, 4], "z": [5, 6], "potentials": [0.1, 0.2]},
            "AREA02": {
                "x": [7, 8],
                "y": [9, 10],
                "z": [11, 12],
                "potentials": [0.3, 0.4],
            },
        }

        message = self.translator.create_neuron_data_message(cortical_data)

        # Should use categories format for multiple areas
        self.assertEqual(message[0], ByteStructureID.NEURON_CATEGORIES)

        # Decode and verify
        decoded = self.translator.decode_message(message)
        self.assertEqual(decoded["message_type"], "neuron_data")
        self.assertEqual(len(decoded["data"]), 2)
        self.assertEqual(decoded["data"]["AREA02"]["potentials"][1], 0.4)


class TestUtilityFunctions(unittest.TestCase):
    """Test cases for utility functions."""

    def test_validate_cortical_id(self):
        """Test cortical ID validation."""
        # Normal case
        self.assertEqual(validate_cortical_id("AREA01"), "AREA01")

        # Too short
        self.assertEqual(validate_cortical_id("ABC"), "ABC   ")

        # Too long
        self.assertEqual(validate_cortical_id("TOOLONGID"), "TOOLON")

        # Empty ID should raise error
        with self.assertRaises(ValueError):
            validate_cortical_id("")

    def test_is_compressed(self):
        """Test compression detection."""
        # Create sample data
        test_data = b"A" * 1000

        # Compress with zlib
        compressed = ByteStructureEncoder.compress(test_data)

        # Check detection
        self.assertTrue(is_compressed(compressed))
        self.assertFalse(is_compressed(test_data))


if __name__ == "__main__":
    unittest.main()
