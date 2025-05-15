"""
Tests for the FEAGI Bytes serialization module.
"""

import unittest
import json
import numpy as np

from feagi_bytes import ByteStructureID, ByteStructureEncoder, ByteStructureDecoder


class TestByteStructureSerialization(unittest.TestCase):
    """Test cases for byte structure serialization."""
    
    def setUp(self):
        self.encoder = ByteStructureEncoder()
        self.decoder = ByteStructureDecoder()
    
    def test_json_roundtrip(self):
        """Test JSON encoding and decoding."""
        # Create test data
        test_data = {"name": "test", "value": 42, "nested": {"key": "value"}}
        
        # Encode
        encoded = self.encoder.encode_json(test_data)
        
        # Verify header
        self.assertEqual(encoded[0], ByteStructureID.JSON)
        self.assertEqual(encoded[1], 1)  # Version 1
        
        # Decode
        decoded = self.decoder.decode_json(encoded)
        
        # Verify data matches original
        self.assertEqual(decoded, test_data)
    
    def test_raw_image_roundtrip(self):
        """Test raw image encoding and decoding."""
        # Create test image
        test_image = np.zeros((10, 20, 3), dtype=np.uint8)
        test_image[0, 0] = [255, 0, 0]  # BGR format
        test_image[5, 10] = [0, 255, 0]
        
        # Encode
        encoded = self.encoder.encode_raw_image(test_image)
        
        # Verify header
        self.assertEqual(encoded[0], ByteStructureID.RAW_IMAGE)
        
        # Decode
        decoded = self.decoder.decode_raw_image(encoded)
        
        # Verify dimensions and content
        self.assertEqual(decoded.shape, test_image.shape)
        np.testing.assert_array_equal(decoded, test_image)
    
    def test_compression(self):
        """Test compression and decompression."""
        # Create test data
        test_data = b"A" * 1000
        
        # Compress
        compressed = self.encoder.compress(test_data)
        
        # Verify compression flag
        self.assertEqual(compressed[0], 1)
        
        # Verify it's actually compressed (smaller)
        self.assertLess(len(compressed), len(test_data))
        
        # Decompress
        decompressed = self.decoder.decompress(compressed)
        
        # Verify it matches original
        self.assertEqual(decompressed, test_data)


if __name__ == "__main__":
    unittest.main() 