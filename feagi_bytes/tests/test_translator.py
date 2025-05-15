"""
Tests for the FEAGI Bytes translator module.
"""

import unittest
import json

from feagi_bytes import ByteStructureID, ByteStructureTranslator


class TestByteStructureTranslator(unittest.TestCase):
    """Test cases for byte structure translator."""
    
    def setUp(self):
        self.translator = ByteStructureTranslator()
    
    def test_message_roundtrip(self):
        """Test message encoding and decoding."""
        # Create test message
        test_message = {
            "message_type": "command",
            "command": "start",
            "parameters": {
                "speed": 10,
                "direction": "forward"
            }
        }
        
        # Encode
        encoded = self.translator.create_message(test_message)
        
        # Verify it's a valid byte structure
        self.assertEqual(encoded[0], ByteStructureID.JSON)
        
        # Decode
        decoded = self.translator.decode_message(encoded)
        
        # Verify it matches original
        self.assertEqual(decoded, test_message)
    
    def test_version_negotiation(self):
        """Test version negotiation."""
        # Register client capabilities
        client_id = "test_client"
        capabilities = {
            "structure_versions": {
                str(ByteStructureID.JSON): [1],
                str(ByteStructureID.RAW_IMAGE): [1]
            }
        }
        
        self.translator.register_client_capabilities(client_id, capabilities)
        
        # Get negotiated version
        version = self.translator.get_supported_version(client_id, ByteStructureID.JSON)
        self.assertEqual(version, 1)


if __name__ == "__main__":
    unittest.main() 