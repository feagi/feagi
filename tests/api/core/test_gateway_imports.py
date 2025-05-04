#!/usr/bin/env python
"""
Test script for verifying API Gateway imports from different locations.
"""

import os
import sys
import unittest


class TestGatewayImports(unittest.TestCase):
    """Test class for verifying API Gateway imports."""
    
    def test_direct_imports(self):
        """Test importing the API Gateway directly from different locations."""
        # Test 1: Direct import from gateway package
        try:
            from feagi.api.gateway import APIGateway, get_api_gateway
            self.assertTrue(True, "Successfully imported from feagi.api.gateway")
        except ImportError as e:
            self.fail(f"Failed to import from feagi.api.gateway: {e}")
        
        # Test 2: Import from api package
        try:
            from feagi.api import APIGateway, get_api_gateway
            self.assertTrue(True, "Successfully imported from feagi.api")
        except ImportError as e:
            self.fail(f"Failed to import from feagi.api: {e}")
        
        # Test 3: Test implementation in gateway/api_gateway.py
        try:
            from feagi.api.gateway.api_gateway import APIGateway, get_api_gateway
            self.assertTrue(True, "Successfully imported from feagi.api.gateway.api_gateway")
        except ImportError as e:
            self.fail(f"Failed to import from feagi.api.gateway.api_gateway: {e}")
    
    def test_import_equivalence(self):
        """Test that imports from different locations refer to the same class."""
        # Test 4: Check if imports refer to the same class
        try:
            from feagi.api import APIGateway as APIGateway1
            from feagi.api.gateway import APIGateway as APIGateway2
            from feagi.api.gateway.api_gateway import APIGateway as APIGateway3
            
            self.assertIs(APIGateway1, APIGateway2, "APIGateway1 is not APIGateway2")
            self.assertIs(APIGateway1, APIGateway3, "APIGateway1 is not APIGateway3")
            self.assertIs(APIGateway2, APIGateway3, "APIGateway2 is not APIGateway3")
        except ImportError as e:
            self.fail(f"Failed during class comparison: {e}")
    
    def test_factory_function(self):
        """Test that get_api_gateway returns the same instance from different imports."""
        # Test 5: Check if get_api_gateway returns the same instance
        try:
            from feagi.api import get_api_gateway as get_api_gateway1
            from feagi.api.gateway import get_api_gateway as get_api_gateway2
            
            g1 = get_api_gateway1()
            g2 = get_api_gateway2()
            
            self.assertIs(g1, g2, "get_api_gateway1() does not return same instance as get_api_gateway2()")
        except ImportError as e:
            self.fail(f"Failed during factory function test: {e}")
            
    def test_singleton_pattern(self):
        """Test that the singleton pattern works correctly."""
        # Test 6: Test singleton pattern
        try:
            from feagi.api import APIGateway
            
            g1 = APIGateway()
            g2 = APIGateway()
            
            self.assertIs(g1, g2, "APIGateway singleton pattern is not working")
        except Exception as e:
            self.fail(f"Failed during singleton test: {e}")


if __name__ == "__main__":
    unittest.main() 