#!/usr/bin/env python
"""
Comprehensive test script for the API Gateway implementation.
"""

import os
import sys
import unittest
from typing import Dict, Any
from unittest.mock import MagicMock, patch

# Import the gateway
from feagi.api import APIGateway, get_api_gateway
from feagi.api.core.services import CoreAPIService


class TestAPIGateway(unittest.TestCase):
    """Test cases for the API Gateway implementation."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Reset the singleton before each test
        APIGateway._instance = None
        
    def test_singleton_pattern(self):
        """Test that the gateway implements the singleton pattern correctly."""
        # Create two instances and verify they're the same object
        g1 = APIGateway()
        g2 = APIGateway()
        
        self.assertIs(g1, g2, "Multiple instances should be the same object")
        
        # Use the factory function and verify it returns the same instance
        g3 = get_api_gateway()
        
        self.assertIs(g1, g3, "Factory function should return the same singleton instance")
    
    def test_gateway_properties(self):
        """Test that gateway properties are initialized correctly."""
        mock_core_api = MagicMock(spec=CoreAPIService)
        gateway = APIGateway(core_api=mock_core_api)
        
        # Test core_api property
        self.assertIs(gateway.core_api, mock_core_api, "core_api property should return the provided instance")
        
        # Test zmq_client property
        self.assertTrue(hasattr(gateway, "zmq_client"), "Gateway should have zmq_client property")
    
    def test_auto_initialization(self):
        """Test that the gateway auto-initializes when no core_api is provided."""
        with patch("feagi.api.gateway.api_gateway.logger") as mock_logger:
            gateway = APIGateway()  # No core_api provided
            
            # Should have logged about using mock core API
            mock_logger.warning.assert_any_call("Using mock Core API")
            
            # Should have a core_api property
            self.assertIsNotNone(gateway.core_api, "core_api should be auto-initialized")
            
    def test_authentication_methods(self):
        """Test authentication and authorization methods."""
        gateway = APIGateway()
        
        # Test authenticate method
        self.assertTrue(hasattr(gateway, "authenticate"), "Gateway should have authenticate method")
        self.assertTrue(callable(gateway.authenticate), "authenticate should be callable")
        
        # Simple authentication should work (placeholder implementation)
        self.assertTrue(gateway.authenticate({}), "Default authenticate should return True")
        
        # Test authorize method
        self.assertTrue(hasattr(gateway, "authorize"), "Gateway should have authorize method")
        self.assertTrue(callable(gateway.authorize), "authorize should be callable")
        
        # Simple authorization should work (placeholder implementation)
        self.assertTrue(gateway.authorize("test", "read", {}), "Default authorize should return True")
    
    def test_route_request_method(self):
        """Test the route_request method."""
        gateway = APIGateway()
        
        # Test route_request method exists
        self.assertTrue(hasattr(gateway, "route_request"), "Gateway should have route_request method")
        self.assertTrue(callable(gateway.route_request), "route_request should be callable")
        
    def test_rate_limiting(self):
        """Test rate limiting functionality."""
        gateway = APIGateway()
        
        # Test rate limiting method
        self.assertTrue(hasattr(gateway, "check_rate_limit"), "Gateway should have check_rate_limit method")
        self.assertTrue(callable(gateway.check_rate_limit), "check_rate_limit should be callable")
        
        # Default implementation should allow all requests
        self.assertTrue(gateway.check_rate_limit("client1", "/api/test"), 
                       "Default rate limit implementation should return True")
    
    def test_monitoring_methods(self):
        """Test monitoring functionality."""
        gateway = APIGateway()
        
        # Test record_request method
        self.assertTrue(hasattr(gateway, "record_request"), "Gateway should have record_request method")
        self.assertTrue(callable(gateway.record_request), "record_request should be callable")
        
        # Test should not raise exceptions
        try:
            gateway.record_request("REST", "/api/test", 200, 0.1)
            passed = True
        except Exception:
            passed = False
        
        self.assertTrue(passed, "record_request should not raise exceptions")
        
        # Test get_metrics method
        self.assertTrue(hasattr(gateway, "get_metrics"), "Gateway should have get_metrics method")
        self.assertTrue(callable(gateway.get_metrics), "get_metrics should be callable")
        
        # Should return a dictionary
        metrics = gateway.get_metrics()
        self.assertIsInstance(metrics, Dict, "get_metrics should return a dictionary")


if __name__ == "__main__":
    unittest.main() 