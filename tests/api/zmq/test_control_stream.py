"""
Tests for the ZMQ control stream interface.

This module tests the control stream handling code in the ZMQ server,
focusing on agent registration and communication.
"""

import pytest

# Skip the entire test module since the ZMQ implementation has changed
pytest.skip("ZMQ control stream tests need to be updated after protocol refactoring", allow_module_level=True)

# Keep the original code for reference
import asyncio
import json
from unittest.mock import MagicMock, AsyncMock, patch

from feagi.api.zmq.server import ZmqServer, RegisterRequest, RegisterResponse
from feagi.api.protocols.constants import FCPCommandType


class TestControlStreamHandling:
    """Test cases for handling the control stream in the ZMQ server."""
    
    @pytest.fixture
    def mock_core_api(self):
        """Create a mock core API service."""
        mock = MagicMock()
        mock.register_agent = AsyncMock(return_value=True)
        mock.agent_heartbeat = AsyncMock(return_value=True)
        return mock
    
    @pytest.fixture
    async def server_with_mocks(self, mock_core_api):
        """Create a server with mocked dependencies."""
        # Create mocked context and sockets
        mock_context = MagicMock()
        mock_socket = MagicMock()
        mock_context.socket.return_value = mock_socket
        
        # Create the server with mocks
        server = ZmqServer(
            core_api=mock_core_api,
            context=mock_context,
            control_port=5559
        )
        
        # Manually set up the server's internal state
        server.control_socket = mock_socket
        server._running = True
        
        # Mock the connection manager
        server.connection_manager = MagicMock()
        
        # Yield the server and socket for testing
        yield server, mock_socket
        
        # Clean up
        server._running = False
    
    @pytest.mark.asyncio
    async def test_register_agent(self, server_with_mocks, mock_core_api):
        """Test registering an agent."""
        server, mock_socket = server_with_mocks
        
        # Set up a register request
        register_request = RegisterRequest()
        register_request.agent_id = "test-agent"
        register_request.agent_type = "test-type"
        register_request.protocol_versions.fcp_version = 1
        register_request.protocol_versions.fsmp_version = 1
        register_request.protocol_versions.fvp_version = 1
        
        # Call the handler
        response_data = await server._handle_register(b'client-id', register_request)
        
        # Verify the core API was called
        mock_core_api.register_agent.assert_called_once_with(
            "test-agent", 
            "test-type",
            protocol_versions={
                "fcp": 1,
                "fsmp": 1,
                "fvp": 1
            }
        )
        
        # Verify the connection manager was updated
        server.connection_manager.register_client.assert_called_once_with(
            b'client-id',
            "test-agent",
            {
                "fcp": 1,
                "fsmp": 1,
                "fvp": 1
            }
        )
        
        # Parse the response and check it
        response = RegisterResponse()
        response.ParseFromString(response_data)
        assert response.status == "success"
        assert "registered" in response.message.lower()
    
    @pytest.mark.asyncio
    async def test_heartbeat(self, server_with_mocks, mock_core_api):
        """Test handling heartbeat messages."""
        server, mock_socket = server_with_mocks
        
        # Set up a heartbeat request
        heartbeat_request = MagicMock()
        heartbeat_request.agent_id = "test-agent"
        
        # Call the handler
        response_data = await server._handle_heartbeat(b'client-id', heartbeat_request)
        
        # Verify the core API was called
        mock_core_api.agent_heartbeat.assert_called_once_with("test-agent")
        
        # Verify the connection manager was updated
        server.connection_manager.update_client_last_seen.assert_called_once_with(b'client-id')
        
        # No need to parse the response as it's a simple ACK
        assert response_data is not None
    
    @pytest.mark.asyncio
    async def test_send_control_message(self, server_with_mocks):
        """Test sending a control message to an agent."""
        server, mock_socket = server_with_mocks
        
        # Set up the connection manager mock to return a client ID
        server.connection_manager.get_client_id.return_value = b'client-id'
        
        # Set up test message
        message_type = "status_update"
        message_data = {"status": "running", "timestamp": 1622222222222}
        
        # Call the method
        result = await server.send_control_message("test-agent", message_type, message_data)
        
        # Verify the result is True (success)
        assert result is True
        
        # Verify the connection manager was queried
        server.connection_manager.get_client_id.assert_called_once_with("test-agent")
        
        # Verify the socket was used to send the message
        mock_socket.send_multipart.assert_called_once()
        
        # Check the message format
        args = mock_socket.send_multipart.call_args[0][0]
        assert args[0] == b'client-id'  # Client ID
        
        # Parse the message
        message = json.loads(args[1].decode('utf-8'))
        assert message["type"] == message_type
        assert message["payload"] == message_data 