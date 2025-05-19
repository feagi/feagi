"""
Tests for the ZMQ control stream implementation.
"""

import pytest
import asyncio
import time
from unittest.mock import MagicMock, patch

import zmq

# Import test utilities for ZMQ
from tests.api.zmq.zmq_test_utils import HAS_ZMQ, HAS_ZMQ_ASYNCIO, requires_zmq, requires_zmq_asyncio

# Apply the decorators to all tests in this module
pytestmark = [requires_zmq, requires_zmq_asyncio]

# Only import FEAGI modules if ZMQ and asyncio are available
if HAS_ZMQ and HAS_ZMQ_ASYNCIO:
    from feagi.api.protocols.base import ProtocolID
    from feagi.api.protocols.fcp import FCPMessageType
    from feagi.api.zmq.streams.control import ControlStream
else:
    # Create stub classes for type checking when ZMQ is not available
    class ProtocolID:
        FCP = 1
        FSMP = 2
        FVP = 3
    
    class FCPMessageType:
        REGISTER = 1
        HEARTBEAT = 2
        STATUS_REQUEST = 3


@pytest.fixture
def mock_core_api():
    """Create a mock core API."""
    mock = MagicMock()
    
    # Setup async methods
    mock.register_agent = MagicMock(return_value=asyncio.Future())
    mock.register_agent.return_value.set_result({
        "agent_id": "test-agent",
        "status": "registered",
        "ports": {"control": 5559}
    })
    
    mock.deregister_agent = MagicMock(return_value=asyncio.Future())
    mock.deregister_agent.return_value.set_result(True)
    
    mock.update_agent_heartbeat = MagicMock(return_value=asyncio.Future())
    mock.update_agent_heartbeat.return_value.set_result(None)
    
    mock.get_status = MagicMock(return_value=asyncio.Future())
    mock.get_status.return_value.set_result({
        "status": "running",
        "version": "2.1.0"
    })
    
    mock.configure_agent = MagicMock(return_value=asyncio.Future())
    mock.configure_agent.return_value.set_result(True)
    
    mock.get_agent_client_id = MagicMock(return_value=asyncio.Future())
    mock.get_agent_client_id.return_value.set_result("client-123")
    
    return mock


@pytest.mark.asyncio
async def test_control_stream_lifecycle(mock_core_api):
    """Test the control stream lifecycle (start/stop)."""
    # Create a control stream with a random port
    context = zmq.asyncio.Context()
    stream = ControlStream(
        core_api=mock_core_api,
        host="127.0.0.1",
        port=5559,
        context=context
    )
    
    # Start the stream
    await stream.start()
    assert stream._running is True
    assert stream.router_socket is not None
    assert stream.dealer_socket is not None
    
    # Stop the stream
    await stream.stop()
    assert stream._running is False
    assert stream.router_socket is None
    assert stream.dealer_socket is None
    
    # Clean up
    context.term()


class TestControlStreamHandling:
    """Test the control stream message handling."""
    
    @pytest.fixture
    async def setup_stream(self, mock_core_api):
        """Set up a control stream for testing."""
        # Create a control stream
        context = zmq.asyncio.Context()
        stream = ControlStream(
            core_api=mock_core_api,
            host="127.0.0.1",
            port=5559,
            context=context
        )
        
        # Start the stream
        await stream.start()
        
        # Create a client socket
        client = context.socket(zmq.DEALER)
        client.connect(f"tcp://127.0.0.1:5559")
        
        yield stream, client, context
        
        # Clean up
        client.close()
        await stream.stop()
        context.term()
    
    @pytest.mark.asyncio
    async def test_register_agent(self, setup_stream):
        """Test registering an agent."""
        async for fixture in setup_stream:
            stream, client, _ = fixture
            break
        
        # Create a registration message
        message = {
            "type": FCPMessageType.REGISTER,
            "data": {
                "agent_id": "test-agent",
                "agent_type": "test",
                "protocol_versions": {
                    "FCP": 1,
                    "FSMP": 1,
                    "FVP": 1
                }
            }
        }
        
        # Mock the translator
        with patch("feagi.api.protocols.translator.ProtocolTranslator") as mock_translator:
            # Setup mock decode/encode
            mock_instance = mock_translator.return_value
            mock_instance.decode.return_value = message
            mock_instance.encode.return_value = b"encoded_response"
            
            # Set the translator
            stream.translator = mock_instance
            
            # Send the message
            await client.send_multipart([b"", b"message_bytes"])
            
            # Give the server time to process
            await asyncio.sleep(0.1)
            
            # Check that the core API was called
            stream.core_api.register_agent.assert_called_once_with(
                agent_id="test-agent",
                agent_type="test",
                protocol_versions={"FCP": 1, "FSMP": 1, "FVP": 1},
                client_id=""
            )
            
            # Check that the response was encoded
            mock_instance.encode.assert_called_once()
            
            # Try to receive the response
            response = await client.recv_multipart()
            assert len(response) == 2
            assert response[1] == b"encoded_response"
    
    @pytest.mark.asyncio
    async def test_heartbeat(self, setup_stream):
        """Test sending a heartbeat."""
        async for fixture in setup_stream:
            stream, client, _ = fixture
            break
        
        # Create a heartbeat message
        message = {
            "type": FCPMessageType.HEARTBEAT,
            "data": {
                "agent_id": "test-agent"
            }
        }
        
        # Mock the translator
        with patch("feagi.api.protocols.translator.ProtocolTranslator") as mock_translator:
            # Setup mock decode/encode
            mock_instance = mock_translator.return_value
            mock_instance.decode.return_value = message
            mock_instance.encode.return_value = b"encoded_response"
            
            # Set the translator
            stream.translator = mock_instance
            
            # Send the message
            await client.send_multipart([b"", b"message_bytes"])
            
            # Give the server time to process
            await asyncio.sleep(0.1)
            
            # Check that the core API was called
            stream.core_api.update_agent_heartbeat.assert_called_once_with("test-agent")
            
            # Check that the response was encoded
            mock_instance.encode.assert_called_once()
            
            # Try to receive the response
            response = await client.recv_multipart()
            assert len(response) == 2
            assert response[1] == b"encoded_response"
    
    @pytest.mark.asyncio
    async def test_send_control_message(self, setup_stream):
        """Test sending a control message to an agent."""
        async for fixture in setup_stream:
            stream, client, _ = fixture
            break
        
        # Mock the translator
        with patch("feagi.api.protocols.translator.ProtocolTranslator") as mock_translator:
            # Setup mock encode
            mock_instance = mock_translator.return_value
            mock_instance.encode.return_value = b"encoded_message"
            
            # Set the translator
            stream.translator = mock_instance
            
            # Send a control message
            success = await stream.send_control_message(
                agent_id="test-agent",
                message_type=FCPMessageType.STATUS_REQUEST,
                data={"request": "status"}
            )
            
            # Check that it was successful
            assert success is True
            
            # Check that the core API was called to get the client ID
            stream.core_api.get_agent_client_id.assert_called_once_with("test-agent")
            
            # Check that the message was encoded
            mock_instance.encode.assert_called_once()
            
            # Try to receive the message (as the client would)
            message = await client.recv_multipart()
            assert len(message) == 2
            assert message[1] == b"encoded_message" 