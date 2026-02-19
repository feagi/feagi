"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

# Tests for the FeagiClient class.

import asyncio
import pytest
from unittest.mock import MagicMock, patch, AsyncMock

from feagi_connector import FeagiClient
from feagi_connector.protocols import FSMPChannel

TEST_AGENT_DESCRIPTOR_B64 = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"


@pytest.fixture
def mock_command_client():
    """Create a mock command client."""
    mock = AsyncMock()
    mock.ping.return_value = {"status": "ok"}
    mock.get_status.return_value = {"status": {"running": True}}
    return mock


@pytest.fixture
def mock_sensory_client():
    """Create a mock sensory client."""
    mock = AsyncMock()
    mock.connect.return_value = True
    mock.register_agent.return_value = True
    mock.send_sensory_data.return_value = True
    mock.receive_motor_data.side_effect = [
        (FSMPChannel.MOTOR_ARM.value, b"arm_data"),
        None,  # No data on second call
    ]
    return mock


@pytest.fixture
def mock_viz_client():
    """Create a mock visualization client."""
    mock = AsyncMock()
    mock.connect.return_value = True
    mock.register_viz_agent.return_value = True
    mock.is_connected.return_value = True
    mock.receive_visualization_data.side_effect = [
        {"message_type": "activity", "data": {"neurons": {}}},
        None,  # No data on second call
    ]
    return mock


@pytest.fixture
def feagi_client(mock_command_client, mock_sensory_client, mock_viz_client):
    """Create a FeagiClient with mocked components."""
    with patch("feagi_connector.client.FeagiCommandClient", return_value=mock_command_client), \
         patch("feagi_connector.client.FeagiSensoryClient", return_value=mock_sensory_client), \
         patch("feagi_connector.client.FeagiVizClient", return_value=mock_viz_client):
        client = FeagiClient(
            host="test-host",
            agent_id=TEST_AGENT_DESCRIPTOR_B64,
        )
        yield client


@pytest.mark.asyncio
async def test_connect(feagi_client, mock_command_client, mock_sensory_client):
    """Test the connect method."""
    # Connect to FEAGI
    result = await feagi_client.connect()
    
    # Check result
    assert result is True
    
    # Verify calls
    mock_command_client.ping.assert_called_once()
    mock_sensory_client.connect.assert_called_once()
    mock_sensory_client.register_agent.assert_called_once_with(agent_type="external")
    
    # Check state
    assert feagi_client.connected is True


@pytest.mark.asyncio
async def test_connect_failure_ping(feagi_client, mock_command_client):
    """Test connect failure when ping fails."""
    # Set up mock to fail
    mock_command_client.ping.return_value = {"error": "timeout"}
    
    # Connect to FEAGI
    result = await feagi_client.connect()
    
    # Check result
    assert result is False
    
    # Check state
    assert feagi_client.connected is False


@pytest.mark.asyncio
async def test_disconnect(feagi_client):
    """Test the disconnect method."""
    # Set up client state
    feagi_client.connected = True
    feagi_client.heartbeat_task = asyncio.create_task(asyncio.sleep(0))
    
    # Disconnect from FEAGI
    await feagi_client.disconnect()
    
    # Check state
    assert feagi_client.connected is False
    assert feagi_client.heartbeat_task is None


@pytest.mark.asyncio
async def test_send_sensory_data(feagi_client, mock_sensory_client):
    """Test sending sensory data."""
    # Set up client state
    feagi_client.connected = True
    
    # Send sensory data with enum
    result = await feagi_client.send_sensory_data(FSMPChannel.VISION, b"test_data")
    
    # Check result
    assert result is True
    
    # Verify calls
    mock_sensory_client.send_sensory_data.assert_called_once_with(FSMPChannel.VISION.value, b"test_data")


@pytest.mark.asyncio
async def test_send_sensory_data_not_connected(feagi_client, mock_sensory_client):
    """Test sending sensory data when not connected."""
    # Set up client state
    feagi_client.connected = False
    
    # Send sensory data
    result = await feagi_client.send_sensory_data(FSMPChannel.VISION, b"test_data")
    
    # Check result
    assert result is False
    
    # Verify no calls
    mock_sensory_client.send_sensory_data.assert_not_called()


@pytest.mark.asyncio
async def test_register_motor_callback(feagi_client):
    """Test registering a motor callback."""
    # Set up client state
    feagi_client.connected = True
    
    # Define a callback
    callback = MagicMock()
    
    # Register the callback
    await feagi_client.register_motor_callback(callback)
    
    # Check that callback was registered
    assert feagi_client.motor_callback is callback
    assert feagi_client.motor_listen_task is not None


@pytest.mark.asyncio
async def test_register_visualization_callbacks(feagi_client, mock_viz_client):
    """Test registering visualization callbacks."""
    # Define callbacks
    activity_callback = MagicMock()
    structure_callback = MagicMock()
    
    # Register the callbacks
    result = await feagi_client.register_visualization_callbacks(
        activity_callback=activity_callback,
        structure_callback=structure_callback
    )
    
    # Check result
    assert result is True
    
    # Check that callbacks were registered
    assert feagi_client.activity_callback is activity_callback
    assert feagi_client.structure_callback is structure_callback
    assert feagi_client.viz_listen_task is not None
    
    # Verify calls
    mock_viz_client.is_connected.assert_called_once()
    mock_viz_client.register_viz_agent.assert_called_once()


@pytest.mark.asyncio
async def test_get_status(feagi_client, mock_command_client):
    """Test getting FEAGI status."""
    # Get status
    result = await feagi_client.get_status()
    
    # Check result
    assert result == {"status": {"running": True}}
    
    # Verify calls
    mock_command_client.get_status.assert_called_once()


@pytest.mark.asyncio
async def test_motor_listen_loop(feagi_client, mock_sensory_client):
    """Test the motor listen loop."""
    # Set up client state
    feagi_client.connected = True
    feagi_client.motor_callback = MagicMock()
    
    # Start the motor listen task
    task = asyncio.create_task(feagi_client._motor_listen_loop())
    
    # Let it run for a bit
    await asyncio.sleep(0.1)
    
    # Cancel the task
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    
    # Verify calls
    assert mock_sensory_client.receive_motor_data.call_count >= 1
    feagi_client.motor_callback.assert_called_once_with(FSMPChannel.MOTOR_ARM.value, b"arm_data")


@pytest.mark.asyncio
async def test_viz_listen_loop(feagi_client, mock_viz_client):
    """Test the visualization listen loop."""
    # Set up client state
    feagi_client.connected = True
    feagi_client.activity_callback = MagicMock()
    feagi_client.structure_callback = MagicMock()
    
    # Start the visualization listen task
    task = asyncio.create_task(feagi_client._viz_listen_loop())
    
    # Let it run for a bit
    await asyncio.sleep(0.1)
    
    # Cancel the task
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    
    # Verify calls
    assert mock_viz_client.receive_visualization_data.call_count >= 1
    feagi_client.activity_callback.assert_called_once()
    feagi_client.structure_callback.assert_not_called()  # No structure data was returned 