"""
Implementation tests for the FCL Sampler.

This module contains tests for the FCLSampler class implementation from feagi.npu.burst_engine.
"""

import time
import threading
import pytest
from queue import Queue, Empty
from unittest.mock import Mock, patch, MagicMock

from feagi.npu.burst_engine import FCLSampler


class MockConnectomeManager:
    """Mock connectome manager for testing FCLSampler."""
    def __init__(self):
        self.area1 = Mock(id=1, properties={'fcl_sample_rate': 20})
        self.area2 = Mock(id=2, properties={'fcl_sample_rate': 5})
        self.area3 = Mock(id=3, properties={})  # No explicit rate
        
        self._areas = {
            1: self.area1,
            2: self.area2,
            3: self.area3
        }
        
    def get_fcl_for_area(self, area_id):
        """Return an FCL for a specific area."""
        return set([area_id * 10, area_id * 10 + 1])


class MockFCLManager:
    """Mock FCL manager for testing FCLSampler."""
    def __init__(self, should_raise_exception=False):
        self.get_global_fcl_called = 0
        self.get_area_fcl_calls = []
        self.should_raise_exception = should_raise_exception
        
    def get_global_fcl(self):
        """Get the global FCL."""
        self.get_global_fcl_called += 1
        if self.should_raise_exception:
            raise Exception("Test exception")
        return set([1, 2, 3])
        
    def get_area_fcl(self, area_id):
        """Get FCL for a specific area."""
        self.get_area_fcl_calls.append(area_id)
        if self.should_raise_exception:
            raise Exception(f"Test exception for area {area_id}")
        return set([area_id * 10, area_id * 10 + 1])


@pytest.fixture
def mock_connectome_manager():
    """Create a mock connectome manager for testing."""
    return MockConnectomeManager()


@pytest.fixture
def mock_fcl_manager():
    """Create a mock FCL manager for testing."""
    return MockFCLManager()


@pytest.fixture
def output_queue():
    """Create a queue for testing FCLSampler output."""
    return Queue(maxsize=10)


def test_fcl_sampler_initialization():
    """Test FCLSampler initialization with various parameters."""
    # Setup mocks
    fcl_manager = MockFCLManager()
    output_queue = Queue(maxsize=10)
    connectome_manager = MockConnectomeManager()
    
    # Initialize with necessary arguments
    sampler = FCLSampler(fcl_manager, 10, output_queue)
    
    assert sampler.fcl_manager == fcl_manager
    assert sampler.sample_frequency == 10
    assert sampler.output_queue == output_queue
    assert sampler.connectome_manager is None
    
    # Test with connectome manager
    sampler = FCLSampler(fcl_manager, 10, output_queue, connectome_manager)
    
    assert sampler.connectome_manager == connectome_manager


def test_fcl_sampler_with_connectome_manager():
    """Test FCLSampler behavior with a connectome manager."""
    # Setup mocks
    fcl_manager = MockFCLManager()
    output_queue = Queue(maxsize=10)
    connectome_manager = MockConnectomeManager()
    
    # Initialize sampler
    sampler = FCLSampler(fcl_manager, 10, output_queue, connectome_manager)
    
    # Run sampler in a thread
    thread = threading.Thread(target=sampler.run)
    thread.daemon = True
    thread.start()
    
    # Let it run briefly
    time.sleep(0.2)
    
    # Stop the sampler
    sampler.stop()
    thread.join(timeout=1)
    
    # Check that we got samples
    samples = []
    while not output_queue.empty():
        try:
            samples.append(output_queue.get_nowait())
        except Empty:
            break
    
    assert len(samples) > 0
    
    # Check that get_area_fcl was called for areas
    assert len(fcl_manager.get_area_fcl_calls) > 0
    assert set(fcl_manager.get_area_fcl_calls).issubset({1, 2, 3})


def test_fcl_sampler_without_connectome_manager():
    """Test FCLSampler behavior without a connectome manager."""
    # Setup mocks
    fcl_manager = MockFCLManager()
    output_queue = Queue(maxsize=10)
    
    # Initialize sampler
    sampler = FCLSampler(fcl_manager, 10, output_queue)
    
    # Run sampler in a thread
    thread = threading.Thread(target=sampler.run)
    thread.daemon = True
    thread.start()
    
    # Let it run briefly
    time.sleep(0.2)
    
    # Stop the sampler
    sampler.stop()
    thread.join(timeout=1)
    
    # Check that we got samples
    samples = []
    while not output_queue.empty():
        try:
            samples.append(output_queue.get_nowait())
        except Empty:
            break
    
    assert len(samples) > 0
    
    # Check that get_global_fcl was called
    assert fcl_manager.get_global_fcl_called > 0


def test_fcl_sampler_with_error():
    """Test FCLSampler behavior when errors occur."""
    # Create FCL manager that raises exceptions
    error_fcl_manager = MockFCLManager(should_raise_exception=True)
    output_queue = Queue(maxsize=10)
    connectome_manager = MockConnectomeManager()
    
    # Create sampler with the error-raising FCL manager
    sampler = FCLSampler(error_fcl_manager, 10, output_queue, connectome_manager)
    
    # Mock the logger to check error logging
    with patch('feagi.npu.burst_engine.logger.error') as mock_logger:
        # Run sampler in a thread
        thread = threading.Thread(target=sampler.run)
        thread.daemon = True
        thread.start()
        
        # Let it run briefly
        time.sleep(0.2)
        
        # Stop the sampler
        sampler.stop()
        thread.join(timeout=1)
        
        # Check that errors were logged
        assert mock_logger.called


def test_fcl_sampler_update_area_sample_rate():
    """Test FCLSampler update_area_sample_rate method."""
    # Setup mocks
    fcl_manager = MockFCLManager()
    output_queue = Queue(maxsize=10)
    connectome_manager = MockConnectomeManager()
    
    # Initialize sampler with connectome manager
    sampler = FCLSampler(fcl_manager, 10, output_queue, connectome_manager)
    
    # Check initial sample rate
    assert connectome_manager.area1.properties['fcl_sample_rate'] == 20
    
    # Update sample rate
    sampler.update_area_sample_rate(1, 30)
    
    # Check updated sample rate
    assert connectome_manager.area1.properties['fcl_sample_rate'] == 30
    
    # Update non-existent area (should not raise exception)
    sampler.update_area_sample_rate(999, 40)
    
    # Update without connectome manager (should not raise exception)
    sampler = FCLSampler(fcl_manager, 10, output_queue)
    sampler.update_area_sample_rate(1, 50)


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 