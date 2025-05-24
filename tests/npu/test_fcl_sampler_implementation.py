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
    """Test FCLSampler with connectome manager providing area-specific sample rates."""
    fcl_manager = MockFCLManager()
    output_queue = Queue(maxsize=10)
    connectome_manager = MockConnectomeManager()
    
    sampler = FCLSampler(fcl_manager, 50, output_queue, connectome_manager)
    
    # Enable visualization subscribers - THIS IS CRITICAL!
    sampler.set_visualization_subscribers(True)
    
    # Run for a short period
    t = threading.Thread(target=sampler.run)
    t.start()
    time.sleep(0.1)  # 100ms
    sampler.stop()
    t.join(timeout=2)
    
    # Check samples
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    assert len(samples) > 0
    # Check that we have samples from the mocked areas
    area_ids = [sample[0] for sample in samples if isinstance(sample, tuple)]
    assert len(area_ids) > 0


def test_fcl_sampler_without_connectome_manager():
    """Test FCLSampler without connectome manager (global sampling mode)."""
    fcl_manager = MockFCLManager()
    output_queue = Queue(maxsize=10)
    
    sampler = FCLSampler(fcl_manager, 50, output_queue)
    
    # Enable visualization subscribers - THIS IS CRITICAL!
    sampler.set_visualization_subscribers(True)
    
    # Run for a short period
    t = threading.Thread(target=sampler.run)
    t.start()
    time.sleep(0.1)  # 100ms
    sampler.stop()
    t.join(timeout=2)
    
    # Check samples
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    assert len(samples) > 0


def test_fcl_sampler_with_error():
    """Test FCLSampler error handling."""
    with patch('feagi.npu.burst_engine.logger') as mock_logger:
        fcl_manager = MockFCLManager(should_raise_exception=True)
        output_queue = Queue(maxsize=10)
        
        sampler = FCLSampler(fcl_manager, 50, output_queue)
        
        # Enable visualization subscribers - THIS IS CRITICAL!
        sampler.set_visualization_subscribers(True)
        
        # Run for a short period
        t = threading.Thread(target=sampler.run)
        t.start()
        time.sleep(0.1)  # 100ms
        sampler.stop()
        t.join(timeout=2)
        
        # Check that error was logged
        assert mock_logger.called


def test_fcl_sampler_update_area_sample_rate():
    """Test updating area sample rate."""
    fcl_manager = MockFCLManager()
    output_queue = Queue(maxsize=10)
    connectome_manager = MockConnectomeManager()
    
    sampler = FCLSampler(fcl_manager, 10, output_queue, connectome_manager)
    
    # Update sample rate
    sampler.update_area_sample_rate('area1', 30)
    
    # Check that the rate was updated
    assert connectome_manager.area1.properties['fcl_sample_rate'] == 30


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 