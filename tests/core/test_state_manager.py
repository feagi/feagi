import os
import pytest
import tempfile
from feagi.core.state_manager import (
    FeagiStateManager, GenomeState, ConnectomeState, 
    ServiceState, SimulationState
)

@pytest.fixture
def temp_state_file():
    """Create a temporary file for state testing"""
    fd, path = tempfile.mkstemp()
    os.close(fd)
    yield path
    # Cleanup after test
    if os.path.exists(path):
        os.unlink(path)

@pytest.fixture
def state_manager(temp_state_file):
    """Create a state manager instance with a temp file"""
    # Don't use the singleton to ensure test isolation
    return FeagiStateManager(temp_state_file)

def test_init_creates_file(temp_state_file):
    """Test that initializing creates the file if it doesn't exist"""
    if os.path.exists(temp_state_file):
        os.unlink(temp_state_file)
    
    assert not os.path.exists(temp_state_file)
    FeagiStateManager(temp_state_file)
    assert os.path.exists(temp_state_file)

def test_singleton_pattern():
    """Test that the singleton pattern works correctly"""
    instance1 = FeagiStateManager.instance()
    instance2 = FeagiStateManager.instance()
    assert instance1 is instance2

def test_genome_state(state_manager):
    """Test getting and setting genome state"""
    # Default should be MISSING
    assert state_manager.get_genome_state() == GenomeState.MISSING
    
    # Test setting and getting
    state_manager.set_genome_state(GenomeState.LOADING)
    assert state_manager.get_genome_state() == GenomeState.LOADING
    
    # Test another value
    state_manager.set_genome_state(GenomeState.LOADED)
    assert state_manager.get_genome_state() == GenomeState.LOADED

def test_connectome_state(state_manager):
    """Test getting and setting connectome state"""
    # Default should be MISSING
    assert state_manager.get_connectome_state() == ConnectomeState.MISSING
    
    # Test setting and getting
    state_manager.set_connectome_state(ConnectomeState.INITIALIZING)
    assert state_manager.get_connectome_state() == ConnectomeState.INITIALIZING
    
    # Test another value
    state_manager.set_connectome_state(ConnectomeState.READY)
    assert state_manager.get_connectome_state() == ConnectomeState.READY

def test_service_states(state_manager):
    """Test getting and setting various service states"""
    # API State
    assert state_manager.get_api_state() == ServiceState.UNAVAILABLE
    state_manager.set_api_state(ServiceState.READY)
    assert state_manager.get_api_state() == ServiceState.READY
    
    # ZMQ State
    assert state_manager.get_zmq_state() == ServiceState.UNAVAILABLE
    state_manager.set_zmq_state(ServiceState.INITIALIZING)
    assert state_manager.get_zmq_state() == ServiceState.INITIALIZING
    
    # Burst Engine State
    assert state_manager.get_burst_engine_state() == ServiceState.UNAVAILABLE
    state_manager.set_burst_engine_state(ServiceState.READY)
    assert state_manager.get_burst_engine_state() == ServiceState.READY

def test_agent_count(state_manager):
    """Test getting and setting agent count"""
    assert state_manager.get_agent_count() == 0
    
    # Set to a specific value
    state_manager.set_agent_count(5)
    assert state_manager.get_agent_count() == 5
    
    # Update to another value
    state_manager.set_agent_count(10)
    assert state_manager.get_agent_count() == 10

def test_burst_frequency(state_manager):
    """Test getting and setting burst frequency"""
    assert state_manager.get_burst_frequency() == 0.0
    
    # Set to a specific value
    state_manager.set_burst_frequency(30.5)
    assert state_manager.get_burst_frequency() == pytest.approx(30.5)

def test_simulation_state(state_manager):
    """Test getting and setting simulation state"""
    assert state_manager.get_simulation_state() == SimulationState.STOPPED
    
    # Set to running
    state_manager.set_simulation_state(SimulationState.RUNNING)
    assert state_manager.get_simulation_state() == SimulationState.RUNNING
    
    # Set to paused
    state_manager.set_simulation_state(SimulationState.PAUSED)
    assert state_manager.get_simulation_state() == SimulationState.PAUSED

def test_state_version(state_manager):
    """Test that state_version increments properly"""
    initial_version = state_manager.get_state_version()
    
    # Update a state and check version increment
    state_manager.set_genome_state(GenomeState.LOADING)
    assert state_manager.get_state_version() == initial_version + 1
    
    # Another update
    state_manager.set_api_state(ServiceState.READY)
    assert state_manager.get_state_version() == initial_version + 2

def test_high_level_helpers(state_manager):
    """Test high-level helper methods"""
    # Initially nothing should be ready
    assert not state_manager.is_genome_loaded()
    assert not state_manager.is_connectome_ready()
    assert not state_manager.is_burst_engine_ready()
    assert not state_manager.is_simulation_running()
    
    # Set up states
    state_manager.set_genome_state(GenomeState.LOADED)
    state_manager.set_connectome_state(ConnectomeState.READY)
    state_manager.set_burst_engine_state(ServiceState.READY)
    state_manager.set_simulation_state(SimulationState.RUNNING)
    
    # Now all should be ready
    assert state_manager.is_genome_loaded()
    assert state_manager.is_connectome_ready()
    assert state_manager.is_burst_engine_ready()
    assert state_manager.is_simulation_running()

def test_sync_to_disk(state_manager, temp_state_file):
    """Test that sync_to_disk flushes to disk"""
    # Set some state
    state_manager.set_genome_state(GenomeState.LOADED)
    state_manager.sync_to_disk()
    
    # Create a new instance pointing to same file
    new_manager = FeagiStateManager(temp_state_file)
    
    # Should read the same state
    assert new_manager.get_genome_state() == GenomeState.LOADED 