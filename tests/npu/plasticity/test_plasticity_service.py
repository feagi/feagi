"""
Test suite for PlasticityService thread and command queue operations.

Tests the PlasticityService including:
- Thread lifecycle management
- Burst notification system
- Command queue operations with drop-on-full policy
- Memory area registration
- Statistics collection
- Error handling and robustness

Version: 3.0
"""

import pytest
import threading
import time
import queue
from typing import Dict, List
from unittest.mock import Mock, patch, MagicMock

from feagi.npu.fire_ledger import FireLedgerInterface
from feagi.npu.fire_queue import FiringNeuron
from feagi.npu.plasticity.service import PlasticityService, PlasticityConfig
from feagi.npu.plasticity.memory_neuron_array import MemoryNeuronArray, MemoryNeuronLifecycleConfig
from feagi.npu.plasticity.pattern_detector import PatternConfig


def create_firing_neurons(neuron_data: Dict[int, List[int]]) -> Dict[int, List[FiringNeuron]]:
    """Helper function to create FiringNeuron objects from simple data."""
    neurons_by_area = {}
    for area_idx, neuron_ids in neuron_data.items():
        firing_neurons = []
        for neuron_id in neuron_ids:
            firing_neuron = FiringNeuron(
                neuron_id=neuron_id,
                cortical_idx=area_idx,
                membrane_potential=1.5,
                coordinates=(0, 0, 0),
                threshold=1.0,
                pre_fire_potential=0.8,
                consecutive_fire_count=1,
                refractory_counter=0
            )
            firing_neurons.append(firing_neuron)
        neurons_by_area[area_idx] = firing_neurons
    return neurons_by_area


class MockNPUInterface:
    """Mock NPU interface for testing."""
    
    def __init__(self, queue_capacity: int = 100):
        self.queue_capacity = queue_capacity
        self.command_queue = queue.Queue(maxsize=queue_capacity)
        self.plasticity_queue_configured = False
        
    def configure_plasticity_queue(self, capacity: int):
        """Configure plasticity command queue."""
        self.plasticity_queue_configured = True
        self.queue_capacity = capacity
        
    def enqueue_plasticity_commands(self, commands: list):
        """Enqueue plasticity commands with drop-on-full policy."""
        if self.command_queue.full():
            raise queue.Full("Command queue is full")
        
        for command in commands:
            self.command_queue.put(command, block=False)
    
    def get_queued_commands(self) -> list:
        """Get all queued commands (for testing)."""
        commands = []
        while not self.command_queue.empty():
            try:
                commands.append(self.command_queue.get_nowait())
            except queue.Empty:
                break
        return commands


class MockStateManager:
    """Mock state manager for testing."""
    
    def __init__(self):
        self.plasticity_dropped_ops = 0
        
    def increment_plasticity_dropped_ops(self, count: int):
        """Track dropped operations."""
        self.plasticity_dropped_ops += count
        
    def get_plasticity_counters(self) -> dict:
        """Get plasticity counters."""
        return {'dropped_ops': self.plasticity_dropped_ops}


class TestPlasticityService:
    """Test PlasticityService thread operations."""
    
    def fire_ledger(self):
        """Create Fire Ledger for testing."""
        return FireLedgerInterface(default_window_size=10)
    
    def npu_interface(self):
        """Create mock NPU interface."""
        return MockNPUInterface(queue_capacity=10)
    
    def state_manager(self):
        """Create mock state manager."""
        return MockStateManager()
    
    def memory_neuron_array(self):
        """Create memory neuron array."""
        return MemoryNeuronArray(capacity=100)
    
    def plasticity_config(self):
        """Create plasticity configuration."""
        return PlasticityConfig(
            queue_capacity=10,
            max_ops_per_burst=5,
            stdp={
                'lookback_steps': 20,
                'tau_pre': 20.0,
                'tau_post': 20.0,
                'a_plus': 0.01,
                'a_minus': 0.012
            },
            memory={
                'lookback_steps': 50,
                'pattern_duration': 10,
                'min_activation_count': 3,
                'default_temporal_depth': 3,
                'pattern_cache_size': 100,
                'initial_lifespan': 20,
                'lifespan_growth_rate': 3.0,
                'longterm_threshold': 100,
                'max_reactivations': 1000
            }
        )
    
    def test_service_initialization(
        self, 
        fire_ledger, 
        npu_interface, 
        plasticity_config, 
        state_manager,
        memory_neuron_array
    ):
        """Test PlasticityService initialization."""
        
        service = PlasticityService(
            fire_ledger=fire_ledger,
            npu_interface=npu_interface,
            plasticity_config=plasticity_config,
            state_manager=state_manager,
            memory_neuron_array=memory_neuron_array
        )
        
        # Verify initialization
        assert service._ledger is fire_ledger
        assert service._npu is npu_interface
        assert service._cfg is plasticity_config
        assert service._state_manager is state_manager
        assert service._memory_neuron_array is memory_neuron_array
        assert not service._running
        assert service._latest_timestep == -1
        
        # Verify NPU interface configuration
        assert npu_interface.plasticity_queue_configured
        assert npu_interface.queue_capacity == plasticity_config.queue_capacity
        
        print("✅ Service initialization test passed")
    
    def test_thread_lifecycle(
        self, 
        fire_ledger, 
        npu_interface, 
        plasticity_config, 
        state_manager,
        memory_neuron_array
    ):
        """Test thread start/stop lifecycle."""
        
        service = PlasticityService(
            fire_ledger=fire_ledger,
            npu_interface=npu_interface,
            plasticity_config=plasticity_config,
            state_manager=state_manager,
            memory_neuron_array=memory_neuron_array
        )
        
        # Test start
        assert not service._running
        service.start()
        assert service._running
        
        # Give thread time to start
        time.sleep(0.1)
        
        # Test stop
        service.stop()
        assert not service._running
        
        print("✅ Thread lifecycle test passed")
    
    def test_burst_notification(
        self, 
        fire_ledger, 
        npu_interface, 
        plasticity_config, 
        state_manager,
        memory_neuron_array
    ):
        """Test burst notification system."""
        
        service = PlasticityService(
            fire_ledger=fire_ledger,
            npu_interface=npu_interface,
            plasticity_config=plasticity_config,
            state_manager=state_manager,
            memory_neuron_array=memory_neuron_array
        )
        
        # Start service
        service.start()
        time.sleep(0.1)  # Let thread start
        
        # Test burst notifications
        test_timesteps = [100, 101, 102, 103]
        
        for timestep in test_timesteps:
            service.notify_burst(timestep)
            time.sleep(0.05)  # Give time to process
            
            # Verify timestep was received
            assert service._latest_timestep == timestep
        
        # Stop service
        service.stop()
        
        print("✅ Burst notification test passed")
    
    def test_memory_area_registration(
        self, 
        fire_ledger, 
        npu_interface, 
        plasticity_config, 
        state_manager,
        memory_neuron_array
    ):
        """Test memory area registration."""
        
        service = PlasticityService(
            fire_ledger=fire_ledger,
            npu_interface=npu_interface,
            plasticity_config=plasticity_config,
            state_manager=state_manager,
            memory_neuron_array=memory_neuron_array
        )
        
        # Register memory areas
        success1 = service.register_memory_area(
            area_idx=42,
            temporal_depth=5,
            upstream_areas=[1, 2, 3]
        )
        assert success1
        
        success2 = service.register_memory_area(
            area_idx=43,
            temporal_depth=7,
            upstream_areas=[4, 5, 6],
            lifecycle_config=MemoryNeuronLifecycleConfig(
                initial_lifespan=30,
                lifespan_growth_rate=5.0,
                longterm_threshold=150
            )
        )
        assert success2
        
        # Verify registration
        assert 42 in service._memory_areas
        assert 43 in service._memory_areas
        
        assert service._memory_areas[42]['temporal_depth'] == 5
        assert service._memory_areas[42]['upstream_areas'] == [1, 2, 3]
        
        assert service._memory_areas[43]['temporal_depth'] == 7
        assert service._memory_areas[43]['upstream_areas'] == [4, 5, 6]
        
        # Verify lifecycle configs
        assert 42 in service._memory_lifecycle_configs
        assert 43 in service._memory_lifecycle_configs
        
        config43 = service._memory_lifecycle_configs[43]
        assert config43.initial_lifespan == 30
        assert config43.lifespan_growth_rate == 5.0
        assert config43.longterm_threshold == 150
        
        print("✅ Memory area registration test passed")
    
    def test_command_queue_operations(
        self, 
        fire_ledger, 
        npu_interface, 
        plasticity_config, 
        state_manager,
        memory_neuron_array
    ):
        """Test command queue operations and drop-on-full policy."""
        
        # Use small queue for testing
        small_npu = MockNPUInterface(queue_capacity=3)
        
        service = PlasticityService(
            fire_ledger=fire_ledger,
            npu_interface=small_npu,
            plasticity_config=plasticity_config,
            state_manager=state_manager,
            memory_neuron_array=memory_neuron_array
        )
        
        # Register memory area
        service.register_memory_area(
            area_idx=42,
            temporal_depth=3,
            upstream_areas=[1, 2]
        )
        
        # Create test activity in fire ledger
        test_patterns = [
            {1: [10, 11], 2: [20, 21]},
            {1: [11, 12], 2: [21, 22]},
            {1: [12, 13], 2: [22, 23]},
        ]
        
        for timestep, neuron_data in enumerate(test_patterns):
            neurons_by_area = create_firing_neurons(neuron_data)
            fire_ledger.archive_timestep(timestep, neurons_by_area)
        
        # Start service and process
        service.start()
        time.sleep(0.1)
        
        # Notify burst to trigger processing
        service.notify_burst(2)
        time.sleep(0.1)  # Give time to process
        
        # Check that commands were enqueued
        commands = small_npu.get_queued_commands()
        assert len(commands) > 0
        
        # Verify command structure
        for command in commands:
            assert 'type' in command
            assert command['type'] in ['create_memory_neuron', 'reactivate_memory_neuron']
            if command['type'] == 'create_memory_neuron':
                assert 'neuron_id' in command
                assert 'area_idx' in command
                assert 'pattern_hash' in command
                assert 'timestep' in command
        
        service.stop()
        
        print("✅ Command queue operations test passed")
    
    def test_drop_on_full_policy(
        self, 
        fire_ledger, 
        npu_interface, 
        plasticity_config, 
        state_manager,
        memory_neuron_array
    ):
        """Test drop-on-full policy when command queue is saturated."""
        
        # Create NPU interface that always reports full queue
        class FullQueueNPU(MockNPUInterface):
            def enqueue_plasticity_commands(self, commands: list):
                raise queue.Full("Queue is always full")
        
        full_npu = FullQueueNPU()
        
        service = PlasticityService(
            fire_ledger=fire_ledger,
            npu_interface=full_npu,
            plasticity_config=plasticity_config,
            state_manager=state_manager,
            memory_neuron_array=memory_neuron_array
        )
        
        # Register memory area
        service.register_memory_area(
            area_idx=42,
            temporal_depth=3,
            upstream_areas=[1, 2]
        )
        
        # Create activity
        fire_ledger.archive_timestep(0, create_firing_neurons({1: [10], 2: [20]}))
        fire_ledger.archive_timestep(1, create_firing_neurons({1: [11], 2: [21]}))
        fire_ledger.archive_timestep(2, create_firing_neurons({1: [12], 2: [22]}))
        
        # Start service
        service.start()
        time.sleep(0.1)
        
        initial_dropped = state_manager.plasticity_dropped_ops
        
        # Notify burst - should trigger command generation and dropping
        service.notify_burst(2)
        time.sleep(0.1)
        
        # Verify operations were dropped
        final_dropped = state_manager.plasticity_dropped_ops
        assert final_dropped > initial_dropped
        
        service.stop()
        
        print("✅ Drop-on-full policy test passed")
    
    def test_memory_processing_pipeline(
        self, 
        fire_ledger, 
        npu_interface, 
        plasticity_config, 
        state_manager,
        memory_neuron_array
    ):
        """Test complete memory processing pipeline."""
        
        service = PlasticityService(
            fire_ledger=fire_ledger,
            npu_interface=npu_interface,
            plasticity_config=plasticity_config,
            state_manager=state_manager,
            memory_neuron_array=memory_neuron_array
        )
        
        # Register memory area
        service.register_memory_area(
            area_idx=42,
            temporal_depth=3,
            upstream_areas=[1, 2, 3]
        )
        
        # Create temporal patterns that will repeat the same temporal sequence
        patterns = [
            {1: [10, 11], 2: [20, 21], 3: [30]},    # Timestep 0
            {1: [11, 12], 2: [21, 22], 3: [31]},    # Timestep 1
            {1: [12, 13], 2: [22, 23], 3: [32]},    # Timestep 2
            {1: [10, 11], 2: [20, 21], 3: [30]},    # Timestep 3 - same as 0
            {1: [11, 12], 2: [21, 22], 3: [31]},    # Timestep 4 - same as 1
            {1: [12, 13], 2: [22, 23], 3: [32]},    # Timestep 5 - same as 2
        ]
        
        for timestep, neuron_data in enumerate(patterns):
            neurons_by_area = create_firing_neurons(neuron_data)
            fire_ledger.archive_timestep(timestep, neurons_by_area)
        
        # Start service
        service.start()
        time.sleep(0.1)
        
        # Process first pattern (should create memory neuron)
        service.notify_burst(2)
        time.sleep(0.1)
        
        # Check memory neuron was created
        stats_after_first = memory_neuron_array.get_stats()
        assert stats_after_first.active_neurons > 0
        
        # Process the same temporal sequence again (timestep 5 has same sequence as timestep 2)
        # Timestep 2: [timestep 2, timestep 1, timestep 0]
        # Timestep 5: [timestep 5, timestep 4, timestep 3] = [same as 2, same as 1, same as 0]
        service.notify_burst(5)
        time.sleep(0.1)
        
        # Check that neuron was reactivated (not a new one created)
        stats_after_repeat = memory_neuron_array.get_stats()
        assert stats_after_repeat.active_neurons == stats_after_first.active_neurons
        
        # Check commands generated
        commands = npu_interface.get_queued_commands()
        
        # Should have both create and reactivate commands
        create_commands = [cmd for cmd in commands if cmd['type'] == 'create_memory_neuron']
        reactivate_commands = [cmd for cmd in commands if cmd['type'] == 'reactivate_memory_neuron']
        
        assert len(create_commands) > 0
        assert len(reactivate_commands) > 0
        
        service.stop()
        
        print("✅ Memory processing pipeline test passed")
    
    def test_statistics_collection(
        self, 
        fire_ledger, 
        npu_interface, 
        plasticity_config, 
        state_manager,
        memory_neuron_array
    ):
        """Test comprehensive statistics collection."""
        
        service = PlasticityService(
            fire_ledger=fire_ledger,
            npu_interface=npu_interface,
            plasticity_config=plasticity_config,
            state_manager=state_manager,
            memory_neuron_array=memory_neuron_array
        )
        
        # Register multiple memory areas
        service.register_memory_area(42, 3, [1, 2])
        service.register_memory_area(43, 5, [3, 4])
        
        # Get initial statistics
        initial_stats = service.get_memory_stats()
        
        assert 'service_stats' in initial_stats
        assert 'memory_array_stats' in initial_stats
        assert 'pattern_detector_stats' in initial_stats
        assert 'registered_memory_areas' in initial_stats
        assert 'memory_area_configs' in initial_stats
        
        assert initial_stats['registered_memory_areas'] == 2
        assert 42 in initial_stats['memory_area_configs']
        assert 43 in initial_stats['memory_area_configs']
        
        # Verify service stats structure
        service_stats = initial_stats['service_stats']
        expected_keys = [
            'memory_patterns_detected',
            'memory_neurons_created',
            'memory_neurons_reactivated',
            'memory_neurons_aged',
            'memory_neurons_converted_ltm',
            'plasticity_commands_enqueued',
            'plasticity_commands_dropped'
        ]
        
        for key in expected_keys:
            assert key in service_stats
            assert isinstance(service_stats[key], int)
        
        print("✅ Statistics collection test passed")
    
    def test_error_handling_robustness(
        self, 
        fire_ledger, 
        npu_interface, 
        plasticity_config, 
        state_manager,
        memory_neuron_array
    ):
        """Test error handling and service robustness."""
        
        service = PlasticityService(
            fire_ledger=fire_ledger,
            npu_interface=npu_interface,
            plasticity_config=plasticity_config,
            state_manager=state_manager,
            memory_neuron_array=memory_neuron_array
        )
        
        # Test registration with invalid parameters
        success = service.register_memory_area(
            area_idx=-1,  # Invalid area index
            temporal_depth=0,  # Invalid temporal depth
            upstream_areas=[]  # Empty upstream areas
        )
        # Should handle gracefully (implementation dependent)
        
        # Test with corrupted fire ledger
        with patch.object(fire_ledger, 'get_temporal_pattern_sequence', side_effect=Exception("Corrupted data")):
            service.start()
            time.sleep(0.1)
            
            # Should not crash the service
            service.notify_burst(100)
            time.sleep(0.1)
            
            assert service._running  # Service should still be running
            
            service.stop()
        
        # Test with memory neuron array errors
        with patch.object(memory_neuron_array, 'age_memory_neurons', side_effect=Exception("Memory error")):
            service.start()
            time.sleep(0.1)
            
            service.notify_burst(101)
            time.sleep(0.1)
            
            # Service should handle error gracefully
            assert service._running
            
            service.stop()
        
        print("✅ Error handling robustness test passed")


if __name__ == "__main__":
    # Run tests directly
    test_suite = TestPlasticityService()
    
    # Create fixtures
    fire_ledger = test_suite.fire_ledger()
    npu_interface = test_suite.npu_interface()
    state_manager = test_suite.state_manager()
    memory_neuron_array = test_suite.memory_neuron_array()
    plasticity_config = test_suite.plasticity_config()
    
    print("🧵 Running PlasticityService Tests...")
    print("=" * 50)
    
    try:
        test_suite.test_service_initialization(
            fire_ledger, npu_interface, plasticity_config, state_manager, memory_neuron_array
        )
        test_suite.test_thread_lifecycle(
            fire_ledger, npu_interface, plasticity_config, state_manager, memory_neuron_array
        )
        test_suite.test_burst_notification(
            fire_ledger, npu_interface, plasticity_config, state_manager, memory_neuron_array
        )
        test_suite.test_memory_area_registration(
            fire_ledger, npu_interface, plasticity_config, state_manager, memory_neuron_array
        )
        test_suite.test_command_queue_operations(
            fire_ledger, npu_interface, plasticity_config, state_manager, memory_neuron_array
        )
        test_suite.test_drop_on_full_policy(
            fire_ledger, npu_interface, plasticity_config, state_manager, memory_neuron_array
        )
        test_suite.test_memory_processing_pipeline(
            fire_ledger, npu_interface, plasticity_config, state_manager, memory_neuron_array
        )
        test_suite.test_statistics_collection(
            fire_ledger, npu_interface, plasticity_config, state_manager, memory_neuron_array
        )
        test_suite.test_error_handling_robustness(
            fire_ledger, npu_interface, plasticity_config, state_manager, memory_neuron_array
        )
        
        print("=" * 50)
        print("🎉 All PlasticityService Tests PASSED!")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        raise
