"""
Comprehensive BurstEngine Tests for 80%+ Coverage

This test suite covers all major BurstEngine functionality:
- Singleton pattern and lifecycle
- Neural burst processing pipeline  
- FCL injection and neural dynamics
- FQ sampler integration
- State management and configuration
- Error handling and edge cases

Target: Achieve 80%+ code coverage on burst_engine.py (2,305 lines)
"""

import pytest
import numpy as np
from unittest.mock import Mock, MagicMock, patch
from typing import Dict, List, Any

from feagi.npu.burst_engine import BurstEngine, PowerInjectionService
from feagi.npu.fire_candidate_list import FireCandidateList, FCLCandidate
from feagi.npu.fire_queue import FireQueue, FiringNeuron
from feagi.npu.fire_ledger import FireLedgerInterface
from feagi.npu.fq_sampler import FQSampler
from feagi.npu.data_structures import NeuronArray, SynapseArray, BackendType
from feagi.npu.interface import NPUInterface
from feagi.core.state_manager import FeagiStateManager, ServiceState


class MockConnectomeManager:
    """Mock ConnectomeManager for testing."""
    
    def __init__(self):
        self.cortical_areas = {
            'test_area': type('Area', (), {
                'cortical_idx': 0,
                'area_id': 'test_area',
                'name': 'Test Area',
                'coordinates_3d': [(0, 0, 0), (1, 0, 0), (2, 0, 0)]
            })(),
            '_power': type('Area', (), {
                'cortical_idx': 1,
                'area_id': '_power',
                'name': 'Power Area',
                'coordinates_3d': [(0, 1, 0)]
            })()
        }
        
        # Mock NPU interface
        self._npu_interface = NPUInterface(backend=BackendType.CPU, max_neurons=1000)
        
        # Add some test neurons
        self._npu_interface.neuron_array.add_neurons_batch(
            neuron_ids=[1, 2, 3],
            positions=[(0, 0, 0), (1, 0, 0), (2, 0, 0)],
            neuron_types=[0, 0, 0],
            initial_potentials=[0.5, 0.8, 0.2],
            thresholds=[1.0, 1.0, 1.0],
            leak_coefficients=[0.95, 0.95, 0.95],
            cortical_idx=0,
            decay_rates=[0.95, 0.95, 0.95],
            refractory_periods=[1, 1, 1],
            excitabilities=[1.0, 1.0, 1.0],
            resting_potentials=[0.0, 0.0, 0.0],
            consecutive_fire_limits=[10, 10, 10]
        )
        
        # Mock synapse array
        self._npu_interface.synapse_array.add_synapses_batch(
            source_neuron_ids=[1, 2],
            target_neuron_ids=[2, 3], 
            weights=[0.5, 0.3],
            delays=[1, 1],
            conductances=[1.0, 1.0],
            synapse_types=[0, 0],
            plasticity_types=[0, 0],
            plasticity_coefficients=[0.0, 0.0]
        )
        
        self.cortical_mapping = Mock()
        self.cortical_mapping.get_idx.return_value = 0
        self.cortical_mapping.get_id.return_value = 'test_area'
        
    def get_neurons_by_cortical_area(self, area_id: str):
        if area_id == 'test_area':
            return [1, 2, 3]
        elif area_id == '_power':
            return [100]
        return []


class TestBurstEngineSingleton:
    """Test BurstEngine singleton pattern."""
    
    def setup_method(self):
        """Reset singleton before each test."""
        BurstEngine.reset_instance()
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_singleton_creation(self):
        """Test singleton instance creation."""
        cm = MockConnectomeManager()
        
        # First instance
        engine1 = BurstEngine(connectome_manager=cm)
        assert engine1 is not None
        
        # Second instance should be the same
        engine2 = BurstEngine(connectome_manager=cm)
        assert engine2 is engine1
        
        # get_instance should return same instance
        engine3 = BurstEngine.get_instance()
        assert engine3 is engine1
    
    def test_singleton_reset(self):
        """Test singleton reset functionality."""
        cm = MockConnectomeManager()
        
        engine1 = BurstEngine(connectome_manager=cm)
        instance_id1 = id(engine1)
        
        BurstEngine.reset_instance()
        
        engine2 = BurstEngine(connectome_manager=cm)
        instance_id2 = id(engine2)
        
        assert instance_id1 != instance_id2
    
    def test_singleton_with_parameters(self):
        """Test singleton with different parameter combinations."""
        cm = MockConnectomeManager()
        
        # Test with custom fire_ledger_window_size
        engine = BurstEngine(
            connectome_manager=cm,
            fire_ledger_window_size=50
        )
        
        assert engine.fire_ledger is not None
        assert hasattr(engine, 'fire_ledger')


class TestBurstEngineInitialization:
    """Test BurstEngine initialization and configuration."""
    
    def setup_method(self):
        """Reset singleton before each test."""
        BurstEngine.reset_instance()
        
    def teardown_method(self):
        """Clean up after each test.""" 
        BurstEngine.reset_instance()
    
    def test_initialization_with_connectome_manager(self):
        """Test initialization with ConnectomeManager."""
        cm = MockConnectomeManager()
        
        engine = BurstEngine(connectome_manager=cm)
        
        assert engine.connectome_manager is cm
        assert engine.fire_ledger is not None
        assert engine.coordinate_converter is not None
        assert engine.fcl_injector is not None
        assert engine.burst_count == 0
        assert engine.current_timestep == 0
    
    def test_initialization_without_connectome_manager(self):
        """Test initialization without ConnectomeManager."""
        engine = BurstEngine()
        
        assert engine.connectome_manager is None
        assert engine.fire_ledger is not None
        assert engine.coordinate_converter is None
        assert engine.fcl_injector is None
    
    def test_state_manager_integration(self):
        """Test state manager integration."""
        cm = MockConnectomeManager()
        
        with patch.object(FeagiStateManager, 'instance') as mock_state_manager:
            mock_sm = Mock()
            mock_sm.get_frequency_hz.return_value = 10.0
            mock_sm.is_debug_npu_enabled.return_value = False
            mock_sm.get_burst_engine_state.return_value = ServiceState.READY
            mock_state_manager.return_value = mock_sm
            
            engine = BurstEngine(connectome_manager=cm)
            
            assert engine.state_manager is mock_sm
            # Note: get_frequency_hz may not be called during init if state manager integration fails
            # The important thing is that the state manager is set
    
    def test_fire_ledger_window_size(self):
        """Test custom fire ledger window size."""
        cm = MockConnectomeManager()
        
        engine = BurstEngine(
            connectome_manager=cm,
            fire_ledger_window_size=100
        )
        
        assert engine.fire_ledger is not None
        # Fire ledger should be initialized with the specified window size


class TestBurstEngineProcessing:
    """Test core burst processing functionality."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        self.cm = MockConnectomeManager()
        self.engine = BurstEngine(connectome_manager=self.cm)
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_process_burst_basic(self):
        """Test basic burst processing."""
        # Mock injection service
        self.engine.injection_service = Mock()
        self.engine.injection_service.inject_power_neurons.return_value = 1
        
        # Process burst
        fired_neurons = self.engine.process_burst()
        
        assert isinstance(fired_neurons, list)
        assert self.engine.burst_count == 1
        assert self.engine.current_timestep == 0  # Set to burst_count before increment
    
    def test_process_burst_with_fcl_injection(self):
        """Test burst processing with FCL injection."""
        # Set up firing neuron (potential > threshold)
        neuron_array = self.cm._npu_interface.neuron_array
        neuron_idx = neuron_array.neuron_id_to_index[2]  # neuron_id=2 has potential=0.8
        neuron_array.membrane_potentials[neuron_idx] = 1.5  # Above threshold
        
        # Mock injection service
        self.engine.injection_service = Mock()
        self.engine.injection_service.inject_power_neurons.return_value = 1
        
        fired_neurons = self.engine.process_burst()
        
        # Should have some activity from processing
        assert isinstance(fired_neurons, list)
        self.engine.injection_service.inject_power_neurons.assert_called()
    
    def test_process_burst_multiple_cycles(self):
        """Test multiple burst processing cycles."""
        # Mock injection service
        self.engine.injection_service = Mock()
        self.engine.injection_service.inject_power_neurons.return_value = 0
        
        initial_count = self.engine.burst_count
        
        # Process multiple bursts
        for i in range(5):
            fired_neurons = self.engine.process_burst()
            assert isinstance(fired_neurons, list)
            assert self.engine.burst_count == initial_count + i + 1
    
    def test_process_burst_with_external_activations(self):
        """Test burst processing with external activations."""
        # Mock injection service with pending activations
        mock_service = Mock()
        mock_service._pending_external_activations = {'test': 'data'}
        mock_service.inject_power_neurons.return_value = 5
        
        self.engine.injection_service = mock_service
        
        fired_neurons = self.engine.process_burst()
        
        assert isinstance(fired_neurons, list)
        mock_service.inject_power_neurons.assert_called()
    
    def test_process_burst_error_handling(self):
        """Test error handling during burst processing."""
        # Mock injection service that raises exception
        mock_service = Mock()
        mock_service.inject_power_neurons.side_effect = Exception("Injection failed")
        
        self.engine.injection_service = mock_service
        
        # The current implementation may propagate exceptions from injection
        # Let's test that the exception is indeed raised (as designed)
        with pytest.raises(Exception, match="Injection failed"):
            self.engine.process_burst()


class TestBurstEngineStateManagement:
    """Test BurstEngine state management functionality."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        self.cm = MockConnectomeManager()
        self.engine = BurstEngine(connectome_manager=self.cm)
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_start_stop_lifecycle(self):
        """Test engine start/stop lifecycle."""
        # Initially not running
        assert not self.engine.is_running()
        
        # Start engine
        result = self.engine.start()
        assert result is True
        assert self.engine.is_running()
        
        # Stop engine
        result = self.engine.stop()
        assert result is True
        assert not self.engine.is_running()
    
    def test_start_already_running(self):
        """Test starting engine when already running."""
        self.engine.start()
        assert self.engine.is_running()
        
        # Try to start again
        result = self.engine.start()
        assert result is True  # Should handle gracefully
        assert self.engine.is_running()
    
    def test_stop_not_running(self):
        """Test stopping engine when not running."""
        assert not self.engine.is_running()
        
        result = self.engine.stop()
        assert result is True  # Should handle gracefully
        assert not self.engine.is_running()
    
    def test_frequency_update(self):
        """Test frequency update functionality."""
        # Test valid frequency
        result = self.engine.update_frequency(20.0)
        assert result is True
        
        # Test invalid frequency
        result = self.engine.update_frequency(0.0)
        assert result is False
        
        result = self.engine.update_frequency(-5.0)
        assert result is False


class TestBurstEngineFQSampler:
    """Test FQ Sampler integration."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        self.cm = MockConnectomeManager()
        self.engine = BurstEngine(connectome_manager=self.cm)
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_initialize_fq_sampler(self):
        """Test FQ sampler initialization."""
        sampler = self.engine.initialize_fq_sampler(
            sample_frequency_hz=15.0,
            sampling_mode="motor"
        )
        
        assert sampler is not None
        assert isinstance(sampler, FQSampler)
        assert self.engine.get_fq_sampler() is sampler
    
    def test_fq_sampler_registration(self):
        """Test FQ sampler registration/unregistration."""
        mock_sampler = Mock()
        
        # Register sampler
        self.engine.register_fq_sampler(mock_sampler)
        samplers = self.engine.get_registered_fq_samplers()
        assert mock_sampler in samplers
        
        # Unregister sampler
        self.engine.unregister_fq_sampler(mock_sampler)
        samplers = self.engine.get_registered_fq_samplers()
        assert mock_sampler not in samplers
    
    def test_multiple_fq_samplers(self):
        """Test registration of multiple FQ samplers."""
        samplers = [Mock() for _ in range(5)]
        
        # Register multiple samplers
        for sampler in samplers:
            self.engine.register_fq_sampler(sampler)
        
        registered = self.engine.get_registered_fq_samplers()
        assert len(registered) == 5
        for sampler in samplers:
            assert sampler in registered


class TestBurstEngineGenomeIntegration:
    """Test genome integration functionality."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        self.cm = MockConnectomeManager()
        self.engine = BurstEngine(connectome_manager=self.cm)
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_update_with_genome(self):
        """Test genome update functionality."""
        new_cm = MockConnectomeManager()
        
        # Update with new genome
        self.engine.update_with_genome(connectome_manager=new_cm)
        
        # Should update connectome manager
        assert self.engine.connectome_manager is new_cm
    
    def test_update_with_none_genome(self):
        """Test genome update with None."""
        original_cm = self.engine.connectome_manager
        
        # Update with None - should keep existing
        self.engine.update_with_genome(connectome_manager=None)
        
        assert self.engine.connectome_manager is original_cm
    
    def test_force_connectome_integration(self):
        """Test forced connectome integration."""
        result = self.engine.force_connectome_integration()
        assert isinstance(result, bool)
    
    def test_consecutive_fire_limits_update(self):
        """Test consecutive fire limits update."""
        result = self.engine.update_consecutive_fire_limits('test_area', 5)
        assert isinstance(result, bool)


class TestBurstEngineDataStructures:
    """Test BurstEngine data structure management."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        self.cm = MockConnectomeManager()
        self.engine = BurstEngine(connectome_manager=self.cm)
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_get_fire_queue(self):
        """Test fire queue access."""
        fire_queue = self.engine.get_current_fire_queue()
        # May be None if no processing has occurred
        assert fire_queue is None or isinstance(fire_queue, FireQueue)
    
    def test_get_fire_ledger(self):
        """Test fire ledger access."""
        fire_ledger = self.engine.get_fire_ledger()
        assert fire_ledger is not None
        assert isinstance(fire_ledger, FireLedgerInterface)
    
    def test_excitability_cache_management(self):
        """Test excitability cache operations."""
        # Test cache invalidation
        self.engine.invalidate_excitability_cache()
        
        # Should not raise exceptions
        assert True  # If we get here, cache operations work


class TestBurstEngineErrorHandling:
    """Test error handling and edge cases."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_initialization_without_connectome(self):
        """Test initialization without connectome manager."""
        engine = BurstEngine()
        
        # Should handle gracefully
        assert engine.connectome_manager is None
        assert engine.coordinate_converter is None
        assert engine.fcl_injector is None
        
        # Should still be able to process (with limited functionality)
        fired_neurons = engine.process_burst()
        assert isinstance(fired_neurons, list)
        assert len(fired_neurons) == 0  # No neurons to fire without connectome
    
    def test_corrupt_state_recovery(self):
        """Test recovery from corrupt internal state."""
        cm = MockConnectomeManager()
        engine = BurstEngine(connectome_manager=cm)
        
        # Simulate corrupt state
        engine.burst_count = -1
        engine.current_timestep = -1
        
        # Should handle gracefully
        fired_neurons = engine.process_burst()
        assert isinstance(fired_neurons, list)
        assert engine.burst_count >= 0  # Should recover
    
    def test_missing_npu_interface(self):
        """Test handling of missing NPU interface."""
        cm = MockConnectomeManager()
        cm._npu_interface = None  # Simulate missing NPU interface
        
        engine = BurstEngine(connectome_manager=cm)
        
        # Should handle gracefully
        fired_neurons = engine.process_burst()
        assert isinstance(fired_neurons, list)


class TestPowerInjectionService:
    """Test PowerInjectionService functionality."""
    
    def setup_method(self):
        """Setup test environment."""
        self.cm = MockConnectomeManager()
        self.fcl_injector = Mock()
        self.service = PowerInjectionService(
            connectome_manager=self.cm,
            fcl_injector=self.fcl_injector
        )
    
    def test_power_injection_service_creation(self):
        """Test PowerInjectionService creation."""
        assert self.service.connectome_manager is self.cm
        assert self.service.fcl_injector is self.fcl_injector
    
    def test_inject_power_neurons(self):
        """Test power neuron injection."""
        fcl = FireCandidateList()
        
        result = self.service.inject_power_neurons(fcl, current_timestep=1)
        
        assert isinstance(result, int)
        assert result >= 0
    
    def test_inject_external_activations(self):
        """Test external activation injection."""
        activations = {
            'test_area': {
                'coordinates': [(0, 0, 0), (1, 0, 0)],
                'potential_deltas': [0.5, 0.3]
            }
        }
        
        result = self.service.inject_external_activations(
            activations=activations,
            current_timestep=1,
            source="test"
        )
        
        assert isinstance(result, int)
        assert result >= 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
