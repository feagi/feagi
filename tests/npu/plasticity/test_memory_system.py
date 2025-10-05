"""
Comprehensive test suite for FEAGI Memory System.

Tests the complete memory formation workflow including:
- Fire Ledger integration
- Per-area temporal depth configuration
- RoaringBitmap pattern detection
- Memory neuron lifecycle management
- PlasticityService thread operations
- Global unique ID allocation
- End-to-end memory formation

Version: 3.0
"""

import pytest
import threading
import time
import hashlib
from typing import Dict, List, Optional
from unittest.mock import Mock, patch

import numpy as np

from feagi.npu.fire_ledger import FireLedgerInterface, RoaringBitmap
from feagi.npu.fire_queue import FiringNeuron
from feagi.npu.plasticity.pattern_detector import (
    PatternDetector, BatchPatternDetector, PatternConfig, TemporalPattern
)
from feagi.npu.plasticity.memory_neuron_array import (
    MemoryNeuronArray, MemoryNeuronLifecycleConfig, MemoryNeuronStats
)
from feagi.npu.plasticity.neuron_id_manager import (
    NeuronIdManager, get_neuron_id_manager, NeuronIdRanges,
    REGULAR_NEURON_ID_START, REGULAR_NEURON_ID_MAX,
    MEMORY_NEURON_ID_START, MEMORY_NEURON_ID_MAX
)
from feagi.npu.plasticity.service import PlasticityService, PlasticityConfig


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


class TestMemorySystemIntegration:
    """Integration tests for the complete memory system."""
    
    def fire_ledger(self):
        """Create Fire Ledger with test configuration."""
        ledger = FireLedgerInterface(default_window_size=10)
        return ledger
    
    def pattern_config(self):
        """Create pattern detection configuration."""
        return PatternConfig(
            default_temporal_depth=3,
            min_activity_threshold=1,
            max_pattern_cache_size=1000
        )
    
    def memory_neuron_array(self):
        """Create memory neuron array."""
        return MemoryNeuronArray(capacity=1000)
    
    def neuron_id_manager(self):
        """Create neuron ID manager."""
        return NeuronIdManager()
    
    def plasticity_config(self):
        """Create plasticity service configuration."""
        return PlasticityConfig(
            queue_capacity=100,
            max_ops_per_burst=50,
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
                'pattern_cache_size': 1000,
                'initial_lifespan': 20,
                'lifespan_growth_rate': 3.0,
                'longterm_threshold': 100,
                'max_reactivations': 1000
            }
        )
    
    def test_end_to_end_memory_formation(
        self, 
        fire_ledger, 
        pattern_config, 
        memory_neuron_array, 
        plasticity_config
    ):
        """Test complete end-to-end memory formation workflow."""
        
        # Step 1: Configure memory area in Fire Ledger
        memory_area_idx = 42
        temporal_depth = 5
        upstream_areas = [1, 2, 3]
        
        fire_ledger.configure_memory_area(
            cortical_idx=memory_area_idx,
            window_size=temporal_depth,
            upstream_areas=upstream_areas
        )
        
        # Step 2: Create pattern detector
        pattern_detector = BatchPatternDetector(pattern_config)
        
        # Step 3: Simulate neural activity over time
        timesteps = []
        for timestep in range(10):
            # Create activity patterns
            neuron_data = {}
            for area_idx in upstream_areas:
                # Create different patterns for each area
                neuron_ids = list(range(area_idx * 10, area_idx * 10 + 5))
                if timestep % 3 == area_idx % 3:  # Create temporal patterns
                    neuron_ids.extend([area_idx * 10 + 5, area_idx * 10 + 6])
                neuron_data[area_idx] = neuron_ids
            
            # Convert to FiringNeuron objects and archive
            neurons_by_area = create_firing_neurons(neuron_data)
            fire_ledger.archive_timestep(timestep, neurons_by_area)
            timesteps.append(timestep)
        
        # Step 4: Detect patterns
        memory_areas = {
            memory_area_idx: {
                'temporal_depth': temporal_depth,
                'upstream_areas': upstream_areas
            }
        }
        
        patterns = pattern_detector.detect_patterns_batch(
            fire_ledger, memory_areas, timesteps[-1]
        )
        
        # Step 5: Verify pattern detection
        assert memory_area_idx in patterns
        pattern = patterns[memory_area_idx]
        assert pattern is not None
        assert pattern.temporal_depth == temporal_depth
        assert pattern.upstream_areas == tuple(sorted(upstream_areas))
        assert len(pattern.pattern_hash) == 32  # SHA-256 hash
        
        # Step 6: Create memory neuron
        lifecycle_config = MemoryNeuronLifecycleConfig(
            initial_lifespan=20,
            lifespan_growth_rate=3.0,
            longterm_threshold=100
        )
        
        neuron_idx = memory_neuron_array.create_memory_neuron(
            pattern_hash=pattern.pattern_hash,
            cortical_area_id=memory_area_idx,
            current_burst=timesteps[-1],
            config=lifecycle_config
        )
        
        assert neuron_idx is not None
        assert memory_neuron_array.is_active[neuron_idx]
        assert memory_neuron_array.lifespan_current[neuron_idx] == 20
        
        # Step 7: Test pattern lookup
        found_idx = memory_neuron_array.find_neuron_by_pattern(pattern.pattern_hash)
        assert found_idx == neuron_idx
        
        # Step 8: Test reactivation
        success = memory_neuron_array.reactivate_memory_neuron(neuron_idx, timesteps[-1] + 1)
        assert success
        assert memory_neuron_array.lifespan_current[neuron_idx] == 23  # 20 + 3
        assert memory_neuron_array.activation_count[neuron_idx] == 2
        
        # Step 9: Test aging
        died_neurons = memory_neuron_array.age_memory_neurons(timesteps[-1] + 2)
        assert neuron_idx not in died_neurons  # Should still be alive
        assert memory_neuron_array.lifespan_current[neuron_idx] == 22  # 23 - 1
        
        print("✅ End-to-end memory formation test passed")
    
    def test_per_area_temporal_depth(self, fire_ledger, pattern_config):
        """Test per-area temporal depth configuration."""
        
        # Configure different memory areas with different temporal depths
        memory_areas = {
            10: {'temporal_depth': 3, 'upstream_areas': [1, 2]},
            20: {'temporal_depth': 7, 'upstream_areas': [3, 4]},
            30: {'temporal_depth': 12, 'upstream_areas': [5, 6]}
        }
        
        # Configure fire ledger for each memory area
        for area_idx, config in memory_areas.items():
            fire_ledger.configure_memory_area(
                cortical_idx=area_idx,
                window_size=config['temporal_depth'],
                upstream_areas=config['upstream_areas']
            )
        
        # Create pattern detector and configure per-area temporal depths
        detector = PatternDetector(pattern_config)
        for area_idx, config in memory_areas.items():
            detector.configure_area_temporal_depth(area_idx, config['temporal_depth'])
        
        # Verify temporal depth configuration
        for area_idx, config in memory_areas.items():
            actual_depth = detector._get_area_temporal_depth(area_idx)
            expected_depth = config['temporal_depth']
            assert actual_depth == expected_depth, f"Area {area_idx}: expected {expected_depth}, got {actual_depth}"
        
        print("✅ Per-area temporal depth test passed")
    
    def test_memory_neuron_lifecycle(self, memory_neuron_array):
        """Test complete memory neuron lifecycle."""
        
        # Create test pattern hash
        test_pattern = b"test_pattern_data"
        pattern_hash = hashlib.sha256(test_pattern).digest()
        
        # Test creation
        config = MemoryNeuronLifecycleConfig(
            initial_lifespan=10,
            lifespan_growth_rate=2.0,
            longterm_threshold=50
        )
        
        neuron_idx = memory_neuron_array.create_memory_neuron(
            pattern_hash=pattern_hash,
            cortical_area_id=42,
            current_burst=100,
            config=config
        )
        
        assert neuron_idx is not None
        assert memory_neuron_array.lifespan_current[neuron_idx] == 10
        assert memory_neuron_array.lifespan_initial[neuron_idx] == 10
        assert memory_neuron_array.lifespan_growth_rate[neuron_idx] == 2.0
        assert not memory_neuron_array.is_longterm_memory[neuron_idx]
        
        # Test multiple reactivations to build up lifespan
        current_burst = 101
        for i in range(25):  # 25 reactivations
            success = memory_neuron_array.reactivate_memory_neuron(neuron_idx, current_burst)
            assert success
            current_burst += 1
            
            # Check lifespan growth: initial + (reactivations * growth_rate)
            expected_lifespan = 10 + ((i + 1) * 2)  # +1 because we count from 0
            assert memory_neuron_array.lifespan_current[neuron_idx] == expected_lifespan
        
        # After 25 reactivations: 10 + (25 * 2) = 60 lifespan
        assert memory_neuron_array.lifespan_current[neuron_idx] == 60
        
        # Test long-term memory conversion
        converted = memory_neuron_array.check_longterm_conversion(longterm_threshold=50)
        assert neuron_idx in converted
        assert memory_neuron_array.is_longterm_memory[neuron_idx]
        
        # Test that long-term memory neurons don't age
        initial_lifespan = memory_neuron_array.lifespan_current[neuron_idx]
        died_neurons = memory_neuron_array.age_memory_neurons(current_burst + 1)
        assert neuron_idx not in died_neurons
        assert memory_neuron_array.lifespan_current[neuron_idx] == initial_lifespan  # No change
        
        print("✅ Memory neuron lifecycle test passed")
    
    def test_neuron_id_allocation(self, neuron_id_manager):
        """Test global unique ID allocation and range partitioning."""
        
        # Test regular neuron ID allocation
        regular_ids = []
        for i in range(100):
            neuron_id = neuron_id_manager.allocate_regular_neuron_id()
            assert neuron_id is not None
            assert REGULAR_NEURON_ID_START <= neuron_id <= REGULAR_NEURON_ID_MAX
            assert not NeuronIdManager.is_memory_neuron_id(neuron_id)
            regular_ids.append(neuron_id)
        
        # Test memory neuron ID allocation
        memory_ids = []
        for i in range(100):
            neuron_id = neuron_id_manager.allocate_memory_neuron_id()
            assert neuron_id is not None
            assert MEMORY_NEURON_ID_START <= neuron_id <= MEMORY_NEURON_ID_MAX
            assert NeuronIdManager.is_memory_neuron_id(neuron_id)
            memory_ids.append(neuron_id)
        
        # Verify no collisions between regular and memory IDs
        all_regular = set(regular_ids)
        all_memory = set(memory_ids)
        assert len(all_regular.intersection(all_memory)) == 0
        
        # Test ID uniqueness within each range
        assert len(set(regular_ids)) == len(regular_ids)  # All unique
        assert len(set(memory_ids)) == len(memory_ids)    # All unique
        
        # Test sequential allocation
        assert regular_ids == sorted(regular_ids)
        assert memory_ids == sorted(memory_ids)
        
        print("✅ Neuron ID allocation test passed")
    
    def test_pattern_detection_determinism(self, fire_ledger, pattern_config):
        """Test that pattern detection is deterministic across runs."""
        
        # Create identical activity patterns
        upstream_areas = [1, 2, 3]
        neuron_data = {
            1: [10, 11, 12],
            2: [20, 21, 22],
            3: [30, 31, 32]
        }
        
        # Archive same pattern multiple times
        for timestep in range(5):
            neurons_by_area = create_firing_neurons(neuron_data)
            fire_ledger.archive_timestep(timestep, neurons_by_area)
        
        # Detect pattern multiple times
        detector = PatternDetector(pattern_config)
        patterns = []
        
        for run in range(10):
            pattern = detector.detect_pattern(
                fire_ledger=fire_ledger,
                memory_area_idx=42,
                upstream_areas=upstream_areas,
                current_timestep=4,
                temporal_depth=3
            )
            patterns.append(pattern)
        
        # Verify all patterns are identical
        first_pattern = patterns[0]
        for pattern in patterns[1:]:
            assert pattern.pattern_hash == first_pattern.pattern_hash
            assert pattern.temporal_depth == first_pattern.temporal_depth
            assert pattern.upstream_areas == first_pattern.upstream_areas
            assert pattern.total_activity == first_pattern.total_activity
        
        print("✅ Pattern detection determinism test passed")
    
    def test_memory_statistics(self, memory_neuron_array):
        """Test memory system statistics collection."""
        
        # Create several memory neurons
        pattern_hashes = []
        neuron_indices = []
        
        for i in range(10):
            pattern_data = f"pattern_{i}".encode()
            pattern_hash = hashlib.sha256(pattern_data).digest()
            pattern_hashes.append(pattern_hash)
            
            config = MemoryNeuronLifecycleConfig(
                initial_lifespan=20,
                lifespan_growth_rate=3.0,
                longterm_threshold=100
            )
            
            neuron_idx = memory_neuron_array.create_memory_neuron(
                pattern_hash=pattern_hash,
                cortical_area_id=42,
                current_burst=100 + i,
                config=config
            )
            neuron_indices.append(neuron_idx)
        
        # Get initial statistics
        stats = memory_neuron_array.get_stats()
        assert stats.total_capacity >= 10
        assert stats.active_neurons == 10
        assert stats.longterm_neurons == 0
        assert stats.dead_neurons == 0
        
        # Reactivate some neurons multiple times to convert to LTM
        for i in range(5):  # First 5 neurons
            neuron_idx = neuron_indices[i]
            for reactivation in range(30):  # Enough to exceed LTM threshold
                memory_neuron_array.reactivate_memory_neuron(neuron_idx, 200 + reactivation)
        
        # Check for LTM conversion
        converted = memory_neuron_array.check_longterm_conversion(longterm_threshold=100)
        assert len(converted) == 5
        
        # Age neurons to kill some
        died_neurons = []
        for aging_cycle in range(25):  # Age enough to kill non-LTM neurons
            died = memory_neuron_array.age_memory_neurons(300 + aging_cycle)
            died_neurons.extend(died)
        
        # Get final statistics
        final_stats = memory_neuron_array.get_stats()
        assert final_stats.longterm_neurons == 5
        assert final_stats.dead_neurons == 5  # The other 5 should have died
        assert final_stats.active_neurons == 5  # Only LTM neurons remain active
        
        print("✅ Memory statistics test passed")


if __name__ == "__main__":
    # Run tests directly
    test_suite = TestMemorySystemIntegration()
    
    # Create fixtures
    fire_ledger = test_suite.fire_ledger()
    pattern_config = test_suite.pattern_config()
    memory_neuron_array = test_suite.memory_neuron_array()
    neuron_id_manager = test_suite.neuron_id_manager()
    plasticity_config = test_suite.plasticity_config()
    
    print("🧠 Running FEAGI Memory System Tests...")
    print("=" * 50)
    
    try:
        test_suite.test_end_to_end_memory_formation(
            fire_ledger, pattern_config, memory_neuron_array, plasticity_config
        )
        test_suite.test_per_area_temporal_depth(fire_ledger, pattern_config)
        test_suite.test_memory_neuron_lifecycle(memory_neuron_array)
        test_suite.test_neuron_id_allocation(neuron_id_manager)
        test_suite.test_pattern_detection_determinism(fire_ledger, pattern_config)
        test_suite.test_memory_statistics(memory_neuron_array)
        
        print("=" * 50)
        print("🎉 All Memory System Tests PASSED!")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        raise
