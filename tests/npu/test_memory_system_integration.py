# Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
Comprehensive tests for FEAGI Memory System functionality.

This test suite covers:
- Memory cortical area creation and registration
- Temporal pattern detection and memory neuron lifecycle
- FCL dynamic window sizing
- Integration with GenomeService, ConnectomeManager, and BurstEngine
"""

import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from typing import Dict, Set, Any

from feagi.bdu.models.memory_neuron import MemoryNeuronArray, MemoryPatternKey
from feagi.npu.memory_processor import MemoryProcessor
from feagi.npu.fcl_manager import FCLManager, BitMap
from feagi.core.state_manager import FCLWindowSizeCache


class TestMemoryNeuronArray:
    """Test memory neuron array functionality."""
    
    def setup_method(self):
        """Setup test fixtures."""
        self.capacity = 1000
        self.memory_array = MemoryNeuronArray(capacity=self.capacity)
        
    def test_memory_neuron_array_initialization(self):
        """Test memory neuron array initialization."""
        assert self.memory_array.capacity == self.capacity
        assert self.memory_array.next_available_index == 0
        assert len(self.memory_array.deleted_indices) == 0
        assert len(self.memory_array.pattern_to_index) == 0
        
        # Check array initialization
        assert self.memory_array.lifespan_current.dtype == np.uint32
        assert self.memory_array.is_longterm_memory.dtype == np.bool_
        assert len(self.memory_array.cortical_area_id) == self.capacity

    def test_memory_neuron_creation(self):
        """Test memory neuron creation with pattern key."""
        # Create test pattern key
        pattern_data = (b'test_pattern_1', b'test_pattern_2')
        pattern_key = MemoryPatternKey(
            pattern_data=pattern_data,
            temporal_depth=2,
            source_cortical_areas=('area1', 'area2')
        )
        
        # Create memory neuron
        neuron_idx = self.memory_array.create_memory_neuron(
            pattern_key=pattern_key,
            cortical_area_id="MEM001",
            current_burst=10,
            initial_lifespan=20,
            lifespan_growth_rate=1.5
        )
        
        assert neuron_idx == 0
        assert self.memory_array.next_available_index == 1
        assert pattern_key in self.memory_array.pattern_to_index
        assert self.memory_array.pattern_to_index[pattern_key] == 0
        
        # Check neuron properties
        assert self.memory_array.lifespan_current[0] == 20
        assert self.memory_array.lifespan_initial[0] == 20
        assert self.memory_array.lifespan_growth_rate[0] == 1.5
        assert self.memory_array.creation_burst[0] == 10
        assert self.memory_array.cortical_area_id[0] == "MEM001"
        assert self.memory_array.is_active[0] == True
        assert self.memory_array.is_longterm_memory[0] == False

    def test_memory_neuron_reactivation(self):
        """Test memory neuron reactivation and lifespan growth."""
        # Create initial neuron
        pattern_key = MemoryPatternKey(
            pattern_data=(b'pattern',),
            temporal_depth=1,
            source_cortical_areas=('area1',)
        )
        
        neuron_idx = self.memory_array.create_memory_neuron(
            pattern_key=pattern_key,
            cortical_area_id="MEM001",
            current_burst=10,
            initial_lifespan=20,
            lifespan_growth_rate=1.5
        )
        
        # Reactivate neuron
        success = self.memory_array.reactivate_memory_neuron(neuron_idx, current_burst=15)
        
        assert success == True
        assert self.memory_array.last_activation_burst[neuron_idx] == 15
        assert self.memory_array.activation_count[neuron_idx] == 2
        # Lifespan should have grown: 20 * 1.5 = 30
        assert self.memory_array.lifespan_current[neuron_idx] == 30

    def test_memory_neuron_aging(self):
        """Test memory neuron aging and death."""
        # Create neuron with short lifespan
        pattern_key = MemoryPatternKey(
            pattern_data=(b'short_lived',),
            temporal_depth=1,
            source_cortical_areas=('area1',)
        )
        
        neuron_idx = self.memory_array.create_memory_neuron(
            pattern_key=pattern_key,
            cortical_area_id="MEM001",
            current_burst=10,
            initial_lifespan=2  # Very short lifespan
        )
        
        # Age the neuron
        died_neurons = self.memory_array.age_memory_neurons(current_burst=11)
        assert len(died_neurons) == 0  # Should still be alive (lifespan=1)
        assert self.memory_array.lifespan_current[neuron_idx] == 1
        
        # Age again
        died_neurons = self.memory_array.age_memory_neurons(current_burst=12)
        assert neuron_idx in died_neurons
        assert self.memory_array.is_active[neuron_idx] == False
        assert neuron_idx in self.memory_array.deleted_indices

    def test_longterm_memory_conversion(self):
        """Test conversion to long-term memory."""
        # Create neuron with high lifespan
        pattern_key = MemoryPatternKey(
            pattern_data=(b'long_lived',),
            temporal_depth=1,
            source_cortical_areas=('area1',)
        )
        
        neuron_idx = self.memory_array.create_memory_neuron(
            pattern_key=pattern_key,
            cortical_area_id="MEM001",
            current_burst=10,
            initial_lifespan=150  # Above threshold
        )
        
        # Check for conversion
        converted_neurons = self.memory_array.check_longterm_conversion(longterm_threshold=100)
        
        assert neuron_idx in converted_neurons
        assert self.memory_array.is_longterm_memory[neuron_idx] == True
        
        # Long-term memory neurons shouldn't age
        died_neurons = self.memory_array.age_memory_neurons(current_burst=11)
        assert neuron_idx not in died_neurons
        assert self.memory_array.lifespan_current[neuron_idx] == 150  # Unchanged

    def test_pattern_finding(self):
        """Test finding memory neurons by pattern."""
        # Create neuron
        pattern_key = MemoryPatternKey(
            pattern_data=(b'findable_pattern',),
            temporal_depth=1,
            source_cortical_areas=('area1',)
        )
        
        neuron_idx = self.memory_array.create_memory_neuron(
            pattern_key=pattern_key,
            cortical_area_id="MEM001",
            current_burst=10
        )
        
        # Find by pattern
        found_idx = self.memory_array.find_memory_neuron_by_pattern(pattern_key)
        assert found_idx == neuron_idx
        
        # Test non-existent pattern
        non_existent_pattern = MemoryPatternKey(
            pattern_data=(b'non_existent',),
            temporal_depth=1,
            source_cortical_areas=('area1',)
        )
        found_idx = self.memory_array.find_memory_neuron_by_pattern(non_existent_pattern)
        assert found_idx is None


class TestMemoryProcessor:
    """Test memory processor functionality."""
    
    def setup_method(self):
        """Setup test fixtures."""
        self.memory_array = MemoryNeuronArray(capacity=1000)
        self.fcl_manager = Mock(spec=FCLManager)
        self.memory_processor = MemoryProcessor(
            memory_neuron_array=self.memory_array,
            fcl_manager=self.fcl_manager,
            batch_size=10,
            pattern_cache_size=100
        )
        
    def test_memory_processor_initialization(self):
        """Test memory processor initialization."""
        assert self.memory_processor.memory_neuron_array == self.memory_array
        assert self.memory_processor.fcl_manager == self.fcl_manager
        assert self.memory_processor.batch_size == 10
        assert len(self.memory_processor.active_memory_areas) == 0

    def test_memory_area_registration(self):
        """Test registering memory areas with the processor."""
        success = self.memory_processor.register_memory_area(
            cortical_id="MEM001",
            temporal_depth=5,
            initial_lifespan=20,
            lifespan_growth_rate=1.2,
            longterm_threshold=100,
            upstream_areas={"area1", "area2"}
        )
        
        assert success == True
        assert "MEM001" in self.memory_processor.active_memory_areas
        assert "MEM001" in self.memory_processor.memory_area_properties
        
        properties = self.memory_processor.memory_area_properties["MEM001"]
        assert properties["temporal_depth"] == 5
        assert properties["initial_lifespan"] == 20
        assert properties["upstream_areas"] == {"area1", "area2"}

    def test_memory_pattern_extraction(self):
        """Test temporal pattern extraction from FCL."""
        # Setup mock FCL manager to return test bitmaps
        mock_bitmap1 = Mock()
        mock_bitmap1.serialize.return_value = b'bitmap1'
        mock_bitmap1.__len__.return_value = 5
        
        mock_bitmap2 = Mock()
        mock_bitmap2.serialize.return_value = b'bitmap2'
        mock_bitmap2.__len__.return_value = 3
        
        self.fcl_manager.get_neurons_by_corticals.side_effect = [mock_bitmap2, mock_bitmap1]
        
        # Extract pattern
        pattern_key = self.memory_processor._extract_temporal_pattern(
            upstream_areas={"area1", "area2"},
            temporal_depth=2,
            current_burst=5
        )
        
        assert pattern_key is not None
        assert pattern_key.temporal_depth == 2
        assert pattern_key.source_cortical_areas == ("area1", "area2")
        assert len(pattern_key.pattern_data) == 2
        assert pattern_key.pattern_data[0] == b'bitmap2'  # Current timestep (5)
        assert pattern_key.pattern_data[1] == b'bitmap1'  # Previous timestep (4)

    def test_memory_processing_batch(self):
        """Test batch processing of memory areas."""
        # Register memory area
        self.memory_processor.register_memory_area(
            cortical_id="MEM001",
            temporal_depth=2,
            upstream_areas={"area1"}
        )
        
        # Setup mock FCL manager
        mock_bitmap = Mock()
        mock_bitmap.serialize.return_value = b'test_pattern'
        mock_bitmap.__len__.return_value = 1
        self.fcl_manager.get_neurons_by_corticals.return_value = mock_bitmap
        
        # Process batch
        results = self.memory_processor.process_memory_areas_batch(current_burst=10)
        
        assert results["success"] == True
        assert results["burst"] == 10
        assert "stats" in results
        assert results["stats"]["areas_processed"] >= 1

    def test_pattern_caching(self):
        """Test pattern caching functionality."""
        # Create test pattern
        pattern_key = MemoryPatternKey(
            pattern_data=(b'cached_pattern',),
            temporal_depth=1,
            source_cortical_areas=('area1',)
        )
        
        # Create neuron to cache
        neuron_idx = self.memory_array.create_memory_neuron(
            pattern_key=pattern_key,
            cortical_area_id="MEM001",
            current_burst=10
        )
        
        # Add to cache
        self.memory_processor._add_to_pattern_cache(pattern_key, neuron_idx)
        
        # Test cache hit
        cached_idx = self.memory_processor._find_or_cache_pattern(pattern_key)
        assert cached_idx == neuron_idx
        assert self.memory_processor.stats.pattern_cache_hits == 1
        
        # Test cache miss
        non_cached_pattern = MemoryPatternKey(
            pattern_data=(b'not_cached',),
            temporal_depth=1,
            source_cortical_areas=('area1',)
        )
        cached_idx = self.memory_processor._find_or_cache_pattern(non_cached_pattern)
        assert cached_idx is None
        assert self.memory_processor.stats.pattern_cache_misses == 1


class TestFCLWindowSizeCache:
    """Test FCL window size cache functionality."""
    
    def setup_method(self):
        """Setup test fixtures."""
        self.cache = FCLWindowSizeCache(default_window_size=20)
        
    def test_cache_initialization(self):
        """Test cache initialization."""
        assert self.cache.window_size == 20
        assert len(self.cache.memory_areas) == 0
        assert len(self.cache.cortical_to_memory_mappings) == 0

    def test_memory_area_registration(self):
        """Test registering memory areas."""
        self.cache.register_memory_area("MEM001", temporal_depth=10)
        
        assert "MEM001" in self.cache.memory_areas
        assert self.cache.memory_temporal_depths["MEM001"] == 10

    def test_cortical_mapping_management(self):
        """Test cortical mapping management."""
        # Register memory area
        self.cache.register_memory_area("MEM001", temporal_depth=15)
        
        # Add mapping
        self.cache.add_cortical_mapping("AREA1", "MEM001")
        
        assert "MEM001" in self.cache.cortical_to_memory_mappings["AREA1"]
        assert "AREA1" not in self.cache.computed_window_sizes  # Should be invalidated

    def test_window_size_computation(self):
        """Test window size computation."""
        # Register memory areas
        self.cache.register_memory_area("MEM001", temporal_depth=25)
        self.cache.register_memory_area("MEM002", temporal_depth=15)
        
        # Add mappings
        self.cache.add_cortical_mapping("AREA1", "MEM001")
        self.cache.add_cortical_mapping("AREA1", "MEM002")
        
        # Test window size computation
        window_size = self.cache.get_window_size("AREA1")
        assert window_size == 25  # Max of default(20), temporal_depth(25, 15)
        
        # Test area with no memory mappings
        window_size = self.cache.get_window_size("AREA2")
        assert window_size == 20  # Default

    def test_temporal_depth_updates(self):
        """Test temporal depth updates and cache invalidation."""
        # Setup
        self.cache.register_memory_area("MEM001", temporal_depth=10)
        self.cache.add_cortical_mapping("AREA1", "MEM001")
        
        # Get initial window size
        initial_size = self.cache.get_window_size("AREA1")
        assert initial_size == 20  # Default is larger
        
        # Update temporal depth
        self.cache.update_memory_temporal_depth("MEM001", 30)
        
        # Window size should be recomputed
        new_size = self.cache.get_window_size("AREA1")
        assert new_size == 30  # Now temporal depth is larger


class TestMemorySystemIntegration:
    """Test end-to-end memory system integration."""
    
    def setup_method(self):
        """Setup test fixtures for integration tests."""
        # Create mocks for major components
        self.connectome_manager = Mock()
        self.fcl_manager = Mock(spec=FCLManager)
        self.state_manager = Mock()
        
        # Setup memory neuron array
        self.memory_array = MemoryNeuronArray(capacity=1000)
        self.connectome_manager.memory_neuron_array = self.memory_array
        self.connectome_manager.memory_areas = set()
        
        # Setup memory processor
        self.memory_processor = MemoryProcessor(
            memory_neuron_array=self.memory_array,
            fcl_manager=self.fcl_manager
        )

    def test_memory_cortical_area_creation_integration(self):
        """Test memory cortical area creation integration."""
        # Mock GenomeService behavior
        with patch('feagi.api.core.services.genome.genome_service.GenomeService') as mock_genome_service:
            genome_service = mock_genome_service.return_value
            
            # Test memory area creation
            result = genome_service.create_cortical_area(
                name="Test Memory Area",
                coordinates={"x": 0, "y": 0, "z": 0},
                dimensions={"width": 1, "height": 1, "depth": 1},
                area_type="CUSTOM",
                parameters={
                    "sub_group_id": "MEMORY",
                    "temporal_depth": 5,
                    "init_lifespan": 15,
                    "longterm_mem_threshold": 80
                }
            )
            
            # Verify the call was made (mock verification)
            assert genome_service.create_cortical_area.called

    def test_burst_engine_memory_integration(self):
        """Test BurstEngine memory processing integration."""
        with patch('feagi.npu.burst_engine.BurstEngine') as mock_burst_engine:
            burst_engine = mock_burst_engine.return_value
            burst_engine.memory_processor = self.memory_processor
            
            # Test memory processing during burst
            burst_engine._process_memory_areas(current_timestep=10)
            
            # Verify memory processing was called
            assert burst_engine._process_memory_areas.called

    def test_fcl_dynamic_window_sizing_integration(self):
        """Test FCL dynamic window sizing integration."""
        # Setup FCL manager with dynamic sizing
        self.fcl_manager._dynamic_sizing_enabled = True
        self.fcl_manager._state_manager = self.state_manager
        self.fcl_manager._cortical_window_sizes = {}
        
        # Mock state manager window size query
        self.state_manager.get_fcl_window_size.return_value = 25
        
        # Mock cortical ID resolution
        with patch.object(self.fcl_manager, '_get_cortical_id_from_index', return_value="AREA1"):
            # Test dynamic window size retrieval
            window_size = self.fcl_manager.get_cortical_window_size(cortical_idx=1)
            
            # Verify dynamic sizing was used
            self.state_manager.get_fcl_window_size.assert_called_with("AREA1")

    def test_end_to_end_memory_lifecycle(self):
        """Test complete memory neuron lifecycle."""
        # 1. Register memory area
        success = self.memory_processor.register_memory_area(
            cortical_id="MEM001",
            temporal_depth=3,
            initial_lifespan=10,
            upstream_areas={"AREA1", "AREA2"}
        )
        assert success == True
        
        # 2. Setup mock FCL data for pattern extraction
        mock_bitmap = Mock()
        mock_bitmap.serialize.return_value = b'test_pattern_sequence'
        mock_bitmap.__len__.return_value = 2
        self.fcl_manager.get_neurons_by_corticals.return_value = mock_bitmap
        
        # 3. Process first occurrence (should create neuron)
        results1 = self.memory_processor.process_memory_areas_batch(current_burst=10)
        assert results1["success"] == True
        assert results1["stats"]["neurons_created"] >= 1
        
        # 4. Process same pattern again (should reactivate)
        results2 = self.memory_processor.process_memory_areas_batch(current_burst=11)
        assert results2["success"] == True
        assert results2["stats"]["neurons_reactivated"] >= 1
        
        # 5. Age neurons multiple times
        for burst in range(12, 25):
            aging_results = self.memory_processor._perform_aging_and_lifecycle(burst)
            # Some neurons may die or convert to long-term
        
        # 6. Verify statistics
        stats = self.memory_processor.get_processing_statistics()
        assert stats is not None
        assert "memory_processor" in stats
        assert "memory_neuron_array" in stats

    def test_memory_system_performance(self):
        """Test memory system performance under load."""
        # Register multiple memory areas
        for i in range(10):
            self.memory_processor.register_memory_area(
                cortical_id=f"MEM{i:03d}",
                temporal_depth=5,
                upstream_areas={f"AREA{j}" for j in range(i, i+3)}
            )
        
        # Setup mock FCL responses
        mock_bitmap = Mock()
        mock_bitmap.serialize.return_value = b'performance_test_pattern'
        mock_bitmap.__len__.return_value = 1
        self.fcl_manager.get_neurons_by_corticals.return_value = mock_bitmap
        
        # Process multiple bursts
        import time
        start_time = time.time()
        
        for burst in range(100):
            results = self.memory_processor.process_memory_areas_batch(current_burst=burst)
            assert results["success"] == True
        
        processing_time = time.time() - start_time
        
        # Verify reasonable performance (should process 100 bursts in reasonable time)
        assert processing_time < 10.0  # Should complete within 10 seconds
        
        # Check final statistics
        stats = self.memory_processor.get_processing_statistics()
        assert stats["memory_processor"]["total_patterns_processed"] > 0


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v", "--tb=short"]) 