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

"""
Performance tests for ConnectomeManager.

These tests verify that ConnectomeManager can handle extreme cases:
- Large numbers of neurons
- Areas with extreme dimensions
- Multiple neurons per voxel
- Large numbers of synapses
- Serialization/deserialization of large brains
"""

import pytest
import numpy as np
import os
import tempfile
from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType, CorticalArea


@pytest.fixture
def connectome_large(large_config):
    """Create a ConnectomeManager with large capacity."""
    return ConnectomeManager(large_config)


@pytest.fixture
def connectome_medium(medium_config):
    """Create a ConnectomeManager with medium capacity."""
    return ConnectomeManager(medium_config)


@pytest.fixture
def extreme_area(connectome_medium):
    """Create a cortical area with extreme dimensions."""
    area_id = 1
    area = connectome_medium.add_cortical_area(
        area_id=area_id,
        name="Extreme Area",
        area_type="interconnect",
        dimensions=(20000, 1, 1),  # One extremely long dimension
        position=(0, 0, 0)
    )
    return area_id, area


@pytest.fixture
def dense_area(connectome_medium):
    """Create a small area that will contain multiple neurons per voxel."""
    area_id = 2
    area = connectome_medium.add_cortical_area(
        area_id=area_id,
        name="Dense Area",
        area_type="interconnect",
        dimensions=(3, 3, 3),  # Small dimensions
        position=(0, 0, 0)
    )
    return area_id, area


@pytest.mark.performance
def test_extreme_dimension_area(connectome_medium, extreme_area):
    """Test working with areas that have extreme dimensions."""
    area_id = extreme_area[0]
    
    # Create neurons at different points along the extreme dimension
    neuron_ids = []
    positions = [0, 100, 1000, 10000, 19999]  # Various positions including the extreme end
    
    for pos in positions:
        neuron_id = connectome_medium.create_neuron(
            area_id=area_id,
            position=(pos, 0, 0)
        )
        neuron_ids.append(neuron_id)
    
    # Verify neurons were created
    assert len(neuron_ids) == len(positions)
    
    # Verify positions
    for i, neuron_id in enumerate(neuron_ids):
        position = connectome_medium.get_neuron_position(neuron_id)
        assert position[0] == positions[i]
    
    # Test retrieving neurons from a range
    middle_neurons = connectome_medium.query_neurons_by_area_and_position(
        area_id,
        x_range=(500, 5000),
    )
    assert len(middle_neurons) == 1  # Should find the neuron at position 1000
    assert neuron_ids[2] in middle_neurons


@pytest.mark.performance
def test_multiple_neurons_per_voxel(connectome_medium, dense_area):
    """Test creating multiple neurons in the same voxel position."""
    area_id = dense_area[0]
    
    # Create multiple neurons at the same position
    num_neurons_per_pos = 10
    positions = [(0, 0, 0), (1, 1, 1), (2, 2, 2)]
    
    neuron_ids = []
    for pos in positions:
        for i in range(num_neurons_per_pos):
            neuron_id = connectome_medium.create_neuron(
                area_id=area_id,
                position=pos
            )
            neuron_ids.append((neuron_id, pos, i))
    
    # Verify total count
    total_neurons = len(positions) * num_neurons_per_pos
    assert len(neuron_ids) == total_neurons
    
    # Verify we can get all neurons at a position
    for pos in positions:
        neurons_at_pos = connectome_medium.get_neurons_at_position(area_id, pos)
        assert len(neurons_at_pos) == num_neurons_per_pos
        
        # Verify each index is represented
        for i in range(num_neurons_per_pos):
            neuron = connectome_medium.get_neuron_at_position(area_id, pos, i)
            assert neuron is not None


@pytest.mark.performance
@pytest.mark.slow
def test_many_neurons(connectome_large):
    """Test creating a large number of neurons."""
    # Create a large cortical area
    area_id = 1
    area = connectome_large.add_cortical_area(
        area_id=area_id,
        name="Large Area",
        area_type="interconnect",
        dimensions=(100, 100, 10),  # 100,000 voxels
        position=(0, 0, 0)
    )
    
    # Create a large number of neurons
    num_neurons = 10000  # 10K neurons (adjust based on available memory/time)
    
    neuron_ids = []
    for i in range(num_neurons):
        # Distribute evenly through the volume
        x = i % 100
        y = (i // 100) % 100
        z = (i // 10000) % 10
        
        neuron_id = connectome_large.create_neuron(
            area_id=area_id,
            position=(x, y, z)
        )
        neuron_ids.append(neuron_id)
    
    # Verify neuron count
    assert connectome_large.get_neuron_count() == num_neurons
    assert len(connectome_large.get_neurons_by_area(area_id)) == num_neurons


@pytest.mark.performance
@pytest.mark.slow
def test_many_synapses(connectome_medium):
    """Test creating a large number of synapses."""
    # Create a medium-sized cortical area
    area_id = 1
    area = connectome_medium.add_cortical_area(
        area_id=area_id,
        name="Medium Area",
        area_type="interconnect",
        dimensions=(20, 20, 5),  # 2,000 voxels
        position=(0, 0, 0)
    )
    
    # Create 100 source neurons and 100 target neurons
    num_sources = 100
    num_targets = 100
    
    source_ids = []
    for i in range(num_sources):
        neuron_id = connectome_medium.create_neuron(
            area_id=area_id,
            position=(i % 20, i // 20, 0)
        )
        source_ids.append(neuron_id)
    
    target_ids = []
    for i in range(num_targets):
        neuron_id = connectome_medium.create_neuron(
            area_id=area_id,
            position=(i % 20, i // 20, 1)
        )
        target_ids.append(neuron_id)
    
    # Create a dense synaptic connection pattern (each source to all targets)
    synapse_count = 0
    for src_id in source_ids:
        for tgt_id in target_ids:
            result = connectome_medium.create_synapse(
                pre_neuron_id=src_id,
                post_neuron_id=tgt_id,
                weight=0.5
            )
            if result:
                synapse_count += 1
    
    # Verify all synapses were created
    expected_synapses = num_sources * num_targets
    assert synapse_count == expected_synapses
    
    # Verify outgoing connections
    for src_id in source_ids:
        outgoing = connectome_medium.get_outgoing_connections(src_id)
        assert len(outgoing) == num_targets
    
    # Verify incoming connections
    for tgt_id in target_ids:
        incoming = connectome_medium.get_incoming_connections(tgt_id)
        assert len(incoming) == num_sources


@pytest.mark.performance
def test_serialization(connectome_medium, dense_area, tmp_path):
    """Test serialization of a brain with significant data."""
    area_id = dense_area[0]
    
    # Create some neurons and synapses
    num_neurons = 1000
    neuron_ids = []
    
    # Create neurons
    for i in range(num_neurons):
        x = i % 3
        y = (i // 3) % 3
        z = (i // 9) % 3
        neuron_id = connectome_medium.create_neuron(
            area_id=area_id,
            position=(x, y, z)
        )
        neuron_ids.append(neuron_id)
    
    # Create some synapses (connect every 10th neuron to the next 5)
    for i in range(0, num_neurons, 10):
        for j in range(1, 6):
            if i + j < num_neurons:
                connectome_medium.create_synapse(
                    pre_neuron_id=neuron_ids[i],
                    post_neuron_id=neuron_ids[i + j],
                    weight=0.5,
                    is_plastic=True
                )
    
    # Save to a temporary file
    filename = str(tmp_path / "brain.npz")
    
    # Serialize
    connectome_medium.serialize_brain_state(filename)
    
    # Verify the file exists and has content
    assert os.path.exists(filename)
    assert os.path.getsize(filename) > 0
    
    # Note: Deserialization would need more complex handling to test properly
    # We would need to:
    # 1. Create a new connectome with no pre-existing areas
    # 2. Add the same area structure before deserializing
    # 3. Handle the case where neurons already exist
    #
    # This is beyond the scope of a simple performance test 