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
Test module for Neuron and NeuronArray classes.
"""

import pytest
import numpy as np
import torch
from feagi.bdu.models.neuron import Neuron, NeuronArray

@pytest.fixture
def neuron_array():
    """Create a test NeuronArray with a small capacity for testing."""
    return NeuronArray(max_neurons=100)

@pytest.fixture
def populated_neuron_array():
    """Create a test NeuronArray with some neurons already created."""
    na = NeuronArray(max_neurons=100)
    
    # Create test neurons
    na.create_neuron(cortical_idx=1, position=(0, 0, 0), threshold=1.0)
    na.create_neuron(cortical_idx=1, position=(1, 0, 0), threshold=0.8)
    na.create_neuron(cortical_idx=2, position=(0, 0, 0), threshold=1.2)
    
    return na

def test_neuron_array_init():
    """Test NeuronArray initialization."""
    na = NeuronArray(max_neurons=1000)
    
    # Check array sizes
    assert len(na.membrane_potentials) == 1000
    assert len(na.thresholds) == 1000
    assert len(na.valid_mask) == 1000
    
    # Check default values - convert PyTorch tensors to numpy if needed
    if isinstance(na.membrane_potentials, torch.Tensor):
        assert torch.all(na.membrane_potentials == 0.0).item()
        assert torch.all(na.thresholds == 1.0).item()
        assert torch.all(na.valid_mask == False).item()
    else:
        assert np.all(na.membrane_potentials == 0.0)
        assert np.all(na.thresholds == 1.0)
        assert np.all(na.valid_mask == False)

def test_create_neuron(neuron_array):
    """Test neuron creation in NeuronArray."""
    na = neuron_array
    
    # Create a neuron
    neuron_id = na.create_neuron(position=(1, 2, 3))
    assert neuron_id == 0
    
    # Check coordinate properties - using coordinates_x/y/z for consistency
    assert na.coordinates_x[0] == 1
    assert na.coordinates_y[0] == 2
    assert na.coordinates_z[0] == 3

def test_get_neuron_property(populated_neuron_array):
    """Test getting neuron properties."""
    na = populated_neuron_array
    
    # Check properties for a specific neuron
    assert na.get_neuron_property(0, "cortical_idx") == 1
    assert na.get_neuron_property(0, "position") == (0, 0, 0)
    assert na.get_neuron_property(0, "threshold") == 1.0
    
    # Check for neuron with different properties
    assert np.isclose(na.get_neuron_property(1, "threshold"), 0.8)
    
    # Check error for non-existent neuron
    with pytest.raises(KeyError):
        na.get_neuron_property(999, "threshold")
        
    # Check error for non-existent property
    with pytest.raises(KeyError):
        na.get_neuron_property(0, "nonexistent_property")

def test_set_neuron_property(populated_neuron_array):
    """Test setting neuron properties."""
    na = populated_neuron_array
    
    # Set properties for a neuron
    na.set_neuron_property(0, "threshold", 0.5)
    na.set_neuron_property(0, "position", (5, 6, 7))
    
    # Check if properties were updated
    assert na.get_neuron_property(0, "threshold") == 0.5
    assert na.get_neuron_property(0, "position") == (5, 6, 7)
    
    # Check error for non-existent neuron
    with pytest.raises(KeyError):
        na.set_neuron_property(999, "threshold", 0.5)
        
    # Check error for non-existent property
    with pytest.raises(KeyError):
        na.set_neuron_property(0, "nonexistent_property", 0.5)
        
    # Check error for invalid position
    with pytest.raises(ValueError):
        na.set_neuron_property(0, "position", 0.5)  # Not a tuple

def test_delete_neuron(populated_neuron_array):
    """Test neuron deletion."""
    na = populated_neuron_array
    
    # Check initial count
    if isinstance(na.valid_mask, torch.Tensor):
        assert torch.sum(na.valid_mask).item() == 3
    else:
        assert np.sum(na.valid_mask) == 3
    
    # Delete a neuron
    result = na.delete_neuron(0)
    
    # Check result
    assert result == True
    
    # Check if neuron was deleted
    if isinstance(na.valid_mask, torch.Tensor):
        assert torch.sum(na.valid_mask).item() == 2
    else:
        assert np.sum(na.valid_mask) == 2
    
    # Check if neuron properties are reset
    with pytest.raises(KeyError):
        na.get_neuron_property(0, "threshold")
    
    # Try to delete the same neuron again
    result = na.delete_neuron(0)
    assert result == False  # Should return False for non-existent neuron

def test_get_neurons_by_cortical_area(populated_neuron_array):
    """Test getting neurons by cortical area."""
    na = populated_neuron_array
    
    # Get neurons in cortical area 1
    area1_neurons = na.get_neurons_by_cortical_area(1)
    assert len(area1_neurons) == 2
    assert 0 in area1_neurons
    assert 1 in area1_neurons
    
    # Get neurons in cortical area 2
    area2_neurons = na.get_neurons_by_cortical_area(2)
    assert len(area2_neurons) == 1
    assert 2 in area2_neurons
    
    # Get neurons in non-existent cortical area
    area3_neurons = na.get_neurons_by_cortical_area(3)
    assert len(area3_neurons) == 0

def test_get_neuron_count(populated_neuron_array):
    """Test getting neuron count."""
    na = populated_neuron_array
    
    # Check initial count
    assert na.get_neuron_count() == 3
    
    # Delete a neuron
    na.delete_neuron(0)
    
    # Check updated count
    assert na.get_neuron_count() == 2

def test_neuron_wrapper_class():
    """Test the Neuron wrapper class for API compatibility."""
    # Create a Neuron object
    neuron = Neuron(
        neuron_id=42,
        cortical_id="test_area",
        position=(1, 2, 3),
        threshold=0.5,
        membrane_potential=0.2,
        decay_rate=0.4
    )
    
    # Check properties
    assert neuron.id == 42
    assert neuron.cortical_id == "test_area"
    assert neuron.position == (1, 2, 3)
    assert neuron.threshold == 0.5
    assert neuron.membrane_potential == 0.2
    assert neuron.decay_rate == 0.4
    
    # Test to_dict method
    neuron_dict = neuron.to_dict()
    assert neuron_dict["id"] == 42
    assert neuron_dict["cortical_id"] == "test_area"
    assert neuron_dict["position"] == (1, 2, 3)
    
    # Test from_dict method
    recreated_neuron = Neuron.from_dict(neuron_dict)
    assert recreated_neuron.id == 42
    assert recreated_neuron.cortical_id == "test_area"
    assert recreated_neuron.position == (1, 2, 3)

def test_gpu_transfer():
    """Test GPU transfer methods (conditionally skips if no GPU available)."""
    na = NeuronArray(max_neurons=10)
    
    # Create some test neurons
    for i in range(3):
        na.create_neuron(cortical_idx=1, position=(i, 0, 0))
    
    # Skip test if torch.cuda is not available
    try:
        if not torch.cuda.is_available():
            pytest.skip("CUDA not available, skipping GPU transfer test")
    except ImportError:
        pytest.skip("PyTorch not available, skipping GPU transfer test")
    
    # Try transferring to GPU
    result = na.to_gpu()
    
    # If transfer was successful
    if result:
        # Check if device is set to cuda
        assert na.device == "cuda"
        
        # Check tensor types
        assert isinstance(na.membrane_potentials, torch.Tensor)
        
        # Transfer back to CPU
        na.to_cpu()
        assert na.device == "cpu"
        assert isinstance(na.membrane_potentials, np.ndarray) 