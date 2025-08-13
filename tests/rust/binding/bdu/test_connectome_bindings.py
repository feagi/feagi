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
Tests for Python-Rust bindings of the ConnectomeManager.

These tests verify that the Rust implementation of ConnectomeManager
can be correctly accessed from Python through bindings.
"""

import pytest

# Import both Python and Rust implementations
from feagi.bdu.connectome_manager import ConnectomeManager as PyConnectomeManager
from feagi.bdu.connectome_manager import NeuronPropertyType

# This would be the import for the Rust implementation via bindings
# We're using a try/except to gracefully handle if it's not yet available
try:
    from feagi_rust.bdu import ConnectomeManager as RustConnectomeManager

    RUST_AVAILABLE = True
except ImportError:
    RUST_AVAILABLE = False


@pytest.fixture
def minimal_config():
    """Create a minimal config for testing."""
    from feagi.utils.config import FeagiConfig

    config = FeagiConfig()
    config.set("connectome.max_neurons", 100)
    config.set("connectome.max_synapses_per_neuron", 10)
    config.set("connectome.fcl_window_size", 3)
    return config


@pytest.fixture
def py_connectome(minimal_config):
    """Create a Python ConnectomeManager instance."""
    return PyConnectomeManager(minimal_config)


@pytest.fixture
def rust_connectome(minimal_config):
    """Create a Rust ConnectomeManager instance."""
    if not RUST_AVAILABLE:
        pytest.skip("Rust bindings not available")
    return RustConnectomeManager(minimal_config)


@pytest.mark.binding
def test_connectome_creation(rust_connectome):
    """Test creating a ConnectomeManager through bindings."""
    # Simply test that the object was created successfully
    assert rust_connectome is not None
    assert hasattr(rust_connectome, "add_cortical_area")
    assert hasattr(rust_connectome, "create_neuron")


@pytest.mark.binding
def test_area_creation_parity(py_connectome, rust_connectome):
    """Test that area creation works similarly in Python and Rust."""
    # Create identical areas in both implementations
    py_area_id = 1
    py_area = py_connectome.add_cortical_area(
        area_id=py_area_id,
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),
        position=(0, 0, 0),
    )

    rust_area_id = 1
    rust_area = rust_connectome.add_cortical_area(
        area_id=rust_area_id,
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),
        position=(0, 0, 0),
    )

    # Verify both implementations return similar results
    assert py_area.id == rust_area.id
    assert py_area.name == rust_area.name
    assert py_area.dimensions == rust_area.dimensions
    assert py_area.position == rust_area.position


@pytest.mark.binding
def test_neuron_creation_parity(py_connectome, rust_connectome):
    """Test that neuron creation works similarly in Python and Rust."""
    # Create identical areas in both implementations
    py_area_id = 1
    py_connectome.add_cortical_area(
        area_id=py_area_id,
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),
        position=(0, 0, 0),
    )

    rust_area_id = 1
    rust_connectome.add_cortical_area(
        area_id=rust_area_id,
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),
        position=(0, 0, 0),
    )

    # Create neurons in both implementations
    py_neuron_id = py_connectome.create_neuron(
        area_id=py_area_id,
        position=(2, 2, 1),
        threshold=1.0,
        refractory_period=5,
        decay_rate=0.9,
        resting_potential=0.0,
    )

    rust_neuron_id = rust_connectome.create_neuron(
        area_id=rust_area_id,
        position=(2, 2, 1),
        threshold=1.0,
        refractory_period=5,
        decay_rate=0.9,
        resting_potential=0.0,
    )

    # Verify both implementations create neurons properly
    py_pos = py_connectome.get_neuron_position(py_neuron_id)
    rust_pos = rust_connectome.get_neuron_position(rust_neuron_id)
    assert py_pos == rust_pos

    py_threshold = py_connectome.get_neuron_property(
        py_neuron_id, NeuronPropertyType.THRESHOLD
    )
    rust_threshold = rust_connectome.get_neuron_property(
        rust_neuron_id, NeuronPropertyType.THRESHOLD
    )
    assert py_threshold == rust_threshold


@pytest.mark.binding
def test_synapse_creation_parity(py_connectome, rust_connectome):
    """Test that synapse creation works similarly in Python and Rust."""
    # Set up identical areas and neurons in both implementations
    py_area_id = 1
    py_connectome.add_cortical_area(
        area_id=py_area_id,
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),
        position=(0, 0, 0),
    )

    rust_area_id = 1
    rust_connectome.add_cortical_area(
        area_id=rust_area_id,
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),
        position=(0, 0, 0),
    )

    # Create pre and post neurons in both
    py_pre_id = py_connectome.create_neuron(area_id=py_area_id, position=(0, 0, 0))
    py_post_id = py_connectome.create_neuron(area_id=py_area_id, position=(1, 0, 0))

    rust_pre_id = rust_connectome.create_neuron(
        area_id=rust_area_id, position=(0, 0, 0)
    )
    rust_post_id = rust_connectome.create_neuron(
        area_id=rust_area_id, position=(1, 0, 0)
    )

    # Create synapses
    py_result = py_connectome.create_synapse(
        pre_neuron_id=py_pre_id,
        post_neuron_id=py_post_id,
        weight=1.5,
        is_plastic=True,
        plasticity_coeff=0.1,
        plasticity_decay=0.01,
    )

    rust_result = rust_connectome.create_synapse(
        pre_neuron_id=rust_pre_id,
        post_neuron_id=rust_post_id,
        weight=1.5,
        is_plastic=True,
        plasticity_coeff=0.1,
        plasticity_decay=0.01,
    )

    # Verify both implementations return success
    assert py_result == rust_result

    # Check connections in both
    py_outgoing = py_connectome.get_outgoing_connections(py_pre_id)
    rust_outgoing = rust_connectome.get_outgoing_connections(rust_pre_id)

    assert len(py_outgoing) == len(rust_outgoing)
    assert py_outgoing[0][1] == rust_outgoing[0][1]  # Weight comparison


@pytest.mark.binding
def test_membrane_update_parity(py_connectome, rust_connectome):
    """Test that membrane potential updates behave similarly in Python and Rust."""
    # Set up identical networks in both implementations
    py_area_id = 1
    py_connectome.add_cortical_area(
        area_id=py_area_id,
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),
        position=(0, 0, 0),
    )

    rust_area_id = 1
    rust_connectome.add_cortical_area(
        area_id=rust_area_id,
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),
        position=(0, 0, 0),
    )

    # Create pre and post neurons with identical properties
    py_pre_id = py_connectome.create_neuron(
        area_id=py_area_id, position=(0, 0, 0), threshold=1.0
    )
    py_post_id = py_connectome.create_neuron(
        area_id=py_area_id,
        position=(1, 0, 0),
        threshold=0.5,  # Lower threshold to ensure firing
    )

    rust_pre_id = rust_connectome.create_neuron(
        area_id=rust_area_id, position=(0, 0, 0), threshold=1.0
    )
    rust_post_id = rust_connectome.create_neuron(
        area_id=rust_area_id,
        position=(1, 0, 0),
        threshold=0.5,  # Lower threshold to ensure firing
    )

    # Create identical synapses
    py_connectome.create_synapse(
        pre_neuron_id=py_pre_id, post_neuron_id=py_post_id, weight=1.5
    )

    rust_connectome.create_synapse(
        pre_neuron_id=rust_pre_id, post_neuron_id=rust_post_id, weight=1.5
    )

    # Get internal indices
    py_pre_idx = py_connectome._neuron_id_to_index[py_pre_id]
    rust_pre_idx = rust_connectome._neuron_id_to_index[rust_pre_id]

    # Add pre neurons to FCL
    py_connectome.fcl_manager.add_to_current_fcl([py_pre_idx])
    rust_connectome.fcl_manager.add_to_current_fcl([rust_pre_idx])

    # Update membrane potentials
    py_firing = py_connectome.update_membrane_potentials(current_timestep=1)
    rust_firing = rust_connectome.update_membrane_potentials(current_timestep=1)

    # Verify both implementations produce similar firing patterns
    assert py_post_id in py_firing
    assert rust_post_id in rust_firing
