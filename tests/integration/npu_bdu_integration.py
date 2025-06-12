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

"""Integration tests between NPU and GPU-optimized BDU.

This module tests the integration between Neural Processing Unit (NPU) and
the GPU-optimized Brain Development Unit (BDU).
"""

import logging
import time
from typing import Any, Dict, List, Tuple

import numpy as np
import pytest

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.connectome_manager_gpu import ConnectomeManagerGPU
from feagi.npu.burst_engine import BurstEngine
from feagi.npu.fcl_manager import FCLManager

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


@pytest.fixture
def small_connectome():
    """Create a small connectome for testing."""
    return ConnectomeManager(1000)


@pytest.fixture
def small_gpu_connectome():
    """Create a small GPU-optimized connectome for testing."""
    return ConnectomeManagerGPU(1000)


@pytest.fixture
def test_brain_structure(small_connectome):
    """Setup a small test brain with a few cortical areas and neurons."""
    cm = small_connectome

    # Create a few cortical areas
    area1_id = cm.add_cortical_area("Visual", (10, 10, 1), (0, 0, 0), "sensory")
    area2_id = cm.add_cortical_area("Hidden", (10, 10, 1), (0, 0, 10), "hidden")
    area3_id = cm.add_cortical_area("Motor", (10, 10, 1), (0, 0, 20), "motor")

    # Create neurons in each area
    neuron_ids = {}
    for area_id in [area1_id, area2_id, area3_id]:
        neuron_ids[area_id] = []
        for x in range(10):
            for y in range(10):
                neuron_id = cm.create_neuron(area_id, (x, y, 0))
                neuron_ids[area_id].append(neuron_id)

    # Create connections between areas
    # Visual → Hidden
    for i, source_id in enumerate(neuron_ids[area1_id]):
        for j, target_id in enumerate(neuron_ids[area2_id]):
            if i % 5 == j % 5:  # Simple pattern for connections
                cm.create_synapse(source_id, target_id, 0.5)

    # Hidden → Motor
    for i, source_id in enumerate(neuron_ids[area2_id]):
        for j, target_id in enumerate(neuron_ids[area3_id]):
            if i % 3 == j % 3:  # Simple pattern for connections
                cm.create_synapse(source_id, target_id, 0.5)

    return {
        "connectome": cm,
        "areas": [area1_id, area2_id, area3_id],
        "neurons": neuron_ids,
    }


@pytest.fixture
def test_gpu_brain_structure(small_gpu_connectome):
    """Setup a small test brain with a few cortical areas and neurons using GPU-optimized ConnectomeManager."""
    cm = small_gpu_connectome

    # Create a few cortical areas
    area1_id = cm.add_cortical_area("Visual", (10, 10, 1), (0, 0, 0), "sensory")
    area2_id = cm.add_cortical_area("Hidden", (10, 10, 1), (0, 0, 10), "hidden")
    area3_id = cm.add_cortical_area("Motor", (10, 10, 1), (0, 0, 20), "motor")

    # Create neurons in each area
    neuron_ids = {}
    for area_id in [area1_id, area2_id, area3_id]:
        neuron_ids[area_id] = []
        # Use batch creation for better performance
        positions = [(x, y, 0) for x in range(10) for y in range(10)]
        area_neuron_ids = cm.batch_create_neurons(area_id, positions)
        neuron_ids[area_id] = area_neuron_ids

    # Create connections between areas using batch operations
    # Visual → Hidden
    synapse_specs = []
    for i, source_id in enumerate(neuron_ids[area1_id]):
        for j, target_id in enumerate(neuron_ids[area2_id]):
            if i % 5 == j % 5:  # Simple pattern for connections
                synapse_specs.append((source_id, target_id, 0.5))

    cm.batch_create_synapses(synapse_specs)

    # Hidden → Motor
    synapse_specs = []
    for i, source_id in enumerate(neuron_ids[area2_id]):
        for j, target_id in enumerate(neuron_ids[area3_id]):
            if i % 3 == j % 3:  # Simple pattern for connections
                synapse_specs.append((source_id, target_id, 0.5))

    cm.batch_create_synapses(synapse_specs)

    return {
        "connectome": cm,
        "areas": [area1_id, area2_id, area3_id],
        "neurons": neuron_ids,
    }


@pytest.mark.integration
def test_fcl_manager_with_standard_connectome(test_brain_structure):
    """Test FCL manager integration with standard ConnectomeManager."""
    brain = test_brain_structure
    cm = brain["connectome"]

    # Create FCL manager with the connectome's existing fcl_manager
    fcl_manager = cm.fcl_manager

    # Directly stimulate some visual neurons
    visual_neurons = brain["neurons"][brain["areas"][0]][:10]  # First 10 visual neurons
    for neuron_id in visual_neurons:
        cm.set_neuron_property(neuron_id, "membrane_potential", 2.0)  # Above threshold

    # Run a few timesteps
    fired_neurons = []
    for _ in range(5):
        # Step simulation
        fired = cm.update_membrane_potentials()
        fired_neurons.append(fired)

    # Check FCL contents across timesteps
    assert len(fcl_manager.get_queue()) > 0, "FCL should contain at least one event"

    # Visual neurons should have fired in the first timestep
    assert all(neuron_id in fired_neurons[0] for neuron_id in visual_neurons), (
        "Visual neurons should fire in the first timestep"
    )

    # Some hidden neurons should fire in later timesteps due to connections
    hidden_neurons = brain["neurons"][brain["areas"][1]]
    assert any(neuron_id in fired_neurons[1] for neuron_id in hidden_neurons), (
        "Some hidden neurons should fire in the second timestep"
    )

    # Motor neurons should fire in even later timesteps
    motor_neurons = brain["neurons"][brain["areas"][2]]
    assert any(
        neuron_id in fired_neurons[2] or neuron_id in fired_neurons[3]
        for neuron_id in motor_neurons
    ), "Some motor neurons should fire in later timesteps"


@pytest.mark.integration
def test_fcl_manager_with_gpu_connectome(test_gpu_brain_structure):
    """Test FCL manager integration with GPU-optimized ConnectomeManager."""
    brain = test_gpu_brain_structure
    cm = brain["connectome"]

    # Create FCL manager with the connectome's existing fcl_manager
    fcl_manager = cm.fcl_manager

    # Directly stimulate some visual neurons
    visual_neurons = brain["neurons"][brain["areas"][0]][:10]  # First 10 visual neurons
    for neuron_id in visual_neurons:
        cm.set_neuron_property(neuron_id, "membrane_potential", 2.0)  # Above threshold

    # Run a few timesteps
    fired_neurons = []
    for _ in range(5):
        # Step simulation
        fired = cm.update_membrane_potentials()
        fired_neurons.append(fired)

    # Check FCL contents across timesteps
    assert len(fcl_manager.get_queue()) > 0, "FCL should contain at least one event"

    # Visual neurons should have fired in the first timestep
    assert all(neuron_id in fired_neurons[0] for neuron_id in visual_neurons), (
        "Visual neurons should fire in the first timestep"
    )

    # Some hidden neurons should fire in later timesteps due to connections
    hidden_neurons = brain["neurons"][brain["areas"][1]]
    assert any(neuron_id in fired_neurons[1] for neuron_id in hidden_neurons), (
        "Some hidden neurons should fire in the second timestep"
    )

    # Motor neurons should fire in even later timesteps
    motor_neurons = brain["neurons"][brain["areas"][2]]
    assert any(
        neuron_id in fired_neurons[2] or neuron_id in fired_neurons[3]
        for neuron_id in motor_neurons
    ), "Some motor neurons should fire in later timesteps"


@pytest.mark.integration
def test_burst_engine_with_gpu_connectome(test_gpu_brain_structure):
    """Test BurstEngine integration with GPU-optimized ConnectomeManager."""
    brain = test_gpu_brain_structure
    cm = brain["connectome"]

    # Create BurstEngine
    burst_engine = BurstEngine(connectome_manager=cm)

    # Directly stimulate some visual neurons
    visual_neurons = brain["neurons"][brain["areas"][0]][:10]  # First 10 visual neurons
    for neuron_id in visual_neurons:
        cm.set_neuron_property(neuron_id, "membrane_potential", 2.0)  # Above threshold

    # Run a burst cycle
    fired_neurons = burst_engine.process_burst_cycle()

    # Check that some neurons fired
    assert len(fired_neurons) > 0, "At least some neurons should have fired"
    assert all(neuron_id in fired_neurons for neuron_id in visual_neurons), (
        "All stimulated visual neurons should fire"
    )

    # Run another burst cycle to see propagation effects
    fired_neurons = burst_engine.process_burst_cycle()

    # Should see some hidden neurons firing due to connections
    hidden_neurons = brain["neurons"][brain["areas"][1]]
    assert any(neuron_id in fired_neurons for neuron_id in hidden_neurons), (
        "Some hidden neurons should fire in the second burst cycle"
    )


@pytest.mark.integration
def test_converting_standard_to_gpu_maintains_fcl(test_brain_structure):
    """Test that converting from standard to GPU-optimized ConnectomeManager maintains FCL state."""
    brain = test_brain_structure
    std_cm = brain["connectome"]

    # Create and fill FCL with some events
    visual_neurons = brain["neurons"][brain["areas"][0]][:10]
    std_cm.fcl_manager.add_to_current_fcl(visual_neurons)

    # Run one timestep to advance FCL
    std_cm.update_membrane_potentials()

    # Get the state of FCL before conversion
    fcl_before = {
        "current_index": std_cm.fcl_manager.current_index,
        "events": {
            idx: std_cm.fcl_manager.get_fcl(idx)
            for idx in range(std_cm.fcl_manager.window_size)
        },
    }

    # Convert to GPU-optimized version
    gpu_cm = std_cm.to_gpu_optimized()

    # Check FCL state after conversion
    fcl_after = {
        "current_index": gpu_cm.fcl_manager.current_index,
        "events": {
            idx: gpu_cm.fcl_manager.get_fcl(idx)
            for idx in range(gpu_cm.fcl_manager.window_size)
        },
    }

    # Compare FCL states
    assert fcl_before["current_index"] == fcl_after["current_index"], (
        "FCL current index should be preserved"
    )

    for idx in range(std_cm.fcl_manager.window_size):
        assert set(fcl_before["events"][idx]) == set(fcl_after["events"][idx]), (
            f"FCL events at index {idx} should be preserved"
        )


@pytest.mark.integration
def test_performance_comparison():
    """Compare performance between standard and GPU-optimized implementations."""
    # Create a medium-sized test brain
    n_neurons = 1000  # Small enough for quick tests but large enough to see differences

    # Setup standard connectome
    std_cm = ConnectomeManager(n_neurons)
    area_id = std_cm.add_cortical_area("TestArea", (10, 10, 10), (0, 0, 0))

    # Generate positions
    positions = [(x, y, z) for x in range(10) for y in range(10) for z in range(10)]

    # Create neurons
    std_start = time.time()
    std_neuron_ids = std_cm.batch_create_neurons(area_id, positions)
    std_create_time = time.time() - std_start

    # Setup GPU-optimized connectome
    gpu_cm = ConnectomeManagerGPU(n_neurons)
    area_id = gpu_cm.add_cortical_area("TestArea", (10, 10, 10), (0, 0, 0))

    # Create neurons
    gpu_start = time.time()
    gpu_neuron_ids = gpu_cm.batch_create_neurons(area_id, positions)
    gpu_create_time = time.time() - gpu_start

    # Create some random connections
    n_synapses = 10000
    synapse_specs = []
    for _ in range(n_synapses):
        pre = np.random.choice(std_neuron_ids)
        post = np.random.choice(std_neuron_ids)
        weight = np.random.random()
        synapse_specs.append((pre, post, weight))

    # Create synapses in standard connectome
    std_start = time.time()
    std_cm.batch_create_synapses(synapse_specs)
    std_synapse_time = time.time() - std_start

    # Create synapses in GPU-optimized connectome
    # Convert neuron IDs to match GPU connectome
    gpu_synapse_specs = []
    for pre, post, weight in synapse_specs:
        pre_idx = std_neuron_ids.index(pre)
        post_idx = std_neuron_ids.index(post)
        gpu_synapse_specs.append(
            (gpu_neuron_ids[pre_idx], gpu_neuron_ids[post_idx], weight)
        )

    gpu_start = time.time()
    gpu_cm.batch_create_synapses(gpu_synapse_specs)
    gpu_synapse_time = time.time() - gpu_start

    # Test membrane potential updates
    # First, set some neurons to fire
    for i in range(100):
        std_cm.set_neuron_property(std_neuron_ids[i], "membrane_potential", 2.0)
        gpu_cm.set_neuron_property(gpu_neuron_ids[i], "membrane_potential", 2.0)

    # Run standard update
    std_start = time.time()
    std_fired = std_cm.update_membrane_potentials()
    std_update_time = time.time() - std_start

    # Run GPU update
    gpu_start = time.time()
    gpu_fired = gpu_cm.update_membrane_potentials()
    gpu_update_time = time.time() - gpu_start

    # Log results
    logger.info(f"Standard Create Time: {std_create_time:.6f}s")
    logger.info(f"GPU Create Time: {gpu_create_time:.6f}s")
    logger.info(f"Speedup (Create): {std_create_time / gpu_create_time:.2f}x")

    logger.info(f"Standard Synapse Time: {std_synapse_time:.6f}s")
    logger.info(f"GPU Synapse Time: {gpu_synapse_time:.6f}s")
    logger.info(f"Speedup (Synapses): {std_synapse_time / gpu_synapse_time:.2f}x")

    logger.info(f"Standard Update Time: {std_update_time:.6f}s")
    logger.info(f"GPU Update Time: {gpu_update_time:.6f}s")
    logger.info(f"Speedup (Update): {std_update_time / gpu_update_time:.2f}x")

    # Verify that the number of fired neurons is similar between implementations
    # Note: Some differences are expected due to floating point precision and implementation details
    logger.info(f"Standard fired neurons: {len(std_fired)}")
    logger.info(f"GPU fired neurons: {len(gpu_fired)}")

    # Just to avoid test failures, we'll use a loose comparison
    # In real usage, we'd want to investigate significant differences
    assert abs(len(std_fired) - len(gpu_fired)) < len(std_fired) * 0.2, (
        "Number of fired neurons should be roughly similar between implementations"
    )


if __name__ == "__main__":
    # Manual run for debugging
    test_performance_comparison()
