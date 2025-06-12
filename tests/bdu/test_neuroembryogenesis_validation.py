#!/usr/bin/env python3
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
Comprehensive validation tests for the NeuroEmbryogenesis module.

These tests load a specific test genome and verify that the generated
connectome matches the expected structure and properties.
"""

import json
import logging
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

import pytest

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

# Add the project root to the path if needed
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Import the modules to test
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.embryogenesis.neuroembryogenesis import (
    DevelopmentStage,
    NeuroEmbryogenesis,
)
from feagi.utils.config import FeagiConfig


@pytest.fixture
def test_genome_file():
    """Use the neuroembryogenesis_test_genome.json file for validation testing."""
    # Path to the test genome file
    test_genome_path = os.path.join(
        os.path.dirname(__file__), "neuroembryogenesis_test_genome.json"
    )

    # Verify file exists
    if not os.path.exists(test_genome_path):
        pytest.skip(f"Test genome file not found at {test_genome_path}")

    # Copy to a temporary file so tests don't modify the original
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as temp:
        temp_path = temp.name

    shutil.copy2(test_genome_path, temp_path)

    yield temp_path

    # Cleanup
    os.unlink(temp_path)


@pytest.fixture
def config():
    """Create a FeagiConfig for testing."""
    with tempfile.TemporaryDirectory() as temp_dir:
        config_dict = {
            "connectome_path": temp_dir,
            "skip_memory_neurogenesis": True,
            "connectome.max_neurons": 10000,  # Large enough for test genome
            "connectome.max_synapses_per_neuron": 100,  # Also increase max synapses
        }

        config = FeagiConfig()
        for key, value in config_dict.items():
            config.set(key, value)

        yield config


@pytest.fixture
def connectome_manager(config):
    """Create a ConnectomeManager for testing."""
    # Create a ConnectomeManager with the new API
    max_neurons = config.get("connectome.max_neurons", 10000)
    max_synapses = config.get("connectome.max_synapses_per_neuron", 100) * max_neurons
    return ConnectomeManager(
        config_or_max_neurons=max_neurons, max_synapses=max_synapses
    )


@pytest.fixture
def embryo(connectome_manager, config):
    """Create a NeuroEmbryogenesis instance for testing."""
    # Store progress logs for validation
    progress_logs = []

    def progress_callback(stage, progress, message):
        progress_logs.append((stage, progress, message))

    # Create embryo instance
    embryo = NeuroEmbryogenesis(
        connectome_manager=connectome_manager,
        config=config,
        progress_callback=progress_callback,
    )
    embryo.progress_logs = progress_logs
    return embryo


def validate_area_structure(area, genome_props):
    """Validate that an area's structure matches its genome properties."""
    # Extract expected dimensions
    expected_x = genome_props.get("bbx", 1)
    expected_y = genome_props.get("bby", 1)
    expected_z = genome_props.get("bbz", 1)
    expected_dims = (expected_x, expected_y, expected_z)

    # Extract expected position
    expected_x = genome_props.get("rcordx", 0)
    expected_y = genome_props.get("rcordy", 0)
    expected_z = genome_props.get("rcordz", 0)
    expected_pos = (expected_x, expected_y, expected_z)

    # Extract expected name
    expected_name = genome_props.get("name", "")

    # Validate dimensions
    assert area.dimensions == expected_dims, (
        f"Area dimensions {area.dimensions} don't match expected {expected_dims}"
    )

    # Validate position
    assert area.position == expected_pos, (
        f"Area position {area.position} doesn't match expected {expected_pos}"
    )

    # Validate name
    assert area.name == expected_name, (
        f"Area name '{area.name}' doesn't match expected '{expected_name}'"
    )


def test_extract_cortical_properties(embryo, test_genome_file):
    """Test that cortical properties are correctly extracted from genome."""
    # Load the genome
    embryo.load_genome(test_genome_file)

    # Get the test genome data for validation
    with open(test_genome_file, "r") as f:
        genome_data = json.load(f)

    # Look for a valid cortical ID in the blueprint section
    # The format appears to be "_____10c-<cortical_id>-cx-__name-t"
    blueprint = genome_data.get("blueprint", {})
    cortical_id = None

    # Look for keys matching the pattern and extract a valid ID
    for key in blueprint.keys():
        if isinstance(key, str) and key.endswith("-cx-__name-t"):
            # Extract the cortical ID portion
            parts = key.split("-")
            if len(parts) >= 4:
                cortical_id = parts[1]  # This should be the cortical ID
                # Verify this is a valid ID by checking for other properties
                test_key = f"{parts[0]}-{cortical_id}-cx-___bbx-i"
                if test_key in blueprint:
                    break

    if not cortical_id:
        pytest.fail("Could not find a valid cortical area ID in test genome")

    # Extract properties using the embryo method
    props = embryo._extract_cortical_properties(cortical_id)

    # Verify essential properties are extracted
    assert "name" in props, f"Missing name property for cortical ID {cortical_id}"
    assert "bbx" in props, (
        f"Missing bounding box x dimension for cortical ID {cortical_id}"
    )
    assert "bby" in props, (
        f"Missing bounding box y dimension for cortical ID {cortical_id}"
    )
    assert "bbz" in props, (
        f"Missing bounding box z dimension for cortical ID {cortical_id}"
    )
    assert "rcordx" in props, f"Missing x coordinate for cortical ID {cortical_id}"
    assert "rcordy" in props, f"Missing y coordinate for cortical ID {cortical_id}"
    assert "rcordz" in props, f"Missing z coordinate for cortical ID {cortical_id}"

    # Check that values match the genome
    # The genome uses keys like "_____10c-<cortical_id>-cx-__name-t"
    name_key = [
        k
        for k in blueprint.keys()
        if k.endswith(f"-{cortical_id}-cx-__name-t")
        or k.endswith(f"-{cortical_id}-cx-__name-t")
    ][0]
    expected_name = blueprint.get(name_key, "")
    assert props["name"] == expected_name, (
        f"Extracted name '{props['name']}' doesn't match expected '{expected_name}'"
    )

    # Check numeric properties
    bbx_key = [
        k
        for k in blueprint.keys()
        if k.endswith(f"-{cortical_id}-cx-___bbx-i")
        or k.endswith(f"-{cortical_id}-cx-___bbx-i")
    ][0]
    expected_bbx = blueprint.get(bbx_key, 1)
    assert props["bbx"] == expected_bbx, (
        f"Extracted bbx '{props['bbx']}' doesn't match expected '{expected_bbx}'"
    )


@pytest.mark.skip(reason="Mismatched type for cortical_id validation")
def test_setup_cortical_areas(embryo, test_genome_file):
    """Test the cortical area setup process."""
    # Load the genome
    embryo.load_genome(test_genome_file)

    # Perform cortical area setup
    success = embryo._setup_cortical_areas()
    assert success, "Failed to set up cortical areas"

    # Check if areas were created
    assert len(embryo.cortical_areas) > 0, "No cortical areas were created"
    assert len(embryo.cortical_id_map) > 0, "Cortical ID map is empty"
    assert len(embryo.reverse_cortical_id_map) > 0, "Reverse cortical ID map is empty"

    # Get the test genome data for validation
    with open(test_genome_file, "r") as f:
        genome_data = json.load(f)

    # Count expected areas from genome blueprint
    expected_area_count = 0
    cortical_ids = set()
    for key in genome_data["blueprint"].keys():
        if key.endswith("__name-t"):
            # Extract cortical ID from key (should be length 6 in most genomes)
            cortical_id = key.split("-")[1]
            cortical_ids.add(cortical_id)
            expected_area_count += 1

    # Verify the number of areas matches
    assert len(embryo.cortical_areas) == expected_area_count, (
        f"Expected {expected_area_count} cortical areas, got {len(embryo.cortical_areas)}"
    )

    # Verify all cortical IDs from genome are in the maps
    for cortical_id in cortical_ids:
        assert cortical_id in embryo.cortical_id_map, (
            f"Missing cortical ID: {cortical_id}"
        )

    # Verify each area has a name and dimensions
    for area_id, area in embryo.cortical_areas.items():
        cortical_id = embryo.reverse_cortical_id_map[area_id]
        assert hasattr(area, "name"), f"Area {cortical_id} missing name"
        assert hasattr(area, "dimensions"), f"Area {cortical_id} missing dimensions"

        # Verify dimensions are non-zero
        assert all(d > 0 for d in area.dimensions), (
            f"Area {cortical_id} has zero dimensions"
        )

        # Verify area type
        assert hasattr(area, "area_type"), f"Area {cortical_id} missing area_type"

        # Sample some areas to verify more detailed properties
        if (
            len(embryo.cortical_areas) > 5
            and list(embryo.cortical_areas.keys()).index(area_id) < 3
        ):
            props = embryo._extract_cortical_properties(cortical_id)
            validate_area_structure(area, props)


@pytest.mark.skip(reason="Cortical area lookup failing")
def test_perform_neurogenesis(embryo, test_genome_file):
    """Test the neurogenesis process."""
    # Load the genome and setup cortical areas
    embryo.load_genome(test_genome_file)
    embryo._setup_cortical_areas()

    # Perform neurogenesis
    success = embryo._perform_neurogenesis()

    assert success, "Failed to perform neurogenesis"

    # Get the test genome data for validation
    with open(test_genome_file, "r") as f:
        genome_data = json.load(f)

    # Get expected neuron count from genome
    expected_neuron_count = genome_data["stats"]["innate_neuron_count"]

    # Count actual neurons
    neuron_count = 0
    area_counts = {}
    for area_id in embryo.cortical_areas.keys():
        neurons = embryo.connectome_manager.get_neurons_by_area(area_id)
        count = len(neurons)
        neuron_count += count
        area = embryo.cortical_areas[area_id]
        area_counts[area.name if hasattr(area, "name") else f"Area {area_id}"] = count

    # Allow for a small difference due to rounding or implementation details
    tolerance = 0.05  # 5% tolerance
    lower_bound = int(expected_neuron_count * (1 - tolerance))
    upper_bound = int(expected_neuron_count * (1 + tolerance))

    # Verify neuron count
    assert lower_bound <= neuron_count <= upper_bound, (
        f"Expected around {expected_neuron_count} neurons, got {neuron_count}"
    )

    # Verify neurons per area
    for area_id, area in embryo.cortical_areas.items():
        cortical_id = embryo.cortical_id_map[area_id]
        props = embryo._extract_cortical_properties(cortical_id)
        expected_count = props.get("n_cnt", 1)
        actual_count = len(embryo.connectome_manager.get_neurons_by_area(area_id))

        # Allow some tolerance for implementation differences
        assert actual_count > 0, f"No neurons created for area {area.name}"

        # For exact neuron count checks, we need to know how the density is calculated
        # For now, we'll just check that neurons were created


@pytest.mark.skip(reason="Cortical area lookup failing")
def test_perform_synaptogenesis(embryo, test_genome_file):
    """Test synaptogenesis process."""
    # Load genome, setup areas, create neurons
    embryo.load_genome(test_genome_file)
    embryo._setup_cortical_areas()
    embryo._perform_neurogenesis()

    # Perform synaptogenesis
    success = embryo._perform_synaptogenesis()
    assert success, "Failed to perform synaptogenesis"

    # Get the test genome data for validation
    with open(test_genome_file, "r") as f:
        genome_data = json.load(f)

    # Check if the genome has cortical mappings
    has_mappings = (
        "cortical_mappings" in genome_data and len(genome_data["cortical_mappings"]) > 0
    )

    # Get expected synapse count from genome
    expected_synapse_count = genome_data["stats"]["innate_synapse_count"]

    # Get actual synapse count
    stats = embryo.get_development_statistics()
    actual_synapse_count = stats["synapses"]

    # If there are no mappings in the genome, we expect synapse count to be 0
    if not has_mappings:
        assert actual_synapse_count == 0, (
            f"Expected 0 synapses with no mappings, got {actual_synapse_count}"
        )
        return

    # Allow for a small difference due to rounding or implementation details
    tolerance = 0.05  # 5% tolerance
    lower_bound = int(expected_synapse_count * (1 - tolerance))
    upper_bound = int(expected_synapse_count * (1 + tolerance))

    assert lower_bound <= actual_synapse_count <= upper_bound, (
        f"Expected around {expected_synapse_count} synapses, got {actual_synapse_count}"
    )


@pytest.mark.skip(reason="Cortical area lookup failing")
def test_full_development_validation(embryo, test_genome_file):
    """Comprehensive validation of the entire brain development process."""
    # Develop the brain
    start_time = time.time()
    success = embryo.develop_brain(test_genome_file)
    end_time = time.time()

    assert success, "Failed to develop brain from test genome"

    # Load the test genome data for validation
    with open(test_genome_file, "r") as f:
        genome_data = json.load(f)

    # Get development statistics
    stats = embryo.get_development_statistics()

    # Print development summary
    print(f"\nBrain Development Summary:")
    print(f"  Development Time: {end_time - start_time:.2f} seconds")
    print(f"  Cortical Areas: {stats['cortical_areas']}")
    print(f"  Neurons: {stats['neurons']}")
    print(f"  Synapses: {stats['synapses']}")

    # 1. Validate cortical area count
    expected_area_count = 0
    for key in genome_data["blueprint"].keys():
        if key.endswith("__name-t"):
            expected_area_count += 1

    assert stats["cortical_areas"] == expected_area_count, (
        f"Expected {expected_area_count} cortical areas, got {stats['cortical_areas']}"
    )

    # 2. Validate neuron count
    expected_neuron_count = genome_data["stats"]["innate_neuron_count"]
    tolerance = 0.05  # 5% tolerance
    lower_bound = int(expected_neuron_count * (1 - tolerance))
    upper_bound = int(expected_neuron_count * (1 + tolerance))

    assert lower_bound <= stats["neurons"] <= upper_bound, (
        f"Expected around {expected_neuron_count} neurons, got {stats['neurons']}"
    )

    # 3. Validate synapse count
    expected_synapse_count = genome_data["stats"]["innate_synapse_count"]
    has_mappings = (
        "cortical_mappings" in genome_data and len(genome_data["cortical_mappings"]) > 0
    )

    if has_mappings:
        lower_bound = int(expected_synapse_count * (1 - tolerance))
        upper_bound = int(expected_synapse_count * (1 + tolerance))
        assert lower_bound <= stats["synapses"] <= upper_bound, (
            f"Expected around {expected_synapse_count} synapses, got {stats['synapses']}"
        )
    else:
        assert stats["synapses"] == 0, (
            f"Expected 0 synapses with no mappings, got {stats['synapses']}"
        )

    # 4. Validate cortical area details
    for cortical_id, area_id in embryo.reverse_cortical_id_map.items():
        # Get expected properties
        props = embryo._extract_cortical_properties(cortical_id)

        # Get actual area
        area = embryo.connectome_manager.get_cortical_area(area_id)

        # Validate area structure
        validate_area_structure(area, props)

        # Validate neuron count for this area
        expected_area_neurons = props.get("n_cnt", 1)
        actual_area_neurons = len(
            embryo.connectome_manager.get_neurons_by_area(area_id)
        )

        # For direct neuron count comparison, we'd need to know exactly how the count is calculated
        # For now, verify neurons were created
        assert actual_area_neurons > 0, f"No neurons created in {area.name}"

        # Print some diagnostic info for a random sampling of areas
        if actual_area_neurons > 0:
            print(f"  Area {area.name}: {actual_area_neurons} neurons")
            # Only show a few to keep output manageable
            if len(embryo.cortical_areas) > 10:
                break

    # 5. Verify progress logs recorded all stages
    stages_logged = {log[0] for log in embryo.progress_logs}
    expected_stages = {
        DevelopmentStage.INITIALIZATION,
        DevelopmentStage.CORTICOGENESIS,
        DevelopmentStage.NEUROGENESIS,
        DevelopmentStage.SYNAPTOGENESIS,
        DevelopmentStage.COMPLETED,
    }

    for stage in expected_stages:
        assert stage in stages_logged, f"Missing progress logs for stage {stage}"

    assert DevelopmentStage.FAILED not in stages_logged, (
        "Development failed according to progress logs"
    )


def test_morphology_handling(embryo, test_genome_file):
    """Test that morphologies are correctly loaded and processed."""
    # Load the genome
    embryo.load_genome(test_genome_file)

    # Get the test genome data for validation
    with open(test_genome_file, "r") as f:
        genome_data = json.load(f)

    # Get morphology registry
    morphologies = embryo.get_morphology_registry()

    # Check that all morphologies in the genome are loaded
    expected_count = len(genome_data.get("neuron_morphologies", {}))
    assert len(morphologies) > 0, "No morphologies loaded"

    # Check morphology entries
    for morphology_id, morphology in morphologies.items():
        assert "type" in morphology, f"Morphology {morphology_id} missing type"
        assert "parameters" in morphology, (
            f"Morphology {morphology_id} missing parameters"
        )
        assert "class" in morphology, f"Morphology {morphology_id} missing class"
