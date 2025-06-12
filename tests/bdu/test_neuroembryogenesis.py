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
Pytest tests for the Neuroembryogenesis module.

Tests the functionality of the Neuroembryogenesis module for developing
a brain from a genome file.
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
sys.path.insert(0, project_root)

# Import the modules to test
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.embryogenesis.neuroembryogenesis import (
    DevelopmentStage,
    NeuroEmbryogenesis,
)
from feagi.utils.config import FeagiConfig


@pytest.fixture
def genome_file():
    """Use the essential_genome.json file for testing."""
    # Path to the essential genome file
    essential_genome_path = os.path.join(
        project_root, "feagi/evo/defaults/genome/essential_genome.json"
    )

    # Copy to a temporary file so tests don't modify the original
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as temp:
        temp_path = temp.name

    shutil.copy2(essential_genome_path, temp_path)

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
            "connectome.max_neurons": 100000,  # Increase to handle large essential genome
            "connectome.max_synapses_per_neuron": 1000,  # Also increase max synapses
        }

        try:
            config = FeagiConfig(**config_dict)
        except TypeError:
            # Try alternative initialization methods if needed
            try:
                config = FeagiConfig(config_dict)
            except TypeError:
                config = FeagiConfig()
                for key, value in config_dict.items():
                    config.set(key, value)

        yield config


@pytest.fixture
def connectome_manager(config):
    """Create a ConnectomeManager for testing."""
    # Create a ConnectomeManager with the new API
    max_neurons = config.get("connectome.max_neurons", 100000)
    max_synapses = config.get("connectome.max_synapses_per_neuron", 1000) * max_neurons
    return ConnectomeManager(
        config_or_max_neurons=max_neurons, max_synapses=max_synapses
    )


@pytest.fixture
def embryo(connectome_manager, config):
    """Create a Neuroembryogenesis instance for testing."""
    progress_logs = []

    def progress_callback(stage, progress, message):
        progress_logs.append((stage, progress, message))

    embryo = NeuroEmbryogenesis(
        connectome_manager=connectome_manager,
        config=config,
        progress_callback=progress_callback,
    )
    embryo.progress_logs = progress_logs
    return embryo


def test_neuroembryogenesis_initialization(embryo):
    """Test that Neuroembryogenesis initializes correctly."""
    assert embryo is not None
    assert embryo.connectome_manager is not None
    assert embryo.stage == DevelopmentStage.INITIALIZATION


def test_load_genome(embryo, genome_file):
    """Test that a genome can be loaded."""
    success = embryo.load_genome(genome_file)
    assert success
    assert embryo.genome is not None
    assert "blueprint" in embryo.genome


def test_cortical_area_setup(embryo, genome_file):
    """Test setting up cortical areas from a genome."""
    # Load the genome first
    embryo.load_genome(genome_file)

    # Setup cortical areas
    success = embryo._setup_cortical_areas()
    assert success

    # Check that areas were created
    assert len(embryo.cortical_areas) > 0

    # Verify the connectome manager has the areas
    area_ids = set(embryo.cortical_id_map.keys())
    assert len(area_ids) > 0


def test_neurogenesis(embryo, genome_file):
    """Test neurogenesis process."""
    total_start = time.time()

    # Load the genome and setup areas
    genome_load_start = time.time()
    embryo.load_genome(genome_file)
    genome_load_end = time.time()
    print(f"\nGenome load time: {genome_load_end - genome_load_start:.3f} seconds")

    setup_areas_start = time.time()
    embryo._setup_cortical_areas()
    setup_areas_end = time.time()
    print(
        f"Cortical area setup time: {setup_areas_end - setup_areas_start:.3f} seconds"
    )

    # Perform neurogenesis with timing
    neurogenesis_start = time.time()
    success = embryo._perform_neurogenesis()
    neurogenesis_end = time.time()

    # Get detailed metrics
    total_neurons = embryo.development_stats["neurons"]
    total_areas = len(embryo.cortical_areas)
    neurogenesis_duration = neurogenesis_end - neurogenesis_start

    print(f"Neurogenesis execution time: {neurogenesis_duration:.3f} seconds")
    print(f"Created {total_neurons} neurons across {total_areas} areas")

    if total_neurons > 0:
        print(f"Rate: {total_neurons / neurogenesis_duration:.2f} neurons/second")

    total_end = time.time()
    print(f"Total test time: {total_end - total_start:.3f} seconds")

    assert success


def test_synaptogenesis(embryo, genome_file):
    """Test synaptogenesis process."""
    total_start = time.time()

    # Load the genome, setup areas, and create neurons
    setup_start = time.time()
    embryo.load_genome(genome_file)
    embryo._setup_cortical_areas()
    embryo._perform_neurogenesis()
    setup_end = time.time()
    print(
        f"\nSetup time (genome, areas, neurons): {setup_end - setup_start:.3f} seconds"
    )

    # Count neurons before synaptogenesis
    count_start = time.time()
    neuron_count = sum(
        len(embryo.connectome_manager.get_neurons_by_area(area_id))
        for area_id in embryo.cortical_areas.keys()
    )
    count_end = time.time()
    print(f"Neuron counting time: {count_end - count_start:.3f} seconds")
    print(f"Created {neuron_count} neurons across {len(embryo.cortical_areas)} areas")

    # Determine if genome has any cortical mappings
    mappings_start = time.time()
    has_mappings = (
        "cortical_mappings" in embryo.genome
        and len(embryo.genome["cortical_mappings"]) > 0
    )
    mappings_end = time.time()
    print(f"Mapping check time: {mappings_end - mappings_start:.3f} seconds")

    # Print mapping information
    if has_mappings:
        mapping_count = len(embryo.genome["cortical_mappings"])
        print(f"Found {mapping_count} cortical mappings in genome")
    else:
        print("No cortical mappings found in genome")

    # Perform synaptogenesis with timing
    synapse_start = time.time()
    success = embryo._perform_synaptogenesis()
    synapse_end = time.time()
    print(f"Synaptogenesis execution time: {synapse_end - synapse_start:.3f} seconds")

    assert success

    # Get the synapse count from the statistics
    stats_start = time.time()
    stats = embryo.get_development_statistics()
    stats_end = time.time()
    print(f"Statistics collection time: {stats_end - stats_start:.3f} seconds")

    print(f"Created {stats['synapses']} synapses")

    # If there are no mappings in the genome, we expect synapse count to be 0
    # Otherwise, there should be synapses created
    if has_mappings:
        assert stats["synapses"] > 0
    else:
        assert stats["synapses"] == 0

    total_end = time.time()
    print(f"Total test time: {total_end - total_start:.3f} seconds")


def test_full_development(embryo, genome_file):
    """Test the full brain development process."""
    # Measure overall time
    total_start = time.time()

    # Measure development time
    develop_start = time.time()
    success = embryo.develop_brain(genome_file)
    develop_end = time.time()
    print(f"\nBrain development time: {develop_end - develop_start:.3f} seconds")

    assert success

    # Check development statistics
    stats = embryo.get_development_statistics()

    # Detailed timing breakdown from statistics
    print("\nDetailed Development Timing:")
    for stage, duration in stats.get("stage_durations", {}).items():
        print(f"  {stage}: {duration:.3f} seconds")

    # Print statistics
    print(f"\nDevelopment Statistics:")
    print(f"  Cortical Areas: {stats['cortical_areas']}")
    print(f"  Neurons: {stats['neurons']}")
    print(f"  Synapses: {stats['synapses']}")

    # Check if duration is a timedelta object and convert to seconds if needed
    duration = stats["duration"]
    if hasattr(duration, "total_seconds"):
        duration_seconds = duration.total_seconds()
        print(f"  Development Duration: {duration_seconds:.3f} seconds")
    else:
        print(f"  Development Duration: {duration}")

    # Assertions for development statistics
    assert stats["cortical_areas"] > 0
    assert stats["neurons"] > 0

    # Check if the genome has cortical mappings
    with open(genome_file, "r") as f:
        genome_data = json.load(f)

    has_mappings = (
        "cortical_mappings" in genome_data and len(genome_data["cortical_mappings"]) > 0
    )

    # If there are no mappings, synapses will be 0
    if has_mappings:
        assert stats["synapses"] > 0
    else:
        # No mappings, so it's expected to have 0 synapses
        assert stats["synapses"] == 0

    assert stats["duration"] is not None

    # Print detailed progress logs
    print("\nProgress Log Summary:")
    stage_logs = {}
    for stage, progress, message in embryo.progress_logs:
        if stage not in stage_logs:
            stage_logs[stage] = []
        stage_logs[stage].append((progress, message))

    for stage, logs in stage_logs.items():
        print(f"  {stage}:")
        print(f"    First: {logs[0][1]} ({logs[0][0]}%)")
        print(f"    Last: {logs[-1][1]} ({logs[-1][0]}%)")

    total_end = time.time()
    print(f"\nTotal test time: {total_end - total_start:.3f} seconds")


def test_synapse_manager_use_in_development(embryo, genome_file):
    """Test that the SynapseManager is properly used during development."""
    # Develop the brain
    success = embryo.develop_brain(genome_file)
    assert success

    # Check if the genome has cortical mappings
    with open(genome_file, "r") as f:
        genome_data = json.load(f)

    has_mappings = (
        "cortical_mappings" in genome_data and len(genome_data["cortical_mappings"]) > 0
    )

    # Get the synapse count from the development statistics
    stats = embryo.get_development_statistics()
    synapse_count = stats["synapses"]

    # Check that synapses were created if mappings exist
    if has_mappings:
        assert synapse_count > 0, (
            "Expected synapses to be created with cortical mappings"
        )
    else:
        # No mappings, so it's expected to have 0 synapses
        assert synapse_count == 0, "Expected 0 synapses with no cortical mappings"
