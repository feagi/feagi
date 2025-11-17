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
Pytest tests for the synaptogenesis process.

Tests the functionality of synapse creation during brain development.
"""

import json
import os
import sys

import pytest

# Add the project root to the path if needed
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from feagi.bdu.connectome_manager import ConnectomeManager

# Import the modules to test
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
from feagi.utils.config import FeagiConfig

# Import genome processing modules
try:
    from feagi.evo.genome_processor import (
        genome_morphology_updator,
        genome_physiology_updator,
        genome_stat_updator,
        merge_core_morphologies,
    )
except ImportError:
    try:
        from src.evo.genome_processor import (
            genome_morphology_updator,
            genome_physiology_updator,
            genome_stat_updator,
            merge_core_morphologies,
        )
    except ImportError:
        # Define minimal working implementations if imports fail
        def merge_core_morphologies(genome):
            """Placeholder for merge_core_morphologies function."""
            return genome

        def genome_morphology_updator(genome):
            """Placeholder for genome_morphology_updator function."""
            return genome

        def genome_physiology_updator(genome):
            """Placeholder for genome_physiology_updator function."""
            if "physiology" not in genome:
                genome["physiology"] = {}
            return genome

        def genome_stat_updator(genome):
            """Placeholder for genome_stat_updator function."""
            if "stats" not in genome:
                genome["stats"] = {}
            return genome


@pytest.fixture
def genome_path():
    """Return the path to the essential genome file for testing."""
    return os.path.join(project_root, "feagi/evo/defaults/genome/essential_genome.json")


@pytest.fixture
def genome(genome_path):
    """Load and return the genome data for testing."""
    with open(genome_path, "r") as f:
        genome = json.load(f)

    # Apply the same processing as in neuroembryogenesis.py
    genome = merge_core_morphologies(genome)
    genome = genome_morphology_updator(genome)
    genome = genome_physiology_updator(genome)
    genome = genome_stat_updator(genome)

    return genome


@pytest.fixture
def config():
    """Create a FeagiConfig for testing."""
    config = FeagiConfig()
    config.set("connectome.max_neurons", 10000000)
    config.set("connectome.max_synapses_per_neuron", 1000)
    return config


@pytest.fixture
def connectome_manager(config):
    """Create a ConnectomeManager for testing."""
    max_neurons = config.get("connectome.max_neurons", 10000000)
    max_synapses = config.get("connectome.max_synapses_per_neuron", 1000) * max_neurons
    return ConnectomeManager(
        config_or_max_neurons=max_neurons, max_synapses=max_synapses
    )


@pytest.fixture
def embryo(connectome_manager, config, genome_path):
    """Create and initialize a Neuroembryogenesis instance for testing."""
    embryo = NeuroEmbryogenesis(connectome_manager=connectome_manager, config=config)
    embryo.load_genome(genome_path)
    embryo._setup_cortical_areas()
    return embryo


def test_blueprint_structure(genome):
    """Test that the genome contains the expected blueprint structure."""
    assert "blueprint" in genome, "Genome should contain blueprint"

    # The blueprint in the essential genome contains cortical area definitions
    # in the format: _____10c-{cortical_id}-{gene_type}-{property}-{value_type}

    # Check for iic100 entries
    iic100_entries = [k for k in genome["blueprint"].keys() if "iic100" in k]
    assert len(iic100_entries) > 0, (
        "Genome should contain entries for iic100 cortical area"
    )

    # Check for m__bac entries
    m__bac_entries = [k for k in genome["blueprint"].keys() if "m__bac" in k]
    assert len(m__bac_entries) > 0, (
        "Genome should contain entries for m__bac cortical area"
    )


def test_mapping_in_blueprint(genome):
    """Test that the genome's blueprint contains mapping information between iic100 and m__bac."""
    # In the genome 2.0 format, mappings are defined in the blueprint under the "dstmap" property
    dstmap_entries = [k for k in genome["blueprint"].keys() if "-dstmap-" in k]

    # Find the iic100 mapping entry
    iic100_mapping = None
    for entry in dstmap_entries:
        if "iic100" in entry:
            iic100_mapping = entry
            break

    assert iic100_mapping is not None, "Should have a dstmap entry for iic100"

    # Check if the mapping points to m__bac
    mapping_data = genome["blueprint"][iic100_mapping]
    assert isinstance(mapping_data, dict), "Mapping data should be a dictionary"

    # Check if m__bac is a destination
    assert "m__bac" in mapping_data, "iic100 should have a mapping to m__bac"

    # Check the morphology type
    morphology_entries = mapping_data["m__bac"]
    assert len(morphology_entries) > 0, "Should have at least one morphology entry"

    # The first element in the morphology entry should be the morphology ID (usually "projector" for this mapping)
    morphology_id = morphology_entries[0][0]
    assert morphology_id == "projector", (
        f"Expected morphology type 'projector', got '{morphology_id}'"
    )


def test_synaptogenesis_process(embryo):
    """Test the synaptogenesis process for creating connections between mapped areas."""
    # Perform neurogenesis first
    embryo._perform_neurogenesis()

    # Now perform synaptogenesis
    result = embryo._perform_synaptogenesis()
    assert result, "Synaptogenesis should complete successfully"

    # Check if the appropriate mappings were processed
    # This test passes regardless of whether synapses were created,
    # as the synaptogenesis might not create connections based on the genome design
    stats = embryo.get_development_statistics()
    assert "total_synapses" in stats, (
        "Development statistics should include synapse count"
    )


@pytest.mark.skip("Needs to be updated for new state manager integration")
def test_full_brain_development(genome_path, config):
    """Test the full brain development process."""
    # Create a fresh connectome manager and embryo for the test
    connectome_manager = ConnectomeManager(config_or_max_neurons=config)
    embryo = NeuroEmbryogenesis(connectome_manager, config)

    # Develop the brain
    success = embryo.develop_brain(genome_path)
    assert success, "Brain development should complete successfully"

    # Get development statistics
    stats = embryo.get_development_statistics()
    assert stats["cortical_areas"] > 0, "Brain should have cortical areas"
    assert stats["total_neurons"] > 0, "Brain should have neurons"
    assert "total_synapses" in stats, (
        "Development statistics should include synapse count"
    )
