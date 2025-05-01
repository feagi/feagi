#!/usr/bin/env python3
"""
Pytest tests for the Neuroembryogenesis module.

Tests the functionality of the Neuroembryogenesis module for developing
a brain from a genome file.
"""

import os
import sys
import logging
import pytest
from pathlib import Path
import json
import tempfile
import shutil

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Add the project root to the path if needed
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, project_root)

# Import the modules to test
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.neuroembryogenesis import Neuroembryogenesis, DevelopmentStage
from feagi.utils.config import FeagiConfig


@pytest.fixture
def genome_file():
    """Use the essential_genome.json file for testing."""
    # Path to the essential genome file
    essential_genome_path = os.path.join(project_root, "feagi/evo/defaults/genome/essential_genome.json")
    
    # Copy to a temporary file so tests don't modify the original
    with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as temp:
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
            "connectome.max_neurons": 10000000,  # Increase to handle large essential genome
            "connectome.max_synapses_per_neuron": 1000  # Also increase max synapses
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
    return ConnectomeManager(config)


@pytest.fixture
def embryo(connectome_manager, config):
    """Create a Neuroembryogenesis instance for testing."""
    progress_logs = []
    
    def progress_callback(stage, progress, message):
        progress_logs.append((stage, progress, message))
    
    embryo = Neuroembryogenesis(
        connectome_manager=connectome_manager,
        config=config,
        progress_callback=progress_callback
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
    # Load the genome and setup areas
    embryo.load_genome(genome_file)
    embryo._setup_cortical_areas()
    
    # Perform neurogenesis
    success = embryo._perform_neurogenesis()
    assert success
    
    # Get all neurons from the connectome manager
    neuron_count = 0
    for area_id in embryo.cortical_areas.keys():
        neurons = embryo.connectome_manager.get_neurons_by_area(area_id)
        neuron_count += len(neurons)
    
    # Verify neurons were created
    assert neuron_count > 0


def test_synaptogenesis(embryo, genome_file):
    """Test synaptogenesis process."""
    # Load the genome, setup areas, and create neurons
    embryo.load_genome(genome_file)
    embryo._setup_cortical_areas()
    embryo._perform_neurogenesis()
    
    # Determine if genome has any cortical mappings
    has_mappings = "cortical_mappings" in embryo.genome and len(embryo.genome["cortical_mappings"]) > 0
    
    # Perform synaptogenesis
    success = embryo._perform_synaptogenesis()
    assert success
    
    # Get the synapse count from the statistics
    stats = embryo.get_development_statistics()
    
    # If there are no mappings in the genome, we expect synapse count to be 0
    # Otherwise, there should be synapses created
    if has_mappings:
        assert stats["synapses"] > 0
    else:
        assert stats["synapses"] == 0


def test_full_development(embryo, genome_file):
    """Test the full brain development process."""
    # Develop the brain
    success = embryo.develop_brain(genome_file)
    assert success
    
    # Check development statistics
    stats = embryo.get_development_statistics()
    assert stats["cortical_areas"] > 0
    assert stats["neurons"] > 0
    
    # Check if the genome has cortical mappings
    with open(genome_file, 'r') as f:
        genome_data = json.load(f)
    
    has_mappings = "cortical_mappings" in genome_data and len(genome_data["cortical_mappings"]) > 0
    
    # If there are no mappings, synapses will be 0
    if has_mappings:
        assert stats["synapses"] > 0
    else:
        # No mappings, so it's expected to have 0 synapses
        assert stats["synapses"] == 0
        
    assert stats["duration"] is not None


def test_synapse_manager_use_in_development(embryo, genome_file):
    """Test that the SynapseManager is properly used during development."""
    # Develop the brain
    success = embryo.develop_brain(genome_file)
    assert success
    
    # Check if the genome has cortical mappings
    with open(genome_file, 'r') as f:
        genome_data = json.load(f)
    
    has_mappings = "cortical_mappings" in genome_data and len(genome_data["cortical_mappings"]) > 0
    
    # Check that the SynapseManager has stored synapses
    synapse_count = embryo.connectome_manager.synapse_manager.get_synapse_count()
    
    if has_mappings:
        assert synapse_count > 0
    else:
        # No mappings, so it's expected to have 0 synapses
        assert synapse_count == 0
    
    # Check that the statistics match what's in the SynapseManager
    stats = embryo.get_development_statistics()
    assert stats["synapses"] == synapse_count 