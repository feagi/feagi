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
Pytest tests for synaptogenesis performance.

This tests the performance of the synaptogenesis process.
"""

import cProfile
import json
import logging
import os
import pstats
import sys
import tempfile
import time

import pytest

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

# Add the project root to the path if needed
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from feagi.bdu.connectome_manager import ConnectomeManager

# Import the modules to test
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
from feagi.utils.config import FeagiConfig


@pytest.fixture
def genome_file():
    """Use the essential_genome.json file for testing."""
    # Path to the essential genome file
    essential_genome_path = os.path.join(
        project_root, "feagi_core/feagi/evo/defaults/genome/essential_genome.json"
    )

    # Copy to a temporary file
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False, mode="w") as temp:
        temp_path = temp.name

        if os.path.exists(essential_genome_path):
            # Read the essential genome
            with open(essential_genome_path, "r") as f:
                genome = json.load(f)
        else:
            # Create a minimal genome if file not found
            print(
                f"Warning: Essential genome not found at {essential_genome_path}, using minimal test genome"
            )
            genome = {
                "version": "1.0",
                "neuron_morphologies": {
                    "default": {
                        "type": "simplified",
                        "soma_size": 1.0,
                        "dendrite_length": 5.0,
                        "axon_length": 10.0,
                    }
                },
                "blueprint": {
                    "test_area": {
                        "area_name": "Test Area",
                        "init_neuron_count": 100,
                        "dimensions": [5, 5, 2],
                        "coordinates": [0, 0, 0],
                        "type": "custom",
                        "morphology": "default",
                    }
                },
                "cortical_mappings": [
                    {
                        "source": "test_area",
                        "destination": "test_area",
                        "pattern": "one-to-one",
                        "synapse_definition": {"type": "excitatory", "polarity": 1},
                    }
                ],
            }

        # Write genome to temp file
        json.dump(genome, temp, indent=2)

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
            "connectome.max_neurons": 10000000,
            "connectome.max_synapses_per_neuron": 1000,
        }

        config = FeagiConfig()
        for key, value in config_dict.items():
            config.set(key, value)

        yield config


@pytest.fixture
def connectome_manager(config):
    """Create a ConnectomeManager for testing."""
    # Create a ConnectomeManager with limited neuron count for faster testing
    max_neurons = 100000  # Increased to handle essential genome
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


def run_with_profiling(function, *args, **kwargs):
    """Run a function with cProfile to collect performance metrics."""
    profiler = cProfile.Profile()
    profiler.enable()

    result = function(*args, **kwargs)

    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats("cumtime")

    return result, stats


def test_synaptogenesis_performance(embryo, genome_file):
    """Test the performance of the synaptogenesis process with detailed profiling."""
    # Start total timing
    total_start = time.time()

    # Setup phase
    setup_start = time.time()
    embryo.load_genome(genome_file)
    embryo._setup_cortical_areas()
    embryo._perform_neurogenesis()
    setup_end = time.time()
    setup_time = setup_end - setup_start
    print(f"\nSetup time (genome, areas, neurons): {setup_time:.6f} seconds")

    # Count neurons in each area
    area_stats = {}
    total_neurons = 0
    for area_id, area in embryo.connectome_manager.cortical_areas.items():
        neurons = embryo.connectome_manager.get_neurons_by_area(area_id)
        neuron_count = len(neurons)
        total_neurons += neuron_count
        area_stats[area.name] = neuron_count

    print(
        f"Created {total_neurons} neurons across {len(embryo.connectome_manager.cortical_areas)} cortical areas"
    )
    print("Top 5 areas by neuron count:")
    for name, count in sorted(area_stats.items(), key=lambda x: x[1], reverse=True)[:5]:
        print(f"  {name}: {count} neurons")

    # Run synaptogenesis with profiling
    print("\nRunning synaptogenesis with performance profiling...")
    _, stats = run_with_profiling(embryo._perform_synaptogenesis)

    # Get execution time from the performance statistics
    synapse_time = stats.total_tt
    print(f"Synaptogenesis execution time (from profiler): {synapse_time:.6f} seconds")

    # Get synapse count
    synapse_count = embryo.get_development_statistics()["total_synapses"]
    print(f"Created {synapse_count} synapses")

    # Calculate performance metrics
    if synapse_count > 0:
        synapses_per_second = synapse_count / synapse_time
        print(f"Performance: {synapses_per_second:.1f} synapses/second")
        print(
            f"Average time per synapse: {synapse_time * 1000000 / synapse_count:.2f} μs"
        )

    # Print profiling information
    print("\nTop 10 function calls by cumulative time:")
    print("-" * 80)
    print("{:<40} {:>10} {:>10} {:>10}".format("Function", "Calls", "Time", "Per Call"))
    print("-" * 80)

    for func, (calls, _, cum_time, _, _) in list(stats.stats.items())[:10]:
        func_name = f"{func[2]}:{func[1]}" if func[2] != "~" else func[2]
        if len(func_name) > 40:
            func_name = "..." + func_name[-37:]
        print(
            "{:<40} {:>10d} {:>10.6f} {:>10.6f}".format(
                func_name, calls, cum_time, cum_time / calls if calls > 0 else 0
            )
        )

    total_end = time.time()
    print(f"\nTotal test time: {total_end - total_start:.6f} seconds")

    # No assertions needed as this is a performance benchmark


if __name__ == "__main__":
    # Allow running this test file directly
    pytest.main(["-xvs", __file__])
