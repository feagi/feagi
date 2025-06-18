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
Pytest benchmark tests for synapse creation performance.

This tests the performance of creating 1000 synapses of all supported types:
- Function morphologies (projector, memory, last_to_first, etc.)
- Vector morphologies (lateral connections, directional vectors)
- Pattern morphologies (spatial patterns, wildcards)
- All synapse types (EXCITATORY, INHIBITORY, MODULATORY, PLASTIC)
"""

import cProfile
import json
import logging
import os
import pstats
import sys
import tempfile
import time
from datetime import datetime
from typing import Dict, List, Tuple

import numpy as np
import pytest

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

# Add the project root to the path if needed
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
from feagi.bdu.synapse_array import SynapseType
from feagi.utils.config import FeagiConfig


@pytest.fixture
def config():
    """Create a FeagiConfig for testing."""
    with tempfile.TemporaryDirectory() as temp_dir:
        config_dict = {
            "connectome_path": temp_dir,
            "skip_memory_neurogenesis": True,
            "connectome.max_neurons": 10000,
            "connectome.max_synapses_per_neuron": 1000,
        }

        config = FeagiConfig()
        for key, value in config_dict.items():
            config.set(key, value)

        yield config


@pytest.fixture
def connectome_manager(config):
    """Create a ConnectomeManager for testing."""
    max_neurons = 10000
    max_synapses = config.get("connectome.max_synapses_per_neuron", 1000) * max_neurons
    return ConnectomeManager(
        config_or_max_neurons=max_neurons, max_synapses=max_synapses
    )


@pytest.fixture
def neuro_embryogenesis(connectome_manager):
    """Create a NeuroEmbryogenesis instance for testing."""
    return NeuroEmbryogenesis(connectome_manager)


def save_benchmark_results(test_name: str, results: Dict, detailed_results: Dict = None):
    """Save benchmark results to the logs folder with timestamp."""
    # Create logs directory if it doesn't exist
    logs_dir = os.path.join(project_root, "tests", "performance", "logs")
    os.makedirs(logs_dir, exist_ok=True)
    
    # Create timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Create summary data
    summary = {
        "timestamp": datetime.now().isoformat(),
        "test_name": test_name,
        "results": results,
        "system_info": {
            "python_version": sys.version,
            "platform": sys.platform,
        }
    }
    
    if detailed_results:
        summary["detailed_results"] = detailed_results
    
    # Save individual test result
    filename = f"synapse_benchmark_{test_name}_{timestamp}.json"
    filepath = os.path.join(logs_dir, filename)
    
    with open(filepath, 'w') as f:
        json.dump(summary, f, indent=2)
    
    print(f"📊 Benchmark results saved to: {filepath}")
    
    # Also append to master log file
    master_log = os.path.join(logs_dir, "synapse_benchmark_history.jsonl")
    with open(master_log, 'a') as f:
        f.write(json.dumps(summary) + '\n')
    
    return filepath


def cleanup_connectome(connectome_manager: ConnectomeManager):
    """Clean up the connectome by removing all areas and resetting state."""
    # Get all cortical area IDs
    area_ids = list(connectome_manager.cortical_areas.keys())
    
    # Delete all cortical areas (this will also delete neurons and synapses)
    for area_id in area_ids:
        try:
            connectome_manager.delete_cortical_area(area_id, delete_neurons=True)
        except Exception as e:
            print(f"Warning: Could not delete area {area_id}: {e}")
    
    # Clear cortical areas dict
    connectome_manager.cortical_areas.clear()
    
    # Reset brain regions
    if hasattr(connectome_manager, 'brain_regions'):
        connectome_manager.brain_regions.clear()
    
    # Reset cortical mapping
    if hasattr(connectome_manager, 'cortical_mapping'):
        connectome_manager.cortical_mapping.clear(preserve_core_areas=False)
    
    # Reset next cortical index
    if hasattr(connectome_manager, 'next_cortical_idx'):
        connectome_manager.next_cortical_idx = 1


def setup_test_neurons(connectome_manager: ConnectomeManager) -> Tuple[List[int], List[int]]:
    """Setup test neurons for synapse creation."""
    # Clean up first
    cleanup_connectome(connectome_manager)
    
    # Create two cortical areas for testing
    area1_id = connectome_manager.add_cortical_area(
        name="test_area_1", 
        dimensions=(10, 10, 10), 
        position=(0, 0, 0),
        area_type="test"
    )
    area2_id = connectome_manager.add_cortical_area(
        name="test_area_2", 
        dimensions=(10, 10, 10), 
        position=(10, 0, 0),
        area_type="test"
    )

    # Create neurons in both areas
    src_neurons = []
    dst_neurons = []

    # Create 500 neurons in each area
    for i in range(500):
        # Source neurons in area 1
        x, y, z = i % 10, (i // 10) % 10, i // 100
        src_neuron = connectome_manager.create_neuron(
            cortical_id=area1_id, position=(x, y, z)
        )
        src_neurons.append(src_neuron)

        # Destination neurons in area 2
        dst_neuron = connectome_manager.create_neuron(
            cortical_id=area2_id, position=(x, y, z)
        )
        dst_neurons.append(dst_neuron)

    return src_neurons, dst_neurons


def run_with_profiling(function, *args, **kwargs):
    """Run a function with cProfile to collect performance metrics."""
    profiler = cProfile.Profile()
    profiler.enable()

    result = function(*args, **kwargs)

    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats("cumtime")

    return result, stats


def benchmark_synapse_creation(
    connectome_manager: ConnectomeManager,
    src_neurons: List[int],
    dst_neurons: List[int],
    synapse_type: SynapseType,
    count: int = 1000,
    test_name: str = "synapse_creation"
) -> Dict[str, float]:
    """Benchmark synapse creation performance."""
    
    def create_synapses():
        """Create synapses for benchmarking."""
        created_synapses = []
        
        for i in range(count):
            src_idx = i % len(src_neurons)
            dst_idx = i % len(dst_neurons)
            
            # Vary parameters based on synapse type
            weight = 1.0
            is_plastic = False
            plasticity_coeff = 0.0
            plasticity_decay = 0.0
            
            if synapse_type == SynapseType.INHIBITORY:
                weight = -0.5
            elif synapse_type == SynapseType.MODULATORY:
                weight = 0.3
            elif synapse_type == SynapseType.PLASTIC:
                weight = 0.8
                is_plastic = True
                plasticity_coeff = 0.01
                plasticity_decay = 0.001
            
            synapse_id = connectome_manager.create_synapse(
                pre_neuron_id=src_neurons[src_idx],
                post_neuron_id=dst_neurons[dst_idx],
                weight=weight,
                synapse_type=synapse_type,
                is_plastic=is_plastic,
                plasticity_coeff=plasticity_coeff,
                plasticity_decay=plasticity_decay
            )
            created_synapses.append(synapse_id)
        
        return created_synapses

    # Run with profiling
    start_time = time.time()
    created_synapses, stats = run_with_profiling(create_synapses)
    end_time = time.time()

    # Calculate metrics
    total_time = end_time - start_time
    synapses_per_second = count / total_time if total_time > 0 else 0
    avg_time_per_synapse = (total_time * 1000000) / count  # microseconds

    print(f"\n{test_name} Performance:")
    print(f"  Created {len(created_synapses)} synapses")
    print(f"  Total time: {total_time:.6f} seconds")
    print(f"  Performance: {synapses_per_second:.1f} synapses/second")
    print(f"  Average time per synapse: {avg_time_per_synapse:.2f} μs")

    return {
        "count": len(created_synapses),
        "total_time": total_time,
        "synapses_per_second": synapses_per_second,
        "avg_time_per_synapse_us": avg_time_per_synapse,
        # Note: profiler_stats excluded from JSON serialization
    }


def benchmark_morphology_synapse_creation(
    connectome_manager: ConnectomeManager,
    src_neurons: List[int],
    dst_neurons: List[int],
    morphology_type: str,
    count: int = 1000,
    test_name: str = "morphology_synapse_creation"
) -> Dict[str, float]:
    """Benchmark synapse creation with different morphology patterns."""
    
    def create_morphology_synapses():
        """Create synapses with morphology-specific patterns."""
        created_synapses = []
        
        for i in range(count):
            src_idx = i % len(src_neurons)
            
            # Apply morphology-specific connection patterns
            if morphology_type == "vector":
                # Vector morphology: lateral connections (neighboring neurons)
                dst_idx = (src_idx + 1) % len(dst_neurons)
                weight = 1.0
            elif morphology_type == "pattern":
                # Pattern morphology: spatial pattern connections
                dst_idx = (src_idx * 2) % len(dst_neurons)
                weight = 0.8
            elif morphology_type == "function":
                # Function morphology: algorithmic connections (projector-like)
                dst_idx = (src_idx + len(dst_neurons) // 2) % len(dst_neurons)
                weight = 1.2
            else:
                # Default: one-to-one mapping
                dst_idx = src_idx % len(dst_neurons)
                weight = 1.0
            
            synapse_id = connectome_manager.create_synapse(
                pre_neuron_id=src_neurons[src_idx],
                post_neuron_id=dst_neurons[dst_idx],
                weight=weight,
                synapse_type=SynapseType.EXCITATORY
            )
            created_synapses.append(synapse_id)
        
        return created_synapses

    # Run with profiling
    start_time = time.time()
    created_synapses, stats = run_with_profiling(create_morphology_synapses)
    end_time = time.time()

    # Calculate metrics
    total_time = end_time - start_time
    synapses_per_second = count / total_time if total_time > 0 else 0
    avg_time_per_synapse = (total_time * 1000000) / count  # microseconds

    print(f"\n{test_name} Performance:")
    print(f"  Created {len(created_synapses)} synapses")
    print(f"  Total time: {total_time:.6f} seconds")
    print(f"  Performance: {synapses_per_second:.1f} synapses/second")
    print(f"  Average time per synapse: {avg_time_per_synapse:.2f} μs")

    return {
        "count": len(created_synapses),
        "total_time": total_time,
        "synapses_per_second": synapses_per_second,
        "avg_time_per_synapse_us": avg_time_per_synapse,
        "morphology_type": morphology_type,
        # Note: profiler_stats excluded from JSON serialization
    }


def test_excitatory_synapse_benchmark(connectome_manager):
    """Benchmark EXCITATORY synapse creation performance."""
    print("\n" + "="*80)
    print("EXCITATORY SYNAPSE BENCHMARK")
    print("="*80)
    
    src_neurons, dst_neurons = setup_test_neurons(connectome_manager)
    
    # Individual synapse creation
    individual_results = benchmark_synapse_creation(
        connectome_manager, src_neurons, dst_neurons, 
        SynapseType.EXCITATORY, count=1000, test_name="Individual EXCITATORY"
    )
    
    # Save results
    save_benchmark_results("excitatory", individual_results)
    
    # Performance assertions
    assert individual_results["count"] == 1000, "Should create exactly 1000 individual synapses"
    assert individual_results["synapses_per_second"] > 1000, "Should create at least 1000 synapses/second"


def test_inhibitory_synapse_benchmark(connectome_manager):
    """Benchmark INHIBITORY synapse creation performance."""
    print("\n" + "="*80)
    print("INHIBITORY SYNAPSE BENCHMARK")
    print("="*80)
    
    src_neurons, dst_neurons = setup_test_neurons(connectome_manager)
    
    # Individual synapse creation
    individual_results = benchmark_synapse_creation(
        connectome_manager, src_neurons, dst_neurons, 
        SynapseType.INHIBITORY, count=1000, test_name="Individual INHIBITORY"
    )
    
    # Save results
    save_benchmark_results("inhibitory", individual_results)
    
    # Performance assertions
    assert individual_results["count"] == 1000, "Should create exactly 1000 individual synapses"
    assert individual_results["synapses_per_second"] > 1000, "Should create at least 1000 synapses/second"


def test_modulatory_synapse_benchmark(connectome_manager):
    """Benchmark MODULATORY synapse creation performance."""
    print("\n" + "="*80)
    print("MODULATORY SYNAPSE BENCHMARK")
    print("="*80)
    
    src_neurons, dst_neurons = setup_test_neurons(connectome_manager)
    
    # Individual synapse creation
    individual_results = benchmark_synapse_creation(
        connectome_manager, src_neurons, dst_neurons, 
        SynapseType.MODULATORY, count=1000, test_name="Individual MODULATORY"
    )
    
    # Save results
    save_benchmark_results("modulatory", individual_results)
    
    # Performance assertions
    assert individual_results["count"] == 1000, "Should create exactly 1000 individual synapses"
    assert individual_results["synapses_per_second"] > 1000, "Should create at least 1000 synapses/second"


def test_plastic_synapse_benchmark(connectome_manager):
    """Benchmark PLASTIC synapse creation performance."""
    print("\n" + "="*80)
    print("PLASTIC SYNAPSE BENCHMARK")
    print("="*80)
    
    src_neurons, dst_neurons = setup_test_neurons(connectome_manager)
    
    # Individual synapse creation
    individual_results = benchmark_synapse_creation(
        connectome_manager, src_neurons, dst_neurons, 
        SynapseType.PLASTIC, count=1000, test_name="Individual PLASTIC"
    )
    
    # Save results
    save_benchmark_results("plastic", individual_results)
    
    # Performance assertions
    assert individual_results["count"] == 1000, "Should create exactly 1000 individual synapses"
    assert individual_results["synapses_per_second"] > 1000, "Should create at least 1000 synapses/second"


def test_morphology_types_benchmark(connectome_manager):
    """Benchmark all morphology types: Function, Vector, and Pattern."""
    print("\n" + "="*80)
    print("MORPHOLOGY TYPES BENCHMARK")
    print("="*80)
    
    src_neurons, dst_neurons = setup_test_neurons(connectome_manager)
    
    morphology_types = ["function", "vector", "pattern"]
    results = {}
    
    # Test each morphology type
    for morphology_type in morphology_types:
        print(f"\nTesting {morphology_type.upper()} morphology...")
        
        individual_results = benchmark_morphology_synapse_creation(
            connectome_manager, src_neurons, dst_neurons, 
            morphology_type, count=333, test_name=f"Individual {morphology_type.upper()}"
        )
        
        results[morphology_type] = individual_results
    
    # Performance summary
    print("\n" + "="*80)
    print("MORPHOLOGY PERFORMANCE SUMMARY")
    print("="*80)
    print(f"{'Morphology':<12} {'Rate (s/s)':<12} {'Avg Time (μs)':<15}")
    print("-" * 42)
    
    for morphology_type in morphology_types:
        result = results[morphology_type]
        print(f"{morphology_type.upper():<12} {result['synapses_per_second']:<12.1f} {result['avg_time_per_synapse_us']:<15.2f}")
    
    # Calculate total performance
    total_synapses = sum(result["count"] for result in results.values())
    total_time = sum(result["total_time"] for result in results.values())
    overall_rate = total_synapses / total_time if total_time > 0 else 0
    
    print(f"\nOVERALL MORPHOLOGY PERFORMANCE:")
    print(f"  Total synapses created: {total_synapses}")
    print(f"  Total time: {total_time:.6f} seconds")
    print(f"  Overall rate: {overall_rate:.1f} synapses/second")
    
    # Save results
    save_benchmark_results("morphology_types", {
        "individual_results": results,
        "total_synapses": total_synapses,
        "total_time": total_time,
        "overall_rate": overall_rate
    })
    
    # Performance assertions
    assert total_synapses == 999, "Should create exactly 999 total synapses (333*3)"
    assert overall_rate > 100000, "Overall rate should exceed 100,000 synapses/second"
    
    for morphology_type, result in results.items():
        assert result["synapses_per_second"] > 50000, f"{morphology_type} should exceed 50,000 synapses/second"


def test_all_synapse_types_benchmark(connectome_manager):
    """Comprehensive benchmark of all synapse types."""
    print("\n" + "="*80)
    print("COMPREHENSIVE SYNAPSE TYPE BENCHMARK")
    print("="*80)
    
    src_neurons, dst_neurons = setup_test_neurons(connectome_manager)
    
    synapse_types = [
        (SynapseType.EXCITATORY, "EXCITATORY"),
        (SynapseType.INHIBITORY, "INHIBITORY"),
        (SynapseType.MODULATORY, "MODULATORY"),
        (SynapseType.PLASTIC, "PLASTIC")
    ]
    
    results = {}
    
    # Test each synapse type
    for synapse_type, type_name in synapse_types:
        print(f"\nTesting {type_name} synapses...")
        
        individual_results = benchmark_synapse_creation(
            connectome_manager, src_neurons, dst_neurons, 
            synapse_type, count=250, test_name=f"Individual {type_name}"
        )
        
        results[type_name] = {
            "individual": individual_results,
        }
    
    # Performance summary
    print("\n" + "="*80)
    print("PERFORMANCE SUMMARY")
    print("="*80)
    print(f"{'Type':<12} {'Individual (s/s)':<15}")
    print("-" * 30)
    
    for type_name in ["EXCITATORY", "INHIBITORY", "MODULATORY", "PLASTIC"]:
        individual_rate = results[type_name]["individual"]["synapses_per_second"]
        print(f"{type_name:<12} {individual_rate:<15.1f}")
    
    # Calculate total performance
    total_synapses = sum(
        results[type_name]["individual"]["count"]
        for type_name in results
    )
    total_time = sum(
        results[type_name]["individual"]["total_time"]
        for type_name in results
    )
    overall_rate = total_synapses / total_time if total_time > 0 else 0
    
    print(f"\nOVERALL PERFORMANCE:")
    print(f"  Total synapses created: {total_synapses}")
    print(f"  Total time: {total_time:.6f} seconds")
    print(f"  Overall rate: {overall_rate:.1f} synapses/second")
    
    # Save comprehensive results
    save_benchmark_results("comprehensive_synapse_types", {
        "individual_results": results,
        "total_synapses": total_synapses,
        "total_time": total_time,
        "overall_rate": overall_rate
    })
    
    # Performance assertions
    assert total_synapses == 1000, "Should create exactly 1000 total synapses (250*4)"
    assert overall_rate > 5000, "Overall rate should exceed 5000 synapses/second"


def test_large_self_mapping_vector_benchmark(connectome_manager, neuro_embryogenesis):
    """
    Benchmark the performance issue identified in test_mode_1 with iv00_C self-mapping.
    
    This test reproduces the 49-second delay caused by processing a 64x64x3 area
    with test_vector_1_1_1 morphology (vector [1,1,1]).
    """
    print("\n" + "="*80)
    print("LARGE SELF-MAPPING VECTOR BENCHMARK")
    print("="*80)
    print("Reproducing the iv00_C test_vector_1_1_1 performance issue...")
    
    # Create a large vision-like area similar to iv00_C (64x64x3 = 12,288 neurons)
    large_area_id = connectome_manager.add_cortical_area(
        name="test_vision_area",
        dimensions=(64, 64, 3),  # Same as iv00_C in test_genome_1.json
        position=(0, 0, 0),
        area_type="test"
    )
    
    print(f"Created large area: {large_area_id} with dimensions 64x64x3 = {64*64*3:,} potential neurons")
    
    # Create neurons in the area (simulate realistic neuron density)
    positions = []
    for x in range(0, 64, 2):  # Every 2nd position to simulate realistic density
        for y in range(0, 64, 2):
            for z in range(3):
                positions.append((x, y, z))
    
    neuron_ids = connectome_manager.batch_create_neurons(large_area_id, positions)
    print(f"Created {len(neuron_ids):,} neurons in large area")
    
    # Create the problematic self-mapping with vector [1,1,1]
    mapping_data = {
        large_area_id: {
            large_area_id: [  # Self-mapping like iv00_C -> iv00_C
                {
                    "morphology_id": "test_vector_1_1_1",
                    "morphology_scalar": [1, 1, 1],
                    "postSynapticCurrent_multiplier": 1.0,
                    "plasticity_flag": False,
                    "plasticity_constant": 1.0,
                    "ltp_multiplier": 1.0,
                    "ltd_multiplier": 1.0,
                }
            ]
        }
    }
    
    # Set up the test_vector_1_1_1 morphology in the genome
    if not hasattr(neuro_embryogenesis, 'genome') or neuro_embryogenesis.genome is None:
        neuro_embryogenesis.genome = {}
    
    if 'neuron_morphologies' not in neuro_embryogenesis.genome:
        neuro_embryogenesis.genome['neuron_morphologies'] = {}
    
    neuro_embryogenesis.genome['neuron_morphologies']['test_vector_1_1_1'] = {
        "type": "vectors",
        "parameters": {
            "vectors": [[1, 1, 1]]  # The problematic vector causing delays
        },
        "morphology_name": "test_vector_1_1_1",
        "class": "custom"
    }
    
    print(f"Testing self-mapping with vector [1,1,1] on {len(neuron_ids):,} neurons...")
    print("This should demonstrate the O(N) performance bottleneck...")
    
    # Benchmark the problematic operation
    start_time = time.time()
    success = neuro_embryogenesis.update_cortical_mapping(mapping_data)
    end_time = time.time()
    
    total_time = end_time - start_time
    
    print(f"\n🔍 PERFORMANCE ANALYSIS:")
    print(f"  Operation completed: {'✅ SUCCESS' if success else '❌ FAILED'}")
    print(f"  Total time: {total_time:.3f} seconds")
    print(f"  Neurons processed: {len(neuron_ids):,}")
    print(f"  Time per neuron: {(total_time / len(neuron_ids) * 1000):.3f} ms")
    
    # Get synapse count
    final_synapse_count = connectome_manager.get_synapse_count()
    synapses_created = final_synapse_count
    
    print(f"  Synapses created: {synapses_created:,}")
    if synapses_created > 0:
        print(f"  Synapses per second: {synapses_created / total_time:.0f}")
    
    # Performance expectations
    if total_time > 10.0:
        print(f"\n⚠️  PERFORMANCE ISSUE CONFIRMED:")
        print(f"    Taking {total_time:.1f}s for {len(neuron_ids):,} neurons is too slow!")
        print(f"    Expected: < 1s for vectorized operations")
        print(f"    Root cause: O(N) individual neuron processing instead of vectorized batch operations")
    elif total_time > 1.0:
        print(f"\n🟡 MODERATE PERFORMANCE CONCERN:")
        print(f"    {total_time:.1f}s is acceptable but could be optimized")
    else:
        print(f"\n✅ GOOD PERFORMANCE:")
        print(f"    {total_time:.1f}s is within acceptable range")
    
    print("="*80)
    
    # Cleanup
    cleanup_connectome(connectome_manager)
    
    assert success, "Large self-mapping should succeed"
    assert total_time < 60.0, f"Should complete within 60s, took {total_time:.1f}s"


if __name__ == "__main__":
    # Allow running this test file directly
    pytest.main(["-xvs", __file__]) 