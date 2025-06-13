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

Example demonstrating the use of different array backends with ConnectomeManagerGPU.

This script creates a simple neural network and compares the performance of
different array backends (NumPy, PyTorch, CuPy, WebGPU) for common operations.
"""

import argparse
import logging
import time
from typing import Dict

import numpy as np

from feagi.bdu.connectome_manager_gpu import ConnectomeManagerGPU
from feagi.bdu.models.array_backend import BackendType

# Optional imports for WebGPU integration
try:
    from feagi.bdu.webgpu_integration import WEBGPU_AVAILABLE, ConnectomeManagerWebGPU
except ImportError:
    WEBGPU_AVAILABLE = False

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def create_simple_network(
    backend_type: BackendType, neuron_count: int = 10000, synapse_density: float = 0.01
) -> ConnectomeManagerGPU:
    """Create a simple neural network using the specified backend.

    Args:
        backend_type: Array backend to use
        neuron_count: Number of neurons in the network
        synapse_density: Fraction of possible connections that exist

    Returns:
        ConnectomeManagerGPU instance
    """
    logger.info(
        f"Creating network with {neuron_count} neurons using {backend_type.value} backend"
    )

    # Create connectome manager
    connectome = ConnectomeManagerGPU(max_neurons=neuron_count, backend=backend_type)

    # Add neurons
    logger.info("Adding neurons...")
    neuron_ids = connectome.add_neurons(neuron_count)

    # Set neuron properties using batch operations
    logger.info("Setting neuron properties...")
    connectome.batch_update_neuron_properties(
        neuron_ids=neuron_ids, property_name="threshold", values=1.0
    )

    connectome.batch_update_neuron_properties(
        neuron_ids=neuron_ids, property_name="decay_rate", values=0.9
    )

    # Create random synapses
    logger.info("Creating synapses...")
    synapse_count = int(neuron_count * neuron_count * synapse_density)

    # Create batches to avoid memory issues
    batch_size = 10000
    for i in range(0, synapse_count, batch_size):
        batch_count = min(batch_size, synapse_count - i)

        pre_neurons = np.random.choice(neuron_ids, batch_count)
        post_neurons = np.random.choice(neuron_ids, batch_count)
        weights = np.random.uniform(0.1, 0.5, batch_count)

        connectome.batch_add_synapses(
            pre_neurons=pre_neurons.tolist(),
            post_neurons=post_neurons.tolist(),
            weights=weights.tolist(),
        )

    logger.info(
        f"Created network with {connectome.neuron_count} neurons and {connectome.synapse_count} synapses"
    )
    return connectome


def benchmark_operations(
    connectome: ConnectomeManagerGPU, iterations: int = 10
) -> Dict[str, float]:
    """Benchmark common operations on the connectome.

    Args:
        connectome: ConnectomeManagerGPU instance
        iterations: Number of iterations to run each benchmark

    Returns:
        Dictionary of operation names and average times
    """
    results = {}

    # Benchmark membrane potential updates
    logger.info("Benchmarking membrane potential updates...")
    start_time = time.time()
    for _ in range(iterations):
        connectome.update_membrane_potentials(decay_factor=0.9)
    end_time = time.time()
    results["update_membrane_potentials"] = (end_time - start_time) / iterations

    # Benchmark firing propagation
    logger.info("Benchmarking firing propagation...")
    # Set random neurons to above threshold to simulate firing
    neuron_ids = list(connectome.neuron_id_to_index.keys())
    fire_count = int(len(neuron_ids) * 0.01)  # 1% firing rate
    fire_neurons = np.random.choice(neuron_ids, fire_count, replace=False)

    # Set membrane potentials to trigger firing
    connectome.batch_update_neuron_properties(
        neuron_ids=fire_neurons.tolist(),
        property_name="membrane_potential",
        values=2.0,  # Above threshold
    )

    start_time = time.time()
    for _ in range(iterations):
        # Find neurons that should fire
        firing_neurons = connectome.find_neurons_above_threshold()

        # Process firing
        if firing_neurons:
            connectome.process_firing_neurons(firing_neurons)
    end_time = time.time()
    results["process_firing"] = (end_time - start_time) / iterations

    # Benchmark property batch update
    logger.info("Benchmarking batch property updates...")
    start_time = time.time()
    for _ in range(iterations):
        connectome.batch_update_neuron_properties(
            neuron_ids=neuron_ids[:1000],
            property_name="threshold",
            values=np.random.uniform(0.5, 1.5, 1000).tolist(),
        )
    end_time = time.time()
    results["batch_update_properties"] = (end_time - start_time) / iterations

    return results


def benchmark_webgpu(
    connectome: ConnectomeManagerGPU, iterations: int = 10
) -> Dict[str, float]:
    """Benchmark WebGPU-accelerated operations.

    Args:
        connectome: ConnectomeManagerGPU instance
        iterations: Number of iterations to run each benchmark

    Returns:
        Dictionary of operation names and average times
    """
    if not WEBGPU_AVAILABLE:
        logger.warning("WebGPU not available, skipping benchmark")
        return {"webgpu_update": float("nan")}

    try:
        # Create WebGPU wrapper
        logger.info("Initializing WebGPU integration...")
        webgpu_connectome = ConnectomeManagerWebGPU(connectome)

        # Benchmark membrane potential updates
        logger.info("Benchmarking WebGPU membrane potential updates...")
        start_time = time.time()
        for _ in range(iterations):
            webgpu_connectome.update_membrane_potentials()
        end_time = time.time()

        return {"webgpu_update": (end_time - start_time) / iterations}

    except Exception as e:
        logger.error(f"Error during WebGPU benchmark: {e}")
        return {"webgpu_update": float("nan")}


def compare_backends(
    neuron_count: int = 10000, iterations: int = 5, synapse_density: float = 0.01
) -> Dict[str, Dict[str, float]]:
    """Compare different backends for common operations.

    Args:
        neuron_count: Number of neurons in the network
        iterations: Number of iterations to run each benchmark
        synapse_density: Fraction of possible connections that exist

    Returns:
        Dictionary of backend names and their benchmark results
    """
    backends_to_test = [BackendType.NUMPY]

    # Check which optional backends are available
    try:
        import torch

        if torch.cuda.is_available():
            backends_to_test.append(BackendType.PYTORCH)
    except ImportError:
        pass

    try:
        import cupy

        backends_to_test.append(BackendType.CUPY)
    except ImportError:
        pass

    results = {}

    for backend_type in backends_to_test:
        try:
            logger.info(f"\n===== Testing {backend_type.value} backend =====")

            # Create network
            connectome = create_simple_network(
                backend_type, neuron_count, synapse_density
            )

            # Benchmark operations
            backend_results = benchmark_operations(connectome, iterations)

            # If using CPU backend, also benchmark WebGPU integration
            if backend_type == BackendType.NUMPY and WEBGPU_AVAILABLE:
                webgpu_results = benchmark_webgpu(connectome, iterations)
                backend_results.update(webgpu_results)

            results[backend_type.value] = backend_results

        except Exception as e:
            logger.error(f"Error testing {backend_type.value} backend: {e}")
            results[backend_type.value] = {"error": str(e)}

    return results


def print_results(results: Dict[str, Dict[str, float]]):
    """Print benchmark results in a formatted table.

    Args:
        results: Dictionary of backend names and their benchmark results
    """
    # Find all unique operations across all backends
    operations = set()
    for backend_results in results.values():
        operations.update(backend_results.keys())
    operations = sorted(operations)

    # Print header
    print("\n===== Benchmark Results (seconds) =====")
    header = "Operation".ljust(30)
    for backend in results.keys():
        header += f"{backend}".ljust(15)
    print(header)
    print("-" * (30 + 15 * len(results)))

    # Print results for each operation
    for op in operations:
        row = op.ljust(30)
        for backend, backend_results in results.items():
            if op in backend_results:
                row += f"{backend_results[op]:.6f}".ljust(15)
            else:
                row += "N/A".ljust(15)
        print(row)


def main():
    """Run the backend comparison example."""
    parser = argparse.ArgumentParser(description="Compare array backends for FEAGI")
    parser.add_argument(
        "--neuron-count",
        type=int,
        default=10000,
        help="Number of neurons in the network",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=5,
        help="Number of iterations to run each benchmark",
    )
    parser.add_argument(
        "--synapse-density",
        type=float,
        default=0.01,
        help="Fraction of possible connections that exist",
    )

    args = parser.parse_args()

    # Run comparison
    results = compare_backends(
        neuron_count=args.neuron_count,
        iterations=args.iterations,
        synapse_density=args.synapse_density,
    )

    # Print results
    print_results(results)


if __name__ == "__main__":
    main()
