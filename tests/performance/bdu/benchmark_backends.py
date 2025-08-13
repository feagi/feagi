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

"""Benchmark script for comparing performance of different array backends.

This script compares the performance of different array backends (NumPy, PyTorch, CuPy, WebGPU)
for common operations in FEAGI, including neuron updates and synaptic propagation.
"""

import argparse
import logging
import time
from typing import Any, Dict, List

import matplotlib.pyplot as plt
import numpy as np

from feagi.bdu.models.array_backend import ArrayBackend, BackendType

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configure backends to test
BACKENDS_TO_TEST = [BackendType.NUMPY]

# Check which optional backends are available
try:
    import torch

    if torch.cuda.is_available():
        BACKENDS_TO_TEST.append(BackendType.PYTORCH)
except ImportError:
    pass

try:
    import cupy

    BACKENDS_TO_TEST.append(BackendType.CUPY)
except ImportError:
    pass

try:
    import wgpu

    BACKENDS_TO_TEST.append(BackendType.WEBGPU)
except ImportError:
    pass


class BenchmarkConfig:
    """Configuration for benchmarks."""

    def __init__(
        self,
        neuron_counts: List[int] = None,
        synapse_density: float = 0.01,
        firing_rate: float = 0.01,
        iterations: int = 10,
        warmup: int = 2,
    ):
        """Initialize benchmark configuration.

        Args:
            neuron_counts: List of neuron counts to benchmark
            synapse_density: Fraction of possible connections that exist
            firing_rate: Fraction of neurons that fire each timestep
            iterations: Number of iterations to run each benchmark
            warmup: Number of warmup iterations before timing
        """
        self.neuron_counts = neuron_counts or [1000, 10000, 100000, 1000000]
        self.synapse_density = synapse_density
        self.firing_rate = firing_rate
        self.iterations = iterations
        self.warmup = warmup


def setup_benchmark_data(
    config: BenchmarkConfig, neuron_count: int, backend: ArrayBackend
) -> Dict[str, Any]:
    """Set up test data for benchmarking.

    Args:
        config: Benchmark configuration
        neuron_count: Number of neurons to simulate
        backend: Array backend to use

    Returns:
        Dictionary of test data arrays
    """
    # Calculate number of synapses based on density
    synapse_count = int(neuron_count * neuron_count * config.synapse_density)

    # Create test data
    data = {
        # Neuron data
        "membrane_potentials": backend.zeros((neuron_count,), dtype=np.float32),
        "resting_potentials": backend.zeros((neuron_count,), dtype=np.float32),
        "thresholds": backend.ones((neuron_count,), dtype=np.float32),
        "refractory_periods": backend.ones((neuron_count,), dtype=np.int32),
        "refractory_counters": backend.zeros((neuron_count,), dtype=np.int32),
        "is_active": backend.zeros((neuron_count,), dtype=np.bool_),
        "valid_mask": backend.ones((neuron_count,), dtype=np.bool_),
        # Parameters
        "decay_factor": 0.9,
        # Firing data
        "fire_count": int(neuron_count * config.firing_rate),
    }

    # Generate random firing neuron indices
    fire_indices = np.random.choice(neuron_count, data["fire_count"], replace=False)
    data["fire_indices"] = backend.array(fire_indices, dtype=np.int32)

    # Generate random synapse connectivity (CSR format)
    pre_neurons = np.random.randint(0, neuron_count, synapse_count)
    post_neurons = np.random.randint(0, neuron_count, synapse_count)
    weights = np.random.uniform(0.1, 0.5, synapse_count).astype(np.float32)

    # Convert to CSR format
    try:
        # Try to create CSR matrix directly
        sparse_mat = backend.sparse_csr(
            weights,
            post_neurons,
            np.concatenate(
                [[0], np.cumsum(np.bincount(pre_neurons, minlength=neuron_count))]
            ),
            (neuron_count, neuron_count),
        )
        data["synapses"] = sparse_mat
    except (ValueError, AttributeError, NotImplementedError):
        # Fallback to components if CSR not supported
        data["synapse_weights"] = backend.array(weights)
        data["synapse_pre"] = backend.array(pre_neurons, dtype=np.int32)
        data["synapse_post"] = backend.array(post_neurons, dtype=np.int32)

    return data


def benchmark_neuron_update(
    config: BenchmarkConfig, data: Dict[str, Any], backend: ArrayBackend
) -> float:
    """Benchmark neuron membrane potential update and thresholding operation.

    Args:
        config: Benchmark configuration
        data: Test data
        backend: Array backend

    Returns:
        Average time per iteration in seconds
    """
    # Setup for benchmark
    membrane = data["membrane_potentials"]
    thresholds = data["thresholds"]
    refractory = data["refractory_counters"]
    is_active = data["is_active"]
    valid_mask = data["valid_mask"]
    decay = data["decay_factor"]

    # Convert to numpy for comparison if needed
    membrane_np = backend.to_numpy(membrane)
    thresholds_np = backend.to_numpy(thresholds)
    refractory_np = backend.to_numpy(refractory)
    is_active_np = backend.to_numpy(is_active)
    valid_np = backend.to_numpy(valid_mask)

    # Warmup
    for _ in range(config.warmup):
        # Decay membrane potentials
        membrane_np = membrane_np * decay

        # Determine which neurons fire
        firing_mask = (membrane_np >= thresholds_np) & (refractory_np == 0) & valid_np

        # Reset membrane potentials for firing neurons
        membrane_np[firing_mask] = 0.0

        # Set refractory periods for firing neurons
        refractory_np[firing_mask] = 1

        # Mark active neurons
        is_active_np[firing_mask] = True

    # Time iterations
    total_time = 0.0
    for i in range(config.iterations):
        start_time = time.time()

        # Perform operations using backend
        # Decay membrane potentials
        membrane = backend.array(membrane_np * decay)

        # Determine which neurons fire
        firing_mask = (membrane_np >= thresholds_np) & (refractory_np == 0) & valid_np

        # Reset membrane potentials for firing neurons
        membrane_np[firing_mask] = 0.0

        # Set refractory periods for firing neurons
        refractory_np[firing_mask] = 1

        # Mark active neurons
        is_active_np[firing_mask] = True

        backend.synchronize()  # Ensure operations are complete

        end_time = time.time()
        total_time += end_time - start_time

    return total_time / config.iterations


def benchmark_synapse_propagation(
    config: BenchmarkConfig, data: Dict[str, Any], backend: ArrayBackend
) -> float:
    """Benchmark synaptic propagation operation.

    Args:
        config: Benchmark configuration
        data: Test data
        backend: Array backend

    Returns:
        Average time per iteration in seconds
    """
    # Setup for benchmark
    membrane = data["membrane_potentials"]
    fire_indices = data["fire_indices"]

    # Check which format synapses are in
    if "synapses" in data:
        # CSR format
        synapses = data["synapses"]
        use_csr = True
    else:
        # Component arrays
        weights = data["synapse_weights"]
        pre = data["synapse_pre"]
        post = data["synapse_post"]
        use_csr = False

    # Convert to numpy for comparison
    membrane_np = backend.to_numpy(membrane)
    if use_csr:
        # For sparse matrix format
        if hasattr(synapses, "toarray"):  # scipy.sparse
            from scipy import sparse

            synapse_matrix = synapses
        else:
            # Convert to numpy compatible format
            data_np = backend.to_numpy(
                synapses.data
                if hasattr(synapses, "data")
                else backend.to_numpy(data["synapse_weights"])
            )
            indices_np = backend.to_numpy(
                synapses.indices
                if hasattr(synapses, "indices")
                else backend.to_numpy(data["synapse_post"])
            )
            indptr_np = backend.to_numpy(
                synapses.indptr if hasattr(synapses, "indptr") else None
            )

            if indptr_np is not None:
                from scipy import sparse

                synapse_matrix = sparse.csr_matrix((data_np, indices_np, indptr_np))
            else:
                use_csr = False
                weights_np = data_np
                pre_np = np.zeros_like(indices_np)  # Not available
                post_np = indices_np
    else:
        # Component arrays
        weights_np = backend.to_numpy(weights)
        pre_np = backend.to_numpy(pre)
        post_np = backend.to_numpy(post)

    fire_indices_np = backend.to_numpy(fire_indices)

    # Warmup
    for _ in range(config.warmup):
        if use_csr:
            # Use sparse matrix for propagation
            for idx in fire_indices_np:
                # Get outgoing connections for this neuron
                row = synapse_matrix.getrow(idx)

                # Apply weights to post-synaptic neurons
                targets = row.indices
                weights = row.data

                if len(targets) > 0:
                    membrane_np[targets] += weights
        else:
            # Use component arrays
            for idx in fire_indices_np:
                # Find outgoing connections
                mask = pre_np == idx
                targets = post_np[mask]
                connection_weights = weights_np[mask]

                if len(targets) > 0:
                    for t, w in zip(targets, connection_weights):
                        membrane_np[t] += w

    # Time iterations
    total_time = 0.0
    for i in range(config.iterations):
        # Reset membrane potentials for clean measurement
        membrane_np.fill(0.0)
        membrane = backend.array(membrane_np)

        start_time = time.time()

        if use_csr:
            # Use sparse matrix for propagation
            for idx in fire_indices_np:
                # Get outgoing connections for this neuron
                row = synapse_matrix.getrow(idx)

                # Apply weights to post-synaptic neurons
                targets = row.indices
                weights = row.data

                if len(targets) > 0:
                    membrane_np[targets] += weights
        else:
            # Use component arrays
            for idx in fire_indices_np:
                # Find outgoing connections
                mask = pre_np == idx
                targets = post_np[mask]
                connection_weights = weights_np[mask]

                if len(targets) > 0:
                    for t, w in zip(targets, connection_weights):
                        membrane_np[t] += w

        membrane = backend.array(membrane_np)

        backend.synchronize()  # Ensure operations are complete

        end_time = time.time()
        total_time += end_time - start_time

    return total_time / config.iterations


def run_benchmarks(config: BenchmarkConfig) -> Dict[str, Dict[str, List[float]]]:
    """Run all benchmarks for all backends and neuron counts.

    Args:
        config: Benchmark configuration

    Returns:
        Dictionary of results for each benchmark, backend, and neuron count
    """
    results = {
        "neuron_update": {backend.value: [] for backend in BACKENDS_TO_TEST},
        "synapse_propagation": {backend.value: [] for backend in BACKENDS_TO_TEST},
    }

    for neuron_count in config.neuron_counts:
        logger.info(f"Benchmarking with {neuron_count} neurons")

        for backend_type in BACKENDS_TO_TEST:
            logger.info(f"  Using {backend_type.value} backend")

            # Initialize backend
            backend = ArrayBackend(backend_type)

            # Skip if too many neurons for this backend
            if (
                backend_type in [BackendType.CUPY, BackendType.WEBGPU]
                and neuron_count > 1_000_000
            ):
                logger.info(
                    f"    Skipping {backend_type.value} for {neuron_count} neurons (too large)"
                )
                results["neuron_update"][backend_type.value].append(float("nan"))
                results["synapse_propagation"][backend_type.value].append(float("nan"))
                continue

            # Set up benchmark data
            try:
                data = setup_benchmark_data(config, neuron_count, backend)

                # Neuron update benchmark
                logger.info("    Running neuron update benchmark")
                neuron_time = benchmark_neuron_update(config, data, backend)
                results["neuron_update"][backend_type.value].append(neuron_time)
                logger.info(f"    Neuron update: {neuron_time:.6f} seconds")

                # Synapse propagation benchmark
                logger.info("    Running synapse propagation benchmark")
                synapse_time = benchmark_synapse_propagation(config, data, backend)
                results["synapse_propagation"][backend_type.value].append(synapse_time)
                logger.info(f"    Synapse propagation: {synapse_time:.6f} seconds")

            except Exception as e:
                logger.error(
                    f"    Error benchmarking {backend_type.value} with {neuron_count} neurons: {e}"
                )
                results["neuron_update"][backend_type.value].append(float("nan"))
                results["synapse_propagation"][backend_type.value].append(float("nan"))

    return results


def plot_results(config: BenchmarkConfig, results: Dict[str, Dict[str, List[float]]]):
    """Plot benchmark results.

    Args:
        config: Benchmark configuration
        results: Benchmark results
    """
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    # Plot neuron update results
    neuron_data = results["neuron_update"]
    for backend, times in neuron_data.items():
        ax1.plot(config.neuron_counts, times, marker="o", label=backend)

    ax1.set_xlabel("Number of Neurons")
    ax1.set_ylabel("Time (seconds)")
    ax1.set_title("Neuron Update Performance")
    ax1.set_xscale("log")
    ax1.set_yscale("log")
    ax1.grid(True)
    ax1.legend()

    # Plot synapse propagation results
    synapse_data = results["synapse_propagation"]
    for backend, times in synapse_data.items():
        ax2.plot(config.neuron_counts, times, marker="o", label=backend)

    ax2.set_xlabel("Number of Neurons")
    ax2.set_ylabel("Time (seconds)")
    ax2.set_title("Synapse Propagation Performance")
    ax2.set_xscale("log")
    ax2.set_yscale("log")
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.savefig("backend_benchmark_results.png", dpi=300)
    plt.show()


def main():
    """Run the benchmarks."""
    parser = argparse.ArgumentParser(description="Benchmark array backends for FEAGI")
    parser.add_argument(
        "--neuron-counts",
        type=int,
        nargs="+",
        default=[1000, 10000, 100000, 1000000],
        help="List of neuron counts to benchmark",
    )
    parser.add_argument(
        "--synapse-density",
        type=float,
        default=0.01,
        help="Fraction of possible connections that exist",
    )
    parser.add_argument(
        "--firing-rate",
        type=float,
        default=0.01,
        help="Fraction of neurons that fire each timestep",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=5,
        help="Number of iterations to run each benchmark",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=2,
        help="Number of warmup iterations before timing",
    )
    parser.add_argument(
        "--no-plot", action="store_true", help="Disable plotting of results"
    )

    args = parser.parse_args()

    # Create benchmark configuration
    config = BenchmarkConfig(
        neuron_counts=args.neuron_counts,
        synapse_density=args.synapse_density,
        firing_rate=args.firing_rate,
        iterations=args.iterations,
        warmup=args.warmup,
    )

    # Run benchmarks
    logger.info("Running benchmarks...")
    results = run_benchmarks(config)

    # Print results
    logger.info("Benchmark results:")
    for benchmark, backend_results in results.items():
        logger.info(f"  {benchmark}:")
        for backend, times in backend_results.items():
            logger.info(f"    {backend}: {times}")

    # Plot results
    if not args.no_plot:
        logger.info("Plotting results...")
        plot_results(config, results)


if __name__ == "__main__":
    main()
