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
Multi-GPU Support Example

This example demonstrates how to use the multi-GPU support in the FEAGI BDU.
It creates a multi-GPU connectome, adds some neurons and synapses,
and runs a simulation across multiple GPUs.
"""

import argparse
import logging
import os
import sys
import time

import numpy as np

# Add the parent directory to the path so we can import feagi
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from feagi.bdu.connectome_manager_gpu import ConnectomeManagerGPU
from feagi.bdu.models.array_backend import ArrayBackend, BackendType
from feagi.bdu.models.cortical_area import CorticalArea, CorticalFunction
from feagi.bdu.multi_gpu import MultiGPUConfig, PartitionMethod

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def create_test_brain(connectome, num_neurons=10_000, density=0.01):
    """Create a test brain with the given number of neurons and connection density.

    Args:
        connectome: ConnectomeManagerGPU instance
        num_neurons: Number of neurons to create
        density: Connection density (fraction of possible connections)
    """
    logger.info(
        f"Creating test brain with {num_neurons} neurons and {density:.2%} connection density"
    )

    # Create cortical areas
    areas = []
    for i in range(4):
        area = CorticalArea(
            cortical_id=f"area_{i}",
            dimensions=[50, 50, 10],
            position=[i * 60, 0, 0],
            cortical_function=(
                CorticalFunction.SENSORY if i == 0 else CorticalFunction.ASSOCIATION
            ),
        )
        connectome.add_cortical_area(area)
        areas.append(area)

    # Add neurons to each area
    neurons_per_area = num_neurons // len(areas)
    neuron_ids = []

    for i, area in enumerate(areas):
        for j in range(neurons_per_area):
            # Create a position within the area's dimensions
            position = [
                np.random.randint(0, area.dimensions[0]),
                np.random.randint(0, area.dimensions[1]),
                np.random.randint(0, area.dimensions[2]),
            ]

            # Add neuron
            neuron_id = connectome.add_neuron(
                area_id=area.cortical_id,
                position=position,
                threshold=0.5,
                refractory_period=2,
                decay_rate=0.1,
                membrane_potential=0.0,
                resting_potential=0.0,
            )
            neuron_ids.append(neuron_id)

    # Add synapses between neurons
    total_possible_connections = len(neuron_ids) * (len(neuron_ids) - 1)
    num_connections = int(total_possible_connections * density)

    logger.info(
        f"Adding {num_connections} synapses ({density:.2%} of possible connections)"
    )

    # For each source neuron, connect to multiple targets
    connections_per_neuron = num_connections // len(neuron_ids)

    for source_id in neuron_ids:
        # Select random target neurons (different from source)
        target_pool = [nid for nid in neuron_ids if nid != source_id]
        if not target_pool:
            continue

        targets = np.random.choice(
            target_pool,
            size=min(connections_per_neuron, len(target_pool)),
            replace=False,
        )

        # Add synapses
        for target_id in targets:
            weight = np.random.uniform(0.1, 1.0)
            connectome.add_synapse(source_id, target_id, weight)

    # Add some connectivity between areas for signal propagation
    for i in range(len(areas) - 1):
        source_area = areas[i]
        target_area = areas[i + 1]

        # Get neurons from each area
        source_neurons = [
            nid
            for nid in neuron_ids
            if connectome.get_area_for_neuron(nid) == source_area.cortical_id
        ]
        target_neurons = [
            nid
            for nid in neuron_ids
            if connectome.get_area_for_neuron(nid) == target_area.cortical_id
        ]

        # Connect a percentage of neurons between areas
        num_inter_area_connections = int(len(source_neurons) * 0.2)  # 20% connectivity

        for _ in range(num_inter_area_connections):
            source_id = np.random.choice(source_neurons)
            target_id = np.random.choice(target_neurons)
            weight = np.random.uniform(0.5, 1.0)  # Stronger weights between areas
            connectome.add_synapse(source_id, target_id, weight)

    # Get input neurons (from the first area)
    input_neurons = [
        nid
        for nid in neuron_ids
        if connectome.get_area_for_neuron(nid) == areas[0].cortical_id
    ]

    return neuron_ids, input_neurons


def run_simulation(connectome, input_neurons, num_timesteps=100):
    """Run a simulation on the connectome with the given input neurons.

    Args:
        connectome: ConnectomeManagerGPU instance
        input_neurons: List of input neuron IDs
        num_timesteps: Number of timesteps to simulate
    """
    logger.info(f"Running simulation for {num_timesteps} timesteps")

    # Statistics
    total_fired = 0
    start_time = time.time()

    # Run simulation
    for t in range(num_timesteps):
        # Stimulate random input neurons
        if t % 5 == 0:  # Every 5 timesteps
            num_to_stimulate = min(20, len(input_neurons))
            stimulate_neurons = np.random.choice(
                input_neurons, size=num_to_stimulate, replace=False
            )

            # Set membrane potentials above threshold
            for neuron_id in stimulate_neurons:
                idx = connectome.neuron_id_to_index[neuron_id]
                connectome.neuron_array.membrane_potentials[idx] = (
                    1.0  # Well above threshold
                )
                connectome.active_neurons[idx] = True

        # Update membrane potentials
        fired = connectome.update_membrane_potentials(t)
        total_fired += len(fired)

        if t % 10 == 0:
            logger.info(f"Timestep {t}: {len(fired)} neurons fired")

    # Calculate statistics
    elapsed_time = time.time() - start_time
    logger.info("Simulation complete")
    logger.info(f"  Total time: {elapsed_time:.2f} seconds")
    logger.info(f"  Total fired: {total_fired} neurons")
    logger.info(
        f"  Average firing rate: {total_fired / num_timesteps:.2f} neurons/timestep"
    )
    logger.info(f"  Performance: {num_timesteps / elapsed_time:.2f} timesteps/second")

    return elapsed_time, total_fired


def run_comparison(num_neurons=10_000, density=0.01, num_timesteps=100, backends=None):
    """Run a comparison between single-GPU and multi-GPU implementations.

    Args:
        num_neurons: Number of neurons to create
        density: Connection density (fraction of possible connections)
        num_timesteps: Number of timesteps to simulate
        backends: List of backends to test (default: PyTorch and NumPy)
    """
    if backends is None:
        backends = [BackendType.PYTORCH, BackendType.NUMPY]

    results = {}

    for backend in backends:
        backend_name = (
            backend.value if isinstance(backend, BackendType) else str(backend)
        )
        logger.info(f"\n{'=' * 80}\nTesting {backend_name} backend\n{'=' * 80}")

        # Single-GPU implementation
        logger.info("\n--- Single-GPU implementation ---")
        connectome = ConnectomeManagerGPU(num_neurons, backend=backend)
        neuron_ids, input_neurons = create_test_brain(connectome, num_neurons, density)
        elapsed_time_single, total_fired_single = run_simulation(
            connectome, input_neurons, num_timesteps
        )

        # Multi-GPU implementation (if available)
        logger.info("\n--- Multi-GPU implementation ---")
        try:
            # Try to create multi-GPU configuration
            multi_gpu_config = MultiGPUConfig(
                enabled=True,
                partition_method=PartitionMethod.CORTICAL_AREAS,
                backend_type=backend,
            )

            # Check if we actually have multiple GPUs
            if multi_gpu_config.num_devices >= 2:
                connectome_multi = ConnectomeManagerGPU(
                    num_neurons, backend=backend, multi_gpu_config=multi_gpu_config
                )
                neuron_ids, input_neurons = create_test_brain(
                    connectome_multi, num_neurons, density
                )
                elapsed_time_multi, total_fired_multi = run_simulation(
                    connectome_multi, input_neurons, num_timesteps
                )

                # Calculate speedup
                speedup = elapsed_time_single / elapsed_time_multi
                logger.info(f"\nMulti-GPU speedup: {speedup:.2f}x")

                results[backend_name] = {
                    "single_gpu": elapsed_time_single,
                    "multi_gpu": elapsed_time_multi,
                    "speedup": speedup,
                }
            else:
                logger.info("Fewer than 2 GPUs available, skipping multi-GPU test")
                results[backend_name] = {
                    "single_gpu": elapsed_time_single,
                    "multi_gpu": None,
                    "speedup": None,
                }

        except Exception as e:
            logger.error(f"Multi-GPU implementation failed: {e}")
            results[backend_name] = {
                "single_gpu": elapsed_time_single,
                "multi_gpu": None,
                "speedup": None,
            }

    # Print summary
    logger.info("\n\n" + "=" * 80)
    logger.info("Performance Summary")
    logger.info("=" * 80)

    for backend, result in results.items():
        logger.info(f"\n{backend} backend:")
        logger.info(f"  Single-GPU: {result['single_gpu']:.2f} seconds")

        if result["multi_gpu"] is not None:
            logger.info(f"  Multi-GPU:  {result['multi_gpu']:.2f} seconds")
            logger.info(f"  Speedup:    {result['speedup']:.2f}x")
        else:
            logger.info("  Multi-GPU:  Not available")


def main():
    parser = argparse.ArgumentParser(description="FEAGI Multi-GPU Example")
    parser.add_argument("--neurons", type=int, default=10_000, help="Number of neurons")
    parser.add_argument(
        "--density", type=float, default=0.01, help="Connection density"
    )
    parser.add_argument(
        "--timesteps", type=int, default=100, help="Number of timesteps"
    )
    parser.add_argument(
        "--backend",
        type=str,
        default=None,
        choices=["pytorch", "numpy", "cupy", "webgpu", "auto"],
        help="Backend to use",
    )
    parser.add_argument(
        "--precision",
        type=str,
        default="fp32",
        choices=["fp32", "fp16", "mixed"],
        help="Precision to use (for supported backends)",
    )
    parser.add_argument(
        "--all-backends", action="store_true", help="Test all available backends"
    )

    args = parser.parse_args()

    # Determine backends to test
    if args.all_backends:
        # Test all available backends
        backends = []
        for backend_type in BackendType:
            if backend_type != BackendType.AUTO:
                try:
                    backend = ArrayBackend(backend_type)
                    if ArrayBackend._is_backend_available(backend_type):
                        backends.append(backend_type)
                except:
                    pass
    elif args.backend:
        # Use specified backend
        backends = [BackendType(args.backend.lower())]
    else:
        # Default to PyTorch and NumPy
        backends = [BackendType.PYTORCH, BackendType.NUMPY]

    # Run comparison
    run_comparison(
        num_neurons=args.neurons,
        density=args.density,
        num_timesteps=args.timesteps,
        backends=backends,
    )


if __name__ == "__main__":
    main()
