"""
Debug test to investigate power area neuron detection issue.

This test helps understand why SpecialAreaHandler can't find power neurons
even though they appear to be created during neurogenesis.
"""

from pathlib import Path

import numpy as np
import pytest

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
from feagi.npu.special_area_handler import SpecialAreaHandler
from feagi.utils.config import FeagiConfig


def test_debug_power_area_detection():
    """Debug test to understand power area neuron detection."""

    # Find essential genome
    possible_paths = [
        Path("feagi/evo/defaults/genome/essential_genome.json"),
        Path("../../feagi/evo/defaults/genome/essential_genome.json"),
        Path("../defaults/genome/essential_genome.json"),
    ]

    genome_path = None
    for path in possible_paths:
        if path.exists():
            genome_path = str(path.absolute())
            break

    if not genome_path:
        pytest.skip("Essential genome not found")

    # Create connectome and develop brain
    connectome = ConnectomeManager()
    config = FeagiConfig({})

    embryogenesis = NeuroEmbryogenesis(connectome, config)
    success = embryogenesis.develop_brain(genome_path)

    assert success, "Brain development failed"

    print(f"\n=== POWER AREA DEBUG ===")
    print(f"Total cortical areas: {len(connectome.cortical_areas)}")

    # 1. Check if ___pwr area exists
    if "___pwr" in connectome.cortical_areas:
        pwr_area = connectome.cortical_areas["___pwr"]
        print(f"✅ Power area '___pwr' found:")
        print(f"   cortical_idx: {getattr(pwr_area, 'cortical_idx', 'MISSING')}")
        print(f"   name: {getattr(pwr_area, 'name', 'MISSING')}")
        print(f"   dimensions: {getattr(pwr_area, 'dimensions', 'MISSING')}")
    else:
        print(f"❌ Power area '___pwr' NOT found")
        print(f"Available areas: {list(connectome.cortical_areas.keys())}")

    # 2. Check neuron array structure
    if hasattr(connectome, "neuron_array"):
        neuron_array = connectome.neuron_array
        print(f"\nNeuron array info:")
        print(f"   Total neurons: {neuron_array.neuron_count}")
        print(f"   Has cortical_idxs: {hasattr(neuron_array, 'cortical_idxs')}")

        if hasattr(neuron_array, "cortical_idxs") and neuron_array.neuron_count > 0:
            cortical_idxs = neuron_array.cortical_idxs
            print(f"   Cortical_idxs length: {len(cortical_idxs)}")

            # Count neurons by cortical_idx
            from collections import Counter

            idx_counts = Counter(cortical_idxs)
            print(f"   Neurons per cortical_idx: {dict(idx_counts)}")

            # Specifically check cortical_idx=1
            neurons_with_idx_1 = [i for i, idx in enumerate(cortical_idxs) if idx == 1]
            print(f"   Neurons with cortical_idx=1: {neurons_with_idx_1}")

            # NEW: Check active and valid masks for neuron at index 1
            if len(neurons_with_idx_1) > 0:
                power_neuron_idx = neurons_with_idx_1[0]  # Should be index 1
                print(
                    f"\n🔍 POWER NEURON DETAILED ANALYSIS (index {power_neuron_idx}):"
                )

                # Check valid mask
                valid_mask = neuron_array.backend.to_numpy(neuron_array.valid_mask)
                is_valid = (
                    valid_mask[power_neuron_idx]
                    if power_neuron_idx < len(valid_mask)
                    else False
                )
                print(f"   valid_mask[{power_neuron_idx}]: {is_valid}")

                # Check active mask
                active_mask = neuron_array.backend.to_numpy(neuron_array.is_active)
                is_active = (
                    active_mask[power_neuron_idx]
                    if power_neuron_idx < len(active_mask)
                    else False
                )
                print(f"   is_active[{power_neuron_idx}]: {is_active}")

                # Check other properties
                print(
                    f"   membrane_potential: {neuron_array.membrane_potentials[power_neuron_idx]}"
                )
                print(f"   threshold: {neuron_array.thresholds[power_neuron_idx]}")
                print(
                    f"   cortical_idx: {neuron_array.cortical_idxs[power_neuron_idx]}"
                )

                # Check if neuron ID mapping exists
                neuron_id_found = None
                for neuron_id, idx in neuron_array.id_to_index_map.items():
                    if idx == power_neuron_idx:
                        neuron_id_found = neuron_id
                        break
                print(f"   neuron_id: {neuron_id_found}")

                # Check ConnectomeManager mappings
                if hasattr(connectome, "neuron_id_to_index"):
                    if (
                        neuron_id_found
                        and neuron_id_found in connectome.neuron_id_to_index
                    ):
                        print(
                            f"   ConnectomeManager has mapping for neuron_id {neuron_id_found}: ✅"
                        )
                    else:
                        print(
                            f"   ConnectomeManager missing mapping for neuron_id {neuron_id_found}: ❌"
                        )

    # 3. Test direct access methods
    print(f"\n=== TESTING ACCESS METHODS ===")

    try:
        # Method 1: get_neurons_by_cortical_idx
        neurons_by_idx = connectome.get_neurons_by_cortical_idx(1)
        print(f"get_neurons_by_cortical_idx(1): {neurons_by_idx}")
    except Exception as e:
        print(f"get_neurons_by_cortical_idx(1) FAILED: {e}")

    try:
        # Method 2: get_neurons_by_area
        neurons_by_area = connectome.get_neurons_by_area("___pwr")
        print(f"get_neurons_by_area('___pwr'): {neurons_by_area}")
    except Exception as e:
        print(f"get_neurons_by_area('___pwr') FAILED: {e}")

    # NEW: Test the filtered query step by step
    print(f"\n=== STEP-BY-STEP QUERY DEBUG ===")
    if hasattr(connectome, "neuron_array"):
        neuron_array = connectome.neuron_array
        cortical_idx = 1

        # Step 1: Valid range
        valid_range = min(neuron_array.next_index, len(neuron_array.cortical_idxs))
        print(f"valid_range: {valid_range}")

        # Step 2: Get masks
        active_mask = neuron_array.backend.to_numpy(
            neuron_array.is_active[:valid_range]
        )
        valid_mask = neuron_array.backend.to_numpy(
            neuron_array.valid_mask[:valid_range]
        )
        cortical_idxs = neuron_array.backend.to_numpy(
            neuron_array.cortical_idxs[:valid_range]
        )

        print(f"active_mask shape: {active_mask.shape}")
        print(f"valid_mask shape: {valid_mask.shape}")
        print(f"cortical_idxs shape: {cortical_idxs.shape}")

        # Step 3: Create individual masks
        cortical_mask = cortical_idxs == cortical_idx
        print(
            f"cortical_mask (cortical_idx=={cortical_idx}): {np.where(cortical_mask)[0]}"
        )

        # Step 4: Show mask values for power neuron index
        if len(neurons_with_idx_1) > 0:
            power_idx = neurons_with_idx_1[0]
            if power_idx < len(active_mask):
                print(f"Power neuron (index {power_idx}) mask values:")
                print(f"   active_mask[{power_idx}]: {active_mask[power_idx]}")
                print(f"   valid_mask[{power_idx}]: {valid_mask[power_idx]}")
                print(f"   cortical_mask[{power_idx}]: {cortical_mask[power_idx]}")

                # Step 5: Combined mask
                combined_mask = active_mask & valid_mask & cortical_mask
                print(f"   combined_mask[{power_idx}]: {combined_mask[power_idx]}")

                # Step 6: Find all passing indices
                valid_indices = np.where(combined_mask)[0]
                print(f"All valid indices: {valid_indices}")

    # 4. Test SpecialAreaHandler
    print(f"\n=== TESTING SPECIAL AREA HANDLER ===")
    handler = SpecialAreaHandler(connectome)

    power_neurons = handler.get_power_area_neurons()
    print(f"SpecialAreaHandler.get_power_area_neurons(): {power_neurons}")

    all_power_neurons = handler.get_all_power_neurons()
    print(f"SpecialAreaHandler.get_all_power_neurons(): {all_power_neurons}")

    # 5. Check if there's a method mismatch
    print(f"\n=== CHECKING CONNECTOME METHODS ===")
    methods = [m for m in dir(connectome) if "neuron" in m.lower()]
    print(f"Available neuron methods: {methods}")

    # 6. Final verification - manually check neuron assignments
    if hasattr(connectome, "neuron_array") and connectome.neuron_array.neuron_count > 0:
        print(f"\n=== MANUAL VERIFICATION ===")
        neuron_array = connectome.neuron_array

        if hasattr(neuron_array, "cortical_idxs"):
            # Find all neurons assigned to cortical_idx=1 manually
            idx_1_neurons = []
            for neuron_id in range(
                neuron_array.next_index
            ):  # Use next_index instead of count
                if neuron_id < len(neuron_array.cortical_idxs):
                    if neuron_array.cortical_idxs[neuron_id] == 1:
                        idx_1_neurons.append(neuron_id)

            print(f"Manual search for cortical_idx=1 neurons: {idx_1_neurons}")

    print(f"\n=== DEBUG COMPLETE ===")


if __name__ == "__main__":
    test_debug_power_area_detection()
