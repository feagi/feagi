"""
Debug test to investigate the specific FCL injection failure.

This test explores the exact lookup process that's failing in the injection service.
"""

from pathlib import Path

import pytest

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
from feagi.npu.fcl_injection_service import FCLInjectionService
from feagi.npu.special_area_handler import SpecialAreaHandler
from feagi.utils.config import FeagiConfig


def test_debug_injection_lookup():
    """Debug the cortical_id to cortical_idx lookup that's failing."""

    # Load essential genome
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

    # Create connectome
    config = FeagiConfig({})
    connectome = ConnectomeManager()
    embryogenesis = NeuroEmbryogenesis(connectome, config)
    success = embryogenesis.develop_brain(genome_path)
    assert success, "Failed to build connectome"

    # Activate power neuron
    if hasattr(connectome, "neuron_array"):
        neuron_array = connectome.neuron_array
        for i in range(neuron_array.next_index):
            if (
                i < len(neuron_array.cortical_idxs)
                and neuron_array.cortical_idxs[i] == 1
            ):
                neuron_array.is_active[i] = True
                print(f"✅ Activated power neuron at index {i}")

    # Create handler and service
    special_area_handler = SpecialAreaHandler(connectome)
    fcl_injection_service = FCLInjectionService(
        connectome.fcl_manager, special_area_handler
    )

    print(f"\n=== DEBUGGING CORTICAL AREA LOOKUP ===")

    # Debug available cortical areas
    print(f"Available cortical areas in connectome:")
    if hasattr(connectome, "cortical_areas"):
        for area_id, area in connectome.cortical_areas.items():
            print(
                f"  '{area_id}' -> cortical_idx={area.cortical_idx}, name='{area.name}'"
            )
    else:
        print("  No cortical_areas attribute found!")

    # Debug what the injection service is looking for
    print(f"\nDebug injection batch lookup:")
    for timing, batches in fcl_injection_service._injection_batches.items():
        for batch in batches:
            cortical_id = batch.cortical_id.split("_batch_")[0]
            print(
                f"  Batch cortical_id: '{batch.cortical_id}' -> stripped: '{cortical_id}'"
            )

            # Test the lookup logic manually
            cortical_idx = None
            if hasattr(connectome, "cortical_areas"):
                for area_id, area in connectome.cortical_areas.items():
                    print(
                        f"    Comparing '{area_id}' == '{cortical_id}': {area_id == cortical_id}"
                    )
                    if area_id == cortical_id:
                        cortical_idx = area.cortical_idx
                        print(f"    ✅ MATCH FOUND: cortical_idx={cortical_idx}")
                        break

            if cortical_idx is None:
                print(f"    ❌ NO MATCH FOUND for '{cortical_id}'")

    # Try the actual injection
    print(f"\n=== TESTING ACTUAL INJECTION ===")
    result = fcl_injection_service.inject_pre_burst(current_timestep=0)
    print(f"inject_pre_burst result: {result}")

    # Check FCL manager capabilities
    print(f"\n=== FCL MANAGER CAPABILITIES ===")
    fcl_manager = connectome.fcl_manager
    print(f"Has update_fcl: {hasattr(fcl_manager, 'update_fcl')}")
    print(f"Has add_to_current_fcl: {hasattr(fcl_manager, 'add_to_current_fcl')}")

    if hasattr(fcl_manager, "update_fcl"):
        print(f"update_fcl method: {fcl_manager.update_fcl}")
    if hasattr(fcl_manager, "add_to_current_fcl"):
        print(f"add_to_current_fcl method: {fcl_manager.add_to_current_fcl}")


if __name__ == "__main__":
    test_debug_injection_lookup()
