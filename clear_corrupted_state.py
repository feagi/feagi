#!/usr/bin/env python3
"""
Clear Corrupted Connectome State

This script clears any pre-existing corrupted connectome state that was loaded
before the BiDirectionalCorticalMap was properly implemented. This ensures
the system starts fresh and goes through proper neuroembryogenesis.

Usage: python3 clear_corrupted_state.py
"""

import os
import sys

# Add the feagi directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "."))


def clear_corrupted_connectome_state():
    """Clear corrupted pre-existing connectome state."""
    try:
        from feagi.bdu.connectome_manager import ConnectomeManager
        from feagi.core.state_manager import get_state_manager

        print("🧹 Clearing corrupted pre-existing connectome state...")

        # Reset the singleton ConnectomeManager
        if ConnectomeManager._instance is not None:
            print(f"   Found existing ConnectomeManager instance")

            connectome_manager = ConnectomeManager._instance

            # Check what's currently there
            cortical_areas_count = len(connectome_manager.cortical_areas)
            neuron_count = connectome_manager.get_neuron_count()
            mapping_count = len(connectome_manager.cortical_mapping._id_to_idx)

            print(
                f"   Current state: {cortical_areas_count} cortical areas, {neuron_count} neurons, {mapping_count} mapping entries"
            )

            if cortical_areas_count > 0 or neuron_count > 0:
                print("   🗑️  Clearing corrupted data...")

                # Clear all cortical areas (except core reserved ones)
                areas_to_remove = []
                for cortical_id in connectome_manager.cortical_areas:
                    if cortical_id not in connectome_manager.reserved_cortical_areas:
                        areas_to_remove.append(cortical_id)

                for cortical_id in areas_to_remove:
                    print(f"   Removing corrupted cortical area: {cortical_id}")
                    connectome_manager.delete_cortical_area(
                        cortical_id, delete_neurons=True
                    )

                # Clear any remaining neurons (safety cleanup)
                remaining_neurons = connectome_manager.get_neuron_count()
                if remaining_neurons > 0:
                    print(f"   Clearing {remaining_neurons} remaining neurons...")
                    all_neuron_ids = []
                    for cortical_idx in connectome_manager.neuron_array.cortical_idx:
                        if cortical_idx >= 0:  # Valid cortical_idx
                            neuron_id = (
                                connectome_manager.neuron_array.index_to_neuron_id[
                                    cortical_idx
                                ]
                            )
                            if neuron_id > 0:  # Valid neuron_id
                                all_neuron_ids.append(neuron_id)

                    if all_neuron_ids:
                        connectome_manager.delete_neurons(all_neuron_ids)

                # Rebuild cortical mapping from clean state (core areas only)
                print("   🔧 Rebuilding cortical mapping with core areas only...")
                from feagi.bdu.cortical_mapping import BiDirectionalCorticalMap

                connectome_manager.cortical_mapping = BiDirectionalCorticalMap()
                # Core areas will be re-added when needed during proper embryogenesis

                print("   ✅ Corrupted state cleared")
            else:
                print("   ✅ No corrupted data found - state is clean")

            # Reset singleton to force fresh initialization
            ConnectomeManager._instance = None
            ConnectomeManager._initialized = False
            print("   🔄 Reset singleton for fresh initialization")

        # Clear state manager genome state
        state_manager = get_state_manager()
        if state_manager:
            from feagi.core.state_manager import GenomeState

            current_state = state_manager.get_genome_state()
            if current_state != GenomeState.MISSING:
                print(f"   Resetting genome state from {current_state.name} to MISSING")
                state_manager.set_genome_state(GenomeState.MISSING)
                state_manager.set_brain_readiness(False)

        print("🎉 Corrupted state cleanup complete!")
        print(
            "   The system will now require proper genome loading through neuroembryogenesis."
        )
        return True

    except Exception as e:
        print(f"❌ Error during cleanup: {e}")
        import traceback

        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = clear_corrupted_connectome_state()
    sys.exit(0 if success else 1)
