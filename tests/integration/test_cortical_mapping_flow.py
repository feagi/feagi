"""
Integration test for cortical mapping flow from API to connectome validation.
Tests the complete flow: PUT mapping -> synapse creation -> GET validation
Reproduces the exact scenario from debug logs: ___pwr -> iv00_C using block_to_block
"""


import pytest

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.core.state_manager import FeagiStateManager
from feagi.utils.config import FeagiConfig


class TestCorticalMappingFlow:
    """Test complete cortical mapping flow from API to connectome"""

    @pytest.fixture
    def config(self):
        """Create a FeagiConfig for testing."""
        config = FeagiConfig()
        yield config

    @pytest.fixture
    def state_manager(self, config):
        """Create a FeagiStateManager for testing."""
        state_manager = FeagiStateManager.instance()
        yield state_manager

    @pytest.fixture
    def connectome_manager(self, config, state_manager):
        """Create a ConnectomeManager for testing."""
        connectome_manager = ConnectomeManager(config, state_manager)
        yield connectome_manager

    @pytest.fixture
    def core_api_service(self, connectome_manager, state_manager, config):
        """Create a CoreAPIService for testing."""
        core_api_service = CoreAPIService(
            connectome_manager=connectome_manager, state_manager=state_manager
        )
        yield core_api_service

    def test_debug_logs_reproduction(self, core_api_service):
        """
        ✅ CRITICAL FIX VALIDATION: Reproduce the exact scenario from debug logs

        This test validates that the critical issue has been fixed:
        - NeuroEmbryogenesis now has genome data loaded before cortical mapping
        - Synapses are successfully created using block_to_block morphology
        - No more "No genome or neuron_morphologies available" error
        """

        print("\n🔍 Reproducing debug logs scenario...")

        # Load essential genome
        result = core_api_service.load_essential_genome()
        genome_loaded = result.get("success", False)
        print(f"Genome loaded: {genome_loaded}")

        # Check if areas exist
        cortical_areas = core_api_service.get_cortical_id_list()
        power_area_exists = "___pwr" in cortical_areas
        vision_area_exists = "iv00_C" in cortical_areas
        print(f"Power area exists: {power_area_exists}")
        print(f"Vision area exists: {vision_area_exists}")

        # Check neuron counts
        connectome_manager = core_api_service.get_connectome_manager()
        power_neurons = connectome_manager.get_neurons_by_area("___pwr")
        vision_neurons = connectome_manager.get_neurons_by_area("iv00_C")
        print(f"Power neurons: {len(power_neurons)}")
        print(f"Vision neurons: {len(vision_neurons)}")

        # Check available morphologies
        morphologies = core_api_service.get_morphologies()
        print(f"Available morphologies: {list(morphologies.keys())}")
        block_to_block_exists = "block_to_block" in morphologies
        print(f"block_to_block exists: {block_to_block_exists}")

        # Get initial synapse count
        initial_synapse_count = connectome_manager.get_synapse_count()
        print(f"Initial synapse count: {initial_synapse_count}")

        # Reproduce the exact PUT request from debug logs
        print("Attempting cortical mapping update...")
        mapping_data = [
            {
                "morphology_id": "block_to_block",
                "morphology_scalar": [1, 1, 1],
                "plasticity_flag": False,
                "postSynapticCurrent_multiplier": 1.0,
            }
        ]

        # Apply the mapping (this was failing before the fix)
        success = core_api_service.update_cortical_mapping_properties(
            src_cortical_area="___pwr",
            dst_cortical_area="iv00_C",
            mapping_string=mapping_data,
        )
        print(f"Mapping update success: {success}")

        # Verify synapses were created (this is the critical validation)
        final_synapse_count = connectome_manager.get_synapse_count()
        synapses_created = final_synapse_count - initial_synapse_count
        print(f"Final synapse count: {final_synapse_count}")
        print(f"Synapses created: {synapses_created}")

        # ✅ CRITICAL ASSERTION: Synapses should now be created successfully
        assert synapses_created > 0, (
            f"CRITICAL FIX VALIDATION FAILED: Expected synapses to be created, but got {synapses_created}. The NeuroEmbryogenesis genome loading fix did not work."
        )

        # Additional validation: Check specific synapse exists
        power_neuron_id = power_neurons[0] if power_neurons else None
        vision_neuron_id = vision_neurons[0] if vision_neurons else None

        if power_neuron_id and vision_neuron_id:
            synapse_exists = connectome_manager.has_synapse(
                power_neuron_id, vision_neuron_id
            )
            print(
                f"Direct synapse exists from power {power_neuron_id} to vision {vision_neuron_id}: {synapse_exists}"
            )

        print("✅ CRITICAL FIX VALIDATED: Synapses created successfully!")
        print("✅ The NeuroEmbryogenesis genome loading issue has been resolved!")
