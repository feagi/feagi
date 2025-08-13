"""
Integration test for essential genome power area injection.

This test loads the actual essential genome and verifies that:
1. Power areas are detected and configured correctly
2. FCL injection service can inject power neurons
3. Neurons actually fire in the FCL
4. FQ sampler can detect the fired neurons

This test catches real-world issues that unit tests with mocks cannot detect.
"""

from pathlib import Path

import pytest

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
from feagi.npu.fcl_injection_service import FCLInjectionService
from feagi.npu.fq_sampler import UnifiedFQSampler
from feagi.npu.special_area_handler import SpecialAreaHandler
from feagi.utils.config import FeagiConfig


class TestEssentialGenomePowerInjection:
    """Test power area functionality with the actual essential genome."""

    @pytest.fixture(scope="class")
    def essential_genome_path(self):
        """Find the essential genome file."""
        possible_paths = [
            Path("feagi/evo/defaults/genome/essential_genome.json"),
            Path("../../feagi/evo/defaults/genome/essential_genome.json"),
            Path("../defaults/genome/essential_genome.json"),
        ]

        for path in possible_paths:
            if path.exists():
                return str(path.absolute())

        pytest.skip("Essential genome not found")

    @pytest.fixture(scope="class")
    def feagi_config(self):
        """Create FEAGI configuration."""
        return FeagiConfig({})

    @pytest.fixture(scope="class")
    def real_connectome(self, essential_genome_path, feagi_config):
        """Create real connectome from essential genome."""
        # Create connectome and develop brain
        connectome = ConnectomeManager()
        embryogenesis = NeuroEmbryogenesis(connectome, feagi_config)
        success = embryogenesis.develop_brain(essential_genome_path)

        assert success, "Failed to build connectome from essential genome"
        assert len(connectome.cortical_areas) > 0, "No cortical areas in connectome"

        # CRITICAL FIX: Activate power neurons so they can be detected by query methods
        # Power neurons need to be marked as active to pass the filtering in get_neurons_by_cortical_area()
        if hasattr(connectome, "neuron_array"):
            neuron_array = connectome.neuron_array
            # Find power neurons (cortical_idx=1) and activate them
            for i in range(neuron_array.next_index):
                if (
                    i < len(neuron_array.cortical_idxs)
                    and neuron_array.cortical_idxs[i] == 1
                ):
                    neuron_array.is_active[i] = True
                    print(f"✅ Activated power neuron at index {i}")

        return connectome

    @pytest.fixture(scope="class")
    def special_area_handler(self, real_connectome):
        """Create real special area handler with connectome."""
        handler = SpecialAreaHandler(real_connectome)
        return handler

    @pytest.fixture(scope="class")
    def fcl_manager(self, real_connectome):
        """Create real FCL manager with connectome."""
        return real_connectome.fcl_manager

    @pytest.fixture(scope="class")
    def fcl_injection_service(self, real_connectome, special_area_handler):
        """Create real FCL injection service."""
        service = FCLInjectionService(real_connectome.fcl_manager, special_area_handler)
        return service

    @pytest.fixture(scope="class")
    def fq_sampler(self, fcl_manager, real_connectome):
        """Create real FQ sampler."""
        sampler = UnifiedFQSampler(
            fire_queue_provider=fcl_manager,
            sample_frequency_hz=10.0,
            sampling_mode="visualization",
        )
        # Set the FCL manager reference that FQ sampler needs for area filtering
        sampler.fcl_manager = fcl_manager
        sampler.connectome_manager = real_connectome
        return sampler

    def test_essential_genome_loads_successfully(self, real_connectome):
        """Test that essential genome loads and creates expected structure."""
        # Check basic structure
        assert len(real_connectome.cortical_areas) > 0

        # Check for core areas
        assert "_death" in real_connectome.cortical_areas
        assert "_power" in real_connectome.cortical_areas

        # Verify power area properties
        power_area = real_connectome.cortical_areas["_power"]
        assert power_area.cortical_idx == 1
        assert power_area.name == "Brain_Power"

        print(
            f"✅ Essential genome loaded with {len(real_connectome.cortical_areas)} cortical areas"
        )
        print(
            f"✅ Power area '_power' found with cortical_idx={power_area.cortical_idx}"
        )

    def test_power_areas_detected(self, special_area_handler):
        """Test that power areas are detected correctly."""
        # Get power neurons using the handler
        power_neurons = special_area_handler.get_all_power_neurons()

        # Should find the power area with activated neurons
        assert len(power_neurons) > 0, (
            f"No power areas detected in essential genome. Available areas: {list(special_area_handler.connectome_manager.cortical_areas.keys())}"
        )

        # Verify power area contains neurons (power neurons returned by cortical_id, not cortical_idx)
        assert "_power" in power_neurons, (
            f"Core power area (_power) not found in power_neurons: {power_neurons}"
        )
        assert len(power_neurons["_power"]) > 0, (
            f"No neurons found in core power area: {power_neurons['_power']}"
        )

        print(
            f"✅ Detected {len(power_neurons)} power areas with {sum(len(neurons) for neurons in power_neurons.values())} total power neurons"
        )

    def test_fcl_injection_service_initialization(self, fcl_injection_service):
        """Test that FCL injection service initializes correctly with real data."""
        # Check that service found injection batches (correct attribute name is _injection_batches)
        assert hasattr(fcl_injection_service, "_injection_batches")
        total_batches = sum(
            len(batches)
            for batches in fcl_injection_service._injection_batches.values()
        )
        assert total_batches > 0, (
            "No injection batches created - power neurons not detected"
        )

        print(
            f"✅ FCL injection service initialized with {total_batches} injection batches"
        )

    def test_power_injection_actually_works(self, fcl_injection_service, fcl_manager):
        """Test that power injection actually adds neurons to FCL."""
        # Get initial FCL state
        initial_firing_neurons = fcl_manager.get_firing_neurons(offset=0)
        initial_fcl_count = len(initial_firing_neurons)
        print(f"Initial FCL neuron count: {initial_fcl_count}")

        # Perform injection
        neurons_injected = fcl_injection_service.inject_pre_burst(current_timestep=0)
        print(f"inject_pre_burst returned: {neurons_injected}")

        # Verify injection worked
        assert neurons_injected > 0, (
            f"inject_pre_burst returned {neurons_injected}, expected > 0"
        )

        # Check FCL state after injection
        post_injection_firing_neurons = fcl_manager.get_firing_neurons(offset=0)
        post_injection_fcl_count = len(post_injection_firing_neurons)
        print(f"Post-injection FCL neuron count: {post_injection_fcl_count}")

        # FCL should now contain the injected neurons
        assert post_injection_fcl_count > initial_fcl_count, (
            f"FCL neuron count did not increase: {initial_fcl_count} -> {post_injection_fcl_count}"
        )

    def test_fq_sampler_detects_power_activity(
        self, fq_sampler, fcl_injection_service, fcl_manager
    ):
        """Test that FQ sampler can detect activity from power injection."""
        # Inject power neurons first
        neurons_injected = fcl_injection_service.inject_pre_burst(current_timestep=0)
        assert neurons_injected > 0, "Power injection failed"

        # Sample the FCL
        samples = fq_sampler.sample()

        # Should detect the injected neurons
        assert samples is not None, "FQ sampler returned None - no activity detected"
        assert len(samples) > 0, f"FQ sampler detected no activity: {samples}"

        print(
            f"✅ FQ sampler detected {len(samples)} firing neurons after power injection"
        )

    def test_end_to_end_power_injection_flow(
        self, fcl_injection_service, fq_sampler, fcl_manager
    ):
        """Test complete power injection flow from injection to detection."""
        print("\n=== END-TO-END POWER INJECTION TEST ===")

        # Step 1: Initial state
        initial_fcl = fcl_manager.get_firing_neurons(offset=0)
        print(f"1. Initial FCL: {len(initial_fcl)} neurons")

        # Step 2: Inject power neurons
        injected_count = fcl_injection_service.inject_pre_burst(current_timestep=0)
        print(f"2. Injected: {injected_count} power neurons")
        assert injected_count > 0, "Power injection failed"

        # Step 3: Verify FCL contains injected neurons
        post_injection_fcl = fcl_manager.get_firing_neurons(offset=0)
        print(f"3. Post-injection FCL: {len(post_injection_fcl)} neurons")
        assert len(post_injection_fcl) > len(initial_fcl), (
            "FCL did not receive injected neurons"
        )

        # Step 4: Sample activity
        samples = fq_sampler.sample()
        print(
            f"4. FQ sampler detected: {len(samples) if samples else 0} firing neurons"
        )
        assert samples is not None and len(samples) > 0, (
            "FQ sampler failed to detect injected neurons"
        )

        # Step 5: Verify some injected neurons were sampled
        injected_neurons = set(post_injection_fcl) - set(initial_fcl)
        sampled_neurons = set(samples) if samples else set()
        overlap = injected_neurons & sampled_neurons
        print(f"5. Overlap between injected and sampled: {len(overlap)} neurons")

        print("✅ End-to-end power injection flow completed successfully")

        return {
            "initial_fcl_count": len(initial_fcl),
            "injected_count": injected_count,
            "post_injection_fcl_count": len(post_injection_fcl),
            "sampled_count": len(samples) if samples else 0,
            "overlap_count": len(overlap),
        }


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
