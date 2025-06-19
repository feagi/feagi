"""
Test block_to_block morphology mapping between cortical areas.

This test validates the complete flow from REST API call to synapse creation
and verifies that the mapping appears correctly in cortical_info endpoint.
"""


import pytest

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
from feagi.core.state_manager import FeagiStateManager


@pytest.fixture
def test_genome_with_block_to_block():
    """Create a test genome with block_to_block morphology defined."""
    return {
        "neuron_morphologies": {
            "block_to_block": {
                "type": "vectors",
                "parameters": {
                    "vectors": [
                        [0, 0, 0]
                    ]  # Identity mapping - each voxel to same voxel
                },
            }
        },
        "blueprint": {
            "test_src": {
                "cortical_id": "test_src",
                "cortical_name": "Test Source Area",
                "block_boundaries": [3, 3, 3],
                "coordinate": [0, 0, 0],
                "cortical_mapping_dst": {
                    "test_dst": [
                        {
                            "morphology_id": "block_to_block",
                            "morphology_scalar": [1, 1, 1],
                            "postSynapticCurrent_multiplier": 1.0,
                            "plasticity_flag": False,
                            "plasticity_constant": 1.0,
                            "ltp_multiplier": 1.0,
                            "ltd_multiplier": 1.0,
                        }
                    ]
                },
            },
            "test_dst": {
                "cortical_id": "test_dst",
                "cortical_name": "Test Destination Area",
                "block_boundaries": [3, 3, 3],
                "coordinate": [3, 0, 0],
                "cortical_mapping_dst": {},
            },
        },
    }


@pytest.fixture
def connectome_manager():
    """Create a ConnectomeManager for testing."""
    # Reset singleton to ensure fresh instance for each test
    ConnectomeManager.reset_singleton()
    return ConnectomeManager(config_or_max_neurons=10_000)


@pytest.fixture
def neuro_embryogenesis(connectome_manager):
    """Create a NeuroEmbryogenesis instance for testing."""
    return NeuroEmbryogenesis(connectome_manager)


@pytest.fixture
def core_api_service(connectome_manager):
    """Create a CoreAPIService for testing."""
    state_manager = FeagiStateManager.instance()
    return CoreAPIService(connectome_manager, state_manager)


class TestBlockToBlockMapping:
    """Test suite for block_to_block morphology mapping."""

    def test_genome_has_block_to_block_definition(
        self, test_genome_with_block_to_block
    ):
        """Test that the test genome has proper block_to_block definition."""
        genome = test_genome_with_block_to_block

        # Verify block_to_block morphology exists
        assert "block_to_block" in genome["neuron_morphologies"]

        # Verify it's a vector type
        block_to_block = genome["neuron_morphologies"]["block_to_block"]
        assert block_to_block["type"] == "vectors"

        # Verify it has vector parameters
        assert "parameters" in block_to_block
        assert "vectors" in block_to_block["parameters"]
        assert block_to_block["parameters"]["vectors"] == [[0, 0, 0]]

        print("✅ Genome has proper block_to_block definition")

    def test_create_cortical_areas(
        self, connectome_manager, test_genome_with_block_to_block
    ):
        """Test creating 3x3x3 cortical areas."""
        # Create source area only if it doesn't exist
        try:
            src_area = connectome_manager.get_cortical_area("test_src")
            src_area_id = "test_src"
        except:
            src_area_id = connectome_manager.add_cortical_area(
                cortical_id="test_src",
                name="Test Source Area",
                dimensions=(3, 3, 3),
                position=(0, 0, 0),
            )

        # Create destination area only if it doesn't exist
        try:
            dst_area = connectome_manager.get_cortical_area("test_dst")
            dst_area_id = "test_dst"
        except:
            dst_area_id = connectome_manager.add_cortical_area(
                cortical_id="test_dst",
                name="Test Destination Area",
                dimensions=(3, 3, 3),
                position=(3, 0, 0),
            )

        assert src_area_id == "test_src"
        assert dst_area_id == "test_dst"

        # Verify areas exist
        src_area = connectome_manager.get_cortical_area("test_src")
        dst_area = connectome_manager.get_cortical_area("test_dst")

        assert src_area is not None
        assert dst_area is not None
        assert src_area.dimensions == (3, 3, 3)
        assert dst_area.dimensions == (3, 3, 3)

        print("✅ Created 3x3x3 cortical areas")

    def test_create_neurons_in_areas(self, connectome_manager):
        """Test creating neurons in both cortical areas using proper batch creation."""
        # Create areas first only if they don't exist
        try:
            connectome_manager.get_cortical_area("test_src")
        except:
            connectome_manager.add_cortical_area(
                cortical_id="test_src",
                name="Test Source Area",
                dimensions=(3, 3, 3),
                position=(0, 0, 0),
            )

        try:
            connectome_manager.get_cortical_area("test_dst")
        except:
            connectome_manager.add_cortical_area(
                cortical_id="test_dst",
                name="Test Destination Area",
                dimensions=(3, 3, 3),
                position=(3, 0, 0),
            )

        # Create all positions for source area (one neuron per voxel)
        src_positions = []
        for x in range(3):
            for y in range(3):
                for z in range(3):
                    src_positions.append((x, y, z))

        # Create all positions for destination area (one neuron per voxel)
        dst_positions = []
        for x in range(3):
            for y in range(3):
                for z in range(3):
                    dst_positions.append((x, y, z))

        # Check if neurons already exist in areas
        existing_src_neurons = connectome_manager.get_neurons_by_area("test_src")
        existing_dst_neurons = connectome_manager.get_neurons_by_area("test_dst")

        # Only create neurons if they don't already exist
        if len(existing_src_neurons) == 0:
            # Create all neurons in source area at once using batch_create_neurons
            src_neurons = connectome_manager.batch_create_neurons(
                cortical_id="test_src",
                positions=src_positions,
                threshold=1.0,
                membrane_potential=0.0,
                resting_potential=0.0,
                decay_rate=0.5,
                refractory_period=1,
            )
        else:
            src_neurons = existing_src_neurons

        if len(existing_dst_neurons) == 0:
            # Create all neurons in destination area at once using batch_create_neurons
            dst_neurons = connectome_manager.batch_create_neurons(
                cortical_id="test_dst",
                positions=dst_positions,
                threshold=1.0,
                membrane_potential=0.0,
                resting_potential=0.0,
                decay_rate=0.5,
                refractory_period=1,
            )
        else:
            dst_neurons = existing_dst_neurons

        # Verify neurons were created
        assert len(src_neurons) == 27  # 3x3x3
        assert len(dst_neurons) == 27  # 3x3x3

        # Verify neurons exist in areas
        src_area_neurons = connectome_manager.get_neurons_by_area("test_src")
        dst_area_neurons = connectome_manager.get_neurons_by_area("test_dst")

        assert len(src_area_neurons) == 27
        assert len(dst_area_neurons) == 27

        print(
            f"✅ Created {len(src_neurons)} source neurons and {len(dst_neurons)} destination neurons"
        )

    def test_morphology_driven_mapping(
        self, neuro_embryogenesis, connectome_manager, test_genome_with_block_to_block
    ):
        """Test the morphology-driven mapping process."""
        # Setup areas and neurons
        self.test_create_cortical_areas(
            connectome_manager, test_genome_with_block_to_block
        )
        self.test_create_neurons_in_areas(connectome_manager)

        # Load genome into neuro_embryogenesis
        neuro_embryogenesis.genome = test_genome_with_block_to_block

        # Create mapping data in the format expected by update_cortical_mapping
        mapping_data = {
            "test_src": {
                "test_dst": [
                    {
                        "morphology_id": "block_to_block",
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

        # Apply the mapping
        success = neuro_embryogenesis.update_cortical_mapping(mapping_data)
        assert success, "Morphology mapping should succeed"

        # Verify synapses were created
        connection_matrix = connectome_manager.get_connection_matrix(
            "test_src", "test_dst"
        )
        assert connection_matrix is not None, "Connection matrix should exist"

        connections = connection_matrix.get("connections", [])
        assert len(connections) > 0, "Should have created synapses"

        print(f"✅ Created {len(connections)} synapses via morphology-driven mapping")

        # Store connections for other tests to access
        self._last_connections = connections

    def test_validate_voxel_to_voxel_mapping(
        self, neuro_embryogenesis, connectome_manager, test_genome_with_block_to_block
    ):
        """Test that each source voxel maps to corresponding destination voxel."""
        # Run the morphology mapping
        self.test_morphology_driven_mapping(
            neuro_embryogenesis, connectome_manager, test_genome_with_block_to_block
        )

        # Get connections from the previous test
        connections = getattr(self, "_last_connections", [])

        # Get neuron positions for validation
        src_neurons = connectome_manager.get_neurons_by_area("test_src")
        dst_neurons = connectome_manager.get_neurons_by_area("test_dst")

        # Create position to neuron mappings
        src_pos_to_neuron = {}
        for neuron_id in src_neurons:
            pos = connectome_manager.get_neuron_position(neuron_id)
            src_pos_to_neuron[pos] = neuron_id

        dst_pos_to_neuron = {}
        for neuron_id in dst_neurons:
            pos = connectome_manager.get_neuron_position(neuron_id)
            dst_pos_to_neuron[pos] = neuron_id

        # Validate connections
        expected_connections = set()
        for x in range(3):
            for y in range(3):
                for z in range(3):
                    src_pos = (x, y, z)
                    dst_pos = (
                        x,
                        y,
                        z,
                    )  # block_to_block with [0,0,0] vector = identity mapping

                    if src_pos in src_pos_to_neuron and dst_pos in dst_pos_to_neuron:
                        src_neuron = src_pos_to_neuron[src_pos]
                        dst_neuron = dst_pos_to_neuron[dst_pos]
                        expected_connections.add((src_neuron, dst_neuron))

        # Check actual connections
        actual_connections = set()
        for conn in connections:
            if len(conn) >= 2:
                actual_connections.add((conn[0], conn[1]))

        # Validate
        missing_connections = expected_connections - actual_connections
        extra_connections = actual_connections - expected_connections

        print(f"Expected {len(expected_connections)} voxel-to-voxel connections")
        print(f"Found {len(actual_connections)} actual connections")
        print(f"Missing connections: {len(missing_connections)}")
        print(f"Extra connections: {len(extra_connections)}")

        if missing_connections:
            print("Missing connections:")
            for src, dst in list(missing_connections)[:5]:  # Show first 5
                src_pos = connectome_manager.get_neuron_position(src)
                dst_pos = connectome_manager.get_neuron_position(dst)
                print(f"  {src}@{src_pos} -> {dst}@{dst_pos}")

        if extra_connections:
            print("Extra connections:")
            for src, dst in list(extra_connections)[:5]:  # Show first 5
                try:
                    src_pos = connectome_manager.get_neuron_position(src)
                except KeyError:
                    src_pos = "INVALID"
                try:
                    dst_pos = connectome_manager.get_neuron_position(dst)
                except KeyError:
                    dst_pos = "INVALID"
                print(f"  {src}@{src_pos} -> {dst}@{dst_pos}")

        # Debug: Show raw connection data
        print(f"Raw connections: {connections[:5]}")  # Show first 5 raw connections

        # Debug: Show actual neuron IDs in areas
        src_neurons = connectome_manager.get_neurons_by_area("test_src")
        dst_neurons = connectome_manager.get_neurons_by_area("test_dst")
        print(f"Source neuron IDs: {sorted(src_neurons)[:10]}")  # Show first 10
        print(f"Destination neuron IDs: {sorted(dst_neurons)[:10]}")  # Show first 10

        # Debug: Show what neuron 27 is actually connected to
        if 27 in [conn[0] for conn in actual_connections]:
            neuron_27_connections = [
                conn for conn in actual_connections if conn[0] == 27
            ]
            print(f"Neuron 27 actual connections: {neuron_27_connections}")
            for src, dst in neuron_27_connections:
                try:
                    dst_pos = connectome_manager.get_neuron_position(dst)
                    print(f"  27@(2,2,2) -> {dst}@{dst_pos}")
                except KeyError:
                    print(f"  27@(2,2,2) -> {dst}@INVALID")

        assert len(missing_connections) == 0, (
            f"Missing {len(missing_connections)} expected connections"
        )

        print("✅ All voxel-to-voxel connections validated")

    def test_cortical_info_api_endpoint(
        self,
        core_api_service,
        neuro_embryogenesis,
        connectome_manager,
        test_genome_with_block_to_block,
    ):
        """Test that the cortical_info API endpoint shows the mapping."""
        # Run the complete mapping process
        self.test_validate_voxel_to_voxel_mapping(
            neuro_embryogenesis, connectome_manager, test_genome_with_block_to_block
        )

        # Test the cortical_info API endpoint
        cortical_info = core_api_service.get_cortical_area("test_src")

        assert cortical_info is not None, "Cortical area info should be available"

        # Check that the area has the expected properties
        assert cortical_info.get("id") == "test_src"
        assert cortical_info.get("name") == "Test Source Area"

        # Check that mapping information is included
        parameters = cortical_info.get("parameters", {})
        mapping = parameters.get("mapping", {})

        # Should have mapping to test_dst
        assert "test_dst" in mapping, (
            f"Should have mapping to test_dst, got: {list(mapping.keys())}"
        )

        # The mapping should contain the connections we created
        dst_connections = mapping["test_dst"]
        assert len(dst_connections) > 0, "Should have connections to destination area"

        print(
            f"✅ API endpoint shows {len(dst_connections)} connections from test_src to test_dst"
        )

    def test_complete_block_to_block_flow(
        self,
        core_api_service,
        neuro_embryogenesis,
        connectome_manager,
        test_genome_with_block_to_block,
    ):
        """Test the complete flow from genome definition to API visibility."""
        print("\n🧪 Testing complete block_to_block mapping flow...")

        # Step 1: Verify genome definition
        self.test_genome_has_block_to_block_definition(test_genome_with_block_to_block)

        # Step 2: Create cortical areas
        self.test_create_cortical_areas(
            connectome_manager, test_genome_with_block_to_block
        )

        # Step 3: Create neurons
        self.test_create_neurons_in_areas(connectome_manager)

        # Step 4: Apply morphology mapping
        self.test_morphology_driven_mapping(
            neuro_embryogenesis, connectome_manager, test_genome_with_block_to_block
        )

        # Step 5: Validate voxel-to-voxel mapping
        self.test_validate_voxel_to_voxel_mapping(
            neuro_embryogenesis, connectome_manager, test_genome_with_block_to_block
        )

        # Step 6: Test API endpoint
        self.test_cortical_info_api_endpoint(
            core_api_service,
            neuro_embryogenesis,
            connectome_manager,
            test_genome_with_block_to_block,
        )

        print("🎉 Complete block_to_block mapping flow successful!")

        # Store results for debugging but don't return them (pytest doesn't expect return values)
        self._flow_results = {
            "connections_created": len(self._last_connections),
            "areas_created": 2,
            "neurons_created": 54,  # 27 per area
            "api_validated": True,
        }


def test_debug_block_to_block_mapping():
    """Debug test to identify where block_to_block mapping fails."""
    # Reset singleton to ensure fresh instance
    ConnectomeManager.reset_singleton()

    # Create fresh instances
    connectome_manager = ConnectomeManager(config_or_max_neurons=10_000)
    neuro_embryogenesis = NeuroEmbryogenesis(connectome_manager)
    state_manager = FeagiStateManager.instance()
    core_api_service = CoreAPIService(connectome_manager, state_manager)

    # Create test genome
    test_genome = {
        "neuron_morphologies": {
            "block_to_block": {
                "type": "vectors",
                "parameters": {
                    "vectors": [[0, 0, 0]]  # Identity mapping
                },
            }
        }
    }

    # Run the complete test
    test_suite = TestBlockToBlockMapping()
    test_suite.test_complete_block_to_block_flow(
        core_api_service, neuro_embryogenesis, connectome_manager, test_genome
    )

    print(f"\n📊 Test Results: {test_suite._flow_results}")


if __name__ == "__main__":
    test_debug_block_to_block_mapping()
