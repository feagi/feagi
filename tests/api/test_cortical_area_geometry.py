import pytest

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.core.services.cortical_area.cortical_area_service import (
    CorticalAreaService,
)
from feagi.bdu.connectome_manager import ConnectomeManager


class TestCorticalAreaGeometry:
    @pytest.fixture
    def connectome_manager(self):
        """Create a test ConnectomeManager instance."""
        return ConnectomeManager.instance(1000)  # Using integer for max_neurons

    @pytest.fixture
    def cortical_area_service(self, connectome_manager):
        """Create a test CorticalAreaService instance."""
        return CorticalAreaService(connectome_manager)

    @pytest.fixture
    def core_api_service(self, connectome_manager, cortical_area_service):
        """Create a test CoreAPIService instance."""
        return CoreAPIService(connectome_manager)

    @pytest.fixture(autouse=True)
    def clear_connectome(self, connectome_manager):
        # Clear all cortical areas and mappings before each test
        connectome_manager.cortical_areas.clear()
        if hasattr(connectome_manager, "cortical_map"):
            connectome_manager.cortical_map.id_to_idx.clear()
            connectome_manager.cortical_map.idx_to_id.clear()
        # If there are other related structures, clear them as well
        yield

    def test_get_cortical_area_geometry_empty(self, core_api_service):
        """Test geometry endpoint with no cortical areas."""
        result = core_api_service.get_cortical_area_geometry()
        assert isinstance(result, dict)
        assert len(result) == 0

    def test_get_cortical_area_geometry_single_area(
        self, core_api_service, connectome_manager
    ):
        """Test geometry endpoint with a single cortical area."""
        # Add a test cortical area
        area_id = connectome_manager.add_cortical_area(
            name="test_area",
            dimensions=(10, 10, 10),
            position=(0, 0, 0),
            area_type="test",
        )

        result = core_api_service.get_cortical_area_geometry()
        assert isinstance(result, dict)
        assert area_id in result
        assert result[area_id]["name"] == "test_area"
        assert result[area_id]["type"] == "test"
        assert result[area_id]["dimensions"] == (10, 10, 10)
        assert result[area_id]["coordinates"] == (0, 0, 0)

    def test_get_cortical_area_geometry_multiple_areas(
        self, core_api_service, connectome_manager
    ):
        """Test geometry endpoint with multiple cortical areas."""
        # Add multiple test areas
        area_ids = []
        for i in range(3):
            area_id = connectome_manager.add_cortical_area(
                name=f"test_area_{i}",
                dimensions=(10, 10, 10),
                position=(i * 10, i * 10, i * 10),
                area_type="test",
            )
            area_ids.append(area_id)

        result = core_api_service.get_cortical_area_geometry()
        assert isinstance(result, dict)
        assert len(result) == 3

        for i, area_id in enumerate(area_ids):
            assert area_id in result
            assert result[area_id]["name"] == f"test_area_{i}"
            assert result[area_id]["coordinates"] == (i * 10, i * 10, i * 10)

    def test_get_cortical_area_geometry_with_invalid_area(
        self, core_api_service, connectome_manager
    ):
        """Test geometry endpoint with an invalid cortical area."""
        # Add a valid area
        valid_area_id = connectome_manager.add_cortical_area(
            name="valid_area",
            dimensions=(10, 10, 10),
            position=(0, 0, 0),
            area_type="test",
        )

        # Create a list with both valid and invalid areas
        areas = [{"id": valid_area_id}, "invalid_area", (123, "invalid_tuple"), None]

        # Mock the get_all_areas method to return our test data
        core_api_service._cortical_area_service.get_all_areas = lambda: areas

        result = core_api_service.get_cortical_area_geometry()
        assert isinstance(result, dict)
        assert valid_area_id in result
        assert len(result) == 1  # Only the valid area should be included
