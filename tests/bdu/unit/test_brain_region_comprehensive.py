"""
Comprehensive tests for BrainRegion model to achieve high code coverage.

This test suite focuses on covering the missing areas in the brain_region.py module,
including region management, genome operations, and the BrainRegion class.
"""

import json
from types import SimpleNamespace
from unittest.mock import MagicMock, mock_open, patch

import pytest

from feagi.bdu.models.brain_region import (
    BrainRegion,
    change_brain_region_parent,
    change_cortical_area_parent,
    construct_genome_from_region,
    create_region,
    delete_region_with_members,
    generate_hash,
    region_id_2_title,
    region_id_gen,
    relocate_region_members,
    update_region,
)


class TestBrainRegionClass:
    """Test the BrainRegion class functionality."""

    def test_brain_region_initialization(self):
        """Test basic brain region initialization."""
        region = BrainRegion(
            region_id="test_region",
            name="Test Region",
            region_type="sensory",
            properties={"color": "blue", "priority": 1},
        )

        assert region.id == "test_region"
        assert region.name == "Test Region"
        assert region.region_type == "sensory"
        assert region.properties == {"color": "blue", "priority": 1}
        assert len(region.cortical_areas) == 0

    def test_brain_region_initialization_defaults(self):
        """Test brain region initialization with default values."""
        region = BrainRegion(region_id="test_region", name="Test Region")

        assert region.id == "test_region"
        assert region.name == "Test Region"
        assert region.region_type == "custom"
        assert region.properties == {}
        assert len(region.cortical_areas) == 0

    def test_add_area(self):
        """Test adding cortical areas to a region."""
        region = BrainRegion(region_id="test_region", name="Test Region")

        region.add_area("area1")
        region.add_area("area2")

        assert "area1" in region.cortical_areas
        assert "area2" in region.cortical_areas
        assert len(region.cortical_areas) == 2

    def test_add_duplicate_area(self):
        """Test adding duplicate areas (should not create duplicates)."""
        region = BrainRegion(region_id="test_region", name="Test Region")

        region.add_area("area1")
        region.add_area("area1")  # Duplicate

        assert len(region.cortical_areas) == 1
        assert "area1" in region.cortical_areas

    def test_remove_area_success(self):
        """Test successfully removing an area."""
        region = BrainRegion(region_id="test_region", name="Test Region")
        region.add_area("area1")
        region.add_area("area2")

        result = region.remove_area("area1")

        assert result is True
        assert "area1" not in region.cortical_areas
        assert "area2" in region.cortical_areas
        assert len(region.cortical_areas) == 1

    def test_remove_area_not_found(self):
        """Test removing an area that doesn't exist."""
        region = BrainRegion(region_id="test_region", name="Test Region")
        region.add_area("area1")

        result = region.remove_area("nonexistent_area")

        assert result is False
        assert len(region.cortical_areas) == 1

    def test_contains_area(self):
        """Test checking if region contains an area."""
        region = BrainRegion(region_id="test_region", name="Test Region")
        region.add_area("area1")

        assert region.contains_area("area1") is True
        assert region.contains_area("nonexistent_area") is False

    def test_get_all_areas(self):
        """Test getting all areas in a region."""
        region = BrainRegion(region_id="test_region", name="Test Region")
        region.add_area("area1")
        region.add_area("area2")
        region.add_area("area3")

        areas = region.get_all_areas()

        assert isinstance(areas, list)
        assert len(areas) == 3
        assert set(areas) == {"area1", "area2", "area3"}

    def test_get_all_areas_empty(self):
        """Test getting all areas from empty region."""
        region = BrainRegion(region_id="test_region", name="Test Region")

        areas = region.get_all_areas()

        assert isinstance(areas, list)
        assert len(areas) == 0

    def test_to_dict(self):
        """Test converting brain region to dictionary."""
        region = BrainRegion(
            region_id="test_region",
            name="Test Region",
            region_type="motor",
            properties={"speed": "fast"},
        )
        region.add_area("area1")
        region.add_area("area2")

        result = region.to_dict()

        expected = {
            "id": "test_region",
            "name": "Test Region",
            "region_type": "motor",
            "cortical_areas": ["area1", "area2"],  # Order may vary
            "properties": {"speed": "fast"},
        }

        assert result["id"] == expected["id"]
        assert result["name"] == expected["name"]
        assert result["region_type"] == expected["region_type"]
        assert result["properties"] == expected["properties"]
        assert set(result["cortical_areas"]) == set(expected["cortical_areas"])

    def test_update_valid_properties(self):
        """Test updating valid brain region properties."""
        region = BrainRegion(
            region_id="test_region",
            name="Old Name",
            region_type="custom",
            properties={"old_prop": "old_value"},
        )

        updates = {
            "name": "New Name",
            "region_type": "sensory",
            "properties": {"new_prop": "new_value"},
        }

        region.update(updates)

        assert region.name == "New Name"
        assert region.region_type == "sensory"
        assert region.properties == {"old_prop": "old_value", "new_prop": "new_value"}

    def test_update_invalid_property(self):
        """Test updating invalid brain region property."""
        region = BrainRegion(region_id="test_region", name="Test Region")

        with pytest.raises(KeyError, match="Invalid property: invalid_prop"):
            region.update({"invalid_prop": "value"})

    def test_clear_areas(self):
        """Test clearing all areas from a region."""
        region = BrainRegion(region_id="test_region", name="Test Region")
        region.add_area("area1")
        region.add_area("area2")
        region.add_area("area3")

        assert len(region.cortical_areas) == 3

        region.clear_areas()

        assert len(region.cortical_areas) == 0


class TestRegionUtilityFunctions:
    """Test utility functions for brain region management."""

    @patch("feagi.bdu.models.brain_region.state")
    def test_region_id_2_title_found(self, mock_state):
        """Test getting region title by ID when region exists."""
        mock_state.genome = {
            "brain_regions": {
                "region1": {"title": "Visual Cortex"},
                "region2": {"title": "Motor Cortex"},
            }
        }

        result = region_id_2_title("region1")
        assert result == "Visual Cortex"

    @patch("feagi.bdu.models.brain_region.state")
    def test_region_id_2_title_not_found(self, mock_state):
        """Test getting region title by ID when region doesn't exist."""
        mock_state.genome = {"brain_regions": {"region1": {"title": "Visual Cortex"}}}

        result = region_id_2_title("nonexistent_region")
        assert result is None

    @patch("feagi.bdu.models.brain_region.state")
    def test_region_id_2_title_no_title(self, mock_state):
        """Test getting region title when region has no title."""
        mock_state.genome = {
            "brain_regions": {"region1": {"description": "Some description"}}
        }

        result = region_id_2_title("region1")
        assert result is None

    def test_region_id_gen_format(self):
        """Test region ID generation format."""
        region_id = region_id_gen()

        # Should end with _R
        assert region_id.endswith("_R")

        # Should have timestamp and random part
        parts = region_id.split("_")
        assert len(parts) == 3
        assert parts[2] == "R"

        # Timestamp part should be numeric
        assert parts[0].isdigit()

        # Random part should be alphanumeric
        assert parts[1].isalnum()

    def test_region_id_gen_custom_size(self):
        """Test region ID generation with custom size."""
        region_id = region_id_gen(size=10)

        parts = region_id.split("_")
        assert len(parts[1]) == 10  # Random part should be 10 characters

    def test_region_id_gen_custom_chars(self):
        """Test region ID generation with custom character set."""
        region_id = region_id_gen(size=5, chars="ABC")

        parts = region_id.split("_")
        random_part = parts[1]

        # All characters in random part should be from custom set
        for char in random_part:
            assert char in "ABC"

    def test_generate_hash_dict(self):
        """Test hash generation for dictionary data."""
        data = {"key1": "value1", "key2": "value2"}

        hash1 = generate_hash(data)
        hash2 = generate_hash(data)

        # Same data should produce same hash
        assert hash1 == hash2
        assert isinstance(hash1, str)
        assert len(hash1) > 0

    def test_generate_hash_list(self):
        """Test hash generation for list data."""
        data = ["item1", "item2", "item3"]

        hash1 = generate_hash(data)
        hash2 = generate_hash(data)

        # Same data should produce same hash
        assert hash1 == hash2
        assert isinstance(hash1, str)

    def test_generate_hash_string(self):
        """Test hash generation for string data."""
        data = "test string"

        hash1 = generate_hash(data)
        hash2 = generate_hash(data)

        # Same data should produce same hash
        assert hash1 == hash2
        assert isinstance(hash1, str)

    def test_generate_hash_different_data(self):
        """Test that different data produces different hashes."""
        data1 = {"key": "value1"}
        data2 = {"key": "value2"}

        hash1 = generate_hash(data1)
        hash2 = generate_hash(data2)

        # Different data should produce different hashes
        assert hash1 != hash2


class TestRegionManagementFunctions:
    """Test functions for managing brain regions in the genome."""

    @patch("feagi.bdu.models.brain_region.state")
    def test_change_cortical_area_parent_success(self, mock_state):
        """Test successfully changing cortical area parent."""
        mock_state.cortical_area_region_association = {"area1": "old_region"}
        mock_state.genome = {
            "brain_regions": {
                "old_region": {"areas": ["area1", "area2"]},
                "new_region": {"areas": ["area3"]},
            }
        }

        change_cortical_area_parent("area1", "new_region")

        # Check association updated
        assert mock_state.cortical_area_region_association["area1"] == "new_region"

        # Check area removed from old region
        assert "area1" not in mock_state.genome["brain_regions"]["old_region"]["areas"]

        # Check area added to new region
        assert "area1" in mock_state.genome["brain_regions"]["new_region"]["areas"]

    @patch("feagi.bdu.models.brain_region.state")
    def test_change_cortical_area_parent_no_current_parent(self, mock_state):
        """Test changing cortical area parent when no current parent exists."""
        mock_state.cortical_area_region_association = {}

        with pytest.raises(RuntimeError, match="Failed to change cortical area parent"):
            change_cortical_area_parent("area1", "new_region")

    @patch("feagi.bdu.models.brain_region.state")
    def test_change_cortical_area_parent_exception_handling(self, mock_state):
        """Test exception handling in change_cortical_area_parent."""
        mock_state.cortical_area_region_association = {"area1": "old_region"}
        mock_state.genome = {"brain_regions": {}}  # Missing regions

        with pytest.raises(RuntimeError, match="Failed to change cortical area parent"):
            change_cortical_area_parent("area1", "new_region")

    @patch("feagi.bdu.models.brain_region.state")
    def test_change_brain_region_parent(self, mock_state):
        """Test changing brain region parent."""
        mock_state.genome = {
            "brain_regions": {
                "region1": {"parent_region_id": "old_parent"},
                "old_parent": {"regions": ["region1", "region2"]},
                "new_parent": {"regions": ["region3"]},
            }
        }

        change_brain_region_parent("region1", "new_parent")

        # Check parent updated
        assert (
            mock_state.genome["brain_regions"]["region1"]["parent_region_id"]
            == "new_parent"
        )

        # Check region removed from old parent
        assert (
            "region1" not in mock_state.genome["brain_regions"]["old_parent"]["regions"]
        )

        # Check region added to new parent
        assert "region1" in mock_state.genome["brain_regions"]["new_parent"]["regions"]

    @patch("feagi.bdu.models.brain_region.state")
    @patch("feagi.bdu.models.brain_region.region_id_gen")
    def test_create_region_basic(self, mock_id_gen, mock_state):
        """Test creating a basic region."""
        mock_id_gen.return_value = "new_region_id"
        mock_state.genome = {"brain_regions": {"parent_region": {"regions": []}}}
        mock_state.cortical_list = []

        region_data = SimpleNamespace(
            title="New Region",
            region_description="Test region",
            parent_region_id="parent_region",
            coordinates_2d=[10, 20],
            coordinates_3d=[10, 20, 30],
        )

        result = create_region(region_data)

        assert result == "new_region_id"

        # Check region was created
        created_region = mock_state.genome["brain_regions"]["new_region_id"]
        assert created_region["title"] == "New Region"
        assert created_region["description"] == "Test region"
        assert created_region["parent_region_id"] == "parent_region"
        assert created_region["coordinate_2d"] == [10, 20]
        assert created_region["coordinate_3d"] == [10, 20, 30]

        # Check added to parent
        assert (
            "new_region_id"
            in mock_state.genome["brain_regions"]["parent_region"]["regions"]
        )

    @patch("feagi.bdu.models.brain_region.state")
    @patch("feagi.bdu.models.brain_region.region_id_gen")
    @patch("feagi.bdu.models.brain_region.change_cortical_area_parent")
    def test_create_region_with_areas(
        self, mock_change_parent, mock_id_gen, mock_state
    ):
        """Test creating a region with associated areas."""
        mock_id_gen.return_value = "new_region_id"
        mock_state.genome = {"brain_regions": {"parent_region": {"regions": []}}}
        mock_state.cortical_list = ["area1", "area2"]

        region_data = SimpleNamespace(
            title="New Region",
            parent_region_id="parent_region",
            areas=["area1", "area2"],
        )

        result = create_region(region_data)

        assert result == "new_region_id"

        # Check areas were associated
        assert mock_change_parent.call_count == 2
        mock_change_parent.assert_any_call(
            cortical_area_id="area1", new_parent_id="new_region_id"
        )
        mock_change_parent.assert_any_call(
            cortical_area_id="area2", new_parent_id="new_region_id"
        )

    @patch("feagi.bdu.models.brain_region.state")
    def test_update_region_basic_properties(self, mock_state):
        """Test updating basic region properties."""
        mock_state.genome = {
            "brain_regions": {
                "region1": {"title": "Old Title", "description": "Old Description"}
            }
        }

        region_data = {
            "region_id": "region1",
            "title": "New Title",
            "description": "New Description",
        }

        update_region(region_data)

        updated_region = mock_state.genome["brain_regions"]["region1"]
        assert updated_region["title"] == "New Title"
        assert updated_region["description"] == "New Description"

    @patch("feagi.bdu.models.brain_region.state")
    @patch("feagi.bdu.models.brain_region.change_brain_region_parent")
    def test_update_region_parent(self, mock_change_parent, mock_state):
        """Test updating region parent."""
        mock_state.genome = {
            "brain_regions": {"region1": {"parent_region_id": "old_parent"}}
        }

        region_data = {"region_id": "region1", "parent_region_id": "new_parent"}

        update_region(region_data)

        mock_change_parent.assert_called_once_with(
            region_id="region1", new_parent_id="new_parent"
        )

    @patch("feagi.bdu.models.brain_region.state")
    def test_update_region_invalid_property(self, mock_state):
        """Test updating region with invalid property."""
        region_data = {
            "region_id": "region1",
            "area": "invalid",  # This should raise an error
        }

        with pytest.raises(ValueError, match="cannot be updated using this endpoint"):
            update_region(region_data)

    @patch("feagi.bdu.models.brain_region.state")
    @patch("feagi.bdu.models.brain_region.change_cortical_area_parent")
    @patch("feagi.bdu.models.brain_region.change_brain_region_parent")
    def test_delete_region_with_members(
        self, mock_change_region_parent, mock_change_area_parent, mock_state
    ):
        """Test deleting a region and reassigning its members."""
        mock_state.genome = {
            "brain_regions": {
                "region_to_delete": {
                    "parent_region_id": "parent_region",
                    "areas": ["area1", "area2"],
                    "regions": ["subregion1"],
                },
                "parent_region": {"regions": ["region_to_delete", "other_region"]},
            }
        }

        delete_region_with_members("region_to_delete")

        # Check areas were moved to parent
        assert mock_change_area_parent.call_count == 2
        mock_change_area_parent.assert_any_call(
            cortical_area_id="area1", new_parent_id="parent_region"
        )
        mock_change_area_parent.assert_any_call(
            cortical_area_id="area2", new_parent_id="parent_region"
        )

        # Check subregions were moved to parent
        mock_change_region_parent.assert_called_once_with(
            region_id="subregion1", new_parent_id="parent_region"
        )

        # Check region was removed from parent's list
        assert (
            "region_to_delete"
            not in mock_state.genome["brain_regions"]["parent_region"]["regions"]
        )

        # Check region was deleted
        assert "region_to_delete" not in mock_state.genome["brain_regions"]

    @patch("feagi.bdu.models.brain_region.state")
    @patch("feagi.bdu.models.brain_region.change_cortical_area_parent")
    @patch("feagi.bdu.models.brain_region.change_brain_region_parent")
    def test_relocate_region_members_cortical_area(
        self, mock_change_region_parent, mock_change_area_parent, mock_state
    ):
        """Test relocating cortical area members."""
        mock_state.genome = {
            "blueprint": {"area1": {"2d_coordinate": [0, 0]}},
            "brain_regions": {},
        }

        relocation_data = {
            "area1": {"coordinate_2d": [10, 20], "parent_region_id": "new_parent"}
        }

        relocate_region_members(relocation_data)

        # Check coordinates were updated
        assert mock_state.genome["blueprint"]["area1"]["2d_coordinate"] == [10, 20]

        # Check parent was changed
        mock_change_area_parent.assert_called_once_with(
            cortical_area_id="area1", new_parent_id="new_parent"
        )

    @patch("feagi.bdu.models.brain_region.state")
    @patch("feagi.bdu.models.brain_region.change_brain_region_parent")
    def test_relocate_region_members_brain_region(self, mock_change_parent, mock_state):
        """Test relocating brain region members."""
        mock_state.genome = {
            "blueprint": {},
            "brain_regions": {"region1": {"coordinate_2d": [0, 0]}, "new_parent": {}},
        }

        relocation_data = {
            "region1": {"coordinate_2d": [30, 40], "parent_region_id": "new_parent"}
        }

        relocate_region_members(relocation_data)

        # Check coordinates were updated
        assert mock_state.genome["brain_regions"]["region1"]["coordinate_2d"] == [
            30,
            40,
        ]

        # Check parent was changed
        mock_change_parent.assert_called_once_with(
            region_id="region1", new_parent_id="new_parent"
        )

    @patch("feagi.bdu.models.brain_region.state")
    def test_relocate_region_members_invalid_parent(self, mock_state):
        """Test relocating with invalid parent region."""
        mock_state.genome = {
            "blueprint": {},
            "brain_regions": {"region1": {"coordinate_2d": [0, 0]}},
        }

        relocation_data = {"region1": {"parent_region_id": "nonexistent_parent"}}

        with pytest.raises(ValueError, match="is not a valid region id"):
            relocate_region_members(relocation_data)

    @patch("feagi.bdu.models.brain_region.state")
    def test_relocate_region_members_invalid_object(self, mock_state):
        """Test relocating invalid object."""
        mock_state.genome = {"blueprint": {}, "brain_regions": {}}

        relocation_data = {"nonexistent_object": {"coordinate_2d": [10, 20]}}

        with pytest.raises(ValueError, match="is not a valid region nor cortical id"):
            relocate_region_members(relocation_data)


class TestGenomeConstruction:
    """Test genome construction from regions."""

    @patch("feagi.bdu.models.brain_region.state")
    @patch("feagi.bdu.models.brain_region.generate_hash")
    @patch("feagi.evo.genome_processor.genome_v1_v2_converter")
    def test_construct_genome_from_region_basic(
        self, mock_converter, mock_hash, mock_state
    ):
        """Test basic genome construction from region."""
        mock_hash.return_value = "test_hash"
        mock_converter.return_value = {"blueprint": {"area1": {"converted": "data"}}}
        mock_state.genome = {
            "brain_regions": {
                "region1": {
                    "title": "Test Region",
                    "description": "Test Description",
                    "areas": ["area1"],
                    "regions": [],
                }
            },
            "blueprint": {"area1": {"some": "data"}},
            "version": "2.0",
            "physiology": {"test": "physiology"},
        }

        result = construct_genome_from_region("region1")

        # Check basic structure
        assert result["genome_title"] == "Test Region"
        assert result["genome_description"] == "Test Description"
        assert result["version"] == "2.0"
        assert "genome_id" in result
        assert result["genome_id"].endswith("_R")

        # Check blueprint was included
        assert "area1" in result["blueprint"]

        # Check signatures were set
        assert result["signatures"]["genome"] == "test_hash"
        assert result["signatures"]["blueprint"] == "test_hash"
        assert result["signatures"]["physiology"] == "test_hash"

    @patch("feagi.bdu.models.brain_region.state")
    def test_construct_genome_from_region_not_found(self, mock_state):
        """Test genome construction with non-existent region."""
        mock_state.genome = {"brain_regions": {}}

        with pytest.raises(
            ValueError, match="Region ID nonexistent not found in genome"
        ):
            construct_genome_from_region("nonexistent")

    @patch("feagi.bdu.models.brain_region.state")
    @patch("feagi.bdu.models.brain_region.generate_hash")
    @patch("feagi.evo.genome_processor.genome_v1_v2_converter")
    def test_construct_genome_from_region_with_subregions(
        self, mock_converter, mock_hash, mock_state
    ):
        """Test genome construction with subregions."""
        mock_hash.return_value = "test_hash"
        mock_converter.return_value = {
            "blueprint": {
                "area1": {"converted": "data1"},
                "area2": {"converted": "data2"},
            }
        }
        mock_state.genome = {
            "brain_regions": {
                "parent_region": {
                    "title": "Parent Region",
                    "description": "Parent Description",
                    "areas": ["area1"],
                    "regions": ["child_region"],
                },
                "child_region": {
                    "title": "Child Region",
                    "areas": ["area2"],
                    "regions": [],
                },
            },
            "blueprint": {"area1": {"data": "area1"}, "area2": {"data": "area2"}},
            "version": "2.0",
            "physiology": {},
        }

        result = construct_genome_from_region("parent_region")

        # Check both areas are included
        assert "area1" in result["blueprint"]
        assert "area2" in result["blueprint"]

        # Check subregion is included
        assert "child_region" in result["brain_regions"]


class TestEdgeCasesAndErrorHandling:
    """Test edge cases and error handling scenarios."""

    @patch("feagi.bdu.models.brain_region.state")
    def test_create_region_minimal_data(self, mock_state):
        """Test creating region with minimal data."""
        mock_state.genome = {"brain_regions": {"parent_region": {"regions": []}}}
        mock_state.cortical_list = []

        region_data = SimpleNamespace(
            title="Minimal Region", parent_region_id="parent_region"
        )

        with patch("feagi.bdu.models.brain_region.region_id_gen") as mock_id_gen:
            mock_id_gen.return_value = "minimal_region_id"
            result = create_region(region_data)

        assert result == "minimal_region_id"

        # Check defaults were applied
        created_region = mock_state.genome["brain_regions"]["minimal_region_id"]
        assert created_region["description"] == ""
        assert created_region["coordinate_2d"] == [0, 0]
        assert created_region["coordinate_3d"] == [0, 0, 0]

    @patch("feagi.bdu.models.brain_region.state")
    def test_delete_region_not_exists(self, mock_state):
        """Test deleting a region that doesn't exist."""
        mock_state.genome = {"brain_regions": {}}

        # Should not raise an error, just do nothing
        delete_region_with_members("nonexistent_region")

    def test_brain_region_update_properties_merge(self):
        """Test that updating properties merges with existing ones."""
        region = BrainRegion(
            region_id="test_region",
            name="Test Region",
            properties={"existing": "value", "keep": "this"},
        )

        region.update({"properties": {"existing": "new_value", "added": "property"}})

        expected_properties = {
            "existing": "new_value",  # Updated
            "keep": "this",  # Preserved
            "added": "property",  # Added
        }

        assert region.properties == expected_properties

    @patch("feagi.bdu.models.brain_region.state")
    def test_change_cortical_area_parent_area_not_in_old_region(self, mock_state):
        """Test changing parent when area is not in old region's list."""
        mock_state.cortical_area_region_association = {"area1": "old_region"}
        mock_state.genome = {
            "brain_regions": {
                "old_region": {"areas": ["area2"]},  # area1 not in list
                "new_region": {"areas": []},
            }
        }

        # Should not raise error, just skip removal
        change_cortical_area_parent("area1", "new_region")

        # Check association updated
        assert mock_state.cortical_area_region_association["area1"] == "new_region"

        # Check area added to new region
        assert "area1" in mock_state.genome["brain_regions"]["new_region"]["areas"]

    @patch("feagi.bdu.models.brain_region.state")
    def test_change_cortical_area_parent_area_already_in_new_region(self, mock_state):
        """Test changing parent when area is already in new region."""
        mock_state.cortical_area_region_association = {"area1": "old_region"}
        mock_state.genome = {
            "brain_regions": {
                "old_region": {"areas": ["area1"]},
                "new_region": {"areas": ["area1"]},  # Already in new region
            }
        }

        # Should not add duplicate
        change_cortical_area_parent("area1", "new_region")

        # Check association updated
        assert mock_state.cortical_area_region_association["area1"] == "new_region"

        # Check no duplicate in new region
        assert (
            mock_state.genome["brain_regions"]["new_region"]["areas"].count("area1")
            == 1
        )
