"""
Comprehensive tests for cortical_mappings module to achieve high code coverage.

This test suite focuses on covering the missing areas in the cortical_mappings.py module,
including mapping restrictions, defaults, cortical mappings, and coordinate transformations.
"""

from typing import Any, Dict
from unittest.mock import MagicMock, patch

import pytest

from feagi.bdu.connectivity.cortical_mappings import (
    MAPPING_TYPES,
    CorticalMapping,
    CorticalMappingRestrictionsRegistry,
    MappingDefault,
    MappingRestriction,
    ProjectionMapping,
    TopologicalMapping,
    create_cortical_mapping,
    get_mapping_restrictions_registry,
    register_custom_mapping,
)


class TestMappingRestriction:
    """Test the MappingRestriction class functionality."""

    def test_mapping_restriction_initialization_defaults(self):
        """Test MappingRestriction initialization with default values."""
        restriction = MappingRestriction(source_type="IPU", destination_type="OPU")

        assert restriction.source_type == "IPU"
        assert restriction.destination_type == "OPU"
        assert restriction.restricted_morphologies == []
        assert restriction.disallowed_morphologies == []
        assert restriction.max_mappings == -1
        assert restriction.allow_scalar_change is True
        assert restriction.allow_psp_change is True
        assert restriction.allow_inhibitory_change is True
        assert restriction.allow_plasticity_change is True
        assert restriction.allow_plasticity_constant_change is True
        assert restriction.allow_ltp_change is True
        assert restriction.allow_ltd_change is True

    def test_mapping_restriction_initialization_custom(self):
        """Test MappingRestriction initialization with custom values."""
        restriction = MappingRestriction(
            source_type="MEMORY",
            destination_type="OPU",
            restricted_morphologies=["projector"],
            disallowed_morphologies=["inhibitory"],
            max_mappings=5,
            allow_scalar_change=False,
            allow_psp_change=False,
            allow_inhibitory_change=False,
            allow_plasticity_change=False,
            allow_plasticity_constant_change=False,
            allow_ltp_change=False,
            allow_ltd_change=False,
        )

        assert restriction.source_type == "MEMORY"
        assert restriction.destination_type == "OPU"
        assert restriction.restricted_morphologies == ["projector"]
        assert restriction.disallowed_morphologies == ["inhibitory"]
        assert restriction.max_mappings == 5
        assert restriction.allow_scalar_change is False
        assert restriction.allow_psp_change is False
        assert restriction.allow_inhibitory_change is False
        assert restriction.allow_plasticity_change is False
        assert restriction.allow_plasticity_constant_change is False
        assert restriction.allow_ltp_change is False
        assert restriction.allow_ltd_change is False

    def test_has_restricted_morphologies(self):
        """Test has_restricted_morphologies method."""
        # No restricted morphologies
        restriction1 = MappingRestriction("IPU", "OPU")
        assert restriction1.has_restricted_morphologies() is False

        # With restricted morphologies
        restriction2 = MappingRestriction(
            "IPU", "OPU", restricted_morphologies=["projector", "memory"]
        )
        assert restriction2.has_restricted_morphologies() is True

    def test_has_disallowed_morphologies(self):
        """Test has_disallowed_morphologies method."""
        # No disallowed morphologies
        restriction1 = MappingRestriction("IPU", "OPU")
        assert restriction1.has_disallowed_morphologies() is False

        # With disallowed morphologies
        restriction2 = MappingRestriction(
            "IPU", "OPU", disallowed_morphologies=["inhibitory"]
        )
        assert restriction2.has_disallowed_morphologies() is True

    def test_has_max_mappings(self):
        """Test has_max_mappings method."""
        # No max mappings (unlimited)
        restriction1 = MappingRestriction("IPU", "OPU")
        assert restriction1.has_max_mappings() is False

        # With max mappings
        restriction2 = MappingRestriction("IPU", "OPU", max_mappings=10)
        assert restriction2.has_max_mappings() is True

    def test_is_morphology_allowed_no_restrictions(self):
        """Test is_morphology_allowed with no restrictions."""
        restriction = MappingRestriction("IPU", "OPU")

        assert restriction.is_morphology_allowed("projector") is True
        assert restriction.is_morphology_allowed("memory") is True
        assert restriction.is_morphology_allowed("inhibitory") is True

    def test_is_morphology_allowed_with_restricted_morphologies(self):
        """Test is_morphology_allowed with restricted morphologies."""
        restriction = MappingRestriction(
            "IPU", "OPU", restricted_morphologies=["projector", "memory"]
        )

        assert restriction.is_morphology_allowed("projector") is True
        assert restriction.is_morphology_allowed("memory") is True
        assert restriction.is_morphology_allowed("inhibitory") is False

    def test_is_morphology_allowed_with_disallowed_morphologies(self):
        """Test is_morphology_allowed with disallowed morphologies."""
        restriction = MappingRestriction(
            "IPU", "OPU", disallowed_morphologies=["inhibitory"]
        )

        assert restriction.is_morphology_allowed("projector") is True
        assert restriction.is_morphology_allowed("memory") is True
        assert restriction.is_morphology_allowed("inhibitory") is False

    def test_is_morphology_allowed_restricted_takes_precedence(self):
        """Test that restricted morphologies take precedence over disallowed."""
        restriction = MappingRestriction(
            "IPU",
            "OPU",
            restricted_morphologies=["projector"],
            disallowed_morphologies=["projector"],  # Conflicting rules
        )

        # Restricted takes precedence - projector should be allowed
        assert restriction.is_morphology_allowed("projector") is True
        assert restriction.is_morphology_allowed("memory") is False

    def test_to_dict(self):
        """Test converting restriction to dictionary."""
        restriction = MappingRestriction(
            source_type="MEMORY",
            destination_type="OPU",
            restricted_morphologies=["projector"],
            max_mappings=5,
            allow_scalar_change=False,
        )

        result = restriction.to_dict()

        assert result["source_type"] == "MEMORY"
        assert result["destination_type"] == "OPU"
        assert result["restricted_morphologies"] == ["projector"]
        assert result["max_mappings"] == 5
        assert result["allow_scalar_change"] is False
        assert result["has_restricted_morphologies"] is True
        assert result["has_disallowed_morphologies"] is False
        assert result["has_max_mappings"] is True


class TestMappingDefault:
    """Test the MappingDefault class functionality."""

    def test_mapping_default_initialization(self):
        """Test MappingDefault initialization."""
        default = MappingDefault(
            source_type="IPU", destination_type="OPU", default_morphology="projector"
        )

        assert default.source_type == "IPU"
        assert default.destination_type == "OPU"
        assert default.default_morphology == "projector"

    def test_to_dict(self):
        """Test converting default to dictionary."""
        default = MappingDefault(
            source_type="MEMORY", destination_type="OPU", default_morphology="memory"
        )

        result = default.to_dict()

        assert result["source_type"] == "MEMORY"
        assert result["destination_type"] == "OPU"
        assert result["default_morphology"] == "memory"


class TestCorticalMappingRestrictionsRegistry:
    """Test the CorticalMappingRestrictionsRegistry class functionality."""

    def test_registry_initialization(self):
        """Test registry initialization with default restrictions."""
        registry = CorticalMappingRestrictionsRegistry()

        # Should have some default restrictions
        assert len(registry.restrictions) > 0
        assert len(registry.defaults) >= 0

    def test_add_restriction(self):
        """Test adding a restriction to the registry."""
        registry = CorticalMappingRestrictionsRegistry()
        initial_count = len(registry.restrictions)

        restriction = MappingRestriction("TEST", "TEST2")
        registry.add_restriction(restriction)

        assert len(registry.restrictions) == initial_count + 1
        assert restriction in registry.restrictions

    def test_add_default(self):
        """Test adding a default to the registry."""
        registry = CorticalMappingRestrictionsRegistry()
        initial_count = len(registry.defaults)

        default = MappingDefault("TEST", "TEST2", "test_morphology")
        registry.add_default(default)

        assert len(registry.defaults) == initial_count + 1
        assert default in registry.defaults

    def test_get_restriction_found(self):
        """Test getting a restriction that exists."""
        registry = CorticalMappingRestrictionsRegistry()

        restriction = MappingRestriction("TEST_SRC", "TEST_DST")
        registry.add_restriction(restriction)

        result = registry.get_restriction("TEST_SRC", "TEST_DST")
        assert result == restriction

    def test_get_restriction_not_found(self):
        """Test getting a restriction that doesn't exist."""
        registry = CorticalMappingRestrictionsRegistry()

        result = registry.get_restriction("NONEXISTENT", "ALSO_NONEXISTENT")
        assert result is None

    def test_get_restriction_case_sensitive(self):
        """Test that restriction lookup is case sensitive."""
        registry = CorticalMappingRestrictionsRegistry()

        restriction = MappingRestriction("test_src", "test_dst")
        registry.add_restriction(restriction)

        # Should find with exact case
        result = registry.get_restriction("test_src", "test_dst")
        assert result == restriction

        # Should not find with different case
        result = registry.get_restriction("TEST_SRC", "TEST_DST")
        assert result is None

    def test_get_default_found(self):
        """Test getting a default that exists."""
        registry = CorticalMappingRestrictionsRegistry()

        default = MappingDefault("TEST_SRC", "TEST_DST", "test_morphology")
        registry.add_default(default)

        result = registry.get_default("TEST_SRC", "TEST_DST")
        assert result == default

    def test_get_default_not_found(self):
        """Test getting a default that doesn't exist."""
        registry = CorticalMappingRestrictionsRegistry()

        result = registry.get_default("NONEXISTENT", "ALSO_NONEXISTENT")
        assert result is None

    def test_get_default_case_sensitive(self):
        """Test that default lookup is case sensitive."""
        registry = CorticalMappingRestrictionsRegistry()

        default = MappingDefault("test_src", "test_dst", "test_morphology")
        registry.add_default(default)

        # Should find with exact case
        result = registry.get_default("test_src", "test_dst")
        assert result == default

        # Should not find with different case
        result = registry.get_default("TEST_SRC", "TEST_DST")
        assert result is None

    def test_get_all_restrictions(self):
        """Test getting all restrictions as dictionaries."""
        registry = CorticalMappingRestrictionsRegistry()

        restriction = MappingRestriction("TEST", "TEST2")
        registry.add_restriction(restriction)

        result = registry.get_all_restrictions()

        assert isinstance(result, list)
        assert len(result) > 0
        # Should contain our added restriction
        test_restrictions = [r for r in result if r["source_type"] == "TEST"]
        assert len(test_restrictions) == 1

    def test_get_all_defaults(self):
        """Test getting all defaults as dictionaries."""
        registry = CorticalMappingRestrictionsRegistry()

        default = MappingDefault("TEST", "TEST2", "test_morphology")
        registry.add_default(default)

        result = registry.get_all_defaults()

        assert isinstance(result, list)
        # Should contain our added default
        test_defaults = [d for d in result if d["source_type"] == "TEST"]
        assert len(test_defaults) == 1

    def test_to_dict(self):
        """Test converting registry to dictionary."""
        registry = CorticalMappingRestrictionsRegistry()

        result = registry.to_dict()

        assert "restrictions" in result
        assert "defaults" in result
        assert isinstance(result["restrictions"], list)
        assert isinstance(result["defaults"], list)


class TestRegistryFunction:
    """Test the get_mapping_restrictions_registry function."""

    def test_get_mapping_restrictions_registry(self):
        """Test that the function returns a registry instance."""
        registry = get_mapping_restrictions_registry()

        assert isinstance(registry, CorticalMappingRestrictionsRegistry)
        # Should have default restrictions
        assert len(registry.restrictions) > 0


class TestCorticalMapping:
    """Test the base CorticalMapping class functionality."""

    def test_cortical_mapping_initialization(self):
        """Test CorticalMapping initialization."""
        parameters = {
            "morphology_id": "test_morphology",
            "morphology_scalar": [1.0, 1.0, 1.0],
        }

        mapping = CorticalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            mapping_type="test",
            parameters=parameters,
            name="Test Mapping",
            description="Test Description",
            mapping_id="test_mapping_id",
        )

        assert mapping.source_cortical_id == "src_area"
        assert mapping.target_cortical_id == "tgt_area"
        assert mapping.mapping_type == "test"
        assert mapping.parameters == parameters
        assert mapping.name == "Test Mapping"
        assert mapping.description == "Test Description"
        assert mapping.id == "test_mapping_id"

    def test_cortical_mapping_initialization_defaults(self):
        """Test CorticalMapping initialization with defaults."""
        parameters = {"test": "value"}

        mapping = CorticalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            mapping_type="test",
            parameters=parameters,
        )

        assert mapping.name == ""
        assert mapping.description == ""
        assert mapping.id is not None  # Should be generated

    def test_transform_coordinates_not_implemented(self):
        """Test that base class transform_coordinates raises NotImplementedError."""
        mapping = CorticalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            mapping_type="test",
            parameters={},
        )

        with pytest.raises(NotImplementedError):
            mapping.transform_coordinates((0, 0, 0), (10, 10, 10), (5, 5, 5))

    def test_to_dict(self):
        """Test converting mapping to dictionary."""
        parameters = {
            "morphology_id": "test_morphology",
            "morphology_scalar": [1.0, 1.0, 1.0],
        }

        mapping = CorticalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            mapping_type="test",
            parameters=parameters,
            name="Test Mapping",
            description="Test Description",
        )

        result = mapping.to_dict()

        assert result["source_cortical_id"] == "src_area"
        assert result["target_cortical_id"] == "tgt_area"
        assert result["mapping_type"] == "test"
        assert result["parameters"] == parameters
        assert result["name"] == "Test Mapping"
        assert "id" in result

    def test_update_valid_properties(self):
        """Test updating valid mapping properties."""
        mapping = CorticalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            mapping_type="test",
            parameters={},
        )

        updates = {"name": "Updated Name", "parameters": {"new_param": "new_value"}}

        mapping.update(updates)

        assert mapping.name == "Updated Name"
        assert mapping.parameters == {"new_param": "new_value"}

    def test_update_invalid_property(self):
        """Test updating invalid mapping property."""
        mapping = CorticalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            mapping_type="test",
            parameters={},
        )

        with pytest.raises(KeyError, match="Invalid properties"):
            mapping.update({"invalid_property": "new_value"})

    def test_validate_success(self):
        """Test successful mapping validation."""
        mapping = CorticalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            mapping_type="test",
            parameters={},
            name="Test Mapping",  # Name is required for validation
        )

        # Base validation should pass for valid IDs and name
        assert mapping.validate() is True

    def test_validate_empty_source(self):
        """Test validation failure with empty source ID."""
        mapping = CorticalMapping(
            source_cortical_id="",
            target_cortical_id="tgt_area",
            mapping_type="test",
            parameters={},
        )

        assert mapping.validate() is False

    def test_validate_empty_target(self):
        """Test validation failure with empty target ID."""
        mapping = CorticalMapping(
            source_cortical_id="src_area",
            target_cortical_id="",
            mapping_type="test",
            parameters={},
        )

        assert mapping.validate() is False

    def test_validate_same_source_target(self):
        """Test validation failure with same source and target."""
        mapping = CorticalMapping(
            source_cortical_id="same_area",
            target_cortical_id="same_area",
            mapping_type="test",
            parameters={},
        )

        assert mapping.validate() is False

    def test_from_dict(self):
        """Test creating mapping from dictionary."""
        data = {
            "source_cortical_id": "src_area",
            "target_cortical_id": "tgt_area",
            "mapping_type": "test",
            "parameters": {"test": "value"},
            "name": "Test Mapping",
            "description": "Test Description",
            "id": "test_id",
        }

        mapping = CorticalMapping.from_dict(data)

        assert mapping.source_cortical_id == "src_area"
        assert mapping.target_cortical_id == "tgt_area"
        assert mapping.mapping_type == "test"
        assert mapping.parameters == {"test": "value"}
        assert mapping.name == "Test Mapping"
        assert mapping.description == "Test Description"
        assert mapping.id == "test_id"


class TestTopologicalMapping:
    """Test the TopologicalMapping class functionality."""

    def test_topological_mapping_initialization(self):
        """Test TopologicalMapping initialization."""
        mapping = TopologicalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            scale_factors=(2.0, 2.0, 2.0),
            offset=(1, 1, 1),
            plasticity_flag=True,
            psc_multiplier=1.5,
            name="Topo Mapping",
        )

        assert mapping.source_cortical_id == "src_area"
        assert mapping.target_cortical_id == "tgt_area"
        assert mapping.mapping_type == "topological"
        assert mapping.parameters["morphology_id"] == "test_morphology"
        assert mapping.parameters["morphology_scalar"] == [1.0, 1.0, 1.0]
        assert mapping.parameters["scale_factors"] == (2.0, 2.0, 2.0)
        assert mapping.parameters["offset"] == (1, 1, 1)
        assert mapping.parameters["plasticity_flag"] is True
        assert mapping.parameters["psc_multiplier"] == 1.5
        assert mapping.name == "Topo Mapping"

    def test_topological_mapping_defaults(self):
        """Test TopologicalMapping with default values."""
        mapping = TopologicalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
        )

        assert mapping.parameters["scale_factors"] == (1.0, 1.0, 1.0)
        assert mapping.parameters["offset"] == (0, 0, 0)
        assert mapping.parameters["plasticity_flag"] is False
        assert mapping.parameters["psc_multiplier"] == 1.0

    def test_transform_coordinates_basic(self):
        """Test basic coordinate transformation."""
        mapping = TopologicalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            scale_factors=(1.0, 1.0, 1.0),
            offset=(0, 0, 0),
        )

        result = mapping.transform_coordinates(
            source_position=(5, 5, 5),
            source_dimensions=(10, 10, 10),
            target_dimensions=(10, 10, 10),
        )

        # With 1:1 scaling and no offset, should be approximately the same
        assert result == (5, 5, 5)

    def test_transform_coordinates_with_scaling(self):
        """Test coordinate transformation with scaling."""
        mapping = TopologicalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            scale_factors=(2.0, 2.0, 2.0),
            offset=(0, 0, 0),
        )

        result = mapping.transform_coordinates(
            source_position=(2, 2, 2),
            source_dimensions=(5, 5, 5),
            target_dimensions=(10, 10, 10),
        )

        # With 2x scaling, position should be scaled up
        assert result[0] >= 4  # Approximately 4
        assert result[1] >= 4
        assert result[2] >= 4

    def test_transform_coordinates_with_offset(self):
        """Test coordinate transformation with offset."""
        mapping = TopologicalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            scale_factors=(1.0, 1.0, 1.0),
            offset=(2, 2, 2),
        )

        result = mapping.transform_coordinates(
            source_position=(0, 0, 0),
            source_dimensions=(5, 5, 5),
            target_dimensions=(10, 10, 10),
        )

        # With offset, position should be shifted
        assert result[0] >= 2
        assert result[1] >= 2
        assert result[2] >= 2

    def test_transform_coordinates_boundary_clamping(self):
        """Test that coordinates are clamped to target boundaries."""
        mapping = TopologicalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            scale_factors=(10.0, 10.0, 10.0),  # Large scaling
            offset=(0, 0, 0),
        )

        result = mapping.transform_coordinates(
            source_position=(4, 4, 4),
            source_dimensions=(5, 5, 5),
            target_dimensions=(5, 5, 5),
        )

        # Should be clamped to target bounds
        assert 0 <= result[0] < 5
        assert 0 <= result[1] < 5
        assert 0 <= result[2] < 5

    def test_transform_coordinates_single_dimension(self):
        """Test coordinate transformation with single-dimension areas."""
        mapping = TopologicalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            scale_factors=(1.0, 1.0, 1.0),
            offset=(0, 0, 0),
        )

        result = mapping.transform_coordinates(
            source_position=(0, 0, 0),
            source_dimensions=(1, 1, 1),  # Single voxel
            target_dimensions=(5, 5, 5),
        )

        # Should handle single dimensions gracefully
        assert isinstance(result, tuple)
        assert len(result) == 3


class TestProjectionMapping:
    """Test the ProjectionMapping class functionality."""

    def test_projection_mapping_initialization(self):
        """Test ProjectionMapping initialization."""
        mapping = ProjectionMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            projection_type="flatten",
            projection_axis=2,
            plasticity_flag=True,
            psc_multiplier=1.5,
            name="Proj Mapping",
        )

        assert mapping.source_cortical_id == "src_area"
        assert mapping.target_cortical_id == "tgt_area"
        assert mapping.mapping_type == "projection"
        assert mapping.parameters["morphology_id"] == "test_morphology"
        assert mapping.parameters["morphology_scalar"] == [1.0, 1.0, 1.0]
        assert mapping.parameters["projection_type"] == "flatten"
        assert mapping.parameters["projection_axis"] == 2
        assert mapping.parameters["plasticity_flag"] is True
        assert mapping.parameters["psc_multiplier"] == 1.5
        assert mapping.name == "Proj Mapping"

    def test_projection_mapping_defaults(self):
        """Test ProjectionMapping with default values."""
        mapping = ProjectionMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
        )

        assert mapping.parameters["projection_type"] == "flatten"
        assert mapping.parameters["projection_axis"] == 2
        assert mapping.parameters["plasticity_flag"] is False
        assert mapping.parameters["psc_multiplier"] == 1.0

    def test_transform_coordinates_flatten_z_axis(self):
        """Test coordinate transformation with flatten projection along z-axis."""
        mapping = ProjectionMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            projection_type="flatten",
            projection_axis=2,  # z-axis
        )

        result = mapping.transform_coordinates(
            source_position=(5, 3, 7),
            source_dimensions=(10, 6, 8),
            target_dimensions=(10, 6, 1),
        )

        # Should project x,y and set z to 0
        assert result[2] == 0  # z should be 0
        assert 0 <= result[0] < 10  # x should be mapped
        assert 0 <= result[1] < 6  # y should be mapped

    def test_transform_coordinates_flatten_x_axis(self):
        """Test coordinate transformation with flatten projection along x-axis."""
        mapping = ProjectionMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            projection_type="flatten",
            projection_axis=0,  # x-axis
        )

        result = mapping.transform_coordinates(
            source_position=(5, 3, 7),
            source_dimensions=(10, 6, 8),
            target_dimensions=(6, 8, 1),
        )

        # Should project y,z and ignore x
        assert result[2] == 0  # Last dimension set to 0
        assert 0 <= result[0] < 6  # y mapped to first position
        assert 0 <= result[1] < 8  # z mapped to second position

    def test_transform_coordinates_expand_projection(self):
        """Test coordinate transformation with expand projection."""
        mapping = ProjectionMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            projection_type="expand",
            projection_axis=2,  # z-axis
        )

        result = mapping.transform_coordinates(
            source_position=(5, 3, 0),  # 2D source
            source_dimensions=(10, 6, 1),
            target_dimensions=(10, 6, 8),
        )

        # Should expand along z-axis
        assert 0 <= result[0] < 10  # x should be mapped
        assert 0 <= result[1] < 6  # y should be mapped
        assert result[2] == 4  # z should be middle value (8//2)

    def test_transform_coordinates_unknown_projection_type(self):
        """Test coordinate transformation with unknown projection type."""
        mapping = ProjectionMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            projection_type="unknown",
            projection_axis=2,
        )

        with pytest.raises(ValueError, match="Unknown projection type"):
            mapping.transform_coordinates(
                source_position=(5, 3, 7),
                source_dimensions=(10, 6, 8),
                target_dimensions=(10, 6, 1),
            )

    def test_transform_coordinates_boundary_clamping(self):
        """Test that projection coordinates are clamped to boundaries."""
        mapping = ProjectionMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            projection_type="flatten",
            projection_axis=2,
        )

        result = mapping.transform_coordinates(
            source_position=(9, 5, 7),  # Near boundary
            source_dimensions=(10, 6, 8),
            target_dimensions=(5, 3, 1),  # Smaller target
        )

        # Should be clamped to target bounds
        assert 0 <= result[0] < 5
        assert 0 <= result[1] < 3
        assert 0 <= result[2] < 1

    def test_transform_coordinates_single_dimension_source(self):
        """Test projection with single-dimension source."""
        mapping = ProjectionMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            projection_type="flatten",
            projection_axis=2,
        )

        result = mapping.transform_coordinates(
            source_position=(0, 0, 0),
            source_dimensions=(1, 1, 1),  # Single voxel
            target_dimensions=(5, 5, 1),
        )

        # Should handle single dimensions gracefully
        assert isinstance(result, tuple)
        assert len(result) == 3
        assert 0 <= result[0] < 5
        assert 0 <= result[1] < 5


class TestMappingFactory:
    """Test the create_cortical_mapping factory function."""

    def test_create_topological_mapping(self):
        """Test creating a topological mapping."""
        mapping = create_cortical_mapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            mapping_type="topological",
            plasticity_flag=True,
            psc_multiplier=1.5,
            name="Test Mapping",
        )

        assert isinstance(mapping, TopologicalMapping)
        assert mapping.source_cortical_id == "src_area"
        assert mapping.target_cortical_id == "tgt_area"
        assert mapping.parameters["morphology_id"] == "test_morphology"
        assert mapping.parameters["plasticity_flag"] is True
        assert mapping.parameters["psc_multiplier"] == 1.5
        assert mapping.name == "Test Mapping"

    def test_create_topological_mapping_with_properties(self):
        """Test creating a topological mapping with custom properties."""
        properties = {
            "scale_factors": (2.0, 2.0, 2.0),
            "offset": (1, 1, 1),
            "custom_prop": "custom_value",
        }

        mapping = create_cortical_mapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            mapping_type="topological",
            properties=properties,
        )

        assert isinstance(mapping, TopologicalMapping)
        assert mapping.parameters["scale_factors"] == (2.0, 2.0, 2.0)
        assert mapping.parameters["offset"] == (1, 1, 1)
        assert mapping.parameters["custom_prop"] == "custom_value"

    def test_create_projection_mapping(self):
        """Test creating a projection mapping."""
        mapping = create_cortical_mapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            mapping_type="projection",
            plasticity_flag=True,
            psc_multiplier=1.5,
            name="Test Projection",
        )

        assert isinstance(mapping, ProjectionMapping)
        assert mapping.source_cortical_id == "src_area"
        assert mapping.target_cortical_id == "tgt_area"
        assert mapping.parameters["morphology_id"] == "test_morphology"
        assert mapping.parameters["plasticity_flag"] is True
        assert mapping.parameters["psc_multiplier"] == 1.5
        assert mapping.name == "Test Projection"

    def test_create_projection_mapping_with_properties(self):
        """Test creating a projection mapping with custom properties."""
        properties = {
            "projection_type": "expand",
            "projection_axis": 1,
            "custom_prop": "custom_value",
        }

        mapping = create_cortical_mapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            mapping_type="projection",
            properties=properties,
        )

        assert isinstance(mapping, ProjectionMapping)
        assert mapping.parameters["projection_type"] == "expand"
        assert mapping.parameters["projection_axis"] == 1
        assert mapping.parameters["custom_prop"] == "custom_value"

    def test_create_unknown_mapping_type(self):
        """Test creating a mapping with unknown type."""
        with pytest.raises(ValueError, match="Unknown mapping type"):
            create_cortical_mapping(
                source_cortical_id="src_area",
                target_cortical_id="tgt_area",
                morphology_id="test_morphology",
                morphology_scalar=[1.0, 1.0, 1.0],
                mapping_type="unknown_type",
            )


class TestCustomMappingRegistration:
    """Test the register_custom_mapping function."""

    def test_register_valid_custom_mapping(self):
        """Test registering a valid custom mapping."""

        class TestCustomMapping(CorticalMapping):
            def transform_coordinates(
                self, source_position, source_dimensions, target_dimensions
            ):
                return source_position

        original_count = len(MAPPING_TYPES)

        with patch("feagi.bdu.connectivity.cortical_mappings.logger") as mock_logger:
            register_custom_mapping("test_custom", TestCustomMapping)

        assert len(MAPPING_TYPES) == original_count + 1
        assert "test_custom" in MAPPING_TYPES
        assert MAPPING_TYPES["test_custom"] == TestCustomMapping
        mock_logger.info.assert_called_once_with(
            "Registered custom mapping type: test_custom"
        )

        # Clean up
        del MAPPING_TYPES["test_custom"]

    def test_register_invalid_custom_mapping(self):
        """Test registering an invalid custom mapping."""

        class NotAMapping:
            pass

        with pytest.raises(
            TypeError, match="Custom mapping must be a subclass of CorticalMapping"
        ):
            register_custom_mapping("invalid", NotAMapping)

    def test_register_custom_mapping_overwrites_existing(self):
        """Test that registering overwrites existing mapping types."""

        class NewTopologicalMapping(CorticalMapping):
            def transform_coordinates(
                self, source_position, source_dimensions, target_dimensions
            ):
                return (0, 0, 0)

        original_topological = MAPPING_TYPES["topological"]

        try:
            register_custom_mapping("topological", NewTopologicalMapping)

            assert MAPPING_TYPES["topological"] == NewTopologicalMapping
            assert MAPPING_TYPES["topological"] != original_topological
        finally:
            # Restore original
            MAPPING_TYPES["topological"] = original_topological


class TestEdgeCasesAndErrorHandling:
    """Test edge cases and error handling scenarios."""

    def test_mapping_restriction_empty_lists(self):
        """Test MappingRestriction with empty lists."""
        restriction = MappingRestriction(
            "IPU", "OPU", restricted_morphologies=[], disallowed_morphologies=[]
        )

        assert restriction.has_restricted_morphologies() is False
        assert restriction.has_disallowed_morphologies() is False
        assert restriction.is_morphology_allowed("any_morphology") is True

    def test_registry_case_sensitivity_edge_cases(self):
        """Test registry case sensitivity with various cases."""
        registry = CorticalMappingRestrictionsRegistry()

        restriction = MappingRestriction("MiXeD_CaSe", "AnOtHeR_CaSe")
        registry.add_restriction(restriction)

        # Should find with exact case only
        assert registry.get_restriction("MiXeD_CaSe", "AnOtHeR_CaSe") == restriction
        # Should not find with different cases
        assert registry.get_restriction("mixed_case", "another_case") is None
        assert registry.get_restriction("MIXED_CASE", "ANOTHER_CASE") is None

    def test_coordinate_transformation_zero_dimensions(self):
        """Test coordinate transformation with zero dimensions."""
        mapping = TopologicalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
        )

        # Should handle zero dimensions gracefully
        result = mapping.transform_coordinates(
            source_position=(0, 0, 0),
            source_dimensions=(0, 1, 1),
            target_dimensions=(1, 1, 1),
        )

        assert isinstance(result, tuple)
        assert len(result) == 3

    def test_mapping_update_parameters_merge(self):
        """Test that updating parameters merges with existing ones."""
        mapping = CorticalMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            mapping_type="test",
            parameters={"existing": "value", "keep": "this"},
        )

        mapping.update({"parameters": {"existing": "new_value", "added": "parameter"}})

        expected_parameters = {
            "existing": "new_value",  # Updated
            "keep": "this",  # Preserved
            "added": "parameter",  # Added
        }

        assert mapping.parameters == expected_parameters

    def test_projection_mapping_edge_axis_values(self):
        """Test projection mapping with edge axis values."""
        # Test with axis 0
        mapping = ProjectionMapping(
            source_cortical_id="src_area",
            target_cortical_id="tgt_area",
            morphology_id="test_morphology",
            morphology_scalar=[1.0, 1.0, 1.0],
            projection_axis=0,
        )

        result = mapping.transform_coordinates(
            source_position=(1, 2, 3),
            source_dimensions=(2, 3, 4),
            target_dimensions=(3, 4, 1),
        )

        assert isinstance(result, tuple)
        assert len(result) == 3

    def test_mapping_from_dict_missing_fields(self):
        """Test creating mapping from dictionary with missing fields."""
        data = {
            "source_cortical_id": "src_area",
            "target_cortical_id": "tgt_area",
            "mapping_type": "test",
            "parameters": {},
            # Missing name, description, id
        }

        mapping = CorticalMapping.from_dict(data)

        assert mapping.source_cortical_id == "src_area"
        assert mapping.target_cortical_id == "tgt_area"
        assert mapping.name == ""
        assert mapping.description == ""
        assert mapping.id is not None
