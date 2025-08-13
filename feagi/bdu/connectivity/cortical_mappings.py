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

import logging
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

"""Cortical mapping rules and constraints for the BDU.

This module provides the framework for defining and enforcing rules
about how cortical areas can be connected to each other.
"""


class MappingRestriction:
    """Defines restrictions for mappings between cortical area types."""

    def __init__(
        self,
        source_type: str,
        destination_type: str,
        restricted_morphologies: Optional[List[str]] = None,
        disallowed_morphologies: Optional[List[str]] = None,
        max_mappings: int = -1,
        allow_scalar_change: bool = True,
        allow_psp_change: bool = True,
        allow_inhibitory_change: bool = True,
        allow_plasticity_change: bool = True,
        allow_plasticity_constant_change: bool = True,
        allow_ltp_change: bool = True,
        allow_ltd_change: bool = True,
    ):
        """Initialize mapping restriction.

        Args:
            source_type: Source cortical area type (e.g., "IPU", "OPU", "CUSTOM", etc.)
            destination_type: Destination cortical area type
            restricted_morphologies: List of morphology names that are allowed
                (None = all allowed)
            disallowed_morphologies: List of morphology names that are forbidden
                (None = none forbidden)
            max_mappings: Maximum number of mappings allowed (-1 = unlimited)
            allow_scalar_change: Whether scalar values can be modified
            allow_psp_change: Whether post-synaptic potential can be modified
            allow_inhibitory_change: Whether inhibitory flag can be modified
            allow_plasticity_change: Whether plasticity can be modified
            allow_plasticity_constant_change: Whether plasticity constant can be
                modified
            allow_ltp_change: Whether LTP multiplier can be modified
            allow_ltd_change: Whether LTD multiplier can be modified
        """
        self.source_type = source_type
        self.destination_type = destination_type
        self.restricted_morphologies = restricted_morphologies or []
        self.disallowed_morphologies = disallowed_morphologies or []
        self.max_mappings = max_mappings
        self.allow_scalar_change = allow_scalar_change
        self.allow_psp_change = allow_psp_change
        self.allow_inhibitory_change = allow_inhibitory_change
        self.allow_plasticity_change = allow_plasticity_change
        self.allow_plasticity_constant_change = allow_plasticity_constant_change
        self.allow_ltp_change = allow_ltp_change
        self.allow_ltd_change = allow_ltd_change

    def has_restricted_morphologies(self) -> bool:
        """Check if this restriction limits morphologies to a specific set."""
        return len(self.restricted_morphologies) > 0

    def has_disallowed_morphologies(self) -> bool:
        """Check if this restriction forbids specific morphologies."""
        return len(self.disallowed_morphologies) > 0

    def has_max_mappings(self) -> bool:
        """Check if this restriction limits the number of mappings."""
        return self.max_mappings != -1

    def is_morphology_allowed(self, morphology_name: str) -> bool:
        """Check if a morphology is allowed under this restriction.

        Args:
            morphology_name: Name of the morphology to check

        Returns:
            True if the morphology is allowed, False otherwise
        """
        # If there are restricted morphologies, only those are allowed
        if self.has_restricted_morphologies():
            return morphology_name in self.restricted_morphologies

        # If there are disallowed morphologies, anything except those is allowed
        if self.has_disallowed_morphologies():
            return morphology_name not in self.disallowed_morphologies

        # No restrictions, all morphologies allowed
        return True

    def to_dict(self) -> Dict[str, Any]:
        """Convert restriction to dictionary for API response."""
        return {
            "source_type": self.source_type,
            "destination_type": self.destination_type,
            "restricted_morphologies": self.restricted_morphologies,
            "disallowed_morphologies": self.disallowed_morphologies,
            "max_mappings": self.max_mappings,
            "allow_scalar_change": self.allow_scalar_change,
            "allow_psp_change": self.allow_psp_change,
            "allow_inhibitory_change": self.allow_inhibitory_change,
            "allow_plasticity_change": self.allow_plasticity_change,
            "allow_plasticity_constant_change": self.allow_plasticity_constant_change,
            "allow_ltp_change": self.allow_ltp_change,
            "allow_ltd_change": self.allow_ltd_change,
            "has_restricted_morphologies": self.has_restricted_morphologies(),
            "has_disallowed_morphologies": self.has_disallowed_morphologies(),
            "has_max_mappings": self.has_max_mappings(),
        }


class MappingDefault:
    """Defines default settings for mappings between cortical area types."""

    def __init__(
        self, source_type: str, destination_type: str, default_morphology: str
    ):
        """Initialize mapping default.

        Args:
            source_type: Source cortical area type
            destination_type: Destination cortical area type
            default_morphology: Default morphology name to use
        """
        self.source_type = source_type
        self.destination_type = destination_type
        self.default_morphology = default_morphology

    def to_dict(self) -> Dict[str, Any]:
        """Convert default to dictionary for API response."""
        return {
            "source_type": self.source_type,
            "destination_type": self.destination_type,
            "default_morphology": self.default_morphology,
        }


class CorticalMappingRestrictionsRegistry:
    """Registry for managing mapping restrictions and defaults between cortical area types."""

    def __init__(self):
        """Initialize the restrictions registry with default restrictions."""
        self.restrictions: List[MappingRestriction] = []
        self.defaults: List[MappingDefault] = []
        self._initialize_default_restrictions()

    def _initialize_default_restrictions(self):
        """Initialize the default restrictions based on the current Godot configuration."""

        # Memory → OPU: Restricted to "projector" morphology only
        self.add_restriction(
            MappingRestriction(
                source_type="MEMORY",
                destination_type="OPU",
                restricted_morphologies=["projector"],
                allow_scalar_change=False,
                allow_psp_change=True,
                allow_inhibitory_change=True,
                allow_plasticity_change=True,
                allow_plasticity_constant_change=True,
                allow_ltp_change=True,
                allow_ltd_change=True,
            )
        )

        # OPU → Memory: Restricted to "memory" morphology only, very limited changes
        self.add_restriction(
            MappingRestriction(
                source_type="OPU",
                destination_type="MEMORY",
                restricted_morphologies=["memory"],
                max_mappings=1,
                allow_scalar_change=False,
                allow_psp_change=False,
                allow_inhibitory_change=False,
                allow_plasticity_change=False,
                allow_plasticity_constant_change=False,
                allow_ltp_change=False,
                allow_ltd_change=False,
            )
        )

        # OPU → OPU: Disallow "memory" morphology, allow most other changes
        self.add_restriction(
            MappingRestriction(
                source_type="OPU",
                destination_type="OPU",
                disallowed_morphologies=["memory"],
                allow_scalar_change=True,
                allow_psp_change=True,
                allow_inhibitory_change=True,
                allow_plasticity_change=True,
                allow_plasticity_constant_change=True,
                allow_ltp_change=True,
                allow_ltd_change=True,
            )
        )

        # Default settings
        self.add_default(MappingDefault("OPU", "MEMORY", "memory"))
        self.add_default(MappingDefault("OPU", "OPU", "projector"))

    def add_restriction(self, restriction: MappingRestriction):
        """Add a mapping restriction to the registry."""
        self.restrictions.append(restriction)

    def add_default(self, default: MappingDefault):
        """Add a mapping default to the registry."""
        self.defaults.append(default)

    def get_restriction(
        self, source_type: str, destination_type: str
    ) -> Optional[MappingRestriction]:
        """Get the restriction for a specific source/destination combination.

        Args:
            source_type: Source cortical area type
            destination_type: Destination cortical area type

        Returns:
            MappingRestriction if found, None otherwise
        """
        for restriction in self.restrictions:
            # Exact match
            if (
                restriction.source_type == source_type
                and restriction.destination_type == destination_type
            ):
                return restriction

            # Wildcard match (UNKNOWN means any type)
            if (
                restriction.source_type == "UNKNOWN"
                or restriction.source_type == source_type
            ) and (
                restriction.destination_type == "UNKNOWN"
                or restriction.destination_type == destination_type
            ):
                return restriction

        return None

    def get_default(
        self, source_type: str, destination_type: str
    ) -> Optional[MappingDefault]:
        """Get the default morphology for a specific source/destination combination.

        Args:
            source_type: Source cortical area type
            destination_type: Destination cortical area type

        Returns:
            MappingDefault if found, None otherwise
        """
        for default in self.defaults:
            # Exact match
            if (
                default.source_type == source_type
                and default.destination_type == destination_type
            ):
                return default

            # Wildcard match (UNKNOWN means any type)
            if (
                default.source_type == "UNKNOWN" or default.source_type == source_type
            ) and (
                default.destination_type == "UNKNOWN"
                or default.destination_type == destination_type
            ):
                return default

        return None

    def get_all_restrictions(self) -> List[Dict[str, Any]]:
        """Get all restrictions as dictionaries for API response."""
        return [restriction.to_dict() for restriction in self.restrictions]

    def get_all_defaults(self) -> List[Dict[str, Any]]:
        """Get all defaults as dictionaries for API response."""
        return [default.to_dict() for default in self.defaults]

    def to_dict(self) -> Dict[str, Any]:
        """Convert entire registry to dictionary for API response."""
        return {
            "restrictions": self.get_all_restrictions(),
            "defaults": self.get_all_defaults(),
        }


# Global registry instance
_mapping_restrictions_registry = None


def get_mapping_restrictions_registry() -> CorticalMappingRestrictionsRegistry:
    """Get the global mapping restrictions registry."""
    global _mapping_restrictions_registry
    if _mapping_restrictions_registry is None:
        _mapping_restrictions_registry = CorticalMappingRestrictionsRegistry()
    return _mapping_restrictions_registry


class CorticalMapping:
    """Base class for cortical area mappings."""

    def __init__(
        self,
        source_cortical_id: str,
        target_cortical_id: str,
        mapping_type: str,
        parameters: Dict[str, Any],
        name: str = "",
        description: str = "",
        mapping_id: Optional[str] = None,
    ):
        """Initialize a cortical mapping.

        Args:
            source_cortical_id: ID of the source cortical area
            target_cortical_id: ID of the target cortical area
            mapping_type: Type of mapping
            parameters: Mapping-specific parameters
            name: Human-readable name for this mapping
            description: Optional description of the mapping
            mapping_id: Unique identifier for this mapping (optional, generated if not provided)
        """
        self.source_cortical_id = source_cortical_id
        self.target_cortical_id = target_cortical_id
        self.mapping_type = mapping_type
        self.parameters = parameters
        self.name = name
        self.description = description
        self.enabled = True
        self.morphology_id = None  # Associated morphology template
        self.id = (
            mapping_id
            if mapping_id
            else f"{source_cortical_id}_{target_cortical_id}_{mapping_type}"
        )

    def transform_coordinates(
        self,
        source_position: Tuple[int, int, int],
        source_dimensions: Tuple[int, int, int],
        target_dimensions: Tuple[int, int, int],
    ) -> Tuple[int, int, int]:
        """Transform coordinates from source to target space.

        Args:
            source_position: Position in the source area (x, y, z)
            source_dimensions: Dimensions of the source area (width, height, depth)
            target_dimensions: Dimensions of the target area (width, height, depth)

        Returns:
            Transformed position in target space (x, y, z)
        """
        raise NotImplementedError("Subclasses must implement transform_coordinates()")

    def to_dict(self) -> Dict[str, Any]:
        """Convert mapping to dictionary for serialization.

        Returns:
            Dictionary representation of the mapping
        """
        return {
            "id": self.id,
            "name": self.name,
            "source_cortical_id": self.source_cortical_id,
            "target_cortical_id": self.target_cortical_id,
            "mapping_type": self.mapping_type,
            "parameters": self.parameters,
        }

    def update(self, updates: Dict[str, Any]) -> None:
        """Update mapping properties.

        Args:
            updates: Dictionary of properties to update

        Raises:
            KeyError: If an invalid property is specified
        """
        valid_props = {
            "name",
            "source_cortical_id",
            "target_cortical_id",
            "mapping_type",
            "parameters",
        }
        invalid_props = set(updates.keys()) - valid_props

        if invalid_props:
            raise KeyError(f"Invalid properties: {invalid_props}")

        if "name" in updates:
            self.name = updates["name"]

        if "source_cortical_id" in updates:
            self.source_cortical_id = updates["source_cortical_id"]

        if "target_cortical_id" in updates:
            self.target_cortical_id = updates["target_cortical_id"]

        if "mapping_type" in updates:
            self.mapping_type = updates["mapping_type"]

        if "parameters" in updates:
            self.parameters.update(updates["parameters"])

    def validate(self) -> bool:
        """Validate that the mapping has all required properties set.

        Returns:
            True if the mapping is valid, False otherwise
        """
        # Check required fields
        if (
            not self.name
            or not self.source_cortical_id
            or not self.target_cortical_id
            or not self.mapping_type
        ):
            return False

        # All checks passed
        return True

    @classmethod
    def from_dict(cls, data: Dict[str, Any]):
        """Create a mapping from dictionary data.

        Args:
            data: Dictionary representation of the mapping

        Returns:
            Instantiated mapping object
        """
        mapping = cls(
            source_cortical_id=data["source_cortical_id"],
            target_cortical_id=data["target_cortical_id"],
            mapping_type=data["mapping_type"],
            parameters=data["parameters"],
            name=data.get("name", ""),
            description=data.get("description", ""),
            mapping_id=data.get("id"),
        )
        return mapping


class TopologicalMapping(CorticalMapping):
    """Mapping that preserves spatial relationships between areas."""

    def __init__(
        self,
        source_cortical_id: str,
        target_cortical_id: str,
        morphology_id: str,
        morphology_scalar: List[float],
        scale_factors: Optional[Tuple[float, float, float]] = None,
        offset: Optional[Tuple[int, int, int]] = None,
        plasticity_flag: bool = False,
        psc_multiplier: float = 1.0,
        name: str = "",
        mapping_id: Optional[str] = None,
        properties: Optional[Dict[str, Any]] = None,
    ):
        """Initialize a topological mapping.

        Args:
            source_cortical_id: ID of the source cortical area
            target_cortical_id: ID of the target cortical area
            morphology_id: ID of the morphology to use
            morphology_scalar: Scaling factors for the morphology [x, y, z]
            scale_factors: Scale factors for each dimension (default: (1.0, 1.0, 1.0))
            offset: Positional offset in target space (default: (0, 0, 0))
            plasticity_flag: Whether synapses in this mapping should be plastic
            psc_multiplier: Multiplier for post-synaptic currents
            name: Human-readable name for this mapping
            mapping_id: Unique identifier for this mapping (optional, generated if not provided)
            properties: Additional custom properties (optional)
        """
        params = properties or {}
        params.update(
            {
                "scale_factors": scale_factors or (1.0, 1.0, 1.0),
                "offset": offset or (0, 0, 0),
                "morphology_id": morphology_id,
                "morphology_scalar": morphology_scalar,
                "plasticity_flag": plasticity_flag,
                "psc_multiplier": psc_multiplier,
            }
        )

        super().__init__(
            source_cortical_id=source_cortical_id,
            target_cortical_id=target_cortical_id,
            mapping_type="topological",
            parameters=params,
            name=name,
            mapping_id=mapping_id,
        )

    def transform_coordinates(
        self,
        source_position: Tuple[int, int, int],
        source_dimensions: Tuple[int, int, int],
        target_dimensions: Tuple[int, int, int],
    ) -> Tuple[int, int, int]:
        """Transform coordinates based on scale factors and offsets.

        Args:
            source_position: Position in the source area (x, y, z)
            source_dimensions: Dimensions of the source area (width, height, depth)
            target_dimensions: Dimensions of the target area (width, height, depth)

        Returns:
            Transformed position in target space (x, y, z)
        """
        # Normalize coordinates to 0..1 range based on source dimensions
        norm_x = (
            source_position[0] / (source_dimensions[0] - 1)
            if source_dimensions[0] > 1
            else 0
        )
        norm_y = (
            source_position[1] / (source_dimensions[1] - 1)
            if source_dimensions[1] > 1
            else 0
        )
        norm_z = (
            source_position[2] / (source_dimensions[2] - 1)
            if source_dimensions[2] > 1
            else 0
        )

        # Scale by scale factors
        scale_factors = self.parameters["scale_factors"]
        scaled_x = norm_x * scale_factors[0]
        scaled_y = norm_y * scale_factors[1]
        scaled_z = norm_z * scale_factors[2]

        # Convert back to target coordinates
        target_x = int(scaled_x * (target_dimensions[0] - 1) + 0.5)
        target_y = int(scaled_y * (target_dimensions[1] - 1) + 0.5)
        target_z = int(scaled_z * (target_dimensions[2] - 1) + 0.5)

        # Apply offset
        offset = self.parameters["offset"]
        final_x = min(max(target_x + offset[0], 0), target_dimensions[0] - 1)
        final_y = min(max(target_y + offset[1], 0), target_dimensions[1] - 1)
        final_z = min(max(target_z + offset[2], 0), target_dimensions[2] - 1)

        return (final_x, final_y, final_z)


class ProjectionMapping(CorticalMapping):
    """Mapping that projects from a higher-dimensional space to a lower one or vice versa."""

    def __init__(
        self,
        source_cortical_id: str,
        target_cortical_id: str,
        morphology_id: str,
        morphology_scalar: List[float],
        projection_type: str = "flatten",
        projection_axis: int = 2,
        plasticity_flag: bool = False,
        psc_multiplier: float = 1.0,
        name: str = "",
        mapping_id: Optional[str] = None,
        properties: Optional[Dict[str, Any]] = None,
    ):
        """Initialize a projection mapping.

        Args:
            source_cortical_id: ID of the source cortical area
            target_cortical_id: ID of the target cortical area
            morphology_id: ID of the morphology to use
            morphology_scalar: Scaling factors for the morphology [x, y, z]
            projection_type: Type of projection ("flatten" or "expand")
            projection_axis: Axis to project along (0=x, 1=y, 2=z)
            plasticity_flag: Whether synapses in this mapping should be plastic
            psc_multiplier: Multiplier for post-synaptic currents
            name: Human-readable name for this mapping
            mapping_id: Unique identifier for this mapping (optional, generated if not provided)
            properties: Additional custom properties (optional)
        """
        params = properties or {}
        params.update(
            {
                "projection_type": projection_type,
                "projection_axis": projection_axis,
                "morphology_id": morphology_id,
                "morphology_scalar": morphology_scalar,
                "plasticity_flag": plasticity_flag,
                "psc_multiplier": psc_multiplier,
            }
        )

        super().__init__(
            source_cortical_id=source_cortical_id,
            target_cortical_id=target_cortical_id,
            mapping_type="projection",
            parameters=params,
            name=name,
            mapping_id=mapping_id,
        )

    def transform_coordinates(
        self,
        source_position: Tuple[int, int, int],
        source_dimensions: Tuple[int, int, int],
        target_dimensions: Tuple[int, int, int],
    ) -> Tuple[int, int, int]:
        """Transform coordinates based on projection.

        Args:
            source_position: Position in the source area (x, y, z)
            source_dimensions: Dimensions of the source area (width, height, depth)
            target_dimensions: Dimensions of the target area (width, height, depth)

        Returns:
            Transformed position in target space (x, y, z)
        """
        projection_type = self.parameters["projection_type"]
        axis = self.parameters["projection_axis"]

        # Create a normalized position
        norm_pos = [
            (
                source_position[0] / (source_dimensions[0] - 1)
                if source_dimensions[0] > 1
                else 0
            ),
            (
                source_position[1] / (source_dimensions[1] - 1)
                if source_dimensions[1] > 1
                else 0
            ),
            (
                source_position[2] / (source_dimensions[2] - 1)
                if source_dimensions[2] > 1
                else 0
            ),
        ]

        if projection_type == "flatten":
            # Flatten by ignoring the specified axis
            # If projecting along z (axis=2), we'll map x,y and set z to 0
            result = [0, 0, 0]
            idx = 0
            for i in range(3):
                if i != axis:
                    result[idx] = int(norm_pos[i] * (target_dimensions[idx] - 1) + 0.5)
                    idx += 1

            # Handle the remaining dimension (center the projection)
            result[2] = 0  # Usually project to z=0

        elif projection_type == "expand":
            # Expand by duplicating along the specified axis
            # If expanding along z (axis=2), we'll map x,y and replicate across z
            result = [0, 0, 0]
            src_idx = 0
            for i in range(3):
                if i != axis:
                    result[i] = int(
                        norm_pos[src_idx] * (target_dimensions[i] - 1) + 0.5
                    )
                    src_idx += 1
                else:
                    # For the expanded dimension, use middle value
                    result[i] = target_dimensions[i] // 2
        else:
            raise ValueError(f"Unknown projection type: {projection_type}")

        # Ensure within target bounds
        for i in range(3):
            result[i] = min(max(result[i], 0), target_dimensions[i] - 1)

        return tuple(result)


# Registry of available mapping types
MAPPING_TYPES = {"topological": TopologicalMapping, "projection": ProjectionMapping}


def create_cortical_mapping(
    source_cortical_id: str,
    target_cortical_id: str,
    morphology_id: str,
    morphology_scalar: List[float],
    mapping_type: str = "topological",
    plasticity_flag: bool = False,
    psc_multiplier: float = 1.0,
    name: str = "",
    mapping_id: Optional[str] = None,
    properties: Optional[Dict[str, Any]] = None,
) -> CorticalMapping:
    """Factory function to create a cortical mapping.

    Args:
        source_cortical_id: ID of the source cortical area
        target_cortical_id: ID of the target cortical area
        morphology_id: ID of the morphology to use
        morphology_scalar: Scaling factors for the morphology [x, y, z]
        mapping_type: Type of mapping to create (default: "topological")
        plasticity_flag: Whether synapses should be plastic
        psc_multiplier: Multiplier for post-synaptic currents
        name: Human-readable name for this mapping
        mapping_id: Unique identifier (optional, generated if not provided)
        properties: Additional custom properties (optional)

    Returns:
        Instantiated mapping object

    Raises:
        ValueError: If mapping_type is unknown
    """
    if mapping_type not in MAPPING_TYPES:
        raise ValueError(f"Unknown mapping type: {mapping_type}")

    props = properties or {}

    if mapping_type == "topological":
        scale_factors = props.pop("scale_factors", (1.0, 1.0, 1.0))
        offset = props.pop("offset", (0, 0, 0))
        return TopologicalMapping(
            source_cortical_id=source_cortical_id,
            target_cortical_id=target_cortical_id,
            morphology_id=morphology_id,
            morphology_scalar=morphology_scalar,
            scale_factors=scale_factors,
            offset=offset,
            plasticity_flag=plasticity_flag,
            psc_multiplier=psc_multiplier,
            name=name,
            mapping_id=mapping_id,
            properties=props,
        )
    elif mapping_type == "projection":
        projection_type = props.pop("projection_type", "flatten")
        projection_axis = props.pop("projection_axis", 2)
        return ProjectionMapping(
            source_cortical_id=source_cortical_id,
            target_cortical_id=target_cortical_id,
            morphology_id=morphology_id,
            morphology_scalar=morphology_scalar,
            projection_type=projection_type,
            projection_axis=projection_axis,
            plasticity_flag=plasticity_flag,
            psc_multiplier=psc_multiplier,
            name=name,
            mapping_id=mapping_id,
            properties=props,
        )
    else:
        # Generic mapping
        params = props.copy()
        params.update(
            {
                "morphology_id": morphology_id,
                "morphology_scalar": morphology_scalar,
                "plasticity_flag": plasticity_flag,
                "psc_multiplier": psc_multiplier,
            }
        )
        return CorticalMapping(
            source_cortical_id=source_cortical_id,
            target_cortical_id=target_cortical_id,
            mapping_type=mapping_type,
            parameters=params,
            name=name,
            mapping_id=mapping_id,
        )


def register_custom_mapping(mapping_name: str, mapping_class: type) -> None:
    """Register a custom mapping type.

    Args:
        mapping_name: Name for the new mapping type
        mapping_class: Class implementing the mapping

    Raises:
        TypeError: If mapping_class is not a subclass of CorticalMapping
    """
    if not issubclass(mapping_class, CorticalMapping):
        raise TypeError("Custom mapping must be a subclass of CorticalMapping")

    MAPPING_TYPES[mapping_name] = mapping_class
    logger.info(f"Registered custom mapping type: {mapping_name}")
