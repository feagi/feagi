"""Cortical mappings between different brain areas.

This module defines the spatial and topological relationships between
different cortical areas, enabling coordinate transformations and mappings.
"""

from typing import Dict, Any, List, Tuple, Callable, Optional, Union
import numpy as np
import logging
import uuid

logger = logging.getLogger(__name__)


class CorticalMapping:
    """Base class for mappings between cortical areas."""
    
    def __init__(self, source_area_id: str, target_area_id: str, mapping_type: str,
                 parameters: Dict[str, Any]):
        """Initialize a cortical mapping.
        
        Args:
            source_area_id: ID of the source cortical area
            target_area_id: ID of the target cortical area
            mapping_type: Type of mapping (e.g., "topological", "projection")
            parameters: Mapping-specific parameters
        """
        self.id = str(uuid.uuid4())
        self.source_area_id = source_area_id
        self.target_area_id = target_area_id
        self.mapping_type = mapping_type
        self.parameters = parameters
    
    def transform_coordinates(self, source_position: Tuple[int, int, int], 
                             source_dimensions: Tuple[int, int, int],
                             target_dimensions: Tuple[int, int, int]) -> Tuple[int, int, int]:
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
            "source_area_id": self.source_area_id,
            "target_area_id": self.target_area_id,
            "mapping_type": self.mapping_type,
            "parameters": self.parameters
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]):
        """Create a mapping from dictionary data.
        
        Args:
            data: Dictionary representation of the mapping
            
        Returns:
            Instantiated mapping object
        """
        mapping = cls(
            source_area_id=data["source_area_id"],
            target_area_id=data["target_area_id"],
            mapping_type=data["mapping_type"],
            parameters=data["parameters"]
        )
        mapping.id = data.get("id", str(uuid.uuid4()))
        return mapping


class TopologicalMapping(CorticalMapping):
    """Mapping that preserves spatial relationships between areas."""
    
    def __init__(self, source_area_id: str, target_area_id: str, 
                 scale_factors: Optional[Tuple[float, float, float]] = None,
                 offset: Optional[Tuple[int, int, int]] = None,
                 parameters: Optional[Dict[str, Any]] = None):
        """Initialize a topological mapping.
        
        Args:
            source_area_id: ID of the source cortical area
            target_area_id: ID of the target cortical area
            scale_factors: Scale factors for each dimension (default: (1.0, 1.0, 1.0))
            offset: Positional offset in target space (default: (0, 0, 0))
            parameters: Additional parameters (optional)
        """
        params = parameters or {}
        params.update({
            "scale_factors": scale_factors or (1.0, 1.0, 1.0),
            "offset": offset or (0, 0, 0)
        })
        super().__init__(source_area_id, target_area_id, "topological", params)
    
    def transform_coordinates(self, source_position: Tuple[int, int, int], 
                             source_dimensions: Tuple[int, int, int],
                             target_dimensions: Tuple[int, int, int]) -> Tuple[int, int, int]:
        """Transform coordinates based on scale factors and offsets.
        
        Args:
            source_position: Position in the source area (x, y, z)
            source_dimensions: Dimensions of the source area (width, height, depth)
            target_dimensions: Dimensions of the target area (width, height, depth)
            
        Returns:
            Transformed position in target space (x, y, z)
        """
        # Normalize coordinates to 0..1 range based on source dimensions
        norm_x = source_position[0] / (source_dimensions[0] - 1) if source_dimensions[0] > 1 else 0
        norm_y = source_position[1] / (source_dimensions[1] - 1) if source_dimensions[1] > 1 else 0
        norm_z = source_position[2] / (source_dimensions[2] - 1) if source_dimensions[2] > 1 else 0
        
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
    
    def __init__(self, source_area_id: str, target_area_id: str, 
                 projection_type: str = "flatten",
                 projection_axis: int = 2,
                 parameters: Optional[Dict[str, Any]] = None):
        """Initialize a projection mapping.
        
        Args:
            source_area_id: ID of the source cortical area
            target_area_id: ID of the target cortical area
            projection_type: Type of projection ("flatten" or "expand")
            projection_axis: Axis to project along (0=x, 1=y, 2=z)
            parameters: Additional parameters (optional)
        """
        params = parameters or {}
        params.update({
            "projection_type": projection_type,
            "projection_axis": projection_axis
        })
        super().__init__(source_area_id, target_area_id, "projection", params)
    
    def transform_coordinates(self, source_position: Tuple[int, int, int], 
                             source_dimensions: Tuple[int, int, int],
                             target_dimensions: Tuple[int, int, int]) -> Tuple[int, int, int]:
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
            source_position[0] / (source_dimensions[0] - 1) if source_dimensions[0] > 1 else 0,
            source_position[1] / (source_dimensions[1] - 1) if source_dimensions[1] > 1 else 0,
            source_position[2] / (source_dimensions[2] - 1) if source_dimensions[2] > 1 else 0
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
                    result[i] = int(norm_pos[src_idx] * (target_dimensions[i] - 1) + 0.5)
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
MAPPING_TYPES = {
    "topological": TopologicalMapping,
    "projection": ProjectionMapping
}


def create_cortical_mapping(source_area_id: str, target_area_id: str, 
                          mapping_type: str, parameters: Dict[str, Any]) -> CorticalMapping:
    """Factory function to create a cortical mapping.
    
    Args:
        source_area_id: ID of the source cortical area
        target_area_id: ID of the target cortical area
        mapping_type: Type of mapping to create
        parameters: Mapping-specific parameters
        
    Returns:
        Instantiated mapping object
        
    Raises:
        ValueError: If mapping_type is unknown
    """
    if mapping_type not in MAPPING_TYPES:
        raise ValueError(f"Unknown mapping type: {mapping_type}")
    
    mapping_class = MAPPING_TYPES[mapping_type]
    return mapping_class(source_area_id, target_area_id, **parameters)


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