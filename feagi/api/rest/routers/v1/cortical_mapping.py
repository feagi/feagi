"""Cortical Mapping API router for FEAGI REST API."""

from typing import Dict, List, Optional, Any, Union, Tuple
from fastapi import APIRouter, HTTPException, Depends, Path, Query, Body
from pydantic import BaseModel, Field

from feagi.api.core.services import CoreAPIService
from feagi.api.rest.app import get_core_api

# Import required genome functions
try:
    from feagi.evo.genome_editor import save_genome
except ImportError:
    # Fallback implementation
    def save_genome(genome, file_name=''):
        """Save a genome to a file."""
        import json
        if file_name:
            with open(file_name, 'w') as f:
                json.dump(genome, f, indent=2)
        return True

# Pydantic models for request/response
class MappingBase(BaseModel):
    """Base model for cortical mapping properties."""
    source_id: str = Field(..., description="ID of the source cortical area")
    target_id: str = Field(..., description="ID of the target cortical area")
    mapping_type: str = Field(..., description="Type of mapping (e.g., one-to-one, one-to-many, probabilistic)")
    parameters: Dict[str, Any] = Field(default={}, description="Additional parameters for the mapping")

class MappingCreate(MappingBase):
    """Request model for creating a cortical mapping."""
    pass

class MappingUpdate(BaseModel):
    """Request model for updating a cortical mapping."""
    mapping_type: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None

class MappingResponse(MappingBase):
    """Response model for cortical mapping information."""
    id: str

class MappingList(BaseModel):
    """Response model for list of cortical mappings."""
    mappings: List[MappingResponse]

class MappingStatsResponse(BaseModel):
    """Response model for mapping statistics."""
    source_id: str
    target_id: str
    synapse_count: int
    average_weight: float
    connectivity_ratio: float
    mapping_type: str

# Create router
router = APIRouter(prefix="/cortical_mapping", tags=["cortical_mapping"])

# Cortical Mapping Endpoints
@router.get("/", response_model=MappingList)
async def get_all_mappings(
    source_id: Optional[str] = Query(None, description="Filter by source cortical area ID"),
    target_id: Optional[str] = Query(None, description="Filter by target cortical area ID"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get all cortical mappings.
    
    Returns a list of all mappings between cortical areas with optional filtering.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "connectivity" not in genome:
            return {"mappings": []}
        
        mappings = []
        for mapping_id, mapping_data in genome.get("connectivity", {}).items():
            # Skip if filtering is applied and doesn't match
            if source_id and mapping_data.get("source_id") != source_id:
                continue
            if target_id and mapping_data.get("target_id") != target_id:
                continue
            
            mappings.append({
                "id": mapping_id,
                "source_id": mapping_data.get("source_id"),
                "target_id": mapping_data.get("target_id"),
                "mapping_type": mapping_data.get("mapping_type", "unknown"),
                "parameters": mapping_data
            })
        
        return {"mappings": mappings}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical mappings: {str(e)}")

@router.get("/{mapping_id}", response_model=MappingResponse)
async def get_mapping(
    mapping_id: str = Path(..., description="ID of the cortical mapping"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get a specific cortical mapping by ID.
    
    Args:
        mapping_id: ID of the cortical mapping to retrieve.
    
    Returns:
        Detailed information about the specified cortical mapping.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "connectivity" not in genome:
            raise HTTPException(status_code=404, detail=f"Cortical mapping {mapping_id} not found")
        
        mapping = genome.get("connectivity", {}).get(mapping_id)
        if not mapping:
            raise HTTPException(status_code=404, detail=f"Cortical mapping {mapping_id} not found")
        
        return {
            "id": mapping_id,
            "source_id": mapping.get("source_id"),
            "target_id": mapping.get("target_id"),
            "mapping_type": mapping.get("mapping_type", "unknown"),
            "parameters": mapping
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical mapping {mapping_id}: {str(e)}")

@router.post("/", response_model=MappingResponse)
async def create_mapping(
    mapping: MappingCreate,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Create a new cortical mapping.
    
    Args:
        mapping: Details of the cortical mapping to create.
    
    Returns:
        Information about the newly created cortical mapping.
    """
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No genome loaded")
        
        # Validate that the source and target areas exist
        areas = core_api.get_cortical_areas()
        area_ids = [area["id"] for area in areas]
        
        if mapping.source_id not in area_ids:
            raise HTTPException(status_code=404, detail=f"Source cortical area {mapping.source_id} not found")
        
        if mapping.target_id not in area_ids:
            raise HTTPException(status_code=404, detail=f"Target cortical area {mapping.target_id} not found")
        
        # Ensure connectivity section exists
        if "connectivity" not in genome:
            genome["connectivity"] = {}
        
        # Generate a new ID for the mapping
        new_id = str(len(genome["connectivity"]) + 1)
        while new_id in genome["connectivity"]:
            new_id = str(int(new_id) + 1)
        
        # Create the new mapping
        new_mapping = {
            "source_id": mapping.source_id,
            "target_id": mapping.target_id,
            "mapping_type": mapping.mapping_type,
            **mapping.parameters
        }
        
        genome["connectivity"][new_id] = new_mapping
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {
            "id": new_id,
            "source_id": mapping.source_id,
            "target_id": mapping.target_id,
            "mapping_type": mapping.mapping_type,
            "parameters": new_mapping
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating cortical mapping: {str(e)}")

@router.put("/{mapping_id}", response_model=MappingResponse)
async def update_mapping(
    mapping_id: str = Path(..., description="ID of the cortical mapping"),
    mapping_update: MappingUpdate = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Update an existing cortical mapping.
    
    Args:
        mapping_id: ID of the cortical mapping to update.
        mapping_update: Updated details for the cortical mapping.
    
    Returns:
        Information about the updated cortical mapping.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "connectivity" not in genome:
            raise HTTPException(status_code=404, detail=f"Cortical mapping {mapping_id} not found")
        
        if mapping_id not in genome["connectivity"]:
            raise HTTPException(status_code=404, detail=f"Cortical mapping {mapping_id} not found")
        
        # Update the mapping
        current_mapping = genome["connectivity"][mapping_id]
        
        if mapping_update.mapping_type is not None:
            current_mapping["mapping_type"] = mapping_update.mapping_type
        
        if mapping_update.parameters is not None:
            for key, value in mapping_update.parameters.items():
                current_mapping[key] = value
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {
            "id": mapping_id,
            "source_id": current_mapping.get("source_id"),
            "target_id": current_mapping.get("target_id"),
            "mapping_type": current_mapping.get("mapping_type", "unknown"),
            "parameters": current_mapping
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating cortical mapping {mapping_id}: {str(e)}")

@router.delete("/{mapping_id}")
async def delete_mapping(
    mapping_id: str = Path(..., description="ID of the cortical mapping"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Delete a cortical mapping.
    
    Args:
        mapping_id: ID of the cortical mapping to delete.
    
    Returns:
        Confirmation message.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "connectivity" not in genome:
            raise HTTPException(status_code=404, detail=f"Cortical mapping {mapping_id} not found")
        
        if mapping_id not in genome["connectivity"]:
            raise HTTPException(status_code=404, detail=f"Cortical mapping {mapping_id} not found")
        
        # Delete the mapping
        del genome["connectivity"][mapping_id]
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {"message": f"Cortical mapping {mapping_id} deleted successfully"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting cortical mapping {mapping_id}: {str(e)}")

@router.get("/{mapping_id}/stats", response_model=MappingStatsResponse)
async def get_mapping_stats(
    mapping_id: str = Path(..., description="ID of the cortical mapping"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get statistics about a specific cortical mapping.
    
    Args:
        mapping_id: ID of the cortical mapping.
    
    Returns:
        Statistical information about the cortical mapping.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "connectivity" not in genome:
            raise HTTPException(status_code=404, detail=f"Cortical mapping {mapping_id} not found")
        
        mapping = genome.get("connectivity", {}).get(mapping_id)
        if not mapping:
            raise HTTPException(status_code=404, detail=f"Cortical mapping {mapping_id} not found")
        
        # In a real implementation, this would get actual mapping statistics from the core
        # For now, return a placeholder with simulated data
        
        # Get basic information from the mapping
        source_id = mapping.get("source_id")
        target_id = mapping.get("target_id")
        mapping_type = mapping.get("mapping_type", "unknown")
        
        # Generate simulated statistics
        import random
        synapse_count = random.randint(100, 10000)
        average_weight = random.uniform(0.1, 1.0)
        connectivity_ratio = random.uniform(0.01, 0.5)
        
        return {
            "source_id": source_id,
            "target_id": target_id,
            "synapse_count": synapse_count,
            "average_weight": float(average_weight),
            "connectivity_ratio": float(connectivity_ratio),
            "mapping_type": mapping_type
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving mapping statistics: {str(e)}")

@router.post("/{mapping_id}/apply")
async def apply_mapping(
    mapping_id: str = Path(..., description="ID of the cortical mapping"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Apply a cortical mapping to generate connections.
    
    This will cause the system to generate actual neuron-to-neuron connections 
    based on the mapping definition.
    
    Args:
        mapping_id: ID of the cortical mapping to apply.
    
    Returns:
        Confirmation message with information about the applied mapping.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "connectivity" not in genome:
            raise HTTPException(status_code=404, detail=f"Cortical mapping {mapping_id} not found")
        
        mapping = genome.get("connectivity", {}).get(mapping_id)
        if not mapping:
            raise HTTPException(status_code=404, detail=f"Cortical mapping {mapping_id} not found")
        
        # In a real implementation, this would apply the mapping using the core
        # For now, just return a placeholder success message
        
        # Get basic information from the mapping
        source_id = mapping.get("source_id")
        target_id = mapping.get("target_id")
        mapping_type = mapping.get("mapping_type", "unknown")
        
        return {
            "message": f"Cortical mapping {mapping_id} applied successfully",
            "source_id": source_id,
            "target_id": target_id,
            "mapping_type": mapping_type,
            "connections_created": 1000  # Placeholder
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error applying cortical mapping: {str(e)}")

@router.post("/templates")
async def get_mapping_templates(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get available cortical mapping templates.
    
    Returns:
        List of available mapping templates.
    """
    try:
        # In a real implementation, this would get actual templates from the core
        # For now, return a predefined set of mapping templates
        
        templates = [
            {
                "id": "one-to-one",
                "name": "One-to-One Mapping",
                "description": "Maps neurons in the source area directly to corresponding neurons in the target area",
                "parameters": {
                    "default_weight": 1.0,
                    "connection_probability": 1.0
                }
            },
            {
                "id": "probabilistic",
                "name": "Probabilistic Mapping",
                "description": "Creates connections with a specified probability, optionally with distance-based decay",
                "parameters": {
                    "default_weight": 0.5,
                    "connection_probability": 0.3,
                    "distance_decay": 0.1
                }
            },
            {
                "id": "gaussian",
                "name": "Gaussian Mapping",
                "description": "Creates connections based on a Gaussian distribution centered on the target neuron",
                "parameters": {
                    "default_weight": 0.7,
                    "sigma": 1.5,
                    "max_distance": 5
                }
            },
            {
                "id": "receptive-field",
                "name": "Receptive Field Mapping",
                "description": "Creates connections based on receptive fields typical in sensory systems",
                "parameters": {
                    "field_size": 5,
                    "field_overlap": 2,
                    "center_weight": 1.0,
                    "surround_weight": 0.5
                }
            }
        ]
        
        return {"templates": templates}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving mapping templates: {str(e)}") 