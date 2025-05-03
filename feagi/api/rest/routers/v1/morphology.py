"""Morphology API router for FEAGI REST API."""

from typing import Dict, List, Optional, Any
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
class MorphologyBase(BaseModel):
    """Base model for morphology properties."""
    name: str
    parameters: Dict[str, Any] = Field(default={}, description="Parameters of the morphology")

class MorphologyCreate(MorphologyBase):
    """Request model for creating a morphology."""
    pass

class MorphologyUpdate(BaseModel):
    """Request model for updating a morphology."""
    name: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None

class MorphologyResponse(MorphologyBase):
    """Response model for morphology information."""
    id: str

class MorphologyList(BaseModel):
    """Response model for list of morphologies."""
    morphologies: List[MorphologyResponse]

# Create router
router = APIRouter(prefix="/morphology", tags=["morphology"])

# Morphology Endpoints
@router.get("/", response_model=MorphologyList)
async def get_all_morphologies(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get all morphologies.
    
    Returns a list of all morphologies in the current genome.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "morphologies" not in genome:
            return {"morphologies": []}
        
        morphologies = []
        for morph_id, morph_data in genome.get("morphologies", {}).items():
            morphologies.append({
                "id": morph_id,
                "name": morph_data.get("name", f"Morphology {morph_id}"),
                "parameters": morph_data
            })
        
        return {"morphologies": morphologies}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving morphologies: {str(e)}")

@router.get("/{morphology_id}", response_model=MorphologyResponse)
async def get_morphology(
    morphology_id: str = Path(..., description="ID of the morphology"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get a specific morphology by ID.
    
    Args:
        morphology_id: ID of the morphology to retrieve.
    
    Returns:
        Detailed information about the specified morphology.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "morphologies" not in genome:
            raise HTTPException(status_code=404, detail=f"Morphology {morphology_id} not found")
        
        morphology = genome.get("morphologies", {}).get(morphology_id)
        if not morphology:
            raise HTTPException(status_code=404, detail=f"Morphology {morphology_id} not found")
        
        return {
            "id": morphology_id,
            "name": morphology.get("name", f"Morphology {morphology_id}"),
            "parameters": morphology
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving morphology {morphology_id}: {str(e)}")

@router.post("/", response_model=MorphologyResponse)
async def create_morphology(
    morphology: MorphologyCreate,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Create a new morphology.
    
    Args:
        morphology: Details of the morphology to create.
    
    Returns:
        Information about the newly created morphology.
    """
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No genome loaded")
        
        if "morphologies" not in genome:
            genome["morphologies"] = {}
        
        # Generate a new ID for the morphology
        new_id = str(len(genome["morphologies"]) + 1)
        while new_id in genome["morphologies"]:
            new_id = str(int(new_id) + 1)
        
        # Create the new morphology
        new_morphology = {
            "name": morphology.name,
            **morphology.parameters
        }
        
        genome["morphologies"][new_id] = new_morphology
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {
            "id": new_id,
            "name": morphology.name,
            "parameters": new_morphology
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating morphology: {str(e)}")

@router.put("/{morphology_id}", response_model=MorphologyResponse)
async def update_morphology(
    morphology_id: str = Path(..., description="ID of the morphology"),
    morphology_update: MorphologyUpdate = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Update an existing morphology.
    
    Args:
        morphology_id: ID of the morphology to update.
        morphology_update: Updated details for the morphology.
    
    Returns:
        Information about the updated morphology.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "morphologies" not in genome:
            raise HTTPException(status_code=404, detail=f"Morphology {morphology_id} not found")
        
        if morphology_id not in genome["morphologies"]:
            raise HTTPException(status_code=404, detail=f"Morphology {morphology_id} not found")
        
        # Update the morphology
        current_morphology = genome["morphologies"][morphology_id]
        
        if morphology_update.name is not None:
            current_morphology["name"] = morphology_update.name
        
        if morphology_update.parameters is not None:
            for key, value in morphology_update.parameters.items():
                current_morphology[key] = value
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {
            "id": morphology_id,
            "name": current_morphology.get("name", f"Morphology {morphology_id}"),
            "parameters": current_morphology
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating morphology {morphology_id}: {str(e)}")

@router.delete("/{morphology_id}")
async def delete_morphology(
    morphology_id: str = Path(..., description="ID of the morphology"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Delete a morphology.
    
    Args:
        morphology_id: ID of the morphology to delete.
    
    Returns:
        Confirmation message.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "morphologies" not in genome:
            raise HTTPException(status_code=404, detail=f"Morphology {morphology_id} not found")
        
        if morphology_id not in genome["morphologies"]:
            raise HTTPException(status_code=404, detail=f"Morphology {morphology_id} not found")
        
        # Check if the morphology is in use by any cortical areas
        if "blueprint" in genome:
            for area_id, area_data in genome["blueprint"].items():
                if area_data.get("morphology") == morphology_id:
                    raise HTTPException(
                        status_code=400, 
                        detail=f"Cannot delete morphology {morphology_id} as it is in use by cortical area {area_id}"
                    )
        
        # Delete the morphology
        del genome["morphologies"][morphology_id]
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {"message": f"Morphology {morphology_id} deleted successfully"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting morphology {morphology_id}: {str(e)}") 