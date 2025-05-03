"""Region API router for FEAGI REST API."""

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
class RegionBase(BaseModel):
    """Base model for brain region properties."""
    name: str
    description: Optional[str] = None
    properties: Dict[str, Any] = Field(default={}, description="Additional properties for the region")

class RegionCreate(RegionBase):
    """Request model for creating a brain region."""
    pass

class RegionUpdate(BaseModel):
    """Request model for updating a brain region."""
    name: Optional[str] = None
    description: Optional[str] = None
    properties: Optional[Dict[str, Any]] = None

class RegionResponse(RegionBase):
    """Response model for brain region information."""
    id: str
    cortical_areas: List[str] = Field(default=[], description="IDs of cortical areas in this region")

class RegionList(BaseModel):
    """Response model for list of brain regions."""
    regions: List[RegionResponse]

class CorticalAreaMapping(BaseModel):
    """Request model for mapping a cortical area to a region."""
    cortical_area_id: str

# Create router
router = APIRouter(prefix="/region", tags=["region"])

# Region Endpoints
@router.get("/", response_model=RegionList)
async def get_all_regions(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get all brain regions.
    
    Returns a list of all brain regions in the current genome.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "regions" not in genome:
            return {"regions": []}
        
        regions = []
        for region_id, region_data in genome.get("regions", {}).items():
            # Find all cortical areas in this region
            cortical_areas = []
            if "blueprint" in genome:
                for area_id, area_data in genome["blueprint"].items():
                    if area_data.get("region") == region_id:
                        cortical_areas.append(area_id)
            
            regions.append({
                "id": region_id,
                "name": region_data.get("name", f"Region {region_id}"),
                "description": region_data.get("description"),
                "properties": region_data,
                "cortical_areas": cortical_areas
            })
        
        return {"regions": regions}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving brain regions: {str(e)}")

@router.get("/{region_id}", response_model=RegionResponse)
async def get_region(
    region_id: str = Path(..., description="ID of the brain region"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get a specific brain region by ID.
    
    Args:
        region_id: ID of the brain region to retrieve.
    
    Returns:
        Detailed information about the specified brain region.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "regions" not in genome:
            raise HTTPException(status_code=404, detail=f"Brain region {region_id} not found")
        
        region = genome.get("regions", {}).get(region_id)
        if not region:
            raise HTTPException(status_code=404, detail=f"Brain region {region_id} not found")
        
        # Find all cortical areas in this region
        cortical_areas = []
        if "blueprint" in genome:
            for area_id, area_data in genome["blueprint"].items():
                if area_data.get("region") == region_id:
                    cortical_areas.append(area_id)
        
        return {
            "id": region_id,
            "name": region.get("name", f"Region {region_id}"),
            "description": region.get("description"),
            "properties": region,
            "cortical_areas": cortical_areas
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving brain region {region_id}: {str(e)}")

@router.post("/", response_model=RegionResponse)
async def create_region(
    region: RegionCreate,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Create a new brain region.
    
    Args:
        region: Details of the brain region to create.
    
    Returns:
        Information about the newly created brain region.
    """
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No genome loaded")
        
        if "regions" not in genome:
            genome["regions"] = {}
        
        # Generate a new ID for the region
        new_id = str(len(genome["regions"]) + 1)
        while new_id in genome["regions"]:
            new_id = str(int(new_id) + 1)
        
        # Create the new region
        new_region = {
            "name": region.name,
            "description": region.description,
            **region.properties
        }
        
        genome["regions"][new_id] = new_region
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {
            "id": new_id,
            "name": region.name,
            "description": region.description,
            "properties": new_region,
            "cortical_areas": []
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating brain region: {str(e)}")

@router.put("/{region_id}", response_model=RegionResponse)
async def update_region(
    region_id: str = Path(..., description="ID of the brain region"),
    region_update: RegionUpdate = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Update an existing brain region.
    
    Args:
        region_id: ID of the brain region to update.
        region_update: Updated details for the brain region.
    
    Returns:
        Information about the updated brain region.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "regions" not in genome:
            raise HTTPException(status_code=404, detail=f"Brain region {region_id} not found")
        
        if region_id not in genome["regions"]:
            raise HTTPException(status_code=404, detail=f"Brain region {region_id} not found")
        
        # Update the region
        current_region = genome["regions"][region_id]
        
        if region_update.name is not None:
            current_region["name"] = region_update.name
        
        if region_update.description is not None:
            current_region["description"] = region_update.description
        
        if region_update.properties is not None:
            for key, value in region_update.properties.items():
                current_region[key] = value
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        # Find all cortical areas in this region
        cortical_areas = []
        if "blueprint" in genome:
            for area_id, area_data in genome["blueprint"].items():
                if area_data.get("region") == region_id:
                    cortical_areas.append(area_id)
        
        return {
            "id": region_id,
            "name": current_region.get("name", f"Region {region_id}"),
            "description": current_region.get("description"),
            "properties": current_region,
            "cortical_areas": cortical_areas
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating brain region {region_id}: {str(e)}")

@router.delete("/{region_id}")
async def delete_region(
    region_id: str = Path(..., description="ID of the brain region"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Delete a brain region.
    
    Args:
        region_id: ID of the brain region to delete.
    
    Returns:
        Confirmation message.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "regions" not in genome:
            raise HTTPException(status_code=404, detail=f"Brain region {region_id} not found")
        
        if region_id not in genome["regions"]:
            raise HTTPException(status_code=404, detail=f"Brain region {region_id} not found")
        
        # Check if the region is in use by any cortical areas
        if "blueprint" in genome:
            for area_id, area_data in genome["blueprint"].items():
                if area_data.get("region") == region_id:
                    raise HTTPException(
                        status_code=400, 
                        detail=f"Cannot delete brain region {region_id} as it contains cortical area {area_id}"
                    )
        
        # Delete the region
        del genome["regions"][region_id]
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {"message": f"Brain region {region_id} deleted successfully"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting brain region {region_id}: {str(e)}")

@router.post("/{region_id}/cortical_areas", response_model=RegionResponse)
async def add_cortical_area_to_region(
    region_id: str = Path(..., description="ID of the brain region"),
    mapping: CorticalAreaMapping = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Add a cortical area to a brain region.
    
    Args:
        region_id: ID of the brain region.
        mapping: Details of the cortical area to add to the region.
    
    Returns:
        Updated information about the brain region.
    """
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No genome loaded")
        
        # Validate that the region exists
        if "regions" not in genome or region_id not in genome["regions"]:
            raise HTTPException(status_code=404, detail=f"Brain region {region_id} not found")
        
        # Validate that the cortical area exists
        cortical_area_id = mapping.cortical_area_id
        if "blueprint" not in genome or cortical_area_id not in genome["blueprint"]:
            raise HTTPException(status_code=404, detail=f"Cortical area {cortical_area_id} not found")
        
        # Update the cortical area's region
        genome["blueprint"][cortical_area_id]["region"] = region_id
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        # Get updated list of cortical areas in this region
        cortical_areas = []
        for area_id, area_data in genome["blueprint"].items():
            if area_data.get("region") == region_id:
                cortical_areas.append(area_id)
        
        # Get the region information
        region = genome["regions"][region_id]
        
        return {
            "id": region_id,
            "name": region.get("name", f"Region {region_id}"),
            "description": region.get("description"),
            "properties": region,
            "cortical_areas": cortical_areas
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error adding cortical area to region: {str(e)}")

@router.delete("/{region_id}/cortical_areas/{cortical_area_id}")
async def remove_cortical_area_from_region(
    region_id: str = Path(..., description="ID of the brain region"),
    cortical_area_id: str = Path(..., description="ID of the cortical area"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Remove a cortical area from a brain region.
    
    Args:
        region_id: ID of the brain region.
        cortical_area_id: ID of the cortical area to remove from the region.
    
    Returns:
        Confirmation message.
    """
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No genome loaded")
        
        # Validate that the region exists
        if "regions" not in genome or region_id not in genome["regions"]:
            raise HTTPException(status_code=404, detail=f"Brain region {region_id} not found")
        
        # Validate that the cortical area exists
        if "blueprint" not in genome or cortical_area_id not in genome["blueprint"]:
            raise HTTPException(status_code=404, detail=f"Cortical area {cortical_area_id} not found")
        
        # Validate that the cortical area is in this region
        if genome["blueprint"][cortical_area_id].get("region") != region_id:
            raise HTTPException(status_code=400, detail=f"Cortical area {cortical_area_id} is not in region {region_id}")
        
        # Remove the cortical area's region
        if "region" in genome["blueprint"][cortical_area_id]:
            del genome["blueprint"][cortical_area_id]["region"]
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {"message": f"Cortical area {cortical_area_id} removed from region {region_id} successfully"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error removing cortical area from region: {str(e)}") 