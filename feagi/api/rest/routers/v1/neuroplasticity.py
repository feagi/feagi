"""Neuroplasticity API router for FEAGI REST API."""

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
class PlasticityRuleBase(BaseModel):
    """Base model for plasticity rule properties."""
    name: str
    rule_type: str
    parameters: Dict[str, Any] = Field(default={}, description="Parameters of the plasticity rule")

class PlasticityRuleCreate(PlasticityRuleBase):
    """Request model for creating a plasticity rule."""
    pass

class PlasticityRuleUpdate(BaseModel):
    """Request model for updating a plasticity rule."""
    name: Optional[str] = None
    rule_type: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None

class PlasticityRuleResponse(PlasticityRuleBase):
    """Response model for plasticity rule information."""
    id: str

class PlasticityRuleList(BaseModel):
    """Response model for list of plasticity rules."""
    rules: List[PlasticityRuleResponse]

# Create router
router = APIRouter(prefix="/neuroplasticity", tags=["neuroplasticity"])

# Neuroplasticity Endpoints
@router.get("/rules", response_model=PlasticityRuleList)
async def get_all_plasticity_rules(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get all plasticity rules.
    
    Returns a list of all plasticity rules in the current genome.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "plasticity" not in genome:
            return {"rules": []}
        
        rules = []
        for rule_id, rule_data in genome.get("plasticity", {}).items():
            rules.append({
                "id": rule_id,
                "name": rule_data.get("name", f"Rule {rule_id}"),
                "rule_type": rule_data.get("type", "unknown"),
                "parameters": rule_data
            })
        
        return {"rules": rules}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving plasticity rules: {str(e)}")

@router.get("/rules/{rule_id}", response_model=PlasticityRuleResponse)
async def get_plasticity_rule(
    rule_id: str = Path(..., description="ID of the plasticity rule"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get a specific plasticity rule by ID.
    
    Args:
        rule_id: ID of the plasticity rule to retrieve.
    
    Returns:
        Detailed information about the specified plasticity rule.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "plasticity" not in genome:
            raise HTTPException(status_code=404, detail=f"Plasticity rule {rule_id} not found")
        
        rule = genome.get("plasticity", {}).get(rule_id)
        if not rule:
            raise HTTPException(status_code=404, detail=f"Plasticity rule {rule_id} not found")
        
        return {
            "id": rule_id,
            "name": rule.get("name", f"Rule {rule_id}"),
            "rule_type": rule.get("type", "unknown"),
            "parameters": rule
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving plasticity rule {rule_id}: {str(e)}")

@router.post("/rules", response_model=PlasticityRuleResponse)
async def create_plasticity_rule(
    rule: PlasticityRuleCreate,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Create a new plasticity rule.
    
    Args:
        rule: Details of the plasticity rule to create.
    
    Returns:
        Information about the newly created plasticity rule.
    """
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No genome loaded")
        
        if "plasticity" not in genome:
            genome["plasticity"] = {}
        
        # Generate a new ID for the rule
        new_id = str(len(genome["plasticity"]) + 1)
        while new_id in genome["plasticity"]:
            new_id = str(int(new_id) + 1)
        
        # Create the new rule
        new_rule = {
            "name": rule.name,
            "type": rule.rule_type,
            **rule.parameters
        }
        
        genome["plasticity"][new_id] = new_rule
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {
            "id": new_id,
            "name": rule.name,
            "rule_type": rule.rule_type,
            "parameters": new_rule
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating plasticity rule: {str(e)}")

@router.put("/rules/{rule_id}", response_model=PlasticityRuleResponse)
async def update_plasticity_rule(
    rule_id: str = Path(..., description="ID of the plasticity rule"),
    rule_update: PlasticityRuleUpdate = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Update an existing plasticity rule.
    
    Args:
        rule_id: ID of the plasticity rule to update.
        rule_update: Updated details for the plasticity rule.
    
    Returns:
        Information about the updated plasticity rule.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "plasticity" not in genome:
            raise HTTPException(status_code=404, detail=f"Plasticity rule {rule_id} not found")
        
        if rule_id not in genome["plasticity"]:
            raise HTTPException(status_code=404, detail=f"Plasticity rule {rule_id} not found")
        
        # Update the rule
        current_rule = genome["plasticity"][rule_id]
        
        if rule_update.name is not None:
            current_rule["name"] = rule_update.name
        
        if rule_update.rule_type is not None:
            current_rule["type"] = rule_update.rule_type
        
        if rule_update.parameters is not None:
            for key, value in rule_update.parameters.items():
                current_rule[key] = value
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {
            "id": rule_id,
            "name": current_rule.get("name", f"Rule {rule_id}"),
            "rule_type": current_rule.get("type", "unknown"),
            "parameters": current_rule
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating plasticity rule {rule_id}: {str(e)}")

@router.delete("/rules/{rule_id}")
async def delete_plasticity_rule(
    rule_id: str = Path(..., description="ID of the plasticity rule"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Delete a plasticity rule.
    
    Args:
        rule_id: ID of the plasticity rule to delete.
    
    Returns:
        Confirmation message.
    """
    try:
        genome = core_api.get_genome()
        if not genome or "plasticity" not in genome:
            raise HTTPException(status_code=404, detail=f"Plasticity rule {rule_id} not found")
        
        if rule_id not in genome["plasticity"]:
            raise HTTPException(status_code=404, detail=f"Plasticity rule {rule_id} not found")
        
        # Check if the rule is in use by any cortical areas
        if "blueprint" in genome:
            for area_id, area_data in genome["blueprint"].items():
                if area_data.get("plasticity") == rule_id:
                    raise HTTPException(
                        status_code=400, 
                        detail=f"Cannot delete plasticity rule {rule_id} as it is in use by cortical area {area_id}"
                    )
        
        # Delete the rule
        del genome["plasticity"][rule_id]
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {"message": f"Plasticity rule {rule_id} deleted successfully"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting plasticity rule {rule_id}: {str(e)}")

@router.get("/parameters")
async def get_plasticity_parameters(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get global plasticity parameters from the genome.
    
    Returns:
        Global plasticity parameters from the genome.
    """
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No genome loaded")
        
        # Get plasticity parameters from the physiology section
        physiology = genome.get("physiology", {})
        plasticity_params = physiology.get("plasticity", {})
        
        return {
            "global_parameters": plasticity_params
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving plasticity parameters: {str(e)}")

@router.put("/parameters")
async def update_plasticity_parameters(
    parameters: Dict[str, Any] = Body(..., description="Global plasticity parameters"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Update global plasticity parameters in the genome.
    
    Args:
        parameters: Updated global plasticity parameters.
    
    Returns:
        Confirmation message.
    """
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No genome loaded")
        
        # Ensure physiology section exists
        if "physiology" not in genome:
            genome["physiology"] = {}
        
        # Update plasticity parameters
        if "plasticity" not in genome["physiology"]:
            genome["physiology"]["plasticity"] = {}
        
        for key, value in parameters.items():
            genome["physiology"]["plasticity"][key] = value
        
        # Save the updated genome
        if core_api.get_genome_filename():
            save_genome(genome, core_api.get_genome_filename())
        
        return {"message": "Plasticity parameters updated successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating plasticity parameters: {str(e)}") 