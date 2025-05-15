#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================


from fastapi import APIRouter, HTTPException, Depends
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from feagi.utils.logger import setup_logger
logger = setup_logger()

from ...commons import *
from ...schemas import *
from feagi.api.rest.dependencies import get_core_api
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.bdu.connectivity import synaptogenesis_rules


router = APIRouter()


# MorphologyName model for endpoints
class MorphologyName(BaseModel):
    morphology_name: str


# MorphologyInput model for endpoints
class MorphologyInput(BaseModel):
    morphology_name: str
    morphology_type: str
    morphology_parameters: dict


@router.get("/morphology_list")
async def genome_neuron_morphologies(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Returns a comprehensive list of all neuron morphologies.
    
    Note: "Morphology" is the legacy term for connectivity rules in v1 API.
    """
    return core_api.get_morphology_list()


@router.get("/morphology_types")
async def genome_neuron_morphology_types():
    """
    Returns the properties of a neuron morphology.
    
    Note: "Morphology" is the legacy term for connectivity rules in v1 API.
    """
    return {"vectors", "patterns", "composite", "functions"}


@router.get("/list/types")
async def genome_neuron_morphology_type_list(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Returns the properties of a neuron morphology.
    
    Note: "Morphology" is the legacy term for connectivity rules in v1 API.
    """
    report = {}
    for morphology in core_api.get_morphology_list():
        props = core_api.get_morphology_properties(morphology)
        if props:
            report[morphology] = props.get("type")
    return report


@router.get("/morphology_functions")
async def genome_neuron_morphology_functions(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Returns the list of morphology function names.
    
    Note: "Morphology" is the legacy term for connectivity rules in v1 API.
    """
    return core_api.get_morphology_functions()


@router.post("/morphology_properties")
async def genome_neuron_morphology_properties(
    morphology_name: MorphologyName,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Returns the properties of a neuron morphology.
    
    Note: "Morphology" is the legacy term for connectivity rules in v1 API.
    """
    morphology_name = morphology_name.morphology_name
    results = core_api.get_morphology_properties(morphology_name)
    
    if results:
        return results
    else:
        raise HTTPException(status_code=400, detail=f"Morphology named {morphology_name} not found!")


@router.post("/morphology_usage")
async def genome_neuron_morphology_usage_report(
    morphology_name: MorphologyName,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Returns the properties of a neuron morphology.
    
    Note: "Morphology" is the legacy term for connectivity rules in v1 API.
    """
    morphology_name = morphology_name.morphology_name
    results = core_api.get_morphology_properties(morphology_name)
    
    if results:
        usage_list = core_api.get_morphology_usage(morphology_name)
        return usage_list
    else:
        return JSONResponse(status_code=400, content="Morphology not found")


@router.put("/morphology")
async def genome_update_neuron_morphology(
    morphology_input: MorphologyInput,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Updates an existing morphology of non-core class.
    
    Note: "Morphology" is the legacy term for connectivity rules in v1 API.
    """
    # First check if morphology exists
    results = core_api.get_morphology_properties(morphology_input.morphology_name)
    if not results:
        raise HTTPException(status_code=400, detail=f"Morphology {morphology_input.morphology_name} not found!")
    
    # Check if it's a core morphology
    if results.get("class") == "core":
        raise HTTPException(
            status_code=400, 
            detail=f"{morphology_input.morphology_name} is a core morphology and cannot be modified!"
        )
    
    # Update the morphology
    success = core_api.update_morphology(
        name=morphology_input.morphology_name,
        morphology_type=morphology_input.morphology_type,
        parameters=morphology_input.morphology_parameters
    )
    
    if not success:
        raise HTTPException(status_code=500, detail="Failed to update morphology")
    
    # Return success - maintain compatibility with old API behavior
    return {"status": "success"}


@router.post("/morphology")
async def genome_add_neuron_morphology(
    morphology_input: MorphologyInput,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Create new connectivity rule aka. neuron morphology.
    
    Note: "Morphology" is the legacy term for connectivity rules in v1 API.
    """
    # Add the morphology
    success = core_api.add_morphology(
        name=morphology_input.morphology_name,
        morphology_type=morphology_input.morphology_type,
        parameters=morphology_input.morphology_parameters
    )
    
    if not success:
        raise HTTPException(status_code=400, detail="Failed to add morphology or morphology already exists")
    
    # Return success - maintain compatibility with old API behavior
    return {"status": "success"}


@router.delete("/morphology")
async def genome_delete_neuron_morphology(
    morphology_name: MorphologyName,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Delete a neuron morphology.
    
    Note: "Morphology" is the legacy term for connectivity rules in v1 API.
    """
    morphology_name = morphology_name.morphology_name
    
    # Delete the morphology
    success = core_api.delete_morphology(name=morphology_name)
    
    if not success:
        raise HTTPException(
            status_code=400, 
            detail="Failed to delete morphology. It might be in use, a core morphology, or not exist."
        )
    
    # Return success - maintain compatibility with old API behavior
    return {"status": "success"}


@router.get("/morphologies")
async def comprehensive_morphology_list(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Returns all morphologies and all payloads.
    
    Note: "Morphology" is the legacy term for connectivity rules in v1 API.
    """
    if not core_api.genome_is_loaded():
        return {}
        
    return core_api.get_current_genome().get('neuron_morphologies', {})
