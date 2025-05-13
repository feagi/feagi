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


from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from feagi.utils.logger import setup_logger
logger = setup_logger()

from ...commons import *
from ...schemas import *

from feagi.bdu import synaptogenesis_rules
from feagi.bdu import ConnectomeManager


router = APIRouter()



# MorphologyName model for endpoints
class MorphologyName(BaseModel):
    morphology_name: str


# MorphologyInput model for endpoints
class MorphologyInput(BaseModel):
    morphology_name: str
    morphology_type: str
    morphology_parameters: dict


def morphology_usage_list(morphology_name, genome):
    """
    Returns a list of (cortical_area, destination) tuples where the given morphology is used in the connectome.
    """
    usage_list = set()
    for cortical_area in genome['blueprint']:
        for destination in genome['blueprint'][cortical_area].get('cortical_mapping_dst', {}):
            for mapping in genome['blueprint'][cortical_area]['cortical_mapping_dst'][destination]:
                if mapping.get("morphology_id") == morphology_name:
                    usage_list.add((cortical_area, destination))
    return list(usage_list)


@router.get("/morphology_list")
async def genome_neuron_morphologies():
    """
    Returns a comprehensive list of all neuron morphologies.
    """
    morphology_names = set()

    for morphology in connectome.get_genome()['neuron_morphologies']:
        morphology_names.add(morphology)
    return sorted(morphology_names)


@router.get("/morphology_types")
async def genome_neuron_morphology_types():
    """
    Returns the properties of a neuron morphology.
    """
    return {"vectors", "patterns", "composite", "functions"}


@router.get("/list/types")
async def genome_neuron_morphology_type_list():
    """
    Returns the properties of a neuron morphology.
    """
    report = {}
    for morphology in connectome.get_genome()["neuron_morphologies"]:
        if morphology not in report:
            report[morphology] = connectome.get_genome()["neuron_morphologies"][morphology]["type"]
    return report


@router.get("/morphology_functions")
async def genome_neuron_morphology_functions():
    """
    Returns the list of morphology function names.
    """
    morphology_list = set()
    for entry in dir(synaptogenesis_rules):
        if str(entry)[:4] == "syn_":
            morphology_list.add(str(entry))
    return morphology_list


@router.post("/morphology_properties")
async def genome_neuron_morphology_properties(morphology_name: MorphologyName):
    """
    Returns the properties of a neuron morphology.
    """
    morphology_name = morphology_name.morphology_name
    if morphology_name in connectome.get_genome()['neuron_morphologies']:
        results = connectome.get_genome()['neuron_morphologies'][morphology_name]
        results["morphology_name"] = morphology_name
        return results
    else:
        raise HTTPException(status_code=400, detail=f"Morphology named {morphology_name} not found!")


@router.post("/morphology_usage")
async def genome_neuron_morphology_usage_report(morphology_name: MorphologyName):
    """
    Returns the properties of a neuron morphology.
    """
    morphology_name = morphology_name.morphology_name
    if morphology_name in connectome.get_genome()["neuron_morphologies"]:
        usage_list = morphology_usage_list(morphology_name=morphology_name, genome=connectome.get_genome())
        if usage_list:
            return usage_list
        else:
            return JSONResponse(status_code=200, content=[])
    else:
        return JSONResponse(status_code=400, content="Morphology not found")


@router.put("/morphology")
async def genome_update_neuron_morphology(morphology_input: MorphologyInput):
    """
    Updates an exsiting morphology of non-core class
    """
    if morphology_input.morphology_name not in connectome.get_genome()['neuron_morphologies']:
        raise HTTPException(status_code=400, detail=f"Morphology {morphology_input.morphology_name} not found!")
    elif connectome.get_genome()['neuron_morphologies'][morphology_input.morphology_name]["class"] == "core":
        raise HTTPException(status_code=400, detail=f"{morphology_input.morphology_name} is a core morphology and "
                                                    f"cannot be modified!")
    else:
        message = dict()
        message["name"] = morphology_input.morphology_name
        message["type"] = morphology_input.morphology_type
        message["class"] = connectome.get_genome()['neuron_morphologies'][morphology_input.morphology_name]["class"]
        message["parameters"] = morphology_input.morphology_parameters

        message = {'update_morphology_properties': message}
        logger.info("*" * 50 + "\n" + str(message))
        api_queue.put(item=message)


@router.post("/morphology")
async def genome_add_neuron_morphology(morphology_input: MorphologyInput):
    """
    Create new connectivity rule aka. neuron morphology.
    """
    morphology_name = morphology_input.morphology_name
    if morphology_input.morphology_name not in connectome.get_genome()['neuron_morphologies']:
        connectome.get_genome()['neuron_morphologies'][morphology_name] = {}
        connectome.get_genome()['neuron_morphologies'][morphology_name]["type"] = morphology_input.morphology_type
        connectome.get_genome()['neuron_morphologies'][morphology_name]["class"] = "custom"
        connectome.get_genome()['neuron_morphologies'][morphology_name]["parameters"] = \
            morphology_input.morphology_parameters
    else:
        pass


@router.delete("/morphology")
async def genome_delete_neuron_morphology(morphology_name: MorphologyName):
    """
    Returns the properties of a neuron morphology.
    """
    # todo: Needs to be rewritten
    morphology_name = morphology_name.morphology_name
    if morphology_name in connectome.get_genome()['neuron_morphologies']:
        if "class" in connectome.get_genome()['neuron_morphologies'][morphology_name]:
            if connectome.get_genome()['neuron_morphologies'][morphology_name]["class"] == "custom":
                usage = morphology_usage_list(morphology_name=morphology_name, genome=connectome.get_genome())
                if not usage:
                    connectome.get_genome()['neuron_morphologies'].pop(morphology_name)
                else:
                    raise HTTPException(status_code=400, detail="In use morphology cannot be deleted!")
            elif connectome.get_genome()['neuron_morphologies'][morphology_name]["class"] == "core":
                raise HTTPException(status_code=400, detail="Core morphology cannot be deleted!")
        else:
            pass
    else:
        raise HTTPException(status_code=400, detail=f"Morphology with name {morphology_name} not found!")


@router.get("/morphologies")
async def comprehensive_morphology_list():
    """
    Returns all morphologies and all payloads
    """
    if connectome.get_genome()['neuron_morphologies']:
        return connectome.get_genome()['neuron_morphologies']
