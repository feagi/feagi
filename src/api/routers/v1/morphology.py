# Copyright 2016-2024 The FEAGI Authors. All Rights Reserved.
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

from ...commons import *
from ...schemas import *

from src.inf import runtime_data
from src.evo import synaptogenesis_rules
from src.evo.synapse import morphology_usage_list


router = APIRouter()


@router.get("/morphology_list")
async def genome_neuron_morphologies():
    """
    Returns a comprehensive list of all neuron morphologies.
    """
    morphology_names = set()

    for morphology in runtime_data.genome['neuron_morphologies']:
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
    for morphology in runtime_data.genome["neuron_morphologies"]:
        if morphology not in report:
            report[morphology] = runtime_data.genome["neuron_morphologies"][morphology]["type"]
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
    if morphology_name in runtime_data.genome['neuron_morphologies']:
        results = runtime_data.genome['neuron_morphologies'][morphology_name]
        results["morphology_name"] = morphology_name
        return results
    else:
        raise HTTPException(status_code=400, detail=f"Morphology named {morphology_name} not found!")


@router.post("/morphology_usage")
async def genome_neuron_morphology_usage_report(morphology_name: MorphologyName):
    """
    Returns the properties of a neuron morphology.
    """
    morphology_name = morphology_name.name
    if morphology_name in runtime_data.genome["neuron_morphologies"]:
        usage_list = morphology_usage_list(morphology_name=morphology_name, genome=runtime_data.genome)
        if usage_list:
            return usage_list
        else:
            return JSONResponse(status_code=200, content=[])
    else:
        return JSONResponse(status_code=400, content="Morphology not found")


@router.put("/morphology")
async def genome_update_neuron_morphology(morphology_input: MorphologyInput):
    """
    Enables changes against various Burst Engine parameters.
    """

    message = dict()
    message["name"] = morphology_input.morphology_name
    message["type"] = morphology_input.morphology_type
    message["parameters"] = morphology_input.morphology_parameters

    message = {'update_morphology_properties': message}
    print("*" * 50 + "\n", message)
    api_queue.put(item=message)


@router.post("/morphology")
async def genome_add_neuron_morphology(morphology_input: MorphologyInput):
    """
    Enables changes against various Burst Engine parameters.
    """
    morphology_name = morphology_input.morphology_name
    if morphology_input.morphology_name not in runtime_data.genome['neuron_morphologies']:
        runtime_data.genome['neuron_morphologies'][morphology_name] = {}
        runtime_data.genome['neuron_morphologies'][morphology_name]["type"] = morphology_input.morphology_type
        runtime_data.genome['neuron_morphologies'][morphology_name]["class"] = "custom"
        runtime_data.genome['neuron_morphologies'][morphology_name]["parameters"] = \
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
    if morphology_name in runtime_data.genome['neuron_morphologies']:
        if "class" in runtime_data.genome['neuron_morphologies'][morphology_name]:
            if runtime_data.genome['neuron_morphologies'][morphology_name]["class"] == "custom":
                usage = morphology_usage_list(morphology_name=morphology_name, genome=runtime_data.genome)
                if not usage:
                    runtime_data.genome['neuron_morphologies'].pop(morphology_name)
                else:
                    raise HTTPException(status_code=400, detail="In use morphology cannot be deleted!")
            elif runtime_data.genome['neuron_morphologies'][morphology_name]["class"] == "core":
                raise HTTPException(status_code=400, detail="Core morphology cannot be deleted!")
        else:
            pass
    else:
        raise HTTPException(status_code=400, detail=f"Morphology with name {morphology_name} not found!")
