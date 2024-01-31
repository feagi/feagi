
from fastapi import APIRouter

from ...schemas import *
from ...commons import *


router = APIRouter()


# ######  Stimulation #########
# #############################

@router.post("/upload/string")
async def stimulation_string_upload(stimulation_script: Stimulation):
    """
    stimulation_script = {
    "IR_pain": {
        "repeat": 10,
        "definition": [
            [{"i__pro": ["0-0-3"], "o__mot": ["2-0-7"]}, 10],
            [{"i__pro": ["0-0-8"]}, 5],
            [{"i__bat": ["0-0-7"]}, 1],
            [{}, 50]
            ]
    },
    "exploration": {
        "definition": []
    },
    "move_forward": {
        "definition": []
    },
    "charge_batteries": {
        "repeat": 1000,
        "definition": [
            [{"i__inf": ["2-0-0"]}, 2]
        ]
    }
    """

    runtime_data.stimulation_script = stimulation_script.stimulation_script

    # message = stimulation_script.dict()
    # message = {'stimulation_script': message}
    # api_queue.put(item=message)


@router.post("/reset")
async def stimulation_string_upload():
    message = {"stimulation_script": {}}
    message = {'stimulation_script': message}
    api_queue.put(item=message)
