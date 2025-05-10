response_templates = {
    "CORTICAL_AREA_NOT_FOUND": {
        "type": "error",
        "message": "Requested cortical area not found.",
    },
    "CORTICAL_AREA_ID_LENGTH_INVALID": {
        "type": "error",
        "message": "Cortical id length can only be 6 characters"
    },
    "CORTICAL_AREA_CREATION_FAILED": {
        "type": "error",
        "message": "Could not create cortical area."
    },
    "CORTICAL_AREA_DELETION_FAILED": {
        "type": "error",
        "message": "Cortical area deletion failed."
    },
    "CORTICAL_AREA_DELETION_PROHIBITED": {
        "type": "error",
        "message": "Cortical area could not be deleted probably due to its type."
    },
    "CORTICAL_AREA_CREATED": {
        "type": "info",
        "message": "Cortical area has been created."
    },
    "CORTICAL_AREA_PROPERTIES_UPDATED": {
        "type": "info",
        "message": "Cortical area properties have been successfully updated."
    }
}

def generate_response(key: str):
    from fastapi.responses import JSONResponse
    from fastapi import HTTPException
    response_data = response_templates.get(key, None)
    if not response_data:
        raise HTTPException(status_code=404, detail="Response key not found.")
    response_data = dict(response_data)  # Copy to avoid mutating the template
    response_data["code"] = key
    status_code = 200 if response_data["type"] == "info" else 400
    return JSONResponse(content=response_data, status_code=status_code) 