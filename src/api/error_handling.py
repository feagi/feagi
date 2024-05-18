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


import logging

from src.api.feagi_responses import response_list
from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from typing import Optional



# Custom Error Response Schema
class ErrorResponse(BaseModel):
    error_code: str
    message: str
    details: Optional[str]


# Custom Exception Classes
class ItemNotFoundException(HTTPException):
    def __init__(self, item_id: str):
        self.item_id = item_id
        super().__init__(status_code=404, detail=f"Item with ID {item_id} not found")


class DatabaseConnectionError(HTTPException):
    def __init__(self):
        super().__init__(status_code=500, detail="Database connection error")

# ... more custom exceptions as needed ...


# Exception Handlers
async def item_not_found_exception_handler(request: Request, exc: ItemNotFoundException):
    return JSONResponse(
        status_code=status.HTTP_404_NOT_FOUND,
        content=ErrorResponse(error_code="item_not_found", message=str(exc), details=f"Item ID: {exc.item_id}").dict(),
    )


async def database_connection_error_handler(request: Request, exc: DatabaseConnectionError):
    logging.error(f"Database Connection Error: {exc}")
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content=ErrorResponse(error_code="database_error", message="Database connection failed").dict(),
    )


# Utility function to register all handlers to an app
def register_exception_handlers(app):
    app.add_exception_handler(ItemNotFoundException, item_not_found_exception_handler)
    app.add_exception_handler(DatabaseConnectionError, database_connection_error_handler)
    # ... register more handlers ...


def generate_response(key: str):
    response_data = response_list.get(key, None)
    response_data["code"] = key
    if not response_data:
        raise HTTPException(status_code=404, detail="Response key not found.")
    return JSONResponse(content=response_data, status_code=200 if response_data["type"] == "info" else 400)
