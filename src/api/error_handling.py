from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from typing import Optional
import logging


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
