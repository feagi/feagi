from pydantic import BaseModel


class InternalServerErrorResponse(BaseModel):
    error_code: int
    error_message: str
