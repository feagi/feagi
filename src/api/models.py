from pydantic import BaseModel


class UnauthorizedResponse(BaseModel):
    error_code: int
    error_message: str


class InternalServerErrorResponse(BaseModel):
    error_code: int
    error_message: str
