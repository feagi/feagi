from pydantic import BaseModel


class InternalServerErrorResponse(BaseModel):
    error_code: int
    error_message: str


class GeneralErrorResponse(BaseModel):
    error_code: int
    error_message: str
    user_readable_message: str
