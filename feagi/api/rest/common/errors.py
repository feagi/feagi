"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""Error handling utilities for FEAGI REST API."""

from typing import Any, Dict, Optional

from pydantic import BaseModel


class ErrorResponse(BaseModel):
    """Standardized error response model."""

    code: int
    message: str
    details: Optional[Dict[str, Any]] = None


class APIError(Exception):
    """API error exception class for custom error handling."""

    def __init__(
        self, code: int, message: str, details: Optional[Dict[str, Any]] = None
    ):
        """Initialize an API error.

        Args:
            code: HTTP status code.
            message: Error message.
            details: Optional details about the error.
        """
        self.code = code
        self.message = message
        self.details = details
        super().__init__(message)

    def to_response(self) -> ErrorResponse:
        """Convert the exception to an ErrorResponse.

        Returns:
            ErrorResponse instance.
        """
        return ErrorResponse(
            code=self.code, message=self.message, details=self.details
        )
