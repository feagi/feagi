"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""Error types for FEAGI.

This module defines custom error types that can be directly mapped to Rust's enum-based errors.
Using these error types makes it easier to port code to Rust in the future.
"""
from enum import Enum, auto
from typing import Any, Dict, List, Optional, Type, Union


class ErrorCode(Enum):
    """Error codes for FEAGI errors.

    These codes can be directly mapped to Rust error enums.
    """

    # General errors
    UNKNOWN = auto()
    NOT_IMPLEMENTED = auto()
    INVALID_ARGUMENT = auto()
    INVALID_STATE = auto()
    TIMEOUT = auto()

    # Resource errors
    RESOURCE_NOT_FOUND = auto()
    RESOURCE_ALREADY_EXISTS = auto()
    RESOURCE_BUSY = auto()
    RESOURCE_EXHAUSTED = auto()

    # Neural processing errors
    NEURAL_INITIALIZATION_FAILED = auto()
    NEURAL_PROCESSING_FAILED = auto()
    SYNAPSE_CREATION_FAILED = auto()

    # Communication errors
    COMMUNICATION_FAILED = auto()
    SERIALIZATION_FAILED = auto()
    DESERIALIZATION_FAILED = auto()

    # API errors
    API_REQUEST_FAILED = auto()
    API_RESPONSE_INVALID = auto()

    # Authentication errors
    AUTHENTICATION_FAILED = auto()
    AUTHORIZATION_FAILED = auto()

    # File I/O errors
    FILE_NOT_FOUND = auto()
    FILE_ACCESS_DENIED = auto()
    FILE_CORRUPTED = auto()

    # Rust integration errors
    RUST_FUNCTION_CALL_FAILED = auto()
    RUST_MODULE_NOT_AVAILABLE = auto()


class FeagiError(Exception):
    """Base class for all FEAGI errors."""

    def __init__(
        self,
        message: str,
        code: ErrorCode = ErrorCode.UNKNOWN,
        details: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize a FEAGI error.

        Args:
            message: Error message
            code: Error code
            details: Additional details about the error
        """
        self.message = message
        self.code = code
        self.details = details or {}
        super().__init__(message)

    def __str__(self) -> str:
        """Get string representation of the error."""
        if self.details:
            details_str = ", ".join(f"{k}={v}" for k, v in self.details.items())
            return f"{self.code.name}: {self.message} ({details_str})"
        return f"{self.code.name}: {self.message}"

    def to_dict(self) -> Dict[str, Any]:
        """Convert the error to a dictionary."""
        return {
            "code": self.code.name,
            "message": self.message,
            "details": self.details,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "FeagiError":
        """Create an error from a dictionary."""
        code_name = data.get("code", ErrorCode.UNKNOWN.name)
        try:
            code = ErrorCode[code_name]
        except KeyError:
            code = ErrorCode.UNKNOWN

        return cls(
            message=data.get("message", "Unknown error"),
            code=code,
            details=data.get("details", {}),
        )

    def to_rust_error(self) -> str:
        """Generate Rust error enum variant for this error."""
        variant_name = self.code.name

        if self.details:
            # If details exist, create a struct variant
            fields = []
            for key, value in self.details.items():
                # Simplistic type mapping for example
                rust_type = "String"
                if isinstance(value, int):
                    rust_type = "i32"
                elif isinstance(value, float):
                    rust_type = "f64"
                elif isinstance(value, bool):
                    rust_type = "bool"

                fields.append(f"{key}: {rust_type}")

            return f"{variant_name} {{ message: String, {', '.join(fields)} }}"
        else:
            # Simple variant with just a message
            return f"{variant_name}(String)"


# Specific error types


class ResourceNotFoundError(FeagiError):
    """Error raised when a resource is not found."""

    def __init__(
        self,
        resource_type: str,
        resource_id: Union[str, int],
        message: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize a ResourceNotFoundError.

        Args:
            resource_type: Type of resource that was not found
            resource_id: ID of the resource
            message: Optional custom error message
            details: Additional details about the error
        """
        if message is None:
            message = f"{resource_type} with ID {resource_id} not found"

        details = details or {}
        details.update(
            {
                "resource_type": resource_type,
                "resource_id": str(resource_id),
            }
        )

        super().__init__(
            message=message,
            code=ErrorCode.RESOURCE_NOT_FOUND,
            details=details,
        )


class InvalidArgumentError(FeagiError):
    """Error raised when an invalid argument is provided."""

    def __init__(
        self,
        parameter: str,
        value: Any,
        message: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize an InvalidArgumentError.

        Args:
            parameter: Name of the parameter that was invalid
            value: Value that was provided
            message: Optional custom error message
            details: Additional details about the error
        """
        if message is None:
            message = f"Invalid value for parameter '{parameter}': {value}"

        details = details or {}
        details.update(
            {
                "parameter": parameter,
                "value": str(value),
            }
        )

        super().__init__(
            message=message,
            code=ErrorCode.INVALID_ARGUMENT,
            details=details,
        )


class NeuralProcessingError(FeagiError):
    """Error raised when neural processing fails."""

    def __init__(
        self,
        operation: str,
        message: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize a NeuralProcessingError.

        Args:
            operation: Neural processing operation that failed
            message: Optional custom error message
            details: Additional details about the error
        """
        if message is None:
            message = f"Neural processing operation '{operation}' failed"

        details = details or {}
        details.update(
            {
                "operation": operation,
            }
        )

        super().__init__(
            message=message,
            code=ErrorCode.NEURAL_PROCESSING_FAILED,
            details=details,
        )


class RustIntegrationError(FeagiError):
    """Error raised when a Rust integration fails."""

    def __init__(
        self,
        function: str,
        message: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize a RustIntegrationError.

        Args:
            function: Rust function that failed
            message: Optional custom error message
            details: Additional details about the error
        """
        if message is None:
            message = f"Rust function '{function}' call failed"

        details = details or {}
        details.update(
            {
                "function": function,
            }
        )

        super().__init__(
            message=message,
            code=ErrorCode.RUST_FUNCTION_CALL_FAILED,
            details=details,
        )


class RustCompatibilityError(FeagiError):
    """Error raised when there is a compatibility issue with Rust code."""

    def __init__(
        self,
        component: str,
        message: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize a RustCompatibilityError.

        Args:
            component: Component that has compatibility issues
            message: Optional custom error message
            details: Additional details about the error
        """
        if message is None:
            message = f"Rust compatibility issue in component '{component}'"

        details = details or {}
        details.update(
            {
                "component": component,
            }
        )

        super().__init__(
            message=message,
            code=ErrorCode.RUST_MODULE_NOT_AVAILABLE,
            details=details,
        )


# Example Rust error enum generation
def generate_rust_error_enum() -> str:
    """
    Generate a Rust error enum that corresponds to all FEAGI errors.

    Returns:
        Rust code for the error enum
    """
    lines = [
        "/// Auto-generated Rust error enum for FEAGI errors",
        "#[derive(Debug, thiserror::Error)]",
        "pub enum FeagiError {",
    ]

    # Add a variant for each error code
    for code in ErrorCode:
        variant_name = code.name
        lines.append(f"    /// {code.name.replace('_', ' ').title()}")
        lines.append(
            f'    #[error("{code.name.replace("_", " ").title()}: {{message}}")]'
        )
        lines.append(f"    {variant_name} {{")
        lines.append(f"        message: String,")
        lines.append(f"    }},")

    lines.append("}")

    # Add From implementations for common error types
    lines.append("")
    lines.append("// Implement From traits for standard error types")
    lines.append("impl From<std::io::Error> for FeagiError {")
    lines.append("    fn from(error: std::io::Error) -> Self {")
    lines.append("        Self::FILE_ACCESS_DENIED {")
    lines.append("            message: error.to_string(),")
    lines.append("        }")
    lines.append("    }")
    lines.append("}")

    return "\n".join(lines)
