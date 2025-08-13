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

"""
FEAGI v1 API Multi-Transport Decorators

This module provides decorators that allow v1 API methods to be automatically
registered for multiple transport protocols (FastAPI, ZMQ, gRPC, etc.) from
a single endpoint definition.

This ensures v1 API modules remain the single source of truth for all endpoints.
"""

from typing import Any, Callable, Dict, List, Optional, Type, Union

from pydantic import BaseModel

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class EndpointRegistry:
    """Global registry for all v1 API endpoints.

    This registry tracks all decorated endpoints and their metadata, allowing
    transport adapters to automatically register routes.
    """

    def __init__(self):
        self.endpoints: Dict[str, Dict[str, Any]] = {}

    def register(
        self,
        endpoint_id: str,
        methods: List[str],
        path: str,
        handler: Callable,
        request_model: Optional[Type[BaseModel]] = None,
        response_model: Optional[Type[BaseModel]] = None,
        description: Optional[str] = None,
        module: Optional[str] = None,
    ):
        """Register an endpoint in the global registry."""

        self.endpoints[endpoint_id] = {
            "methods": methods,
            "path": path,
            "handler": handler,
            "request_model": request_model,
            "response_model": response_model,
            "description": description,
            "module": module,
            "full_path": f"/v1{path}",  # Add v1 prefix
        }

        logger.debug(
            f"Registered endpoint: {methods} {path} -> {handler.__name__}"
        )

    def get_endpoints_by_module(
        self, module: str
    ) -> Dict[str, Dict[str, Any]]:
        """Get all endpoints for a specific module (e.g., 'system',
        'genome')."""
        return {
            endpoint_id: endpoint_data
            for endpoint_id, endpoint_data in self.endpoints.items()
            if endpoint_data.get("module") == module
        }

    def get_all_endpoints(self) -> Dict[str, Dict[str, Any]]:
        """Get all registered endpoints."""
        return self.endpoints.copy()


# Global endpoint registry
endpoint_registry = EndpointRegistry()


def endpoint(
    methods: Union[str, List[str]],
    path: str,
    request_model: Optional[Type[BaseModel]] = None,
    response_model: Optional[Type[BaseModel]] = None,
    description: Optional[str] = None,
    module: Optional[str] = None,
):
    """Multi-transport endpoint decorator.

    This decorator registers a v1 API method for ALL transport protocols.
    The method will be automatically available via FastAPI, ZMQ, gRPC, etc.

    Args:
        methods: HTTP methods (e.g., 'GET', ['GET', 'POST'])
        path: API path relative to module (e.g., '/health_check')
        request_model: Pydantic model for request validation
        response_model: Pydantic model for response serialization
        description: Endpoint description for documentation
        module: Module name (e.g., 'system', 'genome')

    Example:
        @endpoint('GET', '/health_check', response_model=HealthCheckResponse, module='system')
        async def get_health_check(self) -> HealthCheckResponse:
            return await self.core_api_service.get_system_health()
    """

    def decorator(func: Callable) -> Callable:
        # Normalize methods to list
        method_list = [methods] if isinstance(methods, str) else methods

        # Generate unique endpoint ID
        endpoint_id = f"{module or 'unknown'}:{':'.join(method_list)}:{path}"

        # Register in global registry
        endpoint_registry.register(
            endpoint_id=endpoint_id,
            methods=method_list,
            path=path,
            handler=func,
            request_model=request_model,
            response_model=response_model,
            description=description or func.__doc__,
            module=module,
        )

        # Add metadata to the function for inspection
        func._endpoint_methods = method_list
        func._endpoint_path = path
        func._endpoint_module = module
        func._endpoint_id = endpoint_id
        func._request_model = request_model
        func._response_model = response_model

        return func

    return decorator


def get_endpoint_registry() -> EndpointRegistry:
    """Get the global endpoint registry."""
    return endpoint_registry


def system_endpoint(
    methods: Union[str, List[str]],
    path: str,
    request_model: Optional[Type[BaseModel]] = None,
    response_model: Optional[Type[BaseModel]] = None,
    description: Optional[str] = None,
):
    """Convenience decorator for system module endpoints.

    This is equivalent to @endpoint(..., module='system')
    """
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="system",
    )


def genome_endpoint(
    methods: Union[str, List[str]],
    path: str,
    request_model: Optional[Type[BaseModel]] = None,
    response_model: Optional[Type[BaseModel]] = None,
    description: Optional[str] = None,
):
    """Convenience decorator for genome module endpoints.

    This is equivalent to @endpoint(..., module='genome')
    """
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="genome",
    )


def connectome_endpoint(
    methods: Union[str, List[str]],
    path: str,
    request_model: Optional[Type[BaseModel]] = None,
    response_model: Optional[Type[BaseModel]] = None,
    description: Optional[str] = None,
):
    """Convenience decorator for connectome module endpoints.

    This is equivalent to @endpoint(..., module='connectome')
    """
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="connectome",
    )
