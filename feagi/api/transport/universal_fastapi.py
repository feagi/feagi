"""
Universal FastAPI Transport Wrapper

This module automatically generates FastAPI routes from decorated v1 API endpoints.
It scans the endpoint registry and creates FastAPI routers without any duplication.

The v1 API modules remain the single source of truth for all endpoint definitions.
"""

from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import JSONResponse
from typing import Dict, Any, Callable
import inspect
import asyncio

from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.v1.decorators import get_endpoint_registry
from feagi.api.v1.system import create_system_api
from feagi.api.v1.genome import create_genome_api
from feagi.api.v1.cortical_area import create_cortical_area_api
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class UniversalFastAPIWrapper:
    """
    Universal FastAPI wrapper that auto-generates routes from v1 API decorators.
    
    This wrapper scans the endpoint registry and automatically creates
    FastAPI routes for all decorated endpoints, ensuring perfect consistency
    with other transport protocols.
    """
    
    def __init__(self):
        self.router = APIRouter()
        self._api_instances = {}
        
    def create_router_for_module(self, module_name: str) -> APIRouter:
        """Create a FastAPI router for a specific v1 API module."""
        registry = get_endpoint_registry()
        endpoints = registry.get_endpoints_by_module(module_name)
        
        logger.info(f"Creating FastAPI router for module '{module_name}' with {len(endpoints)} endpoints")
        
        for endpoint_id, endpoint_data in endpoints.items():
            self._add_endpoint_to_router(endpoint_data, module_name)
        
        return self.router
    
    def _add_endpoint_to_router(self, endpoint_data: Dict[str, Any], module_name: str):
        """Add a single endpoint to the FastAPI router."""
        methods = endpoint_data['methods']
        path = endpoint_data['path']
        handler = endpoint_data['handler']
        request_model = endpoint_data.get('request_model')
        response_model = endpoint_data.get('response_model')
        description = endpoint_data.get('description')
        
        # Create FastAPI endpoint wrapper
        fastapi_handler = self._create_fastapi_handler(handler, module_name, request_model)
        
        # Register for each HTTP method
        for method in methods:
            method_lower = method.lower()
            
            # Build route kwargs
            route_kwargs = {
                'path': path,
                'response_model': response_model,
                'description': description,
                'name': f"{module_name}_{handler.__name__}"
            }
            
            # Remove None values
            route_kwargs = {k: v for k, v in route_kwargs.items() if v is not None}
            
            # Register the route
            if method_lower == 'get':
                self.router.get(**route_kwargs)(fastapi_handler)
            elif method_lower == 'post':
                self.router.post(**route_kwargs)(fastapi_handler)
            elif method_lower == 'put':
                self.router.put(**route_kwargs)(fastapi_handler)
            elif method_lower == 'delete':
                self.router.delete(**route_kwargs)(fastapi_handler)
            elif method_lower == 'patch':
                self.router.patch(**route_kwargs)(fastapi_handler)
            else:
                logger.warning(f"Unsupported HTTP method: {method}")
            
            logger.debug(f"Registered FastAPI route: {method} {path} -> {handler.__name__}")
    
    def _create_fastapi_handler(self, 
                               original_handler: Callable, 
                               module_name: str,
                               request_model) -> Callable:
        """Create a FastAPI-compatible handler wrapper."""
        
        def _get_api_instance(core_api_service: CoreAPIService = Depends(get_core_api_service)):
            """Dependency to get the appropriate API instance."""
            if module_name not in self._api_instances:
                if module_name == 'system':
                    self._api_instances[module_name] = create_system_api(core_api_service)
                elif module_name == 'genome':
                    self._api_instances[module_name] = create_genome_api(core_api_service)
                elif module_name == 'cortical_area':
                    self._api_instances[module_name] = create_cortical_area_api(core_api_service)
                # Add other modules as needed:
                # elif module_name == 'connectome':
                #     self._api_instances[module_name] = create_connectome_api(core_api_service)
                else:
                    raise ValueError(f"Unknown module: {module_name}")
            
            return self._api_instances[module_name]
        
        # Inspect the original handler to understand its parameters
        sig = inspect.signature(original_handler)
        params = list(sig.parameters.keys())
        
        # Determine if handler needs request data
        needs_request_data = len(params) > 1  # First param is always 'self'
        is_async = asyncio.iscoroutinefunction(original_handler)
        
        if needs_request_data and request_model:
            # Handler expects request data
            if is_async:
                async def fastapi_handler_with_request(
                    request_data: request_model,
                    api_instance = Depends(_get_api_instance)
                ):
                    try:
                        return await original_handler(api_instance, request_data)
                    except ValueError as e:
                        raise HTTPException(status_code=400, detail=str(e))
                    except Exception as e:
                        logger.error(f"Error in {original_handler.__name__}: {e}")
                        raise HTTPException(status_code=500, detail="Internal server error")
                        
                return fastapi_handler_with_request
            else:
                def fastapi_handler_with_request(
                    request_data: request_model,
                    api_instance = Depends(_get_api_instance)
                ):
                    try:
                        return original_handler(api_instance, request_data)
                    except ValueError as e:
                        raise HTTPException(status_code=400, detail=str(e))
                    except Exception as e:
                        logger.error(f"Error in {original_handler.__name__}: {e}")
                        raise HTTPException(status_code=500, detail="Internal server error")
                        
                return fastapi_handler_with_request
        
        elif needs_request_data and not request_model:
            # Handler expects parameters but no Pydantic model (e.g., path params)
            if is_async:
                async def fastapi_handler_with_params(
                    request: Request,
                    api_instance = Depends(_get_api_instance)
                ):
                    try:
                        # Extract parameters from request
                        # This is a simplified version - you might need more sophisticated parameter extraction
                        body = await request.json() if request.headers.get('content-type') == 'application/json' else {}
                        query_params = dict(request.query_params)
                        
                        # Call handler with extracted data
                        if len(params) == 2:  # self + one param
                            param_name = params[1]
                            value = body.get(param_name) or query_params.get(param_name)
                            return await original_handler(api_instance, value)
                        else:
                            return await original_handler(api_instance, body)
                    except ValueError as e:
                        raise HTTPException(status_code=400, detail=str(e))
                    except Exception as e:
                        logger.error(f"Error in {original_handler.__name__}: {e}")
                        raise HTTPException(status_code=500, detail="Internal server error")
                        
                return fastapi_handler_with_params
            else:
                def fastapi_handler_with_params(
                    request: Request,
                    api_instance = Depends(_get_api_instance)
                ):
                    try:
                        # For sync handlers, we can't await request.json()
                        # This is a limitation - might need to restructure
                        return original_handler(api_instance, {})
                    except ValueError as e:
                        raise HTTPException(status_code=400, detail=str(e))
                    except Exception as e:
                        logger.error(f"Error in {original_handler.__name__}: {e}")
                        raise HTTPException(status_code=500, detail="Internal server error")
                        
                return fastapi_handler_with_params
        
        else:
            # Handler doesn't need request data (e.g., GET endpoints)
            if is_async:
                async def fastapi_handler_simple(api_instance = Depends(_get_api_instance)):
                    try:
                        return await original_handler(api_instance)
                    except ValueError as e:
                        raise HTTPException(status_code=400, detail=str(e))
                    except Exception as e:
                        logger.error(f"Error in {original_handler.__name__}: {e}")
                        raise HTTPException(status_code=500, detail="Internal server error")
                        
                return fastapi_handler_simple
            else:
                def fastapi_handler_simple(api_instance = Depends(_get_api_instance)):
                    try:
                        return original_handler(api_instance)
                    except ValueError as e:
                        raise HTTPException(status_code=400, detail=str(e))
                    except Exception as e:
                        logger.error(f"Error in {original_handler.__name__}: {e}")
                        raise HTTPException(status_code=500, detail="Internal server error")
                        
                return fastapi_handler_simple


def create_system_router() -> APIRouter:
    """Create a FastAPI router for system endpoints."""
    wrapper = UniversalFastAPIWrapper()
    return wrapper.create_router_for_module('system')


def create_genome_router() -> APIRouter:
    """Create a FastAPI router for genome endpoints."""
    wrapper = UniversalFastAPIWrapper()
    return wrapper.create_router_for_module('genome')


def create_cortical_area_router() -> APIRouter:
    """Create a FastAPI router for cortical area endpoints."""
    wrapper = UniversalFastAPIWrapper()
    return wrapper.create_router_for_module('cortical_area')


# Create the router instance that can be imported
def get_system_router() -> APIRouter:
    """Get the system router for use in main FastAPI app."""
    return create_system_router()


def get_genome_router() -> APIRouter:
    """Get the genome router for use in main FastAPI app."""
    return create_genome_router()


def get_cortical_area_router() -> APIRouter:
    """Get the cortical area router for use in main FastAPI app."""
    return create_cortical_area_router() 