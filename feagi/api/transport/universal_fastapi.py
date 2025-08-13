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
Universal FastAPI Transport Wrapper

This module automatically generates FastAPI routes from decorated v1 API endpoints.
It scans the endpoint registry and creates FastAPI routers without any duplication.

The v1 API modules remain the single source of truth for all endpoint definitions.

NOTE: This module is completely disabled in embedded mode to prevent FastAPI imports.
"""

import os

# Check if embedded mode is enabled
EMBEDDED_MODE = os.environ.get("FEAGI_EMBEDDED_MODE", "0") == "1"

if EMBEDDED_MODE:
    # In embedded mode, provide stub functions that return None
    # This prevents any FastAPI imports and router creation

    def create_system_router():
        """Stub function for embedded mode."""
        return None

    def create_genome_router():
        """Stub function for embedded mode."""
        return None

    def create_cortical_area_router():
        """Stub function for embedded mode."""
        return None

    def create_connectome_router():
        """Stub function for embedded mode."""
        return None

    def create_burst_engine_router():
        """Stub function for embedded mode."""
        return None

    def create_neuroplasticity_router():
        """Stub function for embedded mode."""
        return None

    def create_region_router():
        """Stub function for embedded mode."""
        return None

    def create_morphology_router():
        """Stub function for embedded mode."""
        return None

    def create_monitoring_router():
        """Stub function for embedded mode."""
        return None

    def create_simulation_router():
        """Stub function for embedded mode."""
        return None

    def create_feagi_agent_router():
        """Stub function for embedded mode."""
        return None

    def create_insights_router():
        """Stub function for embedded mode."""
        return None

    def create_training_router():
        """Stub function for embedded mode."""
        return None

    def create_cortical_mapping_router():
        """Stub function for embedded mode."""
        return None

    def create_network_router():
        """Stub function for embedded mode."""
        return None

    def create_inputs_router():
        """Stub function for embedded mode."""
        return None

    def create_outputs_router():
        """Stub function for embedded mode."""
        return None

    def create_evolution_router():
        """Stub function for embedded mode."""
        return None

    # Stub functions for the get_* functions as well
    def get_system_router():
        """Stub function for embedded mode."""
        return None

    def get_genome_router():
        """Stub function for embedded mode."""
        return None

    def get_cortical_area_router():
        """Stub function for embedded mode."""
        return None

    def get_connectome_router():
        """Stub function for embedded mode."""
        return None

    def get_burst_engine_router():
        """Stub function for embedded mode."""
        return None

    def get_neuroplasticity_router():
        """Stub function for embedded mode."""
        return None

    def get_region_router():
        """Stub function for embedded mode."""
        return None

    def get_morphology_router():
        """Stub function for embedded mode."""
        return None

    def get_monitoring_router():
        """Stub function for embedded mode."""
        return None

    def get_simulation_router():
        """Stub function for embedded mode."""
        return None

    def get_feagi_agent_router():
        """Stub function for embedded mode."""
        return None

    def get_insights_router():
        """Stub function for embedded mode."""
        return None

    def get_training_router():
        """Stub function for embedded mode."""
        return None

    def get_cortical_mapping_router():
        """Stub function for embedded mode."""
        return None

    def get_network_router():
        """Stub function for embedded mode."""
        return None

    def get_inputs_router():
        """Stub function for embedded mode."""
        return None

    def get_outputs_router():
        """Stub function for embedded mode."""
        return None

    def get_evolution_router():
        """Stub function for embedded mode."""
        return None

    # Also need to provide the UniversalFastAPIWrapper class as a stub
    class UniversalFastAPIWrapper:
        """Stub class for embedded mode."""

        def __init__(self):
            self.router = None

        def create_router_for_module(self, module_name: str):
            return None

        def _add_endpoint_to_router(self, endpoint_data, module_name):
            pass

        def _create_fastapi_handler(self, handler, module_name, request_model):
            return None

else:
    # Normal mode - import all FastAPI dependencies and create actual routers
    import asyncio
    import inspect
    from typing import Any, Callable, Dict

    from fastapi import APIRouter, Depends, HTTPException, UploadFile

    from feagi.api.core.services.core_api_service import CoreAPIService
    from feagi.api.rest.dependencies import get_core_api_service
    from feagi.api.v1.burst_engine import create_burst_engine_api

    def _maybe_file_response(result):
        """Convert result dict with {path, filename} into a FileResponse and
        cleanup."""
        try:
            if (
                isinstance(result, dict)
                and "path" in result
                and result.get("filename")
            ):
                from fastapi.responses import FileResponse
                from starlette.background import BackgroundTask

                zip_path = result["path"]
                filename = result["filename"]

                def _cleanup():
                    try:
                        import os

                        os.remove(zip_path)
                    except Exception:
                        pass

                return FileResponse(
                    path=zip_path,
                    filename=filename,
                    media_type="application/zip",
                    background=BackgroundTask(_cleanup),
                )
        except Exception:
            return result
        return result

    # Import all the new API modules
    from feagi.api.v1.connectome import create_connectome_api
    from feagi.api.v1.cortical_area import create_cortical_area_api
    from feagi.api.v1.cortical_mapping import create_cortical_mapping_api
    from feagi.api.v1.decorators import get_endpoint_registry
    from feagi.api.v1.evolution import create_evolution_api
    from feagi.api.v1.feagi_agent import create_feagi_agent_api
    from feagi.api.v1.genome import create_genome_api
    from feagi.api.v1.inputs import create_inputs_api
    from feagi.api.v1.insights import create_insights_api
    from feagi.api.v1.monitoring import create_monitoring_api
    from feagi.api.v1.morphology import create_morphology_api
    from feagi.api.v1.network import create_network_api
    from feagi.api.v1.neuroplasticity import create_neuroplasticity_api
    from feagi.api.v1.outputs import create_outputs_api
    from feagi.api.v1.physiology import create_physiology_api
    from feagi.api.v1.region import create_region_api
    from feagi.api.v1.simulation import create_simulation_api
    from feagi.api.v1.snapshot import create_snapshot_api
    from feagi.api.v1.system import create_system_api
    from feagi.api.v1.training import create_training_api
    from feagi.utils.logger import setup_logger

    logger = setup_logger(__name__)

    class UniversalFastAPIWrapper:
        """Universal FastAPI wrapper that auto-generates routes from v1 API
        decorators.

        This wrapper scans the endpoint registry and automatically creates
        FastAPI routes for all decorated endpoints, ensuring perfect
        consistency with other transport protocols.
        """

        def __init__(self):
            self.router = APIRouter()
            self._api_instances = {}

        def create_router_for_module(self, module_name: str) -> APIRouter:
            """Create a FastAPI router for a specific v1 API module."""
            # Special-case snapshot: use manual router to avoid param binding issues
            if module_name == "snapshot":
                from feagi.api.transport.universal_fastapi import (
                    create_snapshot_router,
                )

                return create_snapshot_router()
            registry = get_endpoint_registry()
            endpoints = registry.get_endpoints_by_module(module_name)

            logger.info(
                f"Creating FastAPI router for module '{module_name}' with {len(endpoints)} endpoints"
            )

            for _endpoint_id, endpoint_data in endpoints.items():
                self._add_endpoint_to_router(endpoint_data, module_name)

            return self.router

        def _add_endpoint_to_router(
            self, endpoint_data: Dict[str, Any], module_name: str
        ):
            """Add a single endpoint to the FastAPI router."""
            methods = endpoint_data["methods"]
            path = endpoint_data["path"]
            handler = endpoint_data["handler"]
            request_model = endpoint_data.get("request_model")
            response_model = endpoint_data.get("response_model")
            description = endpoint_data.get("description")

            # Create FastAPI endpoint wrapper
            fastapi_handler = self._create_fastapi_handler(
                handler, module_name, request_model, endpoint_data
            )

            # Register for each HTTP method
            for method in methods:
                method_lower = method.lower()

                # Build route kwargs
                route_kwargs = {
                    "path": path,
                    "response_model": response_model,
                    "description": description,
                    "name": f"{module_name}_{handler.__name__}",
                }

                # Remove None values
                route_kwargs = {
                    k: v for k, v in route_kwargs.items() if v is not None
                }

                # Register the route
                if method_lower == "get":
                    self.router.get(**route_kwargs)(fastapi_handler)
                elif method_lower == "post":
                    self.router.post(**route_kwargs)(fastapi_handler)
                elif method_lower == "put":
                    self.router.put(**route_kwargs)(fastapi_handler)
                elif method_lower == "delete":
                    self.router.delete(**route_kwargs)(fastapi_handler)
                elif method_lower == "patch":
                    self.router.patch(**route_kwargs)(fastapi_handler)
                else:
                    logger.warning(f"Unsupported HTTP method: {method}")

                logger.debug(
                    f"Registered FastAPI route: {method} {path} -> {handler.__name__}"
                )

        def _create_fastapi_handler(
            self,
            original_handler: Callable,
            module_name: str,
            request_model,
            endpoint_data: Dict[str, Any],
        ) -> Callable:
            """Create a FastAPI-compatible handler wrapper."""

            def _get_api_instance(
                core_api_service: CoreAPIService = Depends(
                    get_core_api_service
                ),
            ):
                """Dependency to get the appropriate API instance."""
                if module_name not in self._api_instances:
                    if module_name == "system":
                        self._api_instances[module_name] = create_system_api(
                            core_api_service
                        )
                    elif module_name == "genome":
                        self._api_instances[module_name] = create_genome_api(
                            core_api_service
                        )
                    elif module_name == "physiology":
                        self._api_instances[module_name] = (
                            create_physiology_api(core_api_service)
                        )
                    elif module_name == "cortical_area":
                        self._api_instances[module_name] = (
                            create_cortical_area_api(core_api_service)
                        )
                    elif module_name == "connectome":
                        self._api_instances[module_name] = (
                            create_connectome_api(core_api_service)
                        )
                    elif module_name == "burst_engine":
                        self._api_instances[module_name] = (
                            create_burst_engine_api(core_api_service)
                        )
                    elif module_name == "neuroplasticity":
                        self._api_instances[module_name] = (
                            create_neuroplasticity_api(core_api_service)
                        )
                    elif module_name == "region":
                        self._api_instances[module_name] = create_region_api(
                            core_api_service
                        )
                    elif module_name == "morphology":
                        self._api_instances[module_name] = (
                            create_morphology_api(core_api_service)
                        )
                    elif module_name == "monitoring":
                        self._api_instances[module_name] = (
                            create_monitoring_api(core_api_service)
                        )
                    elif module_name == "simulation":
                        self._api_instances[module_name] = (
                            create_simulation_api(core_api_service)
                        )
                    elif module_name == "feagi_agent":
                        self._api_instances[module_name] = (
                            create_feagi_agent_api(core_api_service)
                        )
                    elif module_name == "insights":
                        self._api_instances[module_name] = create_insights_api(
                            core_api_service
                        )
                    elif module_name == "training":
                        self._api_instances[module_name] = create_training_api(
                            core_api_service
                        )
                    elif module_name == "cortical_mapping":
                        self._api_instances[module_name] = (
                            create_cortical_mapping_api(core_api_service)
                        )
                    elif module_name == "network":
                        self._api_instances[module_name] = create_network_api(
                            core_api_service
                        )
                    elif module_name == "inputs":
                        self._api_instances[module_name] = create_inputs_api(
                            core_api_service
                        )
                    elif module_name == "outputs":
                        self._api_instances[module_name] = create_outputs_api(
                            core_api_service
                        )
                    elif module_name == "evolution":
                        self._api_instances[module_name] = (
                            create_evolution_api(core_api_service)
                        )
                    elif module_name == "snapshot":
                        self._api_instances[module_name] = create_snapshot_api(
                            core_api_service
                        )
                    else:
                        raise ValueError(f"Unknown module: {module_name}")

                return self._api_instances[module_name]

            # Inspect the original handler to understand its parameters
            sig = inspect.signature(original_handler)
            params = list(sig.parameters.keys())
            param_annotations = {
                name: param.annotation
                for name, param in sig.parameters.items()
            }

            # Remove 'self' from params for analysis
            handler_params = [p for p in params if p != "self"]

            # Check if any parameter is UploadFile
            has_upload_file = any(
                annotation == UploadFile
                or (
                    hasattr(annotation, "__origin__")
                    and getattr(annotation, "__origin__", None) is UploadFile
                )
                for annotation in param_annotations.values()
            )

            # Get the path for this specific handler (need to access from endpoint_data)
            endpoint_path = endpoint_data.get("path", "")

            # Determine handler characteristics - check if path contains path parameters
            has_path_params = (
                "{" in endpoint_path
                and "}" in endpoint_path
                and len(handler_params) > 0
                and not request_model
                and not has_upload_file
            )
            has_request_body = request_model is not None
            is_async = asyncio.iscoroutinefunction(original_handler)

            if has_upload_file:
                # Special handling for file upload endpoints
                if is_async:

                    async def fastapi_handler_with_file(
                        file: UploadFile,
                        api_instance=Depends(_get_api_instance),
                    ):
                        try:
                            result = await original_handler(api_instance, file)
                            return _maybe_file_response(result)
                        except ValueError as e:
                            raise HTTPException(
                                status_code=400, detail=str(e)
                            ) from e
                        except Exception as e:
                            logger.error(
                                f"Error in {original_handler.__name__}: {e}"
                            )
                            raise HTTPException(
                                status_code=500, detail="Internal server error"
                            ) from e

                    return fastapi_handler_with_file
                else:

                    def fastapi_handler_with_file(
                        file: UploadFile,
                        api_instance=Depends(_get_api_instance),
                    ):
                        try:
                            result = original_handler(api_instance, file)
                            return _maybe_file_response(result)
                        except ValueError as e:
                            raise HTTPException(
                                status_code=400, detail=str(e)
                            ) from e
                        except Exception as e:
                            logger.error(
                                f"Error in {original_handler.__name__}: {e}"
                            )
                            raise HTTPException(
                                status_code=500, detail="Internal server error"
                            ) from e

                    return fastapi_handler_with_file

            elif has_request_body and has_path_params:
                # Handler has both path parameters AND request body
                if is_async:

                    async def fastapi_handler_with_both(
                        request_data: request_model,
                        api_instance=Depends(_get_api_instance),
                        **path_params,
                    ):
                        try:
                            # Pass path parameters first, then request body
                            args = list(path_params.values()) + [request_data]
                            result = await original_handler(
                                api_instance, *args
                            )
                            return _maybe_file_response(result)
                        except ValueError as e:
                            raise HTTPException(
                                status_code=400, detail=str(e)
                            ) from e
                        except Exception as e:
                            logger.error(
                                f"Error in {original_handler.__name__}: {e}"
                            )
                            raise HTTPException(
                                status_code=500, detail="Internal server error"
                            ) from e

                    return fastapi_handler_with_both
                else:

                    def fastapi_handler_with_both(
                        request_data: request_model,
                        api_instance=Depends(_get_api_instance),
                        **path_params,
                    ):
                        try:
                            # Pass path parameters first, then request body
                            args = list(path_params.values()) + [request_data]
                            result = original_handler(api_instance, *args)
                            return _maybe_file_response(result)
                        except ValueError as e:
                            raise HTTPException(
                                status_code=400, detail=str(e)
                            ) from e
                        except Exception as e:
                            logger.error(
                                f"Error in {original_handler.__name__}: {e}"
                            )
                            raise HTTPException(
                                status_code=500, detail="Internal server error"
                            ) from e

                    return fastapi_handler_with_both

            elif has_request_body:
                # Handler expects request body only
                if is_async:

                    async def fastapi_handler_with_request(
                        request_data: request_model,
                        api_instance=Depends(_get_api_instance),
                    ):
                        try:
                            result = await original_handler(
                                api_instance, request_data
                            )
                            return _maybe_file_response(result)
                        except ValueError as e:
                            raise HTTPException(
                                status_code=400, detail=str(e)
                            ) from e
                        except Exception as e:
                            logger.error(
                                f"Error in {original_handler.__name__}: {e}"
                            )
                            raise HTTPException(
                                status_code=500, detail="Internal server error"
                            ) from e

                    return fastapi_handler_with_request
                else:

                    def fastapi_handler_with_request(
                        request_data: request_model,
                        api_instance=Depends(_get_api_instance),
                    ):
                        try:
                            result = original_handler(
                                api_instance, request_data
                            )
                            return _maybe_file_response(result)
                        except ValueError as e:
                            raise HTTPException(
                                status_code=400, detail=str(e)
                            ) from e
                        except Exception as e:
                            logger.error(
                                f"Error in {original_handler.__name__}: {e}"
                            )
                            raise HTTPException(
                                status_code=500, detail="Internal server error"
                            ) from e

                    return fastapi_handler_with_request

            elif has_path_params:
                # Handler expects path parameters only (like /properties/{agent_id})
                # Extract parameter names from the path
                import re

                path_param_names = re.findall(r"\{(\w+)\}", endpoint_path)

                if is_async:
                    # Build handler function dynamically with explicit path parameters
                    if len(path_param_names) == 1:
                        param_name = path_param_names[0]

                        # Create a proper function signature dynamically
                        if param_name == "cortical_area":

                            async def fastapi_handler_with_cortical_area(
                                cortical_area: str,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = await original_handler(
                                        api_instance, cortical_area
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                            return fastapi_handler_with_cortical_area
                        elif param_name == "agent_id":

                            async def fastapi_handler_with_agent_id(
                                agent_id: str,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = await original_handler(
                                        api_instance, agent_id
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                            return fastapi_handler_with_agent_id
                        elif param_name == "neuron_id":

                            async def fastapi_handler_with_neuron_id(
                                neuron_id: int,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = await original_handler(
                                        api_instance, neuron_id
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                            return fastapi_handler_with_neuron_id
                        elif param_name == "cortical_id":

                            async def fastapi_handler_with_cortical_id(
                                cortical_id: str,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = await original_handler(
                                        api_instance, cortical_id
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                            return fastapi_handler_with_cortical_id
                        elif param_name == "snapshot_id":

                            async def fastapi_handler_with_snapshot_id(
                                snapshot_id: str,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = await original_handler(
                                        api_instance, snapshot_id
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                            return fastapi_handler_with_snapshot_id
                        else:
                            # Generic parameter handling
                            async def fastapi_handler_with_path_params(
                                api_instance=Depends(_get_api_instance),
                                **path_params,
                            ):
                                try:
                                    value = path_params.get(param_name)
                                    result = await original_handler(
                                        api_instance, value
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                    else:
                        # Handle multiple path parameters
                        if path_param_names == ["snapshot_id", "fmt"]:

                            async def fastapi_handler_with_snapshot_id_and_fmt(
                                snapshot_id: str,
                                fmt: str,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = await original_handler(
                                        api_instance, snapshot_id, fmt
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                        async def fastapi_handler_with_path_params(
                            api_instance=Depends(_get_api_instance),
                            **path_params,
                        ):
                            try:
                                # Pass path parameters in the order they appear in the method signature
                                args = [
                                    path_params[param]
                                    for param in handler_params
                                    if param in path_params
                                ]
                                result = await original_handler(
                                    api_instance, *args
                                )
                                return _maybe_file_response(result)
                            except ValueError as e:
                                raise HTTPException(
                                    status_code=400, detail=str(e)
                                ) from e
                            except Exception as e:
                                logger.error(
                                    f"Error in {original_handler.__name__}: {e}"
                                )
                                raise HTTPException(
                                    status_code=500,
                                    detail="Internal server error",
                                ) from e

                    return fastapi_handler_with_path_params
                else:
                    # Build handler function dynamically with explicit path parameters
                    if len(path_param_names) == 1:
                        param_name = path_param_names[0]

                        # Create a proper function signature dynamically
                        if param_name == "cortical_area":

                            def fastapi_handler_with_cortical_area(
                                cortical_area: str,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = original_handler(
                                        api_instance, cortical_area
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                            return fastapi_handler_with_cortical_area
                        elif param_name == "agent_id":

                            def fastapi_handler_with_agent_id(
                                agent_id: str,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = original_handler(
                                        api_instance, agent_id
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                            return fastapi_handler_with_agent_id
                        elif param_name == "neuron_id":

                            def fastapi_handler_with_neuron_id(
                                neuron_id: int,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = original_handler(
                                        api_instance, neuron_id
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                            return fastapi_handler_with_neuron_id
                        elif param_name == "cortical_id":

                            def fastapi_handler_with_cortical_id(
                                cortical_id: str,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = original_handler(
                                        api_instance, cortical_id
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                            return fastapi_handler_with_cortical_id
                        elif param_name == "snapshot_id":

                            def fastapi_handler_with_snapshot_id(
                                snapshot_id: str,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = original_handler(
                                        api_instance, snapshot_id
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                            return fastapi_handler_with_snapshot_id
                        else:
                            # Generic parameter handling
                            def fastapi_handler_with_path_params(
                                api_instance=Depends(_get_api_instance),
                                **path_params,
                            ):
                                try:
                                    value = path_params.get(param_name)
                                    result = original_handler(
                                        api_instance, value
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                    else:
                        # Handle multiple path parameters
                        if path_param_names == ["snapshot_id", "fmt"]:

                            def fastapi_handler_with_snapshot_id_and_fmt(
                                snapshot_id: str,
                                fmt: str,
                                api_instance=Depends(_get_api_instance),
                            ):
                                try:
                                    result = original_handler(
                                        api_instance, snapshot_id, fmt
                                    )
                                    return _maybe_file_response(result)
                                except ValueError as e:
                                    raise HTTPException(
                                        status_code=400, detail=str(e)
                                    ) from e
                                except Exception as e:
                                    logger.error(
                                        f"Error in {original_handler.__name__}: {e}"
                                    )
                                    raise HTTPException(
                                        status_code=500,
                                        detail="Internal server error",
                                    ) from e

                        def fastapi_handler_with_path_params(
                            api_instance=Depends(_get_api_instance),
                            **path_params,
                        ):
                            try:
                                # Pass path parameters in the order they appear in the method signature
                                args = [
                                    path_params[param]
                                    for param in handler_params
                                    if param in path_params
                                ]
                                result = original_handler(api_instance, *args)
                                return _maybe_file_response(result)
                            except ValueError as e:
                                raise HTTPException(
                                    status_code=400, detail=str(e)
                                ) from e
                            except Exception as e:
                                logger.error(
                                    f"Error in {original_handler.__name__}: {e}"
                                )
                                raise HTTPException(
                                    status_code=500,
                                    detail="Internal server error",
                                ) from e

                    return fastapi_handler_with_path_params

            else:
                # Handler doesn't need parameters (simple GET endpoints)
                if is_async:

                    async def fastapi_handler_simple(
                        api_instance=Depends(_get_api_instance),
                    ):
                        try:
                            result = await original_handler(api_instance)
                            return _maybe_file_response(result)
                        except ValueError as e:
                            raise HTTPException(
                                status_code=400, detail=str(e)
                            ) from e
                        except Exception as e:
                            logger.error(
                                f"Error in {original_handler.__name__}: {e}"
                            )
                            raise HTTPException(
                                status_code=500, detail="Internal server error"
                            ) from e

                    return fastapi_handler_simple
                else:

                    def fastapi_handler_simple(
                        api_instance=Depends(_get_api_instance),
                    ):
                        try:
                            result = original_handler(api_instance)
                            return _maybe_file_response(result)
                        except ValueError as e:
                            raise HTTPException(
                                status_code=400, detail=str(e)
                            ) from e
                        except Exception as e:
                            logger.error(
                                f"Error in {original_handler.__name__}: {e}"
                            )
                            raise HTTPException(
                                status_code=500, detail="Internal server error"
                            ) from e

                    return fastapi_handler_simple

    def create_system_router() -> APIRouter:
        """Create a FastAPI router for system endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("system")

    def create_genome_router() -> APIRouter:
        """Create a FastAPI router for genome endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("genome")

    def create_cortical_area_router() -> APIRouter:
        """Create a FastAPI router for cortical area endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("cortical_area")

    # === NEW ROUTER CREATION FUNCTIONS ===

    def create_connectome_router() -> APIRouter:
        """Create a FastAPI router for connectome endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("connectome")

    def create_burst_engine_router() -> APIRouter:
        """Create a FastAPI router for burst engine endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("burst_engine")

    def create_neuroplasticity_router() -> APIRouter:
        """Create a FastAPI router for neuroplasticity endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("neuroplasticity")

    def create_region_router() -> APIRouter:
        """Create a FastAPI router for region endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("region")

    def create_morphology_router() -> APIRouter:
        """Create a FastAPI router for morphology endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("morphology")

    def create_monitoring_router() -> APIRouter:
        """Create a FastAPI router for monitoring endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("monitoring")

    def create_simulation_router() -> APIRouter:
        """Create a FastAPI router for simulation endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("simulation")

    def create_feagi_agent_router() -> APIRouter:
        """Create a FastAPI router for feagi agent endpoints."""
        wrapper = UniversalFastAPIWrapper()
        router = wrapper.create_router_for_module("feagi_agent")

        # MANUAL ADDITION: Query parameter version of agent properties endpoint
        # The universal wrapper doesn't support query parameters, so we add this manually
        from fastapi import Depends, HTTPException

        from feagi.api.rest.dependencies import get_core_api_service
        from feagi.api.v1.feagi_agent import create_feagi_agent_api
        from feagi.api.v1.schemas import AgentPropertiesResponse

        @router.get("/properties", response_model=AgentPropertiesResponse)
        async def get_agent_properties_query(
            agent_id: str, core_api_service=Depends(get_core_api_service)
        ):
            """Get agent properties using query parameter format.

            This endpoint supports: /v1/agent/properties?agent_id=<agent_id>
            """
            try:
                agent_api = create_feagi_agent_api(core_api_service)
                return await agent_api.get_agent_properties_query(agent_id)
            except ValueError as e:
                raise HTTPException(status_code=400, detail=str(e)) from e
            except Exception:
                raise HTTPException(
                    status_code=500, detail="Internal server error"
                ) from None

        return router

    def create_insights_router() -> APIRouter:
        """Create a FastAPI router for insights endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("insights")

    def create_training_router() -> APIRouter:
        """Create a FastAPI router for training endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("training")

    def create_cortical_mapping_router() -> APIRouter:
        """Create a FastAPI router for cortical mapping endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("cortical_mapping")

    def create_network_router() -> APIRouter:
        """Create a FastAPI router for network endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("network")

    def create_inputs_router() -> APIRouter:
        """Create a FastAPI router for inputs endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("inputs")

    def create_outputs_router() -> APIRouter:
        """Create a FastAPI router for outputs endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("outputs")

    def create_evolution_router() -> APIRouter:
        """Create a FastAPI router for evolution endpoints."""
        wrapper = UniversalFastAPIWrapper()
        return wrapper.create_router_for_module("evolution")

    def create_snapshot_router() -> APIRouter:
        """Create a FastAPI router for snapshot endpoints (manual wiring to
        avoid param issues)."""
        # @ruff-skip: module has >100 violations - cleanup task: SNAP-ROUTER-RUFF-CLEANUP
        import os

        from fastapi import APIRouter, Depends, HTTPException
        from fastapi.responses import FileResponse
        from starlette.background import BackgroundTask

        from feagi.api.v1.snapshot import (
            SnapshotAPI,
            SnapshotCreateRequest,
            SnapshotCreateResponse,
            SnapshotRestoreRequest,
            SnapshotRestoreResponse,
        )

        router = APIRouter()

        @router.post(
            "/",
            response_model=SnapshotCreateResponse,
            summary="Create brain snapshot",
            description=(
                "Create a brain snapshot folder (manifest + JSON summaries).\n\n"
                "Optional: persist an artifact.\n\n"
                "- format='fgc' with profile='model' → writes <id>/<id>.fgc (FEAGI model container).\n"
                "- format='fgc' with profile='stateful' → writes <id>/<id>.fgs (stateful container).\n"
                "- format='zip' → streams a ZIP when downloaded (not persisted unless requested).\n\n"
                "Compression: 'store' (no compression, best for mmap) or 'deflate'.\n\n"
                "Requires [snapshot] output_dir and temp_dir in feagi_configuration.toml."
            ),
        )
        async def create_snapshot(
            request: SnapshotCreateRequest,
            core_api_service=Depends(get_core_api_service),
        ):
            try:
                api = SnapshotAPI(core_api_service)
                return await api.create_snapshot(request)
            except ValueError as e:
                raise HTTPException(status_code=400, detail=str(e)) from e
            except Exception:
                raise HTTPException(
                    status_code=500, detail="Internal server error"
                ) from None

        @router.get(
            "/{snapshot_id}/artifact/{fmt}",
            summary="Download snapshot artifact",
            description=(
                "Download snapshot artifact as .fgc/.fgs or .zip.\n\n"
                "- .fgc: served from <id>/<id>.fgc; built on-demand if missing\n"
                "- .fgs: served from <id>/<id>.fgs (stateful)\n"
                "- .zip: packaged on-demand to temp and streamed (not persisted by default)"
            ),
        )
        async def get_snapshot_artifact(
            snapshot_id: str,
            fmt: str,
            core_api_service=Depends(get_core_api_service),
        ):
            try:
                api = SnapshotAPI(core_api_service)
                result = await api.get_snapshot_artifact(snapshot_id, fmt)
                if (
                    isinstance(result, dict)
                    and "path" in result
                    and result.get("filename")
                ):
                    file_path = result["path"]
                    filename = result["filename"]
                    if fmt.lower() == "zip":
                        # Clean up the temporary zip after sending
                        def _cleanup():
                            try:
                                os.remove(file_path)
                            except Exception:
                                pass

                        return FileResponse(
                            path=file_path,
                            filename=filename,
                            media_type="application/zip",
                            background=BackgroundTask(_cleanup),
                        )
                    else:
                        # For .fc via this path, the file is persisted; no cleanup here
                        return FileResponse(
                            path=file_path,
                            filename=filename,
                            media_type="application/octet-stream",
                        )
                return result
            except ValueError as e:
                raise HTTPException(status_code=400, detail=str(e)) from e
            except Exception:
                raise HTTPException(
                    status_code=500, detail="Internal server error"
                ) from None

        @router.get("/stream")
        async def stream_snapshot(
            stateful: bool = False,
            compression: bool = True,
            core_api_service=Depends(get_core_api_service),
        ):
            """Create and stream a snapshot without persisting artifacts.

            stateful=true → .fgs; false → .fgc. Always builds to temp_dir and
            deletes after send.
            """
            try:
                # Create folder snapshot first
                api = SnapshotAPI(core_api_service)
                resp = await api.create_snapshot(
                    SnapshotCreateRequest(stateful=stateful, compression=False)
                )
                sid = resp["snapshot_id"]
                # Build container to temp and stream
                from feagi.config.toml_loader import load_feagi_config

                cfg = load_feagi_config()
                snap_cfg = cfg.get("snapshot", {})
                temp_dir = snap_cfg.get("temp_dir")
                if not temp_dir:
                    raise HTTPException(
                        status_code=400, detail="Snapshot temp_dir is required"
                    )
                # Create container in temp
                from pathlib import Path

                from feagi.core.snapshot.container import (
                    MAGIC_FGC,
                    MAGIC_FGS,
                    create_fc_snapshot_from_folder,
                )

                sdir = Path(resp["path"])  # folder just created
                tmpdir = Path(temp_dir)
                tmpdir.mkdir(parents=True, exist_ok=True)
                ext = ".fgs" if stateful else ".fgc"
                magic = MAGIC_FGS if stateful else MAGIC_FGC
                out = tmpdir / f"{sid}{ext}"
                # Build FC container directly in temp_dir
                _ = create_fc_snapshot_from_folder(
                    snapshot_dir=sdir,
                    snapshot_id=sid,
                    compression="store",
                    magic=magic,
                    extension=ext,
                    destination_dir=tmpdir,
                )

                # Stream and cleanup
                def _cleanup():
                    try:
                        os.remove(out)
                    except Exception:
                        pass

                return FileResponse(
                    path=str(out),
                    filename=f"{sid}{ext}",
                    media_type="application/octet-stream",
                    background=BackgroundTask(_cleanup),
                )
            except HTTPException:
                raise
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e)) from e

        @router.post(
            "/{snapshot_id}/restore",
            response_model=SnapshotRestoreResponse,
            summary="Restore snapshot",
            description=(
                "Restore a snapshot by id.\n\n"
                "Profile (default 'model'):\n"
                "- 'model' → uses <id>/<id>.fgc if present; otherwise restores from folder manifest.\n"
                "- 'stateful' → requires <id>/<id>.fgs.\n\n"
                "Mode (default from [snapshot].fc_restore_mode):\n"
                "- 'mmap' (zero-copy, requires store-encoded arrays)\n"
                "- 'load' (copies into RAM)."
            ),
        )
        async def restore_snapshot(
            snapshot_id: str,
            request: SnapshotRestoreRequest | None = None,
            core_api_service=Depends(get_core_api_service),
        ):
            try:
                api = SnapshotAPI(core_api_service)
                parsed = request or SnapshotRestoreRequest()
                return await api.restore_snapshot(snapshot_id, parsed)
            except ValueError as e:
                raise HTTPException(status_code=400, detail=str(e)) from e
            except Exception:
                raise HTTPException(
                    status_code=500, detail="Internal server error"
                ) from None

        @router.post(
            "/upload",
            summary="Upload and restore a snapshot file (.fgc/.fgs)",
            description=(
                "Accepts a .fgc (model) or .fgs (stateful) file, stages it in a temporary folder, "
                "places it under the snapshot output directory, then performs restore."
            ),
        )
        async def upload_snapshot(
            file: UploadFile,
            mode: str = "load",
            core_api_service=Depends(get_core_api_service),
        ):
            try:
                from pathlib import Path

                from feagi.config.toml_loader import load_feagi_config
                from feagi.core.snapshot.container import (
                    MAGIC_FGS,
                    read_fc_header,
                )

                cfg = load_feagi_config()
                snap_cfg = cfg.get("snapshot", {})
                out_root = Path(snap_cfg.get("output_dir", ""))
                tmp_root = Path(snap_cfg.get("temp_dir", ""))
                if not out_root:
                    raise HTTPException(
                        status_code=400,
                        detail="Snapshot output_dir is required",
                    )
                if not tmp_root:
                    raise HTTPException(
                        status_code=400, detail="Snapshot temp_dir is required"
                    )
                tmp_root.mkdir(parents=True, exist_ok=True)
                # Save uploaded file to temp
                import os
                import shutil
                import uuid

                tmp_name = f"upload-{uuid.uuid4().hex}"
                tmp_path = tmp_root / tmp_name
                with open(tmp_path, "wb") as f:
                    shutil.copyfileobj(file.file, f)
                # Inspect header to determine type
                header = read_fc_header(tmp_path)
                magic = header.get("_magic", "")
                is_stateful = magic == MAGIC_FGS.decode(
                    "ascii", errors="ignore"
                )
                # Derive snapshot_id from original filename stem or generate
                stem = Path(file.filename or "uploaded").stem
                if not stem:
                    stem = f"uploaded-{uuid.uuid4().hex}"
                sid = stem
                # Create final folder and move file
                dest_dir = out_root / sid
                dest_dir.mkdir(parents=True, exist_ok=True)
                ext = ".fgs" if is_stateful else ".fgc"
                dest = dest_dir / f"{sid}{ext}"
                try:
                    os.replace(tmp_path, dest)
                except Exception:
                    shutil.copyfile(tmp_path, dest)
                    os.remove(tmp_path)
                # Restore
                api = SnapshotAPI(core_api_service)
                parsed_mode = mode.lower() if isinstance(mode, str) else "load"
                # Enforce mmap eligibility via existing logic inside restore endpoints
                from feagi.api.v1.snapshot import SnapshotRestoreRequest

                req = SnapshotRestoreRequest(
                    mode=parsed_mode,
                    profile=("stateful" if is_stateful else "model"),
                )
                result = await api.restore_snapshot(sid, req)
                return result | {"snapshot_id": sid}
            except HTTPException:
                raise
            except ValueError as e:
                raise HTTPException(status_code=400, detail=str(e)) from e
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e)) from e

        @router.get(
            "/",
            summary="List snapshots",
            description="List available snapshot ids (folder names under [snapshot].output_dir).",
        )
        async def list_snapshots(
            core_api_service=Depends(get_core_api_service),
        ):
            try:
                api = SnapshotAPI(core_api_service)
                return await api.list_snapshots()
            except ValueError as e:
                raise HTTPException(status_code=400, detail=str(e)) from e
            except Exception:
                raise HTTPException(
                    status_code=500, detail="Internal server error"
                ) from None

        @router.delete(
            "/{snapshot_id}",
            summary="Delete snapshot",
            description="Delete a snapshot folder and all of its contents (artifacts included).",
        )
        async def delete_snapshot(
            snapshot_id: str, core_api_service=Depends(get_core_api_service)
        ):
            try:
                api = SnapshotAPI(core_api_service)
                return await api.delete_snapshot(snapshot_id)
            except ValueError as e:
                raise HTTPException(status_code=400, detail=str(e)) from e
            except Exception:
                raise HTTPException(
                    status_code=500, detail="Internal server error"
                ) from None

        return router

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

    def get_connectome_router() -> APIRouter:
        """Get the connectome router for use in main FastAPI app."""
        return create_connectome_router()

    def get_burst_engine_router() -> APIRouter:
        """Get the burst engine router for use in main FastAPI app."""
        return create_burst_engine_router()

    def get_neuroplasticity_router() -> APIRouter:
        """Get the neuroplasticity router for use in main FastAPI app."""
        return create_neuroplasticity_router()

    def get_region_router() -> APIRouter:
        """Get the region router for use in main FastAPI app."""
        return create_region_router()

    def get_morphology_router() -> APIRouter:
        """Get the morphology router for use in main FastAPI app."""
        return create_morphology_router()

    def get_monitoring_router() -> APIRouter:
        """Get the monitoring router for use in main FastAPI app."""
        return create_monitoring_router()

    def get_simulation_router() -> APIRouter:
        """Get the simulation router for use in main FastAPI app."""
        return create_simulation_router()

    def get_feagi_agent_router() -> APIRouter:
        """Get the feagi agent router for use in main FastAPI app."""
        return create_feagi_agent_router()

    def get_insights_router() -> APIRouter:
        """Get the insights router for use in main FastAPI app."""
        return create_insights_router()

    def get_training_router() -> APIRouter:
        """Get the training router for use in main FastAPI app."""
        return create_training_router()

    def get_cortical_mapping_router() -> APIRouter:
        """Get the cortical mapping router for use in main FastAPI app."""
        return create_cortical_mapping_router()

    def get_network_router() -> APIRouter:
        """Get the network router for use in main FastAPI app."""
        return create_network_router()

    def get_inputs_router() -> APIRouter:
        """Get the inputs router for use in main FastAPI app."""
        return create_inputs_router()

    def get_outputs_router() -> APIRouter:
        """Get the outputs router for use in main FastAPI app."""
        return create_outputs_router()

    def get_evolution_router() -> APIRouter:
        """Get the evolution router for use in main FastAPI app."""
        return create_evolution_router()

    def get_snapshot_router() -> APIRouter:
        """Get the snapshot router for use in main FastAPI app."""
        return create_snapshot_router()

    # Force import of API classes to ensure endpoint registration
