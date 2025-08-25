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
FEAGI v1 Cortical Area API - Single Source of Truth

This module contains the ONLY definitions of cortical area API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Any, Dict, List

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.v1.schemas import (
    CorticalAreaIdListResponse,
    CorticalAreaIndexListResponse,
    CorticalAreaMappingRestrictionRequest,
    CorticalAreaMappingRestrictionResponse,
    CorticalAreaMemoryUsageResponse,
    CorticalAreaNameListResponse,
    CorticalAreaPropertiesResponse,
    CorticalAreaTypesResponse,
    CorticalIdListRequest,
    CorticalIdNameMappingResponse,
    CorticalIdRequest,
    CorticalLocationResponse,
    CorticalNameRequest,
    CorticalPropertiesUpdateRequest,
    AddCoreCorticalAreaRequest,
    CustomCorticalAreaRequest,
    MappingRestrictionsRequest,
    MappingRestrictionsResponse,
    NeuronCountResponse,
    SuccessResponse,
)
from feagi.bdu.models.cortical_area import generate_cortical_id
from feagi.utils.logger import setup_logger

from .decorators import endpoint

logger = setup_logger(__name__)


# Define the convenience decorator for cortical area endpoints
def cortical_area_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    """Convenience decorator for cortical area endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="cortical_area",
    )


class CorticalAreaAPI:
    """
    Cortical Area API - Single Source of Truth for ALL Transports

    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.

    This ensures identical behavior across all transports with zero duplication.
    """

    def __init__(self, core_api_service: CoreAPIService):
        """Initialize with core API service dependency."""
        self.core_api_service = core_api_service
        # Cache templates to avoid repeated imports
        self._templates_cache = None

    def _get_template_defaults(self) -> Dict[str, Any]:
        """
        Get default values from templates.py - THE SINGLE SOURCE OF TRUTH for defaults.

        This method imports the cortical_template and returns the default values
        that should be used when properties are missing from the genome data.

        Returns:
            Dict containing default values from cortical_template
        """
        if self._templates_cache is None:
            try:
                from feagi.evo.templates import cortical_template

                self._templates_cache = cortical_template.copy()
            except ImportError:
                logger.error(
                    "CRITICAL: Cannot import cortical_template from templates.py - using minimal fallbacks"
                )
                #  Minimal emergency fallbacks only - should never happen in
                #  normal operation
                self._templates_cache = {}

        return self._templates_cache

    def _get_default_value(self, property_name: str, fallback_value=None):
        """Get default value for a specific property from templates.

        Args:
            property_name: Name of the property in cortical_template
            fallback_value: Emergency fallback if templates unavailable (should never be used)

        Returns:
            Default value from cortical_template or fallback
        """
        defaults = self._get_template_defaults()
        return defaults.get(property_name, fallback_value)

    def _get_structural_default(self, property_type: str):
        """Get acceptable structural/spatial defaults that are not neuron
        properties.

        These are NOT from cortical_template since they are structural, not neural properties.
        They represent spatial positioning and brain organization defaults.

        Args:
            property_type: Type of structural property needed

        Returns:
            Appropriate structural default value
        """
        structural_defaults = {
            # Spatial coordinate defaults - position in 3D brain space
            "coordinate": 0,
            # Spatial dimension defaults - minimum valid brain dimension
            "dimension": 1,
            # Empty collection defaults for structural containers
            "mapping": {},
            "list": [],
            # Brain hierarchy defaults
            "parent_region_id": "root",
            "parent_region_title": "Genome's root brain region",
            "cortical_group": "CUSTOM",
            #  Memory area structural dimensions (always minimal for memory
            #  types)
            "memory_dimensions": [1, 1, 1],
        }
        return structural_defaults.get(property_type, None)

    # ===== Cortical Area Properties =====

    @cortical_area_endpoint(
        "POST",
        "/cortical_area_properties",
        request_model=CorticalIdRequest,
        response_model=CorticalAreaPropertiesResponse,
    )
    def get_cortical_area_properties(
        self, request: CorticalIdRequest
    ) -> CorticalAreaPropertiesResponse:
        """Get properties of a cortical area."""
        try:
            area_data = self.core_api_service.get_cortical_area(
                request.cortical_id
            )
            if not area_data:
                raise KeyError("Cortical area not found")

            # Transform modern FEAGI format to expected legacy format
            parameters = area_data.get("parameters", {})

            #  Extract coordinates using structural defaults for spatial
            #  positioning
            coordinates = area_data.get("coordinates", {})
            if isinstance(coordinates, (list, tuple)):
                # Handle tuple/list format: (x, y, z)
                coordinates_3d = (
                    list(coordinates)
                    if len(coordinates) >= 3
                    else [
                        (
                            coordinates[0]
                            if len(coordinates) > 0
                            else self._get_structural_default("coordinate")
                        ),
                        (
                            coordinates[1]
                            if len(coordinates) > 1
                            else self._get_structural_default("coordinate")
                        ),
                        (
                            coordinates[2]
                            if len(coordinates) > 2
                            else self._get_structural_default("coordinate")
                        ),
                    ]
                )
            else:
                # Handle dict format: {"x": x, "y": y, "z": z}
                coordinates_3d = [
                    coordinates.get(
                        "x", self._get_structural_default("coordinate")
                    ),
                    coordinates.get(
                        "y", self._get_structural_default("coordinate")
                    ),
                    coordinates.get(
                        "z", self._get_structural_default("coordinate")
                    ),
                ]

            #  Extract dimensions using structural defaults for spatial
            #  dimensions
            dimensions = area_data.get("dimensions", {})
            if isinstance(dimensions, (list, tuple)):
                # Handle tuple/list format: (width, height, depth)
                cortical_dimensions = (
                    list(dimensions)
                    if len(dimensions) >= 3
                    else [
                        (
                            dimensions[0]
                            if len(dimensions) > 0
                            else self._get_structural_default("dimension")
                        ),
                        (
                            dimensions[1]
                            if len(dimensions) > 1
                            else self._get_structural_default("dimension")
                        ),
                        (
                            dimensions[2]
                            if len(dimensions) > 2
                            else self._get_structural_default("dimension")
                        ),
                    ]
                )
            else:
                # Handle dict format: {"width": w, "height": h, "depth": d}
                cortical_dimensions = [
                    dimensions.get(
                        "width", self._get_structural_default("dimension")
                    ),
                    dimensions.get(
                        "height", self._get_structural_default("dimension")
                    ),
                    dimensions.get(
                        "depth", self._get_structural_default("dimension")
                    ),
                ]

            #  Build legacy format response with STRICT values (no fallbacks)
            #  - Structural: must exist in area_data
            #  - Neural: must exist either in computed area_data fields or explicit parameters
            legacy_properties = {
                "cortical_id": area_data.get("id", request.cortical_id),
                "cortical_idx": area_data.get(
                    "cortical_idx"
                ),  # CRITICAL FIX: Include cortical_idx in API response
                "cortical_name": area_data.get("name", request.cortical_id),
                "parent_region_id": parameters.get(
                    "parent_region_id",
                    self._get_structural_default("parent_region_id"),
                ),
                "parent_region_title": parameters.get(
                    "parent_region_title",
                    self._get_structural_default("parent_region_title"),
                ),
                "cortical_group": area_data["type"],
                "cortical_sub_group": parameters.get(
                    "subgroup", self._get_default_value("sub_group_id", "")
                ),
                "cortical_neuron_per_vox_count": parameters.get(
                    "neurons_per_voxel", parameters.get("per_voxel_neuron_cnt", 1)
                ),
                "cortical_visibility": parameters.get("gd_vis", False),
                "cortical_synaptic_attractivity": parameters.get("synatt", 0),
                "coordinates_3d": coordinates_3d,
                "coordinates_2d": [
                    parameters.get(
                        "2dcorx", self._get_structural_default("coordinate")
                    ),
                    parameters.get(
                        "2dcory", self._get_structural_default("coordinate")
                    ),
                ],
                "cortical_dimensions": cortical_dimensions,
                "cortical_destinations": parameters.get("mapping", {}),
                "neuron_post_synaptic_potential": float(parameters.get("pstcr", 0.0)),
                "neuron_post_synaptic_potential_max": float(parameters.get("pstcrm", 0.0)),
                "neuron_fire_threshold": float(area_data.get("firing_threshold", 1.0)),
                "neuron_fire_threshold_increment": [
                    float(parameters.get("ftincx", 0.0)),
                    float(parameters.get("ftincy", 0.0)),
                    float(parameters.get("ftincz", 0.0)),
                ],
                "neuron_firing_threshold_limit": int(parameters.get("fthlim", 0)),
                "neuron_refractory_period": int(area_data.get("refractory_period", 0)),
                "neuron_leak_coefficient": float(area_data.get("leak_coefficient", 0.0)),
                "neuron_leak_variability": float(parameters.get("leak_v", 0.0)),
                "neuron_consecutive_fire_count": int(parameters.get("c_fr_c", 0)),
                "neuron_snooze_period": int(parameters.get("snooze", 0)),
                "neuron_degeneracy_coefficient": int(parameters.get("de_gen", 0)),
                "neuron_psp_uniform_distribution": bool(parameters.get("pspuni", 0)),
                "neuron_mp_charge_accumulation": bool(parameters.get("mp_acc", 0)),
                "neuron_mp_driven_psp": bool(parameters.get("mp_psp", 0)),
                "neuron_longterm_mem_threshold": int(parameters.get("mem__t", 0)),
                "neuron_lifespan_growth_rate": float(parameters.get("mem_gr", 0.0)),
                "neuron_init_lifespan": int(parameters.get("mem_ls", 0)),
                "temporal_depth": int(parameters.get("temporal_depth", 1)),
                "neuron_excitability": float(area_data.get("neuron_excitability", 1.0)),
                "transforming": parameters.get(
                    "transforming", False
                ),  # Runtime state flag - not from templates
            }

            return CorticalAreaPropertiesResponse(properties=legacy_properties)
        except ValueError:
            raise ValueError("Invalid cortical area ID length") from None
        except KeyError:
            raise ValueError("Cortical area not found") from None
        except Exception as e:
            raise ValueError(
                f"Error retrieving cortical properties: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "PUT",
        "/cortical_area",
        request_model=CorticalPropertiesUpdateRequest,
        response_model=SuccessResponse,
    )
    def update_cortical_area_properties(
        self, request: CorticalPropertiesUpdateRequest
    ) -> SuccessResponse:
        """Update properties of a cortical area."""
        try:
            # Convert request to dictionary for backward compatibility
            properties = request.model_dump()

            # Extract cortical_id and remove it from properties
            cortical_id = properties.get("cortical_id", None)
            if not cortical_id:
                raise ValueError("cortical_id is required")

            #  Create a copy and remove cortical_id from the properties to
            #  update
            properties_to_update = properties.copy()
            properties_to_update.pop("cortical_id", None)

            # Call the update operation
            result = self.core_api_service.update_cortical_area_properties(
                cortical_id, properties_to_update
            )

            if result:
                return SuccessResponse(
                    success=True,
                    message="Cortical area properties updated successfully",
                )
            else:
                # Check if the cortical area exists for better error messaging
                existing_area = self.core_api_service.get_cortical_area(
                    cortical_id
                )
                if not existing_area:
                    raise ValueError(
                        f"Cortical area '{cortical_id}' does not exist"
                    )
                else:
                    raise ValueError(
                        "Failed to update cortical area properties - write operations may be disabled"
                    )
        except Exception as e:
            raise ValueError(f"Error updating cortical area: {str(e)}") from e

    @cortical_area_endpoint(
        "POST", "/cortical_area", request_model=AddCoreCorticalAreaRequest, response_model=Dict[str, str]
    )
    def add_cortical_area(
        self, new_cortical_properties: AddCoreCorticalAreaRequest
    ) -> Dict[str, str]:
        """Add a new core cortical area."""
        try:
            # Check if connectome is ready
            connectome = self.core_api_service.get_connectome()
            state_manager = self.core_api_service.state_manager
            if not connectome or not state_manager.is_connectome_ready():
                raise ValueError("Connectome is not ready!")

            payload = new_cortical_properties.model_dump()

            # Map request to ConnectomeManager.add_cortical_area signature
            cortical_id = payload.get("cortical_id")
            name = cortical_id or payload.get("name") or "cortical_area"

            coords3 = payload.get("coordinates_3d")
            if not coords3 or len(coords3) != 3:
                raise ValueError("coordinates_3d must be a 3-element list [x, y, z]")
            position = (int(coords3[0]), int(coords3[1]), int(coords3[2]))

            dims = payload.get("cortical_dimensions")
            template_resolution = None
            try:
                # If a core template exists for this cortical_id, use its per-device resolution
                if cortical_id:
                    from feagi.evo.templates import cortical_template

                    if cortical_id in cortical_template:
                        template_resolution = cortical_template[cortical_id].get(
                            "resolution"
                        )
            except Exception:
                template_resolution = None

            if dims and len(dims) == 3:
                dimensions = (int(dims[0]), int(dims[1]), int(dims[2]))
            elif template_resolution and len(template_resolution) == 3:
                # Compute total grid from per-device resolution × device_count (width-wise)
                device_count = int(payload.get("device_count", 1))
                if device_count <= 0:
                    raise ValueError("device_count must be > 0")
                dimensions = (
                    int(template_resolution[0]) * device_count,
                    int(template_resolution[1]),
                    int(template_resolution[2]),
                )
            else:
                # Final fallback: device_count as Nx1x1 if no template found
                device_count = payload.get("device_count")
                if device_count is None:
                    raise ValueError(
                        "cortical_dimensions or device_count (or template resolution) is required"
                    )
                n = int(device_count)
                if n <= 0:
                    raise ValueError("device_count must be > 0")
                dimensions = (n, 1, 1)

            area_type = str(payload.get("cortical_type") or "custom")

            # Pass remaining fields as properties (excluding ones already mapped)
            properties: Dict[str, Any] = {
                k: v
                for k, v in payload.items()
                if k
                not in {
                    "cortical_id",
                    "name",
                    "coordinates_3d",
                    "cortical_dimensions",
                    "device_count",
                    "cortical_type",
                }
            }

            # Preserve per-device resolution in properties if available
            if template_resolution and len(template_resolution) == 3:
                properties["resolution"] = [
                    int(template_resolution[0]),
                    int(template_resolution[1]),
                    int(template_resolution[2]),
                ]

            # Create area through GenomeService to ensure genome blueprint sync
            try:
                genome_service = getattr(self.core_api_service, "_genome_service", None)
                if genome_service and hasattr(genome_service, "create_cortical_area"):
                    created = genome_service.create_cortical_area(
                        name=name,
                        coordinates={"x": position[0], "y": position[1], "z": position[2]},
                        dimensions={
                            "width": dimensions[0],
                            "height": dimensions[1],
                            "depth": dimensions[2],
                        },
                        area_type=area_type,
                        parameters=properties,
                    )
                    if not created or not isinstance(created, dict) or not created.get("cortical_id"):
                        raise ValueError("Genome service did not return created cortical area info")
                else:
                    # Fallback: create directly in connectome (should be rare)
                    connectome.add_cortical_area(
                        name=name,
                        dimensions=dimensions,
                        position=position,
                        area_type=area_type,
                        properties=properties,
                        cortical_id=cortical_id,
                    )
            except Exception as e:
                raise ValueError(f"Failed to create cortical area via genome service: {e}") from e

            # Deterministic readiness gate: ensure area is visible in connectome and mapping
            try:
                from feagi.config.toml_loader import load_feagi_config, get_timeout_config

                cfg = load_feagi_config()
                tcfg = get_timeout_config(cfg)
                # Use a short, bounded wait (<= service_startup seconds)
                import time

                deadline = time.time() + float(tcfg.service_startup)
                target_id = (created.get("cortical_id") if isinstance(created, dict) else None) or cortical_id or name
                while time.time() < deadline:
                    area_obj = connectome.get_cortical_area(target_id)
                    mapped_idx = None
                    try:
                        mapped_idx = connectome.cortical_mapping.get_idx(target_id)
                    except Exception:
                        mapped_idx = None

                    if area_obj is not None and mapped_idx is not None:
                        break
                    time.sleep(0.01)
            except Exception:
                # Non-fatal; proceed even if readiness check fails
                pass

            return {"cortical_id": target_id}
        except Exception as e:
            raise ValueError(f"Error adding cortical area: {str(e)}") from e

    @cortical_area_endpoint(
        "POST",
        "/custom_cortical_area",
        request_model=CustomCorticalAreaRequest,
        response_model=Dict[str, str],
    )
    def add_custom_cortical_area(
        self, request: CustomCorticalAreaRequest
    ) -> Dict[str, str]:
        """Add a new custom cortical area."""
        try:
            # Check if connectome is ready
            connectome = self.core_api_service.get_connectome()
            state_manager = self.core_api_service.state_manager
            if not connectome or not state_manager.is_connectome_ready():
                raise ValueError("Connectome is not ready!")

            # Extract properties from request
            cortical_name = request.cortical_name
            parent_region_id = (
                request.brain_region_id
            )  # brain_region_id maps to parent_region_id
            # Prefer sub_group_id if provided, otherwise use cortical_sub_group
            sub_group_id = request.sub_group_id or request.cortical_sub_group
            copy_of = request.copy_of

            # Validate parent region
            #  ConnectomeManager ensures brain_regions structure exists during
            #  genome loading
            genome_brain_regions = getattr(connectome, "genome", {}).get(
                "brain_regions", {}
            )
            connectome_brain_regions = getattr(connectome, "brain_regions", {})

            if (
                parent_region_id not in genome_brain_regions
                and parent_region_id not in connectome_brain_regions
            ):
                raise ValueError(
                    f"Parent region '{parent_region_id}' does not exist!"
                )

            # Generate cortical ID
            temp_name = cortical_name
            if len(cortical_name) < 3:
                temp_name = cortical_name + "000"

            # Determine if memory area
            is_memory = "MEMORY" in sub_group_id
            if is_memory:
                cortical_dimensions = self._get_structural_default(
                    "memory_dimensions"
                )
            else:
                cortical_dimensions = request.cortical_dimensions

            # Calculate neuron count and validate limits
            if is_memory:
                # MEMORY AREA FIX: Memory areas start empty (0 regular neurons)
                # Memory neurons are created dynamically by MemoryProcessor
                neuron_density = 0
            else:
                # Regular areas use template default
                neuron_density = self._get_default_value(
                    "per_voxel_neuron_cnt", 1
                )

            if copy_of:
                neuron_density = connectome.genome["blueprint"][copy_of][
                    "per_voxel_neuron_cnt"
                ]

            neuron_count = (
                neuron_density
                * cortical_dimensions[0]
                * cortical_dimensions[1]
                * cortical_dimensions[2]
            )
            max_allowable_neuron_count = int(connectome.max_neurons)

            if (
                neuron_count + connectome.get_neuron_count()
                > max_allowable_neuron_count
            ):
                raise ValueError(
                    f"Cannot create new cortical area as neuron count will exceed {max_allowable_neuron_count} threshold"
                )

            # Generate proper cortical ID using FEAGI's standard format
            cortical_id = generate_cortical_id(
                prefix="M" if is_memory else "C", seed=temp_name[:3]
            )

            #  ARCHITECTURE COMPLIANCE: Route through GenomeService instead of
            #  direct ConnectomeManager access
            genome_service = self.core_api_service._genome_service

            #  Create cortical area through proper pipeline: hierarchical
            #  genome -> GenomeService -> connectome
            result = genome_service.create_cortical_area(
                name=cortical_name,
                coordinates={
                    "x": request.coordinates_3d[0],
                    "y": request.coordinates_3d[1],
                    "z": request.coordinates_3d[2],
                },
                dimensions={
                    "width": cortical_dimensions[0],
                    "height": cortical_dimensions[1],
                    "depth": cortical_dimensions[2],
                },
                area_type="memory" if is_memory else "custom",
                parameters={
                    "cortical_group": request.cortical_group,
                    "cortical_sub_group": sub_group_id,
                    "sub_group_id": sub_group_id,  # Add explicit sub_group_id for memory detection
                    "coordinates_2d": request.coordinates_2d,
                    "brain_region_id": parent_region_id,
                    "copy_of": copy_of,
                    "per_voxel_neuron_cnt": neuron_density,
                    "cortical_id": cortical_id,  # Pass the generated ID
                    # Include memory-specific properties if provided
                    **{
                        k: v
                        for k, v in {
                            "init_lifespan": request.init_lifespan,
                            "lifespan_growth_rate": request.lifespan_growth_rate,
                            "longterm_mem_threshold": request.longterm_mem_threshold,
                            "temporal_depth": request.temporal_depth,
                        }.items()
                        if v is not None
                    },
                },
            )

            if not result:
                raise ValueError(
                    "Failed to create cortical area through GenomeService"
                )

            logger.info(
                f"[SUCCESS] Cortical area {cortical_id} created through proper GenomeService pipeline"
            )

            return {"cortical_id": cortical_id}
        except Exception as e:
            raise ValueError(
                f"Error adding custom cortical area: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "DELETE",
        "/cortical_area",
        request_model=CorticalIdRequest,
        response_model=SuccessResponse,
    )
    def delete_cortical_area(
        self, request: CorticalIdRequest
    ) -> SuccessResponse:
        """Delete a single cortical area."""
        try:
            result = self.core_api_service.delete_cortical_area(
                request.cortical_id
            )

            if not result:
                raise ValueError(
                    f"Cortical area {request.cortical_id} not found or could not be deleted"
                )

            return SuccessResponse(
                message=f"Cortical area {request.cortical_id} deleted successfully"
            )
        except Exception as e:
            raise ValueError(f"Error deleting cortical area: {str(e)}") from e

    # ===== Cortical Area Listings =====

    @cortical_area_endpoint(
        "GET",
        "/cortical_area_id_list",
        response_model=CorticalAreaIdListResponse,
    )
    def get_cortical_area_id_list(self) -> CorticalAreaIdListResponse:
        """Get list of cortical area IDs (6-letter strings) present in the
        current genome."""
        try:
            cortical_ids = self.core_api_service.get_cortical_area_id_list()
            return CorticalAreaIdListResponse(cortical_ids=cortical_ids)
        except Exception as e:
            raise ValueError(
                f"Error getting cortical area ID list: {str(e)}"
            ) from e

    @cortical_area_endpoint("GET", "/cortical_area_id_list")
    def get_cortical_area_id_list_legacy(self) -> List[str]:
        """Get list of cortical area IDs (legacy format - returns list directly)."""
        try:
            cortical_ids = self.core_api_service.get_cortical_area_id_list()
            return cortical_ids
        except Exception as e:
            raise ValueError(
                f"Error getting cortical area ID list: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "GET",
        "/cortical_area_index_list",
        response_model=CorticalAreaIndexListResponse,
    )
    def get_cortical_area_index_list(self) -> CorticalAreaIndexListResponse:
        """Get list of cortical area indices (integers) used by the FCL."""
        try:
            indices = self.core_api_service.get_cortical_area_index_list()
            return CorticalAreaIndexListResponse(indices=indices)
        except Exception as e:
            raise ValueError(
                f"Error getting cortical area index list: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "GET",
        "/cortical_area_name_list",
        response_model=CorticalAreaNameListResponse,
    )
    def get_cortical_area_name_list(self) -> CorticalAreaNameListResponse:
        """Get list of cortical area names."""
        try:
            names = self.core_api_service.get_cortical_area_name_list()
            return CorticalAreaNameListResponse(names=names)
        except Exception as e:
            raise ValueError(
                f"Error getting cortical area name list: {str(e)}"
            ) from e

    @cortical_area_endpoint("GET", "/cortical_area_name_list")
    def get_cortical_area_name_list_legacy(self) -> List[str]:
        """Get list of cortical area names (legacy format - returns list directly)."""
        try:
            names = self.core_api_service.get_cortical_area_name_list()
            return names
        except Exception as e:
            raise ValueError(
                f"Error getting cortical area name list: {str(e)}"
            ) from e

    # ===== Cortical Area Location and Geometry =====

    @cortical_area_endpoint(
        "POST",
        "/cortical_name_location",
        request_model=CorticalNameRequest,
        response_model=CorticalLocationResponse,
    )
    def get_cortical_location_by_name(
        self, request: CorticalNameRequest
    ) -> CorticalLocationResponse:
        """Get the 3D location of a cortical area by name."""
        try:
            location = self.core_api_service.get_cortical_location_by_name(
                request.cortical_name
            )
            return CorticalLocationResponse(
                x=location.get(
                    "x", self._get_structural_default("coordinate")
                ),
                y=location.get(
                    "y", self._get_structural_default("coordinate")
                ),
                z=location.get(
                    "z", self._get_structural_default("coordinate")
                ),
            )
        except KeyError:
            raise ValueError(
                f"Cortical area with name '{request.cortical_name}' not found"
            ) from None
        except Exception as e:
            raise ValueError(
                f"Error retrieving cortical location: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "GET", "/cortical_locations_2d", response_model=Dict[str, Any]
    )
    def get_cortical_2d_locations(self) -> Dict[str, Any]:
        """Get 2D locations of cortical areas."""
        try:
            locations = self.core_api_service.get_cortical_2d_locations()
            return locations
        except Exception as e:
            raise ValueError(
                f"Error getting 2D cortical locations: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "GET", "/cortical_area/geometry", response_model=Dict[str, Any]
    )
    def get_cortical_area_geometry(self) -> Dict[str, Any]:
        """Get cortical area geometry information in Godot-compatible format.

        This method returns complete cortical area data in a format that Godot's
        genome loader expects. The structure matches what the Godot bridge requires
        for visualization.

        Returns:
            Dict keyed by cortical_id containing complete cortical area data
        """
        try:
            # Get the list of cortical area IDs first (we know this works)
            cortical_ids = self.core_api_service.get_cortical_area_id_list()

            # If no cortical areas exist, return empty result
            if not cortical_ids:
                return {}

            # Build geometry data for each cortical area
            geometry_data = {}

            for cortical_id in cortical_ids:
                # Get individual cortical area data
                area_data = self.core_api_service.get_cortical_area(
                    cortical_id
                )

                if not area_data:
                    area_data = {
                        "id": cortical_id,
                        "name": cortical_id,
                        "parameters": {},
                        "coordinates": {},
                        "dimensions": {},
                        "type": "unknown",
                    }

                if area_data:
                    # Extract the base data from the API response
                    parameters = area_data.get("parameters", {})
                    coordinates = area_data.get("coordinates", {})
                    dimensions = area_data.get("dimensions", {})

                    # Handle case where dimensions is a tuple
                    if isinstance(dimensions, tuple):
                        dimensions = {
                            "width": dimensions[0],
                            "height": dimensions[1],
                            "depth": dimensions[2],
                        }

                    # Handle case where coordinates is a tuple
                    if isinstance(coordinates, tuple):
                        coordinates = {
                            "x": coordinates[0],
                            "y": coordinates[1],
                            "z": coordinates[2],
                        }

                    #  Build complete cortical area data using template
                    #  defaults for neural properties and structural defaults
                    #  for spatial/organizational properties
                    geometry_data[cortical_id] = {
                        "cortical_id": area_data.get("id", cortical_id),
                        "cortical_name": area_data.get("name", cortical_id),
                        "parent_region_id": parameters.get(
                            "parent_region_id",
                            self._get_structural_default("parent_region_id"),
                        ),
                        "parent_region_title": parameters.get(
                            "parent_region_title",
                            self._get_structural_default(
                                "parent_region_title"
                            ),
                        ),
                        "cortical_group": area_data.get(
                            "type",
                            self._get_structural_default("cortical_group"),
                        ),
                        "cortical_sub_group": parameters.get(
                            "subgroup",
                            self._get_default_value("sub_group_id", ""),
                        ),
                        "cortical_neuron_per_vox_count": parameters.get(
                            "neurons_per_voxel",
                            self._get_default_value("per_voxel_neuron_cnt", 1),
                        ),
                        "visualization": parameters.get(
                            "gd_vis",
                            self._get_default_value("visualization", True),
                        ),
                        "cortical_synaptic_attractivity": parameters.get(
                            "synatt",
                            self._get_default_value(
                                "synapse_attractivity", 100
                            ),
                        ),
                        "coordinates_3d": [
                            coordinates.get(
                                "x", self._get_structural_default("coordinate")
                            ),
                            coordinates.get(
                                "y", self._get_structural_default("coordinate")
                            ),
                            coordinates.get(
                                "z", self._get_structural_default("coordinate")
                            ),
                        ],
                        "coordinates_2d": [
                            parameters.get(
                                "2dcorx",
                                self._get_structural_default("coordinate"),
                            ),
                            parameters.get(
                                "2dcory",
                                self._get_structural_default("coordinate"),
                            ),
                        ],
                        "cortical_dimensions": [
                            dimensions.get(
                                "width",
                                self._get_structural_default("dimension"),
                            ),
                            dimensions.get(
                                "height",
                                self._get_structural_default("dimension"),
                            ),
                            dimensions.get(
                                "depth",
                                self._get_structural_default("dimension"),
                            ),
                        ],
                        "cortical_destinations": parameters.get(
                            "mapping", self._get_structural_default("mapping")
                        ),
                        "neuron_post_synaptic_potential": parameters.get(
                            "pstcr",
                            self._get_default_value("postsynaptic_current", 1),
                        ),
                        "neuron_post_synaptic_potential_max": parameters.get(
                            "pstcrm",
                            self._get_default_value(
                                "postsynaptic_current_max", 99999
                            ),
                        ),
                        "neuron_fire_threshold": parameters.get(
                            "fire_t",
                            self._get_default_value("firing_threshold", 1),
                        ),
                        "neuron_fire_threshold_increment": [
                            parameters.get(
                                "ftincx",
                                self._get_default_value(
                                    "firing_threshold_increment_x", 0
                                ),
                            ),
                            parameters.get(
                                "ftincy",
                                self._get_default_value(
                                    "firing_threshold_increment_y", 0
                                ),
                            ),
                            parameters.get(
                                "ftincz",
                                self._get_default_value(
                                    "firing_threshold_increment_z", 0
                                ),
                            ),
                        ],
                        "neuron_firing_threshold_limit": parameters.get(
                            "fthlim",
                            self._get_default_value(
                                "firing_threshold_limit", 0
                            ),
                        ),
                        "neuron_refractory_period": parameters.get(
                            "refrac",
                            self._get_default_value("refractory_period", 0),
                        ),
                        "neuron_leak_coefficient": parameters.get(
                            "leak_c",
                            self._get_default_value("leak_coefficient", 0),
                        ),
                        "neuron_leak_variability": (
                            parameters.get("leak_v")
                            if parameters.get("leak_v") is not None
                            else self._get_default_value("leak_variability", 0)
                        ),
                        "neuron_consecutive_fire_count": parameters.get(
                            "c_fr_c",
                            self._get_default_value(
                                "consecutive_fire_cnt_max", 0
                            ),
                        ),
                        "neuron_snooze_period": parameters.get(
                            "snooze",
                            self._get_default_value("snooze_length", 0),
                        ),
                        "neuron_degeneracy_coefficient": parameters.get(
                            "de_gen",
                            self._get_default_value("degeneration", 0),
                        ),
                        "neuron_psp_uniform_distribution": parameters.get(
                            "pspuni",
                            self._get_default_value(
                                "psp_uniform_distribution", True
                            ),
                        ),
                        "neuron_mp_charge_accumulation": parameters.get(
                            "mp_acc",
                            self._get_default_value(
                                "mp_charge_accumulation", False
                            ),
                        ),
                        "neuron_mp_driven_psp": parameters.get(
                            "mp_psp",
                            self._get_default_value("mp_driven_psp", False),
                        ),
                        "neuron_longterm_mem_threshold": parameters.get(
                            "mem__t",
                            self._get_default_value(
                                "longterm_mem_threshold", 100
                            ),
                        ),
                        "neuron_lifespan_growth_rate": parameters.get(
                            "mem_gr",
                            self._get_default_value("lifespan_growth_rate", 1),
                        ),
                        "neuron_init_lifespan": parameters.get(
                            "mem_ls",
                            self._get_default_value("init_lifespan", 9),
                        ),
                        "temporal_depth": parameters.get(
                            "temporal_depth",
                            self._get_default_value("temporal_depth", 1),
                        ),
                        "neuron_excitability": parameters.get(
                            "excite",
                            self._get_default_value(
                                "neuron_excitability", 1.0
                            ),
                        ),
                        "transforming": parameters.get(
                            "transforming", False
                        ),  # Runtime state flag - not from templates
                        # Additional fields that might be needed by Godot
                        "dev_count": parameters.get(
                            "dev_count",
                            self._get_default_value("per_voxel_neuron_cnt", 1),
                        ),
                        "cortical_dimensions_per_device": dimensions,
                    }

            return geometry_data
        except Exception as e:
            raise ValueError(
                f"Error getting cortical area geometry: {str(e)}"
            ) from e

    @cortical_area_endpoint("PUT", "/coord_2d", response_model=SuccessResponse)
    def update_2d_coordinates(
        self, coordinates: Dict[str, Any]
    ) -> SuccessResponse:
        """Update 2D coordinates of cortical areas."""
        try:
            success = self.core_api_service.update_2d_coordinates(coordinates)
            if success:
                return SuccessResponse(
                    message="2D coordinates updated successfully"
                )
            else:
                raise ValueError("Failed to update 2D coordinates")
        except Exception as e:
            raise ValueError(f"Error updating 2D coordinates: {str(e)}") from e

    @cortical_area_endpoint("PUT", "/coord_3d", response_model=SuccessResponse)
    def update_3d_coordinates(
        self, coordinates: Dict[str, Any]
    ) -> SuccessResponse:
        """Update 3D coordinates of cortical areas."""
        try:
            success = self.core_api_service.update_3d_coordinates(coordinates)
            if success:
                return SuccessResponse(
                    message="3D coordinates updated successfully"
                )
            else:
                raise ValueError("Failed to update 3D coordinates")
        except Exception as e:
            raise ValueError(f"Error updating 3D coordinates: {str(e)}") from e

    # ===== Cortical Area Types and Options =====

    @cortical_area_endpoint(
        "GET", "/cortical_types", response_model=CorticalAreaTypesResponse
    )
    def get_cortical_area_types(self) -> CorticalAreaTypesResponse:
        """Get available cortical area types."""
        try:
            types = self.core_api_service.get_cortical_area_types()
            return CorticalAreaTypesResponse(types=types)
        except Exception as e:
            raise ValueError(
                f"Error getting cortical area types: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "POST", "/cortical_type_options", request_model=CorticalIdRequest
    )
    def get_cortical_type_options(
        self, request: CorticalIdRequest
    ) -> Dict[str, Any]:
        """Get cortical area type options."""
        try:
            options = self.core_api_service.get_cortical_type_options(
                request.cortical_id
            )
            return options
        except Exception as e:
            raise ValueError(
                f"Error getting cortical type options: {str(e)}"
            ) from e

    # ===== Mapping and Visualization =====

    @cortical_area_endpoint(
        "GET",
        "/cortical_id_name_mapping",
        response_model=CorticalIdNameMappingResponse,
    )
    def get_cortical_id_name_mapping(self) -> CorticalIdNameMappingResponse:
        """Get cortical ID to name mapping table."""
        try:
            mapping = self.core_api_service.get_cortical_id_name_mapping()
            return CorticalIdNameMappingResponse(mapping=mapping)
        except Exception as e:
            raise ValueError(
                f"Error getting cortical ID name mapping: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "GET", "/cortical_map_detailed", response_model=Dict[str, Any]
    )
    def get_detailed_cortical_map(self) -> Dict[str, Any]:
        """Get detailed cortical map."""
        try:
            detailed_map = self.core_api_service.get_detailed_cortical_map()
            return detailed_map
        except Exception as e:
            raise ValueError(
                f"Error getting detailed cortical map: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "GET", "/cortical_visibility", response_model=List[str]
    )
    def get_visualized_cortical_list(self) -> List[str]:
        """Get list of cortical areas currently being visualized."""
        try:
            visualized_list = (
                self.core_api_service.get_visualized_cortical_list()
            )
            return visualized_list
        except Exception as e:
            raise ValueError(
                f"Error getting visualized cortical list: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "PUT", "/suppress_cortical_visibility", response_model=SuccessResponse
    )
    def suppress_cortical_activity_visualization(
        self, cortical_id_list: List[str]
    ) -> SuccessResponse:
        """Suppress cortical activity visualization for specified areas."""
        try:
            success = (
                self.core_api_service.suppress_cortical_activity_visualization(
                    cortical_id_list
                )
            )
            if success:
                return SuccessResponse(
                    message="Cortical activity visualization suppressed"
                )
            else:
                raise ValueError(
                    "Failed to suppress cortical activity visualization"
                )
        except Exception as e:
            raise ValueError(
                f"Error suppressing cortical activity visualization: {str(e)}"
            ) from e

    # ===== Input/Output Processing Units =====

    @cortical_area_endpoint("GET", "/ipu", response_model=List[str])
    def get_current_ipu_list(self) -> List[str]:
        """Get list of current IPU cortical areas."""
        try:
            return self.core_api_service.get_current_ipu_list()
        except Exception as e:
            raise ValueError(f"Error getting IPU list: {str(e)}") from e

    @cortical_area_endpoint("GET", "/ipu")
    def get_current_ipu_list_legacy(self) -> List[str]:
        """Get list of current IPU cortical areas (legacy format - returns list directly)."""
        try:
            return self.core_api_service.get_current_ipu_list()
        except Exception as e:
            raise ValueError(f"Error getting IPU list: {str(e)}") from e

    @cortical_area_endpoint("GET", "/opu", response_model=List[str])
    def get_current_opu_list(self) -> List[str]:
        """Get list of current OPU cortical areas."""
        try:
            return self.core_api_service.get_current_opu_list()
        except Exception as e:
            raise ValueError(f"Error getting OPU list: {str(e)}") from e

    @cortical_area_endpoint("GET", "/opu")
    def get_current_opu_list_legacy(self) -> List[str]:
        """Get list of current OPU cortical areas (legacy format - returns list directly)."""
        try:
            return self.core_api_service.get_current_opu_list()
        except Exception as e:
            raise ValueError(f"Error getting OPU list: {str(e)}") from e

    # ===== Multi-Cortical Operations =====

    @cortical_area_endpoint(
        "POST",
        "/multi/cortical_area_properties",
        request_model=CorticalIdListRequest,
        response_model=List[Dict[str, Any]],
    )
    def get_multiple_cortical_properties(
        self, request: CorticalIdListRequest
    ) -> List[Dict[str, Any]]:
        """Get properties for multiple cortical areas with flexible field name
        support."""
        try:
            cortical_ids = request.cortical_ids

            if not isinstance(cortical_ids, list):
                raise ValueError("cortical_ids must be a list")

            results = []
            for cortical_id in cortical_ids:
                try:
                    # Get properties using the single cortical area method
                    cortical_data = self.core_api_service.get_cortical_area(
                        cortical_id
                    )
                    if cortical_data:
                        results.append(cortical_data)
                except Exception as e:
                    logger.warning(
                        f"Error getting properties for cortical area {cortical_id}: {e}"
                    )
                    # Continue with other cortical areas
                    continue

            return results
        except Exception as e:
            raise ValueError(
                f"Error getting multiple cortical properties: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "PUT", "/multi/cortical_area", response_model=SuccessResponse
    )
    def update_multiple_cortical_properties(
        self, message: Dict[str, Any]
    ) -> SuccessResponse:
        """Update properties for multiple cortical areas."""
        try:
            success = (
                self.core_api_service.update_multiple_cortical_properties(
                    message
                )
            )
            if success:
                return SuccessResponse(
                    message="Multiple cortical area properties updated successfully"
                )
            else:
                raise ValueError(
                    "Failed to update multiple cortical area properties"
                )
        except Exception as e:
            raise ValueError(
                f"Error updating multiple cortical areas: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "DELETE",
        "/multi/cortical_area",
        request_model=CorticalIdListRequest,
        response_model=SuccessResponse,
    )
    def delete_multiple_cortical_areas(
        self, request: CorticalIdListRequest
    ) -> SuccessResponse:
        """Delete multiple cortical areas."""
        try:
            success = self.core_api_service.delete_multiple_cortical_areas(
                request.cortical_ids
            )
            if success:
                return SuccessResponse(
                    message="Multiple cortical areas deleted successfully"
                )
            else:
                raise ValueError("Failed to delete multiple cortical areas")
        except Exception as e:
            raise ValueError(
                f"Error deleting multiple cortical areas: {str(e)}"
            ) from e

    # ===== Neuron Operations =====

    @cortical_area_endpoint(
        "GET",
        "/{cortical_id}/neuron_count",
        response_model=NeuronCountResponse,
    )
    def get_area_neuron_count(self, cortical_id: str) -> NeuronCountResponse:
        """Get neuron count for a specific cortical area."""
        try:
            count = self.core_api_service.get_area_neuron_count(cortical_id)
            return NeuronCountResponse(neuron_count=count)
        except Exception as e:
            raise ValueError(
                f"Error getting neuron count for area {cortical_id}: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "GET",
        "/{cortical_id}/memory_usage",
        response_model=CorticalAreaMemoryUsageResponse,
    )
    def get_area_memory_usage(
        self, cortical_id: str
    ) -> CorticalAreaMemoryUsageResponse:
        """Get detailed memory usage breakdown for a specific cortical area."""
        try:
            memory_usage = (
                self.core_api_service.get_cortical_area_memory_usage(
                    cortical_id
                )
            )
            return memory_usage
        except Exception as e:
            raise ValueError(
                f"Error getting memory usage for area {cortical_id}: {str(e)}"
            ) from e

    @cortical_area_endpoint("PUT", "/reset", response_model=SuccessResponse)
    def reset_cortical_area(self, cortical_list: List[str]) -> SuccessResponse:
        """Reset cortical areas."""
        try:
            success = self.core_api_service.reset_cortical_area(cortical_list)
            if success:
                return SuccessResponse(
                    message="Cortical areas reset successfully"
                )
            else:
                raise ValueError("Failed to reset cortical areas")
        except Exception as e:
            raise ValueError(
                f"Error resetting cortical areas: {str(e)}"
            ) from e

    # ===== Mapping Restrictions =====

    @cortical_area_endpoint(
        "GET",
        "/mapping_restrictions",
        response_model=MappingRestrictionsResponse,
    )
    def get_mapping_restrictions(self) -> MappingRestrictionsResponse:
        """Get all mapping restrictions between cortical area types."""
        try:
            restrictions_data = (
                self.core_api_service.get_mapping_restrictions()
            )
            return MappingRestrictionsResponse(
                restrictions=restrictions_data.get("restrictions", []),
                defaults=restrictions_data.get("defaults", []),
            )
        except Exception as e:
            raise ValueError(
                f"Error getting mapping restrictions: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "POST",
        "/mapping_restrictions",
        request_model=MappingRestrictionsRequest,
        response_model=MappingRestrictionsResponse,
    )
    def get_mapping_restrictions_filtered(
        self, request: MappingRestrictionsRequest
    ) -> MappingRestrictionsResponse:
        """Get mapping restrictions for specific cortical area types."""
        try:
            restrictions_data = self.core_api_service.get_mapping_restrictions(
                source_type=request.source_type,
                destination_type=request.destination_type,
            )

            # Handle both single restriction and full registry responses
            if (
                "restrictions" in restrictions_data
                and "defaults" in restrictions_data
            ):
                # Full registry response
                return MappingRestrictionsResponse(
                    restrictions=restrictions_data["restrictions"],
                    defaults=restrictions_data["defaults"],
                )
            else:
                # Single restriction response - wrap in lists
                restrictions = (
                    [restrictions_data["restriction"]]
                    if restrictions_data.get("restriction")
                    else []
                )
                defaults = (
                    [restrictions_data["default"]]
                    if restrictions_data.get("default")
                    else []
                )
                return MappingRestrictionsResponse(
                    restrictions=restrictions, defaults=defaults
                )
        except Exception as e:
            raise ValueError(
                f"Error getting filtered mapping restrictions: {str(e)}"
            ) from e

    @cortical_area_endpoint(
        "POST",
        "/mapping_restrictions_between_areas",
        request_model=CorticalAreaMappingRestrictionRequest,
        response_model=CorticalAreaMappingRestrictionResponse,
    )
    def get_restrictions_between_cortical_areas(
        self, request: CorticalAreaMappingRestrictionRequest
    ) -> CorticalAreaMappingRestrictionResponse:
        """Get mapping restrictions between two specific cortical areas.

        This endpoint is designed to be compatible with the Godot client's expected interface:
        - get_restrictions_between_2_cortical_areas(source, destination)
        - Returns an object with has_restricted_morphologies() and get_morphologies_restricted_to() methods
        """
        try:
            restriction_data = (
                self.core_api_service.get_restriction_between_cortical_areas(
                    source_cortical_id=request.source_cortical_id,
                    destination_cortical_id=request.destination_cortical_id,
                )
            )

            if not restriction_data:
                #  Return empty restriction for non-existent areas or no
                #  restrictions
                return CorticalAreaMappingRestrictionResponse(
                    source_cortical_id=request.source_cortical_id,
                    destination_cortical_id=request.destination_cortical_id,
                    source_type="UNKNOWN",
                    destination_type="UNKNOWN",
                    restriction=None,
                    default=None,
                    has_restricted_morphologies=False,
                    get_morphologies_restricted_to=[],
                )

            return CorticalAreaMappingRestrictionResponse(**restriction_data)

        except Exception as e:
            raise ValueError(
                f"Error getting restrictions between cortical areas: {str(e)}"
            ) from e

    # ===== DEBUG: Cortical Mapping =====

    @cortical_area_endpoint(
        "GET", "/cortical_idx_mapping", response_model=Dict[str, Any]
    )
    def get_cortical_idx_mapping(self) -> Dict[str, Any]:
        """Get the current cortical_idx to cortical_id mapping for debugging
        corruption issues."""
        try:
            mapping_data = self.core_api_service.get_cortical_idx_mapping()
            return mapping_data
        except Exception as e:
            raise ValueError(
                f"Error getting cortical idx mapping: {str(e)}"
            ) from e


# ===== Factory Function =====


def create_cortical_area_api(
    core_api_service: CoreAPIService,
) -> CorticalAreaAPI:
    """Factory function to create a CorticalAreaAPI instance.

    This function can be used by transport adapters to get a configured
    CorticalAreaAPI instance with the required dependencies.
    """
    return CorticalAreaAPI(core_api_service)
