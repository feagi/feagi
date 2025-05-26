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

"""
FEAGI v1 Cortical Area API - Single Source of Truth

This module contains the ONLY definitions of cortical area API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Dict, Any, Optional, List
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger
from feagi.api.v1.schemas import (
    CorticalAreaPropertiesResponse, CorticalAreaIdListResponse, 
    CorticalAreaIndexListResponse, CorticalAreaNameListResponse,
    CorticalLocationResponse, CorticalIdNameMappingResponse,
    CorticalGeometryResponse, NeuronCountResponse, CorticalIdRequest,
    CorticalNameRequest, CorticalIdListRequest, CoordinateUpdateRequest,
    SuccessResponse, ErrorResponse, CorticalAreaTypesResponse
)
from .decorators import endpoint

logger = setup_logger(__name__)


# Define the convenience decorator for cortical area endpoints
def cortical_area_endpoint(methods, path, request_model=None, response_model=None, description=None):
    """Convenience decorator for cortical area endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module='cortical_area'
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
    
    # ===== Cortical Area Properties =====
    
    @cortical_area_endpoint('POST', '/cortical_area_properties', 
                           request_model=CorticalIdRequest,
                           response_model=CorticalAreaPropertiesResponse)
    def get_cortical_area_properties(self, request: CorticalIdRequest) -> CorticalAreaPropertiesResponse:
        """Get properties of a cortical area."""
        try:
            area_data = self.core_api_service.get_cortical_area(request.cortical_id)
            if not area_data:
                raise KeyError("Cortical area not found")
            
            # Transform modern FEAGI format to expected legacy format
            parameters = area_data.get("parameters", {})
            
            # Extract coordinates
            coordinates = area_data.get("coordinates", {})
            coordinates_3d = [
                coordinates.get("x", 0),
                coordinates.get("y", 0), 
                coordinates.get("z", 0)
            ]
            
            # Extract dimensions
            dimensions = area_data.get("dimensions", {})
            cortical_dimensions = [
                dimensions.get("width", 1),
                dimensions.get("height", 1),
                dimensions.get("depth", 1)
            ]
            
            # Build legacy format response
            legacy_properties = {
                "cortical_id": area_data.get("id", request.cortical_id),
                "cortical_name": area_data.get("name", request.cortical_id),
                "parent_region_id": parameters.get("parent_region_id", "root"),
                "parent_region_title": parameters.get("parent_region_title", "Genome's root brain region"),
                "cortical_group": area_data.get("type", "CUSTOM"),
                "cortical_sub_group": parameters.get("subgroup", ""),
                "cortical_neuron_per_vox_count": parameters.get("neurons_per_voxel", 1),
                "cortical_visibility": parameters.get("gd_vis", True),
                "cortical_synaptic_attractivity": parameters.get("synatt", 100),
                "coordinates_3d": coordinates_3d,
                "coordinates_2d": [
                    parameters.get("2dcorx", 0),
                    parameters.get("2dcory", 0)
                ],
                "cortical_dimensions": cortical_dimensions,
                "cortical_destinations": parameters.get("mapping", {}),
                "neuron_post_synaptic_potential": parameters.get("pstcr", 500),
                "neuron_post_synaptic_potential_max": parameters.get("pstcrm", 35),
                "neuron_fire_threshold": parameters.get("fire_t", 1),
                "neuron_fire_threshold_increment": [
                    parameters.get("ftincx", 0),
                    parameters.get("ftincy", 0),
                    parameters.get("ftincz", 0)
                ],
                "neuron_firing_threshold_limit": parameters.get("fthlim", 0),
                "neuron_refractory_period": parameters.get("refrac", 0),
                "neuron_leak_coefficient": parameters.get("leak_c", 10),
                "neuron_leak_variability": parameters.get("leak_v", 0),
                "neuron_consecutive_fire_count": parameters.get("c_fr_c", 3),
                "neuron_snooze_period": parameters.get("snooze", 0),
                "neuron_degeneracy_coefficient": parameters.get("de_gen", 0),
                "neuron_psp_uniform_distribution": parameters.get("pspuni", False),
                "neuron_mp_charge_accumulation": parameters.get("mp_acc", True),
                "neuron_mp_driven_psp": parameters.get("mp_psp", False),
                "neuron_longterm_mem_threshold": parameters.get("mem__t", 100),
                "neuron_lifespan_growth_rate": parameters.get("mem_gr", 1),
                "neuron_init_lifespan": parameters.get("mem_ls", 9),
                "temporal_depth": parameters.get("temporal_depth", 1),
                "neuron_excitability": parameters.get("excite", 100),
                "transforming": parameters.get("transforming", False)
            }
            
            return CorticalAreaPropertiesResponse(properties=legacy_properties)
        except ValueError:
            raise ValueError("Invalid cortical area ID length")
        except KeyError:
            raise ValueError("Cortical area not found")
        except Exception as e:
            raise ValueError(f"Error retrieving cortical properties: {str(e)}")
    
    @cortical_area_endpoint('PUT', '/cortical_area', response_model=SuccessResponse)
    def update_cortical_area_properties(self, properties: Dict[str, Any]) -> SuccessResponse:
        """Update properties of a cortical area."""
        try:
            # Extract cortical_id and remove it from properties
            cortical_id = properties.get("cortical_id", None)
            if not cortical_id:
                raise ValueError("cortical_id is required")
            
            # Create a copy and remove cortical_id from the properties to update
            properties_to_update = properties.copy()
            properties_to_update.pop("cortical_id", None)
            
            # Use the CoreAPIService to update the properties
            success = self.core_api_service.update_cortical_area_properties(
                cortical_id=cortical_id,
                properties=properties_to_update
            )
            
            if not success:
                raise ValueError("Failed to update cortical area properties")
                
            return SuccessResponse(message=f"Cortical area {cortical_id} update request submitted")
        except Exception as e:
            raise ValueError(f"Error updating cortical area: {str(e)}")
    
    @cortical_area_endpoint('POST', '/cortical_area', response_model=Dict[str, str])
    def add_cortical_area(self, new_cortical_properties: Dict[str, Any]) -> Dict[str, str]:
        """Add a new core cortical area."""
        try:
            # Check if connectome is ready
            connectome = self.core_api_service.get_connectome()
            if not connectome or not connectome.is_connectome_ready():
                raise ValueError("Connectome is not ready!")
            
            cortical_id = new_cortical_properties.get('cortical_id')
            message = {'add_core_cortical_area': new_cortical_properties}
            connectome.add_core_cortical_area(message)
            
            return {'cortical_id': cortical_id}
        except Exception as e:
            raise ValueError(f"Error adding cortical area: {str(e)}")
    
    @cortical_area_endpoint('POST', '/custom_cortical_area', response_model=Dict[str, str])
    def add_custom_cortical_area(self, new_custom_cortical_properties: Dict[str, Any]) -> Dict[str, str]:
        """Add a new custom cortical area."""
        try:
            # Check if connectome is ready
            connectome = self.core_api_service.get_connectome()
            if not connectome or not connectome.is_connectome_ready():
                raise ValueError("Connectome is not ready!")
            
            # Extract properties
            cortical_name = new_custom_cortical_properties.get('cortical_name')
            parent_region_id = new_custom_cortical_properties.get('parent_region_id')
            sub_group_id = new_custom_cortical_properties.get('sub_group_id')
            copy_of = new_custom_cortical_properties.get('copy_of')
            
            # Validate parent region
            if parent_region_id not in connectome.genome["brain_regions"]:
                raise ValueError(f"{parent_region_id} does not exist!")
            
            # Generate cortical ID
            temp_name = cortical_name
            if len(cortical_name) < 3:
                temp_name = cortical_name + "000"
            
            # Determine if memory area
            is_memory = "MEMORY" in sub_group_id
            if is_memory:
                cortical_dimensions = [1, 1, 1]
            else:
                cortical_dimensions = new_custom_cortical_properties.get('cortical_dimensions')
            
            cortical_id = connectome.generate_cortical_id(temp_name[:3], is_memory=is_memory)
            
            # Calculate neuron count and validate limits
            neuron_density = 1
            if copy_of:
                neuron_density = connectome.genome["blueprint"][copy_of]["per_voxel_neuron_cnt"]
            
            neuron_count = neuron_density * cortical_dimensions[0] * cortical_dimensions[1] * cortical_dimensions[2]
            max_allowable_neuron_count = int(connectome.parameters["Limits"]["max_neuron_count"])
            
            if neuron_count + connectome.brain_stats["neuron_count"] > max_allowable_neuron_count:
                raise ValueError(f"Cannot create new cortical area as neuron count will exceed {max_allowable_neuron_count} threshold")
            
            # Prepare message
            message_data = new_custom_cortical_properties.copy()
            message_data.update({
                "is_memory": is_memory,
                "cortical_dimensions": cortical_dimensions,
                "copy_of": copy_of,
                "cortical_id": cortical_id
            })
            
            message = {'add_custom_cortical_area': message_data}
            connectome.add_custom_cortical_area(message)
            
            return {'cortical_id': cortical_id}
        except Exception as e:
            raise ValueError(f"Error adding custom cortical area: {str(e)}")
    
    @cortical_area_endpoint('DELETE', '/cortical_area', 
                           request_model=CorticalIdRequest,
                           response_model=SuccessResponse)
    def delete_cortical_area(self, request: CorticalIdRequest) -> SuccessResponse:
        """Delete a single cortical area."""
        try:
            result = self.core_api_service.delete_cortical_area(request.cortical_id)
            
            if not result:
                raise ValueError(f"Cortical area {request.cortical_id} not found or could not be deleted")
            
            return SuccessResponse(message=f"Cortical area {request.cortical_id} deleted successfully")
        except Exception as e:
            raise ValueError(f"Error deleting cortical area: {str(e)}")
    
    # ===== Cortical Area Listings =====
    
    @cortical_area_endpoint('GET', '/cortical_area_id_list', response_model=CorticalAreaIdListResponse)
    def get_cortical_area_id_list(self) -> CorticalAreaIdListResponse:
        """Get list of cortical area IDs (6-letter strings) present in the current genome."""
        try:
            cortical_ids = self.core_api_service.get_cortical_area_id_list()
            return CorticalAreaIdListResponse(cortical_ids=cortical_ids)
        except Exception as e:
            raise ValueError(f"Error getting cortical area ID list: {str(e)}")

    @cortical_area_endpoint('GET', '/cortical_area_id_list')
    def get_cortical_area_id_list_legacy(self) -> List[str]:
        """Get list of cortical area IDs (legacy format - returns list directly)."""
        try:
            cortical_ids = self.core_api_service.get_cortical_area_id_list()
            return cortical_ids
        except Exception as e:
            raise ValueError(f"Error getting cortical area ID list: {str(e)}")
    
    @cortical_area_endpoint('GET', '/cortical_area_index_list', response_model=CorticalAreaIndexListResponse)
    def get_cortical_area_index_list(self) -> CorticalAreaIndexListResponse:
        """Get list of cortical area indices (integers) used by the FCL."""
        try:
            indices = self.core_api_service.get_cortical_area_index_list()
            return CorticalAreaIndexListResponse(indices=indices)
        except Exception as e:
            raise ValueError(f"Error getting cortical area index list: {str(e)}")
    
    @cortical_area_endpoint('GET', '/cortical_area_name_list', response_model=CorticalAreaNameListResponse)
    def get_cortical_area_name_list(self) -> CorticalAreaNameListResponse:
        """Get list of cortical area names."""
        try:
            names = self.core_api_service.get_cortical_area_name_list()
            return CorticalAreaNameListResponse(names=names)
        except Exception as e:
            raise ValueError(f"Error getting cortical area name list: {str(e)}")

    @cortical_area_endpoint('GET', '/cortical_area_name_list')
    def get_cortical_area_name_list_legacy(self) -> List[str]:
        """Get list of cortical area names (legacy format - returns list directly)."""
        try:
            names = self.core_api_service.get_cortical_area_name_list()
            return names
        except Exception as e:
            raise ValueError(f"Error getting cortical area name list: {str(e)}")
    
    # ===== Cortical Area Location and Geometry =====
    
    @cortical_area_endpoint('POST', '/cortical_name_location',
                           request_model=CorticalNameRequest,
                           response_model=CorticalLocationResponse)
    def get_cortical_location_by_name(self, request: CorticalNameRequest) -> CorticalLocationResponse:
        """Get the 3D location of a cortical area by name."""
        try:
            location = self.core_api_service.get_cortical_location_by_name(request.cortical_name)
            return CorticalLocationResponse(
                x=location.get('x', 0),
                y=location.get('y', 0),
                z=location.get('z', 0)
            )
        except KeyError:
            raise ValueError(f"Cortical area with name '{request.cortical_name}' not found")
        except Exception as e:
            raise ValueError(f"Error retrieving cortical location: {str(e)}")
    
    @cortical_area_endpoint('GET', '/cortical_locations_2d', response_model=Dict[str, Any])
    def get_cortical_2d_locations(self) -> Dict[str, Any]:
        """Get 2D locations of cortical areas."""
        try:
            locations = self.core_api_service.get_cortical_2d_locations()
            return locations
        except Exception as e:
            raise ValueError(f"Error getting 2D cortical locations: {str(e)}")
    
    @cortical_area_endpoint('GET', '/cortical_area/geometry', response_model=Dict[str, Any])
    def get_cortical_area_geometry(self) -> Dict[str, Any]:
        """Get cortical area geometry information in Godot-compatible format.
        
        This method returns complete cortical area data in a format that Godot's
        genome loader expects. The structure matches what the Godot bridge requires
        for visualization.
        
        Returns:
            Dict keyed by cortical_id containing complete cortical area data
        """
        try:
            # Get all cortical area IDs
            cortical_ids = self.core_api_service.get_cortical_area_id_list()
            geometry_data = {}
            
            for cortical_id in cortical_ids:
                # Get individual cortical area data
                area_data = self.core_api_service.get_cortical_area(cortical_id)
                
                if area_data:
                    # Extract the base data from the API response
                    parameters = area_data.get("parameters", {})
                    coordinates = area_data.get("coordinates", {})
                    dimensions = area_data.get("dimensions", {})
                    
                    # Build complete cortical area data
                    geometry_data[cortical_id] = {
                        "cortical_id": area_data.get("id", cortical_id),
                        "cortical_name": area_data.get("name", cortical_id),
                        "parent_region_id": parameters.get("parent_region_id", "root"),
                        "parent_region_title": parameters.get("parent_region_title", "Genome's root brain region"),
                        "cortical_group": area_data.get("type", "CUSTOM"),
                        "cortical_sub_group": parameters.get("subgroup", ""),
                        "cortical_neuron_per_vox_count": parameters.get("neurons_per_voxel", 1),
                        "visualization": parameters.get("gd_vis", True),
                        "cortical_synaptic_attractivity": parameters.get("synatt", 100),
                        "coordinates_3d": [
                            coordinates.get("x", 0),
                            coordinates.get("y", 0), 
                            coordinates.get("z", 0)
                        ],
                        "coordinates_2d": [
                            parameters.get("2dcorx", 0),
                            parameters.get("2dcory", 0)
                        ],
                        "cortical_dimensions": [
                            dimensions.get("width", 1),
                            dimensions.get("height", 1),
                            dimensions.get("depth", 1)
                        ],
                        "cortical_destinations": parameters.get("mapping", {}),
                        "neuron_post_synaptic_potential": parameters.get("pstcr", 500),
                        "neuron_post_synaptic_potential_max": parameters.get("pstcrm", 35),
                        "neuron_fire_threshold": parameters.get("fire_t", 1),
                        "neuron_fire_threshold_increment": [
                            parameters.get("ftincx", 0),
                            parameters.get("ftincy", 0),
                            parameters.get("ftincz", 0)
                        ],
                        "neuron_firing_threshold_limit": parameters.get("fthlim", 0),
                        "neuron_refractory_period": parameters.get("refrac", 0),
                        "neuron_leak_coefficient": parameters.get("leak_c", 10),
                        "neuron_leak_variability": parameters.get("leak_v", 0),
                        "neuron_consecutive_fire_count": parameters.get("c_fr_c", 3),
                        "neuron_snooze_period": parameters.get("snooze", 0),
                        "neuron_degeneracy_coefficient": parameters.get("de_gen", 0),
                        "neuron_psp_uniform_distribution": parameters.get("pspuni", False),
                        "neuron_mp_charge_accumulation": parameters.get("mp_acc", True),
                        "neuron_mp_driven_psp": parameters.get("mp_psp", False),
                        "neuron_longterm_mem_threshold": parameters.get("mem__t", 100),
                        "neuron_lifespan_growth_rate": parameters.get("mem_gr", 1),
                        "neuron_init_lifespan": parameters.get("mem_ls", 9),
                        "temporal_depth": parameters.get("temporal_depth", 1),
                        "neuron_excitability": parameters.get("excite", 100),
                        "transforming": parameters.get("transforming", False),
                        # Additional fields that might be needed by Godot
                        "dev_count": parameters.get("dev_count", 1),
                        "cortical_dimensions_per_device": parameters.get("cortical_dimensions_per_device", dimensions)
                    }
            
            return geometry_data
        except Exception as e:
            raise ValueError(f"Error getting cortical area geometry: {str(e)}")
    
    @cortical_area_endpoint('PUT', '/coord_2d', response_model=SuccessResponse)
    def update_2d_coordinates(self, coordinates: Dict[str, Any]) -> SuccessResponse:
        """Update 2D coordinates of cortical areas."""
        try:
            success = self.core_api_service.update_2d_coordinates(coordinates)
            if success:
                return SuccessResponse(message="2D coordinates updated successfully")
            else:
                raise ValueError("Failed to update 2D coordinates")
        except Exception as e:
            raise ValueError(f"Error updating 2D coordinates: {str(e)}")
    
    @cortical_area_endpoint('PUT', '/coord_3d', response_model=SuccessResponse)
    def update_3d_coordinates(self, coordinates: Dict[str, Any]) -> SuccessResponse:
        """Update 3D coordinates of cortical areas."""
        try:
            success = self.core_api_service.update_3d_coordinates(coordinates)
            if success:
                return SuccessResponse(message="3D coordinates updated successfully")
            else:
                raise ValueError("Failed to update 3D coordinates")
        except Exception as e:
            raise ValueError(f"Error updating 3D coordinates: {str(e)}")
    
    # ===== Cortical Area Types and Options =====
    
    @cortical_area_endpoint('GET', '/cortical_types', response_model=CorticalAreaTypesResponse)
    def get_cortical_area_types(self) -> CorticalAreaTypesResponse:
        """Get available cortical area types."""
        try:
            types = self.core_api_service.get_cortical_area_types()
            return CorticalAreaTypesResponse(types=types)
        except Exception as e:
            raise ValueError(f"Error getting cortical area types: {str(e)}")
    
    @cortical_area_endpoint('POST', '/cortical_type_options',
                           request_model=CorticalIdRequest)
    def get_cortical_type_options(self, request: CorticalIdRequest) -> Dict[str, Any]:
        """Get cortical area type options."""
        try:
            options = self.core_api_service.get_cortical_type_options(request.cortical_id)
            return options
        except Exception as e:
            raise ValueError(f"Error getting cortical type options: {str(e)}")
    
    # ===== Mapping and Visualization =====
    
    @cortical_area_endpoint('GET', '/cortical_id_name_mapping', response_model=CorticalIdNameMappingResponse)
    def get_cortical_id_name_mapping(self) -> CorticalIdNameMappingResponse:
        """Get cortical ID to name mapping table."""
        try:
            mapping = self.core_api_service.get_cortical_id_name_mapping()
            return CorticalIdNameMappingResponse(mapping=mapping)
        except Exception as e:
            raise ValueError(f"Error getting cortical ID name mapping: {str(e)}")
    
    @cortical_area_endpoint('GET', '/cortical_map_detailed', response_model=Dict[str, Any])
    def get_detailed_cortical_map(self) -> Dict[str, Any]:
        """Get detailed cortical map."""
        try:
            detailed_map = self.core_api_service.get_detailed_cortical_map()
            return detailed_map
        except Exception as e:
            raise ValueError(f"Error getting detailed cortical map: {str(e)}")
    
    @cortical_area_endpoint('GET', '/cortical_visibility', response_model=List[str])
    def get_visualized_cortical_list(self) -> List[str]:
        """Get list of cortical areas currently being visualized."""
        try:
            visualized_list = self.core_api_service.get_visualized_cortical_list()
            return visualized_list
        except Exception as e:
            raise ValueError(f"Error getting visualized cortical list: {str(e)}")
    
    @cortical_area_endpoint('PUT', '/suppress_cortical_visibility', response_model=SuccessResponse)
    def suppress_cortical_activity_visualization(self, cortical_id_list: List[str]) -> SuccessResponse:
        """Suppress cortical activity visualization for specified areas."""
        try:
            success = self.core_api_service.suppress_cortical_activity_visualization(cortical_id_list)
            if success:
                return SuccessResponse(message="Cortical activity visualization suppressed")
            else:
                raise ValueError("Failed to suppress cortical activity visualization")
        except Exception as e:
            raise ValueError(f"Error suppressing cortical activity visualization: {str(e)}")
    
    # ===== Input/Output Processing Units =====
    
    @cortical_area_endpoint('GET', '/ipu', response_model=List[str])
    def get_current_ipu_list(self) -> List[str]:
        """Get list of current IPU cortical areas."""
        try:
            return self.core_api_service.get_current_ipu_list()
        except Exception as e:
            raise ValueError(f"Error getting IPU list: {str(e)}")

    @cortical_area_endpoint('GET', '/ipu')
    def get_current_ipu_list_legacy(self) -> List[str]:
        """Get list of current IPU cortical areas (legacy format - returns list directly)."""
        try:
            return self.core_api_service.get_current_ipu_list()
        except Exception as e:
            raise ValueError(f"Error getting IPU list: {str(e)}")

    @cortical_area_endpoint('GET', '/opu', response_model=List[str])
    def get_current_opu_list(self) -> List[str]:
        """Get list of current OPU cortical areas."""
        try:
            return self.core_api_service.get_current_opu_list()
        except Exception as e:
            raise ValueError(f"Error getting OPU list: {str(e)}")

    @cortical_area_endpoint('GET', '/opu')
    def get_current_opu_list_legacy(self) -> List[str]:
        """Get list of current OPU cortical areas (legacy format - returns list directly)."""
        try:
            return self.core_api_service.get_current_opu_list()
        except Exception as e:
            raise ValueError(f"Error getting OPU list: {str(e)}")
    
    # ===== Multi-Cortical Operations =====
    
    @cortical_area_endpoint('POST', '/multi/cortical_area_properties',
                           response_model=List[Dict[str, Any]])
    def get_multiple_cortical_properties(self, request_data: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Get properties for multiple cortical areas with flexible field name support."""
        try:
            # Handle both 'cortical_ids' (new) and 'cortical_id_list' (legacy) field names
            cortical_ids = []
            if 'cortical_ids' in request_data:
                cortical_ids = request_data['cortical_ids']
            elif 'cortical_id_list' in request_data:
                cortical_ids = request_data['cortical_id_list']
            else:
                raise ValueError("Missing required field: 'cortical_ids' or 'cortical_id_list'")
            
            if not isinstance(cortical_ids, list):
                raise ValueError("cortical_ids must be a list")
                
            results = []
            for cortical_id in cortical_ids:
                try:
                    # Get properties using the single cortical area method
                    cortical_data = self.core_api_service.get_cortical_area(cortical_id)
                    if cortical_data:
                        results.append(cortical_data)
                except Exception as e:
                    logger.warning(f"Error getting properties for cortical area {cortical_id}: {e}")
                    # Continue with other cortical areas
                    continue
            
            return results
        except Exception as e:
            raise ValueError(f"Error getting multiple cortical properties: {str(e)}")
    
    @cortical_area_endpoint('PUT', '/multi/cortical_area', response_model=SuccessResponse)
    def update_multiple_cortical_properties(self, message: Dict[str, Any]) -> SuccessResponse:
        """Update properties for multiple cortical areas."""
        try:
            success = self.core_api_service.update_multiple_cortical_properties(message)
            if success:
                return SuccessResponse(message="Multiple cortical area properties updated successfully")
            else:
                raise ValueError("Failed to update multiple cortical area properties")
        except Exception as e:
            raise ValueError(f"Error updating multiple cortical areas: {str(e)}")
    
    @cortical_area_endpoint('DELETE', '/multi/cortical_area',
                           request_model=CorticalIdListRequest,
                           response_model=SuccessResponse)
    def delete_multiple_cortical_areas(self, request: CorticalIdListRequest) -> SuccessResponse:
        """Delete multiple cortical areas."""
        try:
            success = self.core_api_service.delete_multiple_cortical_areas(request.cortical_ids)
            if success:
                return SuccessResponse(message="Multiple cortical areas deleted successfully")
            else:
                raise ValueError("Failed to delete multiple cortical areas")
        except Exception as e:
            raise ValueError(f"Error deleting multiple cortical areas: {str(e)}")
    
    # ===== Neuron Operations =====
    
    @cortical_area_endpoint('GET', '/neuron_count', response_model=NeuronCountResponse)
    def get_area_neuron_count(self, cortical_id: str) -> NeuronCountResponse:
        """Get neuron count for a cortical area."""
        try:
            count = self.core_api_service.get_area_neuron_count(cortical_id)
            return NeuronCountResponse(neuron_count=count)
        except Exception as e:
            raise ValueError(f"Error getting neuron count: {str(e)}")
    
    @cortical_area_endpoint('PUT', '/reset', response_model=SuccessResponse)
    def reset_cortical_area(self, cortical_list: List[str]) -> SuccessResponse:
        """Reset cortical areas."""
        try:
            success = self.core_api_service.reset_cortical_area(cortical_list)
            if success:
                return SuccessResponse(message="Cortical areas reset successfully")
            else:
                raise ValueError("Failed to reset cortical areas")
        except Exception as e:
            raise ValueError(f"Error resetting cortical areas: {str(e)}")


# ===== Factory Function =====

def create_cortical_area_api(core_api_service: CoreAPIService) -> CorticalAreaAPI:
    """
    Factory function to create a CorticalAreaAPI instance.
    
    This function can be used by transport adapters to get a configured
    CorticalAreaAPI instance with the required dependencies.
    """
    return CorticalAreaAPI(core_api_service) 