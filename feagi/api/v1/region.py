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

from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

from .decorators import endpoint
from .schemas import (
    RegionMemberRelocationRequest,
    SuccessResponse,
    CloneBrainRegionRequest,
    CloneBrainRegionResponse,
)

logger = setup_logger(__name__)

"""
FEAGI v1 Region API - Single Source of Truth

This module contains the ONLY definitions of brain region API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""


# ===== Region-specific Schemas =====


class RegionAssociation(BaseModel):
    """Request model for changing region associations."""

    id: str
    new_region_id: str


class NewRegionProperties(BaseModel):
    """Request model for creating a new region."""

    title: str = Field(description="Region title/name")
    parent_region_id: str = Field(description="Parent region ID (use 'root' for top-level)")
    coordinates_2d: List[int] = Field(description="2D coordinates [x, y]")
    coordinates_3d: List[float] = Field(description="3D coordinates [x, y, z]")
    areas: Optional[List[str]] = Field(default=[], description="List of cortical area IDs in this region")
    regions: Optional[List[str]] = Field(default=[], description="List of child region IDs")
    region_id: Optional[str] = Field(default=None, description="Optional region ID (auto-generated if not provided)")


class UpdateRegionProperties(BaseModel):
    """Request model for updating region properties."""

    region_id: str
    region_title: Optional[str] = None
    coordinate_2d: Optional[List[int]] = None
    coordinate_3d: Optional[List[int]] = None

    class Config:
        extra = "forbid"  # Reject unknown fields (e.g., coordinates_3d)


class RegionIdRequest(BaseModel):
    """Request model for operations requiring region ID."""

    id: str


# Define the convenience decorator for region endpoints
def region_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    """Convenience decorator for region endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="region",
    )


class RegionAPI:
    """
    Region API - Single Source of Truth for ALL Transports

    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.

    This ensures identical behavior across all transports with zero duplication.
    """

    def __init__(self, core_api_service: CoreAPIService):
        """Initialize with core API service dependency."""
        self.core_api_service = core_api_service

    # ===== Legacy Region Management Endpoints =====

    @region_endpoint(
        "POST",
        "/region",
        request_model=NewRegionProperties,
        response_model=Dict[str, Any],
    )
    def create_brain_region(
        self, region_data: NewRegionProperties
    ) -> Dict[str, Any]:
        """Create a new brain region."""
        try:
            # Check if connectome is ready
            connectome = self.core_api_service.get_connectome()
            if not connectome:
                raise ValueError("Connectome is not ready!")

            # For new ConnectomeManager, validate parent exists if specified
            if (
                region_data.parent_region_id
                and hasattr(connectome, "brain_regions")
                and connectome.brain_regions
            ):
                if (
                    region_data.parent_region_id
                    not in connectome.brain_regions
                ):
                    raise ValueError(
                        f"{region_data.parent_region_id} is not a valid region id"
                    )

            # Generate region_id if not provided
            region_id = region_data.region_id
            if not region_id:
                import uuid
                region_id = f"region_{uuid.uuid4().hex[:8]}"
            
            # Convert coordinates to the expected format
            coordinates = None
            if region_data.coordinates_3d:
                coordinates = {
                    "x": int(region_data.coordinates_3d[0]),
                    "y": int(region_data.coordinates_3d[1]),
                    "z": int(region_data.coordinates_3d[2])
                }
            
            # Prepare additional parameters
            parameters = {
                "coordinates_2d": region_data.coordinates_2d,
                "areas": region_data.areas,
                "regions": region_data.regions
            }
            
            # Create the region using core API service
            success = self.core_api_service.create_brain_region(
                region_id=region_id,
                region_name=region_data.title,
                parent_region_id=region_data.parent_region_id,
                coordinates=coordinates,
                parameters=parameters
            )
            
            if not success:
                raise ValueError("Failed to create brain region")
            
            # Get complete brain region data structure using proper service method
            try:
                logger.debug(f"Attempting to retrieve complete data for region: {region_id}")
                
                # Use the core API service to get all regions with proper normalization
                # This ensures we get the data after all synchronization is complete
                all_regions = self.core_api_service.get_brain_regions()
                logger.debug(f"Retrieved {len(all_regions)} total regions from core service")
                logger.debug(f"Available region IDs: {[r.get('region_id', 'NO_ID') for r in all_regions]}")
                
                # Log a sample of the data structure
                if all_regions:
                    sample_region = all_regions[0]
                    logger.debug(f"Sample region structure: {sample_region}")
                    logger.debug(f"Sample region keys: {list(sample_region.keys()) if isinstance(sample_region, dict) else 'NOT_DICT'}")
                
                # Find our newly created region
                for region in all_regions:
                    if region.get("region_id") == region_id:
                        logger.debug(f"Found complete region data for {region_id}")
                        logger.debug(f"Complete region structure: {region}")
                        
                        # Verify the structure has all expected fields
                        required_fields = ['region_id', 'title', 'areas', 'inputs', 'outputs']
                        missing_fields = [field for field in required_fields if field not in region]
                        if missing_fields:
                            logger.warning(f"Region {region_id} missing fields: {missing_fields}")
                        
                        logger.info(f"🎯 [REGION-RESPONSE] Returning complete data for {region_id}: {region}")
                        return region
                
                # If not found in normalized list, try direct access as fallback
                logger.warning(f"Region {region_id} not found in normalized list, trying direct access")
                logger.warning(f"Available regions were: {[r.get('region_id', 'NO_ID') for r in all_regions]}")
                from feagi.core.state_manager import FeagiStateManager
                state_manager = FeagiStateManager.instance()
                
                if hasattr(state_manager, 'genome') and state_manager.genome:
                    brain_regions = state_manager.genome.get("brain_regions", {})
                    blueprint = state_manager.genome.get("blueprint", {})
                    
                    if region_id in brain_regions:
                        region_info = brain_regions[region_id]
                        logger.debug(f"Found region in genome: {region_info}")
                        logger.debug(f"Blueprint available: {bool(blueprint)}")
                        logger.debug(f"Areas to process: {region_info.get('areas', region_data.areas or [])}")
                        
                        # Normalize the region structure with inputs/outputs
                        complete_region_data = {
                            "region_id": region_id,
                            "title": region_info.get("title", region_info.get("region_name", f"Region {region_id}")),
                            "description": region_info.get("description", ""),
                            "parent_region_id": region_info.get("parent_region_id"),
                            "coordinate_2d": region_info.get("coordinate_2d", region_data.coordinates_2d or [0, 0]),
                            "coordinate_3d": region_info.get("coordinate_3d", region_data.coordinates_3d or [0, 0, 0]),
                            "areas": region_info.get("areas", region_data.areas or []),
                            "regions": region_info.get("regions", region_data.regions or []),
                            "inputs": region_info.get("inputs", []),
                            "outputs": region_info.get("outputs", []),
                            "signature": region_info.get("signature", "")
                        }
                        
                        # Auto-assign I/O based on cortical area types if areas exist
                        areas = complete_region_data["areas"]
                        if areas and blueprint:
                            auto_inputs = []
                            auto_outputs = []
                            
                            for area_id in areas:
                                area_props = blueprint.get(area_id, {})
                                area_group = area_props.get("group", area_props.get("cortical_group", "")).upper()
                                
                                # IPU areas become inputs
                                if area_group == "IPU":
                                    auto_inputs.append(area_id)
                                    logger.debug(f"Added {area_id} to inputs (IPU)")
                                # OPU areas become outputs
                                elif area_group == "OPU":
                                    auto_outputs.append(area_id)
                                    logger.debug(f"Added {area_id} to outputs (OPU)")
                            
                            # Merge with existing I/O (avoid duplicates)
                            existing_inputs = complete_region_data["inputs"]
                            existing_outputs = complete_region_data["outputs"]
                            complete_region_data["inputs"] = list(set(existing_inputs + auto_inputs))
                            complete_region_data["outputs"] = list(set(existing_outputs + auto_outputs))
                            
                            logger.debug(f"Final I/O assignment - inputs: {complete_region_data['inputs']}, outputs: {complete_region_data['outputs']}")
                        logger.debug(f"Complete data structure being returned: {complete_region_data}")
                        
                        return complete_region_data
                
                # Final fallback - return structure with request data
                logger.warning(f"Could not retrieve region data for {region_id} from any source, using request data")
                logger.debug(f"Fallback data will include areas: {region_data.areas}")
                fallback_response = {
                    "region_id": region_id,
                    "title": region_data.title,
                    "description": "",
                    "parent_region_id": region_data.parent_region_id,
                    "coordinate_2d": region_data.coordinates_2d or [0, 0],
                    "coordinate_3d": region_data.coordinates_3d or [0, 0, 0],
                    "areas": region_data.areas or [],
                    "regions": region_data.regions or [],
                    "inputs": [],  # Will be populated by I/O logic if areas exist
                    "outputs": [],  # Will be populated by I/O logic if areas exist 
                    "signature": ""
                }
                logger.debug(f"Final fallback response structure: {fallback_response}")
                return fallback_response
                
            except Exception as region_fetch_error:
                logger.error(f"Error fetching complete region data for {region_id}: {region_fetch_error}")
                # Return structure with request data and basic I/O assignment
                fallback_data = {
                    "region_id": region_id,
                    "title": region_data.title,
                    "description": "",
                    "parent_region_id": region_data.parent_region_id,
                    "coordinate_2d": region_data.coordinates_2d or [0, 0],
                    "coordinate_3d": region_data.coordinates_3d or [0, 0, 0],
                    "areas": region_data.areas or [],
                    "regions": region_data.regions or [],
                    "inputs": [],
                    "outputs": [],
                    "signature": ""
                }
                
                # Try basic I/O assignment even in error case
                try:
                    from feagi.core.state_manager import FeagiStateManager
                    state_manager = FeagiStateManager.instance()
                    if hasattr(state_manager, 'genome') and state_manager.genome:
                        blueprint = state_manager.genome.get("blueprint", {})
                        areas = fallback_data["areas"]
                        if areas and blueprint:
                            auto_inputs = []
                            auto_outputs = []
                            for area_id in areas:
                                area_props = blueprint.get(area_id, {})
                                area_group = area_props.get("group", area_props.get("cortical_group", "")).upper()
                                if area_group == "IPU":
                                    auto_inputs.append(area_id)
                                elif area_group == "OPU":
                                    auto_outputs.append(area_id)
                            fallback_data["inputs"] = auto_inputs
                            fallback_data["outputs"] = auto_outputs
                except Exception as io_error:
                    logger.warning(f"Could not assign I/O in fallback: {io_error}")
                
                return fallback_data
                
        except Exception as e:
            logger.error(f"Error creating brain region: {e}")
            raise ValueError(f"Failed to create brain region: {str(e)}")

    @region_endpoint(
        "PUT",
        "/region",
        request_model=UpdateRegionProperties,
        response_model=Dict[str, Any],
    )
    def update_region_properties(
        self, region_data: UpdateRegionProperties
    ) -> Dict[str, Any]:
        """Update brain region properties (title, coordinates)."""
        try:
            region_dict = region_data.dict(exclude_unset=True)

            # Validate root region restrictions
            unacceptable_root_fields = [
                "parent_region_id",
                "coordinate_2d",
                "coordinate_3d",
            ]
            if region_data.region_id == "root":
                for field in unacceptable_root_fields:
                    if field in region_dict:
                        raise ValueError(
                            f"{field} cannot be modified for root region"
                        )

            # Prepare args for CoreAPIService.update_brain_region
            region_id = region_data.region_id
            region_name = region_dict.get("region_title")
            coordinates = None
            # Strict coordinates (no legacy aliases): only coordinate_3d accepted
            if region_data.coordinate_3d is not None:
                c3d = region_data.coordinate_3d
                coordinates = {"x": int(c3d[0]), "y": int(c3d[1]), "z": int(c3d[2])}

            # Optional 2D coordinate update passed through parameters (strict key)
            parameters = None
            if region_data.coordinate_2d is not None:
                parameters = {"coordinates_2d": [int(region_data.coordinate_2d[0]), int(region_data.coordinate_2d[1])]}            

            # Call correct service method
            success = self.core_api_service.update_brain_region(
                region_id=region_id,
                region_name=region_name,
                coordinates=coordinates,
                parameters=parameters,
            )
            if not success:
                raise ValueError("Failed to update brain region properties")

            # Get complete brain region data structure using proper service method
            try:
                logger.debug(f"Attempting to retrieve updated data for region: {region_id}")
                
                # Use the core API service to get all regions with proper normalization
                all_regions = self.core_api_service.get_brain_regions()
                
                # Find our updated region
                for region in all_regions:
                    if region.get("region_id") == region_id:
                        logger.debug(f"Found updated region data for {region_id}")
                        return region
                
                logger.warning(f"Updated region {region_id} not found, using fallback structure")
                return {
                    "region_id": region_id,
                    "title": region_name or f"Region {region_id}",
                    "description": "",
                    "parent_region_id": None,
                    "coordinate_2d": region_data.coordinate_2d or [0, 0],
                    "coordinate_3d": region_data.coordinate_3d or [0, 0, 0],
                    "areas": [],
                    "regions": [],
                    "inputs": [],
                    "outputs": [],
                    "signature": ""
                }
                
            except Exception as region_fetch_error:
                logger.error(f"Error fetching updated region data for {region_id}: {region_fetch_error}")
                return {
                    "region_id": region_id,
                    "title": region_name or f"Region {region_id}",
                    "description": "",
                    "parent_region_id": None,
                    "coordinate_2d": region_data.coordinate_2d or [0, 0],
                    "coordinate_3d": region_data.coordinate_3d or [0, 0, 0],
                    "areas": [],
                    "regions": [],
                    "inputs": [],
                    "outputs": [],
                    "signature": ""
                }
                
        except Exception as e:
            logger.error(f"Error updating region properties: {e}")
            raise ValueError(f"Failed to update region properties: {str(e)}")

    @region_endpoint("GET", "/region/{region_id}")
    def view_region_properties(self, region_id: str) -> Dict[str, Any]:
        """Get brain region properties."""
        try:
            connectome = self.core_api_service.get_connectome()
            if not connectome:
                raise ValueError("Connectome is not ready!")

            # Use new ConnectomeManager structure
            if (
                hasattr(connectome, "brain_regions")
                and connectome.brain_regions
                and region_id in connectome.brain_regions
            ):
                return connectome.brain_regions[region_id]
            else:
                raise ValueError(f"{region_id} is not a valid region id")
        except Exception as e:
            logger.error(f"Error getting region properties: {e}")
            raise ValueError(f"Failed to get region properties: {str(e)}")

    @region_endpoint(
        "DELETE",
        "/region",
        request_model=RegionIdRequest,
        response_model=SuccessResponse,
    )
    def delete_region(
        self, region_id_data: RegionIdRequest
    ) -> SuccessResponse:
        """Delete a brain region (moves children to parent)."""
        try:
            region_id = region_id_data.id

            if region_id == "root":
                raise ValueError("Root region cannot be deleted")

            connectome = self.core_api_service.get_connectome()
            if not connectome:
                raise ValueError("Connectome is not ready!")

            # Use new ConnectomeManager structure
            if (
                hasattr(connectome, "brain_regions")
                and connectome.brain_regions
            ):
                if region_id not in connectome.brain_regions:
                    raise ValueError(f"{region_id} is not a valid region id")

            success = self.core_api_service.delete_brain_region(
                region_id, preserve_children=True
            )
            if not success:
                raise ValueError("Failed to delete brain region")

            return SuccessResponse(message="Brain region deleted successfully")
        except Exception as e:
            logger.error(f"Error deleting region: {e}")
            raise ValueError(f"Failed to delete region: {str(e)}")

    @region_endpoint(
        "DELETE",
        "/region_and_members",
        request_model=RegionIdRequest,
        response_model=SuccessResponse,
    )
    def delete_region_and_members(
        self, region_id_data: RegionIdRequest
    ) -> SuccessResponse:
        """Delete a brain region and all its members."""
        try:
            region_id = region_id_data.id

            if region_id == "root":
                raise ValueError("Root region cannot be deleted")

            connectome = self.core_api_service.get_connectome()
            if not connectome:
                raise ValueError("Connectome is not ready!")

            # Use new ConnectomeManager structure
            if (
                hasattr(connectome, "brain_regions")
                and connectome.brain_regions
            ):
                if region_id not in connectome.brain_regions:
                    raise ValueError(f"{region_id} is not a valid region id")

            success = self.core_api_service.delete_brain_region(
                region_id, preserve_children=False
            )
            if not success:
                raise ValueError("Failed to delete brain region and members")

            return SuccessResponse(
                message="Brain region and members deleted successfully"
            )
        except Exception as e:
            logger.error(f"Error deleting region and members: {e}")
            raise ValueError(f"Failed to delete region and members: {str(e)}")

    @region_endpoint("GET", "/regions")
    def list_all_regions(self) -> Dict[str, Dict[str, List[int]]]:
        """List all brain regions (summary with coordinates)."""
        # Return exact legacy format - hardcoded for now
        return {"root": {"coordinate_2d": [0, 0], "coordinate_3d": [0, 0, 0]}}

    @region_endpoint("GET", "/regions_members")
    def list_all_regions_and_members(self) -> Dict[str, Any]:
        """List all brain regions and their members with consistent schema."""
        try:
            # Use the same normalization logic as core API service for consistency
            regions_list = self.core_api_service.get_brain_regions()
            
            # Convert list to dictionary format expected by this endpoint
            result = {}
            for region in regions_list:
                region_id = region["region_id"]
                result[region_id] = {
                    "title": region["title"],
                    "description": region["description"],
                    "parent_region_id": region["parent_region_id"],
                    "coordinate_2d": region["coordinate_2d"],
                    "coordinate_3d": region["coordinate_3d"],
                    "areas": region["areas"],
                    "regions": region["regions"],
                    "inputs": region["inputs"],
                    "outputs": region["outputs"]
                }
            
            return result
            
        except Exception as e:
            logger.error(f"Error getting regions and members: {e}")
            # Fallback to empty root region if error occurs
            return {
                "root": {
                    "title": "Root Brain Region",
                    "description": "Default root region",
                    "parent_region_id": None,
                    "coordinate_2d": [0, 0],
                    "coordinate_3d": [0, 0, 0],
                    "areas": [],
                    "regions": [],
                    "inputs": [],
                    "outputs": []
                }
            }

    @region_endpoint("GET", "/region_titles")
    def list_all_region_titles(self) -> List[tuple]:
        """List all region titles."""
        try:
            connectome = self.core_api_service.get_connectome()
            if (
                not connectome
                or not hasattr(connectome, "brain_regions")
                or not connectome.brain_regions
            ):
                # Return minimal structure for compatibility
                return [("root", "Genome's root brain region")]

            title_list = []
            for region_id, region_data in connectome.brain_regions.items():
                # Use region name or id as title
                region_title = region_data.get("name", region_id)
                title_list.append((region_id, region_title))

            return title_list
        except Exception as e:
            logger.error(f"Error listing region titles: {e}")
            raise ValueError(f"Failed to list region titles: {str(e)}")

    @region_endpoint(
        "PUT",
        "/change_cortical_area_region",
        request_model=RegionAssociation,
        response_model=SuccessResponse,
    )
    def update_cortical_area_region_association(
        self, association_data: RegionAssociation
    ) -> SuccessResponse:
        """Update cortical area region association."""
        try:
            connectome = self.core_api_service.get_connectome()
            if not connectome:
                raise ValueError("Connectome is not ready!")

            #  For compatibility, just use core API service without genome
            #  validation
            success = self.core_api_service.change_cortical_area_parent(
                cortical_area_id=association_data.id,
                new_parent_id=association_data.new_region_id,
            )

            if not success:
                raise ValueError(
                    "Failed to update cortical area region association"
                )

            return SuccessResponse(
                message="Cortical area region association updated successfully"
            )
        except Exception as e:
            logger.error(
                f"Error updating cortical area region association: {e}"
            )
            raise ValueError(
                f"Failed to update cortical area region association: {str(e)}"
            )

    @region_endpoint(
        "PUT",
        "/change_region_parent",
        request_model=RegionAssociation,
        response_model=SuccessResponse,
    )
    def update_brain_region_parent(
        self, association_data: RegionAssociation
    ) -> SuccessResponse:
        """Update brain region parent."""
        try:
            connectome = self.core_api_service.get_connectome()
            if not connectome:
                raise ValueError("Connectome is not ready!")

            # Use new ConnectomeManager structure for validation
            if (
                hasattr(connectome, "brain_regions")
                and connectome.brain_regions
            ):
                # Validate region exists
                if association_data.id not in connectome.brain_regions:
                    raise ValueError(
                        f"{association_data.id} is not a valid brain region id"
                    )

                # Validate new parent region exists
                if (
                    association_data.new_region_id
                    not in connectome.brain_regions
                ):
                    raise ValueError(
                        f"{association_data.new_region_id} is not a valid brain region id"
                    )

            success = self.core_api_service.change_brain_region_parent(
                region_id=association_data.id,
                new_parent_id=association_data.new_region_id,
            )

            if not success:
                raise ValueError("Failed to update brain region parent")

            return SuccessResponse(
                message="Brain region parent updated successfully"
            )
        except Exception as e:
            logger.error(f"Error updating brain region parent: {e}")
            raise ValueError(f"Failed to update brain region parent: {str(e)}")

    @region_endpoint(
        "PUT", "/relocate_members", 
        request_model=RegionMemberRelocationRequest,
        response_model=Dict[str, Any]
    )
    def brain_region_member_relocation(
        self, relocation_data: RegionMemberRelocationRequest
    ) -> Dict[str, Any]:
        """Brain region member relocation.

        Accepts a dictionary for updating coordinates and/or parent region assignments
        of one or more cortical areas. At least one of coordinate_2d or parent_region_id
        must be provided for each entry.

        Returns the complete brain region data structure for the destination region
        (same format as POST/PUT /region/region endpoints).

        Input format:
        {
            "cortical_area_1": {
                "coordinate_2d": [10, 9],
                "parent_region_id": "region_1",
            },
            "cortical_area_2": {
                "coordinate_2d": [4, 93],
                "parent_region_id": "region_2",
            },
            "cortical_area_3": {
                "parent_region_id": "region_3",  # coordinates optional
            }
        }
        """
        try:
            # Extract the dictionary data from the Pydantic RootModel
            relocation_dict = relocation_data.root
            success = self.core_api_service.relocate_region_members(
                relocation_dict
            )
            if not success:
                raise ValueError("Failed to relocate region members")

            # Get the destination brain region and return complete data structure
            try:
                # Extract the first (and likely only) destination region from the relocation data
                destination_region_id = None
                relocation_dict = relocation_data.root
                
                for member_id, member_data in relocation_dict.items():
                    parent_region_id = member_data.get("parent_region_id")
                    if parent_region_id:
                        destination_region_id = parent_region_id
                        break
                
                if not destination_region_id:
                    logger.warning("No destination region found in relocation data, returning success message only")
                    return {"message": "Region members relocated successfully"}
                
                logger.info(f"Fetching complete data for destination region: {destination_region_id}")
                
                # Use the same approach as create/update region endpoints
                all_regions = self.core_api_service.get_brain_regions()
                
                # Find the destination region
                for region in all_regions:
                    if region.get("region_id") == destination_region_id:
                        logger.info(f"Found complete region data for destination {destination_region_id}")
                        return region
                
                logger.warning(f"Destination region {destination_region_id} not found, returning minimal structure")
                return {
                    "region_id": destination_region_id,
                    "title": f"Region {destination_region_id}",
                    "description": "",
                    "parent_region_id": "root",
                    "coordinate_2d": [0, 0],
                    "coordinate_3d": [0, 0, 0],
                    "areas": [],
                    "regions": [],
                    "inputs": [],
                    "outputs": [],
                    "signature": ""
                }
                
            except Exception as fetch_error:
                logger.warning(f"Failed to fetch destination region data: {fetch_error}")
                return {"message": "Region members relocated successfully", "warning": "Could not fetch updated region data"}
                
        except Exception as e:
            logger.error(f"Error relocating region members: {e}")
            raise ValueError(f"Failed to relocate region members: {str(e)}")

    # ===== Region Clone (pending amalgamation) =====

    @region_endpoint(
        "POST",
        "/clone",
        request_model=CloneBrainRegionRequest,
        response_model=CloneBrainRegionResponse,
        description=(
            "Prepare a clone of a brain region using amalgamation pending workflow. "
            "No genome changes until amalgamation destination is submitted."
        ),
    )
    def clone_brain_region(self, request: CloneBrainRegionRequest) -> CloneBrainRegionResponse:
        """Initiate region clone via amalgamation pending.

        - Extract subtree genome from source_region_id
        - Set pending amalgamation with optional region_name
        - Return amalgamation_id and circuit_size for BV to finalize later
        """
        try:
            # Build genome payload from region subtree
            from feagi.bdu.models.brain_region import construct_genome_from_region

            genome_payload = construct_genome_from_region(request.source_region_id)

            # Override genome title if provided
            genome_title = request.region_name or genome_payload.get("genome_title", "Cloned Region")

            # Kick off amalgamation pending using core API service
            result = self.core_api_service.process_amalgamation_request(
                genome_payload=genome_payload,
                genome_id=genome_payload.get("genome_id", request.source_region_id),
                genome_title=genome_title,
            )

            if not result.get("success"):
                raise ValueError(result.get("error", "Failed to start region clone (amalgamation pending)"))

            amalgamation_id: str = result.get("amalgamation_id", "")
            circuit_size = result.get("circuit_size", [1, 1, 1])

            return CloneBrainRegionResponse(
                amalgamation_id=amalgamation_id,
                circuit_size=circuit_size,
                message="Region clone pending created. Finalize via amalgamation destination.",
            )
        except Exception as e:
            logger.error(f"Error initiating region clone: {e}")
            raise ValueError(f"Failed to initiate region clone: {str(e)}")

    # ===== Legacy endpoints provide complete functionality - new endpoints removed =====



# ===== Factory Function =====


def create_region_api(core_api_service: CoreAPIService) -> RegionAPI:
    """Factory function to create a RegionAPI instance.

    This function can be used by transport adapters to get a configured
    RegionAPI instance with the required dependencies.
    """
    return RegionAPI(core_api_service)
