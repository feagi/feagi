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
FEAGI v1 Region API - Single Source of Truth

This module contains the ONLY definitions of brain region API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Any, Dict, List, Optional

from pydantic import BaseModel

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

from .decorators import endpoint
from .schemas import (
    CreateRegionRequest,
    RegionInfoResponse,
    RegionListResponse,
    SuccessResponse,
    UpdateRegionRequest,
)

logger = setup_logger(__name__)


# ===== Region-specific Schemas =====


class RegionAssociation(BaseModel):
    """Request model for changing region associations."""

    id: str
    new_region_id: str


class NewRegionProperties(BaseModel):
    """Request model for creating a new region."""

    region_id: str
    region_title: str
    parent_region_id: str
    coordinate_2d: List[int]
    coordinate_3d: List[int]


class UpdateRegionProperties(BaseModel):
    """Request model for updating region properties."""

    region_id: str
    region_title: Optional[str] = None
    coordinate_2d: Optional[List[int]] = None
    coordinate_3d: Optional[List[int]] = None


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
        response_model=Dict[str, str],
    )
    def create_brain_region(
        self, region_data: NewRegionProperties
    ) -> Dict[str, str]:
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

            # Create the region using core API service
            region_id = self.core_api_service.create_brain_region(
                region_data.dict()
            )
            return {"region_id": region_id}
        except Exception as e:
            logger.error(f"Error creating brain region: {e}")
            raise ValueError(f"Failed to create brain region: {str(e)}")

    @region_endpoint(
        "PUT",
        "/region",
        request_model=UpdateRegionProperties,
        response_model=SuccessResponse,
    )
    def update_region_properties(
        self, region_data: UpdateRegionProperties
    ) -> SuccessResponse:
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

            # Remove parent_region_id if present as it's handled separately
            if "parent_region_id" in region_dict:
                region_dict.pop("parent_region_id")

            success = self.core_api_service.update_brain_region_properties(
                region_dict
            )
            if not success:
                raise ValueError("Failed to update brain region properties")

            return SuccessResponse(
                message="Brain region properties updated successfully"
            )
        except Exception as e:
            logger.error(f"Error updating region properties: {e}")
            raise ValueError(f"Failed to update region properties: {str(e)}")

    @region_endpoint("GET", "/region")
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
        """List all brain regions and their members (returns legacy format)."""
        # Get cortical area IDs and return in legacy format
        try:
            cortical_area_ids = (
                self.core_api_service.get_cortical_area_id_list()
            )
        except Exception:
            cortical_area_ids = []

        # Return exact legacy format
        return {
            "root": {
                "title": "Genome's root brain region",
                "description": None,
                "parent_region_id": None,
                "coordinate_2d": [0, 0],
                "coordinate_3d": [0, 0, 0],
                "areas": cortical_area_ids,
                "regions": [],
                "inputs": [],
                "outputs": [],
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

            # For compatibility, just use core API service without genome validation
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
        "PUT", "/relocate_members", response_model=SuccessResponse
    )
    def brain_region_member_relocation(
        self, relocation_data: Dict[str, Any]
    ) -> SuccessResponse:
        """Brain region member relocation.

        Accepts a dictionary of 2D coordinates of one or more cortical areas and update them in genome.

        Input format:
        {
            "region_id_1": {
                "coordinate_2d": [10, 9],
                "parent_region_id": "fhafsihwfiuhr23r_b",
            },
            "region_id_2": {
                "coordinate_2d": [4, 93],
                "parent_region_id": "dhdfsihwfiuhr23r_b",
            },
            "cortical_area_id": {
                "coordinate_2d": [30, 29],
                "parent_region_id": "gdfsihwfiuhr23r_b",
            }
        }
        """
        try:
            success = self.core_api_service.relocate_region_members(
                relocation_data
            )
            if not success:
                raise ValueError("Failed to relocate region members")

            return SuccessResponse(
                message="Region members relocated successfully"
            )
        except Exception as e:
            logger.error(f"Error relocating region members: {e}")
            raise ValueError(f"Failed to relocate region members: {str(e)}")

    # ===== New API Endpoints (for future use) =====

    @region_endpoint("GET", "/list", response_model=RegionListResponse)
    async def get_regions_list(self) -> RegionListResponse:
        """Get list of all brain regions."""
        try:
            regions = self.core_api_service.get_brain_regions()
            return RegionListResponse(regions=regions)
        except Exception as e:
            logger.error(f"Error getting regions list: {e}")
            raise ValueError(f"Failed to get regions list: {str(e)}")

    @region_endpoint(
        "GET", "/info/{region_id}", response_model=RegionInfoResponse
    )
    async def get_region_info(self, region_id: str) -> RegionInfoResponse:
        """Get information about a specific brain region."""
        try:
            region_info = self.core_api_service.get_brain_region_info(
                region_id
            )
            return RegionInfoResponse(region_info=region_info)
        except Exception as e:
            logger.error(f"Error getting region info: {e}")
            raise ValueError(f"Failed to get region info: {str(e)}")

    @region_endpoint(
        "POST",
        "/create",
        request_model=CreateRegionRequest,
        response_model=SuccessResponse,
    )
    async def create_region(
        self, request: CreateRegionRequest
    ) -> SuccessResponse:
        """Create a new brain region."""
        try:
            success = self.core_api_service.create_brain_region(
                request.region_data
            )
            if not success:
                raise ValueError("Failed to create brain region")

            return SuccessResponse(message="Brain region created successfully")
        except Exception as e:
            logger.error(f"Error creating region: {e}")
            raise ValueError(f"Failed to create region: {str(e)}")

    @region_endpoint(
        "PUT",
        "/update",
        request_model=UpdateRegionRequest,
        response_model=SuccessResponse,
    )
    async def update_region(
        self, request: UpdateRegionRequest
    ) -> SuccessResponse:
        """Update an existing brain region."""
        try:
            success = self.core_api_service.update_brain_region(
                request.region_id, request.updates
            )
            if not success:
                raise ValueError("Failed to update brain region")

            return SuccessResponse(message="Brain region updated successfully")
        except Exception as e:
            logger.error(f"Error updating region: {e}")
            raise ValueError(f"Failed to update region: {str(e)}")

    @region_endpoint(
        "DELETE", "/delete/{region_id}", response_model=SuccessResponse
    )
    async def delete_region_new_api(self, region_id: str) -> SuccessResponse:
        """Delete a brain region."""
        try:
            success = self.core_api_service.delete_brain_region(region_id)
            if not success:
                raise ValueError("Failed to delete brain region")

            return SuccessResponse(message="Brain region deleted successfully")
        except Exception as e:
            logger.error(f"Error deleting region: {e}")
            raise ValueError(f"Failed to delete region: {str(e)}")


# ===== Factory Function =====


def create_region_api(core_api_service: CoreAPIService) -> RegionAPI:
    """Factory function to create a RegionAPI instance.

    This function can be used by transport adapters to get a configured
    RegionAPI instance with the required dependencies.
    """
    return RegionAPI(core_api_service)
