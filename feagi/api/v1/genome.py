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
FEAGI v1 Genome API - Single Source of Truth

This module contains the ONLY definitions of genome API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

import json
from enum import Enum
from typing import Any, Dict

from fastapi import HTTPException, UploadFile
from pydantic import BaseModel

from feagi.api.core.services.core_api_service import CoreAPIService

# Import genome conversion functions for hierarchical <-> flat conversion
from feagi.evo.genome_processor import genome_v1_v2_converter
from feagi.utils.logger import setup_logger

from .decorators import genome_endpoint
from .schemas import (
    AmalgamationHistoryResponse,
    AmalgamationRequest,
    AmalgamationResponse,
    CircuitLibraryResponse,
    CorticalTemplateResponse,
    GenomeDefaultFilesResponse,
    GenomeDownloadResponse,
    GenomeFileNameResponse,
    GenomeNumberResponse,
    GenomeUploadResponse,
    SuccessResponse,
)

logger = setup_logger(__name__)

# ===== Genome-specific Schemas =====


class RewiringMode(str, Enum):
    """Enum for rewiring modes during amalgamation."""

    rewire_all = "rewire_all"
    no_rewiring = "no_rewiring"


class AmalgamationDestinationRequest(BaseModel):
    """Request model for amalgamation destination."""

    circuit_origin_x: int
    circuit_origin_y: int
    circuit_origin_z: int
    amalgamation_id: str
    brain_region_id: str = "root"
    rewire_mode: RewiringMode = RewiringMode.rewire_all


class AppendFileRequest(BaseModel):
    """Request model for appending file to genome."""

    circuit_origin_x: int
    circuit_origin_y: int
    circuit_origin_z: int
    content: str
    filename: str


class GenomeEditResponse(BaseModel):
    """Response model for genome file editing."""

    content: str


class GenomeAPI:
    """
    Genome API - Single Source of Truth for ALL Transports

    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.

    This ensures identical behavior across all transports with zero duplication.
    """

    def __init__(self, core_api_service: CoreAPIService):
        """Initialize with core API service dependency."""
        self.core_api_service = core_api_service

    # ===== Genome Upload Endpoints =====

    @genome_endpoint(
        "POST", "/upload/barebones", response_model=GenomeUploadResponse
    )
    async def upload_barebones_genome(self) -> GenomeUploadResponse:
        """Upload/load the barebones genome."""
        logger.info("Loading barebones genome")
        result = self.core_api_service.load_barebones_genome()

        # Prepare the final response based on loading results
        response_data = GenomeUploadResponse(
            success=result["success"],
            message=(
                "Barebones genome loaded successfully"
                if result["success"] and result.get("genome_validity", True)
                else (
                    f"Barebones genome loaded but marked as invalid: {result.get('message', 'Validation failed')}"
                    if result["success"]
                    and not result.get("genome_validity", True)
                    else f"Barebones genome failed to load: {result.get('error', 'Unknown error')}"
                )
            ),
            genome_number=self.core_api_service.get_genome_counter(),
            details=(
                result
                if not result["success"]
                else (
                    {
                        "success": result["success"],
                        "cortical_area_count": result.get(
                            "cortical_area_count", 0
                        ),
                        "genome_validity": result.get("genome_validity", True),
                        "validation_errors": (
                            result.get("validation_errors", [])
                            if not result.get("genome_validity", True)
                            else []
                        ),
                        "validation_warnings": result.get(
                            "validation_warnings", []
                        ),
                    }
                )
            ),
        )

        return response_data

    @genome_endpoint(
        "POST", "/upload/essential", response_model=GenomeUploadResponse
    )
    async def upload_essential_genome(self) -> GenomeUploadResponse:
        """Upload/load the essential genome."""
        logger.info("Loading essential genome")

        # Load essential genome data and use the single load_genome method
        import json
        from pathlib import Path

        # Get the essential genome file path
        essential_genome_path = (
            Path(__file__).parent.parent.parent
            / "evo"
            / "defaults"
            / "genome"
            / "essential_genome.json"
        )

        if not essential_genome_path.exists():
            logger.error(
                f"Essential genome file not found: {essential_genome_path}"
            )
            result = {
                "success": False,
                "error": f"Essential genome file not found: {essential_genome_path}",
            }
        else:
            # Read the essential genome file
            with open(essential_genome_path, "r") as f:
                genome_data = json.load(f)

            # Use the single load_genome method for consistency and dynamic sizing
            result = self.core_api_service.load_genome(
                genome_data, filename="essential_genome.json"
            )

        # Prepare the final response based on loading results
        response_data = GenomeUploadResponse(
            success=result["success"],
            message=(
                "Essential genome loaded successfully"
                if result["success"] and result.get("genome_validity", True)
                else (
                    f"Essential genome loaded but marked as invalid: {result.get('message', 'Validation failed')}"
                    if result["success"]
                    and not result.get("genome_validity", True)
                    else f"Essential genome failed to load: {result.get('error', 'Unknown error')}"
                )
            ),
            genome_number=self.core_api_service.get_genome_counter(),
            details=(
                result
                if not result["success"]
                else (
                    {
                        "success": result["success"],
                        "cortical_area_count": result.get(
                            "cortical_area_count", 0
                        ),
                        "genome_validity": result.get("genome_validity", True),
                        "validation_errors": (
                            result.get("validation_errors", [])
                            if not result.get("genome_validity", True)
                            else []
                        ),
                        "validation_warnings": result.get(
                            "validation_warnings", []
                        ),
                    }
                )
            ),
        )

        return response_data

    @genome_endpoint(
        "POST", "/upload/file", response_model=GenomeUploadResponse
    )
    async def upload_genome_file(
        self, file: UploadFile
    ) -> GenomeUploadResponse:
        """Upload a genome file from user's computer."""
        try:
            # Validate file type
            if not file.filename:
                raise ValueError("No file provided")

            if not file.filename.endswith((".json", ".txt")):
                raise ValueError("File must be a JSON or text file")

            # Read file content
            file_content = await file.read()
            if not file_content:
                raise ValueError("File is empty")

            # Decode file content
            try:
                content_str = file_content.decode("utf-8")
                genome_data = json.loads(content_str)
            except UnicodeDecodeError:
                raise ValueError("File must be UTF-8 encoded") from None
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid JSON format: {str(e)}") from e

            # Validate genome structure (basic check)
            if not isinstance(genome_data, dict):
                raise ValueError("Genome file must contain a JSON object")

            # Add default fields if missing
            if "genome_title" not in genome_data:
                genome_data["genome_title"] = file.filename

            if "genome_description" not in genome_data:
                genome_data["genome_description"] = (
                    f"Uploaded from {file.filename}"
                )

            # Load the genome
            result = self.core_api_service.load_genome(
                genome_data, filename=file.filename
            )

            # Prepare the final response based on loading results
            response_data = GenomeUploadResponse(
                success=result["success"],
                message=(
                    "Genome file uploaded and loaded successfully"
                    if result["success"]
                    and result.get("genome_validity", True)
                    else (
                        f"Genome file uploaded and loaded but marked as invalid: {result.get('message', 'Validation failed')}"
                        if result["success"]
                        and not result.get("genome_validity", True)
                        else f"Genome file uploaded but failed to load: {result.get('error', 'Unknown error')}"
                    )
                ),
                genome_number=self.core_api_service.get_genome_counter(),
                details=(
                    result
                    if not result["success"]
                    else (
                        {
                            "success": result["success"],
                            "cortical_area_count": result.get(
                                "cortical_area_count", 0
                            ),
                            "genome_validity": result.get(
                                "genome_validity", True
                            ),
                            "validation_errors": (
                                result.get("validation_errors", [])
                                if not result.get("genome_validity", True)
                                else []
                            ),
                            "validation_warnings": result.get(
                                "validation_warnings", []
                            ),
                        }
                    )
                ),
            )

            return response_data
        except Exception as e:
            logger.error(
                f"Failed to upload genome file: {str(e)}", status="[ERR]"
            )
            raise HTTPException(
                status_code=400,
                detail=f"Failed to upload genome file: {str(e)}",
            ) from e

    @genome_endpoint(
        "POST", "/upload/string", response_model=GenomeUploadResponse
    )
    def upload_genome_string(self, genome: dict) -> GenomeUploadResponse:
        """Upload a genome from JSON string."""
        try:
            # Load the genome
            result = self.core_api_service.load_genome(
                genome, filename="uploaded_genome.json"
            )

            # Prepare the final response based on loading results
            response_data = GenomeUploadResponse(
                success=result["success"],
                message=(
                    "Genome string uploaded and loaded successfully"
                    if result["success"]
                    and result.get("genome_validity", True)
                    else (
                        f"Genome string uploaded and loaded but marked as invalid: {result.get('message', 'Validation failed')}"
                        if result["success"]
                        and not result.get("genome_validity", True)
                        else f"Genome string uploaded but failed to load: {result.get('error', 'Unknown error')}"
                    )
                ),
                genome_number=self.core_api_service.get_genome_counter(),
                details=(
                    result
                    if not result["success"]
                    else (
                        {
                            "success": result["success"],
                            "cortical_area_count": result.get(
                                "cortical_area_count", 0
                            ),
                            "genome_validity": result.get(
                                "genome_validity", True
                            ),
                            "validation_errors": (
                                result.get("validation_errors", [])
                                if not result.get("genome_validity", True)
                                else []
                            ),
                            "validation_warnings": result.get(
                                "validation_warnings", []
                            ),
                        }
                    )
                ),
            )

            return response_data
        except Exception as e:
            logger.error(f"Failed to upload genome string: {str(e)}")
            raise ValueError(
                f"Failed to upload genome string: {str(e)}"
            ) from e

    # ===== Genome Information Endpoints =====

    @genome_endpoint(
        "GET", "/file_name", response_model=GenomeFileNameResponse
    )
    def get_genome_file_name(self) -> GenomeFileNameResponse:
        """Get the current genome file name."""
        try:
            filename = self.core_api_service.get_genome_file_name()
            return GenomeFileNameResponse(file_name=filename)
        except Exception as e:
            logger.error(f"Error getting genome file name: {e}")
            raise ValueError(
                f"Failed to get genome file name: {str(e)}"
            ) from e

    @genome_endpoint("GET", "/file_name")
    def get_genome_file_name_legacy_compatible(self) -> str:
        """Get the current genome file name (legacy compatible - returns string directly)."""
        try:
            filename = self.core_api_service.get_genome_filename()
            return filename or ""
        except Exception as e:
            logger.error(f"Error getting genome file name: {e}")
            raise ValueError(
                f"Failed to get genome file name: {str(e)}"
            ) from e

    @genome_endpoint("GET", "/file_name")
    def get_genome_file_name_direct(self) -> str:
        """Get the current genome file name (returns string directly for legacy compatibility)."""
        try:
            filename = self.core_api_service.get_genome_filename()
            return filename or ""
        except Exception as e:
            logger.error(f"Error getting genome file name: {e}")
            raise ValueError(
                f"Failed to get genome file name: {str(e)}"
            ) from e

    @genome_endpoint("GET", "/file_name")
    def get_genome_file_name_legacy_format(self) -> str:
        """Get the current genome file name (legacy format - returns string directly)."""
        try:
            filename = self.core_api_service.get_genome_filename()
            return filename or ""
        except Exception as e:
            logger.error(f"Error getting genome file name: {e}")
            raise ValueError(
                f"Failed to get genome file name: {str(e)}"
            ) from e

    @genome_endpoint(
        "GET", "/genome_number", response_model=GenomeNumberResponse
    )
    def get_genome_number(self) -> GenomeNumberResponse:
        """Get the current genome number."""
        try:
            number = self.core_api_service.get_genome_counter()
            return GenomeNumberResponse(genome_number=number)
        except Exception as e:
            logger.error(f"Error getting genome number: {e}")
            raise ValueError(f"Failed to get genome number: {str(e)}") from e

    @genome_endpoint("GET", "/download", response_model=GenomeDownloadResponse)
    def download_genome(self) -> GenomeDownloadResponse:
        """Download the current genome."""
        try:
            # Get hierarchical genome from service
            genome_data = self.core_api_service.get_current_genome()
            filename = self.core_api_service.get_genome_filename()

            if not genome_data:
                raise ValueError("No genome data available")

            # Convert hierarchical format to flat format for export/download
            # ARCHITECTURE: Hierarchical is for working, flat is for storage/export
            if "blueprint" in genome_data and isinstance(
                genome_data["blueprint"], dict
            ):
                # Check if already flat format (has flattened keys)
                blueprint_keys = list(genome_data["blueprint"].keys())
                if blueprint_keys and not any(
                    "10c-" in key and "-cx-" in key
                    for key in blueprint_keys[:5]
                ):
                    # Convert hierarchical to flat format for export
                    logger.info(
                        "Converting hierarchical genome to flat format for download"
                    )
                    flat_genome = genome_v1_v2_converter(genome_data)
                    genome_data = flat_genome
                    logger.info(
                        f"Converted to flat format with {len(flat_genome.get('blueprint', {}))} entries"
                    )

            return GenomeDownloadResponse(
                genome_data=genome_data, filename=filename
            )
        except Exception as e:
            logger.error(f"Error downloading genome: {e}")
            raise ValueError(f"Failed to download genome: {str(e)}") from e

    @genome_endpoint(
        "GET", "/download_region", response_model=GenomeDownloadResponse
    )
    async def download_genome_from_region(
        self, region_id: str
    ) -> GenomeDownloadResponse:
        """Download genome data from a specific brain region."""
        try:
            from feagi.bdu.models.brain_region import (
                construct_genome_from_region,
            )

            # Get connectome
            connectome = self.core_api_service.get_connectome()
            if not connectome:
                raise ValueError("Connectome not available")

            # Construct genome from region
            genome_data = construct_genome_from_region(connectome, region_id)
            filename = f"genome_region_{region_id}.json"

            return GenomeDownloadResponse(
                genome_data=genome_data, filename=filename
            )
        except Exception as e:
            logger.error(f"Error downloading genome from region: {e}")
            raise ValueError(
                f"Failed to download genome from region: {str(e)}"
            ) from e

    @genome_endpoint(
        "GET", "/defaults/files", response_model=GenomeDefaultFilesResponse
    )
    def get_default_genome_files(self) -> GenomeDefaultFilesResponse:
        """Get list of available default genome files."""
        try:
            files = self.core_api_service.get_default_genome_files()
            return GenomeDefaultFilesResponse(files=files)
        except Exception as e:
            logger.error(f"Error getting default genome files: {e}")
            raise ValueError(
                f"Failed to get default genome files: {str(e)}"
            ) from e

    # ===== Genome Operations =====

    @genome_endpoint("POST", "/reset", response_model=SuccessResponse)
    async def reset_genome(self) -> SuccessResponse:
        """Reset the current genome."""
        try:
            success = self.core_api_service.reset_genome()
            if success:
                return SuccessResponse(message="Genome reset successfully")
            else:
                raise ValueError("Failed to reset genome")
        except Exception as e:
            logger.error(f"Error resetting genome: {e}")
            raise ValueError(f"Failed to reset genome: {str(e)}") from e

    # ===== Amalgamation Endpoints =====

    @genome_endpoint(
        "POST", "/amalgamation_by_payload", response_model=AmalgamationResponse
    )
    async def amalgamate_by_payload(
        self, request: AmalgamationRequest
    ) -> AmalgamationResponse:
        """Perform genome amalgamation using payload data."""
        try:
            result = self.core_api_service.process_amalgamation_request(
                genome_payload=request.genome_payload,
                genome_id=request.genome_id,
                genome_title=request.genome_title,
            )

            return AmalgamationResponse(
                amalgamation_id=result.get("amalgamation_id", ""),
                status="success",
                message="Amalgamation request processed successfully",
            )
        except Exception as e:
            logger.error(f"Error in amalgamation by payload: {e}")
            raise ValueError(
                f"Failed to process amalgamation: {str(e)}"
            ) from e

    @genome_endpoint(
        "POST", "/amalgamation_by_upload", response_model=AmalgamationResponse
    )
    async def amalgamate_by_upload(
        self, file: UploadFile
    ) -> AmalgamationResponse:
        """Perform genome amalgamation using uploaded file."""
        try:
            # Validate file
            if not file.filename:
                raise ValueError("No file provided")

            if not file.filename.endswith((".json", ".txt")):
                raise ValueError("File must be a JSON or text file")

            # Read and parse file content
            file_content = await file.read()
            if not file_content:
                raise ValueError("File is empty")

            try:
                content_str = file_content.decode("utf-8")
                genome_data = json.loads(content_str)
            except UnicodeDecodeError:
                raise ValueError("File must be UTF-8 encoded") from None
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid JSON format: {str(e)}") from e

            result = self.core_api_service.process_amalgamation_request(
                genome_payload=genome_data
            )

            return AmalgamationResponse(
                amalgamation_id=result.get("amalgamation_id", ""),
                status="success",
                message=f"Amalgamation request processed successfully from {file.filename}",
            )
        except Exception as e:
            logger.error(f"Error in amalgamation by upload: {e}")
            raise HTTPException(
                status_code=400,
                detail=f"Failed to process amalgamation: {str(e)}",
            ) from e

    @genome_endpoint(
        "POST",
        "/amalgamation_by_filename",
        response_model=AmalgamationResponse,
    )
    async def amalgamate_by_filename(
        self, request: AmalgamationRequest
    ) -> AmalgamationResponse:
        """Perform genome amalgamation using filename."""
        try:
            result = self.core_api_service.process_amalgamation_request(
                genome_id=request.genome_id, genome_title=request.genome_title
            )

            return AmalgamationResponse(
                amalgamation_id=result.get("amalgamation_id", ""),
                status="success",
                message="Amalgamation request processed successfully",
            )
        except Exception as e:
            logger.error(f"Error in amalgamation by filename: {e}")
            raise ValueError(
                f"Failed to process amalgamation: {str(e)}"
            ) from e

    @genome_endpoint(
        "GET",
        "/amalgamation_history",
        response_model=AmalgamationHistoryResponse,
    )
    def get_amalgamation_history(self) -> AmalgamationHistoryResponse:
        """Get amalgamation history."""
        try:
            history = self.core_api_service.get_amalgamation_history()
            return AmalgamationHistoryResponse(history=history)
        except Exception as e:
            logger.error(f"Error getting amalgamation history: {e}")
            raise ValueError(
                f"Failed to get amalgamation history: {str(e)}"
            ) from e

    @genome_endpoint("GET", "/amalgamation")
    def get_amalgamation(self, amalgamation_id: str) -> Dict[str, Any]:
        """Get specific amalgamation details."""
        try:
            result = self.core_api_service.get_amalgamation_details(
                amalgamation_id
            )
            return result
        except Exception as e:
            logger.error(f"Error getting amalgamation: {e}")
            raise ValueError(f"Failed to get amalgamation: {str(e)}") from e

    @genome_endpoint(
        "DELETE", "/amalgamation_cancellation", response_model=SuccessResponse
    )
    def cancel_amalgamation(self, amalgamation_id: str) -> SuccessResponse:
        """Cancel an amalgamation request."""
        try:
            success = self.core_api_service.cancel_amalgamation(
                amalgamation_id
            )
            if success:
                return SuccessResponse(
                    message="Amalgamation cancelled successfully"
                )
            else:
                raise ValueError("Failed to cancel amalgamation")
        except Exception as e:
            logger.error(f"Error cancelling amalgamation: {e}")
            raise ValueError(f"Failed to cancel amalgamation: {str(e)}") from e

    # ===== Template and Circuit Endpoints =====

    @genome_endpoint(
        "GET", "/cortical_template", response_model=CorticalTemplateResponse
    )
    def get_cortical_template(self) -> CorticalTemplateResponse:
        """Get cortical template."""
        try:
            from feagi.evo.templates import cortical_template

            template = cortical_template()
            return CorticalTemplateResponse(template=template)
        except Exception as e:
            logger.error(f"Error getting cortical template: {e}")
            raise ValueError(
                f"Failed to get cortical template: {str(e)}"
            ) from e

    @genome_endpoint("GET", "/circuits", response_model=CircuitLibraryResponse)
    def get_circuit_library(self) -> CircuitLibraryResponse:
        """Get the circuit library list."""
        try:
            circuits = self.core_api_service.get_circuit_library()
            return CircuitLibraryResponse(circuits=circuits)
        except Exception as e:
            logger.error(f"Error getting circuit library: {e}")
            raise ValueError(f"Failed to get circuit library: {str(e)}") from e

    # ===== Missing Critical Legacy Endpoints =====

    @genome_endpoint("POST", "/amalgamation_destination", response_model=str)
    def amalgamation_destination(
        self, request: AmalgamationDestinationRequest
    ) -> str:
        """Complete amalgamation by specifying destination coordinates."""
        try:
            # Check if there's a pending amalgamation
            if not self.core_api_service.has_pending_amalgamation():
                raise ValueError("No pending amalgamation request found")

            # Prepare amalgamation payload
            payload = {
                "genome_str": self.core_api_service.get_pending_amalgamation_genome(),
                "circuit_origin": [
                    request.circuit_origin_x,
                    request.circuit_origin_y,
                    request.circuit_origin_z,
                ],
                "parent_brain_region": request.brain_region_id,
                "rewire_mode": request.rewire_mode.value,
            }

            # Send to core service for processing
            success = self.core_api_service.complete_amalgamation(
                payload, request.amalgamation_id
            )
            if not success:
                raise ValueError("Failed to complete amalgamation")

            genome_title = (
                self.core_api_service.get_pending_amalgamation_title()
            )
            self.core_api_service.cancel_pending_amalgamation(
                request.amalgamation_id
            )
            self.core_api_service.mark_amalgamation_complete(
                request.amalgamation_id
            )

            return f'Amalgamation for "{genome_title}" is complete.'
        except Exception as e:
            logger.error(f"Error completing amalgamation destination: {e}")
            raise ValueError(
                f"Failed to complete amalgamation destination: {str(e)}"
            ) from e

    @genome_endpoint("POST", "/append-file", response_model=SuccessResponse)
    def append_file_to_genome(
        self, request: AppendFileRequest
    ) -> SuccessResponse:
        """Append a given circuit file to the running genome at a specific location."""
        try:
            # Parse the genome content
            genome_data = json.loads(request.content)

            # Prepare payload for appending
            payload = {
                "genome_str": genome_data,
                "circuit_origin": [
                    request.circuit_origin_x,
                    request.circuit_origin_y,
                    request.circuit_origin_z,
                ],
            }

            # Send to core service for processing
            success = self.core_api_service.append_circuit_to_genome(payload)
            if not success:
                raise ValueError("Failed to append circuit to genome")

            return SuccessResponse(
                message=f"Circuit from {request.filename} successfully appended to genome"
            )
        except json.JSONDecodeError:
            logger.error("Invalid JSON content in append file request")
            raise ValueError("Invalid JSON content provided") from None
        except Exception as e:
            logger.error(f"Error appending file to genome: {e}")
            raise ValueError(
                f"Failed to append file to genome: {str(e)}"
            ) from e

    @genome_endpoint(
        "POST", "/upload/file/edit", response_model=GenomeEditResponse
    )
    async def upload_file_for_editing(
        self, file: UploadFile
    ) -> GenomeEditResponse:
        """Upload a genome file and return its content for editing."""
        try:
            # Validate file
            if not file.filename:
                raise ValueError("No file provided")

            if not file.filename.endswith((".json", ".txt")):
                raise ValueError("File must be a JSON or text file")

            # Read file content
            file_content = await file.read()
            if not file_content:
                raise ValueError("File is empty")

            try:
                content_str = file_content.decode("utf-8")
                # Validate JSON format but return as string
                json.loads(content_str)  # Just for validation
                return GenomeEditResponse(content=content_str)
            except UnicodeDecodeError:
                raise ValueError("File must be UTF-8 encoded") from None
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid JSON format: {str(e)}") from e

        except Exception as e:
            logger.error(f"Error uploading file for editing: {e}")
            raise HTTPException(
                status_code=400,
                detail=f"Failed to upload file for editing: {str(e)}",
            ) from e


# ===== Factory Function =====


def create_genome_api(core_api_service: CoreAPIService) -> GenomeAPI:
    """
    Factory function to create a GenomeAPI instance.

    This function can be used by transport adapters to get a configured
    GenomeAPI instance with the required dependencies.
    """
    return GenomeAPI(core_api_service)
