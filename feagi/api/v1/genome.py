"""
FEAGI v1 Genome API - Single Source of Truth

This module contains the ONLY definitions of genome API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

import os
import json
import tempfile
from typing import Dict, Any, Optional, List
from fastapi import UploadFile
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger
from .schemas import (
    AmalgamationRequest, GenomeUploadResponse, GenomeFileNameResponse,
    GenomeNumberResponse, GenomeDownloadResponse, AmalgamationResponse,
    AmalgamationHistoryResponse, CircuitLibraryResponse, CorticalTemplateResponse,
    GenomeDefaultFilesResponse, SuccessResponse, ErrorResponse
)
from .decorators import genome_endpoint

logger = setup_logger(__name__)


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
    
    @genome_endpoint('POST', '/upload/barebones', response_model=GenomeUploadResponse)
    async def upload_barebones_genome(self) -> GenomeUploadResponse:
        """Upload the barebones genome template."""
        try:
            # Get the path to the barebones genome
            project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../.."))
            barebones_genome_path = os.path.join(project_root, "feagi/evo/defaults/genome/barebones_genome.json")
            
            if not os.path.exists(barebones_genome_path):
                raise ValueError("Barebones genome template not found")
                
            # Load the genome data
            with open(barebones_genome_path, "r") as genome_file:
                genome_data = json.load(genome_file)
            
            # Load the genome using CoreAPIService
            result = self.core_api_service.load_genome(genome_data, filename="barebones_genome.json")
            
            # Update the burst engine
            burst_engine = self.core_api_service.get_burst_engine()
            if burst_engine:
                burst_engine.update_with_genome()
                logger.info("Burst Engine updated with new genome", emoji1="⚡ ")
            
            return GenomeUploadResponse(
                success=True,
                message="Barebones genome uploaded successfully",
                genome_number=self.core_api_service.get_genome_counter(),
                loaded=result
            )
        except Exception as e:
            logger.error(f"Failed to upload barebones genome: {str(e)}", emoji1="❌")
            raise ValueError(f"Error uploading barebones genome: {str(e)}")
    
    @genome_endpoint('POST', '/upload/essential', response_model=GenomeUploadResponse)
    async def upload_essential_genome(self) -> GenomeUploadResponse:
        """Upload the essential genome template."""
        try:
            essential_path = os.path.join(os.path.dirname(__file__), "../../../../../feagi/evo/defaults/genome/essential_genome.json")
            
            if not os.path.exists(essential_path):
                raise ValueError("Essential genome template not found")
                
            with open(essential_path, 'r') as f:
                genome_data = json.load(f)
            
            # Process and load the genome
            from feagi.evo.genome_processor import process_and_load_genome
            result = process_and_load_genome(genome_data, self.core_api_service)
            
            # Update burst engine with new genome
            burst_engine = self.core_api_service.get_burst_engine()
            if burst_engine and result.get("success"):
                burst_engine.update_with_genome()
                logger.info("Burst Engine updated with new genome", emoji1="⚡")
            
            return GenomeUploadResponse(
                success=True,
                message="Essential genome uploaded successfully",
                genome_number=self.core_api_service.get_genome_counter(),
                loaded=result
            )
            
        except Exception as e:
            logger.error(f"Failed to upload essential genome: {str(e)}", emoji1="❌")
            raise ValueError(f"Failed to upload essential genome: {str(e)}")
    
    @genome_endpoint('POST', '/upload/file', response_model=Dict[str, Any])
    async def upload_genome_file(self, file_data: Dict[str, Any]) -> Dict[str, Any]:
        """Upload a genome file from user's computer."""
        try:
            # Extract file content and metadata
            genome_str = file_data.get('content')
            filename = file_data.get('filename', 'uploaded_genome.json')
            
            if not genome_str:
                raise ValueError("No genome content provided")
            
            # Add default fields if missing
            if "genome_title" not in genome_str:
                genome_str["genome_title"] = filename

            if "genome_description" not in genome_str:
                genome_str["genome_description"] = ""

            # Load the genome
            result = self.core_api_service.load_genome(genome_str, filename=filename)
            
            # Update burst engine if available
            burst_engine = self.core_api_service.get_burst_engine()
            if burst_engine:
                burst_engine.update_with_genome()
                logger.info("Burst Engine updated with new genome", emoji1="⚡")
                
            # Return raw response for v1 compatibility
            return {
                "loaded": result, 
                "genome_counter": self.core_api_service.get_genome_counter()
            }
        except Exception as e:
            logger.error(f"Failed to upload genome file: {str(e)}", emoji1="❌")
            raise ValueError(f"Failed to upload genome file: {str(e)}")
    
    @genome_endpoint('POST', '/upload/string', response_model=Dict[str, Any])
    def upload_genome_string(self, genome: dict) -> Dict[str, Any]:
        """Upload a genome from JSON string."""
        try:
            # Load the genome
            result = self.core_api_service.load_genome(genome)
            
            # Update burst engine
            burst_engine = self.core_api_service.get_burst_engine()
            if burst_engine:
                burst_engine.update_with_genome()
            
            return {
                "loaded": result,
                "genome_counter": self.core_api_service.get_genome_counter()
            }
        except Exception as e:
            logger.error(f"Failed to upload genome string: {str(e)}")
            raise ValueError(f"Failed to upload genome string: {str(e)}")
    
    # ===== Genome Information Endpoints =====
    
    @genome_endpoint('GET', '/file_name', response_model=GenomeFileNameResponse)
    def get_genome_file_name(self) -> GenomeFileNameResponse:
        """Get the current genome file name."""
        try:
            filename = self.core_api_service.get_genome_file_name()
            return GenomeFileNameResponse(file_name=filename)
        except Exception as e:
            logger.error(f"Error getting genome file name: {e}")
            raise ValueError(f"Failed to get genome file name: {str(e)}")
    
    @genome_endpoint('GET', '/genome_number', response_model=GenomeNumberResponse)
    def get_genome_number(self) -> GenomeNumberResponse:
        """Get the current genome number."""
        try:
            number = self.core_api_service.get_genome_counter()
            return GenomeNumberResponse(genome_number=number)
        except Exception as e:
            logger.error(f"Error getting genome number: {e}")
            raise ValueError(f"Failed to get genome number: {str(e)}")
    
    @genome_endpoint('GET', '/download', response_model=GenomeDownloadResponse)
    def download_genome(self) -> GenomeDownloadResponse:
        """Download the current genome."""
        try:
            genome_data = self.core_api_service.get_current_genome()
            filename = self.core_api_service.get_genome_file_name() or "current_genome.json"
            
            return GenomeDownloadResponse(
                genome_data=genome_data,
                filename=filename
            )
        except Exception as e:
            logger.error(f"Error downloading genome: {e}")
            raise ValueError(f"Failed to download genome: {str(e)}")
    
    @genome_endpoint('GET', '/download_region', response_model=GenomeDownloadResponse)
    async def download_genome_from_region(self, region_id: str) -> GenomeDownloadResponse:
        """Download genome data from a specific brain region."""
        try:
            from feagi.bdu.models.brain_region import construct_genome_from_region
            
            # Get connectome
            connectome = self.core_api_service.get_connectome()
            if not connectome:
                raise ValueError("Connectome not available")
            
            # Construct genome from region
            genome_data = construct_genome_from_region(connectome, region_id)
            filename = f"genome_region_{region_id}.json"
            
            return GenomeDownloadResponse(
                genome_data=genome_data,
                filename=filename
            )
        except Exception as e:
            logger.error(f"Error downloading genome from region: {e}")
            raise ValueError(f"Failed to download genome from region: {str(e)}")
    
    @genome_endpoint('GET', '/defaults/files', response_model=GenomeDefaultFilesResponse)
    def get_default_genome_files(self) -> GenomeDefaultFilesResponse:
        """Get list of available default genome files."""
        try:
            files = self.core_api_service.get_default_genome_files()
            return GenomeDefaultFilesResponse(files=files)
        except Exception as e:
            logger.error(f"Error getting default genome files: {e}")
            raise ValueError(f"Failed to get default genome files: {str(e)}")
    
    # ===== Genome Operations =====
    
    @genome_endpoint('POST', '/reset', response_model=SuccessResponse)
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
            raise ValueError(f"Failed to reset genome: {str(e)}")
    
    # ===== Amalgamation Endpoints =====
    
    @genome_endpoint('POST', '/amalgamation_by_payload', response_model=AmalgamationResponse)
    async def amalgamate_by_payload(self, request: AmalgamationRequest) -> AmalgamationResponse:
        """Perform genome amalgamation using payload data."""
        try:
            result = self.core_api_service.process_amalgamation_request(
                genome_payload=request.genome_payload,
                genome_id=request.genome_id,
                genome_title=request.genome_title
            )
            
            return AmalgamationResponse(
                amalgamation_id=result.get("amalgamation_id", ""),
                status="success",
                message="Amalgamation request processed successfully"
            )
        except Exception as e:
            logger.error(f"Error in amalgamation by payload: {e}")
            raise ValueError(f"Failed to process amalgamation: {str(e)}")
    
    @genome_endpoint('POST', '/amalgamation_by_upload', response_model=AmalgamationResponse)
    async def amalgamate_by_upload(self, file_data: Dict[str, Any]) -> AmalgamationResponse:
        """Perform genome amalgamation using uploaded file."""
        try:
            # Extract genome data from uploaded file
            genome_str = file_data.get('content')
            if not genome_str:
                raise ValueError("No file content provided")
            
            result = self.core_api_service.process_amalgamation_request(
                genome_payload=genome_str
            )
            
            return AmalgamationResponse(
                amalgamation_id=result.get("amalgamation_id", ""),
                status="success", 
                message="Amalgamation request processed successfully"
            )
        except Exception as e:
            logger.error(f"Error in amalgamation by upload: {e}")
            raise ValueError(f"Failed to process amalgamation: {str(e)}")
    
    @genome_endpoint('POST', '/amalgamation_by_filename', response_model=AmalgamationResponse)
    async def amalgamate_by_filename(self, request: AmalgamationRequest) -> AmalgamationResponse:
        """Perform genome amalgamation using filename."""
        try:
            result = self.core_api_service.process_amalgamation_request(
                genome_id=request.genome_id,
                genome_title=request.genome_title
            )
            
            return AmalgamationResponse(
                amalgamation_id=result.get("amalgamation_id", ""),
                status="success",
                message="Amalgamation request processed successfully"
            )
        except Exception as e:
            logger.error(f"Error in amalgamation by filename: {e}")
            raise ValueError(f"Failed to process amalgamation: {str(e)}")
    
    @genome_endpoint('GET', '/amalgamation_history', response_model=AmalgamationHistoryResponse)
    def get_amalgamation_history(self) -> AmalgamationHistoryResponse:
        """Get amalgamation history."""
        try:
            history = self.core_api_service.get_amalgamation_history()
            return AmalgamationHistoryResponse(history=history)
        except Exception as e:
            logger.error(f"Error getting amalgamation history: {e}")
            raise ValueError(f"Failed to get amalgamation history: {str(e)}")
    
    @genome_endpoint('GET', '/amalgamation')
    def get_amalgamation(self, amalgamation_id: str) -> Dict[str, Any]:
        """Get specific amalgamation details."""
        try:
            result = self.core_api_service.get_amalgamation_details(amalgamation_id)
            return result
        except Exception as e:
            logger.error(f"Error getting amalgamation: {e}")
            raise ValueError(f"Failed to get amalgamation: {str(e)}")
    
    @genome_endpoint('DELETE', '/amalgamation_cancellation', response_model=SuccessResponse)
    def cancel_amalgamation(self, amalgamation_id: str) -> SuccessResponse:
        """Cancel an amalgamation request."""
        try:
            success = self.core_api_service.cancel_amalgamation(amalgamation_id)
            if success:
                return SuccessResponse(message="Amalgamation cancelled successfully")
            else:
                raise ValueError("Failed to cancel amalgamation")
        except Exception as e:
            logger.error(f"Error cancelling amalgamation: {e}")
            raise ValueError(f"Failed to cancel amalgamation: {str(e)}")
    
    # ===== Template and Circuit Endpoints =====
    
    @genome_endpoint('GET', '/cortical_template', response_model=CorticalTemplateResponse)
    def get_cortical_template(self) -> CorticalTemplateResponse:
        """Get cortical template."""
        try:
            from feagi.evo.templates import cortical_template
            template = cortical_template()
            return CorticalTemplateResponse(template=template)
        except Exception as e:
            logger.error(f"Error getting cortical template: {e}")
            raise ValueError(f"Failed to get cortical template: {str(e)}")
    
    @genome_endpoint('GET', '/circuits', response_model=CircuitLibraryResponse)
    def get_circuit_library(self) -> CircuitLibraryResponse:
        """Get available circuits from the circuit library."""
        try:
            circuits = self.core_api_service.get_circuit_library()
            return CircuitLibraryResponse(circuits=circuits)
        except Exception as e:
            logger.error(f"Error getting circuit library: {e}")
            raise ValueError(f"Failed to get circuit library: {str(e)}")


# ===== Factory Function =====

def create_genome_api(core_api_service: CoreAPIService) -> GenomeAPI:
    """
    Factory function to create a GenomeAPI instance.
    
    This function can be used by transport adapters to get a configured
    GenomeAPI instance with the required dependencies.
    """
    return GenomeAPI(core_api_service) 