"""FEAGI v1 Training API"""
from typing import Dict, Any
from feagi.api.core.services.core_api_service import CoreAPIService
from .schemas import TrainingStatusResponse, TrainingConfigRequest, TrainingStatsResponse, SuccessResponse
from .decorators import endpoint

def training_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(methods=methods, path=path, request_model=request_model, response_model=response_model, description=description, module='training')

class TrainingAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
    
    @training_endpoint('GET', '/status', response_model=TrainingStatusResponse)
    async def get_training_status(self) -> TrainingStatusResponse:
        status = self.core_api_service.get_training_status()
        return TrainingStatusResponse(status=status.get('status', 'unknown'), progress=status.get('progress'), config=status.get('config'))
    
    @training_endpoint('POST', '/configure', request_model=TrainingConfigRequest, response_model=SuccessResponse)
    async def configure_training(self, request: TrainingConfigRequest) -> SuccessResponse:
        success = self.core_api_service.configure_training(request.config)
        if not success:
            raise ValueError("Failed to configure training")
        return SuccessResponse(message="Training configured successfully")
    
    @training_endpoint('GET', '/stats', response_model=TrainingStatsResponse)
    async def get_training_stats(self) -> TrainingStatsResponse:
        stats = self.core_api_service.get_training_stats()
        return TrainingStatsResponse(stats=stats)

def create_training_api(core_api_service: CoreAPIService) -> TrainingAPI:
    return TrainingAPI(core_api_service) 