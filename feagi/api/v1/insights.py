"""FEAGI v1 Insights API"""
from typing import Dict, Any
from feagi.api.core.services.core_api_service import CoreAPIService
from .schemas import InsightsDataResponse, AnalyticsResponse
from .decorators import endpoint

def insights_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(methods=methods, path=path, request_model=request_model, response_model=response_model, description=description, module='insights')

class InsightsAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
    
    @insights_endpoint('GET', '/data', response_model=InsightsDataResponse)
    async def get_insights_data(self) -> InsightsDataResponse:
        insights = self.core_api_service.get_insights_data()
        return InsightsDataResponse(insights=insights)
    
    @insights_endpoint('GET', '/analytics', response_model=AnalyticsResponse)
    async def get_analytics(self) -> AnalyticsResponse:
        analytics = self.core_api_service.get_analytics_data()
        return AnalyticsResponse(analytics=analytics)

def create_insights_api(core_api_service: CoreAPIService) -> InsightsAPI:
    return InsightsAPI(core_api_service) 