"""
FEAGI v1 Physiology API - Single Source of Truth

Endpoints to read/update physiology parameters in the active genome, including
sleep trigger fields used by Sleep Manager.
"""
from typing import Any, Dict

from pydantic import BaseModel

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

from .decorators import endpoint

logger = setup_logger(__name__)


class PhysiologyUpdateRequest(BaseModel):
    """Payload to update physiology parameters."""

    physiology: Dict[str, Any]


def physiology_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="physiology",
    )


class PhysiologyAPI:
    def __init__(self, core_api_service: CoreAPIService) -> None:
        self.core_api_service = core_api_service

    @physiology_endpoint("GET", "/")
    async def get_physiology(self) -> Dict[str, Any]:
        """Return current physiology section from genome (with defaults applied)."""
        try:
            genome = self.core_api_service.get_current_genome()
            phys = {}
            if isinstance(genome, dict):
                phys = genome.get("physiology", {}) or {}
            return {"physiology": phys}
        except Exception as e:
            logger.error(f"Failed to get physiology: {e}")
            raise ValueError(f"Failed to get physiology: {str(e)}")

    @physiology_endpoint("PUT", "/", request_model=PhysiologyUpdateRequest)
    async def update_physiology(self, request: PhysiologyUpdateRequest) -> Dict[str, Any]:
        """Update physiology parameters in the active genome and persist to state.
        Only whitelisted keys are accepted.
        """
        try:
            updates = dict(request.physiology or {})
            # Whitelist keys (extendable)
            allowed = {
                "simulation_timestep",
                "max_age",
                "evolution_burst_count",
                "ipu_idle_threshold",
                "plasticity_queue_depth",
                "lifespan_mgmt_interval",
                "sleep_trigger_inactivity_window",
                "sleep_trigger_neural_activity_max",
            }
            filtered = {k: v for k, v in updates.items() if k in allowed}
            if not filtered:
                return {"success": False, "updated": {}}

            # Delegate to core service to apply and revalidate genome
            success = self.core_api_service.update_genome_physiology(filtered)
            return {"success": bool(success), "updated": filtered}
        except Exception as e:
            logger.error(f"Failed to update physiology: {e}")
            raise ValueError(f"Failed to update physiology: {str(e)}")


def create_physiology_api(core_api_service: CoreAPIService) -> PhysiologyAPI:
    return PhysiologyAPI(core_api_service) 