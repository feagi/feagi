import pytest

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.v1.physiology import create_physiology_api


class DummyGenomeService:
    def __init__(self):
        self._genome = {"physiology": {"simulation_timestep": 0.05}}

    def get_genome(self):
        return self._genome

    def update_physiology(self, updates):
        self._genome.setdefault("physiology", {}).update(updates)
        return True


class DummyCoreAPI(CoreAPIService):
    def __init__(self):
        # Bypass BaseService init
        self._genome_service = DummyGenomeService()

    def get_current_genome(self):
        return self._genome_service.get_genome()

    def update_genome_physiology(self, updates):
        return self._genome_service.update_physiology(updates)


@pytest.mark.asyncio
async def test_get_and_update_physiology_endpoints():
    core = DummyCoreAPI()
    api = create_physiology_api(core)

    # GET
    resp = await api.get_physiology()
    assert "physiology" in resp
    assert resp["physiology"]["simulation_timestep"] == 0.05

    # PUT update sleep thresholds
    upd = {
        "physiology": {
            "sleep_trigger_inactivity_window": 7,
            "sleep_trigger_neural_activity_max": 1234,
        }
    }
    result = await api.update_physiology(upd)  # type: ignore[arg-type]
    assert result["success"] is True
    assert result["updated"]["sleep_trigger_inactivity_window"] == 7

    # GET reflect update
    resp2 = await api.get_physiology()
    assert resp2["physiology"]["sleep_trigger_inactivity_window"] == 7
    assert resp2["physiology"]["sleep_trigger_neural_activity_max"] == 1234 