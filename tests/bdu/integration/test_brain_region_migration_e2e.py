"""
End-to-end test: mapping + area relocation preserves properties and updates region I/O.

Validates that:
- cortical_destinations are preserved after relocation
- parent_region_id is synchronized on cortical properties
- region areas/inputs/outputs reflect actual connections
"""

import pytest
from typing import Dict, Any

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.core.state_manager import FeagiStateManager
from feagi.utils.config import FeagiConfig


@pytest.fixture
def core(core_api_setup):
    return core_api_setup


@pytest.fixture
def core_api_setup():
    config = FeagiConfig()
    state = FeagiStateManager.instance()
    cm = ConnectomeManager(config, state)
    svc = CoreAPIService(connectome_manager=cm, state_manager=state)
    return svc


def _create_custom_area(api: CoreAPIService, name: str, coords: list[int], dims: list[int]) -> str:
    payload = {
        "cortical_name": name,
        "parent_region_id": "root",
        "sub_group_id": "TEST",
        "cortical_dimensions": dims,
        "coordinates_3d": coords,
        "cortical_visibility": True,
        "cortical_synaptic_attractivity": 100,
    }
    res = api.create_cortical_area(
        name=name,
        coordinates={"x": coords[0], "y": coords[1], "z": coords[2]},
        dimensions={"width": dims[0], "height": dims[1], "depth": dims[2]},
        area_type="custom",
        parameters={k: v for k, v in payload.items() if k not in {"cortical_name", "coordinates_3d", "cortical_dimensions"}},
    )
    # API returns with key 'cortical_id'
    assert res and res.get("cortical_id")
    return res["cortical_id"]


def _set_mapping(api: CoreAPIService, src_id: str, dst_id: str) -> None:
    mapping = [
        {
            "morphology_id": "block_to_block",
            "morphology_scalar": [1, 1, 1],
            "plasticity_flag": False,
            "postSynapticCurrent_multiplier": 1.0,
        }
    ]
    ok = api.update_cortical_mapping_properties(src_id, dst_id, mapping)
    assert ok is True


def _get_props(api: CoreAPIService, area_id: str) -> Dict[str, Any]:
    # Route via CorticalAreaService.get_area (exposes connectome properties)
    res = api.get_cortical_area(area_id)
    assert res and "parameters" in res
    return res["parameters"]


def _region(api: CoreAPIService, region_id: str) -> Dict[str, Any]:
    # Use get_brain_regions and select
    regions = api.get_brain_regions()
    for r in regions:
        if r.get("region_id") == region_id:
            return r
    raise AssertionError(f"Region {region_id} not found")


def test_mapping_then_relocate_preserves_properties_and_updates_region(core: CoreAPIService):
    # Ensure minimal genome is loaded
    load = core.load_essential_genome()
    assert load.get("success", True) is True

    # Create destination region rX
    rid = "region_e2e"
    core.create_brain_region(
        region_id=rid,
        region_name="rX",
        parent_region_id="root",
        coordinates={"x": -10, "y": 0, "z": 0},
    )

    # Create two areas: A (outside), B (inside rX)
    a_id = _create_custom_area(core, "A_ext", [0, 0, 0], [2, 2, 1])
    b_id = _create_custom_area(core, "B_in", [10, 0, 0], [2, 2, 1])

    # Move B into rX
    ok = core.change_cortical_area_parent(b_id, rid)
    assert ok is True

    # Create mapping A -> B so B should be input for rX
    _set_mapping(core, a_id, b_id)

    # Sanity: Source area parameters must include mapping to dest area
    b_props = _get_props(core, a_id)
    assert "mapping" in b_props

    # Relocate B to rX again (idempotent) and validate no data loss
    ok2 = core.change_cortical_area_parent(b_id, rid)
    assert ok2 is True

    # Validate mapping still present for A (source)
    a_props = _get_props(core, a_id)
    assert "mapping" in a_props
    assert b_id in a_props["mapping"]

    # Validate B parent_region_id synced
    b_props2 = _get_props(core, b_id)
    assert b_props2.get("parent_region_id") == rid

    # Validate region membership and IO
    reg = _region(core, rid)
    areas = set(reg.get("areas", []))
    inputs = set(reg.get("inputs", []))
    outputs = set(reg.get("outputs", []))

    assert b_id in areas
    assert b_id in inputs
    assert b_id not in outputs


