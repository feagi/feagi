import json
import os
from pathlib import Path

import pytest
import asyncio

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.core.state_manager import FeagiStateManager
from feagi.evo.genome_editor import save_genome
from feagi.evo.genome_processor import genome_v1_v2_converter


@pytest.mark.integration
def test_essential_genome_save_roundtrip(tmp_path: Path, capsys):
    # Initialize CoreAPIService with a fresh ConnectomeManager and state
    state_manager = FeagiStateManager.instance()
    cm = ConnectomeManager.instance()
    api = CoreAPIService(connectome_manager=cm, state_manager=state_manager)

    # Load essential genome using v1 API service entry point
    from feagi.api.v1.genome import create_genome_api

    genome_api = create_genome_api(api)

    # Upload essential genome (hierarchical working format will be stored in state)
    upload_resp = asyncio.run(genome_api.upload_essential_genome())
    assert getattr(upload_resp, "success", False)

    # Fetch current genome from the service (hierarchical format)
    original = api.get_current_genome()
    assert isinstance(original, dict) and "blueprint" in original

    # Convert hierarchical (v1) genome dict to v2 flat blueprint using provided converter
    flat_doc = genome_v1_v2_converter({"blueprint": original["blueprint"]})
    assert isinstance(flat_doc, dict) and "blueprint" in flat_doc

    # Prepare flat genome document for saving
    flat_document = {
        "version": original.get("version", "2.0"),
        "blueprint": flat_doc["blueprint"],
        "physiology": original.get("physiology", {}),
        "neuron_morphologies": original.get("neuron_morphologies", {}),
    }

    # Save to a temp file as JSON
    out_file = tmp_path / "roundtrip_flat_genome.json"
    save_genome(genome=flat_document, file_name=str(out_file))
    assert out_file.exists()

    # Reload saved genome
    with out_file.open("r") as f:
        saved = json.load(f)

    # Expected: same as flat_document except dynamic fields
    expected_flat = json.loads(json.dumps(flat_document))

    # Remove known dynamic fields introduced by save_genome
    for doc in (saved, expected_flat):
        doc.pop("timestamp", None)
        doc.pop("signatures", None)

    # Print and compare differences with controlled ignores
    diffs = _diff_json(expected_flat, saved)
    if diffs:
        print("Genome roundtrip differences:")
        for d in diffs:
            print(d)
    assert not diffs, "Saved genome differs from expected flat representation"


def _diff_json(a, b, path_prefix="$"):
    """Return list of human-readable diffs between two JSON-like dicts/lists/scalars."""
    diffs = []
    if type(a) != type(b):
        diffs.append(f"{path_prefix}: type mismatch {type(a).__name__} != {type(b).__name__}")
        return diffs
    if isinstance(a, dict):
        a_keys = set(a.keys())
        b_keys = set(b.keys())
        for k in sorted(a_keys - b_keys):
            diffs.append(f"{path_prefix}.{k}: missing in saved")
        for k in sorted(b_keys - a_keys):
            diffs.append(f"{path_prefix}.{k}: unexpected in saved")
        for k in sorted(a_keys & b_keys):
            diffs.extend(_diff_json(a[k], b[k], f"{path_prefix}.{k}"))
    elif isinstance(a, list):
        if len(a) != len(b):
            diffs.append(f"{path_prefix}: list length {len(a)} != {len(b)}")
            n = min(len(a), len(b))
        else:
            n = len(a)
        for i in range(n):
            diffs.extend(_diff_json(a[i], b[i], f"{path_prefix}[{i}]"))
    else:
        if a != b:
            diffs.append(f"{path_prefix}: {a!r} != {b!r}")
    return diffs 