"""
Device registration contract validation tests for BrainOutput.
"""

import json

from feagi.pns.brain_output import BrainOutput


def _valid_motor_registration() -> dict:
    return {
        "output_units_and_decoder_properties": {
            "PositionalServo": [
                [
                    {
                        "friendly_name": "front_left_leg",
                        "cortical_unit_index": 0,
                        "device_grouping": [
                            {
                                "friendly_name": "front_left_hip",
                                "device_properties": {
                                    "bundle_type": "leg",
                                    "bundle_id": "front_left_leg",
                                    "modality": "motor",
                                    "signal_type": "positional_servo",
                                    "source_model": "/tmp/model.xml",
                                    "source_entity": "hip_joint",
                                },
                            }
                        ],
                    },
                    {},
                ]
            ]
        },
        "feedbacks": {},
    }


def test_device_registration_contract_accepts_valid_payload() -> None:
    manager = BrainOutput()
    payload = _valid_motor_registration()
    validated = manager._validate_device_registration_contract(payload)
    assert validated["output_units_and_decoder_properties"]["PositionalServo"][0][0][
        "friendly_name"
    ] == "front_left_leg"


def test_device_registration_contract_defaults_fill_rust_minimal_export() -> None:
    """Programmatic Rust cache export may use null names and omit device_properties keys."""
    manager = BrainOutput()
    payload: dict = {
        "output_units_and_decoder_properties": {
            "RotaryMotor": [
                [
                    {
                        "friendly_name": None,
                        "cortical_unit_index": 0,
                        "io_configuration_flags": {},
                        "device_grouping": [
                            {
                                "friendly_name": None,
                                "device_properties": {},
                            },
                        ],
                    },
                    {},
                ]
            ]
        },
    }
    filled = BrainOutput._apply_device_registration_contract_defaults(payload)
    validated = manager._validate_device_registration_contract(filled)
    unit = validated["output_units_and_decoder_properties"]["RotaryMotor"][0][0]
    assert unit["friendly_name"] == "RotaryMotor_0"
    assert unit["device_grouping"][0]["friendly_name"] == "RotaryMotor_0_ch0"
    props = unit["device_grouping"][0]["device_properties"]
    assert props["bundle_type"] == {"type": "String", "value": "unspecified"}
    assert props["source_entity"] == {"type": "String", "value": "unspecified"}


def test_device_registration_contract_rejects_missing_required_fields() -> None:
    manager = BrainOutput()
    payload = _valid_motor_registration()
    unit_def = payload["output_units_and_decoder_properties"]["PositionalServo"][0][0]
    unit_def.pop("friendly_name")
    unit_def["device_grouping"][0]["device_properties"].pop("bundle_id")

    try:
        manager._validate_device_registration_contract(payload)
    except RuntimeError as exc:
        message = str(exc)
        assert "friendly_name" in message
        assert "bundle_id" in message
        return
    assert False, "Expected missing required fields to raise RuntimeError"


def test_device_registration_enricher_hook_updates_payload() -> None:
    manager = BrainOutput()
    payload = _valid_motor_registration()

    def enricher(data: dict) -> dict:
        unit_def = data["output_units_and_decoder_properties"]["PositionalServo"][0][0]
        unit_def["friendly_name"] = "front_right_leg"
        return data

    manager.set_device_registration_enricher(enricher)
    updated = manager._device_registration_enricher(payload.copy())
    validated = manager._validate_device_registration_contract(updated)
    assert validated["output_units_and_decoder_properties"]["PositionalServo"][0][0][
        "friendly_name"
    ] == "front_right_leg"


def _run_export_pipeline(manager: BrainOutput, exported: dict) -> dict:
    """Run an exported registration through the same steps ``connect`` applies before sending."""
    registrations = manager._normalize_device_registration_properties(exported)
    registrations = manager._apply_device_registration_contract_defaults(registrations)
    registrations = manager._validate_device_registration_contract(registrations)
    # connect() sends json.dumps(device_regs, sort_keys=True); the wire form must round trip.
    return json.loads(json.dumps(registrations, sort_keys=True))


def test_physical_range_on_each_device_grouping_channel_survives_the_export_pipeline() -> None:
    manager = BrainOutput()
    payload = _valid_motor_registration()
    unit_def = payload["output_units_and_decoder_properties"]["PositionalServo"][0][0]
    template_channel = unit_def["device_grouping"][0]
    ranges = [
        {"value_at_0": -1.57, "value_at_1": 1.57, "unit": "rad"},
        {"value_at_0": 0.0, "value_at_1": 0.25, "unit": "m"},
    ]
    unit_def["device_grouping"] = []
    for idx, physical_range in enumerate(ranges):
        channel = json.loads(json.dumps(template_channel))
        channel["friendly_name"] = f"channel_{idx}"
        channel["physical_range"] = physical_range
        unit_def["device_grouping"].append(channel)
    undeclared = json.loads(json.dumps(template_channel))
    undeclared["friendly_name"] = "channel_without_range"
    unit_def["device_grouping"].append(undeclared)

    sent = _run_export_pipeline(manager, payload)

    channels = sent["output_units_and_decoder_properties"]["PositionalServo"][0][0]["device_grouping"]
    assert [channel.get("physical_range") for channel in channels[:2]] == ranges
    assert "physical_range" not in channels[2], "a channel without a declared range must stay without one"


def test_physical_range_survives_the_enricher_hook() -> None:
    manager = BrainOutput()
    payload = _valid_motor_registration()
    range_ = {"value_at_0": 0.0, "value_at_1": 1.0, "unit": "rad"}
    payload["output_units_and_decoder_properties"]["PositionalServo"][0][0]["device_grouping"][0][
        "physical_range"
    ] = range_

    def enricher(data: dict) -> dict:
        data["output_units_and_decoder_properties"]["PositionalServo"][0][0]["friendly_name"] = "renamed"
        return data

    manager.set_device_registration_enricher(enricher)
    enriched = manager._device_registration_enricher(payload)
    sent = _run_export_pipeline(manager, enriched)

    unit_def = sent["output_units_and_decoder_properties"]["PositionalServo"][0][0]
    assert unit_def["friendly_name"] == "renamed"
    assert unit_def["device_grouping"][0]["physical_range"] == range_
