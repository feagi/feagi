"""
Device registration contract validation tests for BrainOutput.
"""

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
