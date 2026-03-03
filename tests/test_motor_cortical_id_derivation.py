"""Test motor and sensory cortical ID derivation from device registrations."""

import pytest

frpl = pytest.importorskip("feagi_rust_py_libs", reason="feagi_rust_py_libs required")


@pytest.mark.skipif(
    not hasattr(frpl.connector_core.ConnectorAgent, "get_motor_cortical_ids_for_verification"),
    reason="ConnectorAgent.get_motor_cortical_ids_for_verification not available (upgrade feagi-rust-py-libs)",
)
def test_get_motor_cortical_ids_for_verification_positional_servo() -> None:
    """ConnectorAgent derives cortical IDs matching FEAGI auto-create."""
    agent = frpl.connector_core.ConnectorAgent()
    fh = frpl.data_structures.genomic.cortical_area.FrameChangeHandling.Absolute()
    pos = frpl.data_structures.genomic.cortical_area.PercentageNeuronPositioning.Linear()

    agent.motor_positional_servo_register(
        group=0,
        number_channels=3,
        frame_change_handling=fh,
        z_neuron_resolution=10,
        percentage_neuron_positioning=pos,
    )
    agent.motor_positional_servo_register(
        group=1,
        number_channels=3,
        frame_change_handling=fh,
        z_neuron_resolution=10,
        percentage_neuron_positioning=pos,
    )

    ids = agent.get_motor_cortical_ids_for_verification()
    assert isinstance(ids, list)
    assert len(ids) >= 2
    # PositionalServo has 2 cortical areas per group (Absolute + Incremental)
    # Groups 0 and 1 -> 4 areas minimum
    assert len(ids) >= 4
    for cid in ids:
        assert isinstance(cid, str)
        assert len(cid) > 0
    # Expected: b3BzZQUAAAA= (group 0 abs), b3BzZQUAAAE= (group 1 abs), etc.
    assert "b3BzZQUAAAA=" in ids
    assert "b3BzZQUAAAE=" in ids


@pytest.mark.skipif(
    not hasattr(frpl.connector_core.ConnectorAgent, "get_sensory_cortical_ids_for_verification"),
    reason="ConnectorAgent.get_sensory_cortical_ids_for_verification not available (upgrade feagi-rust-py-libs)",
)
def test_get_sensory_cortical_ids_for_verification_digital_gpio() -> None:
    """ConnectorAgent derives sensory cortical IDs matching FEAGI auto-create."""
    agent = frpl.connector_core.ConnectorAgent()
    agent.sensor_DigitalGPIO_register(group=0, number_channels=2)
    agent.sensor_DigitalGPIO_register(group=1, number_channels=2)

    ids = agent.get_sensory_cortical_ids_for_verification()
    assert isinstance(ids, list)
    assert len(ids) >= 2
    for cid in ids:
        assert isinstance(cid, str)
        assert len(cid) > 0
