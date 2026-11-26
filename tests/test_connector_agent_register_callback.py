"""Rust ConnectorAgent exposes `register_callback` used by brain_output motors."""

import pytest

feagi_rust_py_libs = pytest.importorskip("feagi_rust_py_libs")


def test_connector_agent_exposes_register_callback() -> None:
    """Fails on older wheels; required for RotaryMotor/ServoMotor brain_output wiring."""
    agent_cls = feagi_rust_py_libs.connector_core.ConnectorAgent
    assert hasattr(
        agent_cls, "register_callback"
    ), "ConnectorAgent.register_callback missing (upgrade feagi-rust-py-libs)"

    agent = agent_cls()
    assert callable(getattr(agent, "register_callback"))


def test_motor_cortical_unit_enum_names() -> None:
    """Motor outputs use genomic.MotorCorticalUnit variants (not legacy MotorCorticalType)."""
    from feagi_rust_py_libs.data_structures.genomic import MotorCorticalUnit

    assert hasattr(MotorCorticalUnit, "RotaryMotor")
    assert hasattr(MotorCorticalUnit, "PositionalServo")
