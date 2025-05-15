"""
Tests for the Cap'n Proto schema definitions.

These tests verify that the Cap'n Proto schemas can be loaded.
"""

import os
import pytest
import capnp


@pytest.fixture
def schema_path():
    """Path to the Cap'n Proto schema files."""
    # Go up three directories from current file to get to project root, then to feagi_capnp
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    return os.path.join(project_root, "feagi_capnp")


def test_load_common_schema(schema_path):
    """Test that we can load the common schema."""
    schema = capnp.load(os.path.join(schema_path, "common/constants.capnp"))
    assert schema is not None


def test_load_handshake_schema(schema_path):
    """Test that we can load the handshake schema."""
    schema = capnp.load(os.path.join(schema_path, "handshake/v1/handshake.capnp"))
    assert schema is not None


def test_load_fcp_schema(schema_path):
    """Test that we can load the FCP schema."""
    schema = capnp.load(os.path.join(schema_path, "fcp/v1/fcp.capnp"))
    assert schema is not None


def test_load_fsmp_schema(schema_path):
    """Test that we can load the FSMP schema."""
    schema = capnp.load(os.path.join(schema_path, "fsmp/v1/fsmp.capnp"))
    assert schema is not None


def test_load_fvp_schema(schema_path):
    """Test that we can load the FVP schema."""
    schema = capnp.load(os.path.join(schema_path, "fvp/v1/fvp.capnp"))
    assert schema is not None 