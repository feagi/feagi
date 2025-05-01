"""
Unit tests for the Rust implementation of ConnectomeManager.

These tests verify the behavior of the Rust implementation directly,
not through the Python bindings.
"""

import pytest
import numpy as np
import subprocess
import os
import sys
from pathlib import Path


# Skip all tests if Rust is not available
try:
    # Check if cargo is available (indicating Rust is installed)
    subprocess.run(["cargo", "--version"], check=True, capture_output=True)
    RUST_AVAILABLE = True
except (subprocess.SubprocessError, FileNotFoundError):
    RUST_AVAILABLE = False

pytestmark = pytest.mark.skipif(not RUST_AVAILABLE, reason="Rust toolchain not available")


@pytest.fixture(scope="session")
def rust_test_dir():
    """Path to the Rust test directory."""
    return Path(__file__).parent / "../../../../../rust/feagi/tests"


@pytest.fixture(scope="session")
def build_rust_tests(rust_test_dir):
    """Build the Rust test executable."""
    if not RUST_AVAILABLE:
        pytest.skip("Rust toolchain not available")
    
    # Change to the Rust project directory and build the tests
    cwd = os.getcwd()
    try:
        os.chdir(rust_test_dir.parent)  # Go to the Rust project root
        result = subprocess.run(
            ["cargo", "test", "--no-run", "--package", "feagi", "--lib", "bdu::connectome"],
            check=True, capture_output=True
        )
        return True
    except subprocess.SubprocessError as e:
        pytest.skip(f"Failed to build Rust tests: {e}")
        return False
    finally:
        os.chdir(cwd)  # Return to original directory


@pytest.mark.rust
def test_rust_connectome_creation(build_rust_tests, rust_test_dir):
    """Test the Rust ConnectomeManager creation."""
    if not build_rust_tests:
        pytest.skip("Rust tests not built successfully")
    
    # Run the specific Rust test for connectome creation
    cwd = os.getcwd()
    try:
        os.chdir(rust_test_dir.parent)  # Go to the Rust project root
        result = subprocess.run(
            ["cargo", "test", "--package", "feagi", "--lib", "bdu::connectome::tests::test_create_connectome", "--", "--nocapture"],
            check=False, capture_output=True, text=True
        )
        # Check if the test passed
        assert result.returncode == 0, f"Rust test failed: {result.stderr}"
        assert "test bdu::connectome::tests::test_create_connectome ... ok" in result.stdout
    finally:
        os.chdir(cwd)  # Return to original directory


@pytest.mark.rust
def test_rust_area_creation(build_rust_tests, rust_test_dir):
    """Test cortical area creation in Rust ConnectomeManager."""
    if not build_rust_tests:
        pytest.skip("Rust tests not built successfully")
    
    # Run the specific Rust test for area creation
    cwd = os.getcwd()
    try:
        os.chdir(rust_test_dir.parent)  # Go to the Rust project root
        result = subprocess.run(
            ["cargo", "test", "--package", "feagi", "--lib", "bdu::connectome::tests::test_add_cortical_area", "--", "--nocapture"],
            check=False, capture_output=True, text=True
        )
        # Check if the test passed
        assert result.returncode == 0, f"Rust test failed: {result.stderr}"
        assert "test bdu::connectome::tests::test_add_cortical_area ... ok" in result.stdout
    finally:
        os.chdir(cwd)  # Return to original directory


@pytest.mark.rust
def test_rust_neuron_creation(build_rust_tests, rust_test_dir):
    """Test neuron creation in Rust ConnectomeManager."""
    if not build_rust_tests:
        pytest.skip("Rust tests not built successfully")
    
    # Run the specific Rust test for neuron creation
    cwd = os.getcwd()
    try:
        os.chdir(rust_test_dir.parent)  # Go to the Rust project root
        result = subprocess.run(
            ["cargo", "test", "--package", "feagi", "--lib", "bdu::connectome::tests::test_create_neuron", "--", "--nocapture"],
            check=False, capture_output=True, text=True
        )
        # Check if the test passed
        assert result.returncode == 0, f"Rust test failed: {result.stderr}"
        assert "test bdu::connectome::tests::test_create_neuron ... ok" in result.stdout
    finally:
        os.chdir(cwd)  # Return to original directory


@pytest.mark.rust
def test_rust_synapse_creation(build_rust_tests, rust_test_dir):
    """Test synapse creation in Rust ConnectomeManager."""
    if not build_rust_tests:
        pytest.skip("Rust tests not built successfully")
    
    # Run the specific Rust test for synapse creation
    cwd = os.getcwd()
    try:
        os.chdir(rust_test_dir.parent)  # Go to the Rust project root
        result = subprocess.run(
            ["cargo", "test", "--package", "feagi", "--lib", "bdu::connectome::tests::test_create_synapse", "--", "--nocapture"],
            check=False, capture_output=True, text=True
        )
        # Check if the test passed
        assert result.returncode == 0, f"Rust test failed: {result.stderr}"
        assert "test bdu::connectome::tests::test_create_synapse ... ok" in result.stdout
    finally:
        os.chdir(cwd)  # Return to original directory


@pytest.mark.rust
def test_rust_membrane_update(build_rust_tests, rust_test_dir):
    """Test membrane potential updates in Rust ConnectomeManager."""
    if not build_rust_tests:
        pytest.skip("Rust tests not built successfully")
    
    # Run the specific Rust test for membrane updates
    cwd = os.getcwd()
    try:
        os.chdir(rust_test_dir.parent)  # Go to the Rust project root
        result = subprocess.run(
            ["cargo", "test", "--package", "feagi", "--lib", "bdu::connectome::tests::test_update_membrane_potentials", "--", "--nocapture"],
            check=False, capture_output=True, text=True
        )
        # Check if the test passed
        assert result.returncode == 0, f"Rust test failed: {result.stderr}"
        assert "test bdu::connectome::tests::test_update_membrane_potentials ... ok" in result.stdout
    finally:
        os.chdir(cwd)  # Return to original directory 