"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Unit tests for the Rust implementation of ConnectomeManager.

These tests verify the behavior of the Rust implementation directly,
not through the Python bindings.
"""

import os
import subprocess
from pathlib import Path

import pytest

# Skip all tests if Rust is not available
try:
    # Check if cargo is available (indicating Rust is installed)
    subprocess.run(["cargo", "--version"], check=True, capture_output=True)
    RUST_AVAILABLE = True
except (subprocess.SubprocessError, FileNotFoundError):
    RUST_AVAILABLE = False

pytestmark = pytest.mark.skipif(
    not RUST_AVAILABLE, reason="Rust toolchain not available"
)


@pytest.fixture(scope="session")
def rust_test_dir():
    """Path to the Rust test directory (feagi-rust)."""
    return Path(__file__).parent.parent.parent.parent.parent / "feagi-rust"


@pytest.fixture(scope="session")
def build_rust_tests(rust_test_dir):
    """Build the Rust test executable."""
    if not RUST_AVAILABLE:
        pytest.skip("Rust toolchain not available")

    # Change to the Rust project directory and build the tests
    cwd = os.getcwd()
    try:
        os.chdir(rust_test_dir)  # Go to the Rust project root
        result = subprocess.run(
            ["cargo", "test", "--no-run"], check=True, capture_output=True
        )
        return True
    except subprocess.SubprocessError as e:
        pytest.skip(f"Failed to build Rust tests: {e}")
        return False
    finally:
        os.chdir(cwd)  # Return to original directory


@pytest.mark.rust
def test_rust_all(build_rust_tests, rust_test_dir):
    """Run all Rust tests in the feagi-rust package and assert all pass."""
    if not build_rust_tests:
        pytest.skip("Rust tests not built successfully")
    cwd = os.getcwd()
    try:
        os.chdir(rust_test_dir)
        result = subprocess.run(
            ["cargo", "test", "--", "--nocapture"],
            check=False,
            capture_output=True,
            text=True,
        )
        print(result.stdout)
        assert result.returncode == 0, (
            f"Rust tests failed:\n{result.stdout}\n{result.stderr}"
        )
    finally:
        os.chdir(cwd)
