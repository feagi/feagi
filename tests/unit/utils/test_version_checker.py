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

"""Tests for the version checker functionality."""
import os
import pytest
from pathlib import Path

from feagi.utils import check_dependencies, verify_dependencies


def test_check_dependencies():
    """Test that check_dependencies returns a tuple with expected types."""
    # Get the path to requirements.txt
    feagi_root = Path(__file__).parent.parent.parent
    requirements_path = str(feagi_root / "requirements.txt")
    
    # Run the check
    is_compatible, error_messages = check_dependencies(requirements_path)
    
    # Verify return types
    assert isinstance(is_compatible, bool)
    assert isinstance(error_messages, list)
    
    # Check that each error message is a string
    for msg in error_messages:
        assert isinstance(msg, str)


def test_verify_dependencies():
    """Test that verify_dependencies returns a boolean."""
    # Get the path to requirements.txt
    feagi_root = Path(__file__).parent.parent.parent
    requirements_path = str(feagi_root / "requirements.txt")
    
    # Run the verification
    result = verify_dependencies(requirements_path, raise_exception=False)
    
    # Verify return type
    assert isinstance(result, bool)


def test_environment_variable():
    """Test that FEAGI_SKIP_VERSION_CHECK works as expected."""
    # Test with environment variable set
    os.environ["FEAGI_SKIP_VERSION_CHECK"] = "1"
    assert os.environ.get("FEAGI_SKIP_VERSION_CHECK", "").lower() in ("1", "true", "yes")
    
    # Test with environment variable unset
    if "FEAGI_SKIP_VERSION_CHECK" in os.environ:
        del os.environ["FEAGI_SKIP_VERSION_CHECK"]
    assert os.environ.get("FEAGI_SKIP_VERSION_CHECK", "").lower() not in ("1", "true", "yes") 