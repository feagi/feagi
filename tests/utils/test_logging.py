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
Tests for FEAGI logging utilities.
"""

import logging
import os
import tempfile
from unittest.mock import MagicMock, patch

import pytest

from feagi.utils.logger import setup_logger


def test_configure_logger_console_only():
    """Test logger configuration with console output only."""
    logger = setup_logger(
        name="test_console", level=logging.INFO, log_file=None, console=True
    )
    assert logger.name == "test_console"
    assert len(logger.handlers) == 1  # Only console handler


def test_configure_logger_with_file():
    """Test logger configuration with both console and file output."""
    with tempfile.TemporaryDirectory() as temp_dir:
        log_file = os.path.join(temp_dir, "test.log")
        logger = setup_logger(
            name="test_file", level=logging.DEBUG, log_file=log_file, console=False
        )
        assert logger.name == "test_file"
        assert len(logger.handlers) == 1  # Only file handler when console=False
        assert os.path.exists(log_file)


def test_get_logger():
    """Test retrieving a logger instance."""
    logger1 = logging.getLogger("test_get")
    logger2 = logging.getLogger("test_get")

    # Should return the same logger instance
    assert logger1 is logger2
    assert logger1.name == "test_get"


# Add more test functions for other utility modules as needed
