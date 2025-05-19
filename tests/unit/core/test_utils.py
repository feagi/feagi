"""
Tests for utility functions in the FEAGI codebase.
"""

import os
import tempfile
import logging
import pytest
from unittest.mock import patch, MagicMock

from feagi.utils.logger import setup_logger, EmojiAdapter


def test_setup_logger_console_only():
    """Test logger configuration with console output only."""
    logger_adapter = setup_logger(name="test_console", level=20, console=True, log_file=None)
    assert isinstance(logger_adapter, EmojiAdapter)
    assert logger_adapter.logger.name == "test_console"
    assert len(logger_adapter.logger.handlers) == 1  # Only console handler


def test_setup_logger_with_file():
    """Test logger configuration with both console and file output."""
    with tempfile.TemporaryDirectory() as temp_dir:
        log_file = os.path.join(temp_dir, "test.log")
        logger_adapter = setup_logger(
            name="test_file", 
            level=10,  
            log_file=log_file,
            console=True
        )
        assert isinstance(logger_adapter, EmojiAdapter)
        assert logger_adapter.logger.name == "test_file"
        assert len(logger_adapter.logger.handlers) == 2  # Console and file handlers
        assert os.path.exists(log_file)


def test_logger_reuse():
    """Test retrieving a logger instance."""
    logger_adapter1 = setup_logger("test_reuse")
    logger2 = logging.getLogger("test_reuse")
    
    # Should return the same underlying logger instance
    assert logger_adapter1.logger is logger2
    assert logger_adapter1.logger.name == "test_reuse"


# Add more test functions for other utility modules as needed 