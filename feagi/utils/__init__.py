"""Utilities for FEAGI.

This package contains various utility functions and classes used throughout FEAGI."""

from feagi.utils.logger import setup_logger
from feagi.utils.version_checker import check_dependencies, verify_dependencies

__all__ = ["setup_logger", "check_dependencies", "verify_dependencies"] 