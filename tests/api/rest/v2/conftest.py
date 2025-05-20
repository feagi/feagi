"""
Pytest fixtures for v2 API tests.

This module re-exports fixtures from the v1 conftest.py to ensure compatibility.
"""

import pytest
from tests.api.rest.v1.conftest import (
    client, 
    client_factory, 
    core_api_mock, 
    connectome_manager_mock,
    fcl_manager_mock
)

# Re-export the fixtures
# By importing them, they're automatically available to tests in this module 