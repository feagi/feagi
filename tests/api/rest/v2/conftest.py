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