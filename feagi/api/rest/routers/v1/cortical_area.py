#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
FastAPI Cortical Area Router - Auto-Generated from v1 API

This router is automatically generated from decorated v1 API endpoints.
NO endpoint definitions exist here - they are all in /api/v1/cortical_area.py

The universal FastAPI wrapper scans the v1 API decorators and automatically
creates FastAPI routes, ensuring perfect consistency across all transports.
"""

# Import the auto-generated router from the universal wrapper
from feagi.api.transport.universal_fastapi import get_cortical_area_router

# The router is automatically generated from v1 API decorators
# This ensures that:
# 1. v1 API is the single source of truth
# 2. No endpoint duplication across transports  
# 3. Perfect consistency between FastAPI and ZMQ
# 4. Automatic route generation for future endpoints

router = get_cortical_area_router()

# Note: All endpoint implementations are in:
# - Single Source of Truth: feagi.api.v1.cortical_area.CorticalAreaAPI (decorated methods)
# - Auto-Generated FastAPI: feagi.api.transport.universal_fastapi
# - Auto-Generated ZMQ: feagi.api.transport.universal_zmq (future) 