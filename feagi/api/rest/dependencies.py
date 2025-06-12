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

from feagi.bdu import ConnectomeManager
from feagi.core.state_manager import FeagiStateManager

_connectome_instance = None


def get_connectome() -> ConnectomeManager:
    if _connectome_instance is None:
        raise RuntimeError("Connectome instance has not been initialized!")
    return _connectome_instance


def set_connectome_instance(connectome: ConnectomeManager):
    global _connectome_instance
    _connectome_instance = connectome


# Global variable to store the core API service
_core_api_service = None


def set_core_api_service(api_service):
    """Set the core API service instance."""
    global _core_api_service
    _core_api_service = api_service


# RUST/RTOS COMPATIBLE: Alias for direct dependency injection
def set_core_api_service_instance(api_service):
    """Set the core API service instance (Rust/RTOS compatible alias)."""
    set_core_api_service(api_service)


def get_core_api():
    """Get the core API service instance."""
    if _core_api_service is None:
        raise RuntimeError("Core API service not initialized")
    return _core_api_service


# Alias for backward compatibility
get_core_api_service = get_core_api


def get_state_manager():
    """Dependency provider for FeagiStateManager"""
    return FeagiStateManager.instance()
