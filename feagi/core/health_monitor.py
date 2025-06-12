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
Health monitoring module for FEAGI processes.

This module provides health monitoring capabilities for FEAGI processes,
including heartbeat tracking, resource usage monitoring, and failure detection.
"""

# Import the HealthMonitor class from fault_tolerance module
from feagi.core.fault_tolerance import HealthMonitor

# Re-export for compatibility
__all__ = ["HealthMonitor"]
