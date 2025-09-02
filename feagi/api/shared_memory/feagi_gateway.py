"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
SharedMemoryFEAGIGateway for FEAGI.

This module provides a Gateway implementation that communicates with the FEAGI core
using shared memory instead of ZMQ for higher performance and lower resource usage.
"""

import logging
import time

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)
import threading
from typing import Any, Dict, List, Optional

from .data_structures import SharedConfigDict
from .events import EventNotificationSystem, EventType
from .manager import SharedMemoryManager


class SharedMemoryFEAGIGateway:
    """Standalone gateway implementation that uses shared memory for
    communication with FEAGI core.

    This class replaces the ZMQ-based gateway with a more efficient IPC
    mechanism using memory-mapped files and event notifications. It is designed
    as a reference for both Python and Rust implementations, and is fully
    RTOS/embedded compatible.
    """

    def __init__(
        self, process_name: str = "api_server", temp_dir: Optional[str] = None
    ):
        """Initialize the shared memory gateway.

        Args:
            process_name: Unique name for this process (used in event notifications)
            temp_dir: Directory to store shared memory files
        """
        self.logger = logging.getLogger("feagi.api.shared_memory.gateway")
        self.process_name = process_name

        # Initialize shared memory components
        self.memory_manager = SharedMemoryManager(temp_dir=temp_dir)
        self.event_system = EventNotificationSystem(
            process_name, temp_dir=temp_dir
        )

        # Create shared configuration dictionary
        self.config_dict = SharedConfigDict(
            "feagi_config", manager=self.memory_manager
        )

        # Cache for frequently accessed data
        self._cache = {}
        self._cache_timestamps = {}
        self._cache_lock = threading.RLock()

        # Start event notification system
        self.event_system.start()

        # Register event handlers
        self._register_event_handlers()

        self.logger.info(
            f"SharedMemoryFEAGIGateway initialized for process {process_name}"
        )

    def _register_event_handlers(self):
        """Register handlers for different event types."""
        self.event_system.register_handler(
            EventType.CORTICAL_AREA_ADDED,
            lambda event: self._invalidate_cache("cortical_areas"),
        )
        self.event_system.register_handler(
            EventType.CORTICAL_AREA_REMOVED,
            lambda event: self._invalidate_cache("cortical_areas"),
        )
        self.event_system.register_handler(
            EventType.CORTICAL_AREA_UPDATED,
            lambda event: self._invalidate_cache("cortical_areas"),
        )
        self.event_system.register_handler(
            EventType.GENOME_LOADED, lambda event: self._invalidate_cache_all()
        )
        self.event_system.register_handler(
            EventType.CONFIG_UPDATED,
            lambda event: self._invalidate_cache("burst_engine_config"),
        )

    def _invalidate_cache(self, key: str):
        """Invalidate a specific cache entry."""
        with self._cache_lock:
            if key in self._cache:
                del self._cache[key]
                if key in self._cache_timestamps:
                    del self._cache_timestamps[key]

    def _invalidate_cache_all(self):
        """Invalidate all cache entries."""
        with self._cache_lock:
            self._cache.clear()
            self._cache_timestamps.clear()

    def _get_cached(self, key: str, max_age: float = 5.0):
        """Get a value from cache if available and not expired.

        Args:
            key: Cache key
            max_age: Maximum age in seconds

        Returns:
            Cached value or None if not available or expired
        """
        with self._cache_lock:
            if key in self._cache:
                timestamp = self._cache_timestamps.get(key, 0)
                if time.time() - timestamp <= max_age:
                    return self._cache[key]
        return None

    def _set_cache(self, key: str, value: Any):
        """Set a value in cache.

        Args:
            key: Cache key
            value: Value to cache
        """
        with self._cache_lock:
            self._cache[key] = value
            self._cache_timestamps[key] = time.time()

    def get_cortical_areas(self) -> List[Dict[str, Any]]:
        """Get a list of all cortical areas.

        Returns:
            List of dictionaries containing cortical area information
        """
        # Check cache first
        cached = self._get_cached("cortical_areas")
        if cached is not None:
            return cached

        # Otherwise, get from shared memory
        # Get from shared config dictionary
        areas = self.config_dict.get("cortical_areas", [])
        self._set_cache("cortical_areas", areas)
        return areas

    def get_cortical_area(self, area_id: str) -> Optional[Dict[str, Any]]:
        """Get information about a specific cortical area.

        Args:
            area_id: ID of the cortical area

        Returns:
            Dictionary containing cortical area information, or None if not found
        """
        # Check cache first
        cache_key = f"cortical_area_{area_id}"
        cached = self._get_cached(cache_key)
        if cached is not None:
            return cached

        # Otherwise, get from shared memory
        areas = self.get_cortical_areas()
        for area in areas:
            if str(area.get("id")) == str(area_id):
                self._set_cache(cache_key, area)
                return area

        return None

    def get_cortical_area_types(self) -> Dict[str, List[str]]:
        """Get available cortical area types.

        Returns:
            Dictionary of cortical area types
        """
        # Check cache first
        cached = self._get_cached("cortical_area_types")
        if cached is not None:
            return cached

        # Otherwise, get from shared memory
        area_types = self.config_dict.get(
            "cortical_area_types", {"cortical": [], "subcortical": []}
        )
        self._set_cache("cortical_area_types", area_types)
        return area_types

    def get_burst_engine_config(self) -> Dict[str, Any]:
        """Get the burst engine configuration.

        Returns:
            Dictionary containing burst engine configuration
        """
        # Check cache first
        cached = self._get_cached("burst_engine_config")
        if cached is not None:
            return cached

        # Otherwise, get from shared memory
        config = self.config_dict.get(
            "burst_engine_config",
            {
                "burst_duration": 10,
                "inter_burst_interval": 5,
                "maximum_firing_rate": 100,
                "refractory_period": 5,
                "threshold": 0.5,
                "decay_rate": 0.1,
                "firing_threshold": 0.7,
                "membrane_potential_decay": 0.05,
            },
        )
        self._set_cache("burst_engine_config", config)
        return config

    def get_genome_filename(self) -> Optional[str]:
        """Get the filename of the currently loaded genome.

        Returns:
            Filename of the current genome, or None if no genome is loaded
        """
        # Check cache first
        cached = self._get_cached("genome_filename")
        if cached is not None:
            return cached

        # Otherwise, get from shared memory
        filename = self.config_dict.get("genome_filename", None)
        self._set_cache("genome_filename", filename)
        return filename

    def set_burst_engine_config(self, config: Dict[str, Any]) -> bool:
        """Set the burst engine configuration.

        Args:
            config: Dictionary containing burst engine configuration

        Returns:
            True if successful, False otherwise
        """
        # Otherwise, set in shared memory
        result = self.config_dict.set("burst_engine_config", config)
        if result:
            self._invalidate_cache("burst_engine_config")
            self.event_system.send_event(
                EventType.CONFIG_UPDATED, data={"type": "burst_engine"}
            )
        return result

    def load_genome(self, genome_data: Dict[str, Any], filename: str) -> bool:
        """Load a genome from data.

        Args:
            genome_data: Dictionary containing genome data
            filename: Name to associate with the genome

        Returns:
            True if successful, False otherwise
        """
        # Otherwise, set in shared memory
        result = self.config_dict.set("genome", genome_data)
        if result:
            self.config_dict.set("genome_filename", filename)
            self._invalidate_cache_all()
            self.event_system.send_event(
                EventType.GENOME_LOADED, data={"filename": filename}
            )
        return result

    def shutdown(self, delete_shared_memory: bool = False):
        """Shutdown the gateway and clean up resources.

        Args:
            delete_shared_memory: If True, delete shared memory files (default: False)
        """
        self.logger.info("Shutting down SharedMemoryFEAGIGateway")
        self.event_system.stop()
        self.config_dict.region.close(delete_file=delete_shared_memory)
        self.logger.info("SharedMemoryFEAGIGateway shutdown complete")

    def __del__(self):
        """Ensure resources are cleaned up."""
        self.shutdown()
