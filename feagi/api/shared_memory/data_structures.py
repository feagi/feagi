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
Shared Memory Data Structures for FEAGI IPC.

This module provides shared memory data structures used for efficient
inter-process communication in FEAGI.
"""

import json
import logging
import os

from feagi.utils.logger import setup_logger

logger = setup_logger()
from typing import Any, Dict, List, Optional

import numpy as np

from .manager import SharedMemoryManager


class SharedNeuronArray:
    """A shared memory array for storing neuron data that can be accessed by
    multiple processes."""

    # Define neuron property fields and their data types
    NEURON_DTYPE = np.dtype(
        [
            ("id", np.int64),  # Unique neuron ID
            ("x", np.float32),  # X coordinate
            ("y", np.float32),  # Y coordinate
            ("z", np.float32),  # Z coordinate
            ("membrane_potential", np.float32),  # Current membrane potential
            ("firing_threshold", np.float32),  # Firing threshold
            ("refractory_period", np.float32),  # Refractory period in ms
            ("last_fired_timestamp", np.float64),  # Last time the neuron fired
            (
                "cortical_id",
                np.int64,
            ),  # ID of the cortical area this neuron belongs to
            ("neuron_type", np.int32),  # Type of neuron
            ("is_active", np.bool_),  # Whether the neuron is active
            ("reserved1", np.float32),  # Reserved for future use
            ("reserved2", np.float32),  # Reserved for future use
        ]
    )

    def __init__(
        self,
        name: str,
        capacity: int,
        manager: Optional[SharedMemoryManager] = None,
    ):
        """Initialize a shared neuron array.

        Args:
            name: Name of the shared memory region
            capacity: Maximum number of neurons to support
            manager: Optional shared memory manager (will create one if not provided)
        """
        self.name = name
        self.capacity = capacity
        self.logger = logging.getLogger(
            f"feagi.api.shared_memory.neuron_array.{name}"
        )

        # Calculate required size
        self.element_size = self.NEURON_DTYPE.itemsize
        self.required_size = (
            capacity * self.element_size + 1024
        )  # Extra space for metadata

        # Get or create the memory manager
        self._manager = manager or SharedMemoryManager()

        # Create the shared memory region
        self.region = self._manager.create_region(
            name, size=self.required_size
        )

        # Create the numpy array view
        self.array = self.region.as_array(
            shape=(capacity,), dtype=self.NEURON_DTYPE
        )

        # Initialize metadata
        self.count = 0
        self._neuron_id_to_index: Dict[int, int] = {}
        self._cortical_id_to_indices: Dict[int, List[int]] = {}

        # Initialize the array with zeros
        self.array.fill(0)

        self.logger.info(
            f"Initialized SharedNeuronArray '{name}' with capacity {capacity}"
        )

    def add_neuron(
        self,
        neuron_id: int,
        x: float,
        y: float,
        z: float,
        cortical_id: int,
        neuron_type: int = 0,
        firing_threshold: float = 0.5,
        refractory_period: float = 5.0,
        membrane_potential: float = 0.0,
    ) -> bool:
        """Add a neuron to the array.

        Args:
            neuron_id: Unique neuron ID
            x, y, z: Coordinates of the neuron
            cortical_id: ID of the cortical area this neuron belongs to
            neuron_type: Type of neuron (0=default)
            firing_threshold: Threshold at which the neuron fires
            refractory_period: Refractory period in ms
            membrane_potential: Initial membrane potential

        Returns:
            True if added successfully, False if array is full or neuron already exists
        """
        # Check if we have capacity
        if self.count >= self.capacity:
            self.logger.error(f"Cannot add neuron {neuron_id}: array is full")
            return False

        # Check if neuron already exists
        if neuron_id in self._neuron_id_to_index:
            self.logger.warning(f"Neuron {neuron_id} already exists")
            return False

        # Add to the array
        idx = self.count
        self.array[idx]["id"] = neuron_id
        self.array[idx]["x"] = x
        self.array[idx]["y"] = y
        self.array[idx]["z"] = z
        self.array[idx]["membrane_potential"] = membrane_potential
        self.array[idx]["firing_threshold"] = firing_threshold
        self.array[idx]["refractory_period"] = refractory_period
        self.array[idx]["last_fired_timestamp"] = 0.0
        self.array[idx]["cortical_id"] = cortical_id
        self.array[idx]["neuron_type"] = neuron_type
        self.array[idx]["is_active"] = True

        # Update our tracking data structures
        self._neuron_id_to_index[neuron_id] = idx

        if cortical_id not in self._cortical_id_to_indices:
            self._cortical_id_to_indices[cortical_id] = []
        self._cortical_id_to_indices[cortical_id].append(idx)

        self.count += 1
        return True

    def get_neuron(self, neuron_id: int) -> Optional[np.ndarray]:
        """Get a neuron by ID.

        Args:
            neuron_id: ID of the neuron to get

        Returns:
            Neuron data as a numpy record array, or None if not found
        """
        if neuron_id not in self._neuron_id_to_index:
            return None

        idx = self._neuron_id_to_index[neuron_id]
        return self.array[idx]

    def get_neurons_by_cortical_area(self, cortical_id: int) -> np.ndarray:
        """Get all neurons in a cortical area.

        Args:
            cortical_id: ID of the cortical area

        Returns:
            Array of neurons in the cortical area (empty if none found)
        """
        if cortical_id not in self._cortical_id_to_indices:
            return np.empty(0, dtype=self.NEURON_DTYPE)

        indices = self._cortical_id_to_indices[cortical_id]
        return self.array[indices]

    def update_neuron(self, neuron_id: int, **kwargs) -> bool:
        """Update properties of a neuron.

        Args:
            neuron_id: ID of the neuron to update
            **kwargs: Properties to update and their new values

        Returns:
            True if updated successfully, False if neuron not found
        """
        if neuron_id not in self._neuron_id_to_index:
            return False

        idx = self._neuron_id_to_index[neuron_id]

        # Update fields
        for key, value in kwargs.items():
            if key in self.NEURON_DTYPE.names:
                self.array[idx][key] = value
            else:
                self.logger.warning(f"Unknown neuron property: {key}")

        return True

    def remove_neuron(self, neuron_id: int) -> bool:
        """Remove a neuron from the array.

        Args:
            neuron_id: ID of the neuron to remove

        Returns:
            True if removed successfully, False if neuron not found
        """
        if neuron_id not in self._neuron_id_to_index:
            return False

        idx = self._neuron_id_to_index[neuron_id]
        cortical_id = self.array[idx]["cortical_id"]

        # Instead of removing, just mark as inactive
        self.array[idx]["is_active"] = False

        # Update our tracking
        del self._neuron_id_to_index[neuron_id]

        if cortical_id in self._cortical_id_to_indices:
            try:
                self._cortical_id_to_indices[cortical_id].remove(idx)
            except ValueError:
                pass

        return True

    def remove_cortical_area(self, cortical_id: int) -> int:
        """Remove all neurons in a cortical area.

        Args:
            cortical_id: ID of the cortical area to remove

        Returns:
            Number of neurons removed
        """
        if cortical_id not in self._cortical_id_to_indices:
            return 0

        # Get all neurons in this area
        indices = self._cortical_id_to_indices[cortical_id]
        count = len(indices)

        # Mark them all as inactive
        for idx in indices:
            neuron_id = self.array[idx]["id"]
            self.array[idx]["is_active"] = False

            # Update our tracking
            if neuron_id in self._neuron_id_to_index:
                del self._neuron_id_to_index[neuron_id]

        # Clear our tracking
        del self._cortical_id_to_indices[cortical_id]

        return count


class SharedSynapseArray:
    """A shared memory array for storing synaptic connections between
    neurons."""

    # Define synapse data structure
    SYNAPSE_DTYPE = np.dtype(
        [
            ("pre_neuron_id", np.int64),  # ID of the presynaptic neuron
            ("post_neuron_id", np.int64),  # ID of the postsynaptic neuron
            ("weight", np.float32),  # Synaptic weight
            ("delay", np.float32),  # Synaptic delay in ms
            ("plasticity", np.float32),  # Plasticity coefficient
            ("type", np.int32),  # Type of synapse (0=excitatory, 1=inhibitory)
            ("is_active", np.bool_),  # Whether the synapse is active
            (
                "last_update_timestamp",
                np.float64,
            ),  # Last time the synapse was updated
            ("reserved1", np.float32),  # Reserved for future use
        ]
    )

    def __init__(
        self,
        name: str,
        capacity: int,
        manager: Optional[SharedMemoryManager] = None,
    ):
        """Initialize a shared synapse array.

        Args:
            name: Name of the shared memory region
            capacity: Maximum number of synapses to support
            manager: Optional shared memory manager (will create one if not provided)
        """
        self.name = name
        self.capacity = capacity
        self.logger = logging.getLogger(
            f"feagi.api.shared_memory.synapse_array.{name}"
        )

        # Calculate required size
        self.element_size = self.SYNAPSE_DTYPE.itemsize
        self.required_size = (
            capacity * self.element_size + 1024
        )  # Extra space for metadata

        # Get or create the memory manager
        self._manager = manager or SharedMemoryManager()

        # Create the shared memory region
        self.region = self._manager.create_region(
            name, size=self.required_size
        )

        # Create the numpy array view
        self.array = self.region.as_array(
            shape=(capacity,), dtype=self.SYNAPSE_DTYPE
        )

        # Initialize metadata
        self.count = 0
        self._pre_to_indices: Dict[int, List[int]] = (
            {}
        )  # Pre-neuron ID to synapse indices
        self._post_to_indices: Dict[int, List[int]] = (
            {}
        )  # Post-neuron ID to synapse indices

        # Initialize the array with zeros
        self.array.fill(0)

        self.logger.info(
            f"Initialized SharedSynapseArray '{name}' with capacity {capacity}"
        )

    def add_synapse(
        self,
        pre_neuron_id: int,
        post_neuron_id: int,
        weight: float,
        delay: float = 1.0,
        plasticity: float = 0.01,
        synapse_type: int = 0,
    ) -> bool:
        """Add a synapse to the array.

        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron
            weight: Synaptic weight
            delay: Synaptic delay in ms
            plasticity: Plasticity coefficient
            synapse_type: Type of synapse (0=excitatory, 1=inhibitory)

        Returns:
            True if added successfully, False if array is full or synapse already exists
        """
        # Check if we have capacity
        if self.count >= self.capacity:
            self.logger.error("Cannot add synapse: array is full")
            return False

        # Check if synapse already exists
        existing_idx = self._find_synapse_index(pre_neuron_id, post_neuron_id)
        if existing_idx is not None:
            # If it exists but was inactive, reactivate it
            if not self.array[existing_idx]["is_active"]:
                self.array[existing_idx]["weight"] = weight
                self.array[existing_idx]["delay"] = delay
                self.array[existing_idx]["plasticity"] = plasticity
                self.array[existing_idx]["type"] = synapse_type
                self.array[existing_idx]["is_active"] = True
                self.array[existing_idx]["last_update_timestamp"] = 0.0
                return True
            else:
                self.logger.warning(
                    f"Synapse from {pre_neuron_id} to {post_neuron_id} already exists"
                )
                return False

        # Add to the array
        idx = self.count
        self.array[idx]["pre_neuron_id"] = pre_neuron_id
        self.array[idx]["post_neuron_id"] = post_neuron_id
        self.array[idx]["weight"] = weight
        self.array[idx]["delay"] = delay
        self.array[idx]["plasticity"] = plasticity
        self.array[idx]["type"] = synapse_type
        self.array[idx]["is_active"] = True
        self.array[idx]["last_update_timestamp"] = 0.0

        # Update our tracking data structures
        if pre_neuron_id not in self._pre_to_indices:
            self._pre_to_indices[pre_neuron_id] = []
        self._pre_to_indices[pre_neuron_id].append(idx)

        if post_neuron_id not in self._post_to_indices:
            self._post_to_indices[post_neuron_id] = []
        self._post_to_indices[post_neuron_id].append(idx)

        self.count += 1
        return True

    def _find_synapse_index(
        self, pre_neuron_id: int, post_neuron_id: int
    ) -> Optional[int]:
        """Find the index of a synapse by pre and post neuron IDs."""
        if pre_neuron_id not in self._pre_to_indices:
            return None

        for idx in self._pre_to_indices[pre_neuron_id]:
            if self.array[idx]["post_neuron_id"] == post_neuron_id:
                return idx

        return None

    def get_synapse(
        self, pre_neuron_id: int, post_neuron_id: int
    ) -> Optional[np.ndarray]:
        """Get a synapse by pre and post neuron IDs.

        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron

        Returns:
            Synapse data as a numpy record array, or None if not found
        """
        idx = self._find_synapse_index(pre_neuron_id, post_neuron_id)
        if idx is None:
            return None

        return self.array[idx]

    def get_efferent_synapses(self, neuron_id: int) -> np.ndarray:
        """Get all efferent (outgoing) synapses for a neuron.

        Args:
            neuron_id: ID of the neuron

        Returns:
            Array of synapses where the given neuron is presynaptic
        """
        if neuron_id not in self._pre_to_indices:
            return np.empty(0, dtype=self.SYNAPSE_DTYPE)

        indices = self._pre_to_indices[neuron_id]
        # Filter for active synapses
        active_indices = [
            idx for idx in indices if self.array[idx]["is_active"]
        ]
        if not active_indices:
            return np.empty(0, dtype=self.SYNAPSE_DTYPE)

        return self.array[active_indices]

    def get_afferent_synapses(self, neuron_id: int) -> np.ndarray:
        """Get all afferent (incoming) synapses for a neuron.

        Args:
            neuron_id: ID of the neuron

        Returns:
            Array of synapses where the given neuron is postsynaptic
        """
        if neuron_id not in self._post_to_indices:
            return np.empty(0, dtype=self.SYNAPSE_DTYPE)

        indices = self._post_to_indices[neuron_id]
        # Filter for active synapses
        active_indices = [
            idx for idx in indices if self.array[idx]["is_active"]
        ]
        if not active_indices:
            return np.empty(0, dtype=self.SYNAPSE_DTYPE)

        return self.array[active_indices]

    def update_synapse(
        self, pre_neuron_id: int, post_neuron_id: int, **kwargs
    ) -> bool:
        """Update properties of a synapse.

        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron
            **kwargs: Properties to update and their new values

        Returns:
            True if updated successfully, False if synapse not found
        """
        idx = self._find_synapse_index(pre_neuron_id, post_neuron_id)
        if idx is None:
            return False

        # Update fields
        for key, value in kwargs.items():
            if key in self.SYNAPSE_DTYPE.names:
                self.array[idx][key] = value
            else:
                self.logger.warning(f"Unknown synapse property: {key}")

        return True

    def remove_synapse(self, pre_neuron_id: int, post_neuron_id: int) -> bool:
        """Remove a synapse from the array.

        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron

        Returns:
            True if removed successfully, False if synapse not found
        """
        idx = self._find_synapse_index(pre_neuron_id, post_neuron_id)
        if idx is None:
            return False

        # Instead of removing, just mark as inactive
        self.array[idx]["is_active"] = False

        return True

    def remove_neuron_synapses(self, neuron_id: int) -> int:
        """Remove all synapses connected to a neuron.

        Args:
            neuron_id: ID of the neuron

        Returns:
            Number of synapses removed
        """
        count = 0

        # Remove efferent synapses
        if neuron_id in self._pre_to_indices:
            for idx in self._pre_to_indices[neuron_id]:
                self.array[idx]["is_active"] = False
                count += 1

            # Don't delete from tracking as we still need to know the indices

        # Remove afferent synapses
        if neuron_id in self._post_to_indices:
            for idx in self._post_to_indices[neuron_id]:
                self.array[idx]["is_active"] = False
                count += 1

            # Don't delete from tracking as we still need to know the indices

        return count


class SharedConfigDict:
    """A shared memory dictionary for configuration data that can be accessed
    by multiple processes."""

    def __init__(
        self,
        name: str,
        manager: Optional[SharedMemoryManager] = None,
        initial_data: Optional[Dict[str, Any]] = None,
    ):
        """Initialize a shared configuration dictionary.

        Args:
            name: Name of the shared memory region
            manager: Optional shared memory manager (will create one if not provided)
            initial_data: Initial configuration data
        """
        self.name = name
        self.logger = logging.getLogger(
            f"feagi.api.shared_memory.config_dict.{name}"
        )

        # Estimate initial size based on data or use default
        initial_size = 1024 * 1024  # 1MB default
        if initial_data:
            # Rough estimate of JSON size + buffer
            initial_size = max(initial_size, len(json.dumps(initial_data)) * 2)

        # Get or create the memory manager
        self._manager = manager or SharedMemoryManager()

        # Create the shared memory region
        file_exists = os.path.exists(self._manager.region_path(name))
        self.region = self._manager.create_region(name, size=initial_size)

        # Initialize with data if provided, or only if file is new
        if initial_data:
            self.update(initial_data)
        elif not file_exists:
            # Only initialize with empty dictionary if file is new
            self._write_dict({})

    def _read_dict(self) -> Dict[str, Any]:
        """Read the dictionary from shared memory."""
        try:
            data = self.region.read()
            # Find the end of the JSON data (null-terminated)
            null_pos = data.find(b"\0")
            if null_pos > 0:
                data = data[:null_pos]

            if not data:
                return {}

            return json.loads(data.decode("utf-8"))
        except Exception as e:
            self.logger.error(f"Error reading shared config dictionary: {e}")
            return {}

    def _write_dict(self, data: Dict[str, Any]) -> bool:
        """Write the dictionary to shared memory."""
        try:
            json_str = json.dumps(data)
            encoded = json_str.encode("utf-8") + b"\0"

            if len(encoded) > self.region.size:
                self.logger.error(
                    f"Config dictionary too large for shared memory: {len(encoded)} bytes"
                )
                return False

            self.region.write(encoded)
            return True
        except Exception as e:
            self.logger.error(f"Error writing shared config dictionary: {e}")
            return False

    def get(self, key: str, default: Any = None) -> Any:
        """Get a value from the dictionary.

        Args:
            key: Key to get
            default: Default value if key doesn't exist

        Returns:
            Value for the key, or default if not found
        """
        if not self.region.acquire_lock(timeout=1.0):
            self.logger.warning(
                "Could not acquire lock for reading config dictionary"
            )
            return default

        try:
            self.region.reload()
            data = self._read_dict()
            return data.get(key, default)
        finally:
            self.region.release_lock()

    def set(self, key: str, value: Any) -> bool:
        """Set a value in the dictionary.

        Args:
            key: Key to set
            value: Value to set

        Returns:
            True if successful, False otherwise
        """
        if not self.region.acquire_lock(timeout=1.0):
            self.logger.warning(
                "Could not acquire lock for writing config dictionary"
            )
            return False
        try:
            data = self._read_dict()
            data[key] = value
            self._write_dict(data)
            self.region.mmap.flush()
            self.region.file.flush()
            os.fsync(self.region.file.fileno())
            self.region.reload()
            return True
        finally:
            self.region.release_lock()

    def update(self, values: Dict[str, Any]) -> bool:
        """Update multiple values in the dictionary.

        Args:
            values: Dictionary of key-value pairs to update

        Returns:
            True if successful, False otherwise
        """
        # Acquire lock to ensure atomic update
        if not self.region.acquire_lock(timeout=1.0):
            self.logger.warning(
                "Could not acquire lock for updating config dictionary"
            )
            return False

        try:
            data = self._read_dict()
            data.update(values)
            return self._write_dict(data)
        finally:
            self.region.release_lock()

    def delete(self, key: str) -> bool:
        """Delete a key from the dictionary.

        Args:
            key: Key to delete

        Returns:
            True if successful, False if key not found
        """
        # Acquire lock to ensure atomic update
        if not self.region.acquire_lock(timeout=1.0):
            self.logger.warning(
                "Could not acquire lock for updating config dictionary"
            )
            return False

        try:
            data = self._read_dict()
            if key in data:
                del data[key]
                return self._write_dict(data)
            return False
        finally:
            self.region.release_lock()

    def get_all(self) -> Dict[str, Any]:
        """Get all key-value pairs in the dictionary.

        Returns:
            Dictionary containing all key-value pairs
        """
        if not self.region.acquire_lock(timeout=1.0):
            self.logger.warning(
                "Could not acquire lock for reading config dictionary"
            )
            return {}

        try:
            self.region.reload()
            return self._read_dict()
        finally:
            self.region.release_lock()

    def clear(self) -> bool:
        """Clear all keys from the dictionary.

        Returns:
            True if successful, False otherwise
        """
        # Acquire lock to ensure atomic update
        if not self.region.acquire_lock(timeout=1.0):
            self.logger.warning(
                "Could not acquire lock for clearing config dictionary"
            )
            return False

        try:
            return self._write_dict({})
        finally:
            self.region.release_lock()
