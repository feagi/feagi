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

"""Module for neuron mapping interfaces.

This module provides the NeuronMappingProvider interface used by components
to work with external mapping systems (like ConnectomeManager).
The old NeuronArray has been replaced by NPU-owned data structures.
"""

from abc import ABC, abstractmethod
from typing import List, Optional

# MEMORY OPTIMIZATION: Invalid cortical area index for uint16 optimization
INVALID_CORTICAL_IDX = 65535  # Max value for uint16, used instead of -1


class NeuronMappingProvider(ABC):
    """Interface for providing neuron ID-to-index mappings.

    This allows components to work with external mapping systems (like
    ConnectomeManager) instead of maintaining their own redundant mappings.
    """

    @abstractmethod
    def get_neuron_index(self, neuron_id: int) -> Optional[int]:
        """Get the array index for a neuron ID."""
        pass

    @abstractmethod
    def get_neuron_id(self, index: int) -> Optional[int]:
        """Get the neuron ID for an array index."""
        pass

    @abstractmethod
    def set_neuron_mapping(self, neuron_id: int, index: int) -> None:
        """Set a neuron ID to index mapping."""
        pass

    @abstractmethod
    def remove_neuron_mapping(self, neuron_id: int) -> None:
        """Remove a neuron mapping."""
        pass

    @abstractmethod
    def has_neuron(self, neuron_id: int) -> bool:
        """Check if a neuron ID exists."""
        pass

    @abstractmethod
    def get_all_neuron_ids(self) -> List[int]:
        """Get all neuron IDs."""
        pass


# Legacy Neuron class removed - unused in codebase
# All neuron data now managed by Rust NPU with Structure-of-Arrays layout
