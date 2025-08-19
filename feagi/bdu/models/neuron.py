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

"""Module for neuron data structures and interfaces.

This module provides the core Neuron class and NeuronMappingProvider interface.
The old NeuronArray has been replaced by the NPU-owned data structures.
"""

import logging
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

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


class Neuron:
    """Wrapper class for individual neurons.

    This class provides an object-oriented interface to individual neurons for
    API compatibility. The underlying data is now stored in NPU-owned data structures.
    """

    def __init__(
        self,
        neuron_id: int,
        cortical_id: Optional[str] = None,
        position: Optional[Tuple[int, int, int]] = None,
        threshold: float = 1.0,
        membrane_potential: float = 0.0,
        resting_potential: float = 0.0,
        decay_rate: float = 0.5,
        refractory_period: int = 1,
    ):
        """Initialize a Neuron object.

        Args:
            neuron_id: Unique neuron ID
            cortical_id: ID of the cortical area
            position: 3D coordinates (x, y, z)
            threshold: Firing threshold
            membrane_potential: Initial membrane potential
            resting_potential: Resting potential
            decay_rate: Membrane potential decay rate
            refractory_period: Refractory period in timesteps
        """
        self.id = neuron_id
        self.cortical_id = cortical_id or "unknown"
        self.position = position or (0, 0, 0)
        self.threshold = threshold
        self.membrane_potential = membrane_potential
        self.resting_potential = resting_potential
        self.decay_rate = decay_rate
        self.refractory_period = refractory_period

    def to_dict(self) -> Dict[str, Any]:
        """Convert neuron to dictionary representation.

        Returns:
            Dictionary with neuron properties
        """
        return {
            "id": self.id,
            "cortical_id": self.cortical_id,
            "position": self.position,
            "threshold": self.threshold,
            "membrane_potential": self.membrane_potential,
            "resting_potential": self.resting_potential,
            "decay_rate": self.decay_rate,
            "refractory_period": self.refractory_period,
            # Add defaults for optional attributes
            "refractory_counter": getattr(self, "refractory_counter", 0),
            "is_active": getattr(self, "is_active", False),
            "properties": getattr(self, "properties", {}),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Neuron":
        """Create neuron from dictionary representation.

        Args:
            data: Dictionary with neuron properties

        Returns:
            Neuron instance
        """
        neuron = cls(
            neuron_id=data["id"],
            cortical_id=data["cortical_id"],
            position=data["position"],
            threshold=data["threshold"],
            membrane_potential=data["membrane_potential"],
            resting_potential=data["resting_potential"],
            decay_rate=data["decay_rate"],
            refractory_period=data["refractory_period"],
        )
        neuron.refractory_counter = data.get("refractory_counter", 0)
        neuron.is_active = data.get("is_active", False)

        return neuron