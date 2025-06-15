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
Random synapse rule implementation.

This module provides the RandomRule class for creating random
connections between neurons in source and target cortical areas.
"""

import random
from typing import Any, Dict, List, Tuple

from ..synapse_rule import SynapseRule


class RandomRule(SynapseRule):
    """
    Rule that creates random connections between source and target neurons.

    This rule connects a variable number of source neurons to randomly selected
    target neurons.
    """

    def __init__(
        self, source_area: str, target_area: str, parameters: Dict[str, Any] = None
    ):
        """
        Initialize a random connection rule.

        Args:
            source_area: ID of the source cortical area
            target_area: ID of the target cortical area
            parameters:
                - connection_probability: Probability of forming a connection (default: 0.1)
                - max_connections: Maximum number of connections per source neuron (default: 10)
                - seed: Random seed for reproducibility (default: None)
        """
        super().__init__("random", source_area, target_area, parameters)

    def generate_connections(
        self,
        source_neurons: List[int],
        target_neurons: List[int],
        source_positions: Dict[int, Tuple[int, int, int]],
        target_positions: Dict[int, Tuple[int, int, int]],
    ) -> Dict[int, List[int]]:
        """
        Generate random connections between source and target neurons.

        Args:
            source_neurons: List of source neuron IDs
            target_neurons: List of target neuron IDs
            source_positions: Dictionary mapping source neuron IDs to positions
            target_positions: Dictionary mapping target neuron IDs to positions

        Returns:
            Dictionary mapping source neuron IDs to lists of target neuron IDs
        """
        connections = {}

        # Get parameter values
        connection_probability = self.parameters.get("connection_probability", 0.1)
        max_connections = self.parameters.get("max_connections", 10)
        seed = self.parameters.get("seed", None)

        # Set random seed if provided
        if seed is not None:
            random.seed(seed)

        # For each source neuron, randomly select target neurons
        for source_id in source_neurons:
            source_connections = []

            # Randomly select target neurons based on connection probability
            for target_id in target_neurons:
                if random.random() < connection_probability:
                    source_connections.append(target_id)

                    # Stop if we've reached the maximum number of connections
                    if len(source_connections) >= max_connections:
                        break

            # Only add to connections if we have at least one connection
            if source_connections:
                connections[source_id] = source_connections

        return connections
