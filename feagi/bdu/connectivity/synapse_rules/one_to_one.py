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
One-to-one synapse rule implementation.

This module provides the OneToOneRule class for creating one-to-one
mappings between neurons in source and target cortical areas.
"""

from typing import Any, Dict, List, Tuple

from ..synapse_rule import SynapseRule


class OneToOneRule(SynapseRule):
    """
    Rule that connects neurons with the same relative position in source and target areas.

    This rule creates one-to-one mappings between neurons in source and target areas,
    matching them based on their relative positions.
    """

    def __init__(
        self, source_area: str, target_area: str, parameters: Dict[str, Any] = None
    ):
        """
        Initialize a one-to-one mapping rule.

        Args:
            source_area: ID of the source cortical area
            target_area: ID of the target cortical area
            parameters:
                - scale_factor: Factor to scale positions by (default: 1.0)
                - offset: Position offset to apply (default: (0, 0, 0))
        """
        super().__init__("one_to_one", source_area, target_area, parameters)

    def generate_connections(
        self,
        source_neurons: List[int],
        target_neurons: List[int],
        source_positions: Dict[int, Tuple[int, int, int]],
        target_positions: Dict[int, Tuple[int, int, int]],
    ) -> Dict[int, List[int]]:
        """
        Generate one-to-one connections based on relative positions.

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
        scale_factor = self.parameters.get("scale_factor", 1.0)
        offset = self.parameters.get("offset", (0, 0, 0))

        # Create a lookup of target positions to target neurons
        target_pos_to_neuron = {}
        for neuron_id in target_neurons:
            if neuron_id in target_positions:
                pos = target_positions[neuron_id]
                target_pos_to_neuron[pos] = neuron_id

        # For each source neuron, find a matching target neuron
        for source_id in source_neurons:
            if source_id not in source_positions:
                continue

            # Get source position and apply scaling/offset
            sx, sy, sz = source_positions[source_id]
            tx = int(sx * scale_factor) + offset[0]
            ty = int(sy * scale_factor) + offset[1]
            tz = int(sz * scale_factor) + offset[2]

            # Look for a target neuron at the corresponding position
            target_pos = (tx, ty, tz)
            if target_pos in target_pos_to_neuron:
                target_id = target_pos_to_neuron[target_pos]
                connections[source_id] = [target_id]

        return connections
