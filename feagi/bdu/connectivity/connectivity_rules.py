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

"""Connectivity rules governing neural connections between cortical areas.

This module defines the rules that determine how neurons from different
cortical areas connect to each other during development.
"""

import logging
from typing import Any, Dict, Optional

import numpy as np

logger = logging.getLogger(__name__)


class ConnectivityRule:
    """Base class for connectivity rules between cortical areas."""

    def __init__(
        self,
        source_cortical_id: str,
        target_cortical_id: str,
        rule_type: str,
        parameters: Dict[str, Any],
        name: str = "",
        description: str = "",
        rule_id: Optional[str] = None,
    ):
        """Initialize a connectivity rule.

        Args:
            source_cortical_id: ID of the source cortical area
            target_cortical_id: ID of the target cortical area
            rule_type: Type of connectivity rule
            parameters: Rule-specific parameters
            name: Human-readable name for this rule
            description: Optional description of the rule
            rule_id: Unique identifier for this rule (optional, generated if not provided)
        """
        self.source_cortical_id = source_cortical_id
        self.target_cortical_id = target_cortical_id
        self.rule_type = rule_type
        self.parameters = parameters
        self.name = name
        self.description = description
        self.enabled = True
        self.id = (
            rule_id
            if rule_id
            else f"{source_cortical_id}_{target_cortical_id}_{rule_type}"
        )

    def apply(self, connectome) -> int:
        """Apply this rule to create connections.

        Args:
            connectome: The connectome manager object

        Returns:
            Number of connections created
        """
        raise NotImplementedError("Subclasses must implement apply()")

    def to_dict(self) -> Dict[str, Any]:
        """Convert rule to dictionary for serialization.

        Returns:
            Dictionary representation of the rule
        """
        return {
            "id": self.id,
            "name": self.name,
            "source_cortical_id": self.source_cortical_id,
            "target_cortical_id": self.target_cortical_id,
            "rule_type": self.rule_type,
            "parameters": self.parameters,
            "enabled": self.enabled,
            "description": self.description,
        }

    def update(self, updates: Dict[str, Any]) -> None:
        """Update rule properties.

        Args:
            updates: Dictionary of properties to update

        Raises:
            KeyError: If an invalid property is specified
        """
        valid_props = {"name", "rule_type", "parameters", "description", "enabled"}
        invalid_props = set(updates.keys()) - valid_props

        if invalid_props:
            raise KeyError(f"Invalid properties: {invalid_props}")

        if "name" in updates:
            self.name = updates["name"]

        if "rule_type" in updates:
            self.rule_type = updates["rule_type"]

        if "parameters" in updates:
            self.parameters.update(updates["parameters"])

        if "description" in updates:
            self.description = updates["description"]

        if "enabled" in updates:
            self.enabled = updates["enabled"]

    def validate(self) -> bool:
        """Validate that the rule has all required properties set.

        Returns:
            True if the rule is valid, False otherwise
        """
        if (
            not self.name
            or not self.source_cortical_id
            or not self.target_cortical_id
            or not self.rule_type
        ):
            return False
        return True

    @classmethod
    def from_dict(cls, data: Dict[str, Any]):
        """Create a rule from dictionary data.

        Args:
            data: Dictionary representation of the rule

        Returns:
            Instantiated rule object
        """
        rule = cls(
            source_cortical_id=data["source_cortical_id"],
            target_cortical_id=data["target_cortical_id"],
            rule_type=data["rule_type"],
            parameters=data["parameters"],
            name=data.get("name", ""),
            description=data.get("description", ""),
            rule_id=data.get("id"),
        )
        rule.enabled = data.get("enabled", True)
        return rule


class ProbabilisticConnectivityRule(ConnectivityRule):
    """Connectivity rule based on connection probability."""

    def __init__(
        self,
        source_cortical_id: str,
        target_cortical_id: str,
        connection_probability: float,
        max_connections: Optional[int] = None,
        parameters: Optional[Dict[str, Any]] = None,
        name: str = "",
        description: str = "",
        rule_id: Optional[str] = None,
    ):
        """Initialize a probabilistic connectivity rule.

        Args:
            source_cortical_id: ID of the source cortical area
            target_cortical_id: ID of the target cortical area
            connection_probability: Probability of connection (0.0 to 1.0)
            max_connections: Maximum number of connections to create (optional)
            parameters: Additional parameters (optional)
            name: Human-readable name for this rule
            description: Optional description of the rule
            rule_id: Unique identifier for this rule (optional, generated if not provided)
        """
        params = parameters or {}
        params.update(
            {
                "connection_probability": connection_probability,
                "max_connections": max_connections,
            }
        )
        super().__init__(
            source_cortical_id=source_cortical_id,
            target_cortical_id=target_cortical_id,
            rule_type="probabilistic",
            parameters=params,
            name=name,
            description=description,
            rule_id=rule_id,
        )

    def apply(self, connectome) -> int:
        """Apply this rule to create connections based on probability.

        Args:
            connectome: The connectome manager object

        Returns:
            Number of connections created
        """
        if not self.enabled:
            return 0

        logger.info(
            f"Applying probabilistic connectivity rule from {self.source_cortical_id} to {self.target_cortical_id}"
        )

        prob = self.parameters["connection_probability"]
        max_conn = self.parameters.get("max_connections")

        # Get neurons from source and target areas
        source_neurons = connectome.get_neurons_by_area(self.source_cortical_id)
        target_neurons = connectome.get_neurons_by_area(self.target_cortical_id)

        if not source_neurons or not target_neurons:
            logger.warning("No neurons found in source or target area")
            return 0

        # Create connections based on probability
        connections_created = 0

        # This is a simple implementation - would be optimized in a real system
        for source_id in source_neurons:
            potential_targets = []

            # Decide which target neurons to connect to based on probability
            for target_id in target_neurons:
                if np.random.random() < prob:
                    potential_targets.append(target_id)

            # Limit to max_connections if specified
            if max_conn and len(potential_targets) > max_conn:
                potential_targets = np.random.choice(
                    potential_targets, max_conn, replace=False
                ).tolist()

            # Create the connections
            for target_id in potential_targets:
                weight = 1.0  # Default weight, could be parameterized
                connectome.create_synapse(source_id, target_id, weight)
                connections_created += 1

        logger.info(f"Created {connections_created} connections")
        return connections_created


class DistanceBasedConnectivityRule(ConnectivityRule):
    """Connectivity rule based on spatial distance between neurons."""

    def __init__(
        self,
        source_cortical_id: str,
        target_cortical_id: str,
        max_distance: float,
        connection_probability: float = 1.0,
        parameters: Optional[Dict[str, Any]] = None,
        name: str = "",
        description: str = "",
        rule_id: Optional[str] = None,
    ):
        """Initialize a distance-based connectivity rule.

        Args:
            source_cortical_id: ID of the source cortical area
            target_cortical_id: ID of the target cortical area
            max_distance: Maximum distance for connections
            connection_probability: Probability of connection for neurons within distance
            parameters: Additional parameters (optional)
            name: Human-readable name for this rule
            description: Optional description of the rule
            rule_id: Unique identifier for this rule (optional, generated if not provided)
        """
        params = parameters or {}
        params.update(
            {
                "max_distance": max_distance,
                "connection_probability": connection_probability,
            }
        )
        super().__init__(
            source_cortical_id=source_cortical_id,
            target_cortical_id=target_cortical_id,
            rule_type="distance_based",
            parameters=params,
            name=name,
            description=description,
            rule_id=rule_id,
        )

    def apply(self, connectome) -> int:
        """Apply this rule to create connections based on distance.

        Args:
            connectome: The connectome manager object

        Returns:
            Number of connections created
        """
        if not self.enabled:
            return 0

        logger.info(
            f"Applying distance-based connectivity rule from {self.source_cortical_id} to {self.target_cortical_id}"
        )

        max_dist = self.parameters["max_distance"]
        prob = self.parameters["connection_probability"]

        # Implementation details would depend on how positions are stored in the connectome
        # This is a placeholder for the actual implementation
        connections_created = 0

        # A more sophisticated implementation would use spatial indices for efficiency

        logger.info(f"Created {connections_created} connections")
        return connections_created


# Registry of available rule types
RULE_TYPES = {
    "probabilistic": ProbabilisticConnectivityRule,
    "distance_based": DistanceBasedConnectivityRule,
}


def create_connectivity_rule(
    source_cortical_id: str,
    target_cortical_id: str,
    rule_type: str,
    parameters: Dict[str, Any],
    name: str = "",
    description: str = "",
    rule_id: Optional[str] = None,
) -> ConnectivityRule:
    """Factory function to create a connectivity rule.

    Args:
        source_cortical_id: ID of the source cortical area
        target_cortical_id: ID of the target cortical area
        rule_type: Type of rule to create
        parameters: Rule-specific parameters
        name: Human-readable name for this rule
        description: Optional description of the rule
        rule_id: Unique identifier for this rule (optional, generated if not provided)

    Returns:
        Instantiated rule object

    Raises:
        ValueError: If rule_type is unknown
    """
    if rule_type not in RULE_TYPES:
        raise ValueError(f"Unknown connectivity rule type: {rule_type}")

    rule_class = RULE_TYPES[rule_type]

    if rule_type == "probabilistic":
        connection_prob = parameters.get("connection_probability", 0.1)
        max_connections = parameters.get("max_connections")

        return ProbabilisticConnectivityRule(
            source_cortical_id=source_cortical_id,
            target_cortical_id=target_cortical_id,
            connection_probability=connection_prob,
            max_connections=max_connections,
            parameters=parameters,
            name=name,
            description=description,
            rule_id=rule_id,
        )

    elif rule_type == "distance_based":
        max_distance = parameters.get("max_distance", 10.0)
        connection_prob = parameters.get("connection_probability", 1.0)

        return DistanceBasedConnectivityRule(
            source_cortical_id=source_cortical_id,
            target_cortical_id=target_cortical_id,
            max_distance=max_distance,
            connection_probability=connection_prob,
            parameters=parameters,
            name=name,
            description=description,
            rule_id=rule_id,
        )

    else:
        # Create a basic rule for other types
        return ConnectivityRule(
            source_cortical_id=source_cortical_id,
            target_cortical_id=target_cortical_id,
            rule_type=rule_type,
            parameters=parameters,
            name=name,
            description=description,
            rule_id=rule_id,
        )


def register_custom_rule(rule_name: str, rule_class: type) -> None:
    """Register a custom connectivity rule type.

    Args:
        rule_name: Name for the new rule type
        rule_class: Class implementing the rule

    Raises:
        TypeError: If rule_class is not a subclass of ConnectivityRule
    """
    if not issubclass(rule_class, ConnectivityRule):
        raise TypeError("Custom rule must be a subclass of ConnectivityRule")

    RULE_TYPES[rule_name] = rule_class
    logger.info(f"Registered custom connectivity rule: {rule_name}")
