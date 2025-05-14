"""Connectivity rules governing neural connections between cortical areas.

This module defines the rules that determine how neurons from different
cortical areas connect to each other during development.
"""

from typing import Dict, Any, List, Tuple, Callable, Optional, Union
import numpy as np
import logging

logger = logging.getLogger(__name__)


class ConnectivityRule:
    """Base class for connectivity rules between cortical areas."""
    
    def __init__(self, source_area_id: str, target_area_id: str, rule_type: str, parameters: Dict[str, Any]):
        """Initialize a connectivity rule.
        
        Args:
            source_area_id: ID of the source cortical area
            target_area_id: ID of the target cortical area
            rule_type: Type of connectivity rule
            parameters: Rule-specific parameters
        """
        self.source_area_id = source_area_id
        self.target_area_id = target_area_id
        self.rule_type = rule_type
        self.parameters = parameters
        self.enabled = True
        self.id = f"{source_area_id}_{target_area_id}_{rule_type}"
    
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
            "source_area_id": self.source_area_id,
            "target_area_id": self.target_area_id,
            "rule_type": self.rule_type,
            "parameters": self.parameters,
            "enabled": self.enabled
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]):
        """Create a rule from dictionary data.
        
        Args:
            data: Dictionary representation of the rule
            
        Returns:
            Instantiated rule object
        """
        rule = cls(
            source_area_id=data["source_area_id"],
            target_area_id=data["target_area_id"],
            rule_type=data["rule_type"],
            parameters=data["parameters"]
        )
        rule.enabled = data.get("enabled", True)
        return rule


class ProbabilisticConnectivityRule(ConnectivityRule):
    """Connectivity rule based on connection probability."""
    
    def __init__(self, source_area_id: str, target_area_id: str, 
                 connection_probability: float, max_connections: Optional[int] = None,
                 parameters: Optional[Dict[str, Any]] = None):
        """Initialize a probabilistic connectivity rule.
        
        Args:
            source_area_id: ID of the source cortical area
            target_area_id: ID of the target cortical area
            connection_probability: Probability of connection (0.0 to 1.0)
            max_connections: Maximum number of connections to create (optional)
            parameters: Additional parameters (optional)
        """
        params = parameters or {}
        params.update({
            "connection_probability": connection_probability,
            "max_connections": max_connections
        })
        super().__init__(source_area_id, target_area_id, "probabilistic", params)
    
    def apply(self, connectome) -> int:
        """Apply this rule to create connections based on probability.
        
        Args:
            connectome: The connectome manager object
            
        Returns:
            Number of connections created
        """
        if not self.enabled:
            return 0
        
        logger.info(f"Applying probabilistic connectivity rule from {self.source_area_id} to {self.target_area_id}")
        
        prob = self.parameters["connection_probability"]
        max_conn = self.parameters.get("max_connections")
        
        # Get neurons from source and target areas
        source_neurons = connectome.get_neurons_by_area(self.source_area_id)
        target_neurons = connectome.get_neurons_by_area(self.target_area_id)
        
        if not source_neurons or not target_neurons:
            logger.warning(f"No neurons found in source or target area")
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
    
    def __init__(self, source_area_id: str, target_area_id: str, 
                 max_distance: float, connection_probability: float = 1.0,
                 parameters: Optional[Dict[str, Any]] = None):
        """Initialize a distance-based connectivity rule.
        
        Args:
            source_area_id: ID of the source cortical area
            target_area_id: ID of the target cortical area
            max_distance: Maximum distance for connections
            connection_probability: Probability of connection for neurons within distance
            parameters: Additional parameters (optional)
        """
        params = parameters or {}
        params.update({
            "max_distance": max_distance,
            "connection_probability": connection_probability
        })
        super().__init__(source_area_id, target_area_id, "distance_based", params)
    
    def apply(self, connectome) -> int:
        """Apply this rule to create connections based on distance.
        
        Args:
            connectome: The connectome manager object
            
        Returns:
            Number of connections created
        """
        if not self.enabled:
            return 0
        
        logger.info(f"Applying distance-based connectivity rule from {self.source_area_id} to {self.target_area_id}")
        
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
    "distance_based": DistanceBasedConnectivityRule
}


def create_connectivity_rule(source_area_id: str, target_area_id: str, 
                           rule_type: str, parameters: Dict[str, Any]) -> ConnectivityRule:
    """Factory function to create a connectivity rule.
    
    Args:
        source_area_id: ID of the source cortical area
        target_area_id: ID of the target cortical area
        rule_type: Type of rule to create
        parameters: Rule-specific parameters
        
    Returns:
        Instantiated rule object
        
    Raises:
        ValueError: If rule_type is unknown
    """
    if rule_type not in RULE_TYPES:
        raise ValueError(f"Unknown connectivity rule type: {rule_type}")
    
    rule_class = RULE_TYPES[rule_type]
    return rule_class(source_area_id, target_area_id, **parameters)


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