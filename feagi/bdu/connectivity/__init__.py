"""Connectivity-related modules for the Brain Development Unit (BDU).

This package contains modules for connectivity rules, cortical mappings,
and synaptogenesis rules - all governing how neurons connect between areas.
"""

# Import synaptogenesis_rules for easy access
from feagi.bdu.connectivity import synaptogenesis_rules
from feagi.bdu.connectivity.connectivity_rules import ConnectivityRule
from feagi.bdu.connectivity.cortical_mappings import CorticalMapping
from feagi.bdu.connectivity.function_rules import DistanceBasedRule, OneToOneRule
from feagi.bdu.connectivity.mapping_utils import (
    build_power_connections,
    get_detailed_cortical_map,
)
from feagi.bdu.connectivity.pattern_rules import PatternRule
from feagi.bdu.connectivity.synapse_rule import SynapseRule
from feagi.bdu.connectivity.vector_rules import RandomRule

__all__ = [
    "synaptogenesis_rules",
    "ConnectivityRule",
    "CorticalMapping",
    "build_power_connections",
    "get_detailed_cortical_map",
    "SynapseRule",
    "OneToOneRule",
    "RandomRule",
    "DistanceBasedRule",
    "PatternRule",
]
