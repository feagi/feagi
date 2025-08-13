"""Connectivity-related modules for the Brain Development Unit (BDU).

This package contains modules for connectivity rules, cortical mappings,
and synaptogenesis rules - all governing how neurons connect between areas.
"""

# Import rules package
# Import synaptogenesis for easy access
from feagi.bdu.connectivity import rules, synaptogenesis
from feagi.bdu.connectivity.cortical_mappings import CorticalMapping

# Function rules are imported dynamically by synaptogenesis
from feagi.bdu.connectivity.mapping_utils import (
    build_power_connections,
    get_detailed_cortical_map,
)

__all__ = [
    "synaptogenesis",
    "CorticalMapping",
    "build_power_connections",
    "get_detailed_cortical_map",
    "rules",
]
