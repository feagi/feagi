"""Connectivity-related modules for the Brain Development Unit (BDU).

This package contains modules for connectivity rules, cortical mappings,
and synaptogenesis rules - all governing how neurons connect between areas.
"""

from feagi.bdu.connectivity.connectivity_rules import ConnectivityRule
from feagi.bdu.connectivity.cortical_mappings import CorticalMapping
from feagi.bdu.connectivity.synaptogenesis import SynaptogenesisRule
from feagi.bdu.connectivity.synapse_rule import SynapseRule
from feagi.bdu.connectivity.mapping_utils import get_detailed_cortical_map, build_power_connections 

# Import synaptogenesis_rules for easy access
from feagi.bdu.connectivity import synaptogenesis_rules 