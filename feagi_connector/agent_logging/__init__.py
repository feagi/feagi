"""
FEAGI Connector Logging

Configures logging for FEAGI agent components.
"""

from .setup import (
    setup_agent_logging,
    log_sensory_neuron_data,
    log_motor_neuron_data,
    setup_performance_logging
)

__all__ = [
    'setup_agent_logging',
    'log_sensory_neuron_data', 
    'log_motor_neuron_data',
    'setup_performance_logging'
] 