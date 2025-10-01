"""
Logging Setup

Configures logging for FEAGI agent components.
"""

import logging
import logging.handlers
from pathlib import Path
from typing import Optional


def setup_agent_logging(
    log_level: str = "INFO",
    log_file: Optional[str] = "feagi_agent.log",
    neuron_log_file: Optional[str] = "neuron_data.log"
) -> tuple:
    """
    Set up logging for the FEAGI agent.
    
    Returns:
        tuple: (main_logger, neuron_logger)
    """
    # Convert string level to logging constant
    numeric_level = getattr(logging, log_level.upper(), logging.INFO)
    
    # Configure main logging
    logging.basicConfig(
        level=numeric_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(log_file) if log_file else logging.NullHandler()
        ]
    )
    
    # Get main logger
    main_logger = logging.getLogger("feagi_agent")
    
    # Configure neuron data logger
    neuron_logger = logging.getLogger("feagi_agent.neurons")
    if neuron_log_file:
        neuron_handler = logging.FileHandler(neuron_log_file)
        neuron_handler.setFormatter(
            logging.Formatter('%(asctime)s - NEURON - %(message)s')
        )
        neuron_logger.addHandler(neuron_handler)
        neuron_logger.setLevel(logging.INFO)
        neuron_logger.propagate = False  # Don't propagate to main logger
    
    main_logger.info("🔧 Logging configured")
    if neuron_log_file:
        main_logger.info(f"🧠 Neuron logging enabled: {neuron_log_file}")
    
    return main_logger, neuron_logger


def log_sensory_neuron_data(neuron_logger: logging.Logger, sensory_data: dict):
    """Log sensory neuron data in detail."""
    for cortical_id, neuron_dict in sensory_data.items():
        for (x, y, z), potential in neuron_dict.items():
            neuron_logger.info(f"OUT,{cortical_id},{x},{y},{z},{potential:.3f}")


def log_motor_neuron_data(neuron_logger: logging.Logger, motor_data: dict):
    """Log motor neuron data in detail."""
    for cortical_id, neuron_dict in motor_data.items():
        for (x, y, z), potential in neuron_dict.items():
            neuron_logger.info(f"IN,{cortical_id},{x},{y},{z},{potential:.3f}")


def setup_performance_logging(
    performance_log_file: str = "performance.log",
    log_level: str = "INFO"
) -> logging.Logger:
    """Set up performance logging."""
    perf_logger = logging.getLogger("feagi_agent.performance")
    
    # Create performance file handler
    perf_handler = logging.FileHandler(performance_log_file)
    perf_handler.setFormatter(
        logging.Formatter('%(asctime)s - PERF - %(message)s')
    )
    
    perf_logger.addHandler(perf_handler)
    perf_logger.setLevel(getattr(logging, log_level.upper(), logging.INFO))
    perf_logger.propagate = False
    
    return perf_logger 