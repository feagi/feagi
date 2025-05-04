"""Core functionality for the FEAGI framework.

This module provides the critical (Priority 1) components of the FEAGI framework:
- Burst Engine: Manages neuron firing dynamics
- Connectome Manager: Handles access to neuron and synapse data structures
- FCL Manager: Maintains the Fire Candidate List
- Memory & Learning Manager: Applies plasticity rules to synaptic weights
"""
import logging
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)

def create_core_api(config: Dict[str, Any] = None):
    """
    Create and initialize the Core API with all critical (Priority 1) processes.
    
    This function initializes the core components in the correct order:
    1. Connectome Manager
    2. FCL Manager
    3. Burst Engine
    4. Memory & Learning Manager
    
    Args:
        config: Configuration parameters for core components
        
    Returns:
        CoreAPIService instance with all core components initialized
    """
    from .feagi import FEAGI
    from feagi.api.core.services.core_api_service import CoreAPIService
    
    # Create and initialize the FEAGI core instance
    logger.info("Initializing FEAGI core components...")
    
    # Set defaults if config is None
    if config is None:
        config = {}
    
    # Get GPU settings
    use_gpu = config.get("core", {}).get("use_gpu", False)
    
    # Initialize FEAGI core
    feagi_core = FEAGI(use_gpu=use_gpu)
    
    # Create the CoreAPIService wrapper around the FEAGI core
    core_api = CoreAPIService(feagi_core)
    
    logger.info("FEAGI core components initialized successfully")
    return core_api 