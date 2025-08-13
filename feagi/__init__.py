"""FEAGI: Framework for Evolutionary Artificial General Intelligence.

This package provides tools and utilities for creating, training, and deploying
artificial general intelligence models that evolve over time.

The main components are:
- API server: REST API for external communication
- ZMQ server: Messaging system for internal component communication
- Core modules: Neural processing, resource management, and other foundational components
"""

__version__ = "0.1.0"

# Import key components for easy access
from feagi.core.resource_mgr import ResourceManager


# Create a factory function to initialize a complete FEAGI system
def create_feagi(config=None):
    """
    Initialize a complete FEAGI system.

    This is a convenience function that initializes core components and returns
    a dictionary with references to them.

    Args:
        config: Optional configuration dictionary

    Returns:
        Dictionary with references to core FEAGI components
    """
    # Initialize resource manager
    resource_mgr = ResourceManager(config)

    # Return references to core components
    return {
        "resource_mgr": resource_mgr,
        "version": __version__,
        "config": config or {},
    }
