"""
FEAGI CLI tools.

Command-line utilities for agent development and runtime helpers.

Commands:
- feagi bv start: Launch Brain Visualizer using configuration
- feagi create-agent: Scaffold new agent from template (Phase 4)
- feagi build-package: Build marketplace package locally (Phase 4)
"""

from feagi.cli.main import main

__all__ = ["main"]

