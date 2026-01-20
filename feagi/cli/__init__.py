"""
FEAGI CLI tools.

Command-line utilities for agent development and runtime helpers.

Commands:
- feagi bv start: Launch Brain Visualizer using configuration
- feagi bv stop: Stop running Brain Visualizer process
- feagi bv status: Check Brain Visualizer process status
- feagi bv restart: Restart Brain Visualizer process
- feagi create-agent: Scaffold new agent from template (Phase 4)
- feagi build-package: Build marketplace package locally (Phase 4)
"""

from feagi.cli.main import main

__all__ = ["main"]

