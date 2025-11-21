"""
FEAGI Python SDK

Complete SDK for building FEAGI agents, controlling the neural engine,
and creating marketplace packages.

Version 3.0.0 - Clean architecture, no legacy code.

Main modules:
- feagi.engine: Start/stop FEAGI neural engine
- feagi.agent: Agent framework (BaseAgent templates)
- feagi.genome: Runtime genome manipulation
- feagi.connectome: Runtime connectome operations
- feagi.packaging: Build marketplace packages
- feagi.pns: Peripheral Nervous System (communication layer)
- feagi.cli: Command-line tools
"""

__version__ = "2.0.1"

# Import key classes for convenience
# Note: FeagiAgentClient requires feagi_rust_py_libs to be installed
try:
    from feagi.pns import FeagiAgentClient, AgentType
    _pns_available = True
except ImportError:
    _pns_available = False
    FeagiAgentClient = None
    AgentType = None

from feagi.agent import BaseAgent, VideoStreamAgent
from feagi.engine import FeagiEngine

__all__ = [
    "FeagiAgentClient",
    "AgentType",
    "BaseAgent",
    "VideoStreamAgent",
    "FeagiEngine",
    "__version__",
]

def check_rust_sdk():
    """Check if Rust SDK is installed and print status"""
    if _pns_available:
        print("✅ FEAGI Rust SDK is available")
    else:
        print("⚠️  FEAGI Rust SDK not installed")
        print("   Install with: pip install feagi_rust_py_libs")

