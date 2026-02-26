"""
FEAGI Python SDK

Complete SDK for building FEAGI agents, controlling the neural engine,
and creating marketplace packages.

Version 3.0.0 - Clean architecture, no legacy code.

Main modules:
- feagi.engine: Start/stop FEAGI neural engine
- feagi.agent: Agent framework (BaseAgent templates)
- feagi.config: Configuration management and generation
- feagi.paths: Cross-platform directory management
- feagi.genome: Runtime genome manipulation
- feagi.connectome: Runtime connectome operations
- feagi.packaging: Build marketplace packages
- feagi.pns: Peripheral Nervous System (communication layer)
- feagi.cli: Command-line tools
"""

__version__ = "2.1.27"

# Import key classes for convenience, but keep optional dependencies lazy so
# `import feagi` does not require platform-specific extras (e.g., pyserial).
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from feagi.agent import BaseAgent, VideoStreamAgent
    from feagi.engine import FeagiEngine
    from feagi.pns import AgentType, FeagiAgentClient

__all__ = [
    "FeagiAgentClient",
    "AgentType",
    "BaseAgent",
    "VideoStreamAgent",
    "FeagiEngine",
    "__version__",
]

def __getattr__(name: str):
    """Lazy attribute access to avoid optional imports at import time."""
    if name in {"FeagiAgentClient", "AgentType"}:
        from feagi.pns import AgentType as _AgentType
        from feagi.pns import FeagiAgentClient as _FeagiAgentClient

        return _FeagiAgentClient if name == "FeagiAgentClient" else _AgentType
    if name in {"BaseAgent", "VideoStreamAgent"}:
        from feagi.agent import BaseAgent as _BaseAgent
        from feagi.agent import VideoStreamAgent as _VideoStreamAgent

        return _BaseAgent if name == "BaseAgent" else _VideoStreamAgent
    if name == "FeagiEngine":
        from feagi.engine import FeagiEngine as _FeagiEngine

        return _FeagiEngine
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

def check_rust_sdk():
    """Check if Rust SDK is installed and print status"""
    try:
        _ = __getattr__("FeagiAgentClient")
        print("[OK] FEAGI Rust SDK is available")
    except Exception:
        print("[WARN]  FEAGI Rust SDK not installed")
        print("   Install with: pip install feagi_rust_py_libs")

