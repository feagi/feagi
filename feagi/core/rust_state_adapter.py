"""Adapter layer between Python FeagiStateManager and Rust StateManager.

This module provides a thin delegation layer that routes performance-critical
state operations to Rust while keeping Python-specific orchestration logic in Python.

Architecture:
- Hot path (burst engine state, agents, locks) → Rust
- Python orchestration (API, transactions, debug) → Python
"""

import logging
from typing import Optional, Dict, Any
import feagi_rust_py_libs

logger = logging.getLogger(__name__)


class RustStateDelegate:
    """Delegates performance-critical state operations to Rust StateManager.
    
    This class provides a clean interface for Python code to access the Rust
    state manager without exposing Rust implementation details.
    """
    
    def __init__(self):
        """Initialize Rust state manager."""
        try:
            self._rust_state = feagi_rust_py_libs.feagi_python.StateManager()
            logger.info("✅ Rust StateManager initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize Rust StateManager: {e}")
            raise RuntimeError(f"Rust StateManager initialization failed: {e}")
    
    # ===== Burst Engine State (HOT PATH) =====
    
    def get_burst_engine_state(self) -> int:
        """Get burst engine state (0=Unavailable, 1=Initializing, 2=Ready, 3=Running, 4=Paused, 5=Error)."""
        return self._rust_state.get_burst_engine_state()
    
    def set_burst_engine_state(self, state: int) -> None:
        """Set burst engine state."""
        self._rust_state.set_burst_engine_state(state)
    
    def is_burst_engine_running(self) -> bool:
        """Check if burst engine is running (convenience method)."""
        return self.get_burst_engine_state() == 3
    
    # ===== Genome State (HOT PATH) =====
    
    def get_genome_state(self) -> int:
        """Get genome state (0=Missing, 1=Loading, 2=Loaded, 3=Saving, 4=Error)."""
        return self._rust_state.get_genome_state()
    
    def set_genome_state(self, state: int) -> None:
        """Set genome state."""
        self._rust_state.set_genome_state(state)
    
    def is_genome_loaded(self) -> bool:
        """Check if genome is loaded (convenience method)."""
        return self.get_genome_state() == 2
    
    # ===== Brain Readiness (HOT PATH) =====
    
    def is_brain_ready(self) -> bool:
        """Check if brain is ready."""
        return self._rust_state.is_brain_ready()
    
    def set_brain_ready(self, ready: bool) -> None:
        """Set brain readiness."""
        self._rust_state.set_brain_ready(ready)
    
    # ===== Agent Management (HOT PATH) =====
    
    def register_agent(self, agent_id: str, agent_type: str = "sensory") -> Dict[str, Any]:
        """Register an agent.
        
        Args:
            agent_id: Unique agent identifier
            agent_type: One of: sensory, motor, both, visualization, infrastructure
            
        Returns:
            Dict with 'success' (bool) and optional 'error' (str)
        """
        import json
        result = self._rust_state.register_agent(agent_id, agent_type)
        return json.loads(result)
    
    def deregister_agent(self, agent_id: str) -> Dict[str, Any]:
        """Deregister an agent.
        
        Returns:
            Dict with 'success' (bool) and optional 'error' (str)
        """
        import json
        result = self._rust_state.deregister_agent(agent_id)
        return json.loads(result)
    
    def get_agent_count(self) -> int:
        """Get number of registered agents."""
        return self._rust_state.get_agent_count()
    
    def get_all_agents(self) -> list:
        """Get all registered agents.
        
        Returns:
            List of dicts with agent info (agent_id, agent_type, registered_at, last_seen)
        """
        import json
        result = self._rust_state.get_all_agents()
        return json.loads(result)
    
    # ===== Cortical Locking (HOT PATH) =====
    
    def try_lock_cortical_area(self, cortical_area: int) -> bool:
        """Try to lock a cortical area for exclusive access.
        
        Returns:
            True if lock acquired, False if already locked
        """
        return self._rust_state.try_lock_cortical_area(cortical_area)
    
    def unlock_cortical_area(self, cortical_area: int) -> None:
        """Unlock a cortical area."""
        self._rust_state.unlock_cortical_area(cortical_area)
    
    # ===== FCL Cache (HOT PATH) =====
    
    def get_fcl_window(self, cortical_area: int) -> int:
        """Get FCL window size for cortical area."""
        return self._rust_state.get_fcl_window(cortical_area)
    
    def set_fcl_window(self, cortical_area: int, window_size: int) -> None:
        """Set FCL window size for cortical area."""
        self._rust_state.set_fcl_window(cortical_area, window_size)
    
    # ===== Persistence =====
    
    def save_state(self, path: str) -> None:
        """Save state to file."""
        self._rust_state.save_to_file(path)
    
    # ===== State Enums (for convenience) =====
    
    class BurstEngineState:
        UNAVAILABLE = 0
        INITIALIZING = 1
        READY = 2
        RUNNING = 3
        PAUSED = 4
        ERROR = 5
    
    class GenomeState:
        MISSING = 0
        LOADING = 1
        LOADED = 2
        SAVING = 3
        ERROR = 4


# Singleton instance
_rust_delegate: Optional[RustStateDelegate] = None


def get_rust_state_delegate() -> RustStateDelegate:
    """Get singleton Rust state delegate instance."""
    global _rust_delegate
    if _rust_delegate is None:
        _rust_delegate = RustStateDelegate()
    return _rust_delegate


