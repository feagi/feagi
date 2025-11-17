"""
ZMQ State Manager Proxy

Implements FeagiStateManager interface but forwards all queries to main
FEAGI process via ZMQ IPC. Used by API subprocess to access brain state
without shared memory.

Architecture:
- API Subprocess → This Proxy → ZMQ (port 5565) → Main Process Rust → Response
- Leverages existing ApiControlStream infrastructure
- Cross-platform (no shared memory dependency)
"""

import feagi_rust_py_libs.feagi_rust_py_libs as frl
from typing import Dict, Any
from feagi.core.state_errors import Result, StateError


class ZmqStateProxy:
    """Minimal state manager proxy that queries main process via ZMQ."""

    def __init__(self, zmq_address: str):
        """Initialize ZMQ state proxy.
        
        Args:
            zmq_address: ZMQ address of main process API control stream
                         (e.g., "tcp://127.0.0.1:5565")
        """
        print(f"🦀 [ZMQ-PROXY] Initializing proxy for {zmq_address}", flush=True)
        ZmqApiClient = frl.feagi_python.ZmqApiClient
        self._client = ZmqApiClient(zmq_address)
        self._client.connect()
        print("🦀 [ZMQ-PROXY] ✅ Connected to main process", flush=True)

    def get_brain_readiness(self) -> bool:
        """Query brain readiness from main process."""
        print("🦀 [ZMQ-PROXY] get_brain_readiness() called", flush=True)
        response = self._client.request("GET", "/internal/state/brain_readiness")
        result = response.get("body", {}).get("value", False)
        print(f"🦀 [ZMQ-PROXY] get_brain_readiness() → {result}", flush=True)
        return result

    def get_burst_engine_state(self) -> int:
        """Query burst engine state from main process.
        
        Returns:
            0: NOT_STARTED, 1: STARTING, 2: READY, 3: STOPPING, 4: STOPPED
        """
        print("🦀 [ZMQ-PROXY] get_burst_engine_state() called", flush=True)
        response = self._client.request("GET", "/internal/state/burst_engine_state")
        result = response.get("body", {}).get("value", 0)
        print(f"🦀 [ZMQ-PROXY] get_burst_engine_state() → {result}", flush=True)
        return result

    def get_genome_state(self) -> int:
        """Query genome state from main process.
        
        Returns:
            0: NOT_LOADED, 1: LOADING, 2: LOADED
        """
        print("🦀 [ZMQ-PROXY] get_genome_state() called", flush=True)
        response = self._client.request("GET", "/internal/state/genome_state")
        result = response.get("body", {}).get("value", 0)
        print(f"🦀 [ZMQ-PROXY] get_genome_state() → {result}", flush=True)
        return result

    def get_brain_stats(self) -> Dict[str, Any]:
        """Query brain statistics from main process."""
        print("🦀 [ZMQ-PROXY] get_brain_stats() called", flush=True)
        response = self._client.request("GET", "/internal/state/brain_stats")
        result = response.get("body", {})
        print(f"🦀 [ZMQ-PROXY] get_brain_stats() → {result}", flush=True)
        return result

    def is_genome_loaded(self) -> bool:
        """Check if genome is loaded."""
        print("🦀 [ZMQ-PROXY] is_genome_loaded() called", flush=True)
        genome_state = self.get_genome_state()
        return genome_state == 2  # 2 = LOADED

    def get_genome_timestamp(self) -> int:
        """Get genome timestamp - stub (not critical for subprocess)."""
        return 0

    def get_genome_counter(self) -> int:
        """Get genome counter - stub (not critical for subprocess)."""
        return 0

    def get_feagi_session_timestamp(self) -> int:
        """Get FEAGI session timestamp - stub."""
        return 0

    # Stub attributes that health check accesses
    @property
    def connected_agents(self) -> Dict:
        return {}

    @property
    def influxdb(self) -> bool:
        return False

    @property
    def genome_fitness(self) -> float:
        return 0.0

    @property
    def changes_saved_externally(self) -> bool:
        return False

    # Stub methods that aren't needed for read-only API queries
    def is_debug_api_enabled(self) -> bool:
        return False

    def get_api_state(self) -> int:
        return 0

    def get_connectome_state(self) -> int:
        return 2  # 2 = READY (assume ready if genome loaded)

    def get_fq_sampler_state(self) -> int:
        return 0

    def get_exit_condition(self) -> bool:
        return False
    
    # Setter methods (no-op in subprocess, state is read-only)
    def set_api_state(self, state) -> Result:
        return Result.ok(None)  # No-op in API subprocess (read-only proxy)
    
    def set_burst_engine_state(self, state) -> Result:
        return Result.ok(None)
    
    def set_genome_state(self, state) -> Result:
        return Result.ok(None)
    
    def set_brain_stats(self, stats) -> Result:
        return Result.ok(None)
    
    def set_genome_timestamp(self, timestamp) -> Result:
        return Result.ok(None)
    
    def increment_genome_counter(self) -> Result:
        return Result.ok(None)
    
    def set_cortical_list(self, cortical_ids) -> Result:
        return Result.ok(None)
    
    def set_connected_agents(self, agents) -> Result:
        return Result.ok(None)
    
    def set_changes_saved_externally(self, value: bool) -> Result:
        return Result.ok(None)
    
    def set_exit_condition(self, value: bool) -> Result:
        return Result.ok(None)
    
    def set_genome_validity(self, value: bool) -> Result:
        return Result.ok(None)
    
    # Additional stubs for validation methods
    def _validate_state_consistency(self) -> bool:
        return True
    
    def _validate_connectome_ready(self) -> bool:
        return True
    
    def _validate_connectome_stable(self) -> bool:
        return True
    
    # Additional attributes/methods that might be accessed
    @property
    def _state(self):
        """Stub for direct state access."""
        class StubState:
            synapse_count = 0
            neuron_count = 0
        return StubState()
    
    def __getattr__(self, name):
        """Catch-all for any missing methods/attributes - return no-op function or safe default."""
        print(f"🦀 [ZMQ-PROXY] ⚠️ Accessing unimplemented attribute: {name}", flush=True)
        
        # If it looks like a setter (starts with "set_" or "increment_"), return Result.ok(None)
        if name.startswith("set_") or name.startswith("increment_"):
            return lambda *args, **kwargs: Result.ok(None)
        
        # If it looks like a getter (starts with "get_"), return safe default
        if name.startswith("get_"):
            return lambda *args, **kwargs: None
        
        # For boolean checks (starts with "is_"), return False
        if name.startswith("is_"):
            return lambda *args, **kwargs: False
        
        # For other attributes, return None
        return None

