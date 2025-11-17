from typing import Any, Dict

from feagi.core.state_manager import FeagiStateManager
# BurstEngine has been moved to pure Rust - no Python wrapper needed
# Fire Ledger is now in Rust - accessed via rust_npu.get_fire_ledger_history()
# NOTE: Plasticity has been migrated to Rust (feagi-plasticity crate)
# Python plasticity service is disabled until Rust integration is complete
# from feagi.plasticity.service import PlasticityService, PlasticityConfig
# from feagi.plasticity.memory_neuron_array import MemoryNeuronArray, MemoryNeuronLifecycleConfig
from feagi.api.core.services.core_api_service import CoreAPIService


class CoreAPI(CoreAPIService):
    def __init__(self, connectome_manager, config: Dict[str, Any]):
        super().__init__(connectome_manager=connectome_manager, config=config)
        self._connectome_manager = connectome_manager
        self._config = config
        self._state_manager = FeagiStateManager.instance()
        # Burst engine is now pure Rust - no Python instance needed
        # Access via: process_manager.rust_npu_integration
        self._burst_engine = None  # Deprecated - kept for backward compatibility
        self._fcl_manager = None
        self._memory_manager = None
        self._plasticity_service = None

        # NOTE: Plasticity service disabled - migrated to Rust
        # Will be re-enabled when Rust plasticity is integrated with NPU

    # Exposed getters for ProcessManager
    def get_burst_engine(self):
        return self._burst_engine

    def get_connectome_manager(self):
        return self._connectome_manager

    # get_fcl_manager() inherited from CoreAPIService - uses FCLManagerAdapter

    def get_memory_manager(self):
        return self._memory_manager


def make_core_api(connectome_manager, config: Dict[str, Any]):
    """Factory function for CoreAPI to be re-exported as create_core_api."""
    return CoreAPI(connectome_manager, config)


