from typing import Any, Dict

from feagi.core.state_manager import FeagiStateManager
from feagi.npu.burst_engine import BurstEngine
# Fire Ledger is now in Rust - accessed via rust_npu.get_fire_ledger_history()
from feagi.plasticity.service import PlasticityService, PlasticityConfig
from feagi.plasticity.memory_neuron_array import MemoryNeuronArray, MemoryNeuronLifecycleConfig
from feagi.api.core.services.core_api_service import CoreAPIService


class CoreAPI(CoreAPIService):
    def __init__(self, connectome_manager, config: Dict[str, Any]):
        super().__init__(connectome_manager=connectome_manager, config=config)
        self._connectome_manager = connectome_manager
        self._config = config
        self._state_manager = FeagiStateManager.instance()
        self._burst_engine = BurstEngine.get_instance(connectome_manager=connectome_manager, state_manager=self._state_manager)
        self._fcl_manager = None
        self._memory_manager = None
        self._plasticity_service = None

        # Configure PlasticityService from TOML (no fallbacks)
        p_cfg = config.get('plasticity') if isinstance(config, dict) else None
        if isinstance(p_cfg, dict):
            required = ['queue_capacity', 'max_ops_per_burst', 'stdp', 'memory']
            if all(k in p_cfg for k in required):
                svc_cfg = PlasticityConfig(
                    queue_capacity=int(p_cfg['queue_capacity']),
                    max_ops_per_burst=int(p_cfg['max_ops_per_burst']),
                    stdp=dict(p_cfg['stdp']),
                    memory=dict(p_cfg['memory']),
                )
                # Start service
                import sys
                debug_mem = '--debug-mem' in sys.argv
                
                npu_interface = getattr(connectome_manager, '_npu_interface', None)
                # Fire Ledger is now in Rust - PlasticityService will be updated to use Rust API
                # For now, pass None to prevent import errors
                fire_ledger = None  # TODO: Update PlasticityService to use rust_npu.get_fire_ledger_history()
                if npu_interface and isinstance(svc_cfg.queue_capacity, int):
                    if debug_mem:
                        print(f"[DEBUG-MEM] Initializing PlasticityService...")
                        print(f"[DEBUG-MEM] WARNING: Fire Ledger is None - Plasticity needs Rust integration update")
                    
                    svc = PlasticityService(
                        fire_ledger=fire_ledger,
                        npu_interface=npu_interface,
                        plasticity_config=svc_cfg,
                        state_manager=self._state_manager,
                    )
                    svc.start()
                    self._plasticity_service = svc
                    # Link to BurstEngine for per-burst notification
                    setattr(self._burst_engine, '_plasticity_service', svc)
                    
                    if debug_mem:
                        print(f"[DEBUG-MEM] ✅ PlasticityService initialized and linked to BurstEngine")
                elif debug_mem:
                    print(f"[DEBUG-MEM] ❌ Failed to initialize PlasticityService - npu_interface: {npu_interface is not None}, queue_capacity: {svc_cfg.queue_capacity}")

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


