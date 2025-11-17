from typing import Any, Dict

from feagi.core.state_manager import FeagiStateManager
from feagi.npu.burst_engine import BurstEngine
# NOTE: Plasticity has been migrated to Rust (feagi-plasticity crate)
# Python plasticity service is disabled until Rust integration is complete
# from feagi.plasticity.service import PlasticityService, PlasticityConfig


class CoreAPI:
    def __init__(self, connectome_manager, config: Dict[str, Any]):
        self._connectome_manager = connectome_manager
        self._config = config
        self._state_manager = FeagiStateManager.instance()
        self._burst_engine = BurstEngine.get_instance(connectome_manager=connectome_manager, state_manager=self._state_manager)
        self._fcl_manager = None
        self._memory_manager = None
        self._plasticity_service = None

        # NOTE: Plasticity service disabled - migrated to Rust
        # Will be re-enabled when Rust plasticity is integrated with NPU
        # Configure PlasticityService from TOML (no fallbacks)
        # p_cfg = config.get('plasticity') if isinstance(config, dict) else None
        # if isinstance(p_cfg, dict):
        #     required = ['queue_capacity', 'max_ops_per_burst', 'stdp', 'memory']
        #     if all(k in p_cfg for k in required):
        #         svc_cfg = PlasticityConfig(
        #             queue_capacity=int(p_cfg['queue_capacity']),
        #             max_ops_per_burst=int(p_cfg['max_ops_per_burst']),
        #             stdp=dict(p_cfg['stdp']),
        #             memory=dict(p_cfg['memory']),
        #         )
        #         # Start service
        #         npu_interface = getattr(connectome_manager, '_npu_interface', None)
        #         fire_ledger = self._burst_engine.get_fire_ledger()
        #         if npu_interface and isinstance(svc_cfg.queue_capacity, int):
        #             svc = PlasticityService(
        #                 fire_ledger=fire_ledger,
        #                 npu_interface=npu_interface,
        #                 plasticity_config=svc_cfg,
        #                 state_manager=self._state_manager,
        #             )
        #             svc.start()
        #             self._plasticity_service = svc
        #             # Link to BurstEngine for per-burst notification
        #             setattr(self._burst_engine, '_plasticity_service', svc)

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


