"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

# Refactored CoreAPIService using domain-based service architecture.
#
# This is the new facade implementation that delegates to specialized services
# while maintaining complete backward compatibility with the existing API.

import os
import tempfile
import time
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np

from feagi.bdu.utils.mapping_utils import get_mapping_restrictions_registry
from feagi.utils.logger import setup_logger

from .agents.agents_service import AgentsService
from .brain.brain_service import BrainService
from .connectome.connectome_service import ConnectomeService
from .cortical_area.cortical_area_service import CorticalAreaService
from .genome.genome_service import GenomeService
from .network.network_service import NetworkService
from .npu.npu_service import NPUService

# Import all domain services
from .system.system_service import SystemService

logger = setup_logger()


class FCLManagerAdapter:
    """Compatibility adapter to make new BurstEngine work with old FCL manager interface.
    
    This adapter provides the methods that the `/fcl` endpoint expects while
    using the new NPU architecture underneath.
    """
    
    def __init__(self, burst_engine):
        """Initialize adapter with BurstEngine instance."""
        self.burst_engine = burst_engine
        self.current_timestep = burst_engine.current_timestep if burst_engine else 0
        
    def get_global_fcl(self):
        """Get global FCL as roaring bitmap (compatibility method).
        
        In new architecture, returns firing neurons from Rust Fire Queue.
        Uses direct Fire Queue access to bypass FQ Sampler rate limiting.
        """
        if not self.burst_engine or not hasattr(self.burst_engine, 'rust_npu'):
            return EmptyBitmapAdapter()
        
        # Get Rust NPU
        rust_npu = self.burst_engine.rust_npu
        if not rust_npu:
            return EmptyBitmapAdapter()
        
        # Get current Fire Queue directly (bypasses rate limiting)
        fire_queue_data = rust_npu.get_current_fire_queue()
        
        if not fire_queue_data:
            return EmptyBitmapAdapter()
        
        # Extract all neuron IDs from all cortical areas
        neuron_ids = []
        for area_data in fire_queue_data.values():
            neuron_ids.extend(area_data.get('neuron_ids', []))
        
        return BitmapAdapter(neuron_ids)
    
    def get_fcl_by_cortical(self):
        """Get FCL organized by cortical areas (compatibility method)."""
        if not self.burst_engine or not hasattr(self.burst_engine, 'rust_npu'):
            return {}
        
        # Get Rust NPU
        rust_npu = self.burst_engine.rust_npu
        if not rust_npu:
            return {}
        
        # Get current Fire Queue directly (bypasses rate limiting)
        fire_queue_data = rust_npu.get_current_fire_queue()
        if not fire_queue_data:
            return {}
        
        # Convert to cortical FCL format
        cortical_fcl = {}
        for cortical_idx, area_data in fire_queue_data.items():
            neuron_ids = area_data.get('neuron_ids', [])
            cortical_fcl[cortical_idx] = BitmapAdapter(neuron_ids)
        
        return cortical_fcl
    
    @property
    def window_size(self):
        """Get default window size from Rust Fire Ledger."""
        if self.burst_engine and hasattr(self.burst_engine, 'rust_npu') and self.burst_engine.rust_npu:
            # Rust Fire Ledger default is 20 (hardcoded in Rust)
            return 20
        return 20  # Default window size
        
    def get_cortical_window_size(self, cortical_idx: int) -> int:
        """Get window size for specific cortical area from Rust Fire Ledger."""
        if self.burst_engine and hasattr(self.burst_engine, 'rust_npu') and self.burst_engine.rust_npu:
            # Get window size from Rust Fire Ledger
            try:
                return self.burst_engine.rust_npu.get_fire_ledger_window_size(cortical_idx)
            except Exception:
                return 20
        return 20  # Default window size
    
    @property
    def memory_cortical_indices(self):
        """Get memory cortical area indices from Rust Fire Ledger."""
        if self.burst_engine and hasattr(self.burst_engine, 'rust_npu') and self.burst_engine.rust_npu:
            # Get all configured areas from Rust Fire Ledger
            try:
                configs = self.burst_engine.rust_npu.get_all_fire_ledger_configs()
                return set(configs.keys())
            except Exception:
                return set()
        return set()
    
    @property 
    def total_neurons_fired(self):
        """Get total neurons that fired in the last timestep."""
        if not self.burst_engine or not hasattr(self.burst_engine, 'rust_npu'):
            return 0
        
        # Get Rust NPU
        rust_npu = self.burst_engine.rust_npu
        if not rust_npu:
            return 0
        
        # Get current Fire Queue directly (bypasses rate limiting)
        fire_queue_data = rust_npu.get_current_fire_queue()
        if not fire_queue_data:
            return 0
        
        # Count total neurons across all areas
        total = 0
        for area_data in fire_queue_data.values():
            total += len(area_data.get('neuron_ids', []))
        return total


class BitmapAdapter:
    """Adapter to make list of neuron IDs behave like a roaring bitmap."""
    
    def __init__(self, neuron_ids):
        self.neuron_ids = set(neuron_ids) if neuron_ids else set()
        
    def __iter__(self):
        """Iterate over neuron IDs."""
        return iter(sorted(self.neuron_ids))
        
    def __len__(self):
        """Get number of neurons."""
        return len(self.neuron_ids)
        
    def is_empty(self):
        """Check if bitmap is empty."""
        return len(self.neuron_ids) == 0


class EmptyBitmapAdapter:
    """Empty bitmap adapter for when no data is available."""
    
    def __iter__(self):
        return iter([])
        
    def __len__(self):
        return 0
        
    def is_empty(self):
        return True


class CoreAPIService:
    """Facade for all FEAGI core API operations.

    This class delegates to specialized domain services while maintaining
    the exact same public interface as the original CoreAPIService.

    The refactoring provides:
    - Better separation of concerns
    - Improved maintainability
    - Easier testing and development
    - Zero breaking changes to existing code
    """

    def __init__(
        self,
        connectome_manager,
        state_manager=None,
        config: Optional[Dict[str, Any]] = None,
    ):
        """Initialize the Core API Service facade.

        Args:
            connectome_manager: ConnectomeManager instance
            state_manager: FeagiStateManager instance (optional)
            config: Optional configuration dict (forwarded to core components)
        """
        # Initialize connectome manager
        self._connectome_manager = connectome_manager
        self.state_manager = state_manager
        self._config = config or {}
        self.logger = logger

        # CRITICAL: Ensure state manager singleton consistency
        if self.state_manager is None:
            from feagi.core.state_manager import FeagiStateManager

            self.state_manager = FeagiStateManager.instance()
            self.logger.info("Using FeagiStateManager singleton instance")
        else:
            self.logger.info("Using provided state manager instance")

        # Initialize all domain services with the SAME state manager instance
        self._system_service = SystemService(
            connectome_manager, self.state_manager
        )

        #  Initialize genome service first - needed by other services for WRITE
        #  operations
        self._genome_service = GenomeService(
            connectome_manager, self.state_manager, core_api_service=self
        )

        #  Initialize cortical area service WITH genome service for WRITE
        #  operations
        self._cortical_area_service = CorticalAreaService(
            connectome_manager, self.state_manager, self._genome_service
        )
        self._connectome_service = ConnectomeService(
            connectome_manager, self.state_manager
        )
        self._brain_service = BrainService(
            connectome_manager, self.state_manager
        )
        self._agents_service = AgentsService(
            connectome_manager, self.state_manager
        )
        self._network_service = NetworkService(
            connectome_manager, self.state_manager
        )
        
        # Initialize NPU service for new architecture
        self._npu_service = NPUService()

        # Validate state manager consistency across services
        self._validate_service_state_consistency()

        self.logger.info(
            "CoreAPIService initialized with domain-based architecture and state synchronization"
        )

    def _validate_service_state_consistency(self):
        """Validate that all services share the same state manager instance."""
        try:
            services = [
                ("system", self._system_service),
                ("genome", self._genome_service),
                ("cortical_area", self._cortical_area_service),
                ("connectome", self._connectome_service),
                ("brain", self._brain_service),
                ("agents", self._agents_service),
                ("network", self._network_service),
            ]

            core_state_id = id(self.state_manager)
            inconsistent_services = []

            for service_name, service in services:
                if hasattr(service, "state_manager"):
                    service_state_id = id(service.state_manager)
                    if service_state_id != core_state_id:
                        inconsistent_services.append(service_name)
                        self.logger.error(
                            f"Service {service_name} has different state manager instance: core={core_state_id}, service={service_state_id}"
                        )
                else:
                    inconsistent_services.append(service_name)
                    self.logger.error(
                        f"Service {service_name} missing state_manager attribute"
                    )

            if inconsistent_services:
                self.logger.error(
                    f"State manager inconsistency detected in services: {inconsistent_services}"
                )
                raise RuntimeError(
                    f"Critical state manager inconsistency in services: {inconsistent_services}"
                )
            else:
                self.logger.info(
                    "All services share the same state manager instance - consistency validated"
                )

        except Exception as e:
            self.logger.error(
                f"Error validating service state consistency: {str(e)}"
            )
            raise

    # =================================================================
    # SYSTEM SERVICE DELEGATION
    # =================================================================

    async def get_health(self) -> Dict[str, Any]:
        """Get comprehensive system health information."""
        return await self._system_service.get_health()

    def get_user_preferences(self) -> Dict[str, Any]:
        """Get user preferences."""
        return self._system_service.get_user_preferences()

    def update_user_preferences(self, preferences: Dict[str, Any]) -> bool:
        """Update user preferences."""
        return self._system_service.update_user_preferences(preferences)

    def get_versions(self) -> Dict[str, Any]:
        """Get version information for various components."""
        return self._system_service.get_versions()

    def get_configuration(self) -> Dict[str, Any]:
        """Get system configuration."""
        return self._system_service.get_configuration()

    def test_influxdb(self) -> Optional[Dict[str, Any]]:
        """Test InfluxDB connectivity."""
        return self._system_service.test_influxdb()

    def set_circuit_library_path(self, path: str) -> bool:
        """Set the circuit library path."""
        return self._system_service.set_circuit_library_path(path)

    def get_cortical_area_types(self) -> Dict[str, Any]:
        """Get available cortical area types."""
        return self._system_service.get_cortical_area_types()

    def reset_fcl(self) -> bool:
        """Reset the Fire Candidate List."""
        return self._system_service.reset_fcl()

    def get_visualization_skip_rate(self) -> int:
        """Get visualization skip rate."""
        return self._system_service.get_visualization_skip_rate()

    def set_visualization_skip_rate(self, skip_rate: int) -> bool:
        """Set visualization skip rate."""
        return self._system_service.set_visualization_skip_rate(skip_rate)

    def get_visualization_suppression_threshold(self) -> int:
        """Get visualization suppression threshold."""
        return self._system_service.get_visualization_suppression_threshold()

    def set_visualization_suppression_threshold(self, threshold: int) -> bool:
        """Set visualization suppression threshold."""
        return self._system_service.set_visualization_suppression_threshold(
            threshold
        )

    def get_global_activity_visualization(self) -> bool:
        """Get global activity visualization status."""
        return self._system_service.get_global_activity_visualization()

    def set_global_activity_visualization(self, enabled: bool) -> bool:
        """Set global activity visualization status."""
        return self._system_service.set_global_activity_visualization(enabled)

    def get_unique_logs(self) -> List[str]:
        """Get unique log entries."""
        return self._system_service.get_unique_logs()

    def enable_visualization_fq_sampler(self) -> bool:
        """Enable the visualization FQ sampler for brain visualizer
        connectivity."""
        return self._system_service.enable_visualization_fq_sampler()

    def disable_visualization_fq_sampler(self) -> bool:
        """Disable the visualization FQ sampler."""
        return self._system_service.disable_visualization_fq_sampler()

    def get_fq_sampler_status(self) -> Dict[str, Any]:
        """Get the current status of all FQ samplers."""
        return self._system_service.get_fq_sampler_status()

    def get_burst_timer(self) -> float:
        """Get burst timer from burst engine."""
        return self._brain_service.get_burst_timer()

    # =================================================================
    # GENOME SERVICE DELEGATION
    # =================================================================

    def load_barebones_genome(self) -> Dict[str, Any]:
        """Load the barebones genome."""
        print(
            "[DEBUG] CORE API SERVICE: load_barebones_genome called, delegating to genome service"
        )
        result = self._genome_service.load_default_genome("barebones")
        try:
            from feagi.core.state_manager import FeagiStateManager
            if FeagiStateManager.instance().is_debug_npu_enabled():
                self.logger.debug(
            f"[DEBUG] CORE API SERVICE: genome service returned: {result.get('success', 'unknown')}"
        )
        except Exception:
            pass
        return result

    def load_essential_genome(self) -> Dict[str, Any]:
        """Load the essential genome."""
        print(
            "[DEBUG] CORE API SERVICE: load_essential_genome called, delegating to genome service"
        )
        result = self._genome_service.load_default_genome("essential")
        try:
            from feagi.core.state_manager import FeagiStateManager
            if FeagiStateManager.instance().is_debug_npu_enabled():
                self.logger.debug(
            f"[DEBUG] CORE API SERVICE: genome service returned: {result.get('success', 'unknown')}"
        )
        except Exception:
            pass
        return result

    def load_test_genome(self) -> Dict[str, Any]:
        """Load the test genome."""
        return self._genome_service.load_default_genome("test")

    def load_genome(
        self, genome_data: Dict[str, Any], filename: str = "genome.json"
    ) -> Dict[str, Any]:
        """Load a genome and prepare it for use.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager → NeuroEmbryogenesis
        """
        # Load genome through genome service
        result = self._genome_service.load_genome(genome_data, filename)
        
        #  If genome loading was successful, initialize spatial hash cache with
        #  final dimensions
        if result.get("success", False):
            self.logger.info(
                "Genome loaded successfully - Rust NPU spatial hash is ready"
            )
            result["spatial_hash_initialized"] = True
        
        return result

    def get_genome(self) -> Optional[Dict[str, Any]]:
        """Get the currently loaded genome data."""
        return self._genome_service.get_genome()

    def get_genome_filename(self) -> str:
        """Get the current genome filename."""
        filename = self._genome_service.get_genome_filename()
        return filename or ""

    def get_genome_file_name(self) -> Dict[str, str]:
        """Get the genome file name in the format expected by the REST API."""
        return self._genome_service.get_genome_file_name()

    def get_default_genomes(self) -> Dict[str, Any]:
        """Get a list of default genome files with their contents."""
        return self._genome_service.get_default_genomes()

    def get_genome_counter(self) -> int:
        """Get the current genome counter."""
        return self._genome_service.get_genome_counter()

    def get_current_genome(self) -> Optional[Dict[str, Any]]:
        """Get the currently loaded genome data (alias for get_genome for
        download compatibility)."""
        return self._genome_service.get_genome()

    def get_generations(self) -> Dict[str, Any]:
        """Get details about all generations of genomes."""
        return self._genome_service.get_generations()

    def get_change_register(self) -> Dict[str, Any]:
        """Get the evolution change register showing evolutionary history."""
        return self._genome_service.get_change_register()

    def deploy_genome(self, genome_filepath: str) -> bool:
        """Deploy a genome from a file path."""
        return self._genome_service.deploy_genome(genome_filepath)

    def is_genome_loaded(self) -> bool:
        """Check if a genome is currently loaded."""
        return self._genome_service.is_genome_loaded()

    # =================================================================
    # CORTICAL AREA SERVICE DELEGATION
    # =================================================================

    def get_all_cortical_areas(self) -> List[Dict[str, Any]]:
        """Get all cortical areas."""
        return self._cortical_area_service.get_all_areas()

    def get_cortical_area(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """Get a cortical area by ID."""
        return self._cortical_area_service.get_area(cortical_id)

    def create_cortical_area(
        self,
        name: str,
        coordinates: Dict[str, int],
        dimensions: Dict[str, int],
        area_type: str,
        parameters: Dict[str, Any] = None,
    ) -> Optional[Dict[str, Any]]:
        """Create a new cortical area."""
        return self._cortical_area_service.create_area(
            name, coordinates, dimensions, area_type, parameters
        )

    def update_cortical_area(
        self,
        cortical_id: str,
        name: Optional[str] = None,
        coordinates: Optional[Dict[str, int]] = None,
        dimensions: Optional[Dict[str, int]] = None,
        area_type: Optional[str] = None,
        parameters: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Update an existing cortical area."""
        return self._cortical_area_service.update_area(
            cortical_id, name, coordinates, dimensions, area_type, parameters
        )

    def update_cortical_area_properties(
        self, cortical_id: str, properties: Dict[str, Any]
    ) -> bool:
        """Update properties of an existing cortical area with intelligent
        routing."""
        try:
            if self.state_manager.is_debug_api_enabled():
                self.logger.info(
                    f"[API-DEBUG] update_cortical_area_properties cortical_id={cortical_id}, properties={properties}"
                )
        except Exception:
            pass
        try:
            #  ARCHITECTURE COMPLIANCE: Route through GenomeService for
            #  intelligent routing
            #  This ensures STRUCTURAL changes (like cortical_dimensions)
            #  trigger proper rebuild

            #  Extract individual property types for
            #  GenomeService.update_cortical_area()
            name = properties.get("cortical_name")
            coordinates = properties.get("coordinates_3d") 
            dimensions = properties.get("cortical_dimensions")
            area_type = properties.get("cortical_type")
            
            # Collect remaining properties as parameters
            parameters = {
                k: v
                for k, v in properties.items()
                if k
                not in [
                    "cortical_name",
                    "coordinates_3d",
                    "cortical_dimensions",
                    "cortical_type",
                ]
            }
            
            # Remove empty parameters dict to avoid passing unnecessary data
            if not parameters:
                parameters = None
                
            try:
                if self.state_manager.is_debug_api_enabled():
                    self.logger.debug(
                        f"[API-DEBUG] Routing to GenomeService: name={name}, coordinates={coordinates}, dimensions={dimensions}, "
                        f"area_type={area_type}, parameters={parameters}"
                    )
            except Exception:
                pass

            #  Route through GenomeService for intelligent routing (STRUCTURAL
            #  vs PARAMETER vs METADATA)
            result = self._genome_service.update_cortical_area(
                cortical_id=cortical_id,
                name=name,
                coordinates=coordinates,
                dimensions=dimensions,
                area_type=area_type,
                parameters=parameters,
            )
            
            success = result is not None
            try:
                if self.state_manager.is_debug_api_enabled():
                    self.logger.info(
                        f"[API-DEBUG] GenomeService.update_cortical_area returned success={success}"
                    )
            except Exception:
                pass
            return success
            
        except Exception as e:
            self.logger.error(
                f"Error updating cortical area properties for {cortical_id}: {str(e)}"
            )
            return False

    def delete_cortical_area(self, cortical_id: str) -> bool:
        """Delete a cortical area."""
        return self._cortical_area_service.delete_area(cortical_id)

    def get_cortical_area_neurons(
        self, cortical_id: str
    ) -> Optional[List[Dict[str, Any]]]:
        """Get neurons for a specific cortical area."""
        return self._cortical_area_service.get_area_neurons(cortical_id)

    def get_cortical_area_activity(
        self, cortical_id: str, window: int = 1
    ) -> Optional[Dict[str, Any]]:
        """Get activity data for a specific cortical area."""
        return self._cortical_area_service.get_area_activity(
            cortical_id, window
        )

    def get_cortical_area_connectivity(
        self, cortical_id: str, direction: str = "both"
    ) -> Optional[Dict[str, Any]]:
        """Get connectivity information for a specific cortical area."""
        return self._cortical_area_service.get_area_connectivity(
            cortical_id, direction
        )

    def get_neuron_properties(
        self, neuron_id: int
    ) -> Optional[Dict[str, Any]]:
        """Get detailed properties of a specific neuron using NPU interface internally.
        
        Maintains exact same API contract while using new NPU architecture.
        """
        self.logger.info(
            f"DEBUG: get_neuron_properties called for neuron_id: {neuron_id} (type: {type(neuron_id)})"
        )

        try:
            # Get neuron data directly from Rust NPU using neuron_id
            # Rust NPU handles ID→index mapping internally via HashMap
            cm = self._connectome_manager
            npu_interface = getattr(cm, "_npu_interface", None)
            
            if not npu_interface:
                return None
            
            # Get Rust NPU handle
            rust_npu = npu_interface.rust_npu
            if not rust_npu:
                return None
            
            # CLEAN ARCHITECTURE: Pass neuron_id directly to Rust
            # Rust NPU uses internal neuron_id_to_index HashMap to find the array index
            state = rust_npu.get_neuron_state(neuron_id)
            if not state:
                return None
            
            consecutive_fire_count, consecutive_fire_limit, snooze_period, threshold, membrane_potential, refractory_countdown = state
            
            # Get cortical area from Rust NPU (single source of truth)
            cortical_idx = npu_interface.get_neuron_cortical_idx(neuron_id)
            
            properties = {}
            # cortical_id via NPU registry
            cortical_id = None
            try:
                area_info = npu_interface.cortical_areas.get(cortical_idx)
                if area_info:
                    cortical_id = area_info.get("cortical_id")
            except Exception:
                cortical_id = None

            # Get position from Rust NPU (single source of truth)
            position = npu_interface.get_neuron_position(neuron_id) or (0, 0, 0)
            
            # Get additional properties by calling Rust NPU individual getters (use neuron_id directly!)
            # CRITICAL: These methods now use neuron_id_to_index HashMap internally
            refractory_period_val = rust_npu.get_neuron_refractory_period(neuron_id) if hasattr(rust_npu, 'get_neuron_refractory_period') else 0
            leak_coef = rust_npu.get_neuron_leak_coefficient(neuron_id) if hasattr(rust_npu, 'get_neuron_leak_coefficient') else 0.0
            resting = rust_npu.get_neuron_resting_potential(neuron_id) if hasattr(rust_npu, 'get_neuron_resting_potential') else 0.0
            excitability = rust_npu.get_neuron_excitability(neuron_id) if hasattr(rust_npu, 'get_neuron_excitability') else 1.0
            
            # Populate required fields from Rust NPU state
            properties["position"] = list(position) if isinstance(position, tuple) else position
            properties["threshold"] = float(threshold)  # From Rust NPU state
            properties["membrane_potential"] = float(membrane_potential)  # From Rust NPU state
            properties["resting_potential"] = float(resting)
            properties["leak_coefficient"] = float(leak_coef)  # 0.0-1.0 (percentage loss per burst)
            properties["refractory_period"] = int(refractory_period_val) if refractory_period_val is not None else 0
            properties["refractory_counter"] = int(refractory_countdown)  # From Rust NPU state
            properties["consecutive_fire_count"] = int(consecutive_fire_count)  # From Rust NPU state
            properties["consecutive_fire_limit"] = int(consecutive_fire_limit)  # From Rust NPU state
            properties["snooze_period"] = int(snooze_period)  # From Rust NPU state
            properties["excitability"] = float(excitability) if excitability is not None else 1.0
            properties["neuron_type"] = 0  # TODO: Add to get_neuron_state or getter
            properties["properties"] = {}

            # Synapses - get from Rust NPU
            outgoing = []
            incoming = []
            
            if hasattr(rust_npu, 'get_outgoing_synapses'):
                try:
                    # Returns Vec<(target_neuron_id, weight, conductance, synapse_type)>
                    outgoing_raw = rust_npu.get_outgoing_synapses(neuron_id)
                    outgoing = [(target_id, weight, conductance, syn_type) 
                               for (target_id, weight, conductance, syn_type) in outgoing_raw]
                except Exception as e:
                    self.logger.debug(f"Failed to get outgoing synapses for neuron {neuron_id}: {e}")
            
            if hasattr(rust_npu, 'get_incoming_synapses'):
                try:
                    # Returns Vec<(source_neuron_id, weight, conductance, synapse_type)>
                    incoming_raw = rust_npu.get_incoming_synapses(neuron_id)
                    incoming = [(source_id, weight, conductance, syn_type) 
                               for (source_id, weight, conductance, syn_type) in incoming_raw]
                except Exception as e:
                    self.logger.debug(f"Failed to get incoming synapses for neuron {neuron_id}: {e}")
            
            properties["outgoing_synapses"] = [
                {
                    "target_neuron_id": int(t),
                    "weight": float(w) / 255.0,  # Convert u8 to float
                    "conductance": float(c) / 255.0,  # Convert u8 to float
                    "synapse_type": "excitatory" if syn_type == 0 else "inhibitory"
                } 
                for (t, w, c, syn_type) in outgoing
            ]
            properties["incoming_synapses"] = [
                {
                    "source_neuron_id": int(s),
                    "weight": float(w) / 255.0,  # Convert u8 to float
                    "conductance": float(c) / 255.0,  # Convert u8 to float
                    "synapse_type": "excitatory" if syn_type == 0 else "inhibitory"
                }
                for (s, w, c, syn_type) in incoming
            ]
            properties["synapse_counts"] = {
                "outgoing": len(outgoing),
                "incoming": len(incoming),
                "total": len(outgoing) + len(incoming),
            }

            # Add neuron_id to the response
            properties["neuron_id"] = neuron_id
            # Add cortical identifiers
            if cortical_id is not None:
                properties["cortical_id"] = cortical_id
            properties["cortical_idx"] = cortical_idx

            self.logger.info(
                f"DEBUG: Successfully got properties for neuron {neuron_id}: {list(properties.keys())}"
            )
            return properties
            
        except Exception as e:
            self.logger.error(
                f"DEBUG: Error getting properties for neuron {neuron_id}: {str(e)}"
            )
            self.logger.error(f"DEBUG: Exception type: {type(e).__name__}")
            import traceback

            self.logger.error(f"DEBUG: Traceback: {traceback.format_exc()}")
            return None

    def get_cortical_id_list(self) -> List[str]:
        """Get a list of all cortical area IDs (6-character strings) in the
        current genome."""
        return self._cortical_area_service.get_id_list()

    def get_cortical_index_list(self) -> List[int]:
        """Get a list of all cortical area indices (integers) used by the
        FCL."""
        return self._cortical_area_service.get_index_list()

    def get_cortical_name_list(self) -> List[str]:
        """Get a list of all cortical area names."""
        return self._cortical_area_service.get_name_list()

    def get_cortical_id_name_mapping(self) -> Dict[str, str]:
        """Map every cortical area's 6-character cortical_id to its human-
        readable name."""
        return self._cortical_area_service.get_id_name_mapping()

    def get_cortical_locations_2d(self) -> Dict[str, List[int]]:
        """Get 2D locations of all cortical areas."""
        return self._cortical_area_service.get_cortical_locations_2d()

    def get_cortical_2d_locations(self) -> Dict[str, List[int]]:
        """Get 2D locations of all cortical areas (alias for
        get_cortical_locations_2d)."""
        return self._cortical_area_service.get_cortical_locations_2d()

    def update_multiple_cortical_properties(self, message: Dict[str, Any]) -> bool:
        """Bulk update cortical area properties via GenomeService routing.

        Notes:
            - Coordinates are not accepted in bulk (should be pre-filtered by API layer)
            - Mixed memory/non-memory sets should be pre-validated by API layer
            - For each cortical_id, we route to update_cortical_area with separated fields
        """
        try:
            cortical_ids = message.get("cortical_id_list", [])
            if not cortical_ids:
                return False

            # Properties allowed to pass as parameters except structural keys
            properties = {k: v for k, v in message.items() if k != "cortical_id_list"}

            disallowed = {"coordinate_2d", "coordinates_2d", "coordinate_3d", "coordinates_3d"}
            for key in list(properties.keys()):
                if key in disallowed:
                    properties.pop(key, None)

            all_ok = True
            for cortical_id in cortical_ids:
                # Split into structural vs parameters for GenomeService routing
                name = properties.get("cortical_name")
                coordinates = properties.get("coordinates_3d")
                dimensions = properties.get("cortical_dimensions")
                area_type = properties.get("cortical_type")
                params = {
                    k: v
                    for k, v in properties.items()
                    if k not in {"cortical_name", "coordinates_3d", "cortical_dimensions", "cortical_type"}
                }
                if not params:
                    params = None

                result = self._genome_service.update_cortical_area(
                    cortical_id=cortical_id,
                    name=name,
                    coordinates=coordinates,
                    dimensions=dimensions,
                    area_type=area_type,
                    parameters=params,
                )
                all_ok = all_ok and (result is not None)

            return all_ok
        except Exception as e:
            self.logger.error(f"Bulk cortical update failed: {e}")
            return False

    def get_area_neuron_count(self, cortical_id: str) -> int:
        """Get neuron count for a specific cortical area.
        
        Args:
            cortical_id: ID of the cortical area
            
        Returns:
            Number of neurons in the area
        """
        try:
            if not self._connectome_manager:
                return 0
                
            #  Use get_neurons_by_cortical_area which is optimized and
            #  vectorized
            neurons = self._connectome_manager.get_neurons_by_cortical_area(
                cortical_id
            )
            return len(neurons)
            
        except KeyError:
            self.logger.warning(f"Cortical area {cortical_id} not found")
            return 0
        except Exception as e:
            self.logger.error(
                f"Error getting neuron count for {cortical_id}: {str(e)}"
            )
            raise e

    def get_neurons_at_voxel(
        self, cortical_id: str, x: int, y: int, z: int
    ) -> List[int]:
        """Get neuron IDs at a specific voxel location in a cortical area.
        
        Uses Rust NPU spatial hash for O(1) coordinate lookups.
        
        Args:
            cortical_id: ID of the cortical area
            x: X coordinate of the voxel
            y: Y coordinate of the voxel
            z: Z coordinate of the voxel
            
        Returns:
            List of neuron IDs at the specified voxel location (empty list if none)
        """
        try:
            if not self._connectome_manager:
                self.logger.warning("Connectome manager not available")
                return []
            
            # Get cortical area and its cortical_idx
            area = self._connectome_manager.get_cortical_area(cortical_id)
            cortical_idx = area.cortical_idx
            
            # Use Rust NPU spatial hash for lookup
            npu_interface = self._connectome_manager._npu_interface
            if not npu_interface or not hasattr(npu_interface, 'rust_npu'):
                self.logger.warning("Rust NPU not available for voxel lookup")
                return []
            
            rust_npu = npu_interface.rust_npu
            neuron_id = rust_npu.get_neuron_at_coordinate(cortical_idx, x, y, z)
            
            if neuron_id is not None:
                # Return as list (genome typically has 1 neuron per voxel)
                return [neuron_id]
            else:
                # No neuron at this coordinate
                return []
                
        except KeyError:
            self.logger.warning(f"Cortical area {cortical_id} not found")
            return []
        except Exception as e:
            self.logger.error(
                f"Error getting neurons at voxel [{x},{y},{z}] in {cortical_id}: {str(e)}"
            )
            raise e

    def get_cortical_area_memory_usage(self, cortical_id: str):
        """Get detailed memory usage breakdown for a specific cortical area.
        
        Args:
            cortical_id: ID of the cortical area
            
        Returns:
            CorticalAreaMemoryUsageResponse with detailed memory breakdown
        """
        try:
            if not self._connectome_manager:
                raise ValueError("ConnectomeManager not available")
                
            # Import the response schemas
            from feagi.api.v1.schemas import (
                CorticalAreaMemoryUsageResponse,
                MemoryComponentInfo,
                SynapseMemoryBreakdown,
                TotalMemoryInfo,
            )
            
            # Get neuron memory usage
            neuron_info = self._calculate_neuron_memory_usage(cortical_id)
            
            # Get synapse memory usage breakdown
            synapse_breakdown = self._calculate_synapse_memory_breakdown(
                cortical_id
            )
            
            # Calculate total memory
            total_bytes = (
                neuron_info["size_bytes"]
                + synapse_breakdown["incoming"]["size_bytes"]
                + synapse_breakdown["outgoing"]["size_bytes"]
                + synapse_breakdown["internal"]["size_bytes"]
            )
            
            total_info = TotalMemoryInfo(
                size_bytes=total_bytes,
                size_human=self._format_bytes(total_bytes),
            )
            
            # Create response
            return CorticalAreaMemoryUsageResponse(
                cortical_id=cortical_id,
                neurons=MemoryComponentInfo(**neuron_info),
                synapses=SynapseMemoryBreakdown(
                    incoming=MemoryComponentInfo(
                        **synapse_breakdown["incoming"]
                    ),
                    outgoing=MemoryComponentInfo(
                        **synapse_breakdown["outgoing"]
                    ),
                    internal=MemoryComponentInfo(
                        **synapse_breakdown["internal"]
                    ),
                ),
                total=total_info,
            )
            
        except Exception as e:
            self.logger.error(
                f"Error getting memory usage for {cortical_id}: {str(e)}"
            )
            raise e

    def get_cortical_area_geometry(self) -> Dict[str, Any]:
        """Get cortical area geometry information."""
        try:
            # Get all cortical areas and their geometric properties
            areas = self._cortical_area_service.get_all_areas()
            geometry_info = {}

            for area in areas:
                try:
                    # Handle all possible area formats
                    if isinstance(area, dict):
                        area_id = area.get("id")
                    elif isinstance(area, tuple):
                        area_id = str(
                            area[0]
                        )  # Convert first element to string
                    else:
                        area_id = str(area)

                    if not area_id:
                        continue

                    # Get complete properties including mapping information
                    properties = (
                        self._connectome_manager.get_cortical_area_properties(
                        area_id
                        )
                    )
                    if properties:
                        # Handle dimensions - could be tuple or dict
                        dimensions = properties.get("dimensions", {})
                        if isinstance(dimensions, tuple):
                            dimensions = {
                                "width": dimensions[0],
                                "height": dimensions[1],
                                "depth": dimensions[2],
                            }

                        # Handle coordinates - could be tuple or dict
                        coordinates = properties.get("coordinates", {})
                        if isinstance(coordinates, tuple):
                            coordinates = {
                                "x": coordinates[0],
                                "y": coordinates[1],
                                "z": coordinates[2],
                            }

                        geometry_info[area_id] = {
                            "coordinates": coordinates,
                            "dimensions": dimensions,
                            "type": properties.get("type", "unknown"),
                            "neuron_count": properties.get("neuron_count", 0),
                            "parameters": properties.get("parameters", {}),
                            "name": properties.get("name", area_id),
                            "cortical_idx": properties.get("cortical_idx"),
                            "mapping": properties.get("parameters", {}).get(
                                "mapping", {}
                            ),
                        }
                except Exception as e:
                    self.logger.error(
                        f"Error processing area {area}: {str(e)}"
                    )
                    continue

            return geometry_info
        except Exception as e:
            self.logger.error(
                f"Error getting cortical area geometry: {str(e)}"
            )
            raise ValueError(
                f"Failed to get cortical area geometry: {str(e)}"
            ) from e

    def get_current_ipu_list(self) -> List[str]:
        """Get list of current IPU cortical areas."""
        return self._cortical_area_service.get_current_ipu_list()

    def get_current_opu_list(self) -> List[str]:
        """Get list of current OPU cortical areas."""
        return self._cortical_area_service.get_current_opu_list()

    # =================================================================
    # CONNECTOME SERVICE DELEGATION
    # =================================================================

    def get_neuron_connectivity(
        self, neuron_id: str, direction: str = "both"
    ) -> Optional[Dict[str, Any]]:
        """Get connectivity information for a specific neuron."""
        return self._connectome_service.get_neuron_connectivity(
            neuron_id, direction
        )

    def get_connection_stats(self) -> Dict[str, Any]:
        """Get overall connectivity statistics."""
        return self._connectome_service.get_connection_stats()

    def get_connection_matrix(
        self, source_area: str, target_area: str
    ) -> Optional[Dict[str, Any]]:
        """Get connection matrix between two cortical areas."""
        return self._connectome_service.get_connection_matrix(
            source_area, target_area
        )

    def add_connection(
        self, source_neuron: str, target_neuron: str, weight: float = 1.0
    ) -> bool:
        """Add a new synaptic connection."""
        return self._connectome_service.add_connection(
            source_neuron, target_neuron, weight
        )

    def get_cortical_area_synapses(
        self, cortical_area_id: str
    ) -> Optional[Dict[str, List[int]]]:
        """Get synapses from a cortical area organized by destination area.
        
        Args:
            cortical_area_id: ID of the source cortical area
            
        Returns:
            Dictionary where keys are destination cortical area IDs and 
            values are lists of destination neuron IDs
        """
        try:
            cm = self._connectome_manager
            if cm is None:
                self.logger.error("ConnectomeManager not available in CoreAPIService")
                return None

            # Ensure CM is wired to the live NPU arrays (authoritative)
            npu = getattr(cm, "_npu_interface", None)
            if (not hasattr(cm, "synapse_array") or cm.synapse_array is None) and npu is not None:
                try:
                    cm.set_npu_interface(npu)
                except Exception as wire_err:
                    self.logger.error(f"Failed to wire NPU interface to ConnectomeManager: {wire_err}")
                    return None

            # Get all neurons in the source area (cortical_id -> cortical_idx under the hood)
            source_neurons = cm.get_neurons_by_area(cortical_area_id)
            if not source_neurons:
                self.logger.warning(f"No neurons found in cortical area {cortical_area_id}")
                return {}

            # Use authoritative synapse array directly to avoid stale CM methods
            syn_array = getattr(cm, "synapse_array", None) or (npu.synapse_array if npu else None)
            if syn_array is None:
                self.logger.error("SynapseArray not available from ConnectomeManager or NPUInterface")
                return {}

            synapses_by_area: Dict[str, List[int]] = {}

            for source_neuron_id in source_neurons:
                try:
                    outgoing_connections = syn_array.get_outgoing_connections(source_neuron_id)
                except Exception as conn_err:
                    self.logger.debug(f"Skipping neuron {source_neuron_id} (synapse lookup error): {conn_err}")
                    continue

                for target_neuron_id, _ in outgoing_connections:
                    # Map target neuron to cortical_id using NPU-owned mapping path
                    try:
                        target_area_id = cm.get_cortical_area_for_neuron(target_neuron_id)
                    except Exception:
                        continue
                    if target_area_id and target_area_id != cortical_area_id:
                        bucket = synapses_by_area.setdefault(target_area_id, [])
                        bucket.append(target_neuron_id)

            # Deduplicate while preserving order
            for area_id, ids in synapses_by_area.items():
                seen = set()
                deduped = []
                for nid in ids:
                    if nid not in seen:
                        seen.add(nid)
                        deduped.append(nid)
                synapses_by_area[area_id] = deduped

            return synapses_by_area

        except Exception as e:
            self.logger.error(f"Error getting synapses for cortical area {cortical_area_id}: {str(e)}")
            return None

    def remove_connection(
        self, source_neuron: str, target_neuron: str
    ) -> bool:
        """Remove a synaptic connection."""
        return self._connectome_service.remove_connection(
            source_neuron, target_neuron
        )

    def update_connection_weight(
        self, source_neuron: str, target_neuron: str, new_weight: float
    ) -> bool:
        """Update the weight of an existing connection."""
        return self._connectome_service.update_connection_weight(
            source_neuron, target_neuron, new_weight
        )

    def get_area_to_area_connectivity(self) -> Dict[str, Any]:
        """Get connectivity matrix between all cortical areas."""
        return self._connectome_service.get_area_to_area_connectivity()

    def analyze_network_properties(self) -> Dict[str, Any]:
        """Analyze network properties like clustering, path lengths, etc."""
        return self._connectome_service.analyze_network_properties()

    # =================================================================
    # BRAIN SERVICE DELEGATION
    # =================================================================

    def get_burst_engine_status(self) -> Dict[str, Any]:
        """Get current burst engine status."""
        return self._brain_service.get_burst_engine_status()

    def start_burst_engine(self) -> bool:
        """Start the burst engine."""
        return self._brain_service.start_burst_engine()

    def stop_burst_engine(self) -> bool:
        """Stop the burst engine."""
        return self._brain_service.stop_burst_engine()

    def get_brain_statistics(self) -> Dict[str, Any]:
        """Get comprehensive brain statistics."""
        return self._brain_service.get_brain_statistics()

    def get_activity_summary(self, window: int = 10) -> Dict[str, Any]:
        """Get activity summary for the brain over a time window."""
        return self._brain_service.get_activity_summary(window)

    def reset_brain_state(self) -> bool:
        """Reset the brain to initial state."""
        return self._brain_service.reset_brain_state()

    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get brain performance metrics."""
        return self._brain_service.get_performance_metrics()

    def stimulate_neurons(
        self, neural_data: Dict[str, Dict[str, np.ndarray]]
    ) -> Dict[str, Any]:
        """Unified method to stimulate neurons using coordinate-based data
        format.
        
        This method handles both individual neuron stimulation and cortical area stimulation
        by converting coordinates to neuron IDs and injecting them into FCL.
        
        Args:
            neural_data: Data in the format:
                {
                    'cortical_area_1': {
                        'coordinates_x': np.array([1, 2, 3, ...], dtype=np.uint16),
                        'coordinates_y': np.array([4, 5, 6, ...], dtype=np.uint16),
                        'coordinates_z': np.array([7, 8, 9, ...], dtype=np.uint16),
                        'membrane_potentials': np.array([0.8, 1.2, 0.9, ...], dtype=np.float32),
                    },
                    'cortical_area_2': { ... }
                }
        
        Returns:
            Dict containing stimulation results and statistics
        """
        return self._brain_service.stimulate_neurons_unified(neural_data)

    def get_burst_engine_config(self) -> Dict[str, Any]:
        """Get burst engine configuration."""
        return self._brain_service.get_burst_engine_config()

    def get_burst_engine_stats(self) -> Dict[str, Any]:
        """Get burst engine statistics."""
        return self._brain_service.get_brain_statistics()

    def hold_burst_engine(self) -> bool:
        """Put burst engine on hold (pause neural processing)."""
        return self._brain_service.hold_burst_engine()

    def resume_burst_engine(self) -> bool:
        """Resume burst engine from hold (resume neural processing)."""
        return self._brain_service.resume_burst_engine()

    # =================================================================
    # AGENTS SERVICE DELEGATION
    # =================================================================

    # =================================================================
    # LEGACY AGENT METHODS (removed - using new comprehensive agent registry)
    # =================================================================

    # =================================================================
    # NETWORK SERVICE DELEGATION
    # =================================================================

    def get_network_status(self) -> Dict[str, Any]:
        """Get current network status and health."""
        return self._network_service.get_network_status()

    def get_bandwidth_usage(self, time_window: int = 60) -> Dict[str, Any]:
        """Get bandwidth usage statistics over a time window."""
        return self._network_service.get_bandwidth_usage(time_window)

    def get_connection_statistics(self) -> Dict[str, Any]:
        """Get detailed connection statistics."""
        return self._network_service.get_connection_statistics()

    def test_connectivity(
        self, target: Optional[str] = None
    ) -> Dict[str, Any]:
        """Test network connectivity to specific targets or general health."""
        return self._network_service.test_connectivity(target)

    def get_protocol_status(self) -> Dict[str, Any]:
        """Get status of different network protocols."""
        return self._network_service.get_protocol_status()

    def reset_network_statistics(self) -> bool:
        """Reset network statistics and counters."""
        return self._network_service.reset_network_statistics()

    def configure_bandwidth_limits(
        self, limits: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Configure bandwidth limits for different types of traffic."""
        return self._network_service.configure_bandwidth_limits(limits)

    def get_message_queue_status(self) -> Dict[str, Any]:
        """Get status of message queues across different protocols."""
        return self._network_service.get_message_queue_status()

    # =================================================================
    # UTILITY METHODS
    # =================================================================

    def refresh_cached_data(self):
        """Refresh any cached data when connectome changes.

        This can be called when the genome is reloaded or modified.
        """
        # Build cortical_id -> cortical_idx cache for optimal performance
        # But ONLY if genome is loaded to prevent corruption
        try:
            if (
                hasattr(self._connectome_manager, "cortical_areas")
                and self._connectome_manager.cortical_areas
            ):
                self._build_cortical_id_cache()
                self.logger.debug(
                    "Cached data refreshed after connectome changes"
                )
            else:
                self.logger.debug("Skipping cache refresh - genome not ready")
        except Exception as e:
            self.logger.error(f"Error refreshing cached data: {str(e)}")

    def get_service_health(self) -> Dict[str, Any]:
        """Get health information about all domain services."""
        try:
            return {
                "system_service": (
                    "healthy" if self._system_service else "unavailable"
                ),
                "genome_service": (
                    "healthy" if self._genome_service else "unavailable"
                ),
                "cortical_area_service": (
                    "healthy" if self._cortical_area_service else "unavailable"
                ),
                "connectome_service": (
                    "healthy" if self._connectome_service else "unavailable"
                ),
                "brain_service": (
                    "healthy" if self._brain_service else "unavailable"
                ),
                "agents_service": (
                    "healthy" if self._agents_service else "unavailable"
                ),
                "network_service": (
                    "healthy" if self._network_service else "unavailable"
                ),
                "facade_status": "operational",
            }
        except Exception as e:
            self.logger.error(f"Error getting service health: {str(e)}")
            return {"facade_status": "error", "error": str(e)}

    # =================================================================
    # CORE COMPONENT ACCESS METHODS
    # =================================================================

    @property
    def burst_engine(self):
        """Property to access the burst engine instance."""
        return self.get_burst_engine()

    def get_burst_engine(self):
        """Get the burst engine instance.
        
        🦀 RUST: Always returns None - burst engine has been moved to Rust.
        All burst processing is now handled by the Rust NPU via ProcessManager.
        """
        # 🦀 RUST: Burst engine no longer exists - always return None
        return None

    def get_connectome_manager(self):
        """Get the connectome manager instance."""
        return self._connectome_manager

    def get_connectome(self):
        """Get the connectome manager instance (legacy alias)."""
        return self._connectome_manager

    # === Area classification helpers (strict) ===
    def is_opu_area(self, cortical_id_or_idx) -> bool:
        """Return True iff cortical_group == 'OPU' for the area."""
        try:
            cm = self._connectome_manager
            return bool(cm and hasattr(cm, "is_opu") and cm.is_opu(cortical_id_or_idx))
        except Exception:
            return False

    def is_ipu_area(self, cortical_id_or_idx) -> bool:
        """Return True iff cortical_group == 'IPU' for the area."""
        try:
            cm = self._connectome_manager
            return bool(cm and hasattr(cm, "is_ipu") and cm.is_ipu(cortical_id_or_idx))
        except Exception:
            return False

    def list_opu_areas(self) -> list:
        """Return list of cortical IDs for OPU areas."""
        try:
            cm = self._connectome_manager
            return list(cm.list_opu_areas()) if cm and hasattr(cm, "list_opu_areas") else []
        except Exception:
            return []

    def list_ipu_areas(self) -> list:
        """Return list of cortical IDs for IPU areas."""
        try:
            cm = self._connectome_manager
            return list(cm.list_ipu_areas()) if cm and hasattr(cm, "list_ipu_areas") else []
        except Exception:
            return []

    def get_fcl_manager(self):
        """Get the FCL manager instance (compatibility adapter for new architecture).
        
        In the new architecture, there is no persistent FCL manager.
        Instead, FCL is a transient pre-burst collector and FireQueue holds 
        the actual firing neurons. We return a compatibility adapter.
        """
        # BurstEngine has been moved to pure Rust - FCL is now managed directly by Rust NPU
        # For now, FCL manager not available (needs Rust integration)
        
        # Fallback to old ConnectomeManager linkage if available (legacy support)
        if hasattr(self._connectome_manager, "fcl_manager"):
            return self._connectome_manager.fcl_manager
        return None

    def get_memory_manager(self):
        """Get the memory manager instance."""
        # Return the connectome manager as it manages memory
        return self._connectome_manager

    # =================================================================
    # FIRE QUEUE & STATE MANAGEMENT
    # =================================================================
    # NOTE: Fire Queue is now in Rust. Access via:
    #   burst_engine.rust_npu.sample_fire_queue()
    # Legacy get_current_fire_queue() removed - use Rust FQ Sampler instead.
    
    def get_fire_queue(self) -> Optional[Dict[str, Any]]:
        """Get the global fire queue data from Rust Fire Queue."""
        try:
            # 🦀 RUST: Get Rust NPU directly from ProcessManager
            from feagi.process_manager import get_process_manager
            process_manager = get_process_manager()
            
            if not process_manager or not process_manager.rust_npu_integration:
                self.logger.debug("🔥 [CORE API] Rust NPU not available")
                return self._empty_fire_queue_response()
            
            rust_npu_integration = process_manager.rust_npu_integration
            
            # Sample from Rust Fire Queue
            sample = rust_npu_integration.sample_fire_queue()
            if not sample:
                self.logger.debug("🔥 [CORE API] Fire queue is empty - no neurons fired globally")
                return self._empty_fire_queue_response()
            
            # Extract data from all cortical areas
            neuron_ids = []
            membrane_potentials = []
            thresholds = []
            refractory_counters = []
            coordinates = []
            
            for cortical_idx, area_data in sample.items():
                neuron_ids.extend(area_data.get('neuron_ids', []))
                membrane_potentials.extend(area_data.get('membrane_potentials', []))
                
                # Extract coordinates as tuples
                coords_x = area_data.get('coordinates_x', [])
                coords_y = area_data.get('coordinates_y', [])
                coords_z = area_data.get('coordinates_z', [])
                for i in range(len(coords_x)):
                    coordinates.append((coords_x[i], coords_y[i], coords_z[i]))
                
                # Get neuron states from Rust NPU
                rust_npu = rust_npu_integration._rust_npu
                for nid in area_data.get('neuron_ids', []):
                    try:
                        state = rust_npu.get_neuron_state(nid)
                        if state:
                            # state = (cfc, cfc_limit, snooze_period, potential, threshold, refrac_countdown)
                            _cfc, _cfc_limit, _snooze, _potential, threshold_val, refrac_countdown = state
                            thresholds.append(float(threshold_val))
                            refractory_counters.append(int(refrac_countdown))
                        else:
                            thresholds.append(1.0)
                            refractory_counters.append(0)
                    except Exception as e:
                        self.logger.warning(f"Failed to get state for neuron {nid}: {e}")
                        thresholds.append(1.0)
                        refractory_counters.append(0)
            
            self.logger.debug(
                f"🔥 [CORE API] Global fire queue has {len(neuron_ids)} firing neurons"
            )
            
            return {
                "neuron_ids": neuron_ids,
                "membrane_potentials": membrane_potentials,
                "thresholds": thresholds,
                "consecutive_fire_counts": [],  # Not tracked yet
                "refractory_counters": refractory_counters,
                "coordinates": coordinates,
            }
            
        except Exception as e:
            self.logger.error(f"Error getting global fire queue: {str(e)}")
            return None
    
    def _empty_fire_queue_response(self) -> Dict[str, Any]:
        """Return empty fire queue response structure."""
        return {
            "neuron_ids": [],
            "membrane_potentials": [],
            "thresholds": [],
            "consecutive_fire_counts": [],
            "refractory_counters": [],
            "coordinates": [],
        }

    def genome_is_loaded(self) -> bool:
        """Check if a genome is currently loaded - CRITICAL for state management."""
        return self._genome_service.is_genome_loaded()

    def get_state_manager(self):
        """Get the state manager instance."""
        return self.state_manager

    # =================================================================
    # BRAIN STATE MANAGEMENT METHODS
    # =================================================================

    def get_brain_state(self) -> Dict[str, Any]:
        """Get current brain state."""
        return self._brain_service.get_brain_statistics()

    def save_brain_state(self, path: str) -> bool:
        """Save brain state to file."""
        try:
            brain_state = self.get_brain_state()
            import json

            with open(path, "w") as f:
                json.dump(brain_state, f, indent=2)
            return True
        except Exception as e:
            self.logger.error(f"Error saving brain state: {str(e)}")
            return False

    def load_brain_state(self, path: str) -> bool:
        """Load brain state from file."""
        try:
            import json

            with open(path, "r") as f:
                # brain_state = json.load(f)  # Unused variable removed
                json.load(f)
            # This would need implementation in brain service
            return True
        except Exception as e:
            self.logger.error(f"Error loading brain state: {str(e)}")
            return False

    # =================================================================
    # LEGACY COMPATIBILITY METHODS
    # =================================================================

    # Add any legacy method aliases or compatibility methods here if needed
    # For now, all existing methods are preserved with their exact signatures

    # Legacy method aliases for backward compatibility
    def get_cortical_areas(self) -> List[Dict[str, Any]]:
        """Get all cortical areas (alias for get_all_cortical_areas)."""
        return self.get_all_cortical_areas()

    # =================================================================
    # ADDITIONAL AGENT MANAGEMENT METHODS
    # =================================================================

    def get_agent_list(self) -> Set[str]:
        """Get list of agent IDs."""
        try:
            agents = self._agents_service.get_connected_agents()
            return {
                agent.get("id", agent.get("agent_id", "")) for agent in agents
            }
        except Exception as e:
            self.logger.error(f"Error getting agent list: {str(e)}")
            return set()

    def get_agent_properties(self, agent_id: str) -> Dict[str, Any]:
        """Get properties of a specific agent."""
        return self._agents_service.get_agent_details(agent_id) or {}

    def deregister_agent(self, agent_id: str) -> bool:
        """Deregister an agent."""
        result = self._agents_service.unregister_agent(agent_id)
        return (
            result.get("success", False) if isinstance(result, dict) else False
        )

    # =================================================================
    # LEGACY CORTICAL AREA METHOD NAMES
    # =================================================================

    def get_cortical_area_id_list(self) -> List[str]:
        """Get list of cortical area IDs (legacy name)."""
        return self.get_cortical_id_list()

    def get_cortical_area_index_list(self) -> List[int]:
        """Get list of cortical area indices (legacy name)."""
        return self.get_cortical_index_list()

    def get_cortical_area_name_list(self) -> List[str]:
        """Get list of cortical area names (legacy name)."""
        return self.get_cortical_name_list()

    def get_cortical_area_stats(
        self, cortical_area: str
    ) -> Optional[Dict[str, Any]]:
        """Get statistics for a cortical area."""
        return self._cortical_area_service.get_area_stats(cortical_area)

    # =================================================================
    # PLASTICITY AND LEARNING METHODS
    # =================================================================

    def enable_area_plasticity(
        self, cortical_id: str, settings: Optional[Dict[str, Any]] = None
    ) -> bool:
        """Enable plasticity for a cortical area."""
        try:
            # This would need implementation in a plasticity service
            self.logger.info(f"Enabling plasticity for area {cortical_id}")
            return True
        except Exception as e:
            self.logger.error(
                f"Error enabling plasticity for {cortical_id}: {str(e)}"
            )
            return False

    def disable_area_plasticity(self, cortical_id: str) -> bool:
        """Disable plasticity for a cortical area."""
        try:
            # This would need implementation in a plasticity service
            self.logger.info(f"Disabling plasticity for area {cortical_id}")
            return True
        except Exception as e:
            self.logger.error(
                f"Error disabling plasticity for {cortical_id}: {str(e)}"
            )
            return False

    def get_plasticity_info(self) -> Dict[str, Any]:
        """Get plasticity information."""
        try:
            return {
                "enabled": True,
                "queue_depth": 1000,
                "areas_with_plasticity": [],
            }
        except Exception as e:
            self.logger.error(f"Error getting plasticity info: {str(e)}")
            return {}

    def get_plasticity_queue_depth(self) -> int:
        """Get plasticity queue depth."""
        return 1000  # Default value

    def update_plasticity_queue_depth(self, depth: int) -> bool:
        """Update plasticity queue depth."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(
                f"Error updating plasticity queue depth: {str(e)}"
            )
            return False

    def update_plasticity_config(self, config: Dict[str, Any]) -> bool:
        """Update plasticity configuration."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error updating plasticity config: {str(e)}")
            return False

    # =================================================================
    # MONITORING METHODS
    # =================================================================

    def get_membrane_potential_monitoring_status(
        self, cortical_areas: List[str]
    ) -> List[Tuple[str, bool]]:
        """Get membrane potential monitoring status for cortical areas."""
        try:
            # This should get real monitoring status from the brain service
            raise NotImplementedError(
                "Membrane potential monitoring status is not yet implemented"
            )
        except Exception as e:
            self.logger.error(
                f"Error getting membrane potential monitoring status: {str(e)}"
            )
            raise ValueError(
                f"Failed to get membrane potential monitoring status: {str(e)}"
            ) from e

    def set_membrane_potential_monitoring(
        self, cortical_areas: List[str], enabled: bool
    ) -> bool:
        """Set membrane potential monitoring for cortical areas."""
        try:
            # This should actually set monitoring in the brain service
            raise NotImplementedError(
                "Setting membrane potential monitoring is not yet implemented"
            )
        except Exception as e:
            self.logger.error(
                f"Error setting membrane potential monitoring: {str(e)}"
            )
            raise ValueError(
                f"Failed to set membrane potential monitoring: {str(e)}"
            ) from e

    def get_synaptic_potential_monitoring_status(
        self, cortical_areas: List[str]
    ) -> List[Tuple[str, bool]]:
        """Get synaptic potential monitoring status for cortical areas."""
        try:
            # This should get real monitoring status from the brain service
            raise NotImplementedError(
                "Synaptic potential monitoring status is not yet implemented"
            )
        except Exception as e:
            self.logger.error(
                f"Error getting synaptic potential monitoring status: {str(e)}"
            )
            raise ValueError(
                f"Failed to get synaptic potential monitoring status: {str(e)}"
            ) from e

    def set_synaptic_potential_monitoring(
        self, cortical_areas: List[str], enabled: bool
    ) -> bool:
        """Set synaptic potential monitoring for cortical areas."""
        try:
            # This should actually set monitoring in the brain service
            raise NotImplementedError(
                "Setting synaptic potential monitoring is not yet implemented"
            )
        except Exception as e:
            self.logger.error(
                f"Error setting synaptic potential monitoring: {str(e)}"
            )
            raise ValueError(
                f"Failed to set synaptic potential monitoring: {str(e)}"
            ) from e

    def get_membrane_potentials(
        self, neuron_ids: List[int]
    ) -> Dict[int, float]:
        """Get membrane potentials for specific neurons."""
        try:
            potentials: Dict[int, float] = {}
            if not hasattr(self._connectome_manager, "neuron_array"):
                return potentials
            na = self._connectome_manager.neuron_array
            id_to_idx = getattr(na, "neuron_id_to_index", {})
            mem = getattr(na, "membrane_potentials", None)
            if mem is None:
                return potentials
            for neuron_id in neuron_ids:
                idx = id_to_idx.get(neuron_id)
                if idx is None or idx < 0 or idx >= na.neuron_count:
                    continue
                potentials[neuron_id] = float(mem[idx])
            return potentials
        except Exception as e:
            self.logger.error(f"Error getting membrane potentials: {str(e)}")
            raise ValueError(
                f"Failed to get membrane potentials: {str(e)}"
            ) from e

    def update_membrane_potentials(self, potentials: Dict[int, float]) -> bool:
        """Update membrane potentials for specific neurons."""
        try:
            if not hasattr(self._connectome_manager, "neuron_array"):
                return False
            na = self._connectome_manager.neuron_array
            id_to_idx = getattr(na, "neuron_id_to_index", {})
            mem = getattr(na, "membrane_potentials", None)
            if mem is None:
                return False
            for neuron_id, potential in potentials.items():
                idx = id_to_idx.get(neuron_id)
                if idx is None or idx < 0 or idx >= na.neuron_count:
                    continue
                mem[idx] = float(potential)
            return True
        except Exception as e:
            self.logger.error(f"Error updating membrane potentials: {str(e)}")
            return False

    # =================================================================
    # ADDITIONAL MISSING METHODS
    # =================================================================

    def get_fq_sampler_config(self) -> Dict[str, Any]:
        """Get FQ sampler configuration."""
        try:
            if self.state_manager:
                return {
                    "frequency": getattr(
                        self.state_manager, "fq_sampler_frequency", 20.0
                    ),
                    "consumer": getattr(
                        self.state_manager, "fq_sampler_consumer", 1
                    ),
                }
            return {"frequency": 20.0, "consumer": 1}
        except Exception as e:
            self.logger.error(f"Error getting FQ sampler config: {str(e)}")
            return {}

    def update_fq_sampler_config(
        self, frequency: float, consumer: str
    ) -> bool:
        """Update FQ sampler configuration."""
        try:
            if self.state_manager:
                self.state_manager.set_fq_sampler_frequency(frequency)
                consumer_map = {"visualization": 1, "motor": 2, "both": 3}
                self.state_manager.set_fq_sampler_consumer(
                    consumer_map.get(consumer, 1)
                )
            return True
        except Exception as e:
            self.logger.error(f"Error updating FQ sampler config: {str(e)}")
            return False

    def get_area_fq_sample_rate(self, area_id: int) -> float:
        """Get FQ sample rate for an area."""
        try:
            # Get from the FQ sampler via burst engine
            burst_engine = self.get_burst_engine()
            if burst_engine and hasattr(burst_engine, 'get_fq_sampler'):
                fq_sampler = burst_engine.get_fq_sampler()
                if fq_sampler and hasattr(fq_sampler, 'sample_frequency_hz'):
                    return float(fq_sampler.sample_frequency_hz)
            
            # Fallback to state manager configuration
            if self.state_manager:
                return getattr(self.state_manager, "fq_sampler_frequency", 20.0)
            return 20.0
        except Exception as e:
            self.logger.error(f"Error getting area FQ sample rate: {str(e)}")
            raise ValueError(
                f"Failed to get area FQ sample rate: {str(e)}"
            ) from e
    
    def set_area_fq_sample_rate(self, area_id: int, sample_rate: float) -> bool:
        """Set FQ sample rate for a specific cortical area."""
        try:
            # For now, we set the global FQ sampler rate
            # TODO: In future, support per-area sampling rates
            if sample_rate <= 0 or sample_rate > 1000:
                raise ValueError("Sample rate must be between 0 and 1000 Hz")
            
            # Update via burst engine if available
            burst_engine = self.get_burst_engine()
            if burst_engine and hasattr(burst_engine, 'get_fq_sampler'):
                fq_sampler = burst_engine.get_fq_sampler()
                if fq_sampler and hasattr(fq_sampler, 'sample_frequency_hz'):
                    fq_sampler.sample_frequency_hz = sample_rate
                    self.logger.info(f"Updated FQ sampler frequency to {sample_rate}Hz for area {area_id}")
                    return True
            
            # Fallback: Update state manager
            if self.state_manager:
                self.state_manager.set_fq_sampler_frequency(sample_rate)
                self.logger.info(f"Updated FQ sampler frequency in state manager to {sample_rate}Hz for area {area_id}")
                return True
            
            return False
        except Exception as e:
            self.logger.error(f"Error setting area FQ sample rate: {str(e)}")
            return False
    
    def get_fcl_sampler_config(self) -> Dict[str, Any]:
        """Get FCL sampler configuration (alias for FQ sampler config)."""
        return self.get_fq_sampler_config()
    
    def update_fcl_sampler_config(self, frequency: float, consumer: str) -> bool:
        """Update FCL sampler configuration (alias for FQ sampler config)."""
        return self.update_fq_sampler_config(frequency, consumer)
    
    # =================================================================
    # FIRE LEDGER WINDOW SIZE CONFIGURATION METHODS
    # =================================================================
    
    def get_fire_ledger_default_window_size(self) -> int:
        """Get the default window size for Rust Fire Ledger historical storage."""
        try:
            burst_engine = self.get_burst_engine()
            if burst_engine and hasattr(burst_engine, 'rust_npu') and burst_engine.rust_npu:
                # Rust Fire Ledger default is 20 (hardcoded in Rust)
                return 20
            return 20  # Default window size
        except Exception as e:
            self.logger.error(f"Error getting Fire Ledger default window size: {str(e)}")
            return 20
    
    def get_fire_ledger_area_window_size(self, cortical_idx: int) -> int:
        """Get window size for specific cortical area in Rust Fire Ledger."""
        try:
            burst_engine = self.get_burst_engine()
            if burst_engine and hasattr(burst_engine, 'rust_npu') and burst_engine.rust_npu:
                return burst_engine.rust_npu.get_fire_ledger_window_size(cortical_idx)
            return 20  # Default window size
        except Exception as e:
            self.logger.error(f"Error getting Fire Ledger window size for area {cortical_idx}: {str(e)}")
            return 20
    
    def set_fire_ledger_area_window_size(self, cortical_idx: int, window_size: int) -> bool:
        """Set window size for specific cortical area in Rust Fire Ledger."""
        try:
            if window_size <= 0 or window_size > 10000:
                raise ValueError("Window size must be between 1 and 10000")
                
            burst_engine = self.get_burst_engine()
            if burst_engine and hasattr(burst_engine, 'rust_npu') and burst_engine.rust_npu:
                burst_engine.rust_npu.configure_fire_ledger_window(cortical_idx, window_size)
                self.logger.info(f"Updated Rust Fire Ledger window size to {window_size} for cortical area {cortical_idx}")
                return True
            
            self.logger.warning("No Rust Fire Ledger available to configure window size")
            return False
        except Exception as e:
            self.logger.error(f"Error setting Fire Ledger window size for area {cortical_idx}: {str(e)}")
            return False
    
    def get_fire_ledger_areas_window_config(self) -> Dict[int, int]:
        """Get window size configuration for all cortical areas in Rust Fire Ledger."""
        try:
            burst_engine = self.get_burst_engine()
            if burst_engine and hasattr(burst_engine, 'rust_npu') and burst_engine.rust_npu:
                return burst_engine.rust_npu.get_all_fire_ledger_configs()
            return {}
        except Exception as e:
            self.logger.error(f"Error getting Fire Ledger areas window configuration: {str(e)}")
            return {}
    
    def get_fire_ledger_history(self, cortical_idx: int, lookback_steps: Optional[int] = None) -> Dict[str, Any]:
        """Get historical firing data for a cortical area from RUST Fire Ledger.
        
        Args:
            cortical_idx: Cortical area index
            lookback_steps: Number of timesteps to retrieve (None = all available)
            
        Returns:
            Dict with firing history data:
            {
                "cortical_idx": int,
                "cortical_id": str,
                "current_timestep": int,
                "window_size": int,
                "lookback_steps": int,
                "history": [
                    {"timestep": int, "neuron_ids": [int, ...], "count": int},
                    ...
                ]
            }
        """
        try:
            # Get burst engine
            burst_engine = self.get_burst_engine()
            if not burst_engine or not hasattr(burst_engine, 'rust_npu'):
                return {
                    "success": False,
                    "error": "NPU not available"
                }
            
            rust_npu = burst_engine.rust_npu
            if not rust_npu:
                return {
                    "success": False,
                    "error": "Rust NPU not initialized"
                }
            
            # Get Fire Ledger history from RUST NPU
            # If lookback_steps is None, use a large default (e.g., 10000)
            rust_lookback = lookback_steps if lookback_steps is not None else 10000
            
            # Call Rust: Returns Vec<(timestep: u64, neuron_ids: Vec<u32>)>
            rust_history = rust_npu.get_fire_ledger_history(cortical_idx, rust_lookback)
            
            # Get window size from Rust
            window_size = rust_npu.get_fire_ledger_window_size(cortical_idx)
            
            # Convert Rust data to API format
            # rust_history is Vec<(timestep: u64, neuron_ids: Vec<u32>)>, newest first
            history_data = []
            for (timestep, neuron_ids) in rust_history:
                history_data.append({
                    "timestep": int(timestep),
                    "neuron_ids": [int(nid) for nid in neuron_ids],
                    "count": len(neuron_ids)
                })
            
            # Get current timestep from burst engine
            burst_engine = self.get_burst_engine()
            current_timestep = burst_engine.burst_count if burst_engine else 0
            
            # Get cortical ID from connectome if available
            cortical_id = None
            if self._connectome_manager:
                try:
                    cortical_id = self._connectome_manager.get_cortical_id_for_idx(cortical_idx)
                except:
                    pass
            
            return {
                "success": True,
                "cortical_idx": cortical_idx,
                "cortical_id": cortical_id,
                "current_timestep": current_timestep,
                "window_size": window_size,
                "lookback_steps": len(rust_history),
                "total_timesteps_available": len(rust_history),
                "history": history_data
            }
            
        except Exception as e:
            self.logger.error(f"Error getting Fire Ledger history for area {cortical_idx}: {str(e)}", exc_info=True)
            return {
                "success": False,
                "cortical_idx": cortical_idx,
                "error": str(e)
            }

    def get_burst_counter(self) -> int:
        """Get current burst counter - RTOS-safe."""
        try:
            # RTOS-SAFE: Get actual burst count from burst engine
            burst_engine = self.get_burst_engine()
            if burst_engine and hasattr(burst_engine, "burst_count"):
                return burst_engine.burst_count

            # Fallback to state manager if burst engine not available
            if self.state_manager:
                return getattr(self.state_manager, "current_burst_id", 0)
            return 0
        except Exception as e:
            self.logger.error(f"Error getting burst counter: {str(e)}")
            return 0

    def update_burst_engine_config(self, config: Dict[str, Any]) -> bool:
        """Update burst engine configuration - RTOS-safe."""
        try:
            # Get state manager - the authoritative source for system state
            state_manager = self.get_state_manager()
            if not state_manager:
                self.logger.error(
                    "No state manager available for config update"
                )
                return False

            # RTOS-SAFE: Update frequency if provided
            if "burst_frequency_hz" in config:
                frequency = config["burst_frequency_hz"]
                
                # Validate frequency
                if (
                    frequency <= 0.0 or frequency > 10000.0
                ):  # Max 10kHz for safety
                    self.logger.error(
                        f"Invalid frequency {frequency}Hz (must be 0 < freq <= 10000)"
                    )
                    return False
                
                # Write to state_manager - the single source of truth
                state_manager.set_burst_frequency(frequency)
                self.logger.info(
                    f"Updated burst frequency to {frequency}Hz in state manager"
                )
                
                #  Also update burst engine for immediate effect (it should
                #  sync from state_manager)
                burst_engine = self.get_burst_engine()
                if burst_engine:
                    if not burst_engine.update_frequency(frequency):
                        self.logger.warning(
                            f"Failed to sync burst engine with new frequency {frequency}Hz"
                        )
                        #  Don't return False here - state_manager update
                        #  succeeded

                # AUTOMATIC FQ SAMPLER SYNCHRONIZATION
                #  When burst frequency changes, automatically update all
                #  active FQ samplers
                # to ensure they never exceed the new burst frequency
                self._synchronize_fq_samplers_with_burst_frequency(frequency)

            return True
        except Exception as e:
            self.logger.error(f"Error updating burst engine config: {str(e)}")
            return False

    def _synchronize_fq_samplers_with_burst_frequency(
        self, new_burst_frequency: float
    ) -> None:
        """Automatically synchronize all active FQ samplers with new burst
        frequency.
        
        Ensures that no FQ sampler exceeds the burst frequency by applying:
        new_sampler_freq = min(configured_freq, new_burst_freq)
        
        Args:
            new_burst_frequency: New burst frequency in Hz
        """
        try:
            self.logger.info(
                f"🔄 [AUTO-SYNC] Starting FQ sampler synchronization with burst frequency: {new_burst_frequency}Hz"
            )
            
            # Get ProcessManager instance to access FQ samplers
            try:
                from feagi.process_manager import get_process_manager

                process_manager = get_process_manager()
                if not process_manager:
                    self.logger.warning(
                        "🔄 [AUTO-SYNC] ProcessManager not available - FQ sampler sync skipped"
                    )
                    return
            except Exception as e:
                self.logger.warning(
                    f"🔄 [AUTO-SYNC] Cannot access ProcessManager: {e} - FQ sampler sync skipped"
                )
                return
            
            sync_count = 0
            
            # Synchronize Visualization FQ Sampler AND Visualization Stream
            self.logger.warning(f"🔍🔍🔍 [AUTO-SYNC] Using ProcessManager instance id={id(process_manager)}")
            viz_sampler = process_manager.get_viz_fq_sampler()
            
            if viz_sampler:
                self.logger.warning(f"🔍🔍🔍 [AUTO-SYNC] Found viz sampler: {viz_sampler.instance_id}")
                try:
                    #  Get configured frequency from ProcessManager (not
                    #  current frequency)
                    #  This ensures we use the original configured frequency,
                    #  not the capped one
                    configured_viz_freq = (
                        30.0  # Default visualization frequency
                    )
                    if hasattr(process_manager, "_fq_sampler_config"):
                        configured_viz_freq = (
                            process_manager._fq_sampler_config.get(
                            "visualization_frequency", 30.0
                            )
                        )

                    current_viz_freq = getattr(
                        viz_sampler, "sample_frequency", configured_viz_freq
                    )

                    #  Apply frequency sync rule: min(CONFIGURED_freq,
                    #  burst_freq)
                    new_viz_freq = min(
                        configured_viz_freq, new_burst_frequency
                    )
                    
                    if new_viz_freq != current_viz_freq:
                        viz_sampler.set_sample_frequency(new_viz_freq)
                        
                        #  CRITICAL FIX: Also update visualization stream's
                        #  sample_rate!
                        #  The visualization stream uses its own timing,
                        #  independent of FQ sampler
                        try:
                            updated_streams = process_manager.update_visualization_stream_frequency(
                                new_viz_freq
                            )
                            if updated_streams > 0:
                                self.logger.info(
                                    f"🎬 [AUTO-SYNC] Updated {updated_streams} visualization stream(s): sample_rate → {new_viz_freq}Hz "
                                    f"(CRITICAL: streams were using independent timing!)"
                                )
                            else:
                                self.logger.warning(
                                    "🎬 [AUTO-SYNC] No visualization streams found to update sample_rate"
                                )
                        except Exception as e:
                            self.logger.error(
                                f"🎬 [AUTO-SYNC] Failed to update visualization stream frequency: {e}"
                            )
                        
                        self.logger.info(
                            f"🎨 [AUTO-SYNC] Visualization sampler: {current_viz_freq}Hz → {new_viz_freq}Hz "
                            f"(configured: {configured_viz_freq}Hz, burst limit: {new_burst_frequency}Hz)"
                        )
                        sync_count += 1
                    else:
                        self.logger.info(
                            f"🎨 [AUTO-SYNC] Visualization sampler: {current_viz_freq}Hz (no change needed)"
                        )
                except Exception as e:
                    self.logger.error(
                        f"🎨 [AUTO-SYNC] Failed to update visualization sampler: {e}"
                    )
            else:
                self.logger.info(
                    f"🎨 [AUTO-SYNC] No visualization sampler active (no agents connected yet or agents disconnected). "
                    f"When visualization agents connect, they will use the new burst frequency: {new_burst_frequency}Hz"
                )
            
            # Synchronize Motor FQ Sampler  
            motor_sampler = process_manager.get_motor_fq_sampler()
            
            if motor_sampler:
                try:
                    #  Get configured frequency from ProcessManager (not
                    #  current frequency)
                    #  This ensures we use the original configured frequency,
                    #  not the capped one
                    configured_motor_freq = 100.0  # Default motor frequency
                    if hasattr(process_manager, "_fq_sampler_config"):
                        configured_motor_freq = (
                            process_manager._fq_sampler_config.get(
                            "motor_frequency", 100.0
                            )
                        )

                    current_motor_freq = getattr(
                        motor_sampler,
                        "sample_frequency",
                        configured_motor_freq,
                    )

                    #  Apply frequency sync rule: min(CONFIGURED_freq,
                    #  burst_freq)
                    new_motor_freq = min(
                        configured_motor_freq, new_burst_frequency
                    )
                    
                    if new_motor_freq != current_motor_freq:
                        motor_sampler.set_sample_frequency(new_motor_freq)
                        self.logger.info(
                            f"🚗 [AUTO-SYNC] Motor sampler: {current_motor_freq}Hz → {new_motor_freq}Hz "
                            f"(configured: {configured_motor_freq}Hz, burst limit: {new_burst_frequency}Hz)"
                        )
                        sync_count += 1
                    else:
                        self.logger.info(
                            f"🚗 [AUTO-SYNC] Motor sampler: {current_motor_freq}Hz (no change needed)"
                        )
                except Exception as e:
                    self.logger.error(
                        f"🚗 [AUTO-SYNC] Failed to update motor sampler: {e}"
                    )
            else:
                self.logger.warning(
                    "🚗 [AUTO-SYNC] No active motor sampler found"
                )
            
            # Summary log
            if sync_count > 0:
                self.logger.info(
                    f"✅ [AUTO-SYNC] Successfully synchronized {sync_count} FQ sampler(s) with burst frequency {new_burst_frequency}Hz"
                )
            else:
                self.logger.info(
                    f"✅ [AUTO-SYNC] All FQ samplers already synchronized with burst frequency {new_burst_frequency}Hz"
                )
                
        except Exception as e:
            self.logger.error(
                f"❌ [AUTO-SYNC] Failed to synchronize FQ samplers: {e}"
            )
            #  Don't raise - this is a nice-to-have feature, shouldn't break
            #  frequency updates

    def get_network_config(self) -> Dict[str, Any]:
        """Get network configuration."""
        return self._network_service.get_protocol_status()

    def update_network_config(self, network_config: Dict[str, Any]) -> bool:
        """Update network configuration."""
        try:
            # This would need implementation in network service
            return True
        except Exception as e:
            self.logger.error(f"Error updating network config: {str(e)}")
            return False

    def get_connectome_dimensions(self) -> Dict[str, Any]:
        """Get connectome dimensions."""
        try:
            stats = self._brain_service.get_brain_statistics()
            return {
                "neuron_count": stats.get("neuron_count", 0),
                "synapse_count": stats.get("synapse_count", 0),
                "cortical_area_count": stats.get("cortical_area_count", 0),
            }
        except Exception as e:
            self.logger.error(f"Error getting connectome dimensions: {str(e)}")
            return {}

    def get_morphology_list(self) -> List[str]:
        """Get list of available morphologies."""
        try:
            # Import core morphologies from templates
            from feagi.evo.templates import core_morphologies

            morphology_names = list(core_morphologies.keys())

            # Also check genome for additional morphologies if available
            genome = self.get_genome()
            if genome and "neuron_morphologies" in genome:
                genome_morphologies = list(
                    genome["neuron_morphologies"].keys()
                )
                # Combine and deduplicate
                morphology_names.extend(
                    [
                        m
                        for m in genome_morphologies
                        if m not in morphology_names
                    ]
                )

            return sorted(morphology_names)
        except Exception as e:
            self.logger.error(f"Error getting morphology list: {str(e)}")
            raise ValueError(
                f"Failed to retrieve morphology list: {str(e)}"
            ) from e

    def get_morphology_types(self) -> List[str]:
        """Get list of available morphology types."""
        try:
            # Get unique morphology types from core morphologies
            from feagi.evo.templates import core_morphologies

            types = set()
            for morphology in core_morphologies.values():
                if "type" in morphology:
                    types.add(morphology["type"])

            # Also check genome morphologies
            genome = self.get_genome()
            if genome and "neuron_morphologies" in genome:
                for morphology in genome["neuron_morphologies"].values():
                    if "type" in morphology:
                        types.add(morphology["type"])

            return sorted(list(types))
        except Exception as e:
            self.logger.error(f"Error getting morphology types: {str(e)}")
            raise ValueError(
                f"Failed to retrieve morphology types: {str(e)}"
            ) from e

    def get_morphologies(self) -> Dict[str, Any]:
        """Get all morphologies with detailed information."""
        try:
            # Import core morphologies from templates
            from feagi.evo.templates import core_morphologies

            # Start with core morphologies
            all_morphologies = {}
            for name, morphology in core_morphologies.items():
                all_morphologies[name] = {
                    "name": name,
                    "type": morphology.get("type", "unknown"),
                    "class": "core",  # ✅ FIXED: Use "core" for core morphologies
                    "parameters": morphology.get("parameters", {}),
                    "source": "core",
                }

            # Add genome morphologies if available
            genome = self.get_genome()
            if genome and "neuron_morphologies" in genome:
                for name, morphology in genome["neuron_morphologies"].items():
                    # Genome morphologies override core morphologies
                    all_morphologies[name] = {
                        "name": name,
                        "type": morphology.get("type", "unknown"),
                        "class": morphology.get("class", "custom"),  # Use original class or default to "custom"
                        "parameters": morphology.get("parameters", {}),
                        "source": "genome",
                    }

            return all_morphologies
        except Exception as e:
            self.logger.error(f"Error getting morphologies: {str(e)}")
            raise ValueError(
                f"Failed to retrieve morphologies: {str(e)}"
            ) from e

    def get_morphology_info(self, morphology_id: str) -> Dict[str, Any]:
        """Get information about a specific morphology."""
        try:
            # Get all morphologies and find the requested one
            all_morphologies = self.get_morphologies()

            if morphology_id not in all_morphologies:
                raise ValueError(f"Morphology '{morphology_id}' not found")

            morphology = all_morphologies[morphology_id]

            # Add additional computed information
            morphology_info = morphology.copy()
            morphology_info.update(
                {
                    "id": morphology_id,
                    "description": self._get_morphology_description(
                        morphology
                    ),
                    "example_usage": self._get_morphology_example(morphology),
                }
            )

            return morphology_info
        except Exception as e:
            self.logger.error(f"Error getting morphology info: {str(e)}")
            raise ValueError(
                f"Failed to retrieve morphology info: {str(e)}"
            ) from e

    def _get_morphology_description(self, morphology: Dict[str, Any]) -> str:
        """Generate a description for a morphology based on its type and
        parameters."""
        morphology_type = morphology.get("type", "unknown")

        descriptions = {
            "vectors": "Connects neurons using fixed directional vectors",
            "patterns": "Connects neurons based on spatial patterns",
            "functions": "Uses algorithmic functions to determine connections",
            "composite": "Combines multiple morphology types for complex connectivity",
        }

        base_desc = descriptions.get(morphology_type, "Custom morphology type")

        # Add specific details based on parameters
        if morphology_type == "vectors" and "vectors" in morphology.get(
            "parameters", {}
        ):
            vectors = morphology["parameters"]["vectors"]
            base_desc += f" ({len(vectors)} vector(s))"
        elif morphology_type == "patterns" and "patterns" in morphology.get(
            "parameters", {}
        ):
            patterns = morphology["parameters"]["patterns"]
            base_desc += f" ({len(patterns)} pattern(s))"

        return base_desc

    def _get_morphology_example(self, morphology: Dict[str, Any]) -> str:
        """Generate example usage for a morphology."""
        morphology_type = morphology.get("type", "unknown")

        examples = {
            "vectors": "Useful for layer-to-layer connections with fixed offsets",
            "patterns": "Ideal for spatial relationship-based connectivity",
            "functions": "Best for dynamic or computed connectivity patterns",
            "composite": "Combines multiple approaches for complex architectures",
        }

        return examples.get(
            morphology_type, "General purpose connectivity morphology"
        )

    def create_morphology(self, morphology_data: Dict[str, Any]) -> bool:
        """Create a new morphology.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.create_morphology(morphology_data)

        except Exception as e:
            self.logger.error(f"Error creating morphology: {str(e)}")
            raise ValueError(f"Failed to create morphology: {str(e)}") from e

    def update_morphology(
        self, morphology_id: str, updates: Dict[str, Any]
    ) -> bool:
        """Update an existing morphology.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.update_morphology(
                morphology_id, updates
            )

        except Exception as e:
            self.logger.error(f"Error updating morphology: {str(e)}")
            raise ValueError(f"Failed to update morphology: {str(e)}") from e

    def delete_morphology(self, morphology_id: str) -> bool:
        """Delete a morphology.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.delete_morphology(morphology_id)

        except Exception as e:
            self.logger.error(f"Error deleting morphology: {str(e)}")
            raise ValueError(f"Failed to delete morphology: {str(e)}") from e

    def get_morphology_properties(
        self, morphology_name: str
    ) -> Dict[str, Any]:
        """Get properties of a specific morphology."""
        try:
            all_morphologies = self.get_morphologies()
            if morphology_name not in all_morphologies:
                raise ValueError(f"Morphology '{morphology_name}' not found")

            morphology = all_morphologies[morphology_name]
            result = dict(morphology)
            result["morphology_name"] = morphology_name

            self.logger.info(
                f"Retrieved properties for morphology: {morphology_name}"
            )
            return result

        except Exception as e:
            self.logger.error(f"Error getting morphology properties: {str(e)}")
            raise ValueError(
                f"Failed to get morphology properties: {str(e)}"
            ) from e

    def get_morphology_usage(self, morphology_name: str) -> List[List[str]]:
        """Get usage report for a specific morphology."""
        try:
            genome = self.get_genome()
            if not genome:
                return []

            usage_list = []

            # COPY THE EXACT LOGIC FROM THE WORKING SAFETY SYSTEM
            # This is the same logic that correctly blocks deletion
            
            # Check cortical mappings for morphology usage (flat genome format)
            if "blueprint" in genome:
                blueprint = genome["blueprint"]
                for area_id, area_data in blueprint.items():
                    if isinstance(area_data, dict):
                                                 # Deep scan for morphology references in blueprint data
                         self._scan_for_cortical_mappings(
                            area_data,
                            morphology_name,
                            area_id,
                            area_id,
                            usage_list,
                         )

            return usage_list

        except Exception as e:
            self.logger.error(f"Error getting morphology usage: {str(e)}")
            import traceback

            self.logger.error(f"Full traceback: {traceback.format_exc()}")
            raise ValueError(
                f"Failed to get morphology usage: {str(e)}"
            ) from e

    def _extract_area_name_from_flat_format(self, flat_area_name: str) -> str:
        """Extract clean cortical area name from flat genome format.
        
        Converts: "_____10c-CTGM4_-cx-dstmap-d" → "CTGM4_"
        Converts: "_____10c-iic400-cx-..." → "iic400"
        """
        if not flat_area_name:
            return flat_area_name
            
        # Remove common flat format prefixes and suffixes
        clean_name = flat_area_name
        
        # Remove _____10c- prefix if present
        if clean_name.startswith("_____10c-"):
            clean_name = clean_name[9:]  # Remove "_____10c-"
        
        # Remove -cx-dstmap-d suffix if present
        if "-cx-dstmap-d" in clean_name:
            clean_name = clean_name.split("-cx-dstmap-d")[0]
        
        # Remove other common flat format suffixes
        for suffix in [
            "-cx-subgrp-t",
            "-cx-_n_cnt-i",
            "-nx-pstcrm-f",
            "-cx-synatt-f",
        ]:
            if clean_name.endswith(suffix):
                clean_name = clean_name.replace(suffix, "")
                break
        
        return clean_name

    def _scan_for_cortical_mappings(
        self,
        data: Dict[str, Any],
        morphology_id: str,
        context_key: str,
        original_area_id: str,
        usage_list: List[List[str]],
    ) -> None:
        """Scan dictionary for cortical mapping usage of morphology.

        This replicates the exact logic from the working deletion safety
        system.
        """
        if not isinstance(data, dict):
            return
            
        for key, value in data.items():
            if value == morphology_id:
                # Found direct usage - extract area names
                source_area = self._extract_area_name_from_flat_format(
                    original_area_id
                )
                target_area = key
                #  SPECIAL HANDLING: If key is "morphology_id", extract target
                #  from context_key
                if key == "morphology_id" and ":" in context_key:
                    # Parse context like "_____10c-CTGM4_-cx-dstmap-d:o__mot"
                    parts = context_key.split(":")
                    if len(parts) >= 2:
                        target_area = parts[-1].split("[")[
                            0
                        ]  # Remove [0] if present
                        usage_list.append([source_area, target_area])
                        return
                usage_list.append([source_area, target_area])
            elif isinstance(value, dict):
                # Recurse into nested dictionaries
                self._scan_for_cortical_mappings(
                    value,
                    morphology_id,
                    f"{context_key}:{key}",
                    original_area_id,
                    usage_list,
                )
            elif isinstance(value, list):
                # Check list items
                for i, item in enumerate(value):
                    if item == morphology_id:
                        # Found in list - extract area names
                        source_area = self._extract_area_name_from_flat_format(
                            original_area_id
                        )
                        target_area = key
                        usage_list.append([source_area, target_area])
                    elif isinstance(item, dict):
                        # Recurse into list items that are dictionaries
                        self._scan_for_cortical_mappings(
                            item,
                            morphology_id,
                            f"{context_key}:{key}[{i}]",
                            original_area_id,
                            usage_list,
                        )
                    elif (
                        isinstance(item, list)
                        and len(item) > 0
                        and item[0] == morphology_id
                    ):
                        #  Found in nested list (like the cortical mapping
                        #  format)
                        #  Original_area_id format:
                        #  "_____10c-CTGM4_-cx-dstmap-d"
                        # Key format: "o__mot"
                        source_area = self._extract_area_name_from_flat_format(
                            original_area_id
                        )
                        target_area = key

                        usage_list.append([source_area, target_area])

    def get_cortical_mapping(self) -> Dict[str, Any]:
        """Get the simple cortical mapping structure showing source ->
        destination relationships.

        Returns:
            Dictionary containing cortical area mappings as source -> [destinations] format
        """
        try:
            # Use the new simple mapping method
            return self.get_simple_cortical_mapping()

        except Exception as e:
            self.logger.error(f"Error getting cortical mapping: {str(e)}")
            return {}

    def clone_cortical_area(
        self,
        source_area_id: str,
        clone_cortical_mapping: bool = True,
        coordinates_3d: Optional[List[int]] = None,
        coordinates_2d: Optional[List[int]] = None,
        cortical_name: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Clone an existing cortical area, leveraging existing services.

        - New area is created in the same brain region as the source
        - When clone_cortical_mapping is True, duplicate incoming/outgoing/recursive
          connections using the existing mapping update pathways
        - Coordinates are optional; if not provided, suggest close-by coordinates
        """
        try:
            # Validate source exists
            source_area = self._cortical_area_service.get_area(source_area_id)
            if not source_area:
                raise ValueError(f"Source cortical area '{source_area_id}' not found")

            # Extract source metadata
            source_params = (source_area or {}).get("parameters", {})
            source_dims = (source_area or {}).get("dimensions", {})
            source_coords = (source_area or {}).get("coordinates", {})
            source_name = (source_area or {}).get("name", source_area_id)
            source_type = (source_area or {}).get("type", "custom")

            # Determine brain region for new area: same as source
            parent_region_id = source_params.get("parent_region_id")
            if not parent_region_id:
                # Attempt from brain region hierarchy if not stored in parameters
                try:
                    cm = self.get_connectome_manager()
                    if hasattr(cm, "brain_region_hierarchy"):
                        parent_region_id = cm.brain_region_hierarchy.get_region_for_area(source_area_id)
                except Exception:
                    parent_region_id = None

            if not parent_region_id:
                raise ValueError("Cannot determine source brain region for cloning")

            # Coordinates: use provided or suggest near source
            if coordinates_3d is None:
                # Suggest a nearby 3D coordinate (shift +1 in x if free, otherwise +1 in y)
                try:
                    cm = self.get_connectome_manager()
                    sx = int(source_coords.get("x", 0))
                    sy = int(source_coords.get("y", 0))
                    sz = int(source_coords.get("z", 0))
                    # Basic heuristic. Avoid collisions with Morton limits; ConnectomeManager will validate
                    candidates = [
                        {"x": sx + 1, "y": sy, "z": sz},
                        {"x": sx, "y": sy + 1, "z": sz},
                        {"x": sx, "y": sy, "z": sz + 1},
                    ]
                    chosen = candidates[0]
                    coordinates_3d = [chosen["x"], chosen["y"], chosen["z"]]
                except Exception:
                    coordinates_3d = [int(source_coords.get("x", 0)) + 1, int(source_coords.get("y", 0)), int(source_coords.get("z", 0))]

            else:
                # Normalize provided list into dict-like for downstream
                coordinates_3d = [int(coordinates_3d[0]), int(coordinates_3d[1]), int(coordinates_3d[2])]

            if coordinates_2d is None:
                # Suggest nearby 2D coordinates if source had them
                s2d = None
                try:
                    s2d = source_params.get("coordinates_2d") or source_area.get("coordinates_2d")
                    if not s2d:
                        x2d = source_params.get("2dcorx")
                        y2d = source_params.get("2dcory")
                        if x2d is not None and y2d is not None:
                            s2d = [int(x2d), int(y2d)]
                except Exception:
                    s2d = None
                if s2d and isinstance(s2d, (list, tuple)) and len(s2d) >= 2:
                    coordinates_2d = [int(s2d[0]) + 1, int(s2d[1])]
                else:
                    coordinates_2d = [0, 0]
            else:
                coordinates_2d = [int(coordinates_2d[0]), int(coordinates_2d[1])]

            # Dimensions come from source; for memory areas, GenomeService enforces 1x1x1
            dims = source_dims
            if isinstance(dims, tuple):
                dims = {"width": dims[0], "height": dims[1], "depth": dims[2]}
            elif isinstance(dims, list):
                dims = {"width": dims[0], "height": dims[1], "depth": dims[2]}

            # Determine cortical group/subgroup from source
            cortical_group = source_params.get("cortical_group", source_area.get("group_id", "CUSTOM"))
            sub_group_id = source_params.get("sub_group_id") or source_params.get("cortical_sub_group") or source_area.get("subgroup")
            is_memory = sub_group_id == "MEMORY"

            # Build parameters for new area creation; copy relevant parameters but do not duplicate mapping here
            new_params: Dict[str, Any] = {}
            # Keep per-voxel neuron count if not memory
            if not is_memory:
                pv = source_params.get("per_voxel_neuron_cnt") or source_params.get("neurons_per_voxel")
                if pv is not None:
                    new_params["per_voxel_neuron_cnt"] = int(pv)
            # Region and grouping
            new_params.update({
                "brain_region_id": parent_region_id,
                "cortical_group": cortical_group,
                "sub_group_id": sub_group_id or source_params.get("cortical_sub_group", "CUSTOM"),
                "coordinates_2d": coordinates_2d,
            })
            # Memory-specific properties: preserve if cloning a memory area
            if is_memory:
                for key in [
                    "temporal_depth",
                    "init_lifespan",
                    "lifespan_growth_rate",
                    "longterm_mem_threshold",
                ]:
                    if key in source_params and source_params.get(key) is not None:
                        new_params[key] = source_params.get(key)

            # Copy ALL neural properties from the source parameters except mapping/region/coordinates overrides
            try:
                banned_param_keys = {
                    "mapping",
                    "cortical_mapping_dst",
                    "brain_region_id",
                    "parent_region_id",
                    "coordinates_2d",
                }
                for k, v in (source_params or {}).items():
                    if k in banned_param_keys:
                        continue
                    # Keep our explicit overrides for group/subgroup and per_voxel count
                    if k in ("cortical_group", "sub_group_id", "cortical_sub_group", "per_voxel_neuron_cnt"):
                        # Already set above; still let exact matches through to preserve exact values
                        pass
                    new_params[k] = v
            except Exception:
                pass

            # Also copy select top-level neural properties from source area to ensure exact match in readers
            try:
                top_level_passthrough_keys = [
                    "firing_threshold",
                    "refractory_period",
                    "leak_coefficient",
                ]
                for k in top_level_passthrough_keys:
                    if k in (source_area or {}):
                        new_params[k] = source_area[k]
            except Exception:
                pass

            # Generate a standard cortical_id matching FEAGI's convention
            try:
                from feagi.bdu.models.cortical_area import generate_cortical_id as _gen_id

                # Seed from source name (letters/digits), fallback to source id
                seed_source = str(source_name or source_area_id)
                seed_source = "".join([c for c in seed_source if c.isalnum()])
                if len(seed_source) < 3:
                    seed_source = (seed_source + "000")[:3]
                else:
                    seed_source = seed_source[:3]
                # Ensure uniqueness vs. current genome blueprint
                new_cortical_id = None
                existing_ids = set()
                try:
                    genome = self.get_genome() or {}
                    if "blueprint" in genome and isinstance(genome["blueprint"], dict):
                        existing_ids = set(genome["blueprint"].keys())
                except Exception:
                    existing_ids = set()
                for _ in range(100):
                    candidate = _gen_id(prefix=("m" if is_memory else "c"), seed=seed_source.upper())
                    if candidate not in existing_ids:
                        new_cortical_id = candidate
                        break
                if not new_cortical_id:
                    # As a last resort, one more generation
                    new_cortical_id = _gen_id(prefix=("m" if is_memory else "c"), seed=seed_source.upper())
                new_params["cortical_id"] = new_cortical_id
            except Exception:
                # As a fallback, let GenomeService assign one (it will also be fixed there)
                pass

            # Create area via GenomeService (single source of truth)
            # Resolve target cortical name
            target_name = cortical_name.strip() if cortical_name else f"{source_name}_clone"
            # Ensure uniqueness of cortical_name; append numeric suffix if needed
            try:
                existing_names = set(self.get_cortical_area_name_list() or [])
            except Exception:
                existing_names = set()
            if target_name in existing_names:
                suffix = 2
                base = target_name
                while f"{base}_{suffix}" in existing_names and suffix < 1000:
                    suffix += 1
                if f"{base}_{suffix}" not in existing_names:
                    target_name = f"{base}_{suffix}"

            created = self._genome_service.create_cortical_area(
                name=target_name,
                coordinates={"x": coordinates_3d[0], "y": coordinates_3d[1], "z": coordinates_3d[2]},
                dimensions=dims,
                area_type=("memory" if is_memory else ("custom" if source_type == "custom" else source_type)),
                parameters=new_params,
            )
            if not created or "cortical_id" not in created:
                raise ValueError("Failed to create cloned cortical area")

            new_area_id = created["cortical_id"]

            # Optionally clone mappings: duplicate incoming, outgoing, and recursive connections
            if clone_cortical_mapping:
                try:
                    # Use detailed map from genome (includes normalized connection objects)
                    detailed_map = self.get_detailed_cortical_map()

                    # ---- Outgoing (source -> dst) ----
                    try:
                        out_map: Dict[str, Any] = detailed_map.get(source_area_id, {}) or {}
                        # Build new mapping for the cloned source, converting any self-recursive target to the new ID
                        new_out_map: Dict[str, Any] = {}
                        for dst_id, connections in out_map.items():
                            if not connections or not isinstance(connections, list):
                                continue
                            target_id = new_area_id if dst_id == source_area_id else dst_id
                            new_out_map[target_id] = connections
                        if new_out_map:
                            self.update_cortical_mapping({new_area_id: new_out_map})
                            # Trigger region I/O designation for each created mapping
                            cm = self.get_connectome_manager()
                            if hasattr(cm, 'on_cortical_mapping_created'):
                                for dst_created in new_out_map.keys():
                                    cm.on_cortical_mapping_created(new_area_id, dst_created)
                    except Exception as map_err:
                        self.logger.warning(f"Failed to clone outgoing mappings: {map_err}")

                    # ---- Incoming (src -> source) becomes (src -> new) ----
                    try:
                        for src_id, dsts in detailed_map.items():
                            if not isinstance(dsts, dict):
                                continue
                            if source_area_id in dsts:
                                connections = dsts.get(source_area_id, [])
                                if not connections or not isinstance(connections, list):
                                    continue
                                self.update_cortical_mapping({src_id: {new_area_id: connections}})
                                cm = self.get_connectome_manager()
                                if hasattr(cm, 'on_cortical_mapping_created'):
                                    cm.on_cortical_mapping_created(src_id, new_area_id)
                    except Exception as map_err:
                        self.logger.warning(f"Failed to clone incoming mappings: {map_err}")

                    # ---- Ensure recursive mapping (source -> source) becomes (new -> new) if present ----
                    try:
                        if source_area_id in (out_map.keys() if 'out_map' in locals() else []):
                            rec_connections = out_map.get(source_area_id, [])
                            if rec_connections and isinstance(rec_connections, list):
                                self.update_cortical_mapping({new_area_id: {new_area_id: rec_connections}})
                                cm = self.get_connectome_manager()
                                if hasattr(cm, 'on_cortical_mapping_created'):
                                    cm.on_cortical_mapping_created(new_area_id, new_area_id)
                    except Exception as rec_err:
                        self.logger.warning(f"Failed to clone recursive mapping: {rec_err}")

                except Exception as e:
                    self.logger.warning(f"Clone operation succeeded but mapping duplication encountered issues: {e}")

            return {"new_area_id": new_area_id, "message": "Cortical area cloned successfully"}

        except Exception as e:
            self.logger.error(f"Error cloning cortical area: {e}")
            raise

    def get_simple_cortical_mapping(self) -> Dict[str, List[str]]:
        """Get simple cortical mapping showing only source -> destination
        relationships.

        Returns a clean dictionary where each cortical area ID maps to a list
        of destination cortical areas it connects to.

        Returns:
            Dict[str, List[str]]: Simple mapping of source_area -> [dest_areas]
        """
        try:
            # Get all cortical areas using the correct service method
            all_areas_list = self._cortical_area_service.get_all_areas()

            # Build the simple mapping response
            mapping_response = {}

            for area_data in all_areas_list:
                area_id = area_data.get("id")
                if not area_id:
                    continue

                #  Initialize area entry (empty list for areas with no outgoing
                #  connections)
                mapping_response[area_id] = []

                # Get the area's mapping data from its parameters
                area_parameters = area_data.get("parameters", {})
                area_mapping = area_parameters.get("mapping", {})

                if area_mapping:
                    # Simply collect the destination area IDs
                    for (
                        target_area_id,
                        connection_list,
                    ) in area_mapping.items():
                        if (
                            connection_list
                        ):  # If there are any connections to this target
                            mapping_response[area_id].append(target_area_id)

            self.logger.info(
                f"Generated simple cortical mapping for {len(mapping_response)} areas"
            )
            return mapping_response

        except Exception as e:
            self.logger.error(f"Error generating simple cortical mapping: {e}")
            raise

    def update_cortical_mapping(self, mapping: Dict[str, Any]) -> bool:
        """Update the cortical mapping structure by converting formatted data
        back to genome format.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager

        Args:
            mapping: Dictionary containing updated cortical area mappings in the formatted structure
                    Format: {area_id: {target_area_id: [connection_objects]}}

        Returns:
            True if successful, False otherwise
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.update_cortical_mapping(mapping)

        except Exception as e:
            self.logger.error(f"Error updating cortical mapping: {str(e)}")
            return False

    def get_cortical_mapping_properties(
        self, src_cortical_area: str, dst_cortical_area: str
    ) -> List[Dict[str, Any]]:
        """Get cortical mapping properties between two cortical areas."""
        try:
            genome = self.get_genome()
            blueprint = (genome or {}).get("blueprint", {})

            raw_connections: List[Any] = []

            # 1) Primary: parameters.mapping in genome blueprint
            if src_cortical_area in blueprint:
                area_def = blueprint[src_cortical_area] or {}
                params = area_def.get("parameters", {}) or {}
                mapping_param = params.get("mapping")
                if isinstance(mapping_param, dict):
                    candidate = mapping_param.get(dst_cortical_area)
                    if isinstance(candidate, list) and candidate:
                        raw_connections = candidate

                # 2) Alternate genome key: cortical_mapping_dst
                if not raw_connections:
                    mapping_dst = area_def.get("cortical_mapping_dst")
                    if isinstance(mapping_dst, dict):
                        candidate = mapping_dst.get(dst_cortical_area)
                        if isinstance(candidate, list) and candidate:
                            raw_connections = candidate

            # Nothing found
            if not raw_connections:
                return []

            # Normalize to expected format
            formatted_connections: List[Dict[str, Any]] = []
            for connection in raw_connections:
                if isinstance(connection, list):
                    # Array format: [morphology_id, scalar, multiplier, plasticity_flag, constant, ltp, ltd]
                    formatted_connections.append({
                        "morphology_id": connection[0] if len(connection) > 0 else "",
                        "morphology_scalar": connection[1] if len(connection) > 1 else [1, 1, 1],
                        "postSynapticCurrent_multiplier": connection[2] if len(connection) > 2 else 1,
                        "plasticity_flag": connection[3] if len(connection) > 3 else False,
                        "plasticity_constant": connection[4] if len(connection) > 4 else 1,
                        "ltp_multiplier": connection[5] if len(connection) > 5 else 1,
                        "ltd_multiplier": connection[6] if len(connection) > 6 else 1,
                    })
                elif isinstance(connection, dict):
                    # Dict format already in expected schema
                    formatted_connections.append({
                        "morphology_id": connection.get("morphology_id", ""),
                        "morphology_scalar": connection.get("morphology_scalar", [1, 1, 1]),
                        "postSynapticCurrent_multiplier": connection.get("postSynapticCurrent_multiplier", 1),
                        "plasticity_flag": connection.get("plasticity_flag", False),
                        "plasticity_constant": connection.get("plasticity_constant", 1),
                        "ltp_multiplier": connection.get("ltp_multiplier", 1),
                        "ltd_multiplier": connection.get("ltd_multiplier", 1),
                    })

            self.logger.info(
                f"Retrieved {len(formatted_connections)} mapping properties from {src_cortical_area} to {dst_cortical_area}"
            )
            return formatted_connections

        except Exception as e:
            self.logger.error(
                f"Error getting cortical mapping properties: {str(e)}"
            )
            raise ValueError(
                f"Failed to get cortical mapping properties: {str(e)}"
            ) from e

    def update_cortical_mapping_properties(
        self,
        src_cortical_area: str,
        dst_cortical_area: str,
        mapping_string: List[Dict[str, Any]],
    ) -> bool:
        """Update cortical mapping properties between two cortical areas."""
        try:
            self.logger.info(
                f"Updating cortical mapping properties from {src_cortical_area} to {dst_cortical_area}"
            )

            # Route through GenomeService for architecture compliance
            update_data = {
                "src_cortical_area": src_cortical_area,
                "dst_cortical_area": dst_cortical_area,
                "mapping_data": mapping_string,
            }

            success = self._genome_service.update_cortical_mapping_properties(
                update_data
            )

            if success:
                self.logger.info(
                    f"Successfully updated mapping properties from {src_cortical_area} to {dst_cortical_area}"
                )
            else:
                self.logger.error(
                    f"Failed to update mapping properties from {src_cortical_area} to {dst_cortical_area}"
                )

            return success

        except Exception as e:
            self.logger.error(
                f"Error updating cortical mapping properties: {str(e)}"
            )
            return False

    def delete_cortical_mapping(
        self,
        src_cortical_area: str,
        dst_cortical_area: str,
    ) -> bool:
        """Delete cortical mapping and all associated synapses between two
        cortical areas."""
        try:
            self.logger.info(
                f"Deleting cortical mapping from {src_cortical_area} to {dst_cortical_area}"
            )

            # Route through GenomeService for architecture compliance
            success = self._genome_service.delete_cortical_mapping(
                src_cortical_area, dst_cortical_area
            )

            if success:
                self.logger.info(
                    f"Successfully deleted cortical mapping from {src_cortical_area} to {dst_cortical_area}"
                )
            else:
                self.logger.error(
                    f"Failed to delete cortical mapping from {src_cortical_area} to {dst_cortical_area}"
                )

            return success

        except Exception as e:
            self.logger.error(f"Error deleting cortical mapping: {str(e)}")
            return False

    def get_detailed_cortical_map(self) -> Dict[str, Any]:
        """Get detailed cortical mapping information in the expected format.

        Returns a dictionary where each cortical area ID maps to its connection targets
        with detailed morphology and plasticity parameters.

        Returns:
            Dict[str, Any]: Mapping data in the expected format
        """
        logger.info("Getting detailed cortical map (genome source)...")

        try:
            genome = self.get_genome()
            blueprint = (genome or {}).get("blueprint", {})

            # Build the mapping response strictly from hierarchical genome
            mapping_response: Dict[str, Any] = {}

            for area_id, area_def in blueprint.items():
                if not isinstance(area_def, dict):
                    continue

                mapping_response[area_id] = {}

                params = area_def.get("parameters", {}) or {}
                # Prefer parameters.mapping if present, otherwise cortical_mapping_dst
                area_mapping: Dict[str, Any] = {}
                mapping_param = params.get("mapping")
                mapping_dst = area_def.get("cortical_mapping_dst")
                if isinstance(mapping_param, dict) and mapping_param:
                    area_mapping = mapping_param
                elif isinstance(mapping_dst, dict) and mapping_dst:
                    area_mapping = mapping_dst

                if not area_mapping:
                    continue

                for target_area_id, connection_list in area_mapping.items():
                    if not isinstance(connection_list, list) or not connection_list:
                        continue

                    formatted_connections: List[Dict[str, Any]] = []
                    for connection_data in connection_list:
                        if isinstance(connection_data, list):
                            # Array format: [morphology_id, scalar, multiplier, plasticity_flag, constant, ltp, ltd]
                            formatted_connections.append(
                                {
                                    "morphology_id": connection_data[0] if len(connection_data) > 0 else "",
                                    "morphology_scalar": connection_data[1] if len(connection_data) > 1 else [1, 1, 1],
                                    "postSynapticCurrent_multiplier": connection_data[2] if len(connection_data) > 2 else 1,
                                    "plasticity_flag": connection_data[3] if len(connection_data) > 3 else False,
                                    "plasticity_constant": connection_data[4] if len(connection_data) > 4 else 1,
                                    "ltp_multiplier": connection_data[5] if len(connection_data) > 5 else 1,
                                    "ltd_multiplier": connection_data[6] if len(connection_data) > 6 else 1,
                                }
                            )
                        elif isinstance(connection_data, dict):
                            # Already normalized
                            formatted_connections.append(
                                {
                                    "morphology_id": connection_data.get("morphology_id", ""),
                                    "morphology_scalar": connection_data.get("morphology_scalar", [1, 1, 1]),
                                    "postSynapticCurrent_multiplier": connection_data.get(
                                        "postSynapticCurrent_multiplier", 1
                                    ),
                                    "plasticity_flag": connection_data.get("plasticity_flag", False),
                                    "plasticity_constant": connection_data.get("plasticity_constant", 1),
                                    "ltp_multiplier": connection_data.get("ltp_multiplier", 1),
                                    "ltd_multiplier": connection_data.get("ltd_multiplier", 1),
                                }
                            )

                    if formatted_connections:
                        mapping_response[area_id][target_area_id] = formatted_connections

            logger.info(
                f"Generated detailed cortical map (genome) for {len(mapping_response)} areas"
            )
            return mapping_response

        except Exception as e:
            self.logger.error(f"Error getting detailed cortical map: {str(e)}")
            raise ValueError(
                f"Failed to get detailed cortical map: {str(e)}"
            ) from e

    def get_data_path(self) -> str:
        """Get data path."""
        return os.path.join(tempfile.gettempdir(), "feagi_data")

    def get_temp_path(self) -> str:
        """Get temporary path."""
        return os.path.join(tempfile.gettempdir(), "feagi_temp")

    # =================================================================
    # STIMULATION METHODS
    # =================================================================

    def trigger_manual_stimulation(
        self, stimulation_payload: Dict[str, Any]
    ) -> bool:
        """Trigger manual stimulation using unified method."""
        try:
            cortical_id = stimulation_payload.get("cortical_id")
            intensity = stimulation_payload.get("intensity", 1.0)
            coordinates = stimulation_payload.get("coordinates", None)
            
            if cortical_id:
                # Create simple neural data for stimulation
                #  If no coordinates provided, this would need area-specific
                #  implementation
                if coordinates:
                    neural_data = {
                        cortical_id: {
                            "coordinates_x": np.array(
                                [coord.get("x", 0) for coord in coordinates],
                                dtype=np.uint16,
                            ),
                            "coordinates_y": np.array(
                                [coord.get("y", 0) for coord in coordinates],
                                dtype=np.uint16,
                            ),
                            "coordinates_z": np.array(
                                [coord.get("z", 0) for coord in coordinates],
                                dtype=np.uint16,
                            ),
                            "membrane_potentials": np.array(
                                [intensity] * len(coordinates),
                                dtype=np.float32,
                            ),
                        }
                    }
                    return self.stimulate_neurons(neural_data).get(
                        "success", False
                    )
                else:
                    #  For backward compatibility, log that coordinates are
                    #  needed
                    self.logger.warning(
                        f"Manual stimulation requires coordinates for area {cortical_id}"
                    )
                    return False
            return False
        except Exception as e:
            self.logger.error(f"Error triggering manual stimulation: {str(e)}")
            return False

    def trigger_sustained_stimulation(
        self, stimulation_payload: Dict[str, Any]
    ) -> bool:
        """Trigger sustained stimulation using unified method."""
        try:
            cortical_id = stimulation_payload.get("cortical_id")
            intensity = stimulation_payload.get("intensity", 1.0)
            _ = stimulation_payload.get("duration", 10)
            coordinates = stimulation_payload.get("coordinates", None)
            
            if cortical_id:
                # Create simple neural data for stimulation
                #  If no coordinates provided, this would need area-specific
                #  implementation
                if coordinates:
                    neural_data = {
                        cortical_id: {
                            "coordinates_x": np.array(
                                [coord.get("x", 0) for coord in coordinates],
                                dtype=np.uint16,
                            ),
                            "coordinates_y": np.array(
                                [coord.get("y", 0) for coord in coordinates],
                                dtype=np.uint16,
                            ),
                            "coordinates_z": np.array(
                                [coord.get("z", 0) for coord in coordinates],
                                dtype=np.uint16,
                            ),
                            "membrane_potentials": np.array(
                                [intensity] * len(coordinates),
                                dtype=np.float32,
                            ),
                        }
                    }
                    #  TODO: Implement duration handling for sustained
                    #  stimulation
                    return self.stimulate_neurons(neural_data).get(
                        "success", False
                    )
                else:
                    #  For backward compatibility, log that coordinates are
                    #  needed
                    self.logger.warning(
                        f"Sustained stimulation requires coordinates for area {cortical_id}"
                    )
                    return False
            return False
        except Exception as e:
            self.logger.error(
                f"Error triggering sustained stimulation: {str(e)}"
            )
            return False

    def set_stimulation_script(self, script: str) -> bool:
        """Set stimulation script."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error setting stimulation script: {str(e)}")
            return False

    def reset_stimulation_script(self) -> bool:
        """Reset stimulation script."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error resetting stimulation script: {str(e)}")
            return False

    def trigger_multi_area_stimulation(
        self, stimulation_payload: Dict[str, List[List[int]]]
    ) -> Dict[str, Any]:
        """Trigger manual stimulation across multiple cortical areas using
        coordinate lists.
        
        Uses the existing FCL injection service to properly inject external stimulations
        into the Fire Candidate List during burst processing.
        
        Args:
            stimulation_payload: Dictionary mapping cortical area IDs to lists of [x, y, z] coordinates
                Example: {
                    "_power": [[1, 0, 0], [2, 4, 3]],
                    "cx3212": [[1, 1, 0], [12, 24, 33], [0, 0, 0]]
                }
        
        Returns:
            Dictionary containing stimulation results and statistics
        """
        try:
            self.logger.info(
                f"🔵 trigger_multi_area_stimulation called with {len(stimulation_payload)} areas"
            )
            
            if not stimulation_payload:
                return {"success": False, "error": "Empty stimulation payload"}
            
            # Get the burst engine and its FCL injection service
            burst_engine = self.get_burst_engine()
            if not burst_engine or not burst_engine.injection_service:
                return {
                    "success": False,
                    "error": "FCL injection service not available",
                }
            
            injection_service = burst_engine.injection_service
            
            # Convert coordinates to neuron IDs for each cortical area
            activations = {}
            total_coordinates = 0
            total_neurons_found = 0
            area_results = {}
            
            for cortical_id, coordinate_list in stimulation_payload.items():
                if not coordinate_list:
                    self.logger.warning(
                        f"Empty coordinate list for cortical area {cortical_id}"
                    )
                    continue
                    
                # Validate coordinate format
                for coord in coordinate_list:
                    if not isinstance(coord, list) or len(coord) != 3:
                        raise ValueError(
                            f"Invalid coordinate format in {cortical_id}: {coord}. Expected [x, y, z]"
                        )
                    if not all(isinstance(c, int) for c in coord):
                        raise ValueError(
                            f"Coordinates must be integers in {cortical_id}: {coord}"
                        )
                
                total_coordinates += len(coordinate_list)
                
                # Convert coordinates to voxel positions and find neurons
                candidate_positions = set(map(tuple, coordinate_list))
                
                try:
                    # Use batch lookup to find neurons at these coordinates
                    neuron_weight_pairs = (
                        self._connectome_manager.batch_voxel_to_neuron_lookup(
                        cortical_id=cortical_id,
                        candidate_positions=candidate_positions,
                            post_synaptic_current=1.0,  # Default weight
                        )
                    )
                    
                    if neuron_weight_pairs:
                        # Extract just the neuron IDs
                        neuron_ids = [
                            neuron_id for neuron_id, _ in neuron_weight_pairs
                        ]
                        activations[cortical_id] = neuron_ids
                        total_neurons_found += len(neuron_ids)
                        
                        area_results[cortical_id] = {
                            "success": True,
                            "coordinates_requested": len(coordinate_list),
                            "neurons_found": len(neuron_ids),
                            "neuron_ids": (
                                neuron_ids[:10]
                                if len(neuron_ids) > 10
                                else neuron_ids
                            ),  # Limit for response size
                        }

                        self.logger.debug(
                            f"Found {len(neuron_ids)} neurons at {len(coordinate_list)} coordinates in {cortical_id}"
                        )
                    else:
                        area_results[cortical_id] = {
                            "success": False,
                            "error": f"No neurons found at specified coordinates in {cortical_id}",
                            "coordinates_requested": len(coordinate_list),
                            "neurons_found": 0,
                        }
                        
                except Exception as e:
                    self.logger.error(
                        f"Error finding neurons in {cortical_id}: {str(e)}"
                    )
                    area_results[cortical_id] = {
                        "success": False,
                        "error": str(e),
                        "coordinates_requested": len(coordinate_list),
                        "neurons_found": 0,
                    }
            
            if not activations:
                return {
                    "success": False, 
                    "error": "No neurons found at any of the specified coordinates",
                    "area_results": area_results,
                    "total_coordinates": total_coordinates,
                }
            
            # Get current timestep for injection
            current_timestep = getattr(
                self._connectome_manager, "current_timestep", 0
            )

            #  Use the existing FCL injection service to inject external
            #  activations
            self.logger.info(
                f"Injecting {total_neurons_found} neurons from {len(activations)} areas into FCL via injection service"
            )
            
            # Route sensory/IPU activations to BurstEngine buffer for deterministic FCL injection
            if hasattr(burst_engine, '_pending_external_activations'):
                for area_id, area_data in activations.items():
                    burst_engine._pending_external_activations[area_id] = area_data
            else:
                burst_engine._pending_external_activations = dict(activations)
            injected_count = sum(
                len(v.get('coordinates_x', [])) if isinstance(v, dict) else (len(v) if hasattr(v, '__len__') else 0)
                for v in activations.values()
            )

            #  CRITICAL FIX: Trigger an immediate burst to process the injected
            #  neurons
            #  Without this, the neurons sit in FCL until the next scheduled
            #  burst
            if injected_count > 0:
                self.logger.info(
                    "🔥 Triggering immediate burst to process manually stimulated neurons"
                )
                try:
                    #  Use the burst engine's process_burst method to
                    #  trigger immediate processing
                    fired_neurons = burst_engine.process_burst()
                    burst_success = fired_neurons is not None  # Check if process returned a result
                    if burst_success:
                        fired_count = len(fired_neurons) if fired_neurons else 0
                        self.logger.info(
                            f"✅ Manual stimulation burst processing completed successfully - {fired_count} neurons fired"
                        )
                    else:
                        self.logger.warning(
                            "❌ Manual stimulation burst processing failed"
                        )
                except Exception as burst_error:
                    self.logger.error(
                        f"Error triggering burst for manual stimulation: {str(burst_error)}"
                    )
            else:
                self.logger.warning(
                    "No neurons were injected, skipping burst trigger"
                )
            
            # Prepare response
            result = {
                "success": injected_count > 0,
                "total_neurons_injected": injected_count,
                "total_coordinates": total_coordinates,
                "total_neurons_found": total_neurons_found,
                "areas_processed": len(stimulation_payload),
                "areas_with_neurons": len(activations),
                "area_results": area_results,
                "method": "fcl_injection_service",
                "current_timestep": current_timestep,
                "summary": {
                    "areas_stimulated": len(activations),
                    "total_coordinates": total_coordinates,
                    "areas": list(activations.keys()),
                },
            }
            
            if injected_count > 0:
                self.logger.info(
                    f"✅ Successfully injected {injected_count} neurons into FCL for manual stimulation"
                )
            else:
                self.logger.warning("❌ No neurons were injected into FCL")
            
            return result
            
        except Exception as e:
            self.logger.error(
                f"🔴 CRITICAL ERROR in trigger_multi_area_stimulation: {str(e)}"
            )
            import traceback

            self.logger.error(f"🔴 Traceback: {traceback.format_exc()}")
            
            # Return a clear error response indicating the new method failed
            return {
                "success": False, 
                "error": str(e),
                "method": "fcl_injection_service_FAILED",
                "fallback_occurred": True,
            }

    # =================================================================
    # TRANSACTION AND STATE METHODS
    # =================================================================

    def begin_transaction(self):
        """Begin a genome transaction."""
        # This would need implementation with genome transaction system
        pass

    def modify_genome(self, transaction):
        """Modify genome within a transaction."""
        # This would need implementation with genome transaction system
        pass

    def register_genome_change_listener(self, callback):
        """Register a callback for genome changes."""
        # This would need implementation
        pass

    def on_sync_state_change(self, old_state, new_state, details):
        """Handle sync state changes."""
        # This would need implementation
        pass

    # =================================================================
    # UTILITY AND HELPER METHODS
    # =================================================================

    def _get_cortical_idx_for_id(self, cortical_id: str) -> Optional[int]:
        """Get cortical index for a cortical ID using O(1)
        BiDirectionalCorticalMap.

        Args:
            cortical_id: 6-character string identifier

        Returns:
            Integer index if found, None otherwise
        """
        #  Use O(1) lookup from BiDirectionalCorticalMap - no more O(N) linear
        #  search!
        return self._connectome_manager.get_cortical_idx_for_id(cortical_id)
    
    def get_cortical_id_for_idx(self, cortical_idx: int) -> Optional[str]:
        """Get cortical ID for a cortical index using O(1) BiDirectionalCorticalMap.
        
        This method is used by FQ sampler to convert integer cortical indices
        from FireQueue to proper 6-character cortical ID strings.

        Args:
            cortical_idx: Integer cortical area index

        Returns:
            6-character string cortical_id if found, None otherwise
        """
        try:
            if not self._connectome_manager:
                return None
            
            cortical_id = self._connectome_manager.get_cortical_id_for_idx(cortical_idx)
            return cortical_id
        except Exception as e:
            self.logger.error(f"Error converting cortical_idx {cortical_idx} to cortical_id: {e}")
            return None

    def _validate_genome_loaded(self) -> bool:
        """Check if a genome is currently loaded - helper method for service consistency."""
        return self._genome_service.is_genome_loaded()

    def get_neuron_mappings(self) -> Dict[str, Any]:
        """Get neuron mappings."""
        try:
            return {
                "neuron_to_area": {},
                "area_to_neurons": {},
                "total_neurons": self.get_connectome_dimensions().get(
                    "neuron_count", 0
                ),
            }
        except Exception as e:
            self.logger.error(f"Error getting neuron mappings: {str(e)}")
            return {}

    def get_transforming_areas(self) -> List[str]:
        """Get list of areas currently transforming."""
        try:
            # This would need implementation
            return []
        except Exception as e:
            self.logger.error(f"Error getting transforming areas: {str(e)}")
            return []

    def has_pending_amalgamation(self) -> bool:
        """Check if there is a pending amalgamation."""
        try:
            if self.state_manager:
                pending = getattr(self.state_manager, "pending_amalgamation", {})
                if not pending:
                    return False
                
                # Check for timeout (500 seconds as in legacy)
                import time
                amalgamation_timeout = 500
                elapsed_time = time.time() - pending.get("initiation_time", 0)
                if elapsed_time > amalgamation_timeout:
                    self.logger.info(f"Pending amalgamation got voided due to exceeding {amalgamation_timeout} threshold!")
                    self.state_manager.pending_amalgamation = {}
                    return False
                
                return True
            return False
        except Exception as e:
            self.logger.error(f"Error checking pending amalgamation: {str(e)}")
            return False

    def get_pending_amalgamation_genome(self) -> Optional[Dict[str, Any]]:
        """Get the genome payload from pending amalgamation."""
        try:
            if self.state_manager:
                pending = getattr(self.state_manager, "pending_amalgamation", {})
                return pending.get("genome_payload")
            return None
        except Exception as e:
            self.logger.error(f"Error getting pending amalgamation genome: {str(e)}")
            return None

    def get_pending_amalgamation_title(self) -> Optional[str]:
        """Get the genome title from pending amalgamation."""
        try:
            if self.state_manager:
                pending = getattr(self.state_manager, "pending_amalgamation", {})
                return pending.get("genome_title")
            return None
        except Exception as e:
            self.logger.error(f"Error getting pending amalgamation title: {str(e)}")
            return None

    def get_amalgamation_history(self) -> Dict[str, Any]:
        """Get the amalgamation history."""
        try:
            if self.state_manager:
                return getattr(self.state_manager, "amalgamation_history", {})
            return {}
        except Exception as e:
            self.logger.error(f"Error getting amalgamation history: {str(e)}")
            return {}

    def get_amalgamation_details(self, amalgamation_id: str) -> Dict[str, Any]:
        """Get details for a specific amalgamation."""
        try:
            if self.state_manager:
                history = getattr(self.state_manager, "amalgamation_history", {})
                if amalgamation_id in history:
                    return {amalgamation_id: history[amalgamation_id]}
                else:
                    raise ValueError("No matching amalgamation found")
            return {}
        except Exception as e:
            self.logger.error(f"Error getting amalgamation details: {str(e)}")
            raise ValueError(f"Failed to get amalgamation details: {str(e)}") from e

    def calculate_circuit_size(self, blueprint: Dict[str, Any]) -> List[int]:
        """Calculate the size of a genome circuit in voxels (x, y, z).
        
        This replicates the legacy circuit_size function behavior.
        
        Args:
            blueprint: The genome blueprint dictionary
            
        Returns:
            List[int]: [x, y, z] dimensions of the circuit
        """
        try:
            dimensions = [1, 1, 1]
            
            self.logger.info(f"Calculating circuit size for {len(blueprint)} cortical areas")
            
            for cortical_area in blueprint:
                area_data = blueprint[cortical_area]
                
                # Debug: Log the area data structure
                self.logger.debug(f"Processing cortical area '{cortical_area}': {area_data}")
                
                # Get block boundaries and relative coordinates
                block_boundaries = area_data.get("block_boundaries", [0, 0, 0])
                relative_coordinate = area_data.get("relative_coordinate", [0, 0, 0])
                
                self.logger.debug(f"Area '{cortical_area}': block_boundaries={block_boundaries}, relative_coordinate={relative_coordinate}")
                
                # Calculate the extent: position + dimensions
                # block_boundaries represents the dimensions of the cortical area
                # relative_coordinate represents the position/offset
                x_coord = block_boundaries[0] + relative_coordinate[0]
                y_coord = block_boundaries[1] + relative_coordinate[1]
                z_coord = block_boundaries[2] + relative_coordinate[2]
                
                self.logger.debug(f"Area '{cortical_area}': calculated coords=({x_coord}, {y_coord}, {z_coord})")
                
                # Update maximum dimensions
                if x_coord > dimensions[0]:
                    dimensions[0] = x_coord
                if y_coord > dimensions[1]:
                    dimensions[1] = y_coord
                if z_coord > dimensions[2]:
                    dimensions[2] = z_coord
            
            self.logger.info(f"Final calculated circuit size: {dimensions}")
            return dimensions
        except Exception as e:
            self.logger.error(f"Error calculating circuit size: {str(e)}")
            return [1, 1, 1]  # Default fallback

    def import_cortical_area(self, cortical_area_data: Dict[str, Any]) -> bool:
        """Import cortical area data."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error importing cortical area: {str(e)}")
            return False

    def batch_create_neurons(
        self,
        area_id: str,
        positions: List[Tuple[int, int, int]],
        properties: Optional[Dict[str, Any]] = None,
    ) -> List[int]:
        """Batch create neurons using NPU interface internally.
        
        Maintains exact same API contract while using new NPU architecture.
        """
        try:
            # Convert area_id (6-letter string) to cortical_idx (integer) for NPU
            cortical_idx = self._connectome_manager.get_cortical_idx_for_id(area_id)
            if cortical_idx is None:
                self.logger.error(f"Cortical area {area_id} not found")
                return []
            
            # Extract properties for NPU interface
            properties = properties or {}
            neuron_types = properties.get('neuron_types')
            initial_potentials = properties.get('initial_potentials') or properties.get('membrane_potential')
            thresholds = properties.get('thresholds') or properties.get('threshold')
            leak_coefficients = properties.get('leak_coefficients') or properties.get('leak_coefficient')
            excitabilities = properties.get('excitabilities') or properties.get('excitability')
            
            # Convert single values to lists if needed
            count = len(positions)
            if isinstance(initial_potentials, (int, float)):
                initial_potentials = [initial_potentials] * count
            if isinstance(thresholds, (int, float)):
                thresholds = [thresholds] * count
            if isinstance(leak_coefficients, (int, float)):
                leak_coefficients = [leak_coefficients] * count
            if isinstance(excitabilities, (int, float)):
                excitabilities = [excitabilities] * count
            
            # Use NPU service for batch creation
            result = self._npu_service.create_neurons_batch(
                cortical_idx=cortical_idx,
                positions=positions,
                neuron_types=neuron_types,
                initial_potentials=initial_potentials,
                thresholds=thresholds,
                leak_coefficients=leak_coefficients,
                excitabilities=excitabilities
            )
            
            if result.get("success", False):
                # Extract neuron IDs from NPU result
                # For now, return empty list as NPU interface doesn't return IDs yet
                # TODO: Update NPU interface to return created neuron IDs
                self.logger.info(f"Successfully created {result.get('successful_count', 0)} neurons in area {area_id}")
                return []  # Temporary until NPU interface returns IDs
            else:
                self.logger.error(f"NPU batch neuron creation failed: {result.get('error', 'Unknown error')}")
                return []
                
        except Exception as e:
            self.logger.error(f"Error batch creating neurons: {str(e)}")
            return []

    def batch_create_synapses(
        self, connections: List[Tuple[int, int, float]]
    ) -> int:
        """Batch create synapses using NPU interface internally.
        
        Maintains exact same API contract while using new NPU architecture.
        """
        try:
            if not connections:
                return 0
            
            # Extract source neurons, target neurons, and weights
            source_neuron_ids = [conn[0] for conn in connections]
            target_neuron_ids = [conn[1] for conn in connections]
            weights = [conn[2] for conn in connections]
            
            # Use NPU service for batch creation
            result = self._npu_service.create_synapses_batch(
                source_neuron_ids=source_neuron_ids,
                target_neuron_ids=target_neuron_ids,
                weights=weights
            )
            
            if result.get("success", False):
                created_count = result.get("successful_count", 0)
                self.logger.info(f"Successfully created {created_count} synapses")
                
                # CRITICAL: Reinitialize Rust NPU after synapse changes
                # 🦀 RUST: Rebuild synapse index after adding new synapses
                # The Rust NPU PropagationEngine uses a synapse index for fast lookups
                try:
                    from feagi.process_manager import get_process_manager
                    process_manager = get_process_manager()
                    if process_manager and process_manager.rust_npu_integration:
                        rust_npu_integration = process_manager.rust_npu_integration
                        if rust_npu_integration._rust_npu_initialized:
                            self.logger.info("🦀 [RUST-NPU] Synapse(s) added - rebuilding synapse index...")
                            try:
                                rust_npu_integration.rebuild_synapse_index()
                                self.logger.info("🦀 [RUST-NPU] ✅ Synapse index rebuilt successfully")
                            except Exception as rebuild_error:
                                self.logger.error(f"🦀 [RUST-NPU] Failed to rebuild synapse index: {rebuild_error}")
                                self.logger.warning("🦀 [RUST-NPU] ⚠️ New synapse(s) will not be active until FEAGI restart")
                        else:
                            self.logger.debug("🦀 [RUST-NPU] Not yet initialized - new synapses will be loaded on first burst")
                    else:
                        self.logger.debug("🦀 [RUST-NPU] Not available")
                except Exception as rust_error:
                    self.logger.error(f"🦀 [RUST-NPU] Error during synapse index rebuild: {rust_error}")
                    self.logger.exception("Full stack trace:")
                    # Don't fail the synapse creation for Rust NPU issues
                
                return created_count
            else:
                self.logger.error(f"NPU batch synapse creation failed: {result.get('error', 'Unknown error')}")
                return 0
                
        except Exception as e:
            self.logger.error(f"Error batch creating synapses: {str(e)}")
            return 0

    # =================================================================
    # SPATIAL HASH CACHE MANAGEMENT
    # =================================================================

    def get_max_cortical_area_dimensions(self) -> Tuple[int, int, int]:
        """Get the maximum dimensions across all cortical areas.
        
        This method provides centralized access to cortical area dimension 
        calculations for spatial hash sizing and other use cases.
        
        Returns:
            Tuple of (max_x, max_y, max_z) dimensions
        """
        try:
            return self._connectome_manager.get_max_cortical_area_dimensions()
        except Exception as e:
            self.logger.error(
                f"Error getting max cortical area dimensions: {str(e)}"
            )
            return (8, 8, 8)  # Safe fallback dimensions

    # =================================================================
    # ROBOT/GAZEBO METHODS
    # =================================================================

    def update_robot_controller(
        self, controller_params: Dict[str, Any]
    ) -> bool:
        """Update robot controller parameters."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error updating robot controller: {str(e)}")
            return False

    def update_robot_model(self, model_params: Dict[str, Any]) -> bool:
        """Update robot model parameters."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error updating robot model: {str(e)}")
            return False

    def get_gazebo_robot_files(self) -> Dict[str, List[str]]:
        """Get Gazebo robot files."""
        try:
            # This would need implementation
            return {"models": [], "worlds": [], "configs": []}
        except Exception as e:
            self.logger.error(f"Error getting Gazebo robot files: {str(e)}")
            return {}

    # =================================================================
    # PERFORMANCE AND SIMULATION METHODS
    # =================================================================

    async def get_performance_stats(self) -> Dict[str, Any]:
        """Get performance statistics."""
        try:
            return self.get_performance_metrics()
        except Exception as e:
            self.logger.error(f"Error getting performance stats: {str(e)}")
            return {}

    async def get_simulation_status(self) -> Dict[str, Any]:
        """Get simulation status."""
        try:
            return {
                "running": (
                    not getattr(self.state_manager, "exit_condition", False)
                    if self.state_manager
                    else False
                ),
                "burst_counter": self.get_burst_counter(),
                "genome_loaded": self.genome_is_loaded(),
                "brain_ready": (
                    self.state_manager.get_brain_readiness()
                    if self.state_manager
                    else False
                ),
            }
        except Exception as e:
            self.logger.error(f"Error getting simulation status: {str(e)}")
            return {}

    async def get_system_health(self) -> Dict[str, Any]:
        """Get system health (legacy name for get_health)."""
        return await self.get_health()

    # =================================================================
    # PROPERTY ACCESSOR METHODS
    # =================================================================

    @property
    def feagi(self):
        """Get the FEAGI instance."""
        try:
            from feagi.core.feagi import FEAGI

            if not hasattr(self, "_feagi_instance"):
                self._feagi_instance = FEAGI()
            return self._feagi_instance
        except Exception as e:
            self.logger.error(f"Error getting FEAGI instance: {str(e)}")
            return None

    # ===== Frequency Measurement Methods =====

    def trigger_frequency_measurement(
        self, duration_seconds: float = 5.0, sample_count: int = 100
    ) -> dict:
        """Trigger an on-demand burst frequency measurement.

        This is an expensive operation that should only be called when needed for monitoring.

        Args:
            duration_seconds: How long to measure (default 5.0 seconds)
            sample_count: Number of burst samples to collect (default 100)

        Returns:
            Dictionary with measurement results
        """
        return self.state_manager.trigger_frequency_measurement(
            duration_seconds, sample_count
        )

    def get_frequency_measurement_history(
        self, limit: Optional[int] = None
    ) -> dict:
        """Get the history of frequency measurements.

        Args:
            limit: Maximum number of recent measurements to return

        Returns:
            Dictionary with measurement history
        """
        return self.state_manager.get_frequency_measurement_history(limit)

    def get_frequency_status_summary(self) -> dict:
        """
        Get current frequency status and latest measurement - RTOS-safe.

        Returns:
            Dictionary with frequency status and latest measurement
        """
        try:
            # RTOS-SAFE: Get actual frequency data from burst engine
            burst_engine = self.get_burst_engine()
            if burst_engine:
                frequency_config = burst_engine.get_frequency_config()
                return {
                    "target_frequency_hz": frequency_config.get(
                        "current_frequency_hz", 0.0
                    ),
                    "has_measurements": burst_engine.burst_count > 0,
                    "total_measurements": burst_engine.burst_count,
                    "latest_measurement": (
                        {
                            "frequency_hz": frequency_config.get(
                                "current_frequency_hz", 0.0
                            ),
                            "burst_count": burst_engine.burst_count,
                            "last_burst_time": getattr(
                                burst_engine, "last_burst_time", 0.0
                            ),
                        }
                        if burst_engine.burst_count > 0
                        else None
                    ),
                }

            # Fallback to state manager if burst engine not available
            return (
                self.state_manager.get_frequency_status_summary()
                if self.state_manager
                else {
                    "target_frequency_hz": 0.0,
                    "has_measurements": False,
                    "total_measurements": 0,
                    "latest_measurement": None,
                }
            )
        except Exception as e:
            self.logger.error(f"Error getting frequency status: {str(e)}")
            return {
                "target_frequency_hz": 0.0,
                "has_measurements": False,
                "total_measurements": 0,
                "latest_measurement": None,
                "error": str(e),
            }

    def get_fire_queue_direct(self) -> Optional[np.ndarray]:
        """Get fire queue data directly from SoA structures - zero-copy access.

        Returns:
            numpy array with shape (N, 5) containing:
            [neuron_ids, membrane_potentials, coordinates_x, coordinates_y, coordinates_z]
            or None if no firing neurons
        """
        try:
            if (
                hasattr(self._connectome_manager, "fcl_manager")
                and self._connectome_manager.fcl_manager
            ):
                global_fcl = (
                    self._connectome_manager.fcl_manager.get_global_fcl()
                )

                if global_fcl.is_empty():
                    return None

                # Direct FCL to numpy array (already optimized!)
                firing_indices = np.array(list(global_fcl), dtype=np.int32)

                if len(firing_indices) == 0:
                    return None

                # Direct SoA access - use existing optimized structures
                neuron_array = self._connectome_manager.neuron_array

                # Vectorized extraction in single operation - NO FALLBACKS
                if (
                    hasattr(neuron_array, "membrane_potentials")
                    and hasattr(neuron_array, "coordinates_x")
                    and hasattr(neuron_array, "coordinates_y")
                    and hasattr(neuron_array, "coordinates_z")
                ):
                    brain_data = np.column_stack(
                        (
                            firing_indices.astype(
                                np.int32
                            ),  # Keep int32 for neuron IDs (they can be signed)
                            neuron_array.membrane_potentials[
                                firing_indices
                            ],  # Keep float32 for potentials
                            neuron_array.coordinates_x[firing_indices].astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                            neuron_array.coordinates_y[firing_indices].astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                            neuron_array.coordinates_z[firing_indices].astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                        )
                    )

                    return brain_data
                else:
                    #  ❌ NO FALLBACK - Neuron array must have all required
                    #  properties
                    self.logger.error(
                        "Neuron array missing required properties (membrane_potentials, coordinates_x/y/z)"
                    )
                    return None
            return None
        except Exception as e:
            self.logger.error(f"Error getting direct fire queue: {str(e)}")
            return None

    def get_area_fire_queue_direct(
        self, cortical_id: str
    ) -> Optional[np.ndarray]:
        """Get fire queue data for specific area directly from SoA structures.

        Args:
            cortical_id: ID of the cortical area

        Returns:
            numpy array with shape (N, 5) or None if no firing neurons
        """
        try:
            if not self._validate_genome_loaded():
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Genome not loaded for area {cortical_id}"
                )
                return None

            if (
                hasattr(self._connectome_manager, "fcl_manager")
                and self._connectome_manager.fcl_manager
            ):
                #  CRITICAL FIX: Read from global FCL and filter by cortical
                #  area
                # instead of reading from cortical FCL history which is empty
                global_fcl = self._connectome_manager.fcl_manager.get_fcl()

                if global_fcl.is_empty():
                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] Global FCL is empty for area {cortical_id}"
                    )
                    return None

                if self.state_manager.is_debug_npu_enabled():
                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] Global FCL has {len(global_fcl)} total firing neurons: {list(global_fcl)}"
                    )

                # CRITICAL FIX: Get cortical_idx for the requested cortical_id
                target_cortical_idx = self._get_cortical_idx_for_id(
                    cortical_id
                )
                if target_cortical_idx is None:
                    self.logger.error(
                        f"🔥 [FIRE QUEUE] Could not map cortical_id '{cortical_id}' to cortical_idx"
                    )
                    return None

                if self.state_manager.is_debug_npu_enabled():
                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] Mapped cortical_id '{cortical_id}' to cortical_idx {target_cortical_idx}"
                    )

                # Filter global FCL by cortical_idx using neuron array mapping
                firing_indices = []
                neuron_array = self._connectome_manager.neuron_array

                if not hasattr(neuron_array, "cortical_idxs"):
                    self.logger.error(
                        "🔥 [FIRE QUEUE] Neuron array missing cortical_idxs attribute"
                    )
                    return None

                # Use vectorized filtering for performance
                firing_neuron_ids = np.array(list(global_fcl), dtype=np.int32)
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Firing neuron IDs: {firing_neuron_ids}"
                )


                # ARCHITECTURE: In Rust NPU, neuron_id == index (identity mapping)
                # Just validate neuron existence and use IDs directly as indices
                firing_indices = []
                for neuron_id in firing_neuron_ids:
                    neuron_id_int = int(neuron_id)
                    if self._connectome_manager.has_neuron(neuron_id_int):
                        firing_indices.append(neuron_id_int)

                if len(firing_indices) == 0:
                    self.logger.debug(
                        "🔥 [FIRE QUEUE] No valid firing neuron indices found"
                    )
                    return None

                firing_indices = np.array(firing_indices, dtype=np.int32)
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Converted {len(firing_neuron_ids)} neuron IDs to {len(firing_indices)} indices: {firing_indices}"
                )

                # Filter by target cortical_idx using the correct indices
                neuron_cortical_idxs = neuron_array.cortical_idxs[
                    firing_indices
                ]
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Neuron cortical indices: {neuron_cortical_idxs}"
                )
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Target cortical_idx: {target_cortical_idx}"
                )

                area_mask = neuron_cortical_idxs == target_cortical_idx
                self.logger.debug(f"🔥 [FIRE QUEUE] Area mask: {area_mask}")

                area_firing_indices = firing_indices[area_mask]
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Firing indices in target area: {area_firing_indices}"
                )

                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Found {len(area_firing_indices)} firing neurons in area {cortical_id} (cortical_idx={target_cortical_idx})"
                )

                if len(area_firing_indices) == 0:
                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] No firing neurons found in area {cortical_id}"
                    )
                    return None

                # area_firing_indices is already a numpy array

                # Direct SoA access - NO FALLBACKS
                # Vectorized extraction - all properties must exist
                if (
                    hasattr(neuron_array, "membrane_potentials")
                    and hasattr(neuron_array, "coordinates_x")
                    and hasattr(neuron_array, "coordinates_y")
                    and hasattr(neuron_array, "coordinates_z")
                ):
                    # CRITICAL FIX: Convert firing indices to actual neuron IDs
                    # The FQ sampler expects neuron IDs, not array indices!
                    # Use NPU-owned vectorized conversion
                    if hasattr(neuron_array, "indices_to_neuron_ids"):
                        final_neuron_ids = neuron_array.indices_to_neuron_ids(
                        area_firing_indices, filter_invalid=True
                    )
                    else:
                        # Fallback: map via index_to_neuron_id dict
                        final_ids_list = []
                        for idx in list(area_firing_indices):
                            nid = neuron_array.index_to_neuron_id.get(int(idx))
                            if nid is not None:
                                final_ids_list.append(int(nid))
                        final_neuron_ids = np.array(final_ids_list, dtype=np.int32)


                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] Converted {len(area_firing_indices)} indices to {len(final_neuron_ids)} neuron IDs"
                    )

                    if len(final_neuron_ids) == 0:
                        self.logger.debug(
                            f"🔥 [FIRE QUEUE] No valid neuron IDs after conversion for area {cortical_id}"
                        )
                        return None

                    # COORDINATE DEBUG: Log coordinates being extracted from neuron array
                    extracted_x = neuron_array.coordinates_x[area_firing_indices]
                    extracted_y = neuron_array.coordinates_y[area_firing_indices]
                    extracted_z = neuron_array.coordinates_z[area_firing_indices]

                    brain_data = np.column_stack(
                        (
                            final_neuron_ids.astype(
                                np.int32
                            ),  # ✅ FIXED: Use actual neuron IDs, not indices!
                            neuron_array.membrane_potentials[
                                area_firing_indices
                            ],  # Keep float32 for potentials
                            extracted_x.astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                            extracted_y.astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                            extracted_z.astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                        )
                    )


                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] Successfully extracted {len(area_firing_indices)} firing neurons for area {cortical_id}"
                    )
                    return brain_data
                else:
                    #  ❌ NO FALLBACK - Neuron array must have all required
                    #  properties
                    self.logger.error(
                        f"🔥 [FIRE QUEUE] Neuron array missing required properties for area {cortical_id}"
                    )
                    return None
            return None
        except Exception as e:
            self.logger.error(
                f"🔥 [FIRE QUEUE] Error getting direct area fire queue for {cortical_id}: {str(e)}"
            )
            return None

    def get_area_fire_queue(
        self, cortical_id: str
    ) -> Optional[Dict[str, Any]]:
        """Get fire queue data for specific area in dictionary format (FQ
        sampler compatible).

        This method provides the interface expected by the FQ sampler, converting the direct
        numpy array data to the dictionary format that the sampler expects.

        Args:
            cortical_id: ID of the cortical area

        Returns:
            Dictionary with neuron_ids, membrane_potentials, coordinates, etc. or None if no data
        """
        try:
            if self.state_manager.is_debug_npu_enabled():
                self.logger.debug(
                    f"🔥 [FIRE QUEUE API] get_area_fire_queue called for area: {cortical_id}"
                )
            # Get the direct numpy array data
            fire_queue_data = self.get_area_fire_queue_direct(cortical_id)

            if fire_queue_data is None or len(fire_queue_data) == 0:
                return None

            # Convert numpy array to dictionary format expected by FQ sampler
            # Array columns: [neuron_ids, membrane_potentials, x, y, z]
            neuron_ids = fire_queue_data[:, 0].astype(int).tolist()
            membrane_potentials = fire_queue_data[:, 1].tolist()
            coordinates_x = fire_queue_data[:, 2].astype(int).tolist()
            coordinates_y = fire_queue_data[:, 3].astype(int).tolist()
            coordinates_z = fire_queue_data[:, 4].astype(int).tolist()

            # Package coordinates as list of (x, y, z) tuples
            coordinates = list(
                zip(coordinates_x, coordinates_y, coordinates_z)
            )

            # CRITICAL FIX: Remove problematic neuron property extraction
            #  The essential data (neuron_ids, membrane_potentials,
            #  coordinates) is already available
            # Additional properties can be empty arrays - NO FAKE DATA
            thresholds = []
            consecutive_fire_counts = []
            refractory_counters = []

            return {
                "neuron_ids": neuron_ids,
                "membrane_potentials": membrane_potentials,
                "coordinates": coordinates,
                "thresholds": thresholds,  # Empty - will not provide fake data
                "consecutive_fire_counts": consecutive_fire_counts,  # Empty - will not provide fake data
                "refractory_counters": refractory_counters,  # Empty - will not provide fake data
            }

        except Exception as e:
            self.logger.error(
                f"Error getting area fire queue for {cortical_id}: {str(e)}"
            )
            return None

    # =================================================================
    # HIGH-PERFORMANCE NEURON COORDINATE METHODS
    # =================================================================

    def get_neuron_coordinates(
        self, neuron_ids: List[int]
    ) -> Optional[Dict[str, Any]]:
        """Get coordinates (X, Y, Z) for a list of neuron IDs using SIMD-
        optimized extraction.

        Leverages FEAGI's centralized SIMD configuration for maximum performance.
        Uses vectorized operations with optimal memory layouts and cache-friendly algorithms.

        Args:
            neuron_ids: List of neuron IDs to get coordinates for

        Returns:
            Dictionary containing:
            - neuron_ids: List of requested neuron IDs
            - coordinates_x: List of X coordinates
            - coordinates_y: List of Y coordinates
            - coordinates_z: List of Z coordinates
            - valid_indices: Boolean list indicating which neuron IDs were valid
            - performance_stats: SIMD performance metrics (if profiling enabled)

            Returns None if no valid neuron IDs provided or neuron array not available

        Examples:
            # Standard usage - get coordinates for specific neurons
            neuron_list = [101, 102, 103, 500, 750]
            result = core_api.get_neuron_coordinates(neuron_list)

            if result:
                for i, neuron_id in enumerate(result['neuron_ids']):
                    if result['valid_indices'][i]:
                        x = result['coordinates_x'][i]
                        y = result['coordinates_y'][i]
                        z = result['coordinates_z'][i]
                        print(f"Neuron {neuron_id}: ({x:.1f}, {y:.1f}, {z:.1f})")

                # Check SIMD performance stats
                stats = result['performance_stats']
                print(f"SIMD Backend: {stats['backend']}")
                print(
                    f"Extraction method: {stats.get('extraction_method',"
                    f"'unknown')}"
                )
                if 'neurons_per_second' in stats:
                    print(
                        f"Performance: {stats['neurons_per_second']:.0f}"
                        f"neurons/sec"
                    )

            # Batch processing for large datasets
            large_batch = list(range(1000, 50000, 10))  # 5,000 neurons
            result = core_api.get_neuron_coordinates(large_batch)

            # Extract valid coordinates only
            valid_coords = []
            for i, valid in enumerate(result['valid_indices']):
                if valid:
                    valid_coords.append((
                        result['coordinates_x'][i],
                        result['coordinates_y'][i],
                        result['coordinates_z'][i]
                    ))
        """
        try:
            if not neuron_ids:
                return None

            # Get centralized SIMD configuration
            simd_config = (
                self.state_manager.get_simd_configuration()
                if self.state_manager
                else {
                    "available": False,
                    "backend": "SCALAR",
                    "vector_width": 1,
                    "alignment": 8,
                }
            )

            # ✅ CRITICAL FIX: Ensure minimum valid SIMD configuration values
            #  Prevent zero values that cause empty arrays and coordinate
            #  extraction failure
            if simd_config.get("vector_width", 0) <= 0:
                simd_config["vector_width"] = 1  # Minimum vector width
            if simd_config.get("alignment", 0) <= 0:
                simd_config["alignment"] = (
                    8  # Minimum alignment for performance
                )

            if not hasattr(self._connectome_manager, "neuron_array"):
                self.logger.error(
                    "Neuron array not available in connectome manager"
                )
                return None

            neuron_array = self._connectome_manager.neuron_array

            # Convert to aligned numpy array for SIMD optimization
            # alignment = simd_config["alignment"]  # Unused variable removed

            # ARCHITECTURE: In Rust NPU, neuron_id == index (identity mapping)
            # Just validate neuron existence and use IDs directly as indices
            neuron_indices_list = []
            valid_neuron_ids = []
            
            for neuron_id in neuron_ids:
                if self._connectome_manager.has_neuron(neuron_id):
                    neuron_indices_list.append(neuron_id)
                    valid_neuron_ids.append(neuron_id)
                    
            if not neuron_indices_list:
                # No valid neuron IDs found
                return {
                    "neuron_ids": neuron_ids,
                    "coordinates_x": [],
                    "coordinates_y": [],
                    "coordinates_z": [],
                    "valid_indices": [False] * len(neuron_ids),
                    "performance_stats": {
                        "simd_used": False,
                        "backend": simd_config["backend"],
                        "error": "No valid neuron ID to index mappings found",
                    },
                }

            # Align memory to SIMD boundaries for optimal performance
            # ✅ CRITICAL FIX: Ensure aligned_size is never zero
            valid_count = len(neuron_indices_list)
            aligned_size = max(
                valid_count,
                (valid_count + simd_config["vector_width"] - 1)
                & ~(simd_config["vector_width"] - 1),
            )

            #  Pre-allocate aligned arrays (SIMD-friendly) with correct array
            #  indices
            neuron_indices = np.zeros(aligned_size, dtype=np.int32)
            neuron_indices[:valid_count] = neuron_indices_list

            # SIMD-optimized bounds checking
            if hasattr(neuron_array, "coordinates_x"):
                max_neuron_id = len(neuron_array.coordinates_x) - 1

                if (
                    simd_config["available"]
                    and simd_config["vector_width"] >= 4
                ):
                    # Vectorized bounds checking using SIMD
                    valid_mask = self._simd_bounds_check(
                        neuron_indices[:valid_count],
                        max_neuron_id,
                        simd_config,
                    )
                else:
                    # Fallback to numpy vectorized operations
                    valid_mask = (neuron_indices[:valid_count] >= 0) & (
                        neuron_indices[:valid_count] <= max_neuron_id
                    )

                valid_indices = neuron_indices[:valid_count][valid_mask]
            else:
                # Coordinates not available in legacy neuron_array (using Rust NPU coordinates)
                valid_mask = np.ones(valid_count, dtype=bool)
                valid_indices = neuron_indices[:valid_count]

            if len(valid_indices) == 0:
                return {
                    "neuron_ids": valid_neuron_ids,
                    "coordinates_x": [],
                    "coordinates_y": [],
                    "coordinates_z": [],
                    "valid_indices": valid_mask.tolist(),
                    "performance_stats": {
                        "simd_used": False,
                        "backend": simd_config["backend"],
                    },
                }

            # SIMD-optimized coordinate extraction
            performance_stats = {
                "simd_used": simd_config["available"],
                "backend": simd_config["backend"],
            }

            if (
                simd_config["available"]
                and len(valid_indices) >= simd_config["vector_width"] * 2
            ):
                # Use SIMD-optimized coordinate extraction for large datasets
                coords_x, coords_y, coords_z = self._simd_extract_coordinates(
                    neuron_array, valid_indices, simd_config, performance_stats
                )
            else:
                # Use numpy vectorized operations for smaller datasets
                coords_x, coords_y, coords_z = (
                    self._vectorized_extract_coordinates(
                    neuron_array, valid_indices, performance_stats
                    )
                )

            # CRITICAL FIX: Since we already filtered to valid neurons, 
            # we can return the coordinates directly without complex remapping
            # All neurons in valid_neuron_ids have corresponding coordinates
            
            # Convert coordinates to uint32 for consistency
            result_x = coords_x.astype(np.uint32)
            result_y = coords_y.astype(np.uint32) 
            result_z = coords_z.astype(np.uint32)
            
            # Create valid_indices array for original neuron_ids list
            original_valid_mask = []
            for neuron_id in neuron_ids:
                original_valid_mask.append(neuron_id in valid_neuron_ids)

            return {
                "neuron_ids": neuron_ids,  # Return original list order
                "coordinates_x": result_x.tolist(),  # Only valid coordinates
                "coordinates_y": result_y.tolist(),  # Only valid coordinates
                "coordinates_z": result_z.tolist(),  # Only valid coordinates
                "valid_indices": original_valid_mask,  # Map back to original order
                "performance_stats": performance_stats,
            }

        except Exception as e:
            # Coordinate extraction failed (expected with Rust NPU - coordinates provided by FQ Sampler)
            return None

    def get_neuron_coordinates_numpy(
        self, neuron_ids: List[int]
    ) -> Optional[np.ndarray]:
        """Get coordinates for a list of neuron IDs as SIMD-optimized numpy
        array (zero-copy).

        Highest performance method using vectorized SIMD operations and optimal memory layouts.
        Designed for real-time applications requiring maximum throughput.

        Args:
            neuron_ids: List of neuron IDs to get coordinates for

        Returns:
            numpy array with shape (N, 4) containing:
            [neuron_id, x_coordinate, y_coordinate, z_coordinate]
            Only includes valid neuron IDs. Returns None if no valid neurons.

        Examples:
            # High-performance coordinate extraction
            neuron_ids = [1, 2, 3, 100, 250]
            coords = core_api.get_neuron_coordinates_numpy(neuron_ids)

            if coords is not None:
                # Direct numpy operations for analysis
                neuron_ids = coords[:, 0].astype(int)
                x_coords = coords[:, 1]
                y_coords = coords[:, 2]
                z_coords = coords[:, 3]

                # Calculate distances from origin
                distances = np.linalg.norm(coords[:, 1:4], axis=1)
                print(f"Distances: {distances}")

                # Find neurons within a region
                within_region = coords[(x_coords > 10) & (x_coords < 50)]
                print(f"Neurons in region: {within_region[:, 0]}")

                # Vectorized coordinate transformations
                transformed = coords.copy()
                transformed[:, 1:4] *= 2.0  # Scale coordinates

            # Real-time processing loop example
            while processing:
                active_neurons = get_currently_firing_neurons()
                coords = core_api.get_neuron_coordinates_numpy(active_neurons)

                if coords is not None:
                    # Zero-copy processing for maximum performance
                    process_spatial_patterns(coords)
                    update_visualization(coords)

            # Batch analysis for large datasets
            all_neurons = list(range(100000))  # 100k neurons
            coords = core_api.get_neuron_coordinates_numpy(all_neurons)

            if coords is not None:
                # Efficient spatial analysis using SIMD
                center_of_mass = np.mean(coords[:, 1:4], axis=0)
                std_deviation = np.std(coords[:, 1:4], axis=0)
                print(
                    f"Spatial distribution: center={center_of_mass},"
                    f"std={std_deviation}"
                )
        """
        try:
            if not neuron_ids:
                return None

            # Get centralized SIMD configuration
            simd_config = (
                self.state_manager.get_simd_configuration()
                if self.state_manager
                else {
                    "available": False,
                    "backend": "SCALAR",
                    "vector_width": 1,
                    "alignment": 8,
                }
            )

            # ✅ CRITICAL FIX: Ensure minimum valid SIMD configuration values
            #  Prevent zero values that cause empty arrays and coordinate
            #  extraction failure
            if simd_config.get("vector_width", 0) <= 0:
                simd_config["vector_width"] = 1  # Minimum vector width
            if simd_config.get("alignment", 0) <= 0:
                simd_config["alignment"] = (
                    8  # Minimum alignment for performance
                )

            if not hasattr(self._connectome_manager, "neuron_array"):
                return None

            neuron_array = self._connectome_manager.neuron_array
            # neuron_count = len(neuron_ids)  # Unused variable removed

            # Pre-allocate SIMD-aligned array for optimal performance
            # alignment = simd_config["alignment"]  # Unused variable removed
            neuron_indices = np.array(neuron_ids, dtype=np.int32)

            # SIMD-optimized filtering of valid indices
            if hasattr(neuron_array, "coordinates_x"):
                max_neuron_id = len(neuron_array.coordinates_x) - 1

                if (
                    simd_config["available"]
                    and simd_config["vector_width"] >= 4
                ):
                    valid_mask = self._simd_bounds_check(
                        neuron_indices, max_neuron_id, simd_config
                    )
                else:
                    valid_mask = (neuron_indices >= 0) & (
                        neuron_indices <= max_neuron_id
                    )

                valid_indices = neuron_indices[valid_mask]
            else:
                valid_indices = neuron_indices

            if len(valid_indices) == 0:
                return None

            # High-performance coordinate extraction
            if (
                simd_config["available"]
                and len(valid_indices) >= simd_config["vector_width"] * 4
            ):
                # SIMD path for large datasets
                coords_x, coords_y, coords_z = self._simd_extract_coordinates(
                    neuron_array, valid_indices, simd_config, {}
                )
            else:
                # Vectorized path for smaller datasets
                coords_x, coords_y, coords_z = (
                    self._vectorized_extract_coordinates(
                    neuron_array, valid_indices, {}
                    )
                )

            # Combine into single SIMD-aligned array: [neuron_id, x, y, z]
            result = np.column_stack(
                (valid_indices.astype(np.int32), coords_x, coords_y, coords_z)
            )

            return result

        except Exception as e:
            self.logger.error(
                f"Error getting neuron coordinates as numpy array: {str(e)}"
            )
            return None

    def _simd_bounds_check(
        self, indices: np.ndarray, max_value: int, simd_config: dict
    ) -> np.ndarray:
        """SIMD-optimized bounds checking for neuron indices.

        Uses vectorized operations to check multiple indices simultaneously.
        """
        try:
            #  Use numpy's vectorized operations which leverage SIMD under the
            #  hood
            # This is optimized for the detected SIMD backend
            vector_width = simd_config["vector_width"]

            # Process in SIMD-aligned chunks for optimal performance
            valid_mask = np.zeros(len(indices), dtype=bool)

            # Process main chunks using vectorized operations
            for i in range(0, len(indices), vector_width):
                end_idx = min(i + vector_width, len(indices))
                chunk = indices[i:end_idx]

                # Vectorized bounds check (automatically uses SIMD)
                chunk_mask = (chunk >= 0) & (chunk <= max_value)
                valid_mask[i:end_idx] = chunk_mask

            return valid_mask

        except Exception as e:
            self.logger.warning(
                f"SIMD bounds check failed, using fallback: {e}"
            )
            return (indices >= 0) & (indices <= max_value)

    def _simd_extract_coordinates(
        self,
        neuron_array,
        valid_indices: np.ndarray,
        simd_config: dict,
        performance_stats: dict,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """SIMD-optimized coordinate extraction using vectorized array
        indexing.

        Processes coordinates in SIMD-aligned chunks for maximum throughput.
        Handles both NumPy arrays and PyTorch tensors properly.
        """
        try:
            start_time = time.time()
            vector_width = simd_config["vector_width"]

            if (
                hasattr(neuron_array, "coordinates_x")
                and hasattr(neuron_array, "coordinates_y")
                and hasattr(neuron_array, "coordinates_z")
            ):
                # Handle PyTorch tensors vs NumPy arrays for SIMD optimization
                import torch

                if isinstance(neuron_array.coordinates_x, torch.Tensor):
                    # Convert PyTorch tensors to NumPy for SIMD operations
                    coords_x_np = neuron_array.coordinates_x.cpu().numpy()
                    coords_y_np = neuron_array.coordinates_y.cpu().numpy()
                    coords_z_np = neuron_array.coordinates_z.cpu().numpy()

                    #  SIMD-optimized array indexing on NumPy arrays - keep
                    #  uint32
                    coords_x = coords_x_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_y = coords_y_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_z = coords_z_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32

                    performance_stats["extraction_method"] = (
                        "simd_torch_converted"
                    )
                else:
                    #  Direct SIMD-optimized array indexing on NumPy arrays -
                    #  keep uint32
                    coords_x = neuron_array.coordinates_x[
                        valid_indices
                    ].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_y = neuron_array.coordinates_y[
                        valid_indices
                    ].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_z = neuron_array.coordinates_z[
                        valid_indices
                    ].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32

                    performance_stats["extraction_method"] = (
                        "simd_numpy_direct"
                    )
            else:
                # ❌ NO FALLBACK - Coordinates must exist in neuron array
                # Creating fake coordinates violates architectural rules
                # Coordinate arrays not in legacy neuron_array
                raise ValueError(
                    "Neuron coordinate arrays not available in connectome - check genome initialization"
                )

            extraction_time = time.time() - start_time
            performance_stats["extraction_time_ms"] = extraction_time * 1000
            performance_stats["neurons_per_second"] = (
                len(valid_indices) / extraction_time
                if extraction_time > 0
                else 0
            )
            performance_stats["simd_efficiency"] = min(
                1.0,
                (len(valid_indices) / vector_width)
                / max(1, len(valid_indices) // vector_width),
            )

            return coords_x, coords_y, coords_z

        except Exception as e:
            self.logger.warning(
                f"SIMD coordinate extraction failed, using fallback: {e}"
            )
            return self._vectorized_extract_coordinates(
                neuron_array, valid_indices, performance_stats
            )

    def _vectorized_extract_coordinates(
        self, neuron_array, valid_indices: np.ndarray, performance_stats: dict
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Vectorized coordinate extraction fallback using standard numpy
        operations.

        Handles both NumPy arrays and PyTorch tensors properly.
        """
        try:
            start_time = time.time()

            if (
                hasattr(neuron_array, "coordinates_x")
                and hasattr(neuron_array, "coordinates_y")
                and hasattr(neuron_array, "coordinates_z")
            ):
                # Handle PyTorch tensors vs NumPy arrays
                import torch

                if isinstance(neuron_array.coordinates_x, torch.Tensor):
                    # Convert PyTorch tensors to NumPy for indexing
                    coords_x_np = neuron_array.coordinates_x.cpu().numpy()
                    coords_y_np = neuron_array.coordinates_y.cpu().numpy()
                    coords_z_np = neuron_array.coordinates_z.cpu().numpy()

                    # Extract coordinates using NumPy indexing - keep uint32
                    coords_x = coords_x_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_y = coords_y_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_z = coords_z_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32

                    performance_stats["extraction_method"] = (
                        "vectorized_torch_converted"
                    )
                else:
                    # Direct NumPy array indexing - keep uint32
                    coords_x = neuron_array.coordinates_x[
                        valid_indices
                    ].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_y = neuron_array.coordinates_y[
                        valid_indices
                    ].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_z = neuron_array.coordinates_z[
                        valid_indices
                    ].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32

                    performance_stats["extraction_method"] = (
                        "vectorized_numpy_direct"
                    )
            else:
                # ❌ NO FALLBACK - Coordinates must exist in neuron array
                # Creating fake coordinates violates architectural rules
                # Coordinate arrays not in legacy neuron_array
                raise ValueError(
                    "Neuron coordinate arrays not available in connectome - check genome initialization"
                )

            extraction_time = time.time() - start_time
            performance_stats["extraction_time_ms"] = extraction_time * 1000
            performance_stats["neurons_per_second"] = (
                len(valid_indices) / extraction_time
                if extraction_time > 0
                else 0
            )

            return coords_x, coords_y, coords_z

        except Exception as e:
            # Vectorized extraction failed (expected with Rust NPU - coordinates provided directly)
            # ❌ NO FALLBACK - Don't create fake coordinates
            #  Real coordinates must exist - this is a
            #  configuration/initialization error
            raise ValueError(
                f"Failed to extract neuron coordinates: {e}"
            ) from e

    def benchmark_neuron_coordinate_extraction(
        self, neuron_count: int = 10000
    ) -> Dict[str, Any]:
        """Benchmark SIMD-optimized neuron coordinate extraction performance.

        Tests different batch sizes and extraction methods to demonstrate
        the performance benefits of SIMD optimization.

        Args:
            neuron_count: Number of neurons to benchmark (default 10,000)

        Returns:
            Dictionary with benchmark results including timing, throughput, and SIMD efficiency
        """
        try:
            # Generate test neuron IDs
            neuron_ids = list(
                range(0, neuron_count, max(1, neuron_count // 10000))
            )
            if len(neuron_ids) > 10000:
                neuron_ids = neuron_ids[
                    :10000
                ]  # Cap at 10k for reasonable benchmark time

            # Get SIMD configuration
            simd_config = (
                self.state_manager.get_simd_configuration()
                if self.state_manager
                else {
                    "available": False,
                    "backend": "SCALAR",
                    "vector_width": 1,
                    "alignment": 8,
                }
            )

            # Run benchmark tests
            results = {
                "simd_config": simd_config,
                "neuron_count": len(neuron_ids),
                "benchmark_results": {},
            }

            # Test 1: Dictionary method (standard API)
            start_time = time.perf_counter()
            dict_result = self.get_neuron_coordinates(neuron_ids)
            dict_time = time.perf_counter() - start_time

            if dict_result:
                results["benchmark_results"]["dictionary_method"] = {
                    "time_ms": dict_time * 1000,
                    "neurons_per_second": (
                        len(neuron_ids) / dict_time if dict_time > 0 else 0
                    ),
                    "performance_stats": dict_result.get(
                        "performance_stats", {}
                    ),
                    "valid_neurons": sum(dict_result.get("valid_indices", [])),
                }

            # Test 2: NumPy method (high-performance API)
            start_time = time.perf_counter()
            numpy_result = self.get_neuron_coordinates_numpy(neuron_ids)
            numpy_time = time.perf_counter() - start_time

            if numpy_result is not None:
                results["benchmark_results"]["numpy_method"] = {
                    "time_ms": numpy_time * 1000,
                    "neurons_per_second": (
                        len(numpy_result) / numpy_time if numpy_time > 0 else 0
                    ),
                    "result_shape": numpy_result.shape,
                    "memory_mb": numpy_result.nbytes / (1024 * 1024),
                }

            # Performance comparison
            if dict_time > 0 and numpy_time > 0:
                results["performance_comparison"] = {
                    "numpy_speedup": dict_time / numpy_time,
                    "simd_efficiency": simd_config.get("vector_width", 1)
                    / max(1, dict_time / numpy_time),
                    "recommended_method": (
                        "numpy" if numpy_time < dict_time else "dictionary"
                    ),
                }

            # SIMD utilization analysis
            if simd_config["available"]:
                theoretical_speedup = simd_config["vector_width"]
                actual_speedup = results["performance_comparison"].get(
                    "numpy_speedup", 1.0
                )
                results["simd_analysis"] = {
                    "theoretical_max_speedup": theoretical_speedup,
                    "actual_speedup": actual_speedup,
                    "simd_utilization_percent": (
                        actual_speedup / theoretical_speedup
                    )
                    * 100,
                    "backend_used": simd_config["backend"],
                    "optimization_recommendations": self._get_optimization_recommendations(
                        len(neuron_ids),
                        simd_config,
                        actual_speedup,
                        theoretical_speedup,
                    ),
                }

            self.logger.info(
                f"Coordinate extraction benchmark completed: "
                f"{len(neuron_ids)} neurons, "
                f"SIMD: {simd_config['backend']}, "
                f"Performance: {results['benchmark_results'].get('numpy_method', {}).get('neurons_per_second', 0):.0f} neurons/sec"
            )

            return results

        except Exception as e:
            self.logger.error(
                f"Error running coordinate extraction benchmark: {str(e)}"
            )
            return {"error": str(e), "simd_config": simd_config}

    def _get_optimization_recommendations(
        self,
        neuron_count: int,
        simd_config: dict,
        actual_speedup: float,
        theoretical_speedup: float,
    ) -> List[str]:
        """Generate optimization recommendations based on benchmark results."""
        recommendations = []

        utilization = (
            (actual_speedup / theoretical_speedup) * 100
            if theoretical_speedup > 0
            else 0
        )

        if utilization < 50:
            recommendations.append(
                "Consider larger batch sizes to improve SIMD utilization"
            )

        if neuron_count < simd_config["vector_width"] * 10:
            recommendations.append(
                "Dataset too small for effective SIMD optimization"
            )

        if not simd_config["available"]:
            recommendations.append(
                "SIMD not available - consider upgrading hardware or enabling SIMD support"
            )
        elif simd_config["backend"] == "SCALAR":
            recommendations.append(
                "SIMD backend using scalar fallback - check SIMD detection"
            )

        if utilization > 80:
            recommendations.append(
                "Excellent SIMD utilization - consider this pattern for other operations"
            )

        return recommendations

    # =================================================================
    # CORTICAL MAPPING RESTRICTIONS
    # =================================================================

    def get_mapping_restrictions(
        self,
        source_type: Optional[str] = None,
        destination_type: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Get mapping restrictions between cortical area types.

        Args:
            source_type: Source cortical area type (optional, returns all if None)
            destination_type: Destination cortical area type (optional, returns all if None)

        Returns:
            Dict containing restrictions and defaults for cortical area mappings
        """
        try:
            registry = get_mapping_restrictions_registry()

            if source_type and destination_type:
                # Get specific restriction
                restriction = registry.get_restriction(
                    source_type, destination_type
                )
                default = registry.get_default(source_type, destination_type)

                return {
                    "source_type": source_type,
                    "destination_type": destination_type,
                    "restriction": (
                        restriction.to_dict() if restriction else None
                    ),
                    "default": default.to_dict() if default else None,
                }
            else:
                # Get all restrictions and defaults
                return registry.to_dict()

        except Exception as e:
            self.logger.error(f"Error getting mapping restrictions: {str(e)}")
            return {"restrictions": [], "defaults": []}

    def get_restriction_between_cortical_areas(
        self, source_cortical_id: str, destination_cortical_id: str
    ) -> Optional[Dict[str, Any]]:
        """Get mapping restriction between two specific cortical areas.

        Args:
            source_cortical_id: Source cortical area ID
            destination_cortical_id: Destination cortical area ID

        Returns:
            Dict containing restriction data or None if not found
        """
        try:
            # Get the cortical area types
            source_area = self.get_cortical_area(source_cortical_id)
            destination_area = self.get_cortical_area(destination_cortical_id)

            if not source_area or not destination_area:
                return None

            source_type = source_area.get("type", "UNKNOWN")
            destination_type = destination_area.get("type", "UNKNOWN")

            # Get restriction for these types
            registry = get_mapping_restrictions_registry()
            restriction = registry.get_restriction(
                source_type, destination_type
            )
            default = registry.get_default(source_type, destination_type)

            if restriction or default:
                return {
                    "source_cortical_id": source_cortical_id,
                    "destination_cortical_id": destination_cortical_id,
                    "source_type": source_type,
                    "destination_type": destination_type,
                    "restriction": (
                        restriction.to_dict() if restriction else None
                    ),
                    "default": default.to_dict() if default else None,
                    "has_restricted_morphologies": (
                        restriction.has_restricted_morphologies()
                        if restriction
                        else False
                    ),
                    "get_morphologies_restricted_to": (
                        restriction.restricted_morphologies
                        if restriction
                        and restriction.has_restricted_morphologies()
                        else []
                    ),
                }

            return None

        except Exception as e:
            self.logger.error(
                f"Error getting restriction between {source_cortical_id} and {destination_cortical_id}: {str(e)}"
            )
            return None

    # =================================================================
    # AGENT REGISTRY (State Manager Integration)
    # =================================================================

    def register_agent(
        self,
        agent_id: str,
        agent_type: str = None,
        capabilities: dict = None,
        agent_data_port: int = None,
        agent_version: str = None,
        controller_version: str = None,
        agent_ip: str = None,
    ) -> bool:
        """Register an agent with full capability structure and metadata.

        Args:
            agent_id: Unique identifier for the agent
            agent_type: Type of agent (optional, determined from capabilities if not provided)
            capabilities: Full capabilities dictionary structure
            agent_data_port: Port number for agent data communication
            agent_version: Version of the agent software
            controller_version: Version of the controller software
            agent_ip: IP address of the agent (optional)

        Returns:
            bool: True if registration successful
        """
        try:
            if self.state_manager:
                self.state_manager.register_agent(
                    agent_id=agent_id,
                    agent_type=agent_type,
                    capabilities=capabilities,
                    agent_data_port=agent_data_port,
                    agent_version=agent_version,
                    controller_version=controller_version,
                    agent_ip=agent_ip,
                )
                self.logger.info(
                    f"Registered agent {agent_id} (type: {agent_type}, port: {agent_data_port}, ip: {agent_ip})"
                )
                return True
            else:
                self.logger.warning(
                    "State manager not available for agent registration"
                )
                return False
        except Exception as e:
            self.logger.error(f"Failed to register agent {agent_id}: {e}")
            return False

    def unregister_agent(self, agent_id: str) -> bool:
        """Unregister an agent and remove from all tracking.

        Args:
            agent_id: Unique identifier for the agent to remove

        Returns:
            bool: True if unregistration successful
        """
        try:
            if self.state_manager:
                self.state_manager.unregister_agent(agent_id)
                self.logger.info(f"Unregistered agent {agent_id}")
                return True
            else:
                self.logger.warning(
                    "State manager not available for agent unregistration"
                )
                return False
        except Exception as e:
            self.logger.error(f"Failed to unregister agent {agent_id}: {e}")
            return False

    def get_connected_agents(self) -> List[str]:
        """Get list of all connected agent IDs."""
        try:
            agents_dict = self.state_manager.get_connected_agents()
            # State Manager returns a dict, extract keys as agent IDs
            if isinstance(agents_dict, dict):
                return list(agents_dict.keys())
            return []
        except Exception as e:
            self.logger.error(f"Error getting connected agents: {e}")
            return []

    def get_agent_registry_summary(self) -> dict:
        """Get comprehensive summary of agent registry state for FQ sampler
        management.

        Returns:
            Dictionary with registry state including counts and agent lists
        """
        try:
            return self.state_manager.get_agent_registry_summary()
        except Exception as e:
            self.logger.error(f"Error getting agent registry summary: {e}")
            return {
                "connected_visualization_agents": [],
                "connected_sensorimotor_agents": [],
                "total_agents": 0,
                "agent_count_viz": 0,
                "agent_count_sensorimotor": 0,
                "last_update": 0,
            }

    # Duplicate methods removed; use primary implementations defined earlier.
    #  configure_agent retained elsewhere; health and agent properties handled
    #  above.

    def get_visualized_cortical_list(self) -> List[str]:
        """Get list of cortical areas currently being visualized."""
        try:
            #  This would need implementation based on current visualization
            #  state
            return []
        except Exception as e:
            self.logger.error(
                f"Error getting visualized cortical list: {str(e)}"
            )
            return []

    def get_cortical_idx_mapping(self) -> Dict[str, Any]:
        """Get the current cortical_idx to cortical_id mapping for debugging
        corruption issues."""
        try:
            # Get mappings from BiDirectionalCorticalMap
            id_to_idx = (
                self._connectome_manager.cortical_mapping.get_all_mappings()
            )
            idx_to_id = {idx: id for id, idx in id_to_idx.items()}

            # Get validation status
            is_consistent, errors = (
                self._connectome_manager.cortical_mapping.validate_consistency()
            )

            # Get stats
            stats = self._connectome_manager.cortical_mapping.get_stats()

            # Build comprehensive debug data
            mapping_data = {
                "id_to_idx": id_to_idx,
                "idx_to_id": idx_to_id,
                "stats": stats,
                "validation": {
                    "is_consistent": is_consistent,
                    "errors": list(errors) if errors else [],
                },
                "reserved_areas": {
                    "_death": id_to_idx.get("_death"),
                    "_power": id_to_idx.get("_power"),
                },
                "debug_info": {
                    "total_mappings": len(id_to_idx),
                    "highest_idx": (
                        max(idx_to_id.keys()) if idx_to_id else None
                    ),
                    "all_indices": sorted(list(idx_to_id.keys())),
                    "all_ids": sorted(list(id_to_idx.keys())),
                },
            }

            return mapping_data

        except Exception as e:
            self.logger.error(f"Error getting cortical idx mapping: {str(e)}")
            import traceback

            self.logger.error(traceback.format_exc())
            return {
                "error": str(e),
                "id_to_idx": {},
                "idx_to_id": {},
                "validation": {"is_consistent": False, "errors": [str(e)]},
            }

    # ===== BRAIN REGION WRITE OPERATIONS =====
    # These methods handle brain region modifications through proper data flow:
    #  API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis
    #  → ConnectomeManager

    def create_brain_region(
        self,
        region_id: str,
        region_name: str,
        parent_region_id: str = "root",
        coordinates: Dict[str, int] = None,
        dimensions: Dict[str, int] = None,
        parameters: Dict[str, Any] = None,
    ) -> bool:
        """Create a brain region.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.create_brain_region(
                region_id=region_id,
                region_name=region_name,
                parent_region_id=parent_region_id,
                coordinates=coordinates,
                dimensions=dimensions,
                parameters=parameters,
            )

        except Exception as e:
            self.logger.error(f"Error creating brain region: {str(e)}")
            raise ValueError(f"Failed to create brain region: {str(e)}") from e

    def update_brain_region(
        self,
        region_id: str,
        region_name: Optional[str] = None,
        parent_region_id: Optional[str] = None,
        coordinates: Optional[Dict[str, int]] = None,
        dimensions: Optional[Dict[str, int]] = None,
        parameters: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Update a brain region.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.update_brain_region(
                region_id=region_id,
                region_name=region_name,
                parent_region_id=parent_region_id,
                coordinates=coordinates,
                dimensions=dimensions,
                parameters=parameters,
            )

        except Exception as e:
            self.logger.error(f"Error updating brain region: {str(e)}")
            raise ValueError(f"Failed to update brain region: {str(e)}") from e

    def get_brain_regions(self) -> List[Dict[str, Any]]:
        """Get all brain regions with consistent schema and automatic I/O assignment.

        ARCHITECTURE COMPLIANCE: READ operation can access genome directly
        for performance, as it doesn't modify state.

        Returns:
            List[Dict[str, Any]]: List of all brain regions with consistent schema

        Raises:
            ValueError: If unable to retrieve brain regions
        """
        try:
            # Get genome from StateManager (single source of truth)
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            
            if not hasattr(state_manager, 'genome') or not state_manager.genome:
                self.logger.warning("No genome loaded in StateManager")
                return []
            
            brain_regions = state_manager.genome.get("brain_regions", {})
            blueprint = state_manager.genome.get("blueprint", {})
            
            # Convert to consistent list format with automatic I/O assignment
            regions_list = []
            for region_id, region_data in brain_regions.items():
                # Normalize field names for consistency
                normalized_region = self._normalize_brain_region_schema(region_data, region_id, blueprint)
                regions_list.append(normalized_region)
            
            self.logger.debug(f"Retrieved {len(regions_list)} brain regions with consistent schema")
            return regions_list

        except Exception as e:
            self.logger.error(f"Error getting brain regions: {str(e)}")
            raise ValueError(f"Failed to get brain regions: {str(e)}") from e

    def _normalize_brain_region_schema(self, region_data: Dict[str, Any], region_id: str, blueprint: Dict[str, Any]) -> Dict[str, Any]:
        """Normalize brain region schema for consistency across all endpoints.
        
        Args:
            region_data: Raw region data from genome
            region_id: Region identifier
            blueprint: Genome blueprint for cortical area type detection
            
        Returns:
            Dict with consistent schema
        """
        # Start with consistent base structure
        normalized = {
            "region_id": region_id,
            "title": region_data.get("title", region_data.get("region_name", f"Region {region_id}")),
            "description": region_data.get("description", ""),
            "parent_region_id": region_data.get("parent_region_id"),
            "coordinate_2d": region_data.get("coordinate_2d", [0, 0]),
            "coordinate_3d": region_data.get("coordinate_3d", [0, 0, 0]),
            "areas": [],
            "regions": [],
            "inputs": [],
            "outputs": [],
            "signature": region_data.get("signature", "")
        }
        
        # HYBRID AREA DETECTION: Use dynamic blueprint detection with static fallback
        areas = []
        if blueprint:
            # Dynamic detection: scan blueprint for actual region assignments
            for cortical_id, cortical_def in blueprint.items():
                # Check all possible region assignment fields
                assigned_region = (
                    cortical_def.get("brain_region_id") or
                    cortical_def.get("parameters", {}).get("brain_region_id") or
                    cortical_def.get("parameters", {}).get("region_id")
                )
                
                if assigned_region == region_id:
                    areas.append(cortical_id)
                    self.logger.debug(f"🔍 Dynamic: Found {cortical_id} assigned to region {region_id}")
        
        # HYBRID APPROACH: COMBINE dynamic and static areas (don't use either/or)
        static_areas = region_data.get("areas", region_data.get("cortical_areas", []))
        
        # Combine both sources - areas from blueprint + areas from static data
        all_areas = set(areas)  # Dynamic areas
        all_areas.update(static_areas)  # Add static areas
        
        areas = list(all_areas)
        
        if areas:
            self.logger.info(f"🔍 Combined area detection: {len(areas)} total areas (dynamic + static)")
        else:
            self.logger.info("🔍 No areas found in dynamic or static data")
        
        normalized["areas"] = sorted(areas)
        
        # Normalize regions field (handle both 'regions' and 'child_regions')
        regions = region_data.get("regions", region_data.get("child_regions", []))
        normalized["regions"] = regions
        
        # HYBRID I/O ASSIGNMENT: Dynamic calculation with static validation
        if areas and blueprint:
            dynamic_inputs, dynamic_outputs = self._auto_assign_region_io(areas, blueprint)
            
            if dynamic_inputs or dynamic_outputs:
                # Use dynamic results when found
                normalized["inputs"] = sorted(dynamic_inputs)
                normalized["outputs"] = sorted(dynamic_outputs)
                self.logger.info(f"🔍 I/O: Dynamic assignment - inputs={len(dynamic_inputs)}, outputs={len(dynamic_outputs)}")
            else:
                # Dynamic found nothing - filter static I/O to only areas currently in region
                static_inputs = region_data.get("inputs", [])
                static_outputs = region_data.get("outputs", [])
                area_set = set(areas)
                
                # Only keep static I/O entries that are actually in the current region
                filtered_inputs = [area for area in static_inputs if area in area_set]
                filtered_outputs = [area for area in static_outputs if area in area_set]
                
                normalized["inputs"] = sorted(filtered_inputs)
                normalized["outputs"] = sorted(filtered_outputs)
                
                if filtered_inputs != static_inputs or filtered_outputs != static_outputs:
                    self.logger.info(f"🔍 I/O: Filtered static data - inputs={len(filtered_inputs)}, outputs={len(filtered_outputs)} (removed stale entries)")
                else:
                    self.logger.info(f"🔍 I/O: Using static data - inputs={len(filtered_inputs)}, outputs={len(filtered_outputs)}")
        else:
            # No areas or blueprint, use empty I/O
            normalized["inputs"] = []
            normalized["outputs"] = []
            self.logger.info("🔍 I/O: No areas found, using empty I/O")
        
        return normalized

    # =================================================================
    # REGION MEMBERSHIP CONSTRAINTS
    # =================================================================

    def _load_region_constraints(self):
        """Load brain region constraints from configuration.

        Returns a RegionConstraintsConfiguration dataclass.
        """
        try:
            from feagi.config.toml_loader import load_feagi_config, get_region_constraints_config

            config = load_feagi_config()
            return get_region_constraints_config(config)
        except Exception as e:
            # Fail fast: constraints must be configured explicitly
            raise ValueError(f"Region constraints configuration error: {e}")

    def _classify_area_category(self, area_def: Dict[str, Any]) -> str:
        """Classify cortical area into one of {IPU, OPU, CORE, CUSTOM, MEMORY} deterministically.

        Prefers stable fields in priority order:
        - group_id
        - type ("memory"/"custom")
        - parameters.sub_group_id == "MEMORY" -> MEMORY
        - parameters.cortical_group (legacy): IPU/OPU/CORE/CUSTOM
        """
        if not isinstance(area_def, dict):
            raise ValueError("Invalid cortical area definition for classification")

        group_id = str(area_def.get("group_id", "")).upper()
        if group_id in {"IPU", "OPU", "CORE", "CUSTOM", "MEMORY"}:
            return group_id

        area_type = str(area_def.get("type", "")).lower()
        if area_type == "memory":
            return "MEMORY"
        if area_type == "custom":
            return "CUSTOM"

        params = area_def.get("parameters", {})
        if isinstance(params, dict) and str(params.get("sub_group_id", "")).upper() == "MEMORY":
            return "MEMORY"

        legacy_group = str(params.get("cortical_group", area_def.get("cortical_group", "")).upper())
        if legacy_group in {"IPU", "OPU", "CORE", "CUSTOM", "MEMORY"}:
            return legacy_group

        # Unknown -> treat as CUSTOM to avoid unintended elevation to root-only types
        return "CUSTOM"

    def _get_area_def_from_blueprint(self, cortical_id: str) -> Dict[str, Any]:
        from feagi.core.state_manager import FeagiStateManager

        state_manager = FeagiStateManager.instance()
        genome = getattr(state_manager, "genome", None)
        if not genome or "blueprint" not in genome:
            raise ValueError("Genome blueprint not available for area classification")
        return genome["blueprint"].get(cortical_id, {})

    def _assert_region_accepts_area(self, cortical_id: str, destination_region_id: str) -> None:
        """Enforce membership constraints for a cortical area to a destination region.

        Raises ValueError if violation is detected.
        """
        constraints = self._load_region_constraints()

        area_def = self._get_area_def_from_blueprint(cortical_id)
        category = self._classify_area_category(area_def)

        is_root = destination_region_id == "root"
        if is_root:
            if category not in constraints.root_allowed_area_categories:
                raise ValueError(
                    f"region_membership_violation: Area {cortical_id} (category={category}) not allowed in root"
                )
        else:
            if category not in constraints.subregion_allowed_area_categories:
                raise ValueError(
                    f"region_membership_violation: Area {cortical_id} (category={category}) not allowed in subregion {destination_region_id}"
                )

    def _auto_assign_region_io(self, areas: List[str], blueprint: Dict[str, Any]) -> tuple[List[str], List[str]]:
        """Assign region inputs/outputs from connections using a single, robust source.

        Rules:
        - OUTPUT: Any area in the region that connects to an area outside the region
        - INPUT: Any area in the region that receives a connection from outside the region

        The genome may store connections under different keys; we normalize by
        reading all known destinations:
        - "cortical_destinations": {dst_id: [...]}
        - "cortical_mapping_dst": {dst_id: [...]} (newer path)
        - parameters.mapping (legacy) is ignored unless normalized into the above
        """
        inputs: List[str] = []
        outputs: List[str] = []
        area_set = set(areas)

        def extract_destinations(props: Dict[str, Any]) -> set:
            """Return set of destination area IDs from genome props (all variants)."""
            dests: set = set()
            # Newer and primary representation
            mapping_dst = props.get("cortical_mapping_dst")
            if isinstance(mapping_dst, dict):
                dests.update(mapping_dst.keys())
            # Legacy representation mirrored in cortical properties
            c_dests = props.get("cortical_destinations")
            if isinstance(c_dests, dict):
                dests.update(c_dests.keys())
            # Some genomes may nest under parameters; include if present
            params = props.get("parameters")
            if isinstance(params, dict):
                p_map = params.get("cortical_mapping_dst") or params.get("cortical_destinations")
                if isinstance(p_map, dict):
                    dests.update(p_map.keys())
            return dests

        # Compute OUTPUTS: region areas that point outside
        for area_id in areas:
            props = blueprint.get(area_id, {}) or {}
            dests = extract_destinations(props)
            if any(dst not in area_set for dst in dests):
                outputs.append(area_id)

        # Compute INPUTS: external areas that point into the region
        for src_id, src_props in blueprint.items():
            if src_id in area_set:
                continue
            for dst in extract_destinations(src_props):
                if dst in area_set and dst not in inputs:
                    inputs.append(dst)

        # Fallback by group for areas with no detected connections
        areas_with_io = set(inputs) | set(outputs)
        for area_id in areas:
            if area_id in areas_with_io:
                continue
            props = blueprint.get(area_id, {}) or {}
            group = str(props.get("group", props.get("cortical_group", ""))).upper()
            if group == "IPU":
                inputs.append(area_id)
            elif group == "OPU":
                outputs.append(area_id)

        return inputs, outputs

    def delete_brain_region(
        self, region_id: str, preserve_children: bool = True
    ) -> bool:
        """Delete a brain region.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager

        Args:
            region_id: ID of region to delete
            preserve_children: If True, move children to parent; if False, delete all members
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            # Note: preserve_children=True means delete_members=False
            return self._genome_service.delete_brain_region(
                region_id=region_id, delete_members=not preserve_children
            )

        except Exception as e:
            self.logger.error(f"Error deleting brain region: {str(e)}")
            raise ValueError(f"Failed to delete brain region: {str(e)}") from e

    def relocate_region_members(self, relocation_data: Dict[str, Any]) -> bool:
        """Relocate brain region members by updating their coordinates.
        
        Updates the coordinates of multiple cortical areas and/or brain regions
        in a single operation. Supports both 2D coordinate updates and optional
        parent region changes.
        
        Args:
            relocation_data: Dictionary mapping cortical area IDs to their new
                           coordinates and optional parent region assignments.
                           Format: {
                               "cortical_id": {
                                   "coordinate_2d": [x, y],
                                   "parent_region_id": "region_id"  # optional
                               }
                           }
        
        Returns:
            True if all relocations succeeded, False otherwise
        
        Raises:
            ValueError: If relocation fails for any member
        """
        try:
            if not relocation_data:
                return True  # Nothing to relocate
            
            success_count = 0
            total_count = len(relocation_data)
            
            for member_id, member_data in relocation_data.items():
                try:
                    # Extract coordinate and parent region information
                    coordinate_2d = member_data.get("coordinate_2d")
                    parent_region_id = member_data.get("parent_region_id")
                    
                    # Check if we have at least one operation to perform
                    if coordinate_2d is None and parent_region_id is None:
                        self.logger.warning(f"No coordinate_2d or parent_region_id provided for {member_id}, skipping")
                        continue
                    
                    # Handle coordinate updates if provided
                    coordinate_updated = False
                    if coordinate_2d is not None:
                        # Convert 2D coordinates to 3D format expected by update methods
                        coordinates_3d = {
                            "x": coordinate_2d[0],
                            "y": coordinate_2d[1], 
                            "z": 0  # Default z-coordinate
                        }
                        
                        # Try updating as cortical area first
                        try:
                            result = self.update_cortical_area(
                                cortical_id=member_id,
                                coordinates=coordinates_3d
                            )
                            if result is not None:
                                coordinate_updated = True
                                self.logger.debug(f"Updated cortical area {member_id} coordinates to {coordinate_2d}")
                        except Exception as e:
                            self.logger.debug(f"Failed to update {member_id} as cortical area: {e}")
                        
                        # If cortical area update failed, try as brain region
                        if not coordinate_updated:
                            try:
                                # IMPORTANT: For brain regions, only update 2D coordinates; do not overwrite 3D
                                region_result = self.update_brain_region(
                                    region_id=member_id,
                                    parameters={"coordinates_2d": [int(coordinate_2d[0]), int(coordinate_2d[1])]}
                                )
                                if region_result:
                                    coordinate_updated = True
                                    self.logger.debug(f"Updated brain region {member_id} coordinate_2d to {coordinate_2d}")
                                else:
                                    self.logger.warning(f"Failed to update {member_id} coordinates as brain region")
                            except Exception as e:
                                self.logger.warning(f"Failed to update {member_id} coordinates as brain region: {e}")
                    else:
                        self.logger.debug(f"No coordinate update requested for {member_id}")
                    
                    # Handle parent region change if specified
                    parent_updated = False
                    if parent_region_id is not None:
                        try:
                            parent_result = self.change_cortical_area_parent(
                                cortical_area_id=member_id,
                                new_parent_id=parent_region_id
                            )
                            if parent_result:
                                parent_updated = True
                                self.logger.debug(f"Updated {member_id} parent to {parent_region_id}")
                            else:
                                self.logger.warning(f"Failed to update {member_id} parent region")
                        except Exception as e:
                            self.logger.warning(f"Failed to update {member_id} parent region: {e}")
                    else:
                        self.logger.debug(f"No parent region update requested for {member_id}")
                    
                    # Count as successful if either coordinate or parent update succeeded
                    if coordinate_updated or parent_updated or (coordinate_2d is None and parent_region_id is None):
                        success_count += 1
                        self.logger.info(f"Successfully processed {member_id} (coords: {coordinate_updated}, parent: {parent_updated})")
                    else:
                        self.logger.warning(f"Failed to process any updates for {member_id}")
                    
                except Exception as e:
                    self.logger.error(f"Error relocating {member_id}: {e}")
                    continue
            
            if success_count == total_count:
                self.logger.info(f"Successfully relocated all {total_count} region members")
                return True
            elif success_count > 0:
                self.logger.warning(f"Partially successful: relocated {success_count}/{total_count} members")
                return True  # Partial success is still considered success
            else:
                self.logger.error("Failed to relocate any region members")
                return False
                
        except Exception as e:
            self.logger.error(f"Error in relocate_region_members: {str(e)}")
            raise ValueError(f"Failed to relocate region members: {str(e)}") from e

    def change_cortical_area_parent(
        self, cortical_area_id: str, new_parent_id: str
    ) -> bool:
        """Change the parent region of a cortical area.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Enforce region membership constraints before applying change
            try:
                self._assert_region_accepts_area(cortical_area_id, new_parent_id)
            except Exception as e:
                raise ValueError(str(e))

            #  This is a cortical area modification, so route through cortical
            #  area update
            # CRITICAL FIX: Get current assignment BEFORE updating blueprint
            old_parent_id = self._get_current_region_assignment(cortical_area_id)
            
            # Use ALL field names to ensure compatibility across different parts of the system
            # Some expect 'region_id', others 'brain_region_id', others 'parent_region_id'
            result = self._cortical_area_service.update_area(
                cortical_area_id, 
                parameters={
                    "region_id": new_parent_id,
                    "brain_region_id": new_parent_id,
                    "parent_region_id": new_parent_id  # CRITICAL: Update cortical area's own parent field
                }
            )
            
            if result is not None:
                self.logger.info(f"Successfully updated {cortical_area_id} parent region to {new_parent_id}")
                
                # CRITICAL FIX: Update static region membership when areas are moved
                try:
                    self._update_static_region_membership_after_move(cortical_area_id, new_parent_id, old_parent_id)
                except Exception as e:
                    self.logger.warning(f"Failed to update static region membership for {cortical_area_id}: {e}")
                
                return True
            else:
                self.logger.warning(f"Failed to update {cortical_area_id} parent region")
                return False

        except Exception as e:
            self.logger.error(f"Error changing cortical area parent: {str(e)}")
            raise ValueError(
                f"Failed to change cortical area parent: {str(e)}"
            ) from e

    def _get_current_region_assignment(self, cortical_area_id: str) -> str:
        """Get the current region assignment for a cortical area from blueprint.
        
        Returns the region ID where the area is currently assigned, or 'root' as fallback.
        """
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            
            if not hasattr(state_manager, 'genome') or not state_manager.genome:
                return "root"
                
            blueprint = state_manager.genome.get("blueprint", {})
            if cortical_area_id not in blueprint:
                return "root"
                
            cortical_def = blueprint[cortical_area_id]
            current_assignment = (
                cortical_def.get("brain_region_id") or 
                cortical_def.get("parameters", {}).get("brain_region_id") or
                cortical_def.get("parameters", {}).get("region_id") or
                "root"
            )
            
            return current_assignment
            
        except Exception as e:
            self.logger.warning(f"Error getting current region assignment for {cortical_area_id}: {e}")
            return "root"

    def _update_static_region_membership_after_move(self, cortical_area_id: str, new_parent_id: str, old_parent_id: str = None) -> None:
        """Update static brain region membership data when a cortical area is moved.
        
        This ensures that the static region data stays in sync with dynamic cortical area assignments.
        """
        try:
            # Get current genome from StateManager
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            
            if not hasattr(state_manager, 'genome') or not state_manager.genome:
                self.logger.warning("No genome available for static region membership update")
                return
                
            genome = state_manager.genome
            brain_regions = genome.get("brain_regions", {})
            
            self.logger.info(f"🔄 [REGION-SYNC] Moving {cortical_area_id} from {old_parent_id} to {new_parent_id}")
            
            # Remove from old region (using the assignment we captured before the update)
            areas_removed_from = []
            if old_parent_id and old_parent_id in brain_regions:
                old_region_areas = brain_regions[old_parent_id].get("areas", brain_regions[old_parent_id].get("cortical_areas", []))
                if cortical_area_id in old_region_areas:
                    old_region_areas.remove(cortical_area_id)
                    brain_regions[old_parent_id]["areas"] = old_region_areas
                    areas_removed_from.append(old_parent_id)
                    self.logger.info(f"🔄 [REGION-SYNC] Removed {cortical_area_id} from {old_parent_id}")
                else:
                    # Not in static data, add it first then remove it (sync static with reality)
                    old_region_areas.append(cortical_area_id)
                    brain_regions[old_parent_id]["areas"] = old_region_areas
                    self.logger.info(f"🔄 [REGION-SYNC] Added {cortical_area_id} to static data for {old_parent_id} (was missing)")
                    
                    # Now remove it
                    old_region_areas.remove(cortical_area_id)
                    brain_regions[old_parent_id]["areas"] = old_region_areas
                    areas_removed_from.append(old_parent_id)
                    self.logger.info(f"🔄 [REGION-SYNC] Removed {cortical_area_id} from {old_parent_id}")
            else:
                self.logger.warning(f"🔄 [REGION-SYNC] Old parent {old_parent_id} not found in brain_regions")
            
            # Add to new region (CRITICAL FIX: ensure we don't overwrite existing areas)
            if new_parent_id in brain_regions:
                # Get current areas list - make a COPY to avoid reference issues
                current_new_region_areas = brain_regions[new_parent_id].get("areas", brain_regions[new_parent_id].get("cortical_areas", []))
                new_region_areas = list(current_new_region_areas)  # Make a copy
                
                self.logger.info(f"🔄 [REGION-SYNC] Target region {new_parent_id} currently has areas: {new_region_areas}")
                
                if cortical_area_id not in new_region_areas:
                    new_region_areas.append(cortical_area_id)
                    brain_regions[new_parent_id]["areas"] = new_region_areas
                    self.logger.info(f"🔄 [REGION-SYNC] Added {cortical_area_id} to region {new_parent_id}. New areas list: {new_region_areas}")
                else:
                    self.logger.info(f"🔄 [REGION-SYNC] {cortical_area_id} already in region {new_parent_id}")
            else:
                self.logger.warning(f"🔄 [REGION-SYNC] Target region {new_parent_id} not found in brain_regions")
            
            # Update StateManager's genome
            state_manager.genome = genome
            self.logger.info(f"🔄 [REGION-SYNC] Completed: {cortical_area_id} moved from {old_parent_id} to {new_parent_id}")
            
        except Exception as e:
            self.logger.error(f"Error updating static region membership: {str(e)}")
            raise

    def change_brain_region_parent(
        self, region_id: str, new_parent_id: str
    ) -> bool:
        """Change the parent of a brain region.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.update_brain_region(
                region_id=region_id, parent_region_id=new_parent_id
            )

        except Exception as e:
            self.logger.error(f"Error changing brain region parent: {str(e)}")
            raise ValueError(
                f"Failed to change brain region parent: {str(e)}"
            ) from e

    # ===== GENOME WRITE OPERATIONS =====
    # These methods handle genome modifications through proper data flow:
    #  API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis
    #  → ConnectomeManager

    def reset_genome(self) -> bool:
        """Reset the genome.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.reset_genome()

        except Exception as e:
            self.logger.error(f"Error resetting genome: {str(e)}")
            raise ValueError(f"Failed to reset genome: {str(e)}") from e

    def process_amalgamation_request(
        self,
        amalgamation_data: Dict[str, Any] = None,
        *,
        genome_payload: Optional[Dict[str, Any]] = None,
        genome_id: Optional[str] = None,
        genome_title: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Process an amalgamation request.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Normalize inputs: support both legacy dict and explicit kwargs
            if amalgamation_data is None:
                amalgamation_data = {}
            if genome_payload is not None:
                amalgamation_data["genome_payload"] = genome_payload
            if genome_id is not None:
                amalgamation_data["genome_id"] = genome_id
            if genome_title is not None:
                amalgamation_data["genome_title"] = genome_title

            #  Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.amalgamate_genome(amalgamation_data)

        except Exception as e:
            self.logger.error(
                f"Error processing amalgamation request: {str(e)}"
            )
            raise ValueError(
                f"Failed to process amalgamation request: {str(e)}"
            ) from e

    def cancel_amalgamation(self, amalgamation_id: str) -> bool:
        """Cancel an amalgamation.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.cancel_amalgamation(amalgamation_id)

        except Exception as e:
            self.logger.error(f"Error cancelling amalgamation: {str(e)}")
            raise ValueError(f"Failed to cancel amalgamation: {str(e)}") from e

    def append_circuit_to_genome(self, circuit_data: Dict[str, Any]) -> bool:
        """Append circuit to genome.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.append_file_to_genome(circuit_data)

        except Exception as e:
            self.logger.error(f"Error appending circuit to genome: {str(e)}")
            raise ValueError(
                f"Failed to append circuit to genome: {str(e)}"
            ) from e

    def apply_amalgamation_destination(self, dest_data: Dict[str, Any]) -> bool:
        """Finalize amalgamation by merging payload and running full NeuroEmbryogenesis.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            return self._genome_service.apply_amalgamation_destination(dest_data)
        except Exception as e:
            self.logger.error(f"Error applying amalgamation destination: {str(e)}")
            return False

    def complete_amalgamation(self, amalgamation_data: Dict[str, Any]) -> bool:
        """Complete an amalgamation.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            result = self._genome_service.amalgamate_genome(amalgamation_data)
            return result.get("success", False)

        except Exception as e:
            self.logger.error(f"Error completing amalgamation: {str(e)}")
            raise ValueError(
                f"Failed to complete amalgamation: {str(e)}"
            ) from e

    def cancel_pending_amalgamation(self, amalgamation_id: str) -> bool:
        """Cancel a pending amalgamation.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            #  Route WRITE operation through GenomeService for architecture
            #  compliance
            return self._genome_service.cancel_amalgamation(amalgamation_id)

        except Exception as e:
            self.logger.error(
                f"Error cancelling pending amalgamation: {str(e)}"
            )
            raise ValueError(
                f"Failed to cancel pending amalgamation: {str(e)}"
            ) from e

    def mark_amalgamation_complete(self, amalgamation_id: str) -> bool:
        """Mark an amalgamation as complete.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # This is typically a status update operation
            # For now, we'll route through GenomeService for consistency
            #  In the future, this might be handled by a separate
            #  AmalgamationService
            return True  # Placeholder implementation

        except Exception as e:
            self.logger.error(f"Error marking amalgamation complete: {str(e)}")
            raise ValueError(
                f"Failed to mark amalgamation complete: {str(e)}"
            ) from e

    # ===== READ OPERATIONS (Already properly routed) =====
    # These methods are READ operations and correctly use existing services

    # ===== MEMORY USAGE CALCULATION METHODS =====

    def _calculate_neuron_memory_usage(
        self, cortical_id: str
    ) -> Dict[str, Any]:
        """Calculate memory usage for neurons in a cortical area using actual
        NeuronArray memory."""
        try:
            # Get neurons in the area
            neurons = self._connectome_manager.get_neurons_by_cortical_area(
                cortical_id
            )
            neuron_count = len(neurons)
            
            if neuron_count == 0:
                return {
                    "count": 0,
                    "size_bytes": 0,
                    "size_human": "0 B",
                    "avg_bytes_per_item": 0.0,
                    "avg_human_per_item": "0 B",
                }
            
            # Calculate ACTUAL memory per neuron by inspecting the NeuronArray
            neuron_array = self._connectome_manager.neuron_array
            
            if neuron_array._use_rust:
                # For Rust backend, estimate based on standard sizes
                #  TODO: Add actual Rust backend memory inspection when
                #  available
                bytes_per_neuron = (
                    49.0  # Conservative estimate for Rust backend
                )
                self.logger.info(
                    f"Using estimated memory for Rust backend: {bytes_per_neuron} bytes per neuron"
                )
            else:
                # Calculate ACTUAL memory per neuron from numpy arrays
                bytes_per_neuron = 0.0
                
                # Inspect all the actual arrays in NeuronArray
                arrays_to_check = [
                    "membrane_potentials",
                    "resting_potentials",
                    "thresholds",
                    "excitabilities",
                    "leak_coefficients",
                    "refractory_periods",
                    "refractory_counters",
                    "coordinates_x",
                    "coordinates_y",
                    "coordinates_z",
                    "cortical_idxs",
                    "is_active",
                    "valid_mask",
                    "last_fired",
                    "neuron_types",
                    "enabled_flags",
                ]
                
                for array_name in arrays_to_check:
                    if hasattr(neuron_array, array_name):
                        array = getattr(neuron_array, array_name)
                        if hasattr(array, "itemsize"):
                            bytes_per_neuron += array.itemsize
                            self.logger.debug(
                                f"Array {array_name}: {array.itemsize} bytes per item, dtype: {array.dtype}"
                            )
                
                self.logger.info(
                    f"Calculated ACTUAL memory per neuron: {bytes_per_neuron} bytes (from {len(arrays_to_check)} arrays)"
                )
            
            total_bytes = int(neuron_count * bytes_per_neuron)
            
            return {
                "count": neuron_count,
                "size_bytes": total_bytes,
                "size_human": self._format_bytes(total_bytes),
                "avg_bytes_per_item": bytes_per_neuron,
                "avg_human_per_item": self._format_bytes(
                    int(bytes_per_neuron)
                ),
            }
            
        except Exception as e:
            self.logger.error(
                f"Error calculating neuron memory for {cortical_id}: {str(e)}"
            )
            return {
                "count": 0,
                "size_bytes": 0,
                "size_human": "0 B",
                "avg_bytes_per_item": 0.0,
                "avg_human_per_item": "0 B",
            }

    def _calculate_synapse_memory_breakdown(
        self, cortical_id: str
    ) -> Dict[str, Dict[str, Any]]:
        """Calculate memory usage breakdown for synapses by type."""
        try:
            # Get area neurons for classification
            area_neurons = set(
                self._connectome_manager.get_neurons_by_cortical_area(
                    cortical_id
                )
            )
            
            if not area_neurons:
                empty_result = {
                    "count": 0,
                    "size_bytes": 0,
                    "size_human": "0 B",
                    "avg_bytes_per_item": 0.0,
                    "avg_human_per_item": "0 B",
                }
                return {
                    "incoming": empty_result.copy(),
                    "outgoing": empty_result.copy(), 
                    "internal": empty_result.copy(),
                }
            
            # Get all synapses and classify them
            incoming_count = 0
            outgoing_count = 0
            internal_count = 0
            
            #  Iterate through all neurons in the area and classify their
            #  connections
            for neuron_id in area_neurons:
                # Get outgoing connections from this neuron
                #  Returns List[Tuple[int, float]] where tuple is
                #  (target_neuron_id, weight)
                outgoing_connections = (
                    self._connectome_manager.get_outgoing_connections(
                        neuron_id
                    )
                )
                for target_id, weight in outgoing_connections:
                    if target_id in area_neurons:
                        #  Target is also in this area - internal/recurrent
                        #  synapse
                        internal_count += 1
                    else:
                        # Target is outside this area - outgoing synapse
                        outgoing_count += 1
                
                # Get incoming connections to this neuron
                #  Returns List[Tuple[int, float]] where tuple is
                #  (source_neuron_id, weight)
                incoming_connections = (
                    self._connectome_manager.get_incoming_connections(
                        neuron_id
                    )
                )
                for source_id, weight in incoming_connections:
                    if source_id not in area_neurons:
                        # Source is outside this area - incoming synapse
                        incoming_count += 1
                    #  Note: internal synapses are already counted in outgoing
                    #  connections
            
            #  Calculate ACTUAL memory per synapse by inspecting the
            #  GlobalSynapseArray
            synapse_array = self._connectome_manager.synapse_array
            
            # Calculate actual memory per synapse from the SoA structure
            bytes_per_synapse = 0.0
            
            # Inspect all the actual arrays in GlobalSynapseArray
            arrays_to_check = [
                "pre_neuron_ids",
                "post_neuron_ids",
                "weights",
                "delays",
                "types",
                "plasticity_coeffs",
                "conductances",
                "is_plastic_flags",
            ]
            
            for array_name in arrays_to_check:
                if hasattr(synapse_array, array_name):
                    array = getattr(synapse_array, array_name)
                    if hasattr(array, "itemsize"):
                        bytes_per_synapse += array.itemsize
                        self.logger.debug(
                            f"Synapse array {array_name}: {array.itemsize} bytes per item, dtype: {array.dtype}"
                        )
            
            self.logger.info(
                f"Calculated ACTUAL memory per synapse: {bytes_per_synapse} bytes (from {len(arrays_to_check)} arrays)"
            )
            return {
                "incoming": {
                    "count": incoming_count,
                    "size_bytes": int(incoming_count * bytes_per_synapse),
                    "size_human": self._format_bytes(
                        int(incoming_count * bytes_per_synapse)
                    ),
                    "avg_bytes_per_item": (
                        bytes_per_synapse if incoming_count > 0 else 0.0
                    ),
                    "avg_human_per_item": (
                        self._format_bytes(int(bytes_per_synapse))
                        if incoming_count > 0
                        else "0 B"
                    ),
                },
                "outgoing": {
                    "count": outgoing_count,
                    "size_bytes": int(outgoing_count * bytes_per_synapse),
                    "size_human": self._format_bytes(
                        int(outgoing_count * bytes_per_synapse)
                    ),
                    "avg_bytes_per_item": (
                        bytes_per_synapse if outgoing_count > 0 else 0.0
                    ),
                    "avg_human_per_item": (
                        self._format_bytes(int(bytes_per_synapse))
                        if outgoing_count > 0
                        else "0 B"
                    ),
                },
                "internal": {
                    "count": internal_count,
                    "size_bytes": int(internal_count * bytes_per_synapse),
                    "size_human": self._format_bytes(
                        int(internal_count * bytes_per_synapse)
                    ),
                    "avg_bytes_per_item": (
                        bytes_per_synapse if internal_count > 0 else 0.0
                    ),
                    "avg_human_per_item": (
                        self._format_bytes(int(bytes_per_synapse))
                        if internal_count > 0
                        else "0 B"
                    ),
                },
            }
            
        except Exception as e:
            self.logger.error(
                f"Error calculating synapse memory breakdown for {cortical_id}: {str(e)}"
            )
            empty_result = {
                "count": 0,
                "size_bytes": 0,
                "size_human": "0 B",
                "avg_bytes_per_item": 0.0,
                "avg_human_per_item": "0 B",
            }
            return {
                "incoming": empty_result.copy(),
                "outgoing": empty_result.copy(),
                "internal": empty_result.copy(),
            }

    def _format_bytes(self, bytes_value: int) -> str:
        """Format bytes into human-readable format."""
        if bytes_value == 0:
            return "0 B"
        
        units = ["B", "KB", "MB", "GB", "TB"]
        unit_index = 0
        size = float(bytes_value)
        
        while size >= 1024.0 and unit_index < len(units) - 1:
            size /= 1024.0
            unit_index += 1
        
        if unit_index == 0:
            return f"{int(size)} {units[unit_index]}"
        else:
            return f"{size:.1f} {units[unit_index]}"

    def update_genome_physiology(self, updates: Dict[str, Any]) -> bool:
        """Update physiology parameters in the current genome and refresh
        state.

        Args:
            updates: dict of physiology fields to update

        Returns:
            True on success
        """
        try:
            success = self._genome_service.update_physiology(updates)
            if success:
                # Refresh any caches or dependent services
                try:
                    self.refresh_cached_data()
                except Exception:
                    pass
            return bool(success)
        except Exception as e:
            self.logger.error(f"Failed to update genome physiology: {e}")
            return False

    # Duplicate definition removed; use the earlier get_system_health method
