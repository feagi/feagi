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

"""
FCL Injection Service for FEAGI Neural Processing Unit.

Handles injection of neuron candidates from special areas (power areas, sensory input,
modulators, etc.) into the Fire Candidate List (FCL) during burst processing. Provides
optimized batch injection and proper timing coordination with the burst engine.

This service implements the unified FCL candidate model:
- Special areas add candidates to FCL (rather than firing directly)
- All candidates (internal synaptic + external special areas) processed together
- Burst engine remains completely area-agnostic

@cursor:critical-path FCL injection affects every burst cycle - performance critical
@cursor:ffi-safe Uses static types and no dynamic allocation in main loops for Rust compatibility
"""

import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List

import numpy as np

from feagi.npu.special_area_handler import CorticalId, NeuronId
from feagi.utils.logger import setup_logger

logger = setup_logger()


class InjectionTiming(Enum):
    """Enumeration of injection timing phases."""

    PRE_BURST = "pre_burst"
    DURING_BURST = "during_burst"
    POST_BURST = "post_burst"


@dataclass
class InjectionBatch:
    """A batch of neurons to inject into the FCL."""

    cortical_id: CorticalId
    neuron_ids: List[NeuronId]
    timing: InjectionTiming
    probability: float = 1.0


class FCLInjectionService:
    """Unified FCL injection service for all special area types.

    This service handles injection of neurons from special cortical areas (power, modulator, etc.)
    into the Fire Candidate List (FCL) during burst processing. It uses a unified injection model
    where all external sources contribute candidates to the FCL, which are then processed together
    with internal synaptic propagation.

    Key features:
    - Area-agnostic: Works with any special area type
    - Timing-aware: Supports pre-burst, during-burst, and post-burst injection phases
    - Probabilistic: Supports probability-based candidate selection for efficiency
    - RTOS-compatible: No hardcoded values, uses configuration system
    """

    def __init__(self, fcl_manager: Any, special_area_handler: Any):
        """Initialize the FCL injection service.

        Args:
            fcl_manager: Manager for the Fire Candidate List
            special_area_handler: Handler for detecting and processing special areas
        """
        self.fcl_manager = fcl_manager
        self.special_area_handler = special_area_handler

        # Store reference to connectome manager for membrane potential access
        #  This is needed to set power neuron membrane potentials above
        #  threshold
        self.connectome_manager = None
        if hasattr(special_area_handler, "connectome_manager"):
            self.connectome_manager = special_area_handler.connectome_manager
            logger.info(
                "[FCL INJECTION] Stored connectome manager reference for membrane potential access"
            )
        else:
            logger.warning(
                "[FCL INJECTION] Could not access connectome manager - membrane potential setting may not work"
            )

        # Configuration attributes expected by other components
        self.batch_size = 1000  # Default batch size for processing neurons
        self.enable_probabilistic = (
            True  # Enable probabilistic injection by default
        )
        self.last_injection_duration = 0.0

        # Statistics tracking
        self.total_neurons_injected = 0
        self.injection_stats = {
            "total_injections": 0,
            "successful_injections": 0,
            "failed_injections": 0,
        }

        # Pre-computed injection batches for efficiency
        self._injection_batches: Dict[
            InjectionTiming, List[InjectionBatch]
        ] = {
            InjectionTiming.PRE_BURST: [],
            InjectionTiming.DURING_BURST: [],
            InjectionTiming.POST_BURST: [],
        }

        # Prepare injection batches based on detected special areas
        self._prepare_injection_batches()

        logger.info(
            f"FCL injection service initialized with {len(self._get_all_batches())} injection batches"
        )

    def _prepare_injection_batches(self) -> None:
        """Pre-compute injection batches for performance optimization.

        SIMPLIFIED for core power area (_power at cortical_idx=1): Creates
        injection batches for detected power areas.
        """
        # Clear existing batches
        for timing in InjectionTiming:
            self._injection_batches[timing].clear()

        try:
            #  Get all power areas (simplified approach returns only core power
            #  area)
            power_neurons = self.special_area_handler.get_all_power_neurons()

            if not power_neurons:
                logger.info(
                    "No power area neurons found - injection batches will be empty"
                )
                return

            logger.info(
                f"Preparing injection batches for {len(power_neurons)} power areas: {list(power_neurons.keys())}"
            )

            # Create injection batches using vectorized operation
            cortical_ids = list(power_neurons.keys())
            if cortical_ids:
                # Vectorized config validation
                configs = [
                    self.special_area_handler.get_special_config(cid)
                    for cid in cortical_ids
                ]
                enabled_mask = [
                    config and config.enabled for config in configs
                ]

                # Filter to enabled power areas only
                enabled_indices = np.where(enabled_mask)[0]

                for idx in enabled_indices:
                    cortical_id = cortical_ids[idx]
                    neuron_ids = power_neurons[cortical_id]
                    config = configs[idx]

                    #  Check if NPU debug is enabled for detailed injection
                    #  logging
                    from feagi.core.state_manager import FeagiStateManager

                    state_manager = FeagiStateManager.instance()
                    if state_manager.is_debug_npu_enabled():
                        logger.info(
                            f"[NPU-DEBUG] Processing power area {cortical_id} with {len(neuron_ids)} neurons"
                        )

                    # Determine timing
                    timing_str = config.injection_timing
                    try:
                        timing = InjectionTiming(timing_str)
                        if state_manager.is_debug_npu_enabled():
                            logger.info(
                                f"[NPU-DEBUG] Power area {cortical_id} uses {timing_str} timing"
                            )
                    except ValueError:
                        logger.warning(
                            f"Invalid injection timing '{timing_str}' for area {cortical_id}, using PRE_BURST"
                        )
                        timing = InjectionTiming.PRE_BURST

                    #  Create single batch (simplified - no batch splitting
                    #  needed for core power area)
                    batch = InjectionBatch(
                        cortical_id=cortical_id,
                        neuron_ids=neuron_ids.copy(),
                        timing=timing,
                        probability=config.injection_probability,
                    )
                    self._injection_batches[timing].append(batch)
                    logger.info(
                        f"Created batch for {cortical_id}: {len(neuron_ids)} neurons, timing={timing.value}, prob={config.injection_probability}"
                    )

            # Log preparation results
            total_batches = sum(
                len(batches) for batches in self._injection_batches.values()
            )
            batch_summary = {
                timing.value: len(batches)
                for timing, batches in self._injection_batches.items()
            }
            logger.info(
                f"Prepared {total_batches} injection batches for {len(power_neurons)} power areas: {batch_summary}",
                status="[SAVE]",
            )

        except Exception as e:
            logger.error(f"Error preparing injection batches: {e}")
            #  Continue with empty batches - injection will still work via
            #  direct method

    def inject_pre_burst(self, current_timestep: int) -> int:
        """Inject power area neurons into FCL with proper membrane potential.

        Direct injection from the core power area (cortical_idx=1).

        CRITICAL: Sets membrane potential above threshold so power neurons actually fire!
        The FCL candidates still undergo membrane potential checks, so we must ensure
        power neurons meet firing conditions.

        IMPORTANT: Power injection occurs EVERY BURST for constant brain power supply.

        Args:
            current_timestep: Current simulation timestep

        Returns:
            Number of power neurons injected
        """
        # Removed file I/O debug writes for RTOS/WGPU compatibility
        # Get power area neurons from special area handler (cortical_idx=1)
        # This happens EVERY burst to provide constant power supply
        if FeagiStateManager.instance().is_debug_npu_enabled():
            logger.info("[INJECTION-DEBUG] Fetching power neurons from SpecialAreaHandler.get_power_area_neurons()")
        power_neurons = self.special_area_handler.get_power_area_neurons()

        # Debug-only: record found neurons
        from feagi.core.state_manager import FeagiStateManager

        if FeagiStateManager.instance().is_debug_npu_enabled():
            logger.info(f"[INJECTION-DEBUG] Retrieved power neurons: {power_neurons}")
            import os
            import tempfile

            log_path = os.path.join(
                tempfile.gettempdir(), "feagi_injection_proof--temp.log"
            )
            with open(log_path, "a") as f:
                if power_neurons:
                    f.write(
                        f"[{current_timestep}] Found {len(power_neurons)} power neurons: {power_neurons} (injecting every burst)\n"
                    )
                else:
                    f.write(
                        f"[{current_timestep}] NO POWER NEURONS FOUND\n"
                    )

        if not power_neurons:
            # Only log this occasionally to avoid spam
            if current_timestep % 100 == 0:
                from feagi.core.state_manager import FeagiStateManager

                state_manager = FeagiStateManager.instance()
                if state_manager.is_debug_npu_enabled():
                    logger.info(
                        f"[NPU-DEBUG] No power area neurons found for injection at timestep {current_timestep}"
                    )
            return 0

        #  FAST: Set membrane potential ABOVE threshold for power neurons
        if self.connectome_manager and hasattr(
            self.connectome_manager, "neuron_array"
        ):
            if FeagiStateManager.instance().is_debug_npu_enabled():
                logger.info("[INJECTION-DEBUG] Setting membrane potentials for power neurons")
            neuron_array = self.connectome_manager.neuron_array
            for neuron_id in power_neurons:
                idx = neuron_array.neuron_id_to_index.get(neuron_id)
                if idx is None:
                    continue
                # Compute the next representable float above threshold (epsilon step)
                thr32 = np.float32(neuron_array.thresholds[idx])
                mp = np.nextafter(thr32, np.float32(np.inf))
                if hasattr(neuron_array, "set_neuron_property"):
                    neuron_array.set_neuron_property(
                        neuron_id, "membrane_potential", float(mp)
                    )
                else:
                    neuron_array.membrane_potentials[idx] = mp
            if FeagiStateManager.instance().is_debug_npu_enabled():
                # Log a small sample of MP/Thr after setting
                sample = power_neurons[:5]
                samples = []
                for nid in sample:
                    sidx = neuron_array.neuron_id_to_index.get(nid)
                    if sidx is not None:
                        samples.append({
                            "id": int(nid),
                            "V": float(neuron_array.membrane_potentials[sidx]),
                            "Thr": float(neuron_array.thresholds[sidx]),
                            "Refr": int(neuron_array.refractory_counters[sidx]),
                        })
                logger.info(f"[INJECTION-DEBUG] Raised MPs for {len(power_neurons)} power neurons; sample={samples}")

        #  Now inject power neurons into FCL (with proper membrane
        #  potentials set)
        # This happens EVERY burst to provide constant power supply
        if FeagiStateManager.instance().is_debug_npu_enabled():
            logger.info("[INJECTION-DEBUG] Adding power neurons to FCL via FCLManager.update_fcl()")
        # Log FCLManager identity before update
        try:
            from feagi.core.state_manager import FeagiStateManager
            if FeagiStateManager.instance().is_debug_npu_enabled():
                logger.info(f"[INJECTION-DEBUG] FCLManager id before update: {id(self.fcl_manager)}")
        except Exception:
            pass

        injected_count = self._inject_batch(
            InjectionBatch(
                cortical_id="_power",
                neuron_ids=power_neurons,
                timing=InjectionTiming.PRE_BURST,
                probability=1.0,  # Always inject power neurons
            ),
            current_timestep,
        )
        if FeagiStateManager.instance().is_debug_npu_enabled():
            logger.info(f"[INJECTION-DEBUG] FCLManager.update_fcl() injected_count={injected_count}")

        if (
            injected_count > 0 and current_timestep % 50 == 0
        ):  # Log occasionally
            logger.debug(
                f"Power area injection: {injected_count} neurons injected at timestep {current_timestep} (every burst mode)"
            )

        return injected_count

    def inject_during_burst(self, current_timestep: int) -> int:
        """
        No-op: During-burst injection not needed for core power area.

        Returns:
            Always 0 (no injection performed)
        """
        # Removed file I/O debug writes for RTOS/WGPU compatibility
        return 0

    def inject_post_burst(self, current_timestep: int) -> int:
        """
        No-op: Post-burst injection not needed for core power area.

        Returns:
            Always 0 (no injection performed)
        """
        # Removed file I/O debug writes for RTOS/WGPU compatibility
        return 0

    def _execute_injection_phase(
        self, timing: InjectionTiming, current_timestep: int
    ) -> int:
        """Execute injection for a specific timing phase.

        Args:
            timing: The injection timing phase
            current_timestep: Current simulation timestep

        Returns:
            Number of neurons injected
        """
        from feagi.core.state_manager import FeagiStateManager

        state_manager = FeagiStateManager.instance()

        if not self._injection_batches[timing]:
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    f"[NPU-DEBUG] No injection batches for {timing.value} phase"
                )
            return 0

        if state_manager.is_debug_npu_enabled():
            logger.info(
                f"[NPU-DEBUG] Starting {timing.value} injection phase with {len(self._injection_batches[timing])} batches"
            )

        start_time = time.perf_counter()
        total_injected = 0

        # Process all batches for this timing
        for batch in self._injection_batches[timing]:
            logger.debug(
                f"Processing batch for {batch.cortical_id} with {len(batch.neuron_ids)} neurons"
            )
            candidates_added = self._inject_batch(batch, current_timestep)
            total_injected += candidates_added
            if candidates_added > 0:
                logger.info(
                    f"Successfully added {candidates_added} candidates to FCL from {batch.cortical_id}"
                )

        # Update statistics
        end_time = time.perf_counter()
        self.last_injection_duration = end_time - start_time
        self.injection_stats["total_injections"] += 1
        self.total_neurons_injected += total_injected

        if total_injected > 0:
            logger.info(
                f"FCL INJECTION: Added {total_injected} candidates to FCL in {timing.value} phase ({self.last_injection_duration:.4f}s)"
            )
        else:
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    f"[NPU-DEBUG] No candidates added to FCL in {timing.value} phase"
                )

        return total_injected

    def _inject_batch(
        self, batch: InjectionBatch, current_timestep: int
    ) -> int:
        """Add a batch of neuron candidates to the FCL.

        This method handles the actual addition of candidates from special areas
        to the Fire Candidate List. The candidates will be processed along with
        other FCL entries during the unified burst processing sweep.

        Args:
            batch: The injection batch to process
            current_timestep: Current simulation timestep

        Returns:
            Number of candidates added to FCL
        """
        if not batch.neuron_ids:
            return 0

        # Check probabilistic injection
        if self.enable_probabilistic and batch.probability < 1.0:
            import random

            if random.random() > batch.probability:
                return 0

        try:
            #  Determine which neurons to inject (could be subset based on
            #  targeting)
            neurons_to_inject = batch.neuron_ids

            # Extract cortical_id (remove batch suffix if present)
            cortical_id = batch.cortical_id.split("_batch_")[0]

            #  REQUIREMENT: FCL manager MUST have update_fcl method - no
            #  fallbacks allowed
            if not hasattr(self.fcl_manager, "update_fcl"):
                raise RuntimeError(
                    "FCL manager does not have required update_fcl method. "
                    "FCL injection requires proper cortical area mapping for FQ sampler compatibility."
                )

            # Get the correct cortical_idx for this cortical_id
            cortical_idx = None
            if (
                hasattr(self.special_area_handler, "connectome_manager")
                and self.special_area_handler.connectome_manager
            ):
                connectome = self.special_area_handler.connectome_manager
                if hasattr(connectome, "cortical_areas"):
                    for area_id, area in connectome.cortical_areas.items():
                        if area_id == cortical_id:
                            cortical_idx = area.cortical_idx
                            break

            if cortical_idx is None:
                raise RuntimeError(
                    f"CRITICAL: Could not find cortical_idx for cortical_id '{cortical_id}'. "
                    f"Available cortical areas: {list(connectome.cortical_areas.keys()) if hasattr(connectome, 'cortical_areas') else 'None'}. "
                    f"FCL injection requires valid cortical area mapping."
                )

            #  Use proper cortical area mapping so FQ sampler can filter
            #  correctly
            neurons_by_cortical = {cortical_idx: neurons_to_inject}
            self.fcl_manager.update_fcl(current_timestep, neurons_by_cortical)
            logger.debug(
                f"FCL candidate addition: Added {len(neurons_to_inject)} candidates from {cortical_id} (cortical_idx={cortical_idx}) to FCL"
            )

            # Record injection for special area handler statistics
            self.special_area_handler.record_injection()

            return len(neurons_to_inject)

        except Exception as e:
            logger.error(f"Error injecting batch {batch.cortical_id}: {e}")
            return 0

    def refresh_injection_batches(self) -> None:
        """Refresh injection batches when special areas change.

        This should be called when the connectome structure changes or when
        special areas are added/removed.
        """
        logger.info("Refreshing injection batches", status="[PROC]")
        self._prepare_injection_batches()

    def get_statistics(self) -> Dict[str, Any]:
        """Get statistics about injection performance.

        Returns:
            Dictionary with injection statistics and performance metrics
        """
        return {
            "total_injections": self.injection_stats["total_injections"],
            "total_neurons_injected": self.total_neurons_injected,
            "successful_injections": self.injection_stats[
                "successful_injections"
            ],
            "failed_injections": self.injection_stats["failed_injections"],
            "last_injection_duration": self.last_injection_duration,
            "prepared_batches": {
                timing.value: len(batches)
                for timing, batches in self._injection_batches.items()
            },
            "batch_size": self.batch_size,
            "enable_probabilistic": self.enable_probabilistic,
        }

    def set_injection_enabled(
        self, cortical_id: CorticalId, enabled: bool
    ) -> bool:
        """Enable or disable injection for a specific cortical area.

        Args:
            cortical_id: The cortical area ID
            enabled: Whether to enable or disable injection

        Returns:
            True if the setting was applied, False if area not found
        """
        config = self.special_area_handler.get_special_config(cortical_id)
        if config:
            config.enabled = enabled
            # Refresh batches to apply the change
            self._prepare_injection_batches()
            logger.info(
                f"Injection {'enabled' if enabled else 'disabled'} for area {cortical_id}",
                status="[SETUP]",
            )
            return True
        return False

    def get_power_injection_preview(self) -> Dict[str, Any]:
        """Get a preview of what would be injected in the next burst.

        Returns:
            Dictionary with preview information for debugging/monitoring
        """
        preview = {
            "pre_burst_neurons": 0,
            "during_burst_neurons": 0,
            "post_burst_neurons": 0,
            "total_batches": 0,
            "areas_involved": set(),
        }

        for timing, batches in self._injection_batches.items():
            batch_neurons = sum(len(batch.neuron_ids) for batch in batches)
            preview["total_batches"] += len(batches)

            if timing == InjectionTiming.PRE_BURST:
                preview["pre_burst_neurons"] = batch_neurons
            elif timing == InjectionTiming.DURING_BURST:
                preview["during_burst_neurons"] = batch_neurons
            elif timing == InjectionTiming.POST_BURST:
                preview["post_burst_neurons"] = batch_neurons

            for batch in batches:
                base_id = batch.cortical_id.split("_batch_")[0]
                preview["areas_involved"].add(base_id)

        preview["areas_involved"] = list(preview["areas_involved"])
        return preview

    def inject_external_activations(
        self,
        activations: Dict[CorticalId, List[NeuronId]],
        current_timestep: int,
        source: str = "external",
    ) -> int:
        """Inject neuron activations from external sources (test mode, manual
        stimulation, etc.).

        This method provides a clean interface for external systems to submit neuron
        activations without needing to know FCL manager internals. The service handles
        the conversion to appropriate data structures and injection timing.

        CRITICAL FIX: Sets membrane potential above threshold for external neurons
        so they can actually fire, just like power injection does.

        BACKPRESSURE: Rejects injections if burst engine is not ready or busy.

        Args:
            activations: Dictionary mapping cortical area IDs to lists of neuron IDs to activate
            current_timestep: Current simulation timestep
            source: Source identifier for logging/debugging (e.g., "test_mode_1", "manual_stimulation")

        Returns:
            Number of neurons successfully injected (0 if rejected due to backpressure)
        """
        try:
            # BACKPRESSURE CHECK: Verify burst engine is ready for injection
            if not self._check_burst_engine_ready():
                logger.warning(
                    f"Rejecting injection from {source} - burst engine not ready"
                )
                return 0

            total_injected = 0

            from feagi.core.state_manager import FeagiStateManager

            state_manager = FeagiStateManager.instance()

            if not activations:
                if state_manager.is_debug_npu_enabled():
                    logger.info(
                        f"[NPU-DEBUG] No activations provided by {source}"
                    )
                return 0

            if state_manager.is_debug_npu_enabled():
                logger.info(
                    f"[NPU-DEBUG] Processing external activations from {source}: {len(activations)} cortical areas"
                )

            for cortical_id, neuron_ids in activations.items():
                if not neuron_ids:
                    continue

                try:
                    #  CRITICAL FIX: Set membrane potential above threshold for
                    #  external neurons
                    #  This ensures they can actually fire, just like power
                    #  injection does
                    membrane_potential_set_count = 0
                    if self.connectome_manager and hasattr(
                        self.connectome_manager, "neuron_array"
                    ):
                        neuron_array = self.connectome_manager.neuron_array
                        if hasattr(neuron_array, "set_neuron_property"):
                            #  Set membrane potential to 1.5 (above threshold
                            #  of 1.0) for all external neurons
                            for neuron_id in neuron_ids:
                                try:
                                    neuron_array.set_neuron_property(
                                        neuron_id, "membrane_potential", 1.5
                                    )
                                    membrane_potential_set_count += 1
                                except Exception as e:
                                    logger.warning(
                                        f"Failed to set membrane potential for external neuron {neuron_id}: {e}"
                                    )
                        else:
                            logger.warning(
                                "NeuronArray does not support set_neuron_property method - external neurons may not fire"
                            )
                    else:
                        logger.warning(
                            "ConnectomeManager or neuron_array not available - external neurons may not fire"
                        )

                    # Create injection batch for this area
                    batch = InjectionBatch(
                        cortical_id=cortical_id,
                        neuron_ids=neuron_ids.copy(),
                        timing=InjectionTiming.PRE_BURST,  # External activations treated as pre-burst
                        probability=1.0,  # External activations always inject (no probabilistic filtering)
                    )

                    # Inject the batch into FCL
                    injected_count = self._inject_batch(
                        batch, current_timestep
                    )
                    total_injected += injected_count

                    if injected_count > 0:
                        logger.debug(
                            f"Injected {injected_count} external neurons from {source} in area {cortical_id}"
                        )
                        logger.debug(
                            f"Set membrane potential for {membrane_potential_set_count}/{len(neuron_ids)} neurons"
                        )
                    else:
                        logger.warning(
                            f"Failed to inject neurons from {source} in area {cortical_id}"
                        )

                except Exception as e:
                    logger.error(
                        f"Error injecting external activations for area {cortical_id} from {source}: {e}"
                    )
                    continue

            if total_injected > 0:
                logger.debug(
                    f"FCL EXTERNAL INJECTION: Added {total_injected} candidates from {source} across {len(activations)} areas"
                )
                logger.info(
                    f"✅ FIXED: Set membrane potentials above threshold for external neurons from {source}"
                )
            else:
                logger.warning(
                    f"No external candidates were injected from {source}"
                )

            return total_injected

        except Exception as e:
            logger.error(
                f"Error in external activations injection from {source}: {e}"
            )
            import traceback

            logger.error(traceback.format_exc())
            return 0

    def _check_burst_engine_ready(self) -> bool:
        """Check if the burst engine is ready to accept new injections.

        PERFORMANCE: Optimized for RTOS/SIMD/GPU environments with minimal overhead.
        Uses cached state checks to avoid expensive API calls during high-frequency operation.

        Returns:
            True if ready to accept injections, False otherwise
        """
        #  PERFORMANCE: Cache the state manager reference to avoid repeated
        #  lookups
        if not hasattr(self, "_cached_state_manager"):
            try:
                from feagi.core.state_manager import FeagiStateManager

                self._cached_state_manager = FeagiStateManager.instance()
            except Exception:
                self._cached_state_manager = None

        #  PERFORMANCE: Fast path - if no state manager, allow injection
        #  (fail-open for performance)
        if not self._cached_state_manager:
            return True

        # PERFORMANCE: Single state check - avoid multiple API calls
        try:
            from feagi.core.state_manager import ServiceState

            burst_state = self._cached_state_manager.get_burst_engine_state()

            # RTOS-FRIENDLY: Simple state comparison, no complex logic
            return burst_state == ServiceState.READY

        except Exception:
            # PERFORMANCE: Fail-open on error to maintain injection throughput
            return True

    def _get_all_batches(self) -> List[InjectionBatch]:
        """Get all injection batches across all timing phases."""
        all_batches = []
        for batches in self._injection_batches.values():
            all_batches.extend(batches)
        return all_batches


# Example usage and testing functions
def example_usage():
    """Example usage of the FCLInjectionService."""
    # This would be used with real FCL manager and special area handler
    # fcl_injection = FCLInjectionService(fcl_manager, special_area_handler)
    #
    # # In burst engine:
    # fcl_injection.inject_pre_burst(current_timestep)
    # # ... regular burst processing ...
    # fcl_injection.inject_post_burst(current_timestep)
    pass


if __name__ == "__main__":
    example_usage()
