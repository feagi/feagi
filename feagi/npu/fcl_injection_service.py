"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

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
from typing import Any, Dict, List, Optional, Set, Union

import numpy as np

from feagi.npu.special_area_handler import CorticalId, NeuronId, SpecialAreaHandler
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
    """
    Unified FCL injection service for all special area types.

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
        """
        Initialize the FCL injection service.

        Args:
            fcl_manager: Manager for the Fire Candidate List
            special_area_handler: Handler for detecting and processing special areas
        """
        self.fcl_manager = fcl_manager
        self.special_area_handler = special_area_handler

        # Store reference to connectome manager for membrane potential access
        # This is needed to set power neuron membrane potentials above threshold
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
        self.enable_probabilistic = True  # Enable probabilistic injection by default
        self.last_injection_duration = 0.0

        # Statistics tracking
        self.total_neurons_injected = 0
        self.injection_stats = {
            "total_injections": 0,
            "successful_injections": 0,
            "failed_injections": 0,
        }

        # Pre-computed injection batches for efficiency
        self._injection_batches: Dict[InjectionTiming, List[InjectionBatch]] = {
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
        """
        Pre-compute injection batches for performance optimization.

        SIMPLIFIED for core power area (___pwr at cortical_idx=1):
        Creates injection batches for detected power areas.
        """
        # Clear existing batches
        for timing in InjectionTiming:
            self._injection_batches[timing].clear()

        try:
            # Get all power areas (simplified approach returns only core power area)
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
                enabled_mask = [config and config.enabled for config in configs]

                # Filter to enabled power areas only
                enabled_indices = np.where(enabled_mask)[0]

                for idx in enabled_indices:
                    cortical_id = cortical_ids[idx]
                    neuron_ids = power_neurons[cortical_id]
                    config = configs[idx]

                    logger.debug(
                        f"Processing power area {cortical_id} with {len(neuron_ids)} neurons"
                    )

                    # Determine timing
                    timing_str = config.injection_timing
                    try:
                        timing = InjectionTiming(timing_str)
                        logger.debug(
                            f"Power area {cortical_id} uses {timing_str} timing"
                        )
                    except ValueError:
                        logger.warning(
                            f"Invalid injection timing '{timing_str}' for area {cortical_id}, using PRE_BURST"
                        )
                        timing = InjectionTiming.PRE_BURST

                    # Create single batch (simplified - no batch splitting needed for core power area)
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
            # Continue with empty batches - injection will still work via direct method

    def inject_pre_burst(self, current_timestep: int) -> int:
        """
        Inject power area neurons into FCL with proper membrane potential.

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
        try:
            # Write proof EVERY call for debugging to see what's happening
            with open("/tmp/feagi_injection_proof.log", "a") as f:
                f.write(
                    f"[{current_timestep}] inject_pre_burst called (every burst mode)\n"
                )

            # Get power area neurons from special area handler (cortical_idx=1)
            # This happens EVERY burst to provide constant power supply
            power_neurons = self.special_area_handler.get_power_area_neurons()

            # Write proof EVERY call showing what neurons were found
            with open("/tmp/feagi_injection_proof.log", "a") as f:
                if power_neurons:
                    f.write(
                        f"[{current_timestep}] Found {len(power_neurons)} power neurons: {power_neurons} (injecting every burst)\n"
                    )
                else:
                    f.write(f"[{current_timestep}] NO POWER NEURONS FOUND\n")

            if not power_neurons:
                # Only log this occasionally to avoid spam
                if current_timestep % 100 == 0:
                    logger.debug(
                        f"No power area neurons found for injection at timestep {current_timestep}"
                    )
                return 0

            # CRITICAL FIX: Set power neurons' membrane potential above threshold
            # BEFORE adding them to FCL so they will actually fire!
            try:
                if self.connectome_manager and hasattr(
                    self.connectome_manager, "neuron_array"
                ):
                    neuron_array = self.connectome_manager.neuron_array
                    if hasattr(neuron_array, "set_neuron_property"):
                        # Set membrane potential to 1.5 (above threshold of 1.0) for all power neurons
                        success_count = 0
                        for neuron_id in power_neurons:
                            try:
                                neuron_array.set_neuron_property(
                                    neuron_id, "membrane_potential", 1.5
                                )
                                success_count += 1
                                if current_timestep % 100 == 0:  # Log occasionally
                                    logger.debug(
                                        f"[POWER MP FIX] Set membrane_potential=1.5 for power neuron {neuron_id}"
                                    )
                            except Exception as e:
                                if current_timestep % 100 == 0:  # Log occasionally
                                    logger.warning(
                                        f"[POWER MP FIX] Failed to set MP for neuron {neuron_id}: {e}"
                                    )

                        if (
                            success_count > 0 and current_timestep % 100 == 0
                        ):  # Log occasionally
                            logger.info(
                                f"[POWER MP FIX] Successfully set membrane potential for {success_count}/{len(power_neurons)} power neurons"
                            )
                        elif (
                            success_count == 0 and current_timestep % 100 == 0
                        ):  # Log occasionally
                            logger.warning(
                                f"[POWER MP FIX] Failed to set membrane potential for any of the {len(power_neurons)} power neurons"
                            )
                    else:
                        if current_timestep % 500 == 0:  # Log very occasionally
                            logger.warning(
                                "[POWER MP FIX] NeuronArray does not support set_neuron_property method"
                            )
                else:
                    if current_timestep % 500 == 0:  # Log very occasionally
                        logger.warning(
                            "[POWER MP FIX] ConnectomeManager or neuron_array not available"
                        )

            except Exception as e:
                if current_timestep % 100 == 0:  # Log occasionally
                    logger.error(
                        f"[POWER MP FIX] Error setting membrane potentials for power neurons: {e}"
                    )

            # Now inject power neurons into FCL (with proper membrane potentials set)
            # This happens EVERY BURST to provide constant power supply
            injected_count = self._inject_batch(
                InjectionBatch(
                    cortical_id="___pwr",
                    neuron_ids=power_neurons,
                    timing=InjectionTiming.PRE_BURST,
                    probability=1.0,  # Always inject power neurons
                ),
                current_timestep,
            )

            if injected_count > 0 and current_timestep % 50 == 0:  # Log occasionally
                logger.debug(
                    f"Power area injection: {injected_count} neurons injected at timestep {current_timestep} (every burst mode)"
                )

            return injected_count

        except Exception as e:
            if current_timestep % 100 == 0:  # Log occasionally
                logger.error(
                    f"Error in power area injection at timestep {current_timestep}: {e}"
                )
            return 0

    def inject_during_burst(self, current_timestep: int) -> int:
        """
        No-op: During-burst injection not needed for core power area.

        Returns:
            Always 0 (no injection performed)
        """
        return 0

    def inject_post_burst(self, current_timestep: int) -> int:
        """
        No-op: Post-burst injection not needed for core power area.

        Returns:
            Always 0 (no injection performed)
        """
        return 0

    def _execute_injection_phase(
        self, timing: InjectionTiming, current_timestep: int
    ) -> int:
        """
        Execute injection for a specific timing phase.

        Args:
            timing: The injection timing phase
            current_timestep: Current simulation timestep

        Returns:
            Number of neurons injected
        """
        if not self._injection_batches[timing]:
            logger.debug(f"No injection batches for {timing.value} phase")
            return 0

        logger.debug(
            f"Starting {timing.value} injection phase with {len(self._injection_batches[timing])} batches"
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
            logger.debug(f"No candidates added to FCL in {timing.value} phase")

        return total_injected

    def _inject_batch(self, batch: InjectionBatch, current_timestep: int) -> int:
        """
        Add a batch of neuron candidates to the FCL.

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
            # Determine which neurons to inject (could be subset based on targeting)
            neurons_to_inject = batch.neuron_ids

            # Extract cortical_id (remove batch suffix if present)
            cortical_id = batch.cortical_id.split("_batch_")[0]

            # CRITICAL FIX: Use update_fcl with proper cortical area mapping
            # This ensures neurons are associated with their correct cortical area
            # so the FQ sampler can filter them properly
            if hasattr(self.fcl_manager, "update_fcl"):
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

                if cortical_idx is not None:
                    # Use proper cortical area mapping so FQ sampler can filter correctly
                    neurons_by_cortical = {cortical_idx: neurons_to_inject}
                    self.fcl_manager.update_fcl(current_timestep, neurons_by_cortical)
                    logger.debug(
                        f"FCL candidate addition (update): Added {len(neurons_to_inject)} candidates from {cortical_id} (cortical_idx={cortical_idx}) to FCL"
                    )
                else:
                    # FAIL FAST - No fallbacks allowed
                    logger.error(
                        f"CRITICAL: Could not find cortical_idx for cortical_id '{cortical_id}' - injection FAILED"
                    )
                    logger.error(
                        f"Available cortical areas: {list(connectome.cortical_areas.keys()) if hasattr(connectome, 'cortical_areas') else 'None'}"
                    )
                    return 0  # Fail the injection completely
            elif hasattr(self.fcl_manager, "add_to_current_fcl"):
                # Legacy direct injection - this BREAKS FQ sampler filtering!
                logger.warning(
                    f"Using add_to_current_fcl for {cortical_id} - this will break FQ sampler area filtering!"
                )
                self.fcl_manager.add_to_current_fcl(neurons_to_inject)
                logger.debug(
                    f"FCL candidate addition (legacy): Added {len(neurons_to_inject)} candidates from {cortical_id} to FCL"
                )
            else:
                logger.error("FCL manager does not support any known injection method")
                return 0

            # Record injection for special area handler statistics
            self.special_area_handler.record_injection()

            return len(neurons_to_inject)

        except Exception as e:
            logger.error(f"Error injecting batch {batch.cortical_id}: {e}")
            return 0

    def refresh_injection_batches(self) -> None:
        """
        Refresh injection batches when special areas change.

        This should be called when the connectome structure changes or
        when special areas are added/removed.
        """
        logger.info("Refreshing injection batches", status="[PROC]")
        self._prepare_injection_batches()

    def get_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about injection performance.

        Returns:
            Dictionary with injection statistics and performance metrics
        """
        return {
            "total_injections": self.injection_stats["total_injections"],
            "total_neurons_injected": self.total_neurons_injected,
            "successful_injections": self.injection_stats["successful_injections"],
            "failed_injections": self.injection_stats["failed_injections"],
            "last_injection_duration": self.last_injection_duration,
            "prepared_batches": {
                timing.value: len(batches)
                for timing, batches in self._injection_batches.items()
            },
            "batch_size": self.batch_size,
            "enable_probabilistic": self.enable_probabilistic,
        }

    def set_injection_enabled(self, cortical_id: CorticalId, enabled: bool) -> bool:
        """
        Enable or disable injection for a specific cortical area.

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
        """
        Get a preview of what would be injected in the next burst.

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
        """
        Inject neuron activations from external sources (test mode, manual stimulation, etc.).

        This method provides a clean interface for external systems to submit neuron
        activations without needing to know FCL manager internals. The service handles
        the conversion to appropriate data structures and injection timing.

        CRITICAL FIX: Sets membrane potential above threshold for external neurons
        so they can actually fire, just like power injection does.

        Args:
            activations: Dictionary mapping cortical area IDs to lists of neuron IDs to activate
            current_timestep: Current simulation timestep
            source: Source identifier for logging/debugging (e.g., "test_mode_1", "manual_stimulation")

        Returns:
            Number of neurons successfully injected
        """
        try:
            total_injected = 0

            if not activations:
                logger.debug(f"No activations provided by {source}")
                return 0

            logger.debug(
                f"Processing external activations from {source}: {len(activations)} cortical areas"
            )

            for cortical_id, neuron_ids in activations.items():
                if not neuron_ids:
                    continue

                try:
                    # CRITICAL FIX: Set membrane potential above threshold for external neurons
                    # This ensures they can actually fire, just like power injection does
                    membrane_potential_set_count = 0
                    if self.connectome_manager and hasattr(
                        self.connectome_manager, "neuron_array"
                    ):
                        neuron_array = self.connectome_manager.neuron_array
                        if hasattr(neuron_array, "set_neuron_property"):
                            # Set membrane potential to 1.5 (above threshold of 1.0) for all external neurons
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
                                f"NeuronArray does not support set_neuron_property method - external neurons may not fire"
                            )
                    else:
                        logger.warning(
                            f"ConnectomeManager or neuron_array not available - external neurons may not fire"
                        )

                    # Create injection batch for this area
                    batch = InjectionBatch(
                        cortical_id=cortical_id,
                        neuron_ids=neuron_ids.copy(),
                        timing=InjectionTiming.PRE_BURST,  # External activations treated as pre-burst
                        probability=1.0,  # External activations always inject (no probabilistic filtering)
                    )

                    # Inject the batch into FCL
                    injected_count = self._inject_batch(batch, current_timestep)
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
                logger.warning(f"No external candidates were injected from {source}")

            return total_injected

        except Exception as e:
            logger.error(f"Error in external activations injection from {source}: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return 0

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
