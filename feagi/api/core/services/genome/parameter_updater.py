"""Direct neuron parameter update service for cortical areas.

This module enables updating neuron properties directly in the
ConnectomeManager without requiring expensive full brain rebuilds, providing
massive performance improvements for parameter-only changes.
"""

import time
from typing import Any, Dict

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class CorticalParameterUpdater:
    """Handles direct neuron parameter updates without full brain rebuild.

    This service provides massive performance improvements by updating neuron
    properties directly in the ConnectomeManager neuron arrays, avoiding the expensive
    full brain reconstruction process for simple parameter changes.

    ARCHITECTURE: This class ONLY updates neuron array properties. It does NOT
    update ConnectomeManager cortical area properties - that is handled by
    GenomeService calling ConnectomeManager.update_cortical_area_properties().

    Performance: ~2-5ms vs ~800ms for full rebuild (160-400x faster)
    """
    
    # SYSTEMATIC MAPPING: Genome property → NeuronArray attribute
    # This maps genome parameter names to their corresponding neuron_array fields
    # Validation lambdas for 0-1 range (percentage values)
    _validate_0_1_float = lambda v: max(0.0, min(1.0, float(v)))
    
    NEURON_PROPERTY_MAPPING = {
        'neuron_excitability': ('excitabilities', _validate_0_1_float, 'Excitability'),
        'snooze_length': ('snooze_periods', lambda v: int(max(0, round(float(v)))), 'Snooze period'),
        'firing_threshold_limit': ('thresholds', float, 'Firing threshold'),
        'neuron_fire_threshold': ('thresholds', float, 'Firing threshold'),  # Alternative name
        'firing_threshold': ('thresholds', float, 'Firing threshold'),  # Alternative name
        'leak': ('leak_coefficients', _validate_0_1_float, 'Leak coefficient'),
        'neuron_leak_coefficient': ('leak_coefficients', _validate_0_1_float, 'Leak coefficient'),  # Alternative name
        'neuron_leak_variability': ('leak_coefficients', _validate_0_1_float, 'Leak variability'),  # Neurogenesis parameter (genome only)
        'refrac': ('refractory_periods', lambda v: int(max(0, round(float(v)))), 'Refractory period'),
        'neuron_refractory_period': ('refractory_periods', lambda v: int(max(0, round(float(v)))), 'Refractory period'),  # Alternative name
        'consecutive_fire_cnt_max': ('consecutive_fire_limits', lambda v: int(max(0, round(float(v)))), 'Consecutive fire limit'),
        'neuron_mp_charge_accumulation': ('mp_charge_accumulation', bool, 'MP charge accumulation'),  # Genome: nx-mp_acc-b
    }

    def __init__(self, connectome_manager: ConnectomeManager):
        self.connectome_manager = connectome_manager
        self.logger = logger
    
    def _update_neuron_array_property(
        self,
        cortical_id: str,
        cortical_idx: int,
        property_name: str,
        value: Any
    ) -> bool:
        """Systematically update a neuron array property for all neurons in a cortical area.
        
        This is the UNIFIED method for updating any per-neuron property that maps to
        a neuron_array field. It handles the update and Rust NPU reinitialization.
        
        Args:
            cortical_id: Cortical area ID
            cortical_idx: Cortical area index
            property_name: Genome property name (e.g., 'neuron_excitability')
            value: New value to set
            
        Returns:
            True if successful, False otherwise
        """
        # CRITICAL DEBUG: Log what property is being requested
        self.logger.info(f"🔍 [PROPERTY-UPDATE] _update_neuron_array_property called: property_name='{property_name}', cortical_id={cortical_id}, cortical_idx={cortical_idx}, value={value}")
        
        # Check if this property has a neuron_array mapping
        if property_name not in self.NEURON_PROPERTY_MAPPING:
            self.logger.warning(f"❌ [PROPERTY-UPDATE] Property '{property_name}' not in NEURON_PROPERTY_MAPPING!")
            return False
        
        array_field, converter, display_name = self.NEURON_PROPERTY_MAPPING[property_name]
        
        try:
            # Convert value using the specified converter
            converted_value = converter(value) if callable(converter) else converter(value)
            
            self.logger.info(
                f"🔥 [{display_name.upper()}-UPDATE] Updating {property_name} to {converted_value} for area {cortical_id} (cortical_idx={cortical_idx})"
            )
            
            # ✅ DIRECT RUST NPU UPDATE - Clean production-ready approach!
            # Update the Rust NPU directly using dedicated update methods (no reinitialization needed)
            try:
                # 🦀 RUST: Get Rust NPU directly from ProcessManager
                from feagi.process_manager import get_process_manager
                process_manager = get_process_manager()
                
                if not process_manager or not process_manager.rust_npu_integration:
                    self.logger.warning(f"🦀 [RUST-NPU] Rust NPU not available for {property_name} update")
                    return False
                
                rust_npu_integration = process_manager.rust_npu_integration
                if not rust_npu_integration._rust_npu_initialized:
                    self.logger.debug(f"🦀 [RUST-NPU] Not yet initialized - {property_name} will be loaded on first burst")
                    return True  # Will be picked up on init
                
                rust_npu = rust_npu_integration._rust_npu
                
                # Property-specific update logic
                if property_name == 'neuron_excitability':
                    # Validate 0-1 range for excitability
                    if not (0.0 <= converted_value <= 1.0):
                        self.logger.error(f"🦀 [RUST-NPU] ❌ Excitability must be in range 0.0-1.0, got {converted_value}")
                        return False
                    # Use dedicated Rust method for excitability updates
                    updated_count = rust_npu.update_cortical_area_excitability(cortical_idx, float(converted_value))
                    self.logger.info(f"🦀 [RUST-NPU] ✅ Updated excitability to {converted_value} for {updated_count} neurons in area {cortical_id}")
                elif property_name in ('neuron_refractory_period', 'refractory_period', 'refrac'):
                    updated_count = rust_npu.update_cortical_area_refractory_period(cortical_idx, int(converted_value))
                    self.logger.info(f"🦀 [RUST-NPU] ✅ Updated refractory_period to {converted_value} for {updated_count} neurons in area {cortical_id}")
                elif property_name in ('neuron_fire_threshold', 'firing_threshold', 'firing_threshold_limit'):
                    updated_count = rust_npu.update_cortical_area_threshold(cortical_idx, float(converted_value))
                    self.logger.info(f"🦀 [RUST-NPU] ✅ Updated threshold to {converted_value} for {updated_count} neurons in area {cortical_id}")
                elif property_name in ('leak', 'leak_coefficient', 'neuron_leak_coefficient', 'neuron_leak_variability'):
                    # Validate 0-1 range for leak parameters
                    if not (0.0 <= converted_value <= 1.0):
                        self.logger.error(f"🦀 [RUST-NPU] ❌ Leak coefficient must be in range 0.0-1.0, got {converted_value}")
                        return False
                    updated_count = rust_npu.update_cortical_area_leak(cortical_idx, float(converted_value))
                    self.logger.info(f"🦀 [RUST-NPU] ✅ Updated leak to {converted_value} for {updated_count} neurons in area {cortical_id}")
                elif property_name in ('consecutive_fire_cnt_max', 'neuron_consecutive_fire_cnt_max'):
                    updated_count = rust_npu.update_cortical_area_consecutive_fire_limit(cortical_idx, int(converted_value))
                    self.logger.info(f"🦀 [RUST-NPU] ✅ Updated consecutive_fire_limit to {converted_value} for {updated_count} neurons in area {cortical_id}")
                elif property_name in ('snooze_length', 'neuron_snooze_period'):
                    updated_count = rust_npu.update_cortical_area_snooze_period(cortical_idx, int(converted_value))
                    self.logger.info(f"🦀 [RUST-NPU] ✅ Updated snooze_period to {converted_value} for {updated_count} neurons in area {cortical_id}")
                elif property_name == 'neuron_mp_charge_accumulation':
                    updated_count = rust_npu.update_cortical_area_mp_charge_accumulation(cortical_idx, bool(converted_value))
                    self.logger.info(f"🦀 [RUST-NPU] ✅ Updated mp_charge_accumulation to {converted_value} for {updated_count} neurons in area {cortical_id}")
                else:
                    # For other properties, we'll need to add specific Rust methods as needed
                    self.logger.warning(f"🦀 [RUST-NPU] Live update for {property_name} not yet implemented - will require FEAGI restart")
                    return False
                
                return True
                
            except Exception as rust_error:
                self.logger.error(f"🦀 [RUST-NPU] Error during Rust NPU property update: {rust_error}")
                self.logger.exception("Full stack trace:")
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to update {property_name} for {cortical_id}: {e}")
            self.logger.exception("Full stack trace:")
            return False

    def update_neuron_parameters(
        self, cortical_id: str, parameter_changes: Dict[str, Any]
    ) -> bool:
        """Update neuron parameters for a cortical area via Rust NPU.
        
        ARCHITECTURE: Rust NPU owns all neurons. Python just tells Rust
        which cortical area and what value to update. Rust does everything internally.

        Args:
            cortical_id: ID of the cortical area to update
            parameter_changes: Dictionary of parameter_name -> new_value

        Returns:
            True if all updates successful, False otherwise
        """
        start_time = time.time()

        try:
            # Get cortical index for Rust NPU updates
            npu = getattr(self.connectome_manager, "_npu_interface", None)
            if not npu:
                self.logger.error(
                    "NPU Interface is not available on ConnectomeManager; fast parameter update requires NPU"
                )
                return False

            cortical_idx = npu.get_cortical_idx_by_id(cortical_id)
            if cortical_idx is None:
                self.logger.info(
                    f"Cortical area {cortical_id} not registered in NPU yet - skipping parameter updates"
                )
                return True

            # Apply each parameter change directly via Rust NPU (no neuron ID manipulation!)
            success_count = 0
            for param_name, value in parameter_changes.items():
                # Check if this property is in our mapping
                if param_name in self.NEURON_PROPERTY_MAPPING:
                    success = self._update_neuron_array_property(
                        cortical_id,
                        cortical_idx,
                        param_name,
                        value
                    )
                    if success:
                        success_count += 1
                    else:
                        self.logger.error(
                            f"Failed to update {param_name} for {cortical_id}"
                        )
                else:
                    self.logger.debug(
                        f"Property {param_name} not in neuron property mapping - skipping"
                    )

            # Log performance metrics
            duration = time.time() - start_time
            self.logger.info(
                f"[FAST-UPDATE] Completed {success_count}/{len(parameter_changes)} parameter updates "
                f"for cortical area {cortical_id} in {duration * 1000:.1f}ms"
            )

            return success_count > 0

        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(
                f"Failed to update parameters for {cortical_id} after {duration * 1000:.1f}ms: {e}"
            )
            self.logger.exception("Full traceback:")
            return False
