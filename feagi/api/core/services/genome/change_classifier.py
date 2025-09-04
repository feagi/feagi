"""Classification system for cortical area changes to enable intelligent update
routing.

This module determines whether cortical area changes require:
- Full brain rebuild (structural changes)
- Direct neuron parameter updates (parameter changes)
- Metadata updates only (name, etc.)
- Hybrid approach (combination)
"""

from enum import Enum
from typing import Any, Dict, List, Tuple, Union

from feagi.bdu.connectome_manager import NeuronPropertyType
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class ChangeType(Enum):
    """Types of cortical area changes requiring different update strategies."""

    STRUCTURAL = "structural"  # Requires full rebuild
    PARAMETER = "parameter"  # Direct neuron updates
    METADATA = "metadata"  # Simple property updates
    HYBRID = "hybrid"  # Multiple types mixed


class CorticalChangeClassifier:
    """Classifies cortical area changes to route them to optimal update
    mechanisms.

    This enables major performance improvements by avoiding unnecessary full
    brain rebuilds for simple parameter or metadata changes.
    """

    #  Properties requiring full rebuild (affect neuron
    #  topology/count/connections)
    STRUCTURAL_CHANGES = {
        "cortical_dimensions",  # Changes neuron count
        "coordinates_3d",  # Spatial repositioning may affect connections
        "cortical_type",  # Functional role changes
        "cortical_mapping_dst",  # Connection topology
        "per_voxel_neuron_cnt",  # Neuron density changes
        "cortical_neuron_per_vox_count",  # Alternative neuron density parameter name
        "neuron_density",  # Another neuron density parameter name
        "neurons_per_voxel",  # Yet another neuron density parameter name
        "group_id",  # May affect area classification
        "sub_group_id",  # May affect area classification
    }

    # Simple metadata that can be updated without affecting neurons
    METADATA_CHANGES = {
        "cortical_name",  # Just a display name
    }

    # Parameters mappable to direct neuron property updates (avoiding rebuild)
    PARAMETER_TO_NEURON_PROPERTY = {
        # Genome param name → (NeuronPropertyType, conversion_func)
        "firing_threshold": (NeuronPropertyType.THRESHOLD, float),
        "neuron_fire_threshold": (
            NeuronPropertyType.THRESHOLD,
            float,
        ),  # API parameter name
        "refractory_period": (NeuronPropertyType.REFRACTORY_PERIOD, int),
        "neuron_refractory_period": (
            NeuronPropertyType.REFRACTORY_PERIOD,
            int,
        ),  # API parameter name
        "leak_coefficient": (NeuronPropertyType.DECAY_RATE, float),
        "neuron_leak_coefficient": (
            NeuronPropertyType.DECAY_RATE,
            float,
        ),  # API parameter name
        "consecutive_fire_cnt_max": (
            "consecutive_fire_count",
            int,
        ),  # Custom handling needed
        "neuron_consecutive_fire_count": (
            "consecutive_fire_count",
            int,
        ),  # API parameter name
        "firing_threshold_limit": (
            "firing_threshold_limit",
            float,
        ),  # Custom handling needed
        "neuron_firing_threshold_limit": (
            "firing_threshold_limit",
            float,
        ),  # API parameter name
        "snooze_length": ("snooze_length", float),  # Custom handling needed
        "neuron_snooze_period": ("snooze_length", float),  # API parameter name
        "degeneration": ("degeneration", float),  # Custom handling needed
        "neuron_degeneracy_coefficient": (
            "degeneration",
            float,
        ),  # API parameter name
        "postsynaptic_current": (
            "postsynaptic_current",
            float,
        ),  # Custom handling needed
        "postsynaptic_current_max": (
            "postsynaptic_current_max",
            float,
        ),  # Custom handling needed
        "neuron_excitability": (
            "neuron_excitability",
            float,
        ),  # Custom handling needed
        # Memory and lifespan parameters - should be fast updates
        "longterm_mem_threshold": ("longterm_mem_threshold", int),
        "neuron_longterm_mem_threshold": (
            "longterm_mem_threshold",
            int,
        ),  # API parameter name
        "lifespan_growth_rate": ("lifespan_growth_rate", int),
        "neuron_lifespan_growth_rate": (
            "lifespan_growth_rate",
            int,
        ),  # API parameter name
        "init_lifespan": ("init_lifespan", int),
        "neuron_init_lifespan": ("init_lifespan", int),  # API parameter name
        "temporal_depth": (
            "temporal_depth",
            int,
        ),  # Same name for API and genome
    }

    # Parameters that need special handling (require rebuild for now)
    SPECIAL_PARAMETERS = {
        "firing_threshold_increment_x",
        "neuron_fire_threshold_increment",  # API parameter name
        "firing_threshold_increment_y",
        "firing_threshold_increment_z",
        "leak_variability",
        "neuron_leak_variability",  # API parameter name
        "psp_uniform_distribution",
        "neuron_psp_uniform_distribution",  # API parameter name
        "mp_charge_accumulation",
        "mp_driven_psp",
        "is_mem_type",
        "dev_count",
        "synapse_attractivity",
        "visualization",
        "location_generation_type",
    }

    @classmethod
    def classify_changes(cls, changes: Dict[str, Any]) -> ChangeType:
        """Classify if changes are structural, parameter, metadata, or hybrid.

        Args:
            changes: Dictionary of field_name -> new_value changes

        Returns:
            ChangeType indicating the optimal update strategy
        """
        has_structural = any(
            key in cls.STRUCTURAL_CHANGES for key in changes.keys()
        )
        has_parameters = any(
            key in cls.PARAMETER_TO_NEURON_PROPERTY for key in changes.keys()
        )
        has_metadata = any(
            key in cls.METADATA_CHANGES for key in changes.keys()
        )
        has_special = any(
            key in cls.SPECIAL_PARAMETERS for key in changes.keys()
        )

        # Count change types
        change_count = sum(
            [has_structural, has_parameters, has_metadata, has_special]
        )

        if change_count > 1:
            return ChangeType.HYBRID
        elif has_structural or has_special:
            return ChangeType.STRUCTURAL  # Special params need rebuild for now
        elif has_parameters:
            return ChangeType.PARAMETER
        elif has_metadata:
            return ChangeType.METADATA
        else:
            # Unknown changes - be safe and rebuild
            logger.warning(
                f"Unknown change types detected: {list(changes.keys())}"
            )
            return ChangeType.STRUCTURAL

    @classmethod
    def separate_changes_by_type(
        cls, changes: Dict[str, Any]
    ) -> Dict[ChangeType, Dict[str, Any]]:
        """Separate changes into buckets by type for hybrid processing.

        Args:
            changes: Dictionary of all changes

        Returns:
            Dictionary mapping ChangeType to changes of that type
        """
        separated = {
            ChangeType.STRUCTURAL: {},
            ChangeType.PARAMETER: {},
            ChangeType.METADATA: {},
        }

        for key, value in changes.items():
            if key in cls.STRUCTURAL_CHANGES or key in cls.SPECIAL_PARAMETERS:
                separated[ChangeType.STRUCTURAL][key] = value
            elif key in cls.PARAMETER_TO_NEURON_PROPERTY:
                separated[ChangeType.PARAMETER][key] = value
            elif key in cls.METADATA_CHANGES:
                separated[ChangeType.METADATA][key] = value
            else:
                # Unknown - treat as structural to be safe
                separated[ChangeType.STRUCTURAL][key] = value

        return separated

    @classmethod
    def get_neuron_property_mappings(
        cls, parameter_changes: Dict[str, Any]
    ) -> List[Tuple[str, Any, Union[NeuronPropertyType, str], type]]:
        """Get neuron property mappings for parameter changes.

        Args:
            parameter_changes: Parameter changes to map

        Returns:
            List of (param_name, value, property_type, conversion_func) tuples
        """
        mappings = []

        for param_name, value in parameter_changes.items():
            if param_name in cls.PARAMETER_TO_NEURON_PROPERTY:
                property_type, conversion_func = (
                    cls.PARAMETER_TO_NEURON_PROPERTY[param_name]
                )
                mappings.append(
                    (param_name, value, property_type, conversion_func)
                )

        return mappings

    @classmethod
    def log_classification_result(
        cls, changes: Dict[str, Any], change_type: ChangeType
    ) -> None:
        """Log the classification result for debugging and monitoring."""
        change_summary = ", ".join(f"{k}={v}" for k, v in changes.items())
        logger.info(
            f"[CHANGE-CLASSIFIER] Type: {change_type.value} | Changes: {change_summary}"
        )

        if change_type == ChangeType.PARAMETER:
            logger.info(
                "[OPTIMIZATION] Fast parameter update path selected - avoiding full rebuild"
            )
        elif change_type == ChangeType.METADATA:
            logger.info(
                "[OPTIMIZATION] Metadata-only update - minimal processing required"
            )
