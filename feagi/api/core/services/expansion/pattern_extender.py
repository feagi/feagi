"""
Pattern Extender for Cortical Area Expansion

This module handles the intelligent extension of existing synaptic patterns
to newly created neurons during cortical area expansion.
"""

from typing import Dict, Any, List, Tuple, Set, Optional
from feagi.utils.logger import setup_logger
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis

logger = setup_logger(__name__)


class PatternExtender:
    """
    Extends existing synaptic patterns to newly created neurons during expansion.
    
    This class analyzes existing connectivity patterns in a cortical area and 
    applies the same morphology rules to newly created neurons to maintain
    consistent connectivity across the expanded region.
    """
    
    def __init__(self, connectome_manager, state_manager):
        """
        Initialize PatternExtender.
        
        Args:
            connectome_manager: Reference to ConnectomeManager
            state_manager: Reference to FeagiStateManager
        """
        self.connectome_manager = connectome_manager
        self.state_manager = state_manager
        self.logger = logger
        
    def extend_patterns_for_expansion(
        self, 
        cortical_id: str, 
        old_dimensions: Tuple[int, int, int],
        new_dimensions: Tuple[int, int, int],
        new_neurons: Set[int]
    ) -> int:
        """
        Extend existing connectivity patterns to newly created neurons.
        
        Args:
            cortical_id: ID of the expanded cortical area
            old_dimensions: Previous dimensions of the area
            new_dimensions: New dimensions after expansion
            new_neurons: Set of newly created neuron IDs
            
        Returns:
            Number of new synapses created
        """
        self.logger.info(f"🔍 [PATTERN-EXTEND] Starting pattern extension for {cortical_id}")
        self.logger.info(f"🔍 [PATTERN-EXTEND] Dimensions: {old_dimensions} → {new_dimensions}")
        self.logger.info(f"🔍 [PATTERN-EXTEND] Extending patterns to {len(new_neurons)} new neurons")
        self.logger.info(f"🔍 [PATTERN-EXTEND] New neuron IDs: {sorted(list(new_neurons))[:10]}{'...' if len(new_neurons) > 10 else ''}")
        
        try:
            # Find existing cortical mappings for this area
            self.logger.info(f"🔍 [PATTERN-EXTEND] Searching for existing mappings...")
            existing_mappings = self._find_existing_mappings(cortical_id)
            
            self.logger.info(f"🔍 [PATTERN-EXTEND] Found {len(existing_mappings)} existing mappings")
            
            if not existing_mappings:
                self.logger.warning(f"🔍 [PATTERN-EXTEND] No existing mappings found for {cortical_id} - cannot extend patterns")
                return 0
                
            total_synapses_created = 0
            
            # Process each existing mapping to extend patterns
            for i, mapping in enumerate(existing_mappings):
                self.logger.info(f"🔍 [PATTERN-EXTEND] Processing mapping {i+1}/{len(existing_mappings)}")
                self.logger.info(f"🔍 [PATTERN-EXTEND]   - Source: {mapping.get('source', 'unknown')}")
                self.logger.info(f"🔍 [PATTERN-EXTEND]   - Destination: {mapping.get('destination', 'unknown')}")
                self.logger.info(f"🔍 [PATTERN-EXTEND]   - Morphology: {mapping.get('morphology', 'unknown')}")
                
                synapses_created = self._extend_mapping_pattern(
                    cortical_id=cortical_id,
                    mapping=mapping,
                    new_neurons=new_neurons,
                    new_dimensions=new_dimensions
                )
                total_synapses_created += synapses_created
                
                self.logger.info(f"🔍 [PATTERN-EXTEND] Created {synapses_created} synapses for this mapping")
                
            self.logger.info(f"[PATTERN-EXTEND] Created {total_synapses_created} new synapses for {cortical_id}")
            return total_synapses_created
            
        except Exception as e:
            self.logger.error(f"[PATTERN-EXTEND] Error extending patterns for {cortical_id}: {e}")
            return 0
    
    def _find_existing_mappings(self, cortical_id: str) -> List[Dict[str, Any]]:
        """
        Find existing cortical mappings involving the specified area from hierarchical genome.
        
        Args:
            cortical_id: ID of the cortical area
            
        Returns:
            List of cortical mappings that involve this area
        """
        try:
            genome = self.state_manager.genome
            if not genome or "blueprint" not in genome:
                self.logger.warning(f"[PATTERN-EXTEND] No hierarchical genome found")
                return []
                
            existing_mappings = []
            blueprint = genome["blueprint"]
            
            self.logger.info(f"[PATTERN-EXTEND] Searching hierarchical genome for mappings involving {cortical_id}")
            
            # 1. Find outgoing mappings (this area -> others)
            if cortical_id in blueprint:
                area_data = blueprint[cortical_id]
                if isinstance(area_data, dict) and "cortical_mapping_dst" in area_data:
                    cortical_mappings = area_data["cortical_mapping_dst"]
                    if isinstance(cortical_mappings, dict):
                        for dst_area, connection_specs in cortical_mappings.items():
                            if isinstance(connection_specs, list):
                                for spec in connection_specs:
                                    # CRITICAL FIX: Handle both dict and array formats
                                    if isinstance(spec, dict):
                                        # Dict format: {"morphology_id": "lateral_+x", ...}
                                        mapping = {
                                            "source": cortical_id,
                                            "destination": dst_area,
                                            "morphology": spec.get("morphology_id"),
                                            "morphology_parameters": spec
                                        }
                                        existing_mappings.append(mapping)
                                        self.logger.info(f"[PATTERN-EXTEND] Found outgoing mapping (dict): {cortical_id} -> {dst_area} using {spec.get('morphology_id')}")
                                    elif isinstance(spec, list) and len(spec) >= 1:
                                        # Array format: ["lateral_+x", [1, 1, 1], 1.0, False, 1, 1, 1]
                                        # Convert to dictionary format that the rest of the code expects
                                        morphology_id = spec[0] if len(spec) > 0 else None
                                        morphology_dict = {
                                            "morphology_id": morphology_id
                                        }
                                        
                                        # Convert array elements to dictionary format
                                        if len(spec) > 1:
                                            morphology_dict["morphology_scalar"] = spec[1]
                                        if len(spec) > 2:
                                            morphology_dict["postSynapticCurrent_multiplier"] = spec[2]
                                        if len(spec) > 3:
                                            morphology_dict["plasticity_flag"] = spec[3]
                                        
                                        mapping = {
                                            "source": cortical_id,
                                            "destination": dst_area,
                                            "morphology": morphology_id,
                                            "morphology_parameters": morphology_dict  # Use converted dict
                                        }
                                        existing_mappings.append(mapping)
                                        self.logger.info(f"[PATTERN-EXTEND] Found outgoing mapping (array): {cortical_id} -> {dst_area} using {morphology_id}")
                                    else:
                                        self.logger.warning(f"[PATTERN-EXTEND] Unknown outgoing spec format: {type(spec)} - {spec}")
            
            # 2. Find incoming mappings (others -> this area)
            for source_area_id, area_data in blueprint.items():
                if source_area_id != cortical_id and isinstance(area_data, dict):
                    if "cortical_mapping_dst" in area_data:
                        cortical_mappings = area_data["cortical_mapping_dst"]
                        if isinstance(cortical_mappings, dict) and cortical_id in cortical_mappings:
                            connection_specs = cortical_mappings[cortical_id]
                            if isinstance(connection_specs, list):
                                for spec in connection_specs:
                                    # CRITICAL FIX: Handle both dict and array formats
                                    if isinstance(spec, dict):
                                        # Dict format: {"morphology_id": "lateral_+x", ...}
                                        mapping = {
                                            "source": source_area_id,
                                            "destination": cortical_id,
                                            "morphology": spec.get("morphology_id"),
                                            "morphology_parameters": spec
                                        }
                                        existing_mappings.append(mapping)
                                        self.logger.info(f"[PATTERN-EXTEND] Found incoming mapping (dict): {source_area_id} -> {cortical_id} using {spec.get('morphology_id')}")
                                    elif isinstance(spec, list) and len(spec) >= 1:
                                        # Array format: ["lateral_+x", [1, 1, 1], 1.0, False, 1, 1, 1]
                                        morphology_id = spec[0] if len(spec) > 0 else None
                                        morphology_dict = {
                                            "morphology_id": morphology_id
                                        }
                                        
                                        # Convert array elements to dictionary format
                                        if len(spec) > 1:
                                            morphology_dict["morphology_scalar"] = spec[1]
                                        if len(spec) > 2:
                                            morphology_dict["postSynapticCurrent_multiplier"] = spec[2]
                                        if len(spec) > 3:
                                            morphology_dict["plasticity_flag"] = spec[3]
                                        
                                        mapping = {
                                            "source": source_area_id,
                                            "destination": cortical_id,
                                            "morphology": morphology_id,
                                            "morphology_parameters": morphology_dict  # Use converted dict
                                        }
                                        existing_mappings.append(mapping)
                                        self.logger.info(f"[PATTERN-EXTEND] Found incoming mapping (array): {source_area_id} -> {cortical_id} using {morphology_id}")
                                    else:
                                        self.logger.warning(f"[PATTERN-EXTEND] Unknown incoming spec format: {type(spec)} - {spec}")
                        
            self.logger.info(f"[PATTERN-EXTEND] Found {len(existing_mappings)} existing mappings for {cortical_id}")
            for i, mapping in enumerate(existing_mappings):
                self.logger.info(f"[PATTERN-EXTEND]   Mapping {i+1}: {mapping['source']} -> {mapping['destination']} ({mapping['morphology']})")
            
            return existing_mappings
            
        except Exception as e:
            self.logger.error(f"[PATTERN-EXTEND] Error finding mappings for {cortical_id}: {e}")
            import traceback
            self.logger.error(f"[PATTERN-EXTEND] Traceback: {traceback.format_exc()}")
            return []
    
    def _extend_mapping_pattern(
        self,
        cortical_id: str,
        mapping: Dict[str, Any], 
        new_neurons: Set[int],
        new_dimensions: Tuple[int, int, int]
    ) -> int:
        """
        Extend a specific mapping pattern to new neurons.
        
        Args:
            cortical_id: ID of the cortical area being expanded
            mapping: The cortical mapping to extend
            new_neurons: Set of newly created neuron IDs
            new_dimensions: New dimensions of the expanded area
            
        Returns:
            Number of synapses created for this mapping
        """
        try:
            source = mapping.get("source")
            destination = mapping.get("destination")
            morphology_id = mapping.get("morphology")
            
            if not all([source, destination, morphology_id]):
                self.logger.warning(f"[PATTERN-EXTEND] Incomplete mapping: {mapping}")
                return 0
                
            self.logger.info(f"[PATTERN-EXTEND] Extending mapping: {source} → {destination} using {morphology_id}")
            
            # Get morphology definition from genome
            morphology_def = self._get_morphology_definition(morphology_id)
            if not morphology_def:
                self.logger.warning(f"[PATTERN-EXTEND] Morphology {morphology_id} not found")
                return 0
                
            # Check if this is a dimension-sensitive morphology
            dimension_sensitive = morphology_def.get("dimension_sensitive", False)
            
            if dimension_sensitive:
                self.logger.info(f"[PATTERN-EXTEND] Morphology {morphology_id} is dimension-sensitive - full reconstruction needed")
                # For dimension-sensitive morphologies, full reconstruction is handled elsewhere
                return 0
            else:
                self.logger.info(f"[PATTERN-EXTEND] Morphology {morphology_id} is dimension-agnostic - extending pattern")
                return self._apply_morphology_to_new_neurons(
                    cortical_id=cortical_id,
                    mapping=mapping,
                    morphology_def=morphology_def,
                    new_neurons=new_neurons
                )
                
        except Exception as e:
            self.logger.error(f"[PATTERN-EXTEND] Error extending mapping pattern: {e}")
            return 0
    
    def _get_morphology_definition(self, morphology_id: str) -> Optional[Dict[str, Any]]:
        """
        Get morphology definition from genome.
        
        Args:
            morphology_id: ID of the morphology
            
        Returns:
            Morphology definition or None if not found
        """
        try:
            genome = self.state_manager.genome
            if not genome or "neuron_morphologies" not in genome:
                return None
                
            return genome["neuron_morphologies"].get(morphology_id)
            
        except Exception as e:
            self.logger.error(f"[PATTERN-EXTEND] Error getting morphology {morphology_id}: {e}")
            return None
    
    def _apply_morphology_to_new_neurons(
        self,
        cortical_id: str,
        mapping: Dict[str, Any],
        morphology_def: Dict[str, Any],
        new_neurons: Set[int]
    ) -> int:
        """
        Apply morphology rules to new neurons to create synapses.
        
        Args:
            cortical_id: ID of the cortical area
            mapping: The cortical mapping definition
            morphology_def: The morphology definition
            new_neurons: Set of newly created neuron IDs
            
        Returns:
            Number of synapses created
        """
        try:
            # Use NeuroEmbryogenesis to apply the morphology to new neurons
            from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
            neuro_embryo = NeuroEmbryogenesis(self.connectome_manager, self.state_manager)
            
            # Create the mapping in the format expected by NeuroEmbryogenesis
            # Format: {src_area_id: {dst_area_id: [connection_specs]}}
            connection_spec = {
                "morphology_id": mapping["morphology"],
                "morphology_scalar": [1, 1, 1],  # Default scalar
                "postSynapticCurrent_multiplier": 1.0,
                "plasticity_flag": False,
                "plasticity_constant": 1.0,
                "ltp_multiplier": 1.0,
                "ltd_multiplier": 1.0
            }
            
            # Add morphology parameters if available
            morphology_params = mapping.get("morphology_parameters", {})
            if morphology_params:
                connection_spec.update(morphology_params)
            
            # Create the properly formatted mapping
            formatted_mapping = {
                mapping["source"]: {
                    mapping["destination"]: [connection_spec]
                }
            }
            
            # Count synapses before
            initial_synapse_count = self.connectome_manager.get_synapse_count()
            
            # Apply morphology using the existing neuroembryogenesis logic
            # Filter to only process new neurons by temporarily modifying area neurons
            area = self.connectome_manager.cortical_areas.get(cortical_id)
            if not area:
                return 0

            # Temporarily store original neuron list and replace with new neurons only
            original_neurons = area._neuron_indices.copy()
            try:
                # Set area to contain only new neurons for pattern application
                area._neuron_indices = new_neurons
                
                # Apply the cortical mapping to create synapses for new neurons only
                embryogenesis = NeuroEmbryogenesis(self.connectome_manager, self.state_manager)
                
                # CRITICAL FIX: Ensure embryogenesis has the current genome with morphology definitions
                current_genome = self.state_manager.genome
                if current_genome:
                    embryogenesis.genome = current_genome
                    self.logger.info(f"[PATTERN-EXTEND] Set genome on embryogenesis, morphologies available: {len(current_genome.get('neuron_morphologies', {}))}")
                else:
                    self.logger.warning(f"[PATTERN-EXTEND] No genome available in state_manager")
                
                embryogenesis.update_cortical_mapping(formatted_mapping)
                
            finally:
                # Always restore original neuron list
                area._neuron_indices = original_neurons
            
            # Count synapses after
            final_synapse_count = self.connectome_manager.get_synapse_count()
            synapses_created = final_synapse_count - initial_synapse_count
                
            self.logger.info(f"[PATTERN-EXTEND] Applied {mapping['morphology']} to {len(new_neurons)} new neurons: {synapses_created} synapses created")
            return synapses_created
                
        except Exception as e:
            self.logger.error(f"[PATTERN-EXTEND] Error applying morphology to new neurons: {e}")
            import traceback
            self.logger.error(f"[PATTERN-EXTEND] Traceback: {traceback.format_exc()}")
            return 0 