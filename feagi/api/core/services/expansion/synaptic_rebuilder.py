"""Synaptic Rebuilder for Cortical Area Dimensional Changes.

This module handles complete synaptic reconstruction when cortical area 
dimensions change (expansion, contraction, or reshaping). It deletes all 
existing synapses and rebuilds them from scratch using morphology definitions.
"""

import random
from typing import Any, Dict, List, Optional, Set, Tuple

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class SynapticRebuilder:
    """Rebuilds all synaptic connections for cortical areas after dimensional changes.

    ARCHITECTURE: When cortical area dimensions change, we delete ALL existing 
    synapses (incoming, outgoing, recurrent) and rebuild them completely from 
    morphology definitions. This ensures consistent connectivity regardless of 
    whether the area expanded, contracted, or was reshaped.
    """

    def __init__(self, connectome_manager, state_manager):
        """Initialize SynapticRebuilder.

        Args:
            connectome_manager: Reference to ConnectomeManager
            state_manager: Reference to FeagiStateManager
        """
        self.connectome_manager = connectome_manager
        self.state_manager = state_manager
        self.logger = logger

    def rebuild_all_connectivity(
        self,
        cortical_id: str,
        old_dimensions: Tuple[int, int, int],
        new_dimensions: Tuple[int, int, int],
    ) -> int:
        """Rebuild ALL synaptic connections for a cortical area after dimensional change.

        CRITICAL: This method deletes ALL existing synapses (incoming, outgoing, recurrent)
        and rebuilds them completely from morphology definitions. This approach works for
        expansion, contraction, or reshaping.

        Args:
            cortical_id: ID of the cortical area that changed dimensions
            old_dimensions: Previous dimensions of the area  
            new_dimensions: New dimensions after the change

        Returns:
            Number of synapses created after complete rebuild
        """
        self.logger.info(
            f"🔄 [SYNAPTIC-REBUILD] Starting complete synaptic rebuild for {cortical_id}"
        )
        self.logger.info(
            f"🔄 [SYNAPTIC-REBUILD] Dimensions: {old_dimensions} → {new_dimensions}"
        )

        # Count initial synapses for comparison
        initial_synapse_count = self.connectome_manager.get_synapse_count()
        
        # Step 1: Delete ALL existing synapses involving this cortical area
        synapses_deleted = self._delete_all_synapses_for_area(cortical_id)
        
        # Step 2: Rebuild ALL synapses from scratch using morphology definitions
        synapses_created = self._rebuild_synapses_from_morphologies(cortical_id)
        
        # Verify final synapse count
        final_synapse_count = self.connectome_manager.get_synapse_count()
        net_change = final_synapse_count - initial_synapse_count
        
        self.logger.info(
            f"🔄 [SYNAPTIC-REBUILD] Complete rebuild for {cortical_id}:"
        )
        self.logger.info(
            f"   Deleted: {synapses_deleted} existing synapses"
        )
        self.logger.info(
            f"   Created: {synapses_created} new synapses"
        )
        self.logger.info(
            f"   Net change: {net_change:+d} synapses"
        )
        
        return synapses_created

    def _delete_all_synapses_for_area(self, cortical_id: str) -> int:
        """Delete all synapses involving the specified cortical area.
        
        This includes:
        - Incoming synapses (from other areas to this area)
        - Outgoing synapses (from this area to other areas) 
        - Recurrent synapses (within this area)
        
        Args:
            cortical_id: ID of the cortical area
            
        Returns:
            Number of synapses deleted
        """
        try:
            # Get all neurons in this cortical area
            area_neurons = self.connectome_manager.get_neurons_by_area(cortical_id)
            if not area_neurons:
                self.logger.warning(
                    f"🔄 [SYNAPTIC-REBUILD] No neurons found in area {cortical_id} - no synapses to delete"
                )
                return 0
            
            area_neuron_set = set(area_neurons)
            synapses_deleted = 0
            
            self.logger.info(
                f"🔄 [SYNAPTIC-REBUILD] Deleting all synapses for {len(area_neurons)} neurons in {cortical_id}"
            )
            
            # Get all synapses and delete ones involving neurons in this area
            synapses_to_delete = []
            
            # CRITICAL FIX: Direct synapse deletion using SynapseArray
            synapse_array = self.connectome_manager.synapse_array
            
            # Find all synapses involving neurons in this cortical area
            for i in range(synapse_array.count):
                if not synapse_array.valid_mask[i]:
                    continue
                    
                source_id = synapse_array.source_neuron_ids[i]
                target_id = synapse_array.target_neuron_ids[i]
                
                # Delete synapse if EITHER source OR target is in the cortical area
                if source_id in area_neuron_set or target_id in area_neuron_set:
                    synapses_to_delete.append((source_id, target_id))
            
            # Use the correct method name: delete_synapses (not delete_synapses_batch)
            if synapses_to_delete:
                synapses_deleted = self.connectome_manager.delete_synapses(synapses_to_delete)
                self.logger.info(
                    f"🔄 [SYNAPTIC-REBUILD] Successfully identified {len(synapses_to_delete)} synapses to delete"
                )
            else:
                synapses_deleted = 0
                self.logger.info(
                    f"🔄 [SYNAPTIC-REBUILD] No synapses found involving area {cortical_id}"
                )
            
            self.logger.info(
                f"🔄 [SYNAPTIC-REBUILD] Deleted {synapses_deleted} synapses for area {cortical_id}"
            )
            
            return synapses_deleted
            
        except Exception as e:
            self.logger.error(
                f"🔄 [SYNAPTIC-REBUILD] Error deleting synapses for {cortical_id}: {e}"
            )
            return 0

    def _delete_synapses_fallback(self, area_neuron_set: set) -> int:
        """Fallback method - this is no longer needed as we now have proper deletion.
        
        Args:
            area_neuron_set: Set of neuron IDs in the cortical area
            
        Returns:
            Number of synapses deleted (always 0 for this fallback)
        """
        self.logger.info(
            f"🔄 [SYNAPTIC-REBUILD] Fallback method called - this should not happen with the new implementation"
        )
        return 0

    def _rebuild_synapses_from_morphologies(self, cortical_id: str) -> int:
        """Rebuild all synapses for a cortical area using morphology definitions.
        
        This method finds all cortical mappings involving the area and applies
        the morphology rules to create synapses from scratch.
        
        Args:
            cortical_id: ID of the cortical area
            
        Returns:
            Number of synapses created
        """
        try:
            total_synapses_created = 0
            
            # Find all mappings involving this cortical area from the hierarchical genome
            all_mappings = self._find_all_mappings_for_area(cortical_id)
            
            self.logger.info(
                f"🔄 [SYNAPTIC-REBUILD] Found {len(all_mappings)} mappings for {cortical_id}"
            )
            
            if not all_mappings:
                self.logger.warning(
                    f"🔄 [SYNAPTIC-REBUILD] No mappings found for {cortical_id} - no synapses to rebuild"
                )
                return 0
            
            # Rebuild synapses for each mapping
            for i, mapping in enumerate(all_mappings):
                self.logger.info(
                    f"🔄 [SYNAPTIC-REBUILD] Processing mapping {i + 1}/{len(all_mappings)}: {mapping['source']} → {mapping['destination']}"
                )
                
                synapses_created = self._apply_morphology_mapping(
                    mapping['source'],
                    mapping['destination'], 
                    mapping['morphology'],
                    mapping.get('morphology_parameters', {})
                )
                
                total_synapses_created += synapses_created
                
                self.logger.info(
                    f"🔄 [SYNAPTIC-REBUILD] Created {synapses_created} synapses for mapping {mapping['source']} → {mapping['destination']}"
                )
            
            return total_synapses_created
            
        except Exception as e:
            self.logger.error(
                f"🔄 [SYNAPTIC-REBUILD] Error rebuilding synapses for {cortical_id}: {e}"
            )
            import traceback
            self.logger.error(
                f"🔄 [SYNAPTIC-REBUILD] Traceback: {traceback.format_exc()}"
            )
            return 0

    def _find_all_mappings_for_area(self, cortical_id: str) -> List[Dict[str, Any]]:
        """Find all cortical mappings involving the specified cortical area.
        
        Args:
            cortical_id: ID of the cortical area
            
        Returns:
            List of mappings involving this area
        """
        try:
            # CRITICAL FIX: Enhanced genome access with multiple fallback strategies
            genome = None
            
            # Strategy 1: Try state_manager.genome
            if hasattr(self.state_manager, 'genome') and self.state_manager.genome:
                genome = self.state_manager.genome
                self.logger.info(f"🔄 [SYNAPTIC-REBUILD] Using state_manager.genome")
            
            # Strategy 2: Try connectome_manager's genome access
            elif hasattr(self.connectome_manager, '_current_genome') and self.connectome_manager._current_genome:
                genome = self.connectome_manager._current_genome
                self.logger.info(f"🔄 [SYNAPTIC-REBUILD] Using connectome_manager._current_genome")
                
            # Strategy 3: Try alternative genome access via state_manager methods
            elif hasattr(self.state_manager, 'get_genome'):
                try:
                    genome = self.state_manager.get_genome()
                    self.logger.info(f"🔄 [SYNAPTIC-REBUILD] Using state_manager.get_genome()")
                except:
                    pass
            
            if not genome or not isinstance(genome, dict) or "blueprint" not in genome:
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] CRITICAL ERROR: No accessible genome found for {cortical_id}"
                )
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] Genome type: {type(genome)}, has blueprint: {'blueprint' in genome if isinstance(genome, dict) else False}"
                )
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] This will result in 0 synapses being created!"
                )
                return []

            mappings = []
            blueprint = genome["blueprint"]
            
            self.logger.info(
                f"🔄 [SYNAPTIC-REBUILD] Successfully accessed genome blueprint with {len(blueprint)} areas"
            )

            # Search through all areas in the genome to find mappings
            for area_id, area_data in blueprint.items():
                if not isinstance(area_data, dict):
                    continue
                    
                # Check if this area has mappings TO our target cortical_id
                cortical_mapping_dst = area_data.get("cortical_mapping_dst", {})
                if isinstance(cortical_mapping_dst, dict):
                    for dst_area, dst_mappings in cortical_mapping_dst.items():
                        if dst_area == cortical_id:
                            # Found mapping: area_id -> cortical_id
                            if isinstance(dst_mappings, list):
                                for mapping_spec in dst_mappings:
                                    # CRITICAL FIX: Handle DICTIONARY format (not list)
                                    if isinstance(mapping_spec, dict):
                                        morphology_id = mapping_spec.get('morphology_id', '')
                                        if morphology_id:
                                            # Extract ALL parameters from the dictionary
                                            morphology_parameters = {
                                                'morphology_scalar': mapping_spec.get('morphology_scalar', [1, 1, 1]),
                                                'postSynapticCurrent_multiplier': mapping_spec.get('postSynapticCurrent_multiplier', 1.0),
                                                'plasticity_flag': mapping_spec.get('plasticity_flag', False),
                                                'plasticity_constant': mapping_spec.get('plasticity_constant', 1.0),
                                                'ltp_multiplier': mapping_spec.get('ltp_multiplier', 1.0),
                                                'ltd_multiplier': mapping_spec.get('ltd_multiplier', 1.0),
                                            }
                                            mappings.append({
                                                'source': area_id,
                                                'destination': cortical_id,
                                                'morphology': morphology_id,
                                                'morphology_parameters': morphology_parameters
                                            })
                                    # LEGACY SUPPORT: Handle old LIST format if it exists
                                    elif isinstance(mapping_spec, list) and len(mapping_spec) > 0:
                                        morphology_id = mapping_spec[0]
                                        mappings.append({
                                            'source': area_id,
                                            'destination': cortical_id,
                                            'morphology': morphology_id,
                                            'morphology_parameters': {}  # Limited parameters for legacy format
                                        })
                
                # Check if our target cortical_id has mappings FROM this area
                if area_id == cortical_id and cortical_mapping_dst:
                    for dst_area, dst_mappings in cortical_mapping_dst.items():
                        if isinstance(dst_mappings, list):
                            for mapping_spec in dst_mappings:
                                # CRITICAL FIX: Handle DICTIONARY format (not list) - OUTGOING MAPPINGS
                                if isinstance(mapping_spec, dict):
                                    morphology_id = mapping_spec.get('morphology_id', '')
                                    if morphology_id:
                                        # Extract ALL parameters from the dictionary
                                        morphology_parameters = {
                                            'morphology_scalar': mapping_spec.get('morphology_scalar', [1, 1, 1]),
                                            'postSynapticCurrent_multiplier': mapping_spec.get('postSynapticCurrent_multiplier', 1.0),
                                            'plasticity_flag': mapping_spec.get('plasticity_flag', False),
                                            'plasticity_constant': mapping_spec.get('plasticity_constant', 1.0),
                                            'ltp_multiplier': mapping_spec.get('ltp_multiplier', 1.0),
                                            'ltd_multiplier': mapping_spec.get('ltd_multiplier', 1.0),
                                        }
                                        mappings.append({
                                            'source': cortical_id,
                                            'destination': dst_area,
                                            'morphology': morphology_id,
                                            'morphology_parameters': morphology_parameters
                                        })
                                # LEGACY SUPPORT: Handle old LIST format if it exists
                                elif isinstance(mapping_spec, list) and len(mapping_spec) > 0:
                                    morphology_id = mapping_spec[0]
                                    mappings.append({
                                        'source': cortical_id,
                                        'destination': dst_area,
                                        'morphology': morphology_id,
                                        'morphology_parameters': {}  # Limited parameters for legacy format
                                    })

            self.logger.info(
                f"🔄 [SYNAPTIC-REBUILD] Found {len(mappings)} mappings for {cortical_id} in genome"
            )
            
            # ENHANCED DEBUGGING: Log detailed mapping information
            if mappings:
                self.logger.info(f"🔄 [SYNAPTIC-REBUILD] Mappings for {cortical_id}:")
                for i, mapping in enumerate(mappings):
                    params = mapping['morphology_parameters']
                    self.logger.info(
                        f"   {i+1}. {mapping['source']} → {mapping['destination']} ({mapping['morphology']})"
                    )
                    if params:
                        self.logger.info(
                            f"       Params: scalar={params.get('morphology_scalar', 'N/A')}, "
                            f"psc_mult={params.get('postSynapticCurrent_multiplier', 'N/A')}, "
                            f"plasticity={params.get('plasticity_flag', 'N/A')}"
                        )
                    else:
                        self.logger.warning(f"       ❌ No parameters extracted (legacy format?)")
            else:
                self.logger.warning(
                    f"🔄 [SYNAPTIC-REBUILD] NO MAPPINGS FOUND for {cortical_id}!"
                )
                self.logger.warning(
                    f"🔄 [SYNAPTIC-REBUILD] This means NO synapses will be created during rebuild"
                )
                
                # Debug: Show some blueprint structure to help identify the issue
                if blueprint:
                    sample_areas = list(blueprint.keys())[:5]
                    self.logger.info(f"🔄 [SYNAPTIC-REBUILD] Sample areas in blueprint: {sample_areas}")
                    
                    # Check if the target area exists in blueprint
                    if cortical_id in blueprint:
                        area_data = blueprint[cortical_id]
                        has_outgoing = 'cortical_mapping_dst' in area_data
                        self.logger.info(f"🔄 [SYNAPTIC-REBUILD] Area {cortical_id} exists in blueprint, has outgoing mappings: {has_outgoing}")
                    else:
                        self.logger.warning(f"🔄 [SYNAPTIC-REBUILD] Area {cortical_id} NOT found in blueprint!")
            
            return mappings

        except Exception as e:
            self.logger.error(
                f"🔄 [SYNAPTIC-REBUILD] Error finding mappings for {cortical_id}: {e}"
            )
            return []

    def _apply_morphology_mapping(self, src_area_id: str, dst_area_id: str, morphology_id: str, morphology_parameters: Dict[str, Any]) -> int:
        """Apply a morphology mapping to create synapses between two cortical areas.
        
        CRITICAL FIX: Use the EXACT same morphology logic as NeuroEmbryogenesis
        to ensure consistent connectivity patterns.
        
        Args:
            src_area_id: Source cortical area ID
            dst_area_id: Destination cortical area ID
            morphology_id: Morphology to apply
            morphology_parameters: Additional morphology parameters
            
        Returns:
            Number of synapses created
        """
        try:
            # Get source and destination neurons
            src_neurons = self.connectome_manager.get_neurons_by_area(src_area_id)
            dst_neurons = self.connectome_manager.get_neurons_by_area(dst_area_id)
            
            if not src_neurons or not dst_neurons:
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] CRITICAL: Missing neurons for morphology {morphology_id}"
                )
                self.logger.error(
                    f"   Source area {src_area_id}: {len(src_neurons) if src_neurons else 0} neurons"
                )
                self.logger.error(
                    f"   Destination area {dst_area_id}: {len(dst_neurons) if dst_neurons else 0} neurons"
                )
                self.logger.error(
                    f"   Cannot create synapses without both source and destination neurons!"
                )
                return 0
            
            self.logger.info(
                f"🔄 [SYNAPTIC-REBUILD] Applying {morphology_id}: {src_area_id}({len(src_neurons)}) → {dst_area_id}({len(dst_neurons)})"
            )
            
            # CRITICAL FIX: Use NeuroEmbryogenesis morphology application logic
            # This ensures the same connectivity patterns as original creation
            from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
            
            # Create a temporary NeuroEmbryogenesis instance with current genome
            embryogenesis = NeuroEmbryogenesis(self.connectome_manager, self.state_manager)
            
            # CRITICAL FIX: Properly load genome data like the API path does
            # This ensures morphology definitions and all genome structures are available
            # Use the same fallback strategy as _find_all_mappings_for_area
            current_genome = None
            
            # Strategy 1: Try state_manager.genome
            if hasattr(self.state_manager, 'genome') and self.state_manager.genome:
                current_genome = self.state_manager.genome
                self.logger.info(f"🔄 [SYNAPTIC-REBUILD] Using state_manager.genome for NeuroEmbryogenesis")
            
            # Strategy 2: Try connectome_manager's genome access
            elif hasattr(self.connectome_manager, '_current_genome') and self.connectome_manager._current_genome:
                current_genome = self.connectome_manager._current_genome
                self.logger.info(f"🔄 [SYNAPTIC-REBUILD] Using connectome_manager._current_genome for NeuroEmbryogenesis")
            
            if not current_genome:
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] CRITICAL ERROR: No genome available for NeuroEmbryogenesis initialization"
                )
                return 0
            if not embryogenesis._load_genome_data(current_genome):
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] CRITICAL ERROR: Failed to load genome data into NeuroEmbryogenesis"
                )
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] This will cause morphology application to fail or behave differently"
                )
                return 0
            
            self.logger.info(
                f"🔄 [SYNAPTIC-REBUILD] Successfully loaded genome data into NeuroEmbryogenesis"
            )
            
            # VERIFICATION: Ensure morphology definitions are available
            if not hasattr(embryogenesis, 'genome') or not embryogenesis.genome:
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] ERROR: NeuroEmbryogenesis genome not properly set after loading"
                )
                return 0
                
            if 'neuron_morphologies' not in embryogenesis.genome:
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] ERROR: No neuron_morphologies in loaded genome"
                )
                return 0
                
            if morphology_id not in embryogenesis.genome['neuron_morphologies']:
                available_morphologies = list(embryogenesis.genome['neuron_morphologies'].keys())
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] ERROR: Morphology '{morphology_id}' not found in loaded genome"
                )
                self.logger.error(
                    f"🔄 [SYNAPTIC-REBUILD] Available morphologies: {available_morphologies[:10]}..."
                )
                return 0
                
            self.logger.info(
                f"🔄 [SYNAPTIC-REBUILD] Verified morphology '{morphology_id}' is available in loaded genome"
            )
            
            # Extract morphology parameters (with defaults)
            morphology_scalar = morphology_parameters.get('morphology_scalar', [1, 1, 1])
            psc_multiplier = morphology_parameters.get('postSynapticCurrent_multiplier', 1.0)
            plasticity_flag = morphology_parameters.get('plasticity_flag', False)
            plasticity_constant = morphology_parameters.get('plasticity_constant', 1.0)
            ltp_multiplier = morphology_parameters.get('ltp_multiplier', 1.0)
            ltd_multiplier = morphology_parameters.get('ltd_multiplier', 1.0)
            
            # DIAGNOSTIC: Log EXACT parameters being passed to _apply_morphology_mapping
            self.logger.info(
                f"🔄 [SYNAPTIC-REBUILD-DIAGNOSTIC] EXACT PARAMETERS for _apply_morphology_mapping:"
            )
            self.logger.info(f"   src_area_id: {src_area_id}")
            self.logger.info(f"   dst_area_id: {dst_area_id}")
            self.logger.info(f"   src_neurons count: {len(src_neurons)}")
            self.logger.info(f"   dst_neurons count: {len(dst_neurons)}")
            self.logger.info(f"   src_neurons sample: {sorted(src_neurons)[:5] if src_neurons else []}")
            self.logger.info(f"   dst_neurons sample: {sorted(dst_neurons)[:5] if dst_neurons else []}")
            self.logger.info(f"   morphology_id: {morphology_id}")
            self.logger.info(f"   morphology_scalar: {morphology_scalar}")
            self.logger.info(f"   psc_multiplier: {psc_multiplier}")
            self.logger.info(f"   plasticity_flag: {plasticity_flag}")
            self.logger.info(f"   plasticity_constant: {plasticity_constant}")
            self.logger.info(f"   ltp_multiplier: {ltp_multiplier}")
            self.logger.info(f"   ltd_multiplier: {ltd_multiplier}")
            
            # Use the EXACT same morphology application logic as original creation
            synapses_created = embryogenesis._apply_morphology_mapping(
                src_area_id=src_area_id,
                dst_area_id=dst_area_id,
                src_neurons=src_neurons,
                dst_neurons=dst_neurons,
                morphology_id=morphology_id,
                morphology_scalar=morphology_scalar,
                psc_multiplier=psc_multiplier,
                plasticity_flag=plasticity_flag,
                plasticity_constant=plasticity_constant,
                ltp_multiplier=ltp_multiplier,
                ltd_multiplier=ltd_multiplier,
            )
            
            self.logger.info(
                f"🔄 [SYNAPTIC-REBUILD] Created {synapses_created} synapses using NeuroEmbryogenesis morphology logic"
            )
            
            return synapses_created
            
        except Exception as e:
            self.logger.error(
                f"🔄 [SYNAPTIC-REBUILD] Error applying morphology {morphology_id}: {e}"
            )
            import traceback
            self.logger.error(
                f"🔄 [SYNAPTIC-REBUILD] Traceback: {traceback.format_exc()}"
            )
            return 0
