# Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#  ==============================================================================
"""
BDU Interfaces to NPU-owned data structures.

These interfaces provide controlled access for BDU to NPU-owned neuron and synapse arrays
during sleep periods when NPU is not actively processing. This enables BDU to perform
brain restructuring operations (neurogenesis, synaptogenesis) while maintaining NPU
as the primary owner of the data structures.

Key Design Principles:
- NPU is PRIMARY OWNER of neuron and synapse SoA arrays
- BDU gets CONTROLLED ACCESS through these interfaces
- Access is only granted during NPU sleep periods
- Interfaces provide BDU-compatible API while operating on NPU data
- Designed for Rust migration where NPU will be Rust-owned
"""

import logging
from typing import Dict, List, Optional, Tuple, Any

import numpy as np

logger = logging.getLogger(__name__)


class BDUNeuronInterface:
    """BDU interface to NPU-owned neuron data structures.
    
    This provides BDU with controlled access to NPU neuron arrays during sleep periods.
    The interface mimics the BDU NeuronArray API while operating on NPU-owned data.
    """
    
    def __init__(self, npu_neuron_array):
        """Initialize BDU neuron interface.
        
        Args:
            npu_neuron_array: NPU-owned neuron array
        """
        self.npu_neurons = npu_neuron_array
        logger.info("BDU neuron interface initialized")
    
    def add_neuron(self, neuron_id: int, properties: Dict[str, Any]) -> bool:
        """Add a new neuron to NPU-owned arrays.
        
        Args:
            neuron_id: Unique neuron identifier
            properties: Neuron properties (membrane_potential, threshold, etc.)
            
        Returns:
            True if successful
        """
        try:
            if neuron_id in self.npu_neurons.neuron_id_to_index:
                logger.warning(f"Neuron {neuron_id} already exists")
                return False
            
            if self.npu_neurons.neuron_count >= self.npu_neurons.max_neurons:
                logger.error("NPU neuron array is full")
                return False
            
            idx = self.npu_neurons.neuron_count
            
            # Set neuron properties in NPU arrays
            self.npu_neurons.membrane_potentials[idx] = properties.get('membrane_potential', 0.0)
            self.npu_neurons.thresholds[idx] = properties.get('threshold', 1.0)
            self.npu_neurons.decay_rates[idx] = properties.get('decay_rate', 0.9)
            self.npu_neurons.resting_potentials[idx] = properties.get('resting_potential', 0.0)
            self.npu_neurons.refractory_periods[idx] = properties.get('refractory_period', 1)
            self.npu_neurons.cortical_idxs[idx] = properties.get('cortical_idx', 0)
            self.npu_neurons.excitability[idx] = properties.get('excitability', 1.0)
            
            # Update NPU mappings
            self.npu_neurons.neuron_id_to_index[neuron_id] = idx
            self.npu_neurons.index_to_neuron_id[idx] = neuron_id
            self.npu_neurons.valid_mask[idx] = True
            
            self.npu_neurons.neuron_count += 1
            
            logger.debug(f"Added neuron {neuron_id} to NPU arrays at index {idx}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to add neuron {neuron_id}: {e}")
            return False
    
    def remove_neuron(self, neuron_id: int) -> bool:
        """Remove a neuron from NPU-owned arrays.
        
        Args:
            neuron_id: Neuron identifier to remove
            
        Returns:
            True if successful
        """
        try:
            if neuron_id not in self.npu_neurons.neuron_id_to_index:
                logger.warning(f"Neuron {neuron_id} not found")
                return False
            
            idx = self.npu_neurons.neuron_id_to_index[neuron_id]
            
            # Mark as invalid in NPU arrays
            self.npu_neurons.valid_mask[idx] = False
            
            # Remove from NPU mappings
            del self.npu_neurons.neuron_id_to_index[neuron_id]
            del self.npu_neurons.index_to_neuron_id[idx]
            
            logger.debug(f"Removed neuron {neuron_id} from NPU arrays")
            return True
            
        except Exception as e:
            logger.error(f"Failed to remove neuron {neuron_id}: {e}")
            return False
    
    def update_neuron_property(self, neuron_id: int, property_name: str, value: Any) -> bool:
        """Update a neuron property in NPU-owned arrays.
        
        Args:
            neuron_id: Neuron identifier
            property_name: Property to update
            value: New property value
            
        Returns:
            True if successful
        """
        try:
            if neuron_id not in self.npu_neurons.neuron_id_to_index:
                logger.warning(f"Neuron {neuron_id} not found")
                return False
            
            idx = self.npu_neurons.neuron_id_to_index[neuron_id]
            
            # Update property in NPU arrays
            if property_name == 'membrane_potential':
                self.npu_neurons.membrane_potentials[idx] = value
            elif property_name == 'threshold':
                self.npu_neurons.thresholds[idx] = value
            elif property_name == 'decay_rate':
                self.npu_neurons.decay_rates[idx] = value
            elif property_name == 'resting_potential':
                self.npu_neurons.resting_potentials[idx] = value
            elif property_name == 'refractory_period':
                self.npu_neurons.refractory_periods[idx] = value
            elif property_name == 'cortical_idx':
                self.npu_neurons.cortical_idxs[idx] = value
            elif property_name == 'excitability':
                self.npu_neurons.excitability[idx] = value
            else:
                logger.warning(f"Unknown property: {property_name}")
                return False
            
            logger.debug(f"Updated neuron {neuron_id} property {property_name} = {value}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to update neuron {neuron_id} property {property_name}: {e}")
            return False
    
    def get_neuron_properties(self, neuron_id: int) -> Optional[Dict[str, Any]]:
        """Get neuron properties from NPU-owned arrays.
        
        Args:
            neuron_id: Neuron identifier
            
        Returns:
            Dictionary of neuron properties or None if not found
        """
        try:
            if neuron_id not in self.npu_neurons.neuron_id_to_index:
                return None
            
            idx = self.npu_neurons.neuron_id_to_index[neuron_id]
            
            return {
                'membrane_potential': float(self.npu_neurons.membrane_potentials[idx]),
                'threshold': float(self.npu_neurons.thresholds[idx]),
                'decay_rate': float(self.npu_neurons.decay_rates[idx]),
                'resting_potential': float(self.npu_neurons.resting_potentials[idx]),
                'refractory_period': int(self.npu_neurons.refractory_periods[idx]),
                'cortical_idx': int(self.npu_neurons.cortical_idxs[idx]),
                'excitability': float(self.npu_neurons.excitability[idx])
            }
            
        except Exception as e:
            logger.error(f"Failed to get neuron {neuron_id} properties: {e}")
            return None
    
    def get_neuron_count(self) -> int:
        """Get current neuron count from NPU arrays."""
        return self.npu_neurons.neuron_count
    
    def get_all_neuron_ids(self) -> List[int]:
        """Get all valid neuron IDs from NPU arrays."""
        return list(self.npu_neurons.neuron_id_to_index.keys())


class BDUSynapseInterface:
    """BDU interface to NPU-owned synapse data structures.
    
    This provides BDU with controlled access to NPU synapse arrays during sleep periods.
    The interface mimics the BDU GlobalSynapseArray API while operating on NPU-owned data.
    """
    
    def __init__(self, npu_synapse_array):
        """Initialize BDU synapse interface.
        
        Args:
            npu_synapse_array: NPU-owned synapse array
        """
        self.npu_synapses = npu_synapse_array
        logger.info("BDU synapse interface initialized")
    
    def add_synapse(self, source_neuron_id: int, target_neuron_id: int, weight: float) -> bool:
        """Add a new synapse to NPU-owned arrays.
        
        Args:
            source_neuron_id: Source neuron identifier
            target_neuron_id: Target neuron identifier
            weight: Synapse weight
            
        Returns:
            True if successful
        """
        try:
            if self.npu_synapses.synapse_count >= self.npu_synapses.max_synapses:
                logger.error("NPU synapse array is full")
                return False
            
            idx = self.npu_synapses.synapse_count
            
            # Store synapse in NPU arrays
            self.npu_synapses.source_neurons[idx] = source_neuron_id
            self.npu_synapses.target_neurons[idx] = target_neuron_id
            self.npu_synapses.weights[idx] = weight
            
            # Update NPU index
            if source_neuron_id not in self.npu_synapses.source_neuron_index:
                self.npu_synapses.source_neuron_index[source_neuron_id] = []
            self.npu_synapses.source_neuron_index[source_neuron_id].append(idx)
            
            self.npu_synapses.synapse_count += 1
            
            logger.debug(f"Added synapse {source_neuron_id} -> {target_neuron_id} (weight={weight}) at index {idx}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to add synapse {source_neuron_id} -> {target_neuron_id}: {e}")
            return False
    
    def remove_synapse(self, source_neuron_id: int, target_neuron_id: int) -> bool:
        """Remove a synapse from NPU-owned arrays.
        
        Args:
            source_neuron_id: Source neuron identifier
            target_neuron_id: Target neuron identifier
            
        Returns:
            True if successful
        """
        try:
            if source_neuron_id not in self.npu_synapses.source_neuron_index:
                logger.warning(f"No synapses found for source neuron {source_neuron_id}")
                return False
            
            # Find synapse index
            synapse_indices = self.npu_synapses.source_neuron_index[source_neuron_id]
            target_idx = None
            
            for idx in synapse_indices:
                if self.npu_synapses.target_neurons[idx] == target_neuron_id:
                    target_idx = idx
                    break
            
            if target_idx is None:
                logger.warning(f"Synapse {source_neuron_id} -> {target_neuron_id} not found")
                return False
            
            # Remove from index
            self.npu_synapses.source_neuron_index[source_neuron_id].remove(target_idx)
            if not self.npu_synapses.source_neuron_index[source_neuron_id]:
                del self.npu_synapses.source_neuron_index[source_neuron_id]
            
            # Mark as invalid (weight = 0)
            self.npu_synapses.weights[target_idx] = 0.0
            
            logger.debug(f"Removed synapse {source_neuron_id} -> {target_neuron_id}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to remove synapse {source_neuron_id} -> {target_neuron_id}: {e}")
            return False
    
    def get_synapse_count(self) -> int:
        """Get current synapse count from NPU arrays."""
        return self.npu_synapses.synapse_count
    
    def get_synapses_for_neuron(self, source_neuron_id: int) -> List[Tuple[int, float]]:
        """Get all synapses for a source neuron.
        
        Args:
            source_neuron_id: Source neuron identifier
            
        Returns:
            List of (target_neuron_id, weight) tuples
        """
        try:
            if source_neuron_id not in self.npu_synapses.source_neuron_index:
                return []
            
            synapses = []
            for idx in self.npu_synapses.source_neuron_index[source_neuron_id]:
                target_id = self.npu_synapses.target_neurons[idx]
                weight = self.npu_synapses.weights[idx]
                if weight != 0.0:  # Skip removed synapses
                    synapses.append((int(target_id), float(weight)))
            
            return synapses
            
        except Exception as e:
            logger.error(f"Failed to get synapses for neuron {source_neuron_id}: {e}")
            return []
    
    def compact_arrays(self) -> int:
        """Compact NPU synapse arrays by removing invalid entries.
        
        This should be called during sleep periods to maintain array efficiency.
        
        Returns:
            Number of synapses removed
        """
        try:
            valid_indices = []
            new_source_index = {}
            
            # Find valid synapses
            for i in range(self.npu_synapses.synapse_count):
                if self.npu_synapses.weights[i] != 0.0:
                    valid_indices.append(i)
            
            # Compact arrays
            new_count = len(valid_indices)
            for new_idx, old_idx in enumerate(valid_indices):
                if new_idx != old_idx:
                    self.npu_synapses.source_neurons[new_idx] = self.npu_synapses.source_neurons[old_idx]
                    self.npu_synapses.target_neurons[new_idx] = self.npu_synapses.target_neurons[old_idx]
                    self.npu_synapses.weights[new_idx] = self.npu_synapses.weights[old_idx]
                
                # Rebuild index
                source_id = self.npu_synapses.source_neurons[new_idx]
                if source_id not in new_source_index:
                    new_source_index[source_id] = []
                new_source_index[source_id].append(new_idx)
            
            removed_count = self.npu_synapses.synapse_count - new_count
            self.npu_synapses.synapse_count = new_count
            self.npu_synapses.source_neuron_index = new_source_index
            
            logger.info(f"Compacted NPU synapse arrays: removed {removed_count} invalid synapses")
            return removed_count
            
        except Exception as e:
            logger.error(f"Failed to compact synapse arrays: {e}")
            return 0
