"""
NPU Service for Core API

This service provides NPU-specific endpoints for the new architecture where NPU
owns all neuron and synapse data structures. It exposes SIMD-optimized CRUD
operations and cortical area management.

Key Features:
- Cortical area creation/deletion
- Batch neuron operations (create, update, delete)
- Batch synapse operations (create, update, delete)
- Neuron property access and statistics
- Cortical area locking coordination
- Performance monitoring and diagnostics

Architecture:
- Uses the new NPUInterface for all operations
- Respects cortical area locking (BDU operations take precedence)
- Returns structured results with proper error handling
- RTOS/Rust compatible design patterns
"""

from typing import Any, Dict, List, Optional, Tuple
import logging
from dataclasses import asdict

from feagi.utils.logger import setup_logger
from feagi.npu.interface import (
    NPUInterface, 
    NeuronCreationRequest, 
    SynapseCreationRequest,
    NeuronUpdateRequest,
    OperationResult,
    BatchOperationResult
)
from feagi.npu.data_structures import BackendType
from feagi.core.state_manager import FeagiStateManager

logger = setup_logger()


class NPUService:
    """
    NPU Service providing high-level API endpoints for NPU operations.
    
    This service acts as a bridge between REST/ZMQ endpoints and the NPU interface,
    providing proper error handling, validation, and response formatting.
    """
    
    def __init__(self, npu_interface: Optional[NPUInterface] = None):
        """Initialize NPU service.
        
        Args:
            npu_interface: Optional NPU interface instance. If None, creates a new one.
        """
        self.logger = logger
        self._npu_interface = npu_interface
        self._state_manager = None
        
        if self._npu_interface is None:
            self._initialize_npu_interface()
    
    def _initialize_npu_interface(self):
        """Initialize NPU interface with default configuration."""
        try:
            # Default to CPU backend for now
            # TODO: Read backend from configuration
            self._npu_interface = NPUInterface(BackendType.CPU)
            self.logger.info("🧠 NPU Service initialized with CPU backend")
        except Exception as e:
            self.logger.error(f"Failed to initialize NPU interface: {e}")
            self._npu_interface = None
    
    def _get_state_manager(self) -> Optional[FeagiStateManager]:
        """Get State Manager instance for cortical locking."""
        if self._state_manager is None:
            try:
                self._state_manager = FeagiStateManager.instance()
            except Exception as e:
                self.logger.error(f"Failed to get State Manager: {e}")
                self._state_manager = None
        return self._state_manager
    
    def _ensure_npu_available(self) -> bool:
        """Ensure NPU interface is available."""
        if self._npu_interface is None:
            self.logger.error("NPU interface not available")
            return False
        return True
    
    def _format_operation_result(self, result: OperationResult, data: Any = None) -> Dict[str, Any]:
        """Format operation result for API response."""
        return {
            "success": result == OperationResult.SUCCESS,
            "result": result.name,
            "data": data,
            "error": None if result == OperationResult.SUCCESS else result.name
        }
    
    def _format_batch_result(self, result: BatchOperationResult) -> Dict[str, Any]:
        """Format batch operation result for API response."""
        return {
            "success": result.result == OperationResult.SUCCESS,
            "result": result.result.name,
            "successful_count": result.successful_count,
            "failed_indices": result.failed_indices,
            "total_requested": result.successful_count + len(result.failed_indices),
            "error": None if result.result == OperationResult.SUCCESS else result.result.name
        }
    
    # ===== CORTICAL AREA MANAGEMENT =====
    
    def create_cortical_area(self, cortical_idx: int, dimensions: Tuple[int, int, int], 
                           area_type: str = "regular") -> Dict[str, Any]:
        """Create a new cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            dimensions: (width, height, depth) dimensions
            area_type: Type of area ("regular" or "memory")
            
        Returns:
            API response with operation result
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            result = self._npu_interface.create_cortical_area(
                cortical_idx=cortical_idx,
                dimensions=dimensions,
                area_type=area_type
            )
            
            return self._format_operation_result(result, {
                "cortical_idx": cortical_idx,
                "dimensions": dimensions,
                "area_type": area_type
            })
            
        except Exception as e:
            self.logger.error(f"Error creating cortical area {cortical_idx}: {e}")
            return {"success": False, "error": str(e)}
    
    def delete_cortical_area(self, cortical_idx: int) -> Dict[str, Any]:
        """Delete a cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            API response with operation result
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            result = self._npu_interface.delete_cortical_area(cortical_idx)
            
            return self._format_operation_result(result, {
                "cortical_idx": cortical_idx
            })
            
        except Exception as e:
            self.logger.error(f"Error deleting cortical area {cortical_idx}: {e}")
            return {"success": False, "error": str(e)}
    
    # ===== NEURON OPERATIONS =====
    
    def create_neurons_batch(self, cortical_idx: int, positions: List[Tuple[int, int, int]],
                           neuron_types: Optional[List[int]] = None,
                           initial_potentials: Optional[List[float]] = None,
                           thresholds: Optional[List[float]] = None,
                           leak_coefficients: Optional[List[float]] = None,
                           excitabilities: Optional[List[float]] = None) -> Dict[str, Any]:
        """Create multiple neurons in a cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            positions: List of (x, y, z) positions for neurons
            neuron_types: Optional list of neuron types
            initial_potentials: Optional list of initial membrane potentials
            thresholds: Optional list of firing thresholds
            leak_coefficients: Optional list of leak coefficients
            excitabilities: Optional list of excitability values
            
        Returns:
            API response with batch operation result
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            request = NeuronCreationRequest(
                cortical_idx=cortical_idx,
                positions=positions,
                neuron_types=neuron_types,
                initial_potentials=initial_potentials,
                thresholds=thresholds,
                leak_coefficients=leak_coefficients,
                excitabilities=excitabilities
            )
            
            result = self._npu_interface.create_neurons_batch(request)
            
            response = self._format_batch_result(result)
            response["data"] = {
                "cortical_idx": cortical_idx,
                "requested_count": len(positions)
            }
            
            return response
            
        except Exception as e:
            self.logger.error(f"Error creating neurons batch in area {cortical_idx}: {e}")
            return {"success": False, "error": str(e)}
    
    def delete_neurons_batch(self, neuron_ids: List[int]) -> Dict[str, Any]:
        """Delete multiple neurons.
        
        Args:
            neuron_ids: List of neuron IDs to delete
            
        Returns:
            API response with batch operation result
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            result = self._npu_interface.delete_neurons_batch(neuron_ids)
            
            response = self._format_batch_result(result)
            response["data"] = {
                "requested_neuron_ids": neuron_ids
            }
            
            return response
            
        except Exception as e:
            self.logger.error(f"Error deleting neurons batch: {e}")
            return {"success": False, "error": str(e)}
    
    def update_neurons_batch(self, updates: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Update multiple neurons.
        
        Args:
            updates: List of update dictionaries with neuron_id and properties
            
        Returns:
            API response with batch operation result
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            # Convert update dictionaries to NeuronUpdateRequest objects
            update_requests = []
            for update in updates:
                request = NeuronUpdateRequest(
                    neuron_id=update["neuron_id"],
                    properties=update.get("properties", {})
                )
                update_requests.append(request)
            
            result = self._npu_interface.update_neurons_batch(update_requests)
            
            response = self._format_batch_result(result)
            response["data"] = {
                "requested_updates": len(updates)
            }
            
            return response
            
        except Exception as e:
            self.logger.error(f"Error updating neurons batch: {e}")
            return {"success": False, "error": str(e)}
    
    def get_neuron_property(self, neuron_id: int, property_name: str) -> Dict[str, Any]:
        """Get a specific property of a neuron.
        
        Args:
            neuron_id: ID of the neuron
            property_name: Name of the property to retrieve
            
        Returns:
            API response with property value
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            value = self._npu_interface.get_neuron_property(neuron_id, property_name)
            
            return {
                "success": True,
                "data": {
                    "neuron_id": neuron_id,
                    "property_name": property_name,
                    "value": value
                }
            }
            
        except Exception as e:
            self.logger.error(f"Error getting neuron {neuron_id} property {property_name}: {e}")
            return {"success": False, "error": str(e)}
    
    def get_neurons_by_area(self, cortical_idx: int) -> Dict[str, Any]:
        """Get all neurons in a cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            API response with list of neuron IDs
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            neuron_ids = self._npu_interface.get_neurons_by_area(cortical_idx)
            
            return {
                "success": True,
                "data": {
                    "cortical_idx": cortical_idx,
                    "neuron_ids": neuron_ids,
                    "count": len(neuron_ids)
                }
            }
            
        except Exception as e:
            self.logger.error(f"Error getting neurons for area {cortical_idx}: {e}")
            return {"success": False, "error": str(e)}
    
    # ===== SYNAPSE OPERATIONS =====
    
    def create_synapses_batch(self, source_neuron_ids: List[int], target_neuron_ids: List[int],
                            weights: Optional[List[float]] = None,
                            delays: Optional[List[int]] = None) -> Dict[str, Any]:
        """Create multiple synapses.
        
        Args:
            source_neuron_ids: List of source neuron IDs
            target_neuron_ids: List of target neuron IDs
            weights: Optional list of synaptic weights
            delays: Optional list of synaptic delays
            
        Returns:
            API response with batch operation result
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            request = SynapseCreationRequest(
                source_neuron_ids=source_neuron_ids,
                target_neuron_ids=target_neuron_ids,
                weights=weights,
                delays=delays
            )
            
            result = self._npu_interface.create_synapses_batch(request)
            
            response = self._format_batch_result(result)
            response["data"] = {
                "requested_count": len(source_neuron_ids)
            }
            
            return response
            
        except Exception as e:
            self.logger.error(f"Error creating synapses batch: {e}")
            return {"success": False, "error": str(e)}
    
    def delete_synapses_batch(self, synapse_ids: List[int]) -> Dict[str, Any]:
        """Delete multiple synapses.
        
        Args:
            synapse_ids: List of synapse IDs to delete
            
        Returns:
            API response with batch operation result
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            result = self._npu_interface.delete_synapses_batch(synapse_ids)
            
            response = self._format_batch_result(result)
            response["data"] = {
                "requested_synapse_ids": synapse_ids
            }
            
            return response
            
        except Exception as e:
            self.logger.error(f"Error deleting synapses batch: {e}")
            return {"success": False, "error": str(e)}
    
    # ===== STATISTICS AND MONITORING =====
    
    def get_area_statistics(self, cortical_idx: int) -> Dict[str, Any]:
        """Get statistics for a specific cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            API response with area statistics
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            stats = self._npu_interface.get_area_statistics(cortical_idx)
            
            return {
                "success": True,
                "data": stats
            }
            
        except Exception as e:
            self.logger.error(f"Error getting statistics for area {cortical_idx}: {e}")
            return {"success": False, "error": str(e)}
    
    def get_total_statistics(self) -> Dict[str, Any]:
        """Get total system statistics.
        
        Returns:
            API response with total statistics
        """
        if not self._ensure_npu_available():
            return {"success": False, "error": "NPU_NOT_AVAILABLE"}
        
        try:
            stats = self._npu_interface.get_total_statistics()
            
            return {
                "success": True,
                "data": stats
            }
            
        except Exception as e:
            self.logger.error(f"Error getting total statistics: {e}")
            return {"success": False, "error": str(e)}
    
    # ===== CORTICAL AREA LOCKING =====
    
    def get_locked_areas(self) -> Dict[str, Any]:
        """Get list of currently locked cortical areas.
        
        Returns:
            API response with locked areas information
        """
        try:
            state_manager = self._get_state_manager()
            if not state_manager:
                return {"success": False, "error": "STATE_MANAGER_NOT_AVAILABLE"}
            
            locked_areas = state_manager.get_locked_cortical_areas()
            locking_stats = state_manager.get_cortical_locking_statistics()
            
            return {
                "success": True,
                "data": {
                    "locked_areas": locked_areas,
                    "statistics": locking_stats
                }
            }
            
        except Exception as e:
            self.logger.error(f"Error getting locked areas: {e}")
            return {"success": False, "error": str(e)}
    
    def get_npu_status(self) -> Dict[str, Any]:
        """Get NPU status and configuration.
        
        Returns:
            API response with NPU status
        """
        try:
            if not self._npu_interface:
                return {
                    "success": True,
                    "data": {
                        "available": False,
                        "error": "NPU interface not initialized"
                    }
                }
            
            # Get total statistics as a health check
            stats = self._npu_interface.get_total_statistics()
            
            return {
                "success": True,
                "data": {
                    "available": True,
                    "backend": self._npu_interface.backend.name,
                    "statistics": stats
                }
            }
            
        except Exception as e:
            self.logger.error(f"Error getting NPU status: {e}")
            return {"success": False, "error": str(e)}
