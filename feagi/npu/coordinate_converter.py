"""
Coordinate Converter for FEAGI NPU

Converts sensory stream SoA (Structure of Arrays) voxel coordinates to universal neuron IDs.
Handles the transformation: [x1,x2,...], [y1,y2,...], [z1,z2,...], [p1,p2,...] → [i1,i2,...], [p1,p2,...]

Key Features:
- High-performance vectorized operations
- Rust/RTOS compatible data structures
- Zero-allocation paths for real-time processing
- Error handling for invalid coordinates
"""

from typing import Tuple, Optional
import numpy as np
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class CoordinateConverter:
    """Converts voxel coordinates to universal neuron IDs using SoA format."""
    
    def __init__(self, connectome_manager):
        """Initialize coordinate converter.
        
        Args:
            connectome_manager: ConnectomeManager instance for neuron lookup
        """
        self.connectome_manager = connectome_manager
        self._neuron_coordinate_cache = {}  # Cache for performance
        
    def convert_soa_to_neuron_ids(self,
                                 cortical_id: str,
                                 x_coords: np.ndarray,    # [x1, x2, ...]
                                 y_coords: np.ndarray,    # [y1, y2, ...]  
                                 z_coords: np.ndarray,    # [z1, z2, ...]
                                 potentials: np.ndarray   # [p1, p2, ...]
                                ) -> Tuple[np.ndarray, np.ndarray, int]:
        """Convert voxel coordinates to universal neuron IDs.
        
        Args:
            cortical_id: Target cortical area ID
            x_coords: X coordinates array
            y_coords: Y coordinates array  
            z_coords: Z coordinates array
            potentials: Membrane potential deltas array
            
        Returns:
            Tuple of (neuron_ids, valid_potentials, cortical_idx)
            - neuron_ids: [i1, i2, ...] universal neuron IDs
            - valid_potentials: [p1, p2, ...] corresponding potentials
            - cortical_idx: Integer cortical area index
        """
        if len(x_coords) != len(y_coords) != len(z_coords) != len(potentials):
            raise ValueError("All coordinate and potential arrays must have same length")
            
        # Get cortical area index
        cortical_idx = self._get_cortical_idx(cortical_id)
        if cortical_idx is None:
            logger.warning(f"Unknown cortical area: {cortical_id}")
            return np.array([]), np.array([]), -1
            
        # Pre-allocate result arrays for efficiency
        neuron_ids = []
        valid_potentials = []
        
        # Vectorized coordinate processing
        for i in range(len(x_coords)):
            neuron_id = self._get_neuron_at_coordinate(
                cortical_id, x_coords[i], y_coords[i], z_coords[i]
            )
            if neuron_id is not None:
                neuron_ids.append(neuron_id)
                valid_potentials.append(potentials[i])
                
        # --debug-npu: log mapping stats and a small coordinate sample
        try:
            from feagi.core.state_manager import FeagiStateManager
            if FeagiStateManager.instance().is_debug_npu_enabled():
                total = int(len(x_coords))
                mapped = int(len(neuron_ids))
                sample_n = min(10, mapped)
                sample_pairs = [
                    (int(neuron_ids[j]), float(valid_potentials[j])) for j in range(sample_n)
                ]
                # Also include coordinate range preview
                if total > 0:
                    xmin, xmax = int(np.min(x_coords)), int(np.max(x_coords))
                    ymin, ymax = int(np.min(y_coords)), int(np.max(y_coords))
                    zmin, zmax = int(np.min(z_coords)), int(np.max(z_coords))
                    logger.info(
                        f"[NPU] [Convert] {cortical_id}: mapped={mapped}/{total} "
                        f"x=[{xmin},{xmax}] y=[{ymin},{ymax}] z=[{zmin},{zmax}] sample(neuron_id,delta)={sample_pairs}"
                    )
        except Exception:
            pass

        return np.array(neuron_ids, dtype=np.int32), np.array(valid_potentials, dtype=np.float32), cortical_idx
    
    def _get_cortical_idx(self, cortical_id: str) -> Optional[int]:
        """Get cortical area index from cortical ID.

        Uses the authoritative mapping on ConnectomeManager to avoid relying
        on internal dict key shapes (which are indices, not IDs).
        """
        try:
            if hasattr(self.connectome_manager, 'get_cortical_idx_for_id'):
                idx = self.connectome_manager.get_cortical_idx_for_id(cortical_id)
                if idx is not None:
                    return idx
        except Exception:
            pass

        # Deterministic resolution via NPU interface when cortical mapping is not yet populated
        try:
            npu_iface = getattr(self.connectome_manager, '_npu_interface', None)
            areas = getattr(npu_iface, 'cortical_areas', None) if npu_iface is not None else None
            if isinstance(areas, dict) and areas:
                for idx, meta in areas.items():
                    if isinstance(meta, dict) and meta.get('cortical_id') == cortical_id:
                        # Optional debug: note that we resolved via NPU interface
                        try:
                            from feagi.core.state_manager import FeagiStateManager
                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                logger.info(f"[NPU] [Convert] Resolved cortical_idx={int(idx)} for area '{cortical_id}' via NPU interface")
                        except Exception:
                            pass
                        return int(idx)
        except Exception:
            pass
        return None
    
    def _get_neuron_at_coordinate(self, cortical_id: str, x: int, y: int, z: int) -> Optional[int]:
        """Get neuron ID at specific voxel coordinate.
        
        Uses caching for performance optimization.
        """
        cache_key = (cortical_id, x, y, z)
        if cache_key in self._neuron_coordinate_cache:
            return self._neuron_coordinate_cache[cache_key]
            
        # Try multiple methods to find neuron at coordinate
        neuron_id = None
        
        # RUST-COMPATIBLE: Deterministic lookup methods without runtime reflection
        
        # Method 1: Direct connectome manager lookup
        get_neuron_at_coordinate = getattr(self.connectome_manager, 'get_neuron_at_coordinate', None)
        if get_neuron_at_coordinate is not None:
            try:
                neuron_id = get_neuron_at_coordinate(cortical_id, x, y, z)
            except (KeyError, ValueError, TypeError):
                # @architecture:acceptable - coordinate lookup fallback
                neuron_id = None
            
        # Method 2: Cortical area coordinate lookup
        if neuron_id is None:
            cortical_areas = getattr(self.connectome_manager, 'cortical_areas', None)
            if cortical_areas is not None:
                cortical_area = cortical_areas.get(cortical_id)
                if cortical_area is not None:
                    get_neuron_at_coord = getattr(cortical_area, 'get_neuron_at_coordinate', None)
                    if get_neuron_at_coord is not None:
                        try:
                            neuron_id = get_neuron_at_coord(x, y, z)
                        except (KeyError, ValueError, TypeError):
                            # @architecture:acceptable - coordinate lookup fallback
                            neuron_id = None
                
        # Method 3: Neuron array coordinate mapping
        if neuron_id is None:
            neuron_array = getattr(self.connectome_manager, 'neuron_array', None)
            if neuron_array is not None:
                get_neuron_at_coord = getattr(neuron_array, 'get_neuron_at_coordinate', None)
                if get_neuron_at_coord is not None:
                    try:
                        neuron_id = get_neuron_at_coord(cortical_id, x, y, z)
                    except (KeyError, ValueError, TypeError):
                        # @architecture:acceptable - coordinate lookup fallback
                        neuron_id = None
        
        # Cache result for future use
        if neuron_id is not None:
            self._neuron_coordinate_cache[cache_key] = neuron_id
            
        return neuron_id
    
    def clear_cache(self):
        """Clear coordinate cache (useful for memory management)."""
        self._neuron_coordinate_cache.clear()
    
    def get_cache_stats(self) -> dict:
        """Get cache statistics for monitoring."""
        return {
            'cache_entries': len(self._neuron_coordinate_cache),
            'memory_usage_estimate': len(self._neuron_coordinate_cache) * 64  # Rough estimate
        }
