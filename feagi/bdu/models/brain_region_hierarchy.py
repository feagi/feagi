"""
Brain Region Hierarchy Management

This module provides a Rust-friendly hierarchical data structure for managing
brain regions and automatic input/output designation based on cortical mappings.

The design is optimized for:
- Fast ancestry checking (O(log n))
- Efficient area-to-region lookups (O(1))
- Minimal memory footprint
- Rust FFI compatibility
- Thread-safe operations
"""

from typing import Dict, List, Optional, Set, Tuple, Any
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class BrainRegionNode:
    """Rust-friendly brain region node.
    
    This structure is designed for efficient serialization to Rust
    and minimal memory usage.
    """
    region_id: str
    parent_id: Optional[str]
    children_ids: List[str]
    cortical_areas: List[str]
    inputs: List[str]
    outputs: List[str]
    depth: int  # Distance from root (for fast ancestry checks)
    
    def __post_init__(self):
        """Ensure lists are initialized."""
        if self.children_ids is None:
            self.children_ids = []
        if self.cortical_areas is None:
            self.cortical_areas = []
        if self.inputs is None:
            self.inputs = []
        if self.outputs is None:
            self.outputs = []


class BrainRegionHierarchy:
    """Hierarchical brain region management with automatic I/O designation.
    
    This class provides:
    1. Fast ancestry checking for cortical mapping rules
    2. Efficient area-to-region lookups
    3. Automatic input/output designation
    4. Rust-friendly data structures
    5. Thread-safe operations
    
    Design Principles:
    - All operations are O(1) or O(log n)
    - Data structures are Rust FFI compatible
    - Minimal memory footprint
    - Immutable operations where possible
    """
    
    def __init__(self):
        """Initialize the brain region hierarchy."""
        # Core data structures (Rust-friendly)
        self.nodes: Dict[str, BrainRegionNode] = {}
        self.area_to_region: Dict[str, str] = {}  # cortical_area_id -> region_id
        self.depth_map: Dict[str, int] = {}  # region_id -> depth from root
        
        # Performance caches
        self._ancestry_cache: Dict[Tuple[str, str], bool] = {}  # (ancestor, descendant) -> bool
        self._path_cache: Dict[str, List[str]] = {}  # region_id -> path to root
        
        # Thread safety
        self._cache_dirty = False
        
    def load_from_genome(self, genome_data: Dict[str, Any]) -> None:
        """Load brain region hierarchy from genome data.
        
        Args:
            genome_data: Hierarchical genome containing brain_regions section
        """
        brain_regions = genome_data.get("brain_regions", {})
        
        # Clear existing data
        self.nodes.clear()
        self.area_to_region.clear()
        self.depth_map.clear()
        self._clear_caches()
        
        # First pass: Create all nodes
        for region_id, region_data in brain_regions.items():
            node = BrainRegionNode(
                region_id=region_id,
                parent_id=region_data.get("parent_region_id"),
                # Accept both legacy 'regions' and modern 'child_regions'
                children_ids=region_data.get("child_regions", region_data.get("regions", [])).copy(),
                # Accept both legacy 'areas' and modern 'cortical_areas'
                cortical_areas=region_data.get("cortical_areas", region_data.get("areas", [])).copy(),
                inputs=region_data.get("inputs", []).copy(),
                outputs=region_data.get("outputs", []).copy(),
                depth=0  # Will be calculated in second pass
            )
            self.nodes[region_id] = node
            
            # Build area-to-region mapping
            for area_id in node.cortical_areas:
                self.area_to_region[area_id] = region_id
        
        # Second pass: Calculate depths and validate hierarchy
        self._calculate_depths()
        self._validate_hierarchy()
        
        logger.info(f"Loaded brain region hierarchy: {len(self.nodes)} regions, {len(self.area_to_region)} areas")
    
    def _calculate_depths(self) -> None:
        """Calculate depth from root for each region."""
        # Find root nodes (no parent)
        roots = [rid for rid, node in self.nodes.items() if node.parent_id is None]
        
        if not roots:
            logger.warning("No root regions found in hierarchy")
            return
        
        if len(roots) > 1:
            logger.warning(f"Multiple root regions found: {roots}")
        
        # BFS to calculate depths
        queue = [(rid, 0) for rid in roots]
        visited = set()
        
        while queue:
            region_id, depth = queue.pop(0)
            
            if region_id in visited:
                continue
                
            visited.add(region_id)
            
            if region_id in self.nodes:
                self.nodes[region_id].depth = depth
                self.depth_map[region_id] = depth
                
                # Add children to queue
                for child_id in self.nodes[region_id].children_ids:
                    if child_id not in visited:
                        queue.append((child_id, depth + 1))
    
    def _validate_hierarchy(self) -> None:
        """Validate hierarchy consistency."""
        errors = []
        
        for region_id, node in self.nodes.items():
            # Check parent-child consistency
            if node.parent_id and node.parent_id in self.nodes:
                parent_node = self.nodes[node.parent_id]
                if region_id not in parent_node.children_ids:
                    errors.append(f"Region {region_id} not in parent's children list")
            
            # Check children exist
            for child_id in node.children_ids:
                if child_id not in self.nodes:
                    errors.append(f"Child region {child_id} does not exist")
        
        if errors:
            logger.error(f"Hierarchy validation errors: {errors}")
            raise ValueError(f"Invalid brain region hierarchy: {errors}")
    
    def _clear_caches(self) -> None:
        """Clear performance caches."""
        self._ancestry_cache.clear()
        self._path_cache.clear()
        self._cache_dirty = False
    
    def get_region_for_area(self, cortical_area_id: str) -> Optional[str]:
        """Get the region ID that contains a cortical area.
        
        Args:
            cortical_area_id: ID of the cortical area
            
        Returns:
            Region ID or None if area not found
        """
        return self.area_to_region.get(cortical_area_id)
    
    def is_ancestor(self, ancestor_region_id: str, descendant_region_id: str) -> bool:
        """Check if one region is an ancestor of another.
        
        This is the core method for cortical mapping rules.
        
        Args:
            ancestor_region_id: Potential ancestor region
            descendant_region_id: Potential descendant region
            
        Returns:
            True if ancestor_region_id is an ancestor of descendant_region_id
        """
        # Check cache first
        cache_key = (ancestor_region_id, descendant_region_id)
        if cache_key in self._ancestry_cache:
            return self._ancestry_cache[cache_key]
        
        # Same region is not an ancestor of itself
        if ancestor_region_id == descendant_region_id:
            self._ancestry_cache[cache_key] = False
            return False
        
        # Check if both regions exist
        if ancestor_region_id not in self.nodes or descendant_region_id not in self.nodes:
            self._ancestry_cache[cache_key] = False
            return False
        
        # Traverse up from descendant to find ancestor
        current_id = descendant_region_id
        visited = set()
        
        while current_id is not None and current_id not in visited:
            visited.add(current_id)
            
            if current_id not in self.nodes:
                break
                
            node = self.nodes[current_id]
            current_id = node.parent_id
            
            if current_id == ancestor_region_id:
                self._ancestry_cache[cache_key] = True
                return True
        
        self._ancestry_cache[cache_key] = False
        return False
    
    def should_designate_io(self, source_area_id: str, target_area_id: str) -> Tuple[bool, bool]:
        """Determine if areas should be designated as I/O for a new mapping.
        
        Updated rule for clarity and symmetry across region boundaries:
        - If both areas are in the same region: no designation (False, False)
        - If source region is an ancestor of target region (outside → inside):
          designate INPUT on the target only (False, True)
        - Otherwise (different branches or child → parent): designate both
          OUTPUT on source and INPUT on target (True, True)
        
        Args:
            source_area_id: Source cortical area ID
            target_area_id: Target cortical area ID
            
        Returns:
            Tuple of (should_designate_source_as_output, should_designate_target_as_input)
        """
        # Get regions for both areas
        source_region_id = self.get_region_for_area(source_area_id)
        target_region_id = self.get_region_for_area(target_area_id)
        
        # If either area doesn't belong to a region, no designation
        if not source_region_id or not target_region_id:
            logger.debug(f"No region found for areas {source_area_id} or {target_area_id}")
            return False, False
        
        # If areas are in the same region, no designation needed
        if source_region_id == target_region_id:
            logger.debug(f"Areas {source_area_id} and {target_area_id} in same region {source_region_id}")
            return False, False
        
        # If source is ancestor of target (outside → inside): input only
        if self.is_ancestor(source_region_id, target_region_id):
            logger.debug(
                f"Mapping {source_area_id} -> {target_area_id}: source_region={source_region_id} is ancestor of target_region={target_region_id}; designate (False, True)"
            )
            return False, True
        
        # Otherwise (different branches or child → parent): designate both
        logger.debug(
            f"Mapping {source_area_id} -> {target_area_id}: cross-branch or child→parent; designate (True, True)"
        )
        return True, True
    
    def add_input_area(self, region_id: str, area_id: str) -> bool:
        """Add an area as input to a region.
        
        Args:
            region_id: Region to update
            area_id: Area to add as input
            
        Returns:
            True if added successfully
        """
        if region_id not in self.nodes:
            return False
            
        node = self.nodes[region_id]
        
        # Validate area belongs to this region
        if area_id not in node.cortical_areas:
            logger.warning(f"Area {area_id} not in region {region_id}, cannot add as input")
            return False
        
        # Remove from outputs if present (no overlap allowed)
        if area_id in node.outputs:
            node.outputs.remove(area_id)
        
        # Add to inputs if not already present
        if area_id not in node.inputs:
            node.inputs.append(area_id)
            self._cache_dirty = True
            
        return True
    
    def add_output_area(self, region_id: str, area_id: str) -> bool:
        """Add an area as output to a region.
        
        Args:
            region_id: Region to update
            area_id: Area to add as output
            
        Returns:
            True if added successfully
        """
        if region_id not in self.nodes:
            return False
            
        node = self.nodes[region_id]
        
        # Validate area belongs to this region
        if area_id not in node.cortical_areas:
            logger.warning(f"Area {area_id} not in region {region_id}, cannot add as output")
            return False
        
        # Remove from inputs if present (no overlap allowed)
        if area_id in node.inputs:
            node.inputs.remove(area_id)
        
        # Add to outputs if not already present
        if area_id not in node.outputs:
            node.outputs.append(area_id)
            self._cache_dirty = True
            
        return True
    
    def sync_to_genome(self, genome_data: Dict[str, Any]) -> None:
        """Sync hierarchy changes back to genome data.
        
        Args:
            genome_data: Genome data to update
        """
        if "brain_regions" not in genome_data:
            genome_data["brain_regions"] = {}
        
        brain_regions = genome_data["brain_regions"]
        
        for region_id, node in self.nodes.items():
            if region_id not in brain_regions:
                brain_regions[region_id] = {}
            
            region_data = brain_regions[region_id]
            region_data["inputs"] = node.inputs.copy()
            region_data["outputs"] = node.outputs.copy()
            region_data["cortical_areas"] = node.cortical_areas.copy()
            region_data["child_regions"] = node.children_ids.copy()
            region_data["parent_region_id"] = node.parent_id
    
    def get_hierarchy_stats(self) -> Dict[str, Any]:
        """Get hierarchy statistics for debugging.
        
        Returns:
            Dictionary with hierarchy statistics
        """
        return {
            "total_regions": len(self.nodes),
            "total_areas": len(self.area_to_region),
            "max_depth": max(self.depth_map.values()) if self.depth_map else 0,
            "cache_size": len(self._ancestry_cache),
            "cache_dirty": self._cache_dirty,
            "regions_by_depth": {
                depth: [rid for rid, d in self.depth_map.items() if d == depth]
                for depth in set(self.depth_map.values())
            }
        }
