/*!
Morton spatial hash implementation using Roaring bitmaps.

High-performance spatial indexing for neuron positions.
*/

use ahash::AHashMap;
use roaring::RoaringBitmap;
use std::sync::{Arc, RwLock};

use super::morton::{morton_encode_3d, morton_encode_region_3d};

/// Spatial hash system using Morton encoding + Roaring bitmaps
pub struct MortonSpatialHash {
    /// Per-cortical-area bitmaps of occupied positions
    cortical_bitmaps: Arc<RwLock<AHashMap<String, RoaringBitmap>>>,
    
    /// Map Morton code -> list of neuron IDs at that position
    /// Key: (cortical_area, morton_code)
    neuron_map: Arc<RwLock<AHashMap<(String, u64), Vec<u64>>>>,
    
    /// Reverse map: neuron_id -> (area, x, y, z)
    coordinate_map: Arc<RwLock<AHashMap<u64, (String, u32, u32, u32)>>>,
}

impl MortonSpatialHash {
    /// Create a new spatial hash system
    pub fn new() -> Self {
        Self {
            cortical_bitmaps: Arc::new(RwLock::new(AHashMap::new())),
            neuron_map: Arc::new(RwLock::new(AHashMap::new())),
            coordinate_map: Arc::new(RwLock::new(AHashMap::new())),
        }
    }
    
    /// Add a neuron to the spatial hash
    pub fn add_neuron(
        &self,
        cortical_area: String,
        x: u32, y: u32, z: u32,
        neuron_id: u64,
    ) -> bool {
        // Validate coordinates
        if x >= (1 << 21) || y >= (1 << 21) || z >= (1 << 21) {
            return false;
        }
        
        let morton_code = morton_encode_3d(x, y, z);
        
        // Add to cortical bitmap
        {
            let mut bitmaps = self.cortical_bitmaps.write().unwrap();
            bitmaps.entry(cortical_area.clone())
                .or_insert_with(RoaringBitmap::new)
                .insert(morton_code as u32);
        }
        
        // Add to neuron map
        {
            let mut neuron_map = self.neuron_map.write().unwrap();
            let key = (cortical_area.clone(), morton_code);
            neuron_map.entry(key)
                .or_insert_with(Vec::new)
                .push(neuron_id);
        }
        
        // Add to coordinate map
        {
            let mut coord_map = self.coordinate_map.write().unwrap();
            coord_map.insert(neuron_id, (cortical_area, x, y, z));
        }
        
        true
    }
    
    /// Get first neuron at coordinate (or None)
    pub fn get_neuron_at_coordinate(
        &self,
        cortical_area: &str,
        x: u32, y: u32, z: u32,
    ) -> Option<u64> {
        if x >= (1 << 21) || y >= (1 << 21) || z >= (1 << 21) {
            return None;
        }
        
        let morton_code = morton_encode_3d(x, y, z);
        
        // Check if coordinate exists in bitmap
        {
            let bitmaps = self.cortical_bitmaps.read().unwrap();
            if let Some(bitmap) = bitmaps.get(cortical_area) {
                if !bitmap.contains(morton_code as u32) {
                    return None;
                }
            } else {
                return None;
            }
        }
        
        // Get neuron IDs
        let neuron_map = self.neuron_map.read().unwrap();
        let key = (cortical_area.to_string(), morton_code);
        neuron_map.get(&key)
            .and_then(|neurons| neurons.first().copied())
    }
    
    /// Get all neurons at coordinate
    pub fn get_neurons_at_coordinate(
        &self,
        cortical_area: &str,
        x: u32, y: u32, z: u32,
    ) -> Vec<u64> {
        if x >= (1 << 21) || y >= (1 << 21) || z >= (1 << 21) {
            return Vec::new();
        }
        
        let morton_code = morton_encode_3d(x, y, z);
        
        // Check bitmap first (fast)
        {
            let bitmaps = self.cortical_bitmaps.read().unwrap();
            if let Some(bitmap) = bitmaps.get(cortical_area) {
                if !bitmap.contains(morton_code as u32) {
                    return Vec::new();
                }
            } else {
                return Vec::new();
            }
        }
        
        // Get neurons
        let neuron_map = self.neuron_map.read().unwrap();
        let key = (cortical_area.to_string(), morton_code);
        neuron_map.get(&key)
            .map(|neurons| neurons.clone())
            .unwrap_or_default()
    }
    
    /// Get all neurons in a 3D region
    pub fn get_neurons_in_region(
        &self,
        cortical_area: &str,
        x1: u32, y1: u32, z1: u32,
        x2: u32, y2: u32, z2: u32,
    ) -> Vec<u64> {
        // Get area bitmap
        let area_bitmap = {
            let bitmaps = self.cortical_bitmaps.read().unwrap();
            match bitmaps.get(cortical_area) {
                Some(bitmap) => bitmap.clone(),
                None => return Vec::new(),
            }
        };
        
        // Create region bitmap
        let region_codes = morton_encode_region_3d(x1, y1, z1, x2, y2, z2);
        let mut region_bitmap = RoaringBitmap::new();
        for code in region_codes {
            region_bitmap.insert(code as u32);
        }
        
        // Fast intersection
        let intersection = &area_bitmap & &region_bitmap;
        
        // Collect neurons
        let neuron_map = self.neuron_map.read().unwrap();
        let mut result = Vec::new();
        
        for morton_code in intersection {
            let key = (cortical_area.to_string(), morton_code as u64);
            if let Some(neurons) = neuron_map.get(&key) {
                result.extend(neurons);
            }
        }
        
        result
    }
    
    /// Get neuron's position
    pub fn get_neuron_position(&self, neuron_id: u64) -> Option<(String, u32, u32, u32)> {
        let coord_map = self.coordinate_map.read().unwrap();
        coord_map.get(&neuron_id).cloned()
    }
    
    /// Remove a neuron from the spatial hash
    pub fn remove_neuron(&self, neuron_id: u64) -> bool {
        // Get position
        let position = {
            let mut coord_map = self.coordinate_map.write().unwrap();
            coord_map.remove(&neuron_id)
        };
        
        if let Some((area, x, y, z)) = position {
            let morton_code = morton_encode_3d(x, y, z);
            
            // Remove from neuron map
            {
                let mut neuron_map = self.neuron_map.write().unwrap();
                let key = (area.clone(), morton_code);
                if let Some(neurons) = neuron_map.get_mut(&key) {
                    neurons.retain(|&id| id != neuron_id);
                    if neurons.is_empty() {
                        neuron_map.remove(&key);
                    }
                }
            }
            
            // If no more neurons at this position, remove from bitmap
            {
                let neuron_map = self.neuron_map.read().unwrap();
                let key = (area.clone(), morton_code);
                if !neuron_map.contains_key(&key) {
                    let mut bitmaps = self.cortical_bitmaps.write().unwrap();
                    if let Some(bitmap) = bitmaps.get_mut(&area) {
                        bitmap.remove(morton_code as u32);
                    }
                }
            }
            
            true
        } else {
            false
        }
    }
    
    /// Clear all data
    pub fn clear(&self) {
        self.cortical_bitmaps.write().unwrap().clear();
        self.neuron_map.write().unwrap().clear();
        self.coordinate_map.write().unwrap().clear();
    }
    
    /// Get statistics
    pub fn get_stats(&self) -> SpatialHashStats {
        let bitmaps = self.cortical_bitmaps.read().unwrap();
        let coord_map = self.coordinate_map.read().unwrap();
        
        SpatialHashStats {
            total_areas: bitmaps.len(),
            total_neurons: coord_map.len(),
            total_occupied_positions: bitmaps.values().map(|b| b.len() as usize).sum(),
        }
    }
}

impl Default for MortonSpatialHash {
    fn default() -> Self {
        Self::new()
    }
}

/// Statistics about the spatial hash
#[derive(Debug, Clone)]
pub struct SpatialHashStats {
    pub total_areas: usize,
    pub total_neurons: usize,
    pub total_occupied_positions: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add_and_get_neuron() {
        let hash = MortonSpatialHash::new();
        
        assert!(hash.add_neuron("v1".to_string(), 10, 20, 30, 1001));
        
        let neuron = hash.get_neuron_at_coordinate("v1", 10, 20, 30);
        assert_eq!(neuron, Some(1001));
        
        let neurons = hash.get_neurons_at_coordinate("v1", 10, 20, 30);
        assert_eq!(neurons, vec![1001]);
    }

    #[test]
    fn test_multiple_neurons_same_position() {
        let hash = MortonSpatialHash::new();
        
        hash.add_neuron("v1".to_string(), 5, 5, 5, 100);
        hash.add_neuron("v1".to_string(), 5, 5, 5, 101);
        hash.add_neuron("v1".to_string(), 5, 5, 5, 102);
        
        let neurons = hash.get_neurons_at_coordinate("v1", 5, 5, 5);
        assert_eq!(neurons.len(), 3);
        assert!(neurons.contains(&100));
        assert!(neurons.contains(&101));
        assert!(neurons.contains(&102));
    }

    #[test]
    fn test_region_query() {
        let hash = MortonSpatialHash::new();
        
        // Add neurons in a 10x10x10 grid
        for x in 0..10 {
            for y in 0..10 {
                for z in 0..10 {
                    let neuron_id = (x * 100 + y * 10 + z) as u64;
                    hash.add_neuron("v1".to_string(), x, y, z, neuron_id);
                }
            }
        }
        
        // Query a 2x2x2 subregion
        let neurons = hash.get_neurons_in_region("v1", 0, 0, 0, 1, 1, 1);
        assert_eq!(neurons.len(), 8);
    }

    #[test]
    fn test_get_neuron_position() {
        let hash = MortonSpatialHash::new();
        
        hash.add_neuron("v1".to_string(), 42, 84, 126, 999);
        
        let pos = hash.get_neuron_position(999);
        assert_eq!(pos, Some(("v1".to_string(), 42, 84, 126)));
    }

    #[test]
    fn test_remove_neuron() {
        let hash = MortonSpatialHash::new();
        
        hash.add_neuron("v1".to_string(), 10, 20, 30, 1001);
        assert!(hash.remove_neuron(1001));
        
        let neuron = hash.get_neuron_at_coordinate("v1", 10, 20, 30);
        assert_eq!(neuron, None);
    }
}

