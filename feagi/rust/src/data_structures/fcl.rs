/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

use roaring::RoaringBitmap;

/// Fire Candidate List (FCL) implementation
///
/// Uses Roaring Bitmap for efficient storage and SIMD-optimized operations
/// Can be converted to flat arrays for GPU processing
#[derive(Clone)]
pub struct FireCandidateList {
    /// Bitmap storage of neuron IDs that are firing
    bitmap: RoaringBitmap,

    /// Cached array for GPU upload (only generated when needed)
    #[allow(dead_code)]
    gpu_array_cache: Option<Vec<u32>>,

    /// Whether the cache is dirty and needs to be regenerated
    cache_dirty: bool,
}

impl FireCandidateList {
    /// Creates a new, empty FCL
    pub fn new() -> Self {
        Self {
            bitmap: RoaringBitmap::new(),
            gpu_array_cache: None,
            cache_dirty: false,
        }
    }

    /// Creates an FCL from a list of neuron IDs
    pub fn from_neuron_ids(ids: &[u32]) -> Self {
        let mut fcl = Self::new();
        fcl.add_multiple(ids);
        fcl
    }

    /// Adds a neuron to the FCL
    pub fn add(&mut self, neuron_id: u32) {
        self.bitmap.insert(neuron_id);
        self.cache_dirty = true;
    }

    /// Adds multiple neurons to the FCL
    pub fn add_multiple(&mut self, neuron_ids: &[u32]) {
        for id in neuron_ids {
            self.bitmap.insert(*id);
        }
        self.cache_dirty = true;
    }

    /// Removes a neuron from the FCL
    pub fn remove(&mut self, neuron_id: u32) {
        self.bitmap.remove(neuron_id);
        self.cache_dirty = true;
    }

    /// Clears the FCL
    pub fn clear(&mut self) {
        self.bitmap.clear();
        self.gpu_array_cache = None;
        self.cache_dirty = false;
    }

    /// Returns whether the neuron is in the FCL
    pub fn contains(&self, neuron_id: u32) -> bool {
        self.bitmap.contains(neuron_id)
    }

    /// Returns the number of neurons in the FCL
    pub fn len(&self) -> u64 {
        self.bitmap.len()
    }

    /// Returns whether the FCL is empty
    pub fn is_empty(&self) -> bool {
        self.bitmap.is_empty()
    }

    /// Returns a flat array of neuron IDs for GPU processing
    pub fn to_gpu_array(&mut self) -> &[u32] {
        if self.gpu_array_cache.is_none() || self.cache_dirty {
            let array: Vec<u32> = self.bitmap.iter().collect();
            self.gpu_array_cache = Some(array);
            self.cache_dirty = false;
        }

        self.gpu_array_cache.as_ref().unwrap().as_slice()
    }

    /// Performs a bitwise OR operation (union) with another FCL
    pub fn union_with(&mut self, other: &FireCandidateList) {
        self.bitmap |= &other.bitmap;
        self.cache_dirty = true;
    }

    /// Performs a bitwise AND operation (intersection) with another FCL
    pub fn intersect_with(&mut self, other: &FireCandidateList) {
        self.bitmap &= &other.bitmap;
        self.cache_dirty = true;
    }

    /// Performs a bitwise XOR operation (symmetric difference) with another FCL
    pub fn xor_with(&mut self, other: &FireCandidateList) {
        self.bitmap ^= &other.bitmap;
        self.cache_dirty = true;
    }

    /// Returns an iterator over the neuron IDs in the FCL
    pub fn iter(&self) -> impl Iterator<Item = u32> + '_ {
        self.bitmap.iter()
    }

    /// Converts to a Vec<u32> for compatibility with other code
    pub fn to_vec(&self) -> Vec<u32> {
        self.bitmap.iter().collect()
    }

    /// Performs SIMD-accelerated operations (when available)
    /// using the Roaring implementation's built-in SIMD optimizations
    pub fn simd_process_with(&mut self, other: &FireCandidateList, operation: FclOperation) {
        match operation {
            FclOperation::Union => self.union_with(other),
            FclOperation::Intersection => self.intersect_with(other),
            FclOperation::Difference => {
                self.bitmap -= &other.bitmap;
                self.cache_dirty = true;
            },
            FclOperation::SymmetricDifference => self.xor_with(other),
        }
    }
}

/// Operations that can be performed on FCLs
pub enum FclOperation {
    Union,
    Intersection,
    Difference,
    SymmetricDifference,
}

/// GPU-friendly representation of the FCL for compute shader use
#[repr(C)]
pub struct GpuFclData {
    /// Buffer offset to the array of neuron IDs
    pub neuron_ids_offset: u32,

    /// Number of neurons in the FCL
    pub count: u32,
}
