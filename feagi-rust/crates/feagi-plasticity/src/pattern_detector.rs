/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! High-performance temporal pattern detection using native HashSets
//!
//! This module replaces the Python pyroaring implementation with pure Rust,
//! using standard library HashSets for pattern detection and SHA-256 for
//! deterministic pattern hashing.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};
use sha2::{Sha256, Digest};

/// Configuration for pattern detection
#[derive(Debug, Clone)]
pub struct PatternConfig {
    /// Default temporal depth (timesteps to look back)
    pub default_temporal_depth: u32,
    
    /// Minimum neurons required for pattern recognition
    pub min_activity_threshold: usize,
    
    /// Maximum patterns to cache
    pub max_pattern_cache_size: usize,
}

impl Default for PatternConfig {
    fn default() -> Self {
        Self {
            default_temporal_depth: 3,
            min_activity_threshold: 1,
            max_pattern_cache_size: 10000,
        }
    }
}

/// Temporal pattern representation
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct TemporalPattern {
    /// SHA-256 hash of the pattern (32 bytes)
    pub pattern_hash: [u8; 32],
    
    /// Temporal depth used for this pattern
    pub temporal_depth: u32,
    
    /// Upstream cortical area indices
    pub upstream_areas: Vec<u32>,
    
    /// Neuron counts per timestep
    pub timestep_neuron_counts: Vec<usize>,
    
    /// Total activity across all timesteps
    pub total_activity: usize,
}

/// Statistics for pattern detection
#[derive(Debug, Clone, Default)]
pub struct PatternDetectorStats {
    pub patterns_detected: usize,
    pub cache_hits: usize,
    pub cache_misses: usize,
    pub empty_patterns: usize,
    pub set_operations: usize,
}

/// High-performance temporal pattern detector
pub struct PatternDetector {
    config: PatternConfig,
    
    /// Pattern cache (pattern_hash -> pattern)
    pattern_cache: Arc<Mutex<HashMap<[u8; 32], TemporalPattern>>>,
    
    /// LRU access order for cache eviction
    cache_access_order: Arc<Mutex<Vec<[u8; 32]>>>,
    
    /// Per-area temporal depth configuration
    area_temporal_depths: Arc<Mutex<HashMap<u32, u32>>>,
    
    /// Statistics
    stats: Arc<Mutex<PatternDetectorStats>>,
}

impl PatternDetector {
    /// Create a new pattern detector
    pub fn new(config: PatternConfig) -> Self {
        Self {
            config,
            pattern_cache: Arc::new(Mutex::new(HashMap::new())),
            cache_access_order: Arc::new(Mutex::new(Vec::new())),
            area_temporal_depths: Arc::new(Mutex::new(HashMap::new())),
            stats: Arc::new(Mutex::new(PatternDetectorStats::default())),
        }
    }
    
    /// Detect temporal pattern from firing history
    pub fn detect_pattern(
        &self,
        memory_area_idx: u32,
        upstream_areas: &[u32],
        _current_timestep: u64,
        timestep_bitmaps: Vec<HashSet<u32>>,
        temporal_depth: Option<u32>,
    ) -> Option<TemporalPattern> {
        if upstream_areas.is_empty() {
            return None;
        }
        
        // Get temporal depth for this area
        let area_temporal_depth = temporal_depth.unwrap_or_else(|| {
            self.get_area_temporal_depth(memory_area_idx)
        });
        
        if timestep_bitmaps.is_empty() {
            let mut stats = self.stats.lock().unwrap();
            stats.empty_patterns += 1;
            return None;
        }
        
        // Check if pattern has sufficient activity
        let total_activity: usize = timestep_bitmaps.iter()
            .map(|set| set.len())
            .sum();
        
        if total_activity < self.config.min_activity_threshold {
            let mut stats = self.stats.lock().unwrap();
            stats.empty_patterns += 1;
            return None;
        }
        
        // Create deterministic pattern hash
        let pattern_hash = self.create_pattern_hash(&timestep_bitmaps);
        
        // Check cache first
        {
            let cache = self.pattern_cache.lock().unwrap();
            if let Some(pattern) = cache.get(&pattern_hash) {
                self.update_cache_access(pattern_hash);
                let mut stats = self.stats.lock().unwrap();
                stats.cache_hits += 1;
                return Some(pattern.clone());
            }
        }
        
        // Create new pattern
        let timestep_neuron_counts: Vec<usize> = timestep_bitmaps.iter()
            .map(|set| set.len())
            .collect();
        
        let mut sorted_upstream = upstream_areas.to_vec();
        sorted_upstream.sort_unstable();
        
        let pattern = TemporalPattern {
            pattern_hash,
            temporal_depth: area_temporal_depth,
            upstream_areas: sorted_upstream,
            timestep_neuron_counts,
            total_activity,
        };
        
        // Cache the pattern
        self.add_to_cache(pattern.clone());
        
        let mut stats = self.stats.lock().unwrap();
        stats.patterns_detected += 1;
        stats.cache_misses += 1;
        
        Some(pattern)
    }
    
    /// Create deterministic SHA-256 hash from bitmap sequence
    fn create_pattern_hash(&self, timestep_bitmaps: &[HashSet<u32>]) -> [u8; 32] {
        let mut hasher = Sha256::new();
        
        // Serialize each bitmap in temporal order
        for bitmap in timestep_bitmaps {
            // Sort neuron IDs for determinism
            let mut sorted_ids: Vec<u32> = bitmap.iter().copied().collect();
            sorted_ids.sort_unstable();
            
            // Hash length prefix
            let len = sorted_ids.len() as u32;
            hasher.update(len.to_le_bytes());
            
            // Hash sorted neuron IDs
            for id in sorted_ids {
                hasher.update(id.to_le_bytes());
            }
        }
        
        hasher.finalize().into()
    }
    
    /// Add pattern to cache with LRU eviction
    fn add_to_cache(&self, pattern: TemporalPattern) {
        let pattern_hash = pattern.pattern_hash;
        
        let mut cache = self.pattern_cache.lock().unwrap();
        let mut access_order = self.cache_access_order.lock().unwrap();
        
        // Add to cache
        cache.insert(pattern_hash, pattern);
        access_order.push(pattern_hash);
        
        // Evict oldest if cache is full
        if cache.len() > self.config.max_pattern_cache_size {
            if let Some(oldest_hash) = access_order.first().copied() {
                access_order.remove(0);
                cache.remove(&oldest_hash);
            }
        }
    }
    
    /// Update cache access order for LRU
    fn update_cache_access(&self, pattern_hash: [u8; 32]) {
        let mut access_order = self.cache_access_order.lock().unwrap();
        if let Some(pos) = access_order.iter().position(|&h| h == pattern_hash) {
            access_order.remove(pos);
        }
        access_order.push(pattern_hash);
    }
    
    /// Configure temporal depth for a specific memory area
    pub fn configure_area_temporal_depth(&self, memory_area_idx: u32, temporal_depth: u32) {
        let mut depths = self.area_temporal_depths.lock().unwrap();
        depths.insert(memory_area_idx, temporal_depth);
    }
    
    /// Get temporal depth for a memory area
    fn get_area_temporal_depth(&self, memory_area_idx: u32) -> u32 {
        let depths = self.area_temporal_depths.lock().unwrap();
        depths.get(&memory_area_idx)
            .copied()
            .unwrap_or(self.config.default_temporal_depth)
    }
    
    /// Get detection statistics
    pub fn get_stats(&self) -> PatternDetectorStats {
        self.stats.lock().unwrap().clone()
    }
    
    /// Clear pattern cache
    pub fn clear_cache(&self) {
        let mut cache = self.pattern_cache.lock().unwrap();
        let mut access_order = self.cache_access_order.lock().unwrap();
        cache.clear();
        access_order.clear();
    }
    
    /// Reset statistics
    pub fn reset_stats(&self) {
        let mut stats = self.stats.lock().unwrap();
        *stats = PatternDetectorStats::default();
    }
}

/// Batch pattern detector for multiple memory areas
pub struct BatchPatternDetector {
    pub(crate) base_config: PatternConfig,
    pub(crate) detectors: Arc<Mutex<HashMap<u32, PatternDetector>>>,
}

impl BatchPatternDetector {
    /// Create a new batch pattern detector
    pub fn new(base_config: PatternConfig) -> Self {
        Self {
            base_config,
            detectors: Arc::new(Mutex::new(HashMap::new())),
        }
    }
    
    /// Get or create detector for memory area
    pub fn get_detector(&self, memory_area_idx: u32, temporal_depth: u32) -> PatternDetector {
        let mut detectors = self.detectors.lock().unwrap();
        
        if !detectors.contains_key(&memory_area_idx) {
            let detector = PatternDetector::new(self.base_config.clone());
            detector.configure_area_temporal_depth(memory_area_idx, temporal_depth);
            detectors.insert(memory_area_idx, detector);
        }
        
        // Clone the detector for thread-safe access
        detectors.get(&memory_area_idx).unwrap().clone()
    }
    
    /// Get statistics for all detectors
    pub fn get_batch_stats(&self) -> HashMap<u32, PatternDetectorStats> {
        let detectors = self.detectors.lock().unwrap();
        detectors.iter()
            .map(|(&area_idx, detector)| (area_idx, detector.get_stats()))
            .collect()
    }
}

impl Clone for PatternDetector {
    fn clone(&self) -> Self {
        Self {
            config: self.config.clone(),
            pattern_cache: Arc::clone(&self.pattern_cache),
            cache_access_order: Arc::clone(&self.cache_access_order),
            area_temporal_depths: Arc::clone(&self.area_temporal_depths),
            stats: Arc::clone(&self.stats),
        }
    }
}

impl Clone for BatchPatternDetector {
    fn clone(&self) -> Self {
        Self {
            base_config: self.base_config.clone(),
            detectors: Arc::clone(&self.detectors),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_pattern_config_default() {
        let config = PatternConfig::default();
        assert_eq!(config.default_temporal_depth, 3);
        assert_eq!(config.min_activity_threshold, 1);
        assert_eq!(config.max_pattern_cache_size, 10000);
    }
    
    #[test]
    fn test_pattern_detection() {
        let config = PatternConfig::default();
        let detector = PatternDetector::new(config);
        
        // Create test pattern
        let mut bitmap1 = HashSet::new();
        bitmap1.insert(1);
        bitmap1.insert(2);
        
        let mut bitmap2 = HashSet::new();
        bitmap2.insert(3);
        bitmap2.insert(4);
        
        let bitmaps = vec![bitmap1, bitmap2];
        let upstream_areas = vec![1, 2];
        
        let pattern = detector.detect_pattern(
            100, // memory area
            &upstream_areas,
            10, // timestep
            bitmaps,
            None,
        );
        
        assert!(pattern.is_some());
        let pattern = pattern.unwrap();
        assert_eq!(pattern.temporal_depth, 3);
        assert_eq!(pattern.total_activity, 4);
        assert_eq!(pattern.timestep_neuron_counts, vec![2, 2]);
        assert_eq!(pattern.upstream_areas, vec![1, 2]);
    }
    
    #[test]
    fn test_pattern_detection_empty() {
        let config = PatternConfig::default();
        let detector = PatternDetector::new(config);
        
        let bitmaps = vec![];
        let upstream_areas = vec![1];
        
        let pattern = detector.detect_pattern(100, &upstream_areas, 10, bitmaps, None);
        assert!(pattern.is_none());
        
        let stats = detector.get_stats();
        assert_eq!(stats.empty_patterns, 1);
    }
    
    #[test]
    fn test_pattern_detection_no_upstream() {
        let config = PatternConfig::default();
        let detector = PatternDetector::new(config);
        
        let mut bitmap = HashSet::new();
        bitmap.insert(1);
        
        let bitmaps = vec![bitmap];
        let upstream_areas = vec![];
        
        let pattern = detector.detect_pattern(100, &upstream_areas, 10, bitmaps, None);
        assert!(pattern.is_none());
    }
    
    #[test]
    fn test_pattern_detection_below_threshold() {
        let mut config = PatternConfig::default();
        config.min_activity_threshold = 10;
        let detector = PatternDetector::new(config);
        
        let mut bitmap = HashSet::new();
        bitmap.insert(1);
        bitmap.insert(2);
        
        let bitmaps = vec![bitmap];
        let upstream_areas = vec![1];
        
        let pattern = detector.detect_pattern(100, &upstream_areas, 10, bitmaps, None);
        assert!(pattern.is_none());
        
        let stats = detector.get_stats();
        assert_eq!(stats.empty_patterns, 1);
    }
    
    #[test]
    fn test_pattern_cache() {
        let config = PatternConfig::default();
        let detector = PatternDetector::new(config);
        
        let mut bitmap = HashSet::new();
        bitmap.insert(1);
        bitmap.insert(2);
        
        let bitmaps = vec![bitmap.clone()];
        let upstream_areas = vec![1];
        
        // First detection - cache miss
        let pattern1 = detector.detect_pattern(100, &upstream_areas, 10, bitmaps.clone(), None);
        assert!(pattern1.is_some());
        
        let stats = detector.get_stats();
        assert_eq!(stats.cache_misses, 1);
        assert_eq!(stats.cache_hits, 0);
        assert_eq!(stats.patterns_detected, 1);
        
        // Second detection - cache hit
        let pattern2 = detector.detect_pattern(100, &upstream_areas, 11, bitmaps, None);
        assert!(pattern2.is_some());
        
        let stats = detector.get_stats();
        assert_eq!(stats.cache_hits, 1);
        assert_eq!(stats.patterns_detected, 1); // Still 1, cache hit doesn't create new pattern
        
        // Patterns should be equal
        assert_eq!(pattern1.unwrap().pattern_hash, pattern2.unwrap().pattern_hash);
    }
    
    #[test]
    fn test_cache_eviction() {
        let mut config = PatternConfig::default();
        config.max_pattern_cache_size = 2;
        let detector = PatternDetector::new(config);
        
        // Create 3 different patterns
        for i in 0..3 {
            let mut bitmap = HashSet::new();
            bitmap.insert(i);
            detector.detect_pattern(100, &vec![1], 10, vec![bitmap], None);
        }
        
        // Cache should have at most 2 patterns
        let cache = detector.pattern_cache.lock().unwrap();
        assert!(cache.len() <= 2);
    }
    
    #[test]
    fn test_deterministic_hashing() {
        let config = PatternConfig::default();
        let detector = PatternDetector::new(config);
        
        let mut bitmap = HashSet::new();
        bitmap.insert(3);
        bitmap.insert(1);
        bitmap.insert(2);
        
        let hash1 = detector.create_pattern_hash(&[bitmap.clone()]);
        let hash2 = detector.create_pattern_hash(&[bitmap]);
        
        assert_eq!(hash1, hash2);
    }
    
    #[test]
    fn test_different_patterns_different_hashes() {
        let config = PatternConfig::default();
        let detector = PatternDetector::new(config);
        
        let mut bitmap1 = HashSet::new();
        bitmap1.insert(1);
        bitmap1.insert(2);
        
        let mut bitmap2 = HashSet::new();
        bitmap2.insert(1);
        bitmap2.insert(3); // Different from bitmap1
        
        let hash1 = detector.create_pattern_hash(&[bitmap1]);
        let hash2 = detector.create_pattern_hash(&[bitmap2]);
        
        assert_ne!(hash1, hash2);
    }
    
    #[test]
    fn test_temporal_order_sensitivity() {
        let config = PatternConfig::default();
        let detector = PatternDetector::new(config);
        
        let mut bitmap1 = HashSet::new();
        bitmap1.insert(1);
        
        let mut bitmap2 = HashSet::new();
        bitmap2.insert(2);
        
        // Different temporal orders should produce different hashes
        let hash1 = detector.create_pattern_hash(&[bitmap1.clone(), bitmap2.clone()]);
        let hash2 = detector.create_pattern_hash(&[bitmap2, bitmap1]);
        
        assert_ne!(hash1, hash2);
    }
    
    #[test]
    fn test_configure_area_temporal_depth() {
        let config = PatternConfig::default();
        let default_temp_depth = config.default_temporal_depth;
        let detector = PatternDetector::new(config);
        
        detector.configure_area_temporal_depth(100, 5);
        
        let depth = detector.get_area_temporal_depth(100);
        assert_eq!(depth, 5);
        
        // Unconfigured area should use default
        let default_depth = detector.get_area_temporal_depth(999);
        assert_eq!(default_depth, default_temp_depth);
    }
    
    #[test]
    fn test_clear_cache() {
        let config = PatternConfig::default();
        let detector = PatternDetector::new(config);
        
        let mut bitmap = HashSet::new();
        bitmap.insert(1);
        detector.detect_pattern(100, &vec![1], 10, vec![bitmap], None);
        
        // Cache should have entries
        {
            let cache = detector.pattern_cache.lock().unwrap();
            assert!(!cache.is_empty());
        }
        
        detector.clear_cache();
        
        // Cache should be empty
        let cache = detector.pattern_cache.lock().unwrap();
        assert!(cache.is_empty());
    }
    
    #[test]
    fn test_reset_stats() {
        let config = PatternConfig::default();
        let detector = PatternDetector::new(config);
        
        let mut bitmap = HashSet::new();
        bitmap.insert(1);
        detector.detect_pattern(100, &vec![1], 10, vec![bitmap], None);
        
        let stats = detector.get_stats();
        assert!(stats.patterns_detected > 0);
        
        detector.reset_stats();
        
        let stats = detector.get_stats();
        assert_eq!(stats.patterns_detected, 0);
        assert_eq!(stats.cache_hits, 0);
        assert_eq!(stats.cache_misses, 0);
    }
    
    #[test]
    fn test_batch_pattern_detector() {
        let config = PatternConfig::default();
        let batch_detector = BatchPatternDetector::new(config);
        
        let detector = batch_detector.get_detector(100, 5);
        
        let mut bitmap = HashSet::new();
        bitmap.insert(1);
        let pattern = detector.detect_pattern(100, &vec![1], 10, vec![bitmap], None);
        
        assert!(pattern.is_some());
    }
    
    #[test]
    fn test_batch_stats() {
        let config = PatternConfig::default();
        let batch_detector = BatchPatternDetector::new(config);
        
        // Create patterns for multiple areas
        for area_idx in vec![100, 200, 300] {
            let detector = batch_detector.get_detector(area_idx, 3);
            let mut bitmap = HashSet::new();
            bitmap.insert(area_idx);
            detector.detect_pattern(area_idx, &vec![1], 10, vec![bitmap], None);
        }
        
        let stats = batch_detector.get_batch_stats();
        assert_eq!(stats.len(), 3);
        assert!(stats.contains_key(&100));
        assert!(stats.contains_key(&200));
        assert!(stats.contains_key(&300));
    }
}

