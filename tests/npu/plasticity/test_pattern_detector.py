"""
Test suite for RoaringBitmap pattern detection and SHA-256 hashing.

Tests the pattern detection system including:
- RoaringBitmap temporal pattern extraction
- SHA-256 deterministic hashing
- Per-area temporal depth configuration
- Pattern caching and LRU eviction
- Batch pattern detection
- Performance characteristics

Version: 3.0
"""

import pytest
import hashlib
import time
from typing import List, Dict

from feagi.npu.fire_ledger import FireLedgerInterface, RoaringBitmap
from feagi.npu.fire_queue import FiringNeuron
from feagi.npu.plasticity.pattern_detector import (
    PatternDetector, BatchPatternDetector, PatternConfig, TemporalPattern
)


def create_firing_neurons(neuron_data: Dict[int, List[int]]) -> Dict[int, List[FiringNeuron]]:
    """Helper function to create FiringNeuron objects from simple data."""
    neurons_by_area = {}
    for area_idx, neuron_ids in neuron_data.items():
        firing_neurons = []
        for neuron_id in neuron_ids:
            firing_neuron = FiringNeuron(
                neuron_id=neuron_id,
                cortical_idx=area_idx,
                membrane_potential=1.5,
                coordinates=(0, 0, 0),
                threshold=1.0,
                pre_fire_potential=0.8,
                consecutive_fire_count=1,
                refractory_counter=0
            )
            firing_neurons.append(firing_neuron)
        neurons_by_area[area_idx] = firing_neurons
    return neurons_by_area


class TestPatternDetector:
    """Test RoaringBitmap pattern detection system."""
    
    def pattern_config(self):
        """Create pattern detection configuration."""
        return PatternConfig(
            default_temporal_depth=3,
            min_activity_threshold=1,
            max_pattern_cache_size=100
        )
    
    def fire_ledger(self):
        """Create Fire Ledger with test data."""
        ledger = FireLedgerInterface(default_window_size=10)
        
        # Create test activity patterns with FiringNeuron objects
        test_patterns = [
            # Timestep 0
            {1: [10, 11, 12], 2: [20, 21], 3: [30]},
            # Timestep 1
            {1: [11, 12, 13], 2: [21, 22], 3: [31]},
            # Timestep 2
            {1: [12, 13, 14], 2: [22, 23], 3: [32]},
            # Timestep 3
            {1: [13, 14, 15], 2: [23, 24], 3: [33]},
            # Timestep 4 - repeat pattern from timestep 0
            {1: [10, 11, 12], 2: [20, 21], 3: [30]},
        ]
        
        for timestep, neuron_data in enumerate(test_patterns):
            neurons_by_area = create_firing_neurons(neuron_data)
            ledger.archive_timestep(timestep, neurons_by_area)
        
        return ledger
    
    def test_temporal_pattern_extraction(self, pattern_config, fire_ledger):
        """Test extraction of temporal patterns using RoaringBitmap operations."""
        
        detector = PatternDetector(pattern_config)
        
        # Test pattern detection at different timesteps
        pattern1 = detector.detect_pattern(
            fire_ledger=fire_ledger,
            memory_area_idx=42,
            upstream_areas=[1, 2, 3],
            current_timestep=2,
            temporal_depth=3
        )
        
        assert pattern1 is not None
        assert pattern1.temporal_depth == 3
        assert pattern1.upstream_areas == (1, 2, 3)
        assert len(pattern1.pattern_hash) == 32  # SHA-256 hash
        assert pattern1.total_activity > 0
        
        # Test pattern at different timestep
        pattern2 = detector.detect_pattern(
            fire_ledger=fire_ledger,
            memory_area_idx=42,
            upstream_areas=[1, 2, 3],
            current_timestep=3,
            temporal_depth=3
        )
        
        assert pattern2 is not None
        assert pattern2.pattern_hash != pattern1.pattern_hash  # Different patterns
        
        # Test repeating pattern
        pattern3 = detector.detect_pattern(
            fire_ledger=fire_ledger,
            memory_area_idx=42,
            upstream_areas=[1, 2, 3],
            current_timestep=4,
            temporal_depth=3
        )
        
        # Pattern at timestep 4 should be different from timestep 2 
        # because it includes different temporal context
        assert pattern3 is not None
        assert pattern3.pattern_hash != pattern1.pattern_hash
        
        print("✅ Temporal pattern extraction test passed")
    
    def test_sha256_deterministic_hashing(self, pattern_config, fire_ledger):
        """Test SHA-256 hashing produces deterministic results."""
        
        detector = PatternDetector(pattern_config)
        
        # Detect same pattern multiple times
        patterns = []
        for run in range(10):
            pattern = detector.detect_pattern(
                fire_ledger=fire_ledger,
                memory_area_idx=42,
                upstream_areas=[1, 2, 3],
                current_timestep=2,
                temporal_depth=3
            )
            patterns.append(pattern)
        
        # All patterns should be identical
        first_pattern = patterns[0]
        for pattern in patterns[1:]:
            assert pattern.pattern_hash == first_pattern.pattern_hash
            assert pattern.temporal_depth == first_pattern.temporal_depth
            assert pattern.upstream_areas == first_pattern.upstream_areas
            assert pattern.total_activity == first_pattern.total_activity
        
        # Test hash collision resistance
        # Create slightly different patterns and verify different hashes
        pattern_a = detector.detect_pattern(
            fire_ledger=fire_ledger,
            memory_area_idx=42,
            upstream_areas=[1, 2, 3],
            current_timestep=2,
            temporal_depth=3
        )
        
        pattern_b = detector.detect_pattern(
            fire_ledger=fire_ledger,
            memory_area_idx=42,
            upstream_areas=[1, 2],  # Different upstream areas
            current_timestep=2,
            temporal_depth=3
        )
        
        assert pattern_a.pattern_hash != pattern_b.pattern_hash
        
        print("✅ SHA-256 deterministic hashing test passed")
    
    def test_per_area_temporal_depth(self, pattern_config, fire_ledger):
        """Test per-area temporal depth configuration."""
        
        detector = PatternDetector(pattern_config)
        
        # Configure different temporal depths for different areas
        detector.configure_area_temporal_depth(10, 2)  # Short-term memory
        detector.configure_area_temporal_depth(20, 5)  # Medium-term memory
        detector.configure_area_temporal_depth(30, 8)  # Long-term memory
        
        # Test that each area uses its configured temporal depth
        pattern_short = detector.detect_pattern(
            fire_ledger=fire_ledger,
            memory_area_idx=10,
            upstream_areas=[1, 2],
            current_timestep=3
        )
        assert pattern_short.temporal_depth == 2
        
        pattern_medium = detector.detect_pattern(
            fire_ledger=fire_ledger,
            memory_area_idx=20,
            upstream_areas=[1, 2],
            current_timestep=3
        )
        assert pattern_medium.temporal_depth == 5
        
        pattern_long = detector.detect_pattern(
            fire_ledger=fire_ledger,
            memory_area_idx=30,
            upstream_areas=[1, 2],
            current_timestep=3
        )
        assert pattern_long.temporal_depth == 8
        
        # Test default temporal depth for unconfigured area
        pattern_default = detector.detect_pattern(
            fire_ledger=fire_ledger,
            memory_area_idx=99,  # Not configured
            upstream_areas=[1, 2],
            current_timestep=3
        )
        assert pattern_default.temporal_depth == pattern_config.default_temporal_depth
        
        print("✅ Per-area temporal depth test passed")
    
    def test_pattern_caching(self, pattern_config):
        """Test pattern caching and LRU eviction."""
        
        # Use small cache size for testing eviction
        small_config = PatternConfig(
            default_temporal_depth=3,
            min_activity_threshold=1,
            max_pattern_cache_size=3  # Small cache
        )
        
        detector = PatternDetector(small_config)
        
        # Create fire ledger with multiple distinct patterns
        ledger = FireLedgerInterface(default_window_size=10)
        
        patterns_data = [
            {1: [10], 2: [20]},      # Pattern A
            {1: [11], 2: [21]},      # Pattern B  
            {1: [12], 2: [22]},      # Pattern C
            {1: [13], 2: [23]},      # Pattern D
            {1: [14], 2: [24]},      # Pattern E
        ]
        
        for timestep, neuron_data in enumerate(patterns_data):
            neurons_by_area = create_firing_neurons(neuron_data)
            ledger.archive_timestep(timestep, neurons_by_area)
        
        # Detect patterns and verify caching
        detected_patterns = []
        
        for timestep in range(2, 5):  # Detect 3 patterns (A, B, C)
            pattern = detector.detect_pattern(
                fire_ledger=ledger,
                memory_area_idx=42,
                upstream_areas=[1, 2],
                current_timestep=timestep,
                temporal_depth=3
            )
            detected_patterns.append(pattern)
        
        # Cache should be full (3 patterns)
        assert len(detector._pattern_cache) == 3
        
        # Detect one more pattern (D) - should evict oldest (A)
        pattern_d = detector.detect_pattern(
            fire_ledger=ledger,
            memory_area_idx=42,
            upstream_areas=[1, 2],
            current_timestep=5,
            temporal_depth=3
        )
        
        # Cache should still be size 3, but pattern A should be evicted
        assert len(detector._pattern_cache) == 3
        assert detected_patterns[0].pattern_hash not in detector._pattern_cache  # Pattern A evicted
        assert pattern_d.pattern_hash in detector._pattern_cache  # Pattern D added
        
        print("✅ Pattern caching test passed")
    
    def test_empty_pattern_handling(self, pattern_config):
        """Test handling of empty patterns and insufficient activity."""
        
        detector = PatternDetector(pattern_config)
        ledger = FireLedgerInterface(default_window_size=10)
        
        # Create timesteps with no activity
        empty_patterns = [
            {},  # No activity
            {},  # No activity
            {},  # No activity
        ]
        
        for timestep, neuron_data in enumerate(empty_patterns):
            neurons_by_area = create_firing_neurons(neuron_data)
            ledger.archive_timestep(timestep, neurons_by_area)
        
        # Should return None for empty patterns
        pattern = detector.detect_pattern(
            fire_ledger=ledger,
            memory_area_idx=42,
            upstream_areas=[1, 2, 3],
            current_timestep=2,
            temporal_depth=3
        )
        
        assert pattern is None
        
        # Test insufficient activity (below threshold)
        config_high_threshold = PatternConfig(
            default_temporal_depth=3,
            min_activity_threshold=10,  # High threshold
            max_pattern_cache_size=100
        )
        
        detector_high = PatternDetector(config_high_threshold)
        
        # Create pattern with low activity
        low_activity = [
            {1: [10]},  # Only 1 neuron
            {1: [11]},  # Only 1 neuron
            {1: [12]},  # Only 1 neuron
        ]
        
        ledger_low = FireLedgerInterface(default_window_size=10)
        for timestep, neuron_data in enumerate(low_activity):
            neurons_by_area = create_firing_neurons(neuron_data)
            ledger_low.archive_timestep(timestep, neurons_by_area)
        
        pattern_low = detector_high.detect_pattern(
            fire_ledger=ledger_low,
            memory_area_idx=42,
            upstream_areas=[1],
            current_timestep=2,
            temporal_depth=3
        )
        
        assert pattern_low is None  # Below threshold
        
        print("✅ Empty pattern handling test passed")
    
    def test_batch_pattern_detection(self, pattern_config, fire_ledger):
        """Test batch processing of multiple memory areas."""
        
        batch_detector = BatchPatternDetector(pattern_config)
        
        # Configure multiple memory areas
        memory_areas = {
            10: {'temporal_depth': 2, 'upstream_areas': [1, 2]},
            20: {'temporal_depth': 3, 'upstream_areas': [2, 3]},
            30: {'temporal_depth': 4, 'upstream_areas': [1, 3]},
        }
        
        # Detect patterns for all areas in batch
        patterns = batch_detector.detect_patterns_batch(
            fire_ledger, memory_areas, current_timestep=3
        )
        
        # Verify all areas processed
        assert len(patterns) == 3
        assert 10 in patterns
        assert 20 in patterns
        assert 30 in patterns
        
        # Verify each pattern has correct temporal depth
        assert patterns[10].temporal_depth == 2
        assert patterns[20].temporal_depth == 3
        assert patterns[30].temporal_depth == 4
        
        # Verify different upstream areas produce different patterns
        assert patterns[10].pattern_hash != patterns[20].pattern_hash
        assert patterns[20].pattern_hash != patterns[30].pattern_hash
        
        print("✅ Batch pattern detection test passed")
    
    def test_pattern_statistics(self, pattern_config, fire_ledger):
        """Test pattern detection statistics collection."""
        
        detector = PatternDetector(pattern_config)
        
        # Reset statistics
        detector.reset_stats()
        initial_stats = detector.get_stats()
        assert initial_stats['patterns_detected'] == 0
        assert initial_stats['cache_hits'] == 0
        assert initial_stats['cache_misses'] == 0
        
        # Detect some patterns
        for timestep in range(2, 5):
            detector.detect_pattern(
                fire_ledger=fire_ledger,
                memory_area_idx=42,
                upstream_areas=[1, 2, 3],
                current_timestep=timestep,
                temporal_depth=3
            )
        
        stats_after = detector.get_stats()
        assert stats_after['patterns_detected'] == 3
        assert stats_after['cache_misses'] == 3  # All new patterns
        
        # Detect same pattern again (should hit cache)
        detector.detect_pattern(
            fire_ledger=fire_ledger,
            memory_area_idx=42,
            upstream_areas=[1, 2, 3],
            current_timestep=4,  # Same as last detection
            temporal_depth=3
        )
        
        final_stats = detector.get_stats()
        assert final_stats['cache_hits'] == 1  # One cache hit
        
        print("✅ Pattern statistics test passed")
    
    def test_performance_characteristics(self, pattern_config):
        """Test pattern detection performance characteristics."""
        
        detector = PatternDetector(pattern_config)
        
        # Create large fire ledger with many areas and neurons
        ledger = FireLedgerInterface(default_window_size=20)
        
        # Generate large activity patterns
        for timestep in range(10):
            neuron_data = {}
            for area_idx in range(1, 11):  # 10 areas
                neuron_count = 100  # 100 neurons per area
                neuron_ids = list(range(area_idx * 1000, area_idx * 1000 + neuron_count))
                neuron_data[area_idx] = neuron_ids
            
            # Convert to FiringNeuron objects and archive
            neurons_by_area = create_firing_neurons(neuron_data)
            ledger.archive_timestep(timestep, neurons_by_area)
        
        # Measure pattern detection performance
        start_time = time.time()
        
        patterns = []
        for run in range(100):  # 100 pattern detections
            pattern = detector.detect_pattern(
                fire_ledger=ledger,
                memory_area_idx=42,
                upstream_areas=list(range(1, 11)),  # All 10 areas
                current_timestep=9,
                temporal_depth=5
            )
            patterns.append(pattern)
        
        end_time = time.time()
        total_time = end_time - start_time
        avg_time_per_detection = total_time / 100
        
        # Performance assertions (adjust thresholds based on hardware)
        assert avg_time_per_detection < 0.01  # Less than 10ms per detection
        assert all(p is not None for p in patterns)  # All detections successful
        
        # Test that all patterns are identical (deterministic)
        first_hash = patterns[0].pattern_hash
        assert all(p.pattern_hash == first_hash for p in patterns)
        
        print(f"✅ Performance test passed: {avg_time_per_detection*1000:.2f}ms per detection")


if __name__ == "__main__":
    # Run tests directly
    test_suite = TestPatternDetector()
    
    # Create fixtures
    pattern_config = test_suite.pattern_config()
    fire_ledger = test_suite.fire_ledger()
    
    print("🔍 Running Pattern Detection Tests...")
    print("=" * 50)
    
    try:
        test_suite.test_temporal_pattern_extraction(pattern_config, fire_ledger)
        test_suite.test_sha256_deterministic_hashing(pattern_config, fire_ledger)
        test_suite.test_per_area_temporal_depth(pattern_config, fire_ledger)
        test_suite.test_pattern_caching(pattern_config)
        test_suite.test_empty_pattern_handling(pattern_config)
        test_suite.test_batch_pattern_detection(pattern_config, fire_ledger)
        test_suite.test_pattern_statistics(pattern_config, fire_ledger)
        test_suite.test_performance_characteristics(pattern_config)
        
        print("=" * 50)
        print("🎉 All Pattern Detection Tests PASSED!")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        raise
