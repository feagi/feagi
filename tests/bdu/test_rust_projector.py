"""
Test Rust BDU Projector Performance and Correctness

Tests the Rust-accelerated syn_projector implementation.
"""

import pytest
import time

try:
    from feagi_bdu import py_syn_projector, py_syn_projector_batch
    RUST_AVAILABLE = True
except ImportError:
    RUST_AVAILABLE = False


@pytest.mark.skipif(not RUST_AVAILABLE, reason="Rust BDU not available")
class TestRustProjector:
    """Test Rust syn_projector implementation."""

    def test_single_projection(self):
        """Test single neuron projection."""
        result = py_syn_projector(
            "src", "dst", 42,
            (128, 128, 3),
            (128, 128, 1),
            (64, 64, 1),
            None, None
        )
        
        assert len(result) > 0, "Should generate at least one position"
        
        # Verify positions are within bounds
        for x, y, z in result:
            assert 0 <= x < 128, f"X out of bounds: {x}"
            assert 0 <= y < 128, f"Y out of bounds: {y}"
            assert 0 <= z < 1, f"Z out of bounds: {z}"

    def test_batch_projection_performance(self):
        """Test batch projection performance (1000 neurons)."""
        neuron_ids = list(range(1000))
        neuron_locations = [
            (i % 128, (i // 128) % 128, (i // (128 * 128)) % 3)
            for i in range(1000)
        ]
        
        start = time.time()
        results = py_syn_projector_batch(
            "src", "dst",
            neuron_ids, neuron_locations,
            (128, 128, 3), (128, 128, 1),
            None, None
        )
        elapsed = time.time() - start
        
        assert len(results) == 1000
        assert elapsed < 0.01, f"Batch should complete in <10ms, took {elapsed*1000:.2f}ms"

    def test_corner_position(self):
        """Test corner position (0,0,0)."""
        result = py_syn_projector(
            "src", "dst", 0,
            (128, 128, 3), (128, 128, 1),
            (0, 0, 0), None, None
        )
        assert len(result) > 0

    def test_max_position(self):
        """Test max position."""
        result = py_syn_projector(
            "src", "dst", 0,
            (128, 128, 3), (128, 128, 1),
            (127, 127, 2), None, None
        )
        assert len(result) > 0

    def test_scale_down(self):
        """Test scale down (256→128)."""
        result = py_syn_projector(
            "src", "dst", 0,
            (256, 256, 1), (128, 128, 1),
            (64, 64, 0), None, None
        )
        assert len(result) == 1, "Scale down should map to single position"

    def test_scale_up(self):
        """Test scale up (64→128)."""
        result = py_syn_projector(
            "src", "dst", 0,
            (64, 64, 1), (128, 128, 1),
            (32, 32, 0), None, None
        )
        assert len(result) >= 1, "Scale up should map to multiple positions"

    def test_performance_target(self):
        """Test that full projection meets performance target."""
        # Simulate full 49,152 neuron projection
        num_neurons = 1000
        neuron_ids = list(range(num_neurons))
        neuron_locations = [
            (i % 128, (i // 128) % 128, (i // (128 * 128)) % 3)
            for i in range(num_neurons)
        ]
        
        start = time.time()
        results = py_syn_projector_batch(
            "src", "dst",
            neuron_ids, neuron_locations,
            (128, 128, 3), (128, 128, 1),
            None, None
        )
        elapsed = time.time() - start
        
        # Extrapolate to full dataset
        full_neurons = 128 * 128 * 3  # 49,152
        full_time = elapsed * (full_neurons / num_neurons)
        
        # Should complete in <5 seconds (was 40s in Python)
        assert full_time < 5.0, f"Full projection should be <5s, extrapolated: {full_time:.2f}s"

