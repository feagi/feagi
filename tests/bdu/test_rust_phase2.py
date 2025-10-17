"""
Tests for Rust BDU Phase 2 morphologies.

Tests EXPANDER_X, REDUCER_X, and BLOCK_CONNECTION implementations.
"""

import pytest


def test_expander_single():
    """Test single expander - scales coordinates."""
    from feagi_bdu import py_syn_expander
    
    # Scale up 2x
    result = py_syn_expander(
        "src", "dst",
        (5, 5, 5),
        (10, 10, 10),  # src dimensions
        (20, 20, 20),  # dst dimensions
    )
    assert result == (10, 10, 10)


def test_expander_batch():
    """Test batch expander - parallel processing."""
    from feagi_bdu import py_syn_expander_batch
    import time
    
    # Batch process 1000 neurons
    batch_size = 1000
    positions = [(i % 10, (i // 10) % 10, (i // 100) % 10) for i in range(batch_size)]
    
    start = time.time()
    results = py_syn_expander_batch(
        "src", "dst",
        positions,
        (10, 10, 10),
        (20, 20, 20)
    )
    elapsed_ms = (time.time() - start) * 1000
    
    assert len(results) == batch_size
    assert elapsed_ms < 10  # Should be < 10ms
    
    # Verify scaling
    assert results[0] == (0, 0, 0)
    assert results[555] == (10, 10, 10)  # (5,5,5) * 2


def test_reducer_binary_encoding():
    """Test reducer - binary encoding to multiple positions."""
    from feagi_bdu import py_syn_reducer_x
    
    # Binary 5 = 0b101 -> bits 0 and 2 set
    result = py_syn_reducer_x(
        "src", "dst",
        (5, 0, 0),
        (10, 10, 10),
        (8, 1, 1),
        0, 0  # dst_y_index, dst_z_index
    )
    
    # Should map to x=0 and x=2 (bits 0 and 2)
    assert (0, 0, 0) in result
    assert (2, 0, 0) in result
    assert len(result) == 2


def test_block_connection():
    """Test block connection - block mapping with scaling."""
    from feagi_bdu import py_syn_block_connection
    
    # Scaling factor 10: neuron at (25, 5, 3) -> (2, 5, 3)
    result = py_syn_block_connection(
        "src", "dst",
        (25, 5, 3),
        (100, 10, 10),
        (10, 10, 10),
        10  # scaling_factor
    )
    
    assert result == (2, 5, 3)


def test_phase2_all_loaded():
    """Verify all Phase 2 functions are available."""
    from feagi_bdu import (
        py_syn_projector,
        py_syn_projector_batch,
        py_syn_expander,
        py_syn_expander_batch,
        py_syn_reducer_x,
        py_syn_block_connection,
    )
    
    # All should be callable
    assert callable(py_syn_projector)
    assert callable(py_syn_projector_batch)
    assert callable(py_syn_expander)
    assert callable(py_syn_expander_batch)
    assert callable(py_syn_reducer_x)
    assert callable(py_syn_block_connection)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

