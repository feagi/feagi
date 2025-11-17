"""Test Rust Morton integration with ConnectomeManager."""

import pytest


def test_rust_morton_enabled_in_connectome():
    """Verify Rust Morton is enabled in ConnectomeManager."""
    from feagi.bdu.connectome_manager import ConnectomeManager
    from feagi.bdu.rust_morton_hash import RUST_MORTON_AVAILABLE
    
    if not RUST_MORTON_AVAILABLE:
        pytest.skip("Rust Morton not available")
    
    # Create ConnectomeManager
    cm = ConnectomeManager(10000)  # Pass max_neurons as positional arg
    
    # Check Rust Morton is initialized
    assert hasattr(cm, '_rust_morton_hash')
    assert cm._rust_morton_hash is not None


def test_neuron_creation_populates_morton():
    """Test that creating neurons populates Rust Morton hash."""
    from feagi.bdu.connectome_manager import ConnectomeManager
    from feagi.bdu.rust_morton_hash import RUST_MORTON_AVAILABLE
    from feagi.bdu.models.cortical_area import CorticalArea
    
    if not RUST_MORTON_AVAILABLE:
        pytest.skip("Rust Morton not available")
    
    cm = ConnectomeManager(10000)
    
    # Create a cortical area
    area = CorticalArea(
        cortical_id="test_area",
        dimensions=(10, 10, 1),
        cortical_type="custom"
    )
    cm.cortical_areas["test_area"] = area
    
    # Create neurons
    positions = [(0, 0, 0), (1, 1, 0), (2, 2, 0)]
    neuron_ids = cm.batch_create_neurons(
        cortical_id="test_area",
        cortical_idx=0,
        positions=positions
    )
    
    assert len(neuron_ids) == 3
    
    # Verify neurons are in Rust Morton hash
    for i, neuron_id in enumerate(neuron_ids):
        x, y, z = positions[i]
        found = cm._rust_morton_hash.get_neuron_at_coordinate("test_area", x, y, z)
        assert found == neuron_id, f"Expected {neuron_id}, got {found} at position {positions[i]}"


def test_batch_voxel_lookup_uses_morton():
    """Test that batch_voxel_to_neuron_lookup uses Rust Morton."""
    from feagi.bdu.connectome_manager import ConnectomeManager
    from feagi.bdu.rust_morton_hash import RUST_MORTON_AVAILABLE
    from feagi.bdu.models.cortical_area import CorticalArea
    import time
    
    if not RUST_MORTON_AVAILABLE:
        pytest.skip("Rust Morton not available")
    
    cm = ConnectomeManager(10000)
    
    # Create area
    area = CorticalArea(
        cortical_id="test_area",
        dimensions=(100, 100, 1),
        cortical_type="custom"
    )
    cm.cortical_areas["test_area"] = area
    
    # Create 1000 neurons
    positions = [(i % 100, (i // 100) % 100, 0) for i in range(1000)]
    neuron_ids = cm.batch_create_neurons(
        cortical_id="test_area",
        cortical_idx=0,
        positions=positions
    )
    
    # Lookup positions (should use Rust Morton - O(K) not O(N))
    candidate_positions = {(50, 50, 0), (25, 25, 0), (75, 75, 0)}
    
    start = time.time()
    found = cm.batch_voxel_to_neuron_lookup("test_area", candidate_positions)
    elapsed_ms = (time.time() - start) * 1000
    
    # Should find 3 neurons
    assert len(found) == 3
    
    # Should be fast (<1ms with Rust Morton)
    assert elapsed_ms < 10, f"Lookup took {elapsed_ms}ms (should be <10ms with Rust Morton)"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

