"""
Integration test for Rust Morton spatial hash with real genome loading.

Validates that Morton hash is properly populated and used during genome loading
and subsequent neuron lookups.
"""

import pytest
import tempfile
import json
from pathlib import Path


@pytest.fixture
def minimal_genome():
    """Create a minimal test genome with projector mapping."""
    genome = {
        "blueprint": {
            "src_area": {
                "coordinates_3d": {"x": 0, "y": 0, "z": 0},
                "cortical_dimensions": {"x": 10, "y": 10, "z": 1},
                "neuron_params": {},
            },
            "dst_area": {
                "coordinates_3d": {"x": 0, "y": 1, "z": 0},
                "cortical_dimensions": {"x": 10, "y": 10, "z": 1},
                "neuron_params": {},
            }
        },
        "neuron_morphologies": {},
        "cortical_mappings": {
            "src_to_dst": {
                "src_cortical_area": "src_area",
                "dst_cortical_area": "dst_area",
                "morphology_id": "projector",
                "morphology_scalar": [1, 1, 1],
                "postSynapticCurrent_multiplier": 1.0,
            }
        }
    }
    return genome


def test_morton_hash_populated_after_genome_load(minimal_genome):
    """Test that Morton hash is populated after genome loading."""
    from feagi.bdu.connectome_manager import ConnectomeManager
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    from feagi.bdu.rust_morton_hash import RUST_MORTON_AVAILABLE
    
    if not RUST_MORTON_AVAILABLE:
        pytest.skip("Rust Morton not available")
    
    # Create temporary genome file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(minimal_genome, f)
        genome_path = f.name
    
    try:
        # Create ConnectomeManager and develop brain
        cm = ConnectomeManager(10000)
        embryo = NeuroEmbryogenesis(connectome_manager=cm)
        
        # Load genome
        success = embryo.develop_brain(genome_path)
        assert success, "Genome loading failed"
        
        # Verify Morton hash is initialized and populated
        assert hasattr(cm, '_rust_morton_hash'), "Morton hash not initialized"
        assert cm._rust_morton_hash is not None, "Morton hash is None"
        
        # Get stats from Morton hash
        stats = cm._rust_morton_hash.get_statistics()
        
        # Should have neurons from both areas
        assert stats['total_areas'] >= 2, f"Expected >= 2 areas, got {stats['total_areas']}"
        assert stats['total_neurons'] > 0, f"Expected neurons, got {stats['total_neurons']}"
        
        print(f"✅ Morton hash populated: {stats['total_neurons']} neurons across {stats['total_areas']} areas")
        
    finally:
        # Cleanup
        Path(genome_path).unlink(missing_ok=True)


def test_morton_hash_used_for_lookups(minimal_genome):
    """Test that Morton hash is actually used for position lookups."""
    from feagi.bdu.connectome_manager import ConnectomeManager
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    from feagi.bdu.rust_morton_hash import RUST_MORTON_AVAILABLE
    import time
    
    if not RUST_MORTON_AVAILABLE:
        pytest.skip("Rust Morton not available")
    
    # Create temporary genome file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(minimal_genome, f)
        genome_path = f.name
    
    try:
        # Create ConnectomeManager and develop brain
        cm = ConnectomeManager(10000)
        embryo = NeuroEmbryogenesis(connectome_manager=cm)
        
        # Load genome
        success = embryo.develop_brain(genome_path)
        assert success, "Genome loading failed"
        
        # Get a cortical area
        assert len(cm.cortical_areas) > 0, "No cortical areas created"
        cortical_id = list(cm.cortical_areas.keys())[0]
        
        # Get some positions from the area
        area = cm.cortical_areas[cortical_id]
        neuron_ids = list(area.neurons.keys())[:5]  # First 5 neurons
        
        if not neuron_ids:
            pytest.skip("No neurons created in test area")
        
        # Get positions for these neurons
        positions = set()
        for nid in neuron_ids:
            pos = cm.get_neuron_position(nid)
            if pos:
                positions.add(pos)
        
        if not positions:
            pytest.skip("Could not get neuron positions")
        
        # Test batch lookup (should use Morton hash)
        start = time.time()
        found = cm.batch_voxel_to_neuron_lookup(cortical_id, positions, post_synaptic_current=1.0)
        elapsed_ms = (time.time() - start) * 1000
        
        # Should find neurons
        assert len(found) > 0, f"Expected to find neurons at {len(positions)} positions"
        
        # Should be fast with Morton hash (<10ms for small lookups)
        assert elapsed_ms < 50, f"Lookup took {elapsed_ms}ms (should be <50ms with Morton hash)"
        
        print(f"✅ Morton lookup: found {len(found)} neurons in {elapsed_ms:.2f}ms")
        
    finally:
        # Cleanup
        Path(genome_path).unlink(missing_ok=True)


def test_morton_hash_cleared_on_brain_reset():
    """Test that Morton hash is cleared when brain data is reset."""
    from feagi.bdu.connectome_manager import ConnectomeManager
    from feagi.bdu.rust_morton_hash import RUST_MORTON_AVAILABLE
    
    if not RUST_MORTON_AVAILABLE:
        pytest.skip("Rust Morton not available")
    
    cm = ConnectomeManager(10000)
    
    # Manually add some data to Morton hash
    if cm._rust_morton_hash is not None:
        cm._rust_morton_hash.add_neuron("test", 5, 5, 0, 100)
        stats_before = cm._rust_morton_hash.get_statistics()
        assert stats_before['total_neurons'] > 0, "Failed to add neuron to Morton hash"
        
        # Clear brain data
        cm._clear_existing_brain_data()
        
        # Morton hash should be cleared
        stats_after = cm._rust_morton_hash.get_statistics()
        assert stats_after['total_neurons'] == 0, "Morton hash not cleared after brain reset"
        
        print(f"✅ Morton hash properly cleared on brain reset")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

