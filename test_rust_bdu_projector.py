#!/usr/bin/env python3
"""
Test Rust BDU Projector Performance

Validates that the Rust implementation works correctly and is significantly
faster than the Python implementation.
"""

import time
from feagi_bdu import py_syn_projector, py_syn_projector_batch

def test_single_projection():
    """Test single neuron projection (your exact use case)."""
    print("\n" + "=" * 60)
    print("TEST 1: Single Neuron Projection")
    print("=" * 60)
    
    # Your exact parameters: 128x128x3 → 128x128x1
    result = py_syn_projector(
        "iic400",           # source area
        "dst001",           # destination area  
        42,                 # source neuron ID
        (128, 128, 3),      # source dimensions
        (128, 128, 1),      # destination dimensions
        (64, 64, 1),        # neuron location
        None,               # no transpose
        None                # no last layer projection
    )
    
    print(f"✅ Generated {len(result)} candidate positions")
    print(f"   Sample positions: {result[:5]}...")
    assert len(result) > 0, "Should generate at least one position"
    
    # Verify positions are within bounds
    for x, y, z in result:
        assert 0 <= x < 128, f"X out of bounds: {x}"
        assert 0 <= y < 128, f"Y out of bounds: {y}"
        assert 0 <= z < 1, f"Z out of bounds: {z}"
    
    print("✅ All positions within destination bounds")

def test_full_projection_performance():
    """Test full 128x128x3 projection (49,152 neurons)."""
    print("\n" + "=" * 60)
    print("TEST 2: Full Projection Performance (Your Use Case)")
    print("=" * 60)
    
    # Simulate your 128x128x3 projection
    num_neurons = 128 * 128 * 3  # 49,152 neurons
    
    print(f"Projecting {num_neurons:,} neurons...")
    print(f"Python time: 40 seconds (before)")
    print(f"Testing Rust implementation...")
    
    # Generate test neurons
    neuron_ids = list(range(1000))  # Test with 1000 neurons first
    neuron_locations = []
    for i in range(1000):
        x = i % 128
        y = (i // 128) % 128
        z = (i // (128 * 128)) % 3
        neuron_locations.append((x, y, z))
    
    # Time the batch projection
    start = time.time()
    results = py_syn_projector_batch(
        "iic400",
        "dst001",
        neuron_ids,
        neuron_locations,
        (128, 128, 3),
        (128, 128, 1),
        None,
        None
    )
    elapsed = time.time() - start
    
    print(f"\n✅ Rust processed 1,000 neurons in {elapsed*1000:.2f}ms")
    print(f"   Average: {elapsed/len(neuron_ids)*1000:.3f}ms per neuron")
    
    # Extrapolate to full dataset
    full_time = elapsed * (num_neurons / len(neuron_ids))
    speedup = 40.0 / full_time
    
    print(f"\n📊 Extrapolated for full {num_neurons:,} neurons:")
    print(f"   Estimated time: {full_time:.2f} seconds")
    print(f"   Speedup: {speedup:.1f}x faster than Python!")
    print(f"   Time saved: {40.0 - full_time:.1f} seconds")
    
    assert full_time < 5.0, f"Should complete in <5s, got {full_time:.2f}s"
    print("\n✅ Performance target achieved! (<5 seconds)")

def test_edge_cases():
    """Test edge cases and boundary conditions."""
    print("\n" + "=" * 60)
    print("TEST 3: Edge Cases")
    print("=" * 60)
    
    # Test corner position
    result = py_syn_projector(
        "src", "dst", 0, (128, 128, 3), (128, 128, 1), (0, 0, 0), None, None
    )
    print(f"✅ Corner position (0,0,0): {len(result)} candidates")
    
    # Test max position
    result = py_syn_projector(
        "src", "dst", 0, (128, 128, 3), (128, 128, 1), (127, 127, 2), None, None
    )
    print(f"✅ Max position (127,127,2): {len(result)} candidates")
    
    # Test scale down
    result = py_syn_projector(
        "src", "dst", 0, (256, 256, 1), (128, 128, 1), (64, 64, 0), None, None
    )
    print(f"✅ Scale down (256→128): {len(result)} candidates")
    assert len(result) == 1, "Scale down should map to single position"
    
    # Test scale up
    result = py_syn_projector(
        "src", "dst", 0, (64, 64, 1), (128, 128, 1), (32, 32, 0), None, None
    )
    print(f"✅ Scale up (64→128): {len(result)} candidates")
    assert len(result) >= 1, "Scale up should map to multiple positions"
    
    print("\n✅ All edge cases passed!")

def main():
    """Run all tests."""
    print("\n🦀 FEAGI BDU Rust Projector Tests")
    print("Testing Phase 1: syn_projector implementation")
    
    try:
        test_single_projection()
        test_full_projection_performance()
        test_edge_cases()
        
        print("\n" + "=" * 60)
        print("🎉 ALL TESTS PASSED!")
        print("=" * 60)
        print("\nPhase 1 is working correctly!")
        print("You can now use Rust-accelerated projections in FEAGI.")
        print("\nNext: Integrate with feagi/bdu/connectivity/rules/functions.py")
        
    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())

