#!/usr/bin/env python3
"""
Test power injection on a running FEAGI instance.

This test connects to a running FEAGI and validates:
1. _power cortical area exists
2. Power neurons are present
3. They appear in the fire queue (FCL)
"""

import requests
import pytest

FEAGI_BASE_URL = "http://127.0.0.1:8000"

def test_power_area_exists():
    """Test that _power cortical area exists in running FEAGI."""
    response = requests.get(f"{FEAGI_BASE_URL}/v1/connectome/cortical_area/_power/neurons")
    
    assert response.status_code == 200, f"❌ Failed to get _power neurons: {response.status_code}"
    
    neurons = response.json()
    assert isinstance(neurons, list), f"❌ Expected list, got {type(neurons)}"
    assert len(neurons) > 0, "❌ _power area has no neurons"
    
    print(f"\n✅ _power area exists with {len(neurons)} neurons")
    print(f"   Neuron IDs: {[n['id'] for n in neurons]}")
    
    return neurons

def test_power_neurons_in_fcl():
    """Test that power neurons appear in the Fire Queue (FCL)."""
    
    # First, verify power neurons exist
    power_neurons_response = requests.get(f"{FEAGI_BASE_URL}/v1/connectome/cortical_area/_power/neurons")
    assert power_neurons_response.status_code == 200
    
    power_neurons = power_neurons_response.json()
    power_neuron_ids = [int(n['id']) for n in power_neurons]
    
    print(f"\n📋 Power neuron IDs from _power area: {power_neuron_ids}")
    
    # Get current FCL (Fire Queue)
    fcl_response = requests.get(f"{FEAGI_BASE_URL}/v1/burst_engine/fcl")
    
    if fcl_response.status_code != 200:
        pytest.fail(f"❌ Failed to get FCL: {fcl_response.status_code}")
    
    fcl_data = fcl_response.json()
    
    print(f"\n📊 FCL Data:")
    print(f"   Timestep: {fcl_data.get('timestep')}")
    print(f"   Total neurons: {fcl_data.get('total_neurons')}")
    print(f"   Active cortical areas: {fcl_data.get('active_cortical_count')}")
    
    global_fcl = fcl_data.get('global_fcl', [])
    
    if len(global_fcl) == 0:
        print(f"\n⚠️  WARNING: No neurons in FCL (total_neurons=0)")
        print(f"   This means:")
        print(f"   1. Power neurons are NOT being injected, OR")
        print(f"   2. Power neurons are injected but NOT firing (threshold issue), OR")
        print(f"   3. No bursts have been processed yet")
        pytest.fail("❌ No neurons in FCL - power injection not working!")
    
    # Check if power neurons are in the global FCL
    power_neurons_in_fcl = [nid for nid in power_neuron_ids if nid in global_fcl]
    
    print(f"\n🔍 Power Neuron Analysis:")
    print(f"   Power neurons expected: {power_neuron_ids}")
    print(f"   Power neurons in FCL: {power_neurons_in_fcl}")
    print(f"   All neurons in FCL (sample): {global_fcl[:20] if len(global_fcl) > 20 else global_fcl}")
    
    if len(power_neurons_in_fcl) == 0:
        print(f"\n❌ POWER INJECTION FAILURE:")
        print(f"   - Power neurons exist in _power area: {power_neuron_ids}")
        print(f"   - But they are NOT in the Fire Queue")
        print(f"   - This means power neurons are not being injected or not firing")
        pytest.fail(f"❌ Power neurons not found in FCL!")
    
    print(f"\n✅ SUCCESS: {len(power_neurons_in_fcl)}/{len(power_neuron_ids)} power neurons in FCL")
    
    return power_neurons_in_fcl

if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("🧪 TESTING POWER INJECTION ON RUNNING FEAGI")
    print("=" * 80)
    
    try:
        # Test 1
        print("\n[TEST 1] Checking _power area...")
        test_power_area_exists()
        
        # Test 2
        print("\n[TEST 2] Checking power neurons in FCL...")
        test_power_neurons_in_fcl()
        
        print("\n" + "=" * 80)
        print("✅ ALL TESTS PASSED")
        print("=" * 80 + "\n")
        
    except AssertionError as e:
        print(f"\n❌ TEST FAILED: {e}")
        print("=" * 80 + "\n")
        exit(1)
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        print("=" * 80 + "\n")
        exit(1)

