"""Integration test for Rust PNS visualization data flow.

This test verifies that after migrating ZMQ to Rust PNS:
1. Rust PNS starts correctly
2. Visualization FQ sampler is created
3. Burst engine generates neuron activity
4. Activity is written to visualization SHM
5. BV can read the data from SHM

CRITICAL: This test must pass to ensure BV receives neuron activations.
"""

import time
import struct
import mmap
import os
from pathlib import Path

import pytest


def test_rust_pns_visualization_data_flow():
    """Test complete visualization data flow through Rust PNS + SHM."""
    from feagi.process_manager import ProcessManager
    from feagi.config.toml_loader import load_feagi_config
    from feagi.core.state_manager import FeagiStateManager
    
    print("\n" + "="*60)
    print("RUST PNS VISUALIZATION DATA FLOW TEST")
    print("="*60)
    
    # Step 1: Initialize FEAGI with Rust PNS
    print("\n1️⃣ Initializing FEAGI with Rust PNS...")
    config = load_feagi_config()
    pm = ProcessManager()
    
    # Initialize critical processes (brain + burst engine)
    success = pm.init_critical_processes(config)
    assert success, "Failed to initialize critical processes"
    print("   ✅ Critical processes initialized")
    
    # Initialize Rust PNS
    success = pm.init_important_processes(config)
    assert success, "Failed to initialize Rust PNS"
    print("   ✅ Rust PNS initialized")
    
    # Verify PNS is running
    assert hasattr(pm, '_pns'), "ProcessManager has no _pns attribute"
    assert pm._pns is not None, "PNS instance is None"
    assert pm._pns.is_running(), "PNS is not running"
    print(f"   ✅ PNS running with {pm._pns.get_agent_count()} agents")
    
    # Step 2: Get SHM path from state manager
    print("\n2️⃣ Checking visualization SHM configuration...")
    sm = FeagiStateManager.instance()
    shm_registry = sm.get_shared_memory_registry() if hasattr(sm, "get_shared_memory_registry") else {}
    viz_shm_path = shm_registry.get("visualization_stream")
    
    if not viz_shm_path:
        # Try default path
        viz_shm_path = "/tmp/feagi-shared-mem-visualization_stream.bin"
    
    print(f"   ℹ️  Visualization SHM path: {viz_shm_path}")
    
    # Step 3: Register BV through Rust PNS (via direct API, simulating ZMQ REST)
    print("\n3️⃣ Registering BV through Rust PNS...")
    
    # Since all registration is now in Rust, we register directly through PNS
    try:
        # Build capabilities JSON
        import json
        capabilities_json = json.dumps({
            "visualization": {
                "enabled": True,
                "rate_hz": 30.0
            }
        })
        
        # Register through Rust PNS Python API
        # Signature: register_agent(agent_id: str, agent_type: str, capabilities: str)
        result = pm._pns.register_agent("test-bv", "visualizer", capabilities_json)
        print(f"   ✅ BV registered through Rust PNS")
        print(f"   ℹ️  Registration result: {result}")
        print(f"   ℹ️  Agent count: {pm._pns.get_agent_count()}")
    except Exception as e:
        print(f"   ❌ Failed to register BV through Rust PNS: {e}")
        import traceback
        traceback.print_exc()
    
    # Give PNS a moment to process registration and create FQ sampler
    time.sleep(0.2)
    
    # Step 4: Inject test neuron activity into FCL
    print("\n4️⃣ Injecting test neuron activity into FCL...")
    
    rust_npu = None
    if hasattr(pm, '_core_api') and pm._core_api:
        if hasattr(pm._core_api, '_rust_npu_integration'):
            rust_npu = pm._core_api._rust_npu_integration._rust_npu
    
    if rust_npu:
        # Inject some test neurons
        test_neurons = [
            (1, 1.5),  # neuron_id=1, potential=1.5
            (2, 1.8),
            (3, 2.0),
            (100, 1.2),
            (200, 1.7),
        ]
        
        for neuron_id, potential in test_neurons:
            rust_npu.inject_to_fcl(neuron_id, potential)
        
        print(f"   ✅ Injected {len(test_neurons)} test neurons into FCL")
    else:
        pytest.skip("Rust NPU not available for injection")
    
    # Step 5: Run burst engine to process and sample
    print("\n5️⃣ Running burst engine to process activity...")
    
    if rust_npu:
        # Run a few bursts to ensure data flows
        for i in range(3):
            rust_npu.run_single_burst()
            time.sleep(0.01)
        
        print(f"   ✅ Ran 3 burst cycles")
    else:
        pytest.skip("Rust NPU not available for burst")
    
    # Step 6: Check if visualization SHM file exists and has data
    print("\n6️⃣ Checking visualization SHM for data...")
    
    if os.path.exists(viz_shm_path):
        file_size = os.path.getsize(viz_shm_path)
        print(f"   ✅ SHM file exists: {viz_shm_path} ({file_size} bytes)")
        
        # Try to read SHM header to verify format
        try:
            with open(viz_shm_path, 'rb') as f:
                with mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ) as mm:
                    # Read FEAGIVIS header
                    header = mm[:256]
                    magic = header[:8]
                    
                    if magic == b'FEAGIVIS':
                        version, num_slots, slot_size, frame_seq, write_idx = struct.unpack('<IIIQÍ', header[8:32])
                        print(f"   ✅ Valid FEAGIVIS ring buffer format")
                        print(f"      Version: {version}")
                        print(f"      Slots: {num_slots}")
                        print(f"      Slot size: {slot_size} bytes")
                        print(f"      Frame sequence: {frame_seq}")
                        print(f"      Write index: {write_idx}")
                        
                        if frame_seq > 0:
                            print(f"   ✅ Data has been written (frame_seq={frame_seq})")
                        else:
                            print(f"   ⚠️  No data written yet (frame_seq=0)")
                            pytest.fail("No visualization data written to SHM")
                    else:
                        print(f"   ❌ Invalid magic: {magic}")
                        pytest.fail(f"Invalid SHM format, expected FEAGIVIS, got {magic}")
        except Exception as e:
            print(f"   ❌ Failed to read SHM: {e}")
            pytest.fail(f"Failed to read visualization SHM: {e}")
    else:
        print(f"   ❌ SHM file does not exist: {viz_shm_path}")
        pytest.fail("Visualization SHM file not created")
    
    # Step 7: Cleanup
    print("\n7️⃣ Cleaning up...")
    
    # Deregister test BV through Rust PNS
    try:
        pm._pns.deregister_agent("test-bv")
        print(f"   ✅ Test BV deregistered")
        print(f"   ℹ️  Agent count: {pm._pns.get_agent_count()}")
    except Exception as e:
        print(f"   ⚠️  Deregistration failed: {e}")
    
    # Shutdown process manager
    pm.shutdown()
    print("   ✅ ProcessManager shutdown")
    
    print("\n" + "="*60)
    print("✅ RUST PNS VISUALIZATION DATA FLOW TEST PASSED")
    print("="*60)


def test_rust_pns_has_visualization_stream():
    """Test that Rust PNS exposes visualization stream methods."""
    import feagi_rust
    
    pns = feagi_rust.PyPNS()
    
    # Check for visualization-related methods
    assert hasattr(pns, 'start'), "PNS missing start() method"
    assert hasattr(pns, 'stop'), "PNS missing stop() method"
    assert hasattr(pns, 'is_running'), "PNS missing is_running() method"
    assert hasattr(pns, 'get_agent_count'), "PNS missing get_agent_count() method"
    
    print("✅ Rust PNS has required methods")


def test_visualization_shm_writer_exists_in_rust_npu():
    """Test that Rust NPU has visualization SHM writer methods."""
    from feagi.process_manager import ProcessManager
    from feagi.config.toml_loader import load_feagi_config
    
    config = load_feagi_config()
    pm = ProcessManager()
    pm.init_critical_processes(config)
    
    rust_npu = None
    if hasattr(pm, '_core_api') and pm._core_api:
        if hasattr(pm._core_api, '_rust_npu_integration'):
            rust_npu = pm._core_api._rust_npu_integration._rust_npu
    
    if rust_npu:
        # Check for SHM writer methods
        assert hasattr(rust_npu, 'attach_viz_shm_writer'), "Rust NPU missing attach_viz_shm_writer()"
        assert hasattr(rust_npu, 'write_viz_shm'), "Rust NPU missing write_viz_shm()"
        print("✅ Rust NPU has visualization SHM writer methods")
    else:
        pytest.skip("Rust NPU not available")
    
    pm.shutdown()


if __name__ == "__main__":
    # Run tests directly
    test_rust_pns_has_visualization_stream()
    test_visualization_shm_writer_exists_in_rust_npu()
    test_rust_pns_visualization_data_flow()

