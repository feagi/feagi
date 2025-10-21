#!/usr/bin/env python3
"""
Quick test to verify Python bindings work correctly
"""

try:
    from feagi_agent_sdk_py import PyAgentClient, PyAgentConfig, AgentType
    
    print("✅ Import successful!")
    print(f"   - PyAgentClient: {PyAgentClient}")
    print(f"   - PyAgentConfig: {PyAgentConfig}")
    print(f"   - AgentType: {AgentType}")
    
    # Test configuration creation
    print("\n🔧 Testing configuration...")
    config = PyAgentConfig("test_agent", AgentType.Sensory)
    print(f"   ✅ Created config for agent: test_agent")
    
    # Test configuration methods
    config.with_feagi_host("localhost")
    print(f"   ✅ Set FEAGI host")
    
    config.with_vision_capability("camera", 64, 64, 1, "i_vision")
    print(f"   ✅ Added vision capability")
    
    config.with_heartbeat_interval(5.0)
    print(f"   ✅ Set heartbeat interval")
    
    # Test validation
    config.validate()
    print(f"   ✅ Configuration validated")
    
    # Test client creation (won't connect without FEAGI running)
    print("\n🤖 Testing client creation...")
    client = PyAgentClient(config)
    print(f"   ✅ Created client")
    
    # Test properties
    agent_id = client.agent_id()
    is_registered = client.is_registered()
    print(f"   ✅ Agent ID: {agent_id}")
    print(f"   ✅ Is registered: {is_registered}")
    
    print("\n🎉 All tests passed!")
    
except ImportError as e:
    print(f"❌ Import failed: {e}")
    print("\nTo fix:")
    print("  cd /Users/nadji/code/FEAGI-2.0/feagi_core/feagi-rust/crates/feagi-agent-sdk-py")
    print("  maturin develop --release")
except Exception as e:
    print(f"❌ Test failed: {e}")
    import traceback
    traceback.print_exc()

