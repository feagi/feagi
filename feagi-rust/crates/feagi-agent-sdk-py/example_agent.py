#!/usr/bin/env python3
"""
Simple example agent using the Rust-backed Python SDK
"""

import time
from feagi_agent_sdk_py import PyAgentClient, PyAgentConfig, AgentType

def main():
    print("🤖 FEAGI Agent SDK - Python Example")
    print("=" * 50)
    
    # Create configuration
    print("\n📝 Creating configuration...")
    config = PyAgentConfig("python_test_agent", AgentType.Sensory)
    config.with_feagi_host("localhost")
    config.with_vision_capability("camera", 10, 10, 1, "i_vision")
    config.with_heartbeat_interval(5.0)
    config.with_connection_timeout_ms(5000)
    config.with_registration_retries(3)
    
    print(f"   ✅ Agent ID: python_test_agent")
    print(f"   ✅ Heartbeat: 5s")
    print(f"   ✅ Capabilities: 10x10 grayscale vision")
    
    # Validate configuration
    config.validate()
    print("   ✅ Configuration validated")
    
    # Create client
    print("\n🔌 Creating client...")
    client = PyAgentClient(config)
    print(f"   ✅ Client created: {client.agent_id()}")
    
    # Connect to FEAGI
    print("\n🌐 Connecting to FEAGI...")
    try:
        client.connect()
        print(f"   ✅ Connected!")
        print(f"   ✅ Registered: {client.is_registered()}")
    except Exception as e:
        print(f"   ❌ Connection failed: {e}")
        print("\n💡 Make sure Python FEAGI is running:")
        print("   cd /Users/nadji/code/FEAGI-2.0/feagi_core")
        print("   python main.py")
        return
    
    # Send data
    print("\n📤 Sending sensory data...")
    print("   (Press Ctrl+C to stop)")
    
    try:
        frame_count = 0
        while True:
            # Generate sample data (simulating 10x10 image)
            neuron_pairs = []
            for i in range(100):
                potential = float((i + frame_count) % 100)
                neuron_pairs.append((i, potential))
            
            # Send to FEAGI
            client.send_sensory_data(neuron_pairs)
            
            frame_count += 1
            if frame_count % 10 == 0:
                print(f"   📊 Sent {frame_count} frames")
            
            time.sleep(0.1)  # ~10 FPS
            
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n✅ Client will auto-deregister on exit")
    print("👋 Goodbye!")

if __name__ == "__main__":
    main()

