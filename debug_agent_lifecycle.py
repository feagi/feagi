#!/usr/bin/env python3
"""
Debug script to test agent registration and cleanup behavior.

This script will help us understand why both agents are being removed
when only one should be.
"""

import requests
import time
import json

# FEAGI connection details
FEAGI_HOST = "localhost"
FEAGI_PORT = 8000
BASE_URL = f"http://{FEAGI_HOST}:{FEAGI_PORT}"

def register_test_agent(agent_id: str, agent_type: str, capabilities: dict):
    """Register a test agent with FEAGI."""
    registration_data = {
        "agent_id": agent_id,
        "agent_type": agent_type,
        "capabilities": capabilities,
        "metadata": {
            "version": "debug_test",
            "description": f"Debug test agent for {agent_type}"
        }
    }
    
    try:
        response = requests.post(
            f"{BASE_URL}/v1/agent/register",
            json=registration_data,
            timeout=10.0
        )
        
        if response.status_code == 200:
            print(f"✅ Successfully registered {agent_type} agent '{agent_id}'")
            return True
        else:
            print(f"❌ Failed to register {agent_type} agent '{agent_id}': {response.status_code} - {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ Error registering {agent_type} agent '{agent_id}': {e}")
        return False

def list_agents():
    """List all registered agents."""
    try:
        response = requests.get(f"{BASE_URL}/v1/agent/list", timeout=5.0)
        if response.status_code == 200:
            agents = response.json()
            print(f"📋 Current agents: {agents}")
            return agents
        else:
            print(f"❌ Failed to list agents: {response.status_code} - {response.text}")
            return []
    except Exception as e:
        print(f"❌ Error listing agents: {e}")
        return []

def send_heartbeat(agent_id: str):
    """Send a heartbeat for an agent."""
    heartbeat_data = {"agent_id": agent_id}
    
    try:
        response = requests.post(
            f"{BASE_URL}/v1/agent/heartbeat",
            json=heartbeat_data,
            timeout=5.0
        )
        
        if response.status_code == 200:
            print(f"💗 Heartbeat sent successfully for '{agent_id}'")
            return True
        else:
            print(f"💔 Heartbeat failed for '{agent_id}': {response.status_code} - {response.text}")
            return False
            
    except Exception as e:
        print(f"💔 Heartbeat error for '{agent_id}': {e}")
        return False

def deregister_agent(agent_id: str):
    """Deregister an agent."""
    deregistration_data = {"agent_id": agent_id}
    
    try:
        response = requests.delete(
            f"{BASE_URL}/v1/agent/deregister",
            json=deregistration_data,
            timeout=10.0
        )
        
        if response.status_code == 200:
            print(f"✅ Successfully deregistered agent '{agent_id}'")
            return True
        else:
            print(f"❌ Failed to deregister agent '{agent_id}': {response.status_code} - {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ Error deregistering agent '{agent_id}': {e}")
        return False

def main():
    """Run the debug test."""
    print("🧪 Starting agent registration debug test...")
    
    # 1. Register both agents
    print("\n1. Registering both agents...")
    
    brain_viz_registered = register_test_agent(
        "debug_brain_visualizer",
        "brain_visualizer", 
        {"visualization": True}
    )
    
    video_agent_registered = register_test_agent(
        "debug_video_agent",
        "video_agent",
        {"sensory": True, "video": True}
    )
    
    if not (brain_viz_registered and video_agent_registered):
        print("❌ Failed to register both agents, aborting test")
        return
    
    # 2. List agents to verify registration
    print("\n2. Listing agents after registration...")
    initial_agents = list_agents()
    
    # 3. Send heartbeats for both agents
    print("\n3. Sending heartbeats for both agents...")
    send_heartbeat("debug_brain_visualizer")
    send_heartbeat("debug_video_agent")
    
    # 4. Wait a bit, then list agents again
    print("\n4. Waiting 5 seconds, then listing agents again...")
    time.sleep(5)
    agents_after_heartbeat = list_agents()
    
    # 5. Deregister ONLY the video agent
    print("\n5. Deregistering ONLY the video agent...")
    deregister_agent("debug_video_agent")
    
    # 6. Immediately list agents to see if brain visualizer is still there
    print("\n6. Listing agents immediately after video agent deregistration...")
    agents_after_video_removal = list_agents()
    
    # 7. Wait 35 seconds to see if cleanup removes brain visualizer
    print("\n7. Waiting 35 seconds to see if cleanup removes brain visualizer...")
    for i in range(7):
        time.sleep(5)
        print(f"   ... {(i+1)*5}s elapsed")
        current_agents = list_agents()
        
        # Check if brain visualizer is still there
        if "debug_brain_visualizer" not in current_agents:
            print(f"🚨 PROBLEM DETECTED: Brain visualizer removed at {(i+1)*5}s after video agent removal!")
            break
    else:
        print("✅ Brain visualizer survived 35 seconds after video agent removal")
    
    # 8. Clean up remaining agents
    print("\n8. Cleaning up remaining agents...")
    final_agents = list_agents()
    for agent_id in final_agents:
        if agent_id.startswith("debug_"):
            deregister_agent(agent_id)
    
    print("🧪 Debug test completed!")

if __name__ == "__main__":
    main()



