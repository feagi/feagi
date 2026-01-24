#!/usr/bin/env python3
"""
Mixed Transport Agent Example

Demonstrates how to build an agent that can use either ZMQ or WebSocket
based on FEAGI's available transports.

This example shows:
1. Registering with FEAGI
2. Parsing available transports
3. Selecting appropriate transport
4. Connecting with chosen transport
"""

import zmq
import sys
from typing import Dict, Optional


class TransportOption:
    """Represents a transport option from FEAGI."""
    
    def __init__(self, data: Dict):
        self.transport_type = data.get("transport_type", "unknown")
        self.enabled = data.get("enabled", False)
        self.ports = data.get("ports", {})
        self.host = data.get("host", "0.0.0.0")
    
    def __repr__(self):
        return f"Transport({self.transport_type}, enabled={self.enabled}, ports={self.ports})"


class RegistrationInfo:
    """Parsed registration response from FEAGI."""
    
    def __init__(self, response_body: Dict):
        self.status = response_body.get("status", "unknown")
        self.message = response_body.get("message")
        self.zmq_ports = response_body.get("zmq_ports", {})
        self.transports = [
            TransportOption(t) 
            for t in response_body.get("transports", [])
        ]
        self.recommended = response_body.get("recommended_transport", "zmq")
    
    def get_transport(self, transport_type: str) -> Optional[TransportOption]:
        """Get specific transport by type."""
        for transport in self.transports:
            if transport.transport_type == transport_type and transport.enabled:
                return transport
        return None
    
    def choose_transport(self, preference: Optional[str] = None) -> Optional[TransportOption]:
        """Choose best transport based on preference."""
        # Filter enabled transports
        available = [t for t in self.transports if t.enabled]
        
        if not available:
            return None
        
        # Try preference first
        if preference:
            transport = self.get_transport(preference)
            if transport:
                return transport
        
        # Fall back to recommended
        transport = self.get_transport(self.recommended)
        if transport:
            return transport
        
        # Last resort: first available
        return available[0] if available else None


def register_with_feagi(
    feagi_host: str,
    registration_port: int,
    agent_id: str,
    agent_type: str,
    capabilities: Dict
) -> RegistrationInfo:
    """Register agent with FEAGI and get transport options."""
    
    print(f"📝 Registering agent '{agent_id}' with FEAGI...")
    
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f"tcp://{feagi_host}:{registration_port}")
    
    request = {
        "method": "POST",
        "path": "/v1/agent/register",
        "body": {
            "agent_id": agent_id,
            "agent_type": agent_type,
            "capabilities": capabilities
        }
    }
    
    socket.send_json(request)
    response = socket.recv_json()
    socket.close()
    
    # Check status
    status_code = response.get("status", 500)
    if status_code != 200:
        error_msg = response.get("body", {}).get("error", "Unknown error")
        raise Exception(f"Registration failed: {error_msg}")
    
    body = response.get("body", {})
    print(f"✅ Registration successful: {body.get('message', '')}\n")
    
    return RegistrationInfo(body)


def connect_with_zmq(transport: TransportOption, agent_id: str):
    """Connect to FEAGI using ZMQ transport."""
    
    print("🔌 Connecting via ZMQ...")
    print(f"   Sensory: tcp://{transport.host}:{transport.ports['sensory']}")
    print(f"   Motor:   tcp://{transport.host}:{transport.ports['motor']}")
    
    context = zmq.Context()
    
    # Sensory socket (PUSH)
    sensory = context.socket(zmq.PUSH)
    sensory.connect(f"tcp://{transport.host}:{transport.ports['sensory']}")
    
    # Motor socket (SUB)
    motor = context.socket(zmq.SUB)
    motor.connect(f"tcp://{transport.host}:{transport.ports['motor']}")
    motor.subscribe(b"")  # Subscribe to all
    
    print("✅ ZMQ sockets connected\n")
    
    return sensory, motor


async def connect_with_websocket(transport: TransportOption, agent_id: str):
    """Connect to FEAGI using WebSocket transport."""
    
    print("🔌 Connecting via WebSocket...")
    print(f"   Sensory: ws://{transport.host}:{transport.ports['sensory']}/sensory")
    print(f"   Motor:   ws://{transport.host}:{transport.ports['motor']}/motor/{agent_id}")
    
    try:
        import websockets
    except ImportError:
        print("❌ websockets library not installed. Run: pip install websockets")
        return None, None
    
    # Sensory connection
    ws_sensory = await websockets.connect(
        f"ws://{transport.host}:{transport.ports['sensory']}/sensory"
    )
    
    # Motor connection
    ws_motor = await websockets.connect(
        f"ws://{transport.host}:{transport.ports['motor']}/motor/{agent_id}"
    )
    
    print("✅ WebSocket connections established\n")
    
    return ws_sensory, ws_motor


def main():
    print("=" * 70)
    print("FEAGI Multi-Transport Agent Example")
    print("=" * 70)
    print()
    
    # Configuration
    feagi_host = "localhost"
    registration_port = 5563
    # agent_id must be a base64 AgentDescriptor (48-byte payload)
    agent_id = "<agent_descriptor_b64>"
    agent_type = "both"
    capabilities = {
        "sensory": {"rate_hz": 30.0},
        "motor": {"enabled": True, "output_count": 6}
    }
    
    try:
        # Step 1: Register with FEAGI
        reg_info = register_with_feagi(
            feagi_host,
            registration_port,
            agent_id,
            agent_type,
            capabilities
        )
        
        # Step 2: Display available transports
        print("🌐 Available Transports:")
        for transport in reg_info.transports:
            status = "✅" if transport.enabled else "❌"
            print(f"   {status} {transport.transport_type.upper()}")
            for stream, port in transport.ports.items():
                print(f"      - {stream}: {port}")
        print()
        
        print(f"💡 FEAGI recommends: {reg_info.recommended}")
        print()
        
        # Step 3: Choose transport based on scenario
        
        # Scenario A: Auto-select (use recommended)
        print("📌 Scenario A: Auto-select (recommended)")
        auto_transport = reg_info.choose_transport()
        if auto_transport:
            print(f"   Selected: {auto_transport.transport_type}")
        print()
        
        # Scenario B: Prefer WebSocket
        print("📌 Scenario B: Prefer WebSocket")
        ws_transport = reg_info.choose_transport("websocket")
        if ws_transport:
            print(f"   Selected: {ws_transport.transport_type}")
            print(f"   Would connect to: ws://{ws_transport.host}:{ws_transport.ports.get('sensory', 0)}")
        else:
            print("   ❌ WebSocket not available")
        print()
        
        # Scenario C: Force ZMQ
        print("📌 Scenario C: Force ZMQ (high performance)")
        zmq_transport = reg_info.choose_transport("zmq")
        if zmq_transport:
            print(f"   Selected: {zmq_transport.transport_type}")
            print(f"   Would connect to: tcp://{zmq_transport.host}:{zmq_transport.ports.get('sensory', 0)}")
            
            # Actually connect with ZMQ (if FEAGI is running)
            try:
                sensory, motor = connect_with_zmq(zmq_transport, agent_id)
                print("   🎉 Successfully connected via ZMQ!")
                sensory.close()
                motor.close()
            except Exception as e:
                print(f"   ⚠️  Could not connect: {e}")
        else:
            print("   ❌ ZMQ not available")
        print()
        
        # Scenario D: Browser simulation
        print("📌 Scenario D: Browser scenario (WebSocket only capable)")
        if ws_transport:
            print(f"   Browser would use: {ws_transport.transport_type}")
            print(f"   Visualization: ws://{ws_transport.host}:{ws_transport.ports.get('visualization', 0)}/visualization")
        else:
            print("   ❌ Browser cannot connect - WebSocket required but not available!")
        print()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\n💡 Make sure FEAGI is running with:")
        print("   - ZMQ registration on port 5563")
        print("   - WebSocket enabled in configuration")
        return 1
    
    print("=" * 70)
    print("✅ Transport selection demonstration complete!")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())

