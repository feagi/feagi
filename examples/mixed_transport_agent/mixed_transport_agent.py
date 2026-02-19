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

import json
import os
import sys
from typing import Dict, Optional
from urllib import error as url_error
from urllib import request as url_request

from feagi.pns.client import AgentType, FeagiAgentClient


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
        available = [t for t in self.transports if t.enabled]

        if not available:
            return None

        if preference:
            transport = self.get_transport(preference)
            if transport:
                return transport

        transport = self.get_transport(self.recommended)
        if transport:
            return transport

        return available[0] if available else None


def register_with_feagi(
    feagi_host: str,
    feagi_port: int,
    agent_id: str,
    agent_type: str,
    capabilities: Dict,
    agent_data_port: int,
    agent_version: str,
    controller_version: str,
) -> RegistrationInfo:
    """Register agent with FEAGI over HTTP and get transport options."""

    print(f"Registering agent '{agent_id}' with FEAGI...")

    payload = {
        "agent_id": agent_id,
        "agent_type": agent_type,
        "agent_data_port": agent_data_port,
        "agent_version": agent_version,
        "controller_version": controller_version,
        "capabilities": capabilities,
    }

    url = f"http://{feagi_host}:{feagi_port}/v1/agent/register"
    body = json.dumps(payload).encode("utf-8")
    req = url_request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    try:
        with url_request.urlopen(req, timeout=10) as response:
            response_body = json.loads(response.read().decode("utf-8"))
    except url_error.HTTPError as err:
        raise Exception(f"Registration failed: HTTP {err.code}") from err
    except url_error.URLError as err:
        raise Exception(f"Registration failed: {err}") from err

    if response_body.get("status") != "success":
        error_msg = response_body.get("message", "Unknown error")
        raise Exception(f"Registration failed: {error_msg}")

    print(f"Registration successful: {response_body.get('message', '')}\n")

    return RegistrationInfo(response_body)


def connect_with_zmq(
    transport: TransportOption,
    agent_id: str,
    agent_type: str,
    capabilities: Dict,
    auth_token_b64: str,
    heartbeat_interval_s: float,
    connection_timeout_ms: int,
    registration_retries: int,
) -> FeagiAgentClient:
    """Connect to FEAGI using ZMQ transport via Rust SDK."""

    print("Connecting via ZMQ (Rust SDK)...")
    print(f"   Registration: tcp://{transport.host}:{transport.ports['registration']}")
    print(f"   Sensory:      tcp://{transport.host}:{transport.ports['sensory']}")
    print(f"   Motor:        tcp://{transport.host}:{transport.ports['motor']}")

    agent_type_map = {
        "sensory": AgentType.SENSORY,
        "motor": AgentType.MOTOR,
        "both": AgentType.BOTH,
    }
    rust_agent_type = agent_type_map.get(agent_type, AgentType.BOTH)

    client = FeagiAgentClient(agent_id, rust_agent_type)
    client.configure(
        feagi_host=transport.host,
        registration_port=int(transport.ports["registration"]),
        sensory_port=int(transport.ports["sensory"]),
        motor_port=int(transport.ports["motor"]),
        custom_capabilities=capabilities,
        auth_token_b64=auth_token_b64,
        heartbeat_interval=float(heartbeat_interval_s),
        connection_timeout_ms=int(connection_timeout_ms),
        registration_retries=int(registration_retries),
    )
    client.connect()

    print("ZMQ client connected\n")
    return client


async def connect_with_websocket(transport: TransportOption, agent_id: str):
    """Connect to FEAGI using WebSocket transport."""

    print("Connecting via WebSocket...")
    print(f"   Sensory: ws://{transport.host}:{transport.ports['sensory']}/sensory")
    print(f"   Motor:   ws://{transport.host}:{transport.ports['motor']}/motor/{agent_id}")

    try:
        import websockets
    except ImportError:
        print("websockets library not installed. Run: pip install websockets")
        return None, None

    # Sensory connection
    ws_sensory = await websockets.connect(
        f"ws://{transport.host}:{transport.ports['sensory']}/sensory"
    )

    # Motor connection
    ws_motor = await websockets.connect(
        f"ws://{transport.host}:{transport.ports['motor']}/motor/{agent_id}"
    )

    print("WebSocket connections established\n")

    return ws_sensory, ws_motor


def require_env(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise RuntimeError(f"Missing required environment variable: {name}")
    return value


def parse_env_int(name: str) -> int:
    value = require_env(name)
    return int(value)


def parse_env_float(name: str) -> float:
    value = require_env(name)
    return float(value)


def main():
    print("=" * 70)
    print("FEAGI Multi-Transport Agent Example")
    print("=" * 70)
    print()

    feagi_host = require_env("FEAGI_API_HOST")
    feagi_port = parse_env_int("FEAGI_API_PORT")
    agent_data_port = parse_env_int("AGENT_DATA_PORT")
    agent_version = require_env("AGENT_VERSION")
    controller_version = require_env("CONTROLLER_VERSION")
    heartbeat_interval_s = parse_env_float("FEAGI_HEARTBEAT_INTERVAL_S")
    connection_timeout_ms = parse_env_int("FEAGI_CONNECTION_TIMEOUT_MS")
    registration_retries = parse_env_int("FEAGI_REGISTRATION_RETRIES")
    auth_token_b64 = require_env("FEAGI_AUTH_TOKEN_B64")

    agent_id = require_env("AGENT_DESCRIPTOR_B64")
    agent_type = require_env("AGENT_TYPE")
    capabilities = json.loads(require_env("AGENT_CAPABILITIES_JSON"))

    try:
        reg_info = register_with_feagi(
            feagi_host,
            feagi_port,
            agent_id,
            agent_type,
            capabilities,
            agent_data_port,
            agent_version,
            controller_version,
        )

        print("Available Transports:")
        for transport in reg_info.transports:
            status = "yes" if transport.enabled else "no"
            print(f"   {status} {transport.transport_type.upper()}")
            for stream, port in transport.ports.items():
                print(f"      - {stream}: {port}")
        print()

        print(f"FEAGI recommends: {reg_info.recommended}")
        print()

        print("Scenario A: Auto-select (recommended)")
        auto_transport = reg_info.choose_transport()
        if auto_transport:
            print(f"   Selected: {auto_transport.transport_type}")
        print()

        print("Scenario B: Prefer WebSocket")
        ws_transport = reg_info.choose_transport("websocket")
        if ws_transport:
            print(f"   Selected: {ws_transport.transport_type}")
            print(f"   Would connect to: ws://{ws_transport.host}:{ws_transport.ports.get('sensory', 0)}")
        else:
            print("   WebSocket not available")
        print()

        print("Scenario C: Force ZMQ (high performance)")
        zmq_transport = reg_info.choose_transport("zmq")
        if zmq_transport:
            print(f"   Selected: {zmq_transport.transport_type}")
            print(f"   Would connect to: tcp://{zmq_transport.host}:{zmq_transport.ports.get('sensory', 0)}")

            try:
                client = connect_with_zmq(
                    zmq_transport,
                    agent_id,
                    agent_type,
                    capabilities,
                    auth_token_b64,
                    heartbeat_interval_s,
                    connection_timeout_ms,
                    registration_retries,
                )
                print("   Successfully connected via ZMQ!")
                client.disconnect()
            except Exception as e:
                print(f"   Could not connect: {e}")
        else:
            print("   ZMQ not available")
        print()

        print("Scenario D: Browser scenario (WebSocket only capable)")
        if ws_transport:
            print(f"   Browser would use: {ws_transport.transport_type}")
            print(f"   Visualization: ws://{ws_transport.host}:{ws_transport.ports.get('visualization', 0)}/visualization")
        else:
            print("   Browser cannot connect - WebSocket required but not available!")
        print()

    except Exception as e:
        print(f"Error: {e}")
        print("\nVerify FEAGI is running and environment is set:")
        print("   - FEAGI_API_HOST / FEAGI_API_PORT")
        print("   - AGENT_DESCRIPTOR_B64 / AGENT_TYPE / AGENT_CAPABILITIES_JSON")
        return 1

    print("=" * 70)
    print("Transport selection demonstration complete.")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
