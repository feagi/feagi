#!/usr/bin/env python3

import os

app_name = 'godot'

feagi_settings = {
    # "feagi_auth_url": "http://127.0.0.1:9000/v1/k8/feagi_settings/auth_token",
    "feagi_url": None,
    "feagi_dns": None,
    "feagi_host": os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1"),
    "feagi_api_port": os.environ.get('FEAGI_API_PORT', "8000"),
}

agent_settings = {
    "agent_data_port": "30000",
    "agent_id": "godot",
    "agent_type": "monitor",
    'TTL': 2,
    'last_message': 0,
    'godot_websocket_ip': "0.0.0.0",
    'godot_websocket_port': os.environ.get('WS_BRIDGE_PORT', "9050"),
    'burst_duration_threshold': 0.002
}

capabilities = {
}

message_to_feagi = {"data": {}}
