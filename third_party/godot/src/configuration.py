#!/usr/bin/env python3

import os

app_name = 'godot'

feagi_settings = {
    "feagi_host": os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1"),
    "feagi_api_port": os.environ.get('FEAGI_API_PORT', "8000"),
}

host_info = {
    "ip_address": os.environ.get('GADOT_HOST') 
}

agent_settings = {
    "agent_data_port": "30001",
    "agent_id": "godot",
    "agent_type": "monitor",
    'TTL': 2,
    'last_message': 0,
    'godot_websocket_ip': "0.0.0.0",
    'godot_websocket_port': os.environ.get('WS_BRIDGE_PORT', "9050"),
    'burst_duration_threshold': 0.002
}

capabilities = {
    'motor': {
        'count': 4,
        'topic_identifier': '/M',
        'motor_statuses': {}
    },
    'servo': {
        'count': 2,
        'topic_identifier': '/S'
    },
    'infrared': {
        'count': 3,
        'topic_identifier': 'IR'
    }
}

message_to_feagi = {"data": {}}
