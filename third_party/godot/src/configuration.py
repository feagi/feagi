#!/usr/bin/env python3

app_name = 'godot'

feagi_settings = {
    "feagi_host": "feagi",
    "feagi_api_port": "8000",
}

agent_settings = {
    "agent_data_port": "30001",
    "agent_id": "godot",
    "agent_type": "monitor",
    'TTL': 2,
    'last_message': 0,
    'godot_websocket_ip': "192.168.50.218",
    'godot_websocket_port': 9050,
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
