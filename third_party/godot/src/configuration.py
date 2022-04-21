#!/usr/bin/env python3

app_name = 'godot'

network_settings = {
    "feagi_host": "feagi",
    "feagi_api_port": "8000",
    "feagi_outbound_port": "30000",
    "feagi_inbound_port_godot": "30001",
    'TTL': 2,
    'last_message': 0,
    'godot_websocket_port': 9050
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
