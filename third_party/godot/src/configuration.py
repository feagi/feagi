#!/usr/bin/env python3

network_settings = {
    "feagi_ip": "127.0.0.1",
    "feagi_outbound_port": "30000",
    "feagi_inbound_port_godot": "30001",
    'TTL': 2,
    'last_message': 0,
    'feagi_burst_speed':  1,
    "host_name": "",
    "ip_address": ""
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
