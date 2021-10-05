#!/usr/bin/env python3

router_settings = {
    "feagi_ip": "feagi",
    "feagi_port": "30000",
    "ipu_port": "30001",
    'global_timer': 2,
    'TTL': 2,
    'last_message': 0
}

model_properties = {
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
