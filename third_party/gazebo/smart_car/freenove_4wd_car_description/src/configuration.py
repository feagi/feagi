#!/usr/bin/env python3

router_settings = {
    "feagi_ip": "feagi",
    "feagi_port": "30000",
    "ipu_port": "30001",
    'global_timer': 0.5
}

models_properties = {
    'motor': {
        'count': 4,
        'topic_identifier': '/M'
    },
    'servo': {
        'count': 2,
        'topic_identifier': '/S'
    }
}
