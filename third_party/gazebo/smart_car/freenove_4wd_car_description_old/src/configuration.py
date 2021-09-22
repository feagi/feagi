#!/usr/bin/env python3

router_settings = {
    "feagi_ip": "feagi",
    "feagi_port": "30000",
    "ipu_port": "30001",
    'ros_topics': {
        'pub': ['M1', 'M2', 'M3', 'M4', 'Srv1', 'Srv2'],
        'sub': ['IR1', 'IR2', 'IR3', 'Ultrasonic']

        },
    'global_timer': 0.5
}
