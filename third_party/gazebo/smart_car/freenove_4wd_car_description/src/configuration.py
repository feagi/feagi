#!/usr/bin/env python3

router_settings = {
    "feagi_ip": "127.0.0.1",
    "feagi_outbound_port": "30000",
    "feagi_inbound_port": "30002",
    'TTL': 2,
    'last_message': 0,
    'feagi_burst_speed':  1
}

capabilities = {
    "servo": {
        "type": "opu",
        "disabled": True,
        "refresh_rate": 1,
        "cortical_mapping": "o__ser"
    },
    "motor": {
        "type": "opu",
        "disabled": False,
        "count": 4,
        "refresh_rate": 1,
        "cortical_mapping": "o__mot"
    },
    "infrared": {
        "type": "ipu",
        "disabled": False,
        "count": 4,
        "refresh_rate": 1,
        "cortical_mapping": "i__inf"
    },
    "battery": {
        "type": "ipu",
        "disabled": False,
        "count": 4,
        "refresh_rate": 1,
        "cortical_mapping": "i__bat"
    }
}


message_to_feagi = {}

