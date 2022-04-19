"""
Defines the controller properties

Properties.mode [stand_alone, ros, virtual]
"""

network_settings = {
    "feagi_ip": "127.0.0.1",
    "feagi_api_port": "8000",
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
