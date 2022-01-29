"""
Defines the controller properties

Properties.mode [stand_alone, ros, virtual]
"""

network_settings = {
    "feagi_ip": "127.0.0.1",
    "feagi_outbound_port": "30000",
    "feagi_inbound_port": "30001"
}

capabilities = {
    "stimulation": {
        "disabled": False
    },
    "motor": {
        "disabled": True,
        "count": 4
    }
}
