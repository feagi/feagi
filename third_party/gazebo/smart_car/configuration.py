"""
Defines the controller properties

Properties.mode [stand_alone, ros, virtual]
"""

router_settings = {
    # "feagi_ip": "127.0.0.1",
    "feagi_ip": "feagi",
    "feagi_port": "30000",
    "ipu_port": "30001",
    'ros_topics': {
        'pub': ['M1', 'M2', 'M3', 'M4', 'Srv1', 'Srv2'],
        'sub': ['IR1', 'IR2', 'IR3', 'Ultrasonic']

        },
    'global_timer': 0.5
}
