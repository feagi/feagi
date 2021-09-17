"""
Defines the controller properties

Properties.mode [stand_alone, ros, virtual]
"""


controller_settings = {
    'FEAGI_sockets': {
        'pub': {
            'ultrasonic': 20000
            },
        'sub': {
            'motor': "feagi:20003"
            }
        },
    'ros_topics': {
        'pub': ['M1', 'M2', 'M3', 'M4', 'Srv1', 'Srv2'],
        'sub': ['IR1', 'IR2', 'IR3', 'Ultrasonic']

    },
    'timers': {
        'global_timer': 0.5
    },
    'properties': {
        'mode': 'virtual'
    }
}
