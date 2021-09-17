"""
Defines the controller properties

Properties.mode [stand_alone, ros, virtual]
"""


controller_settings = {
    'sockets': {
        'general': {
            'burst_beacon': 'tcp://127.0.0.1:30000',
            'opu_channel': 'tcp://127.0.0.1:23000'
        },
        'pub': {
            'ultrasonic': 'tcp://0.0.0.0:23000',
            'IR1': 'tcp://0.0.0.0:23101',
            'IR2': 'tcp://0.0.0.0:23102',
            'IR3': 'tcp://0.0.0.0:23103',
            },
        'sub': {
            'M1': 'tcp://127.0.0.1:22001',
            'M2': "tcp://127.0.0.1:22002"
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
