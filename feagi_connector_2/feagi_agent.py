import feagi_rust_py_libs as frpl
from cache.sensors_proxy import SensorsProxy

class FeagiAgent:

    def __init__(self):
        self_rust_agent: frpl.connector_core.data.IOCache = frpl.connector_core.data.IOCache()
        self.sensor_devices = SensorsProxy(self_rust_agent)
        