import feagi_rust_py_libs as frpl
from .cache.sensors_proxy import SensorsProxy
from .cache.motors_proxy import MotorsProxy

class FeagiAgent:

    def __init__(self):
        self._rust_agent: frpl.connector_core.caching.IOCache = frpl.connector_core.caching.IOCache()
        self.sensor_devices = SensorsProxy(self._rust_agent)
        self.motor_devices = MotorsProxy(self._rust_agent)
        