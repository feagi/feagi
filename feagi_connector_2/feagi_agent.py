import feagi_rust_py_libs as frpl
from .cache.sensors_proxy import SensorsProxy
from .cache.motors_proxy import MotorsProxy
from feagi_connector_2.cache.sensors_cache_interface import SensorsCacheInterface
from feagi_connector_2.cache.motors_cache_interface import MotorsCacheInterface
from .premade_reflexes import PreMadeReflexes

class FeagiAgent:

    def __init__(self):
        self._rust_agent: frpl.connector_core.caching.IOCache = frpl.connector_core.caching.IOCache()
        self.brain_input = SensorsProxy(self._rust_agent)
        self.brain_output = MotorsProxy(self._rust_agent)

        self.brain_input_cache = SensorsCacheInterface(self._rust_agent)
        self.brain_output_cache = MotorsCacheInterface(self._rust_agent)
        self.premade_reflexes = PreMadeReflexes(self._rust_agent)

    def encode_sensor_data_and_send(self):
        self.brain_input_cache.encode_cached_values_to_bytes()
        byte_data: bytes = self.brain_input_cache.copy_out_encoded_bytes()

        # Magic Send Function! #TODO
        print("Poof!")

