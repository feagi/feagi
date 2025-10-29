import feagi_rust_py_libs as frpl
from pathlib import Path
from .cache.sensors_proxy import SensorsProxy
from .cache.motors_proxy import MotorsProxy
from feagi_connector_2.cache.sensors_cache_interface import SensorsCacheInterface
from feagi_connector_2.cache.motors_cache_interface import MotorsCacheInterface
from .premade_feedbacks import PreMadeFeedBacks
from .agent_state import AgentState
from .feagi_interface import FeagiInterface

class FeagiAgent:

    def __init__(self):
        self._rust_agent: frpl.connector_core.caching.IOCache = frpl.connector_core.caching.IOCache()
        self._agent_state: AgentState = AgentState.NotConnected
        self._interface: FeagiInterface = FeagiInterface()

        self.brain_input = SensorsProxy(self._rust_agent)
        self.brain_output = MotorsProxy(self._rust_agent)

        self.brain_input_cache = SensorsCacheInterface(self._rust_agent)
        self.brain_output_cache = MotorsCacheInterface(self._rust_agent)
        self.premade_feedbacks = PreMadeFeedBacks(self._rust_agent)


    def get_agent_state(self) -> AgentState:
        return self._agent_state

    def load_and_connect_from_config_file(self, load_path: Path) -> None: # same return as connect functions
        pass

    def save_current_config_to_file(self, save_path: Path) -> None: # Success?
        pass


