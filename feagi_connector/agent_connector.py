from src.feagi_interfaces import FEAGIConnectionStatus, AbstractFeagiInterface, DummyFeagiInterface # wtf is going on here
from src.cache import SensorCache


class FeagiAgentConnector:
    _server: AbstractFeagiInterface
    _py_sensor_cache: SensorCache

    def __init__(self, *args, **kwargs):
        raise RuntimeError("Direct instantiation not allowed, use one of the 'create' methods instead.")

    def __new__(cls, *args, **kwargs):
        raise RuntimeError("Direct instantiation not allowed, use one of the 'create' methods instead.")

    @staticmethod
    def _internal_init(server_backend: AbstractFeagiInterface):
        # We don't want users to be calling init directly
        if server_backend.get_current_connectivity_status() != FEAGIConnectionStatus.DISCONNECTED:
            raise Exception("Unable to start Feagi Agent with a running server!")
        obj = object.__new__(FeagiAgentConnector)
        obj._server = server_backend
        obj._py_sensor_cache = SensorCache()
        return obj


    @staticmethod
    def create_dummy_connector() -> "FeagiAgentConnector":
        return FeagiAgentConnector._internal_init(DummyFeagiInterface())

    @property
    def server(self) -> AbstractFeagiInterface:
        return self._server

    @property
    def sensors(self) -> SensorCache:
        return self._py_sensor_cache

    def encode_cache_to_bytes(self):
        self._py_sensor_cache.encode_cached_data_into_bytes()

    def get_most_recent_sensor_bytes(self) -> bytes:
        return self._py_sensor_cache.get_most_recent_sensor_bytes()

