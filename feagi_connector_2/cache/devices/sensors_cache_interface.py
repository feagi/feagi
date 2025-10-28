import feagi_rust_py_libs as frpl

class SensorsCacheInterface:

    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache):
        self._encode_to_neurons_then_bytes_function = getattr(io_cache_ref, "sensors_encode_cached_data_to_bytes")
        self._copy_out_feagi_byte_container_function = getattr(io_cache_ref, "sensor_get_byte_container")

    def encode_cached_values_to_bytes(self):
        self._encode_to_neurons_then_bytes_function()

    def copy_out_encoded_bytes(self) -> bytes:
        fbc: frpl.data_serialization.FeagiByteContainer = self._copy_out_feagi_byte_container_function()
        return fbc.copy_out_as_byte_vector()