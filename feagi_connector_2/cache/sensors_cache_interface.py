import feagi_rust_py_libs as frpl

class SensorsCacheInterface:

    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache):
        self._encode_to_neurons_then_bytes_function = getattr(io_cache_ref, "sensors_encode_cached_data_to_bytes")
        self._copy_out_feagi_byte_container_function = getattr(io_cache_ref, "sensor_get_byte_container")

    async def send_brain_input_to_feagi(self) -> None:
        self.brain_input_cache.encode_cached_values_to_bytes()
        byte_data: bytes = self.brain_input_cache.copy_out_encoded_bytes()

        # Magic Send Function! #TODO
        print("Poof!")


    def adv_encode_cached_values_to_bytes(self) -> None:
        self._encode_to_neurons_then_bytes_function()

    def adv_copy_out_encoded_bytes(self) -> bytes:
        fbc: frpl.data_serialization.FeagiByteContainer = self._copy_out_feagi_byte_container_function()
        return fbc.copy_out_as_byte_vector()