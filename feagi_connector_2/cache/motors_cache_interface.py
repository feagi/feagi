import feagi_rust_py_libs as frpl

class MotorsCacheInterface:

    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache):
        self._load_byte_container_function = getattr(io_cache_ref, "motor_set_byte_container")
        self._decode_byte_container_function = getattr(io_cache_ref, "encode_data_from_bytes_to_cache")

    def load_bytes(self, byte_data: bytes):
        fbc: frpl.data_serialization.FeagiByteContainer = frpl.data_serialization.FeagiByteContainer()
        fbc.load_bytes_and_verify(byte_data)

        self._load_byte_container_function(fbc)

    def decode_bytes_into_cache(self):
        self._decode_byte_container_function()