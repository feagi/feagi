import feagi_rust_py_libs as frpl
from .sensor_types import SensorDevice

# TODO TEMP, WE SHOULD READ FROM RUST TEMPLATE INSTEAD
temp_sensors_dict = {
    "ImageFrame": {
        "image_camera_center" : {
            "register_name": "register_image_frame",
            "write_name": "store_image_frame"
        },
    },
    "SegmentedImageFrame": {
        "image_camera_with_peripheral": {
            "register_name": "register_image_camera_with_peripheral",
            "write_name": "store_image_camera_with_peripheral"
        }
    }
}

class SensorCache:
    def __init__(self):
        try:
            # Try the new API
            self._rust_cache = frpl.connector_core.IOCache()
        except AttributeError:
            try:
                # Fallback to old API
                self._rust_cache = frpl.connector_core.SensorCache()
            except AttributeError:
                # Create a minimal cache stub
                self._rust_cache = None

        for sensor_type, sensors in temp_sensors_dict.items():
            for sensor_name, sensor_details in sensors.items():
                register_name = sensor_details["register_name"]
                write_name = sensor_details["write_name"]

                # Map sensor_name to Rust SensorCorticalType if known
                sensor_enum = None
                try:
                    sensor_enum = getattr(frpl.data_structures.genomic.SensorCorticalType, ''.join(part.capitalize() for part in sensor_name.split('_')))
                except Exception:
                    sensor_enum = None

                instance = SensorDevice(self, register_name, write_name, sensor_enum)
                setattr(self, sensor_name, instance)


    def encode_cached_data_into_bytes(self):
        if self._rust_cache:
            try:
                # Try new API method name
                self._rust_cache.sensor_encode_cached_data_into_bytes()
            except AttributeError:
                try:
                    # Try old API method name
                    self._rust_cache.encode_cached_data_into_bytes()
                except AttributeError:
                    # Skip if neither method exists
                    pass

    def get_most_recent_sensor_bytes(self) -> bytes:
        if self._rust_cache:
            list_bytes = self._rust_cache.retrieve_latest_bytes()
            return bytes(list_bytes)
        return b""