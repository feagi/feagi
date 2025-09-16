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
        "segmented_image_camera": {
            "register_name": "register_image_camera_with_peripheral",
            "write_name": "store_image_camera_with_peripheral"
        }
    }
}

class SensorCache:
    def __init__(self):
        self._rust_cache: frpl.connector_core.SensorCache = frpl.connector_core.SensorCache()

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
        self._rust_cache.encode_cached_data_into_bytes()

    def get_most_recent_sensor_bytes(self) -> bytes:
        list_bytes = self._rust_cache.retrieve_latest_bytes()
        return bytes(list_bytes)