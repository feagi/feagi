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
            "register_name": "sensor_register_segmented_vision_absolute",
            "write_name": "sensor_write_segmented_vision_absolute"
        }
    }
}

class SensorCache:
    def __init__(self):
        try:
            # Try the new API (correct path)
            self._rust_cache = frpl.connector_core.caching.IOCache()
        except AttributeError:
            try:
                # Try alternate path
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
        import logging
        logger = logging.getLogger(__name__)
        if self._rust_cache:
            # Log available methods for debugging
            available = [m for m in dir(self._rust_cache) if 'sensor' in m.lower() and 'encode' in m.lower()]
            logger.debug(f"[CACHE-ENCODE] Available encode methods: {available}")
            
            # IOCache API: sensors_encode_cached_data_to_bytes (plural!)
            if hasattr(self._rust_cache, 'sensors_encode_cached_data_to_bytes'):
                logger.debug("[CACHE-ENCODE] Calling sensors_encode_cached_data_to_bytes()")
                self._rust_cache.sensors_encode_cached_data_to_bytes()
            else:
                logger.warning(f"[CACHE-ENCODE] Method 'sensors_encode_cached_data_to_bytes' not found on {type(self._rust_cache)}")

    def get_most_recent_sensor_bytes(self) -> bytes:
        import logging
        logger = logging.getLogger(__name__)
        logger.debug(f"[CACHE-GET-ENTRY] Called, _rust_cache type: {type(self._rust_cache)}")
        if self._rust_cache:
            # Try sensor_get_bytes() first (available in current version)
            if hasattr(self._rust_cache, 'sensor_get_bytes'):
                logger.debug("[CACHE-GET] Using sensor_get_bytes()")
                try:
                    result = self._rust_cache.sensor_get_bytes()
                    result_bytes = bytes(result) if result else b""
                    logger.debug(f"[CACHE-GET] Got {len(result_bytes)} bytes")
                    return result_bytes
                except BaseException as rust_error:
                    error_msg = str(rust_error)
                    if "index out of bounds" in error_msg or "len is 0" in error_msg:
                        logger.debug(f"[CACHE-GET] Cache is empty: {rust_error}")
                        return b""
                    else:
                        logger.warning(f"[CACHE-GET] Rust error: {rust_error}")
                        return b""
            # Fallback: Try newer API sensor_get_byte_container()
            elif hasattr(self._rust_cache, 'sensor_get_byte_container'):
                logger.debug("[CACHE-GET] Using sensor_get_byte_container()")
                try:
                    container = self._rust_cache.sensor_get_byte_container()
                    if container and hasattr(container, 'copy_out_as_byte_vector'):
                        result_list = container.copy_out_as_byte_vector()
                        result_bytes = bytes(result_list)
                        logger.debug(f"[CACHE-GET] Got {len(result_bytes)} bytes")
                        return result_bytes
                except BaseException as e:
                    logger.warning(f"[CACHE-GET] Error: {e}")
                    return b""
            else:
                logger.warning(f"[CACHE-GET] No known method found on {type(self._rust_cache)}")
                return b""
        return b""