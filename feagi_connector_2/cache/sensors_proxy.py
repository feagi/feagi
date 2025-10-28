import feagi_rust_py_libs as frpl
from devices.sensor_device_types import Percentage1D, ImageFrame

class SensorsProxy:

    def __init__(self, io_cache_ref: frpl.connector_core.data.IOCache):


        self.image_camera_center = ImageFrame(io_cache_ref, "image_camera_center")