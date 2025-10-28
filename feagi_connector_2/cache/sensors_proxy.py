import feagi_rust_py_libs as frpl
from .devices.sensor_device_types import Percentage1D, ImageFrame, MiscData, SegmentedImageFrame

class SensorsProxy:

    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache):
        self.infrared_absolute_linear = Percentage1D(io_cache_ref, "infrared_absolute_linear")
        self.infrared_absolute_fractional = Percentage1D(io_cache_ref, "infrared_absolute_fractional")
        self.infrared_incremental_linear = Percentage1D(io_cache_ref, "infrared_incremental_linear")
        self.infrared_incremental_fractional = Percentage1D(io_cache_ref, "infrared_incremental_fractional")
        self.infrared_inverted_absolute_linear = Percentage1D(io_cache_ref, "infrared_inverted_absolute_linear")
        self.infrared_inverted_absolute_fractional = Percentage1D(io_cache_ref, "infrared_inverted_absolute_fractional")
        self.infrared_inverted_incremental_linear = Percentage1D(io_cache_ref, "infrared_inverted_incremental_linear")
        self.infrared_inverted_incremental_fractional = Percentage1D(io_cache_ref, "infrared_inverted_incremental_fractional")
        self.gpio_digital_absolute_linear = Percentage1D(io_cache_ref, "gpio_digital_absolute_linear")
        self.gpio_digital_absolute_fractional = Percentage1D(io_cache_ref, "gpio_digital_absolute_fractional")
        self.gpio_digital_incremental_linear = Percentage1D(io_cache_ref, "gpio_digital_incremental_linear")
        self.gpio_digital_incremental_fractional = Percentage1D(io_cache_ref, "gpio_digital_incremental_fractional")
        self.proximity_absolute_linear = Percentage1D(io_cache_ref, "proximity_absolute_linear")
        self.proximity_absolute_fractional = Percentage1D(io_cache_ref, "proximity_absolute_fractional")
        self.proximity_incremental_linear = Percentage1D(io_cache_ref, "proximity_incremental_linear")
        self.proximity_incremental_fractional = Percentage1D(io_cache_ref, "proximity_incremental_fractional")
        self.shock_absolute_linear = Percentage1D(io_cache_ref, "shock_absolute_linear")
        self.shock_absolute_fractional = Percentage1D(io_cache_ref, "shock_absolute_fractional")
        self.shock_incremental_linear = Percentage1D(io_cache_ref, "shock_incremental_linear")
        self.shock_incremental_fractional = Percentage1D(io_cache_ref, "shock_incremental_fractional")
        self.battery_gauge_absolute_linear = Percentage1D(io_cache_ref, "battery_gauge_absolute_linear")
        self.battery_gauge_absolute_fractional = Percentage1D(io_cache_ref, "battery_gauge_absolute_fractional")
        self.battery_gauge_incremental_linear = Percentage1D(io_cache_ref, "battery_gauge_incremental_linear")
        self.battery_gauge_incremental_fractional = Percentage1D(io_cache_ref, "battery_gauge_incremental_fractional")
        self.gpio_analog_absolute_linear = Percentage1D(io_cache_ref, "gpio_analog_absolute_linear")
        self.gpio_analog_absolute_fractional = Percentage1D(io_cache_ref, "gpio_analog_absolute_fractional")
        self.gpio_analog_incremental_linear = Percentage1D(io_cache_ref, "gpio_analog_incremental_linear")
        self.gpio_analog_incremental_fractional = Percentage1D(io_cache_ref, "gpio_analog_incremental_fractional")

        self.miscellaneous_absolute = MiscData(io_cache_ref, "miscellaneous_absolute")
        self.miscellaneous_incremental = MiscData(io_cache_ref, "miscellaneous_incremental")

        self.image_camera_center = ImageFrame(io_cache_ref, "image_camera_center_absolute")

        self.segmented_vision_absolute = SegmentedImageFrame(io_cache_ref, "segmented_vision_absolute")