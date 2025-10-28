import feagi_rust_py_libs as frpl
from .devices.motor_device_types import Percentage1D, Percentage4D, MiscData

class MotorsProxy:

    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache):
        self.rotary_motor_absolute_linear = Percentage1D(io_cache_ref, "rotary_motor_absolute_linear")
        self.rotary_motor_absolute_fractional = Percentage1D(io_cache_ref, "rotary_motor_absolute_fractional")
        self.rotary_motor_incremental_linear = Percentage1D(io_cache_ref, "rotary_motor_incremental_linear")
        self.rotary_motor_incremental_fractional = Percentage1D(io_cache_ref, "rotary_motor_incremental_fractional")
        self.positional_servo_absolute_linear = Percentage1D(io_cache_ref, "positional_servo_absolute_linear")
        self.positional_servo_absolute_fractional = Percentage1D(io_cache_ref, "positional_servo_absolute_fractional")
        self.positional_servo_incremental_linear = Percentage1D(io_cache_ref, "positional_servo_incremental_linear")
        self.positional_servo_incremental_fractional = Percentage1D(io_cache_ref, "positional_servo_incremental_fractional")

        self.gaze_absolute_linear = Percentage4D(io_cache_ref, "gaze_absolute_linear")
        self.gaze_incremental_linear = Percentage4D(io_cache_ref, "gaze_incremental_linear")
        self.miscellaneous_absolute = MiscData(io_cache_ref, "miscellaneous_absolute")
        self.miscellaneous_incremental = MiscData(io_cache_ref, "miscellaneous_incremental")



