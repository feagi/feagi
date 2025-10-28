import feagi_rust_py_libs as frpl
from .device import Device, ChannelCount, CorticalGroupIndex, ChannelIndex, NeuronDepth, IOData

# NOTE: For now we will generate based on "wrapped_data_type" in the template. We may need to change to "default_coder_type"



class Percentage1D(Device):
    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache, device_full_name: str):
        super().__init__(io_cache_ref)

        register_fn_name: str = "motor_" + device_full_name + "_try_register"
        read_pre_fn_name: str = "motor_" + device_full_name + "_try_read_preprocessed_cached_value"
        read_post_fn_name: str = "motor_" + device_full_name + "_try_read_postprocessed_cached_value" # TODO inconsistent names

        self._register_function = getattr(self._io_cache, register_fn_name)
        self._read_pre_function = getattr(self._io_cache, read_pre_fn_name)
        self._read_post_function = getattr(self._io_cache, read_post_fn_name)

    def register(self, cortical_group: CorticalGroupIndex, number_of_channels: ChannelCount,
                 z_resolution: NeuronDepth):
        self._register_function(cortical_group, number_of_channels, z_resolution)

    def read_preprocessed_cache_value(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex) -> frpl.connector_core.data.Percentage:
        return self._read_pre_function(cortical_group, channel)

    def read_postprocessed_cache_value(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex) -> frpl.connector_core.data.Percentage:
        return self._read_post_function(cortical_group, channel)

class Percentage4D(Device):
    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache, device_full_name: str):
        super().__init__(io_cache_ref)

        register_fn_name: str = "motor_" + device_full_name + "_try_register"
        read_pre_fn_name: str = "motor_" + device_full_name + "_try_read_preprocessed_cached_value"
        read_post_fn_name: str = "motor_" + device_full_name + "_try_read_postprocessed_cached_value" # TODO inconsistent names

        self._register_function = getattr(self._io_cache, register_fn_name)
        self._read_pre_function = getattr(self._io_cache, read_pre_fn_name)
        self._read_post_function = getattr(self._io_cache, read_post_fn_name)

    def register(self, cortical_group: CorticalGroupIndex, number_of_channels: ChannelCount,
                 z_resolution: NeuronDepth):
        self._register_function(cortical_group, number_of_channels, z_resolution)

    def read_preprocessed_cache_value(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex) -> frpl.connector_core.data.Percentage4D:
        return self._read_pre_function(cortical_group, channel)

    def read_postprocessed_cache_value(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex) -> frpl.connector_core.data.Percentage4D:
        return self._read_post_function(cortical_group, channel)

class MiscData(Device):
    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache, device_full_name: str):
        super().__init__(io_cache_ref)

        register_fn_name: str = "motor_" + device_full_name + "_try_register"
        read_pre_fn_name: str = "motor_" + device_full_name + "_try_read_preprocessed_cached_value"
        read_post_fn_name: str = "motor_" + device_full_name + "_try_read_postprocessed_cached_value" # TODO inconsistent names

        self._register_function = getattr(self._io_cache, register_fn_name)
        self._read_pre_function = getattr(self._io_cache, read_pre_fn_name)
        self._read_post_function = getattr(self._io_cache, read_post_fn_name)

    def register(self, cortical_group: CorticalGroupIndex, number_of_channels: ChannelCount,
                 misc_dimensions: frpl.connector_core.data.descriptors.MiscDataDimensions):
        self._register_function(cortical_group, number_of_channels, misc_dimensions)

    def read_preprocessed_cache_value(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex) -> frpl.connector_core.data.MiscData:
        return self._read_pre_function(cortical_group, channel)

    def read_postprocessed_cache_value(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex) -> frpl.connector_core.data.MiscData:
        return self._read_post_function(cortical_group, channel)