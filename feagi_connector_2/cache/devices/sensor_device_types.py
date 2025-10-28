import feagi_rust_py_libs as frpl
from device import Device, ChannelCount, CorticalGroupIndex, ChannelIndex, NeuronDepth, IOData

# NOTE: For now we will generate based on "wrapped_data_type" in the template. We may need to change to "default_coder_type"



class Percentage1D(Device):
    def __init__(self, io_cache_ref: frpl.connector_core.data.IOCache, device_full_name: str):
        super().__init__(io_cache_ref)

        register_fn_name: str = "sensor_" + device_full_name + "_try_register"
        write_fn_name: str = "sensor_" + device_full_name + "_try_write"
        read_post_fn_name: str = "sensor_" + device_full_name + "_try_read_postprocessed_cache_value"

        self._register_function = getattr(self._io_cache, register_fn_name)(CorticalGroupIndex, ChannelCount, NeuronDepth)
        self._write_function = getattr(self._io_cache, write_fn_name)(CorticalGroupIndex, ChannelIndex, IOData)
        self._read_post_function = getattr(self._io_cache, read_post_fn_name)(CorticalGroupIndex, ChannelIndex)

    def register(self, cortical_group: CorticalGroupIndex, number_of_channels: ChannelCount,
                 z_resolution: NeuronDepth):
        self._register_function(cortical_group, number_of_channels, z_resolution)

    def write(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex, data: IOData):
        self._write_function(cortical_group, channel, data)

    def read_post_processed_cache_value(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex) -> frpl.connector_core.data.Percentage:
        return self._read_post_function(cortical_group, channel)

class ImageFrame(Device):
    def __init__(self, io_cache_ref: frpl.connector_core.data.IOCache, device_full_name: str):
        super().__init__(io_cache_ref)

        register_fn_name: str = "sensor_" + device_full_name + "_try_register"
        write_fn_name: str = "sensor_" + device_full_name + "_try_write"
        read_post_fn_name: str = "sensor_" + device_full_name + "_try_read_postprocessed_cache_value"

        self._register_function = getattr(self._io_cache, register_fn_name)(CorticalGroupIndex, ChannelCount, frpl.data_structures.genomic.descriptors.ImageFrameProperties)
        self._write_function = getattr(self._io_cache, write_fn_name)(CorticalGroupIndex, ChannelIndex, IOData)
        self._read_post_function = getattr(self._io_cache, read_post_fn_name)(CorticalGroupIndex, ChannelIndex)

    def register(self, cortical_group: CorticalGroupIndex, number_of_channels: ChannelCount,
                 image_properties: frpl.data_structures.genomic.descriptors.ImageFrameProperties):
        self._register_function(cortical_group, number_of_channels, image_properties)

    def write(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex, data: IOData):
        self._write_function(cortical_group, channel, data)

    def read_post_processed_cache_value(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex) -> frpl.connector_core.data.ImageFrame:
        return self._read_post_function(cortical_group, channel)