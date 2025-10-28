import feagi_rust_py_libs as frpl
from .device import Device, ChannelCount, CorticalGroupIndex, ChannelIndex, NeuronDepth, IOData

# NOTE: For now we will generate based on "wrapped_data_type" in the template. We may need to change to "default_coder_type"



class Percentage1D(Device):
    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache, device_full_name: str):
        """Initialize the percentage-based sensor proxy.

        Args:
            io_cache_ref (frpl.connector_core.caching.IOCache): Rust IOCache bridge used to call into FEAGI.
            device_full_name (str): Full device identifier suffix (e.g. "infrared_absolute_linear").
        """
        super().__init__(io_cache_ref)

        register_fn_name: str = "sensor_" + device_full_name + "_try_register"
        write_fn_name: str = "sensor_" + device_full_name + "_try_write"
        read_post_fn_name: str = "sensor_" + device_full_name + "_try_read_postprocessed_cache_value"

        self._register_function = getattr(self._io_cache, register_fn_name)
        self._write_function = getattr(self._io_cache, write_fn_name)
        self._read_post_function = getattr(self._io_cache, read_post_fn_name)

    def register(self, cortical_group: CorticalGroupIndex, number_of_channels: ChannelCount,
                 z_resolution: NeuronDepth):
        """Register the sensor with the FEAGI runtime.

        Args:
            cortical_group (CorticalGroupIndex): Identifier of the cortical group the device belongs to.
            number_of_channels (ChannelCount): Number of sensor channels to expose.
            z_resolution (NeuronDepth): Depth resolution to configure for the device.
        """
        self._register_function(cortical_group, number_of_channels, z_resolution)

    def write(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex, data: IOData):
        """Write sensor data into the FEAGI cache.

        Args:
            cortical_group (CorticalGroupIndex): Cortical group receiving the data.
            channel (ChannelIndex): Channel index within the cortical group.
            data (IOData): Payload to store, matching the Percentage data type hierarchy.
        """
        self._write_function(cortical_group, channel, data)

    def read_post_processed_cache_value(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex) -> frpl.connector_core.data.Percentage:
        """Read the post-processed value from the FEAGI cache.

        Args:
            cortical_group (CorticalGroupIndex): Cortical group from which to read.
            channel (ChannelIndex): Channel index within the cortical group.

        Returns:
            frpl.connector_core.data.Percentage: Post-processed percentage value for the given channel.
        """
        return self._read_post_function(cortical_group, channel)

class ImageFrame(Device):
    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache, device_full_name: str):
        """Initialize the image frame sensor proxy.

        Args:
            io_cache_ref (frpl.connector_core.caching.IOCache): Rust IOCache bridge used to call into FEAGI.
            device_full_name (str): Full device identifier suffix (e.g. "image_camera_center").
        """
        super().__init__(io_cache_ref)

        register_fn_name: str = "sensor_" + device_full_name + "_try_register"
        write_fn_name: str = "sensor_" + device_full_name + "_try_write"
        read_post_fn_name: str = "sensor_" + device_full_name + "_try_read_postprocessed_cache_value"

        self._register_function = getattr(self._io_cache, register_fn_name)
        self._write_function = getattr(self._io_cache, write_fn_name)
        self._read_post_function = getattr(self._io_cache, read_post_fn_name)

    def register(self, cortical_group: CorticalGroupIndex, number_of_channels: ChannelCount,
                 image_properties: frpl.connector_core.data.descriptors.ImageFrameProperties):
        """Register the image sensor with the FEAGI runtime.

        Args:
            cortical_group (CorticalGroupIndex): Identifier of the cortical group the device belongs to.
            number_of_channels (ChannelCount): Number of image channels to expose.
            image_properties (frpl.connector_core.data.descriptors.ImageFrameProperties): Metadata describing the image frame.
        """
        self._register_function(cortical_group, number_of_channels, image_properties)

    def write(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex, data: IOData):
        """Write image data into the FEAGI cache.

        Args:
            cortical_group (CorticalGroupIndex): Cortical group receiving the data.
            channel (ChannelIndex): Channel index within the cortical group.
            data (IOData): Payload to store, matching the image frame data type hierarchy.
        """
        self._write_function(cortical_group, channel, data)

    def read_post_processed_cache_value(self, cortical_group: CorticalGroupIndex, channel: ChannelIndex) -> frpl.connector_core.data.ImageFrame:
        """Read the post-processed image frame from the FEAGI cache.

        Args:
            cortical_group (CorticalGroupIndex): Cortical group from which to read.
            channel (ChannelIndex): Channel index within the cortical group.

        Returns:
            frpl.connector_core.data.ImageFrame: Post-processed image frame for the given channel.
        """
        return self._read_post_function(cortical_group, channel)