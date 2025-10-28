import feagi_rust_py_libs as frpl
from .cache.devices.device import CorticalGroupIndex, ChannelIndex


class PreMadeReflexes:

    def __init__(self, io_cache_ref: frpl.connector_core.caching.IOCache):
        self._reflex_absolute_gaze_to_absolute_segmented_vision_function = getattr(io_cache_ref, "reflex_absolute_gaze_to_absolute_segmented_vision")


        pass

    def reflex_absolute_gaze_to_absolute_segmented_vision(self, gaze_cortical_group: CorticalGroupIndex,
                                                          gaze_channel: ChannelIndex,
                                                          segmentation_group: CorticalGroupIndex,
                                                          segmentation_channel: ChannelIndex):
        self._reflex_absolute_gaze_to_absolute_segmented_vision_function(gaze_cortical_group,
                                                                         gaze_channel,
                                                                         segmentation_group,
                                                                         segmentation_channel)

