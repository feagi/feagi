import feagi_rust_py_libs as frpl
from typing import TypeAlias
from abc import ABC

CorticalGroupIndex: TypeAlias = int | frpl.data_structures.genomic.descriptors.CorticalGroupIndex
ChannelCount: TypeAlias = int | frpl.data_structures.genomic.descriptors.CorticalChannelCount
ChannelIndex: TypeAlias = int | frpl.data_structures.genomic.descriptors.CorticalChannelIndex
NeuronDepth: TypeAlias = int
IOData: TypeAlias = frpl.connector_core.data.ImageFrame | frpl.connector_core.data.SegmentedImageFrame | frpl.connector_core.data.MiscData | frpl.connector_core.data.Percentage | frpl.connector_core.data.SignedPercentage | frpl.connector_core.data.Percentage2D | frpl.connector_core.data.SignedPercentage2D | frpl.connector_core.data.Percentage3D | frpl.connector_core.data.SignedPercentage3D | frpl.connector_core.data.Percentage4D | frpl.connector_core.data.SignedPercentage4D


class Device(ABC):
    def __init__(self, io_cache: frpl.connector_core.data.IOCache):
        self._io_cache: frpl.connector_core.data.IOCache = io_cache

    #TODO stage logic proxy
