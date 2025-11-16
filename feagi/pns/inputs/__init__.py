"""
FEAGI Inputs

All data sources that feed into FEAGI.
"""

from feagi.pns.inputs.base import BaseInput
from feagi.pns.inputs.vision import Camera
from feagi.pns.inputs.numeric import NumericStream, Infrared
from feagi.pns.inputs.text import TextStream

__all__ = [
    "BaseInput",
    "Camera",
    "NumericStream",
    "Infrared",
    "TextStream",
]

