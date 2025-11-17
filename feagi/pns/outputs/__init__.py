"""
FEAGI Outputs

All data targets that receive from FEAGI.
"""

from feagi.pns.outputs.base import BaseOutput
from feagi.pns.outputs.motor import ServoMotor, RotaryMotor
from feagi.pns.outputs.numeric import NumericStream
from feagi.pns.outputs.text import TextStream

__all__ = [
    "BaseOutput",
    "ServoMotor",
    "RotaryMotor",
    "NumericStream",
    "TextStream",
]

