"""
FEAGI Connector SDK

Complete SDK library for connecting to FEAGI (Fractal Evolutionary Adaptive General Intelligence).
"""

from feagi_connector.client import FeagiClient
from feagi_connector.agent_connector import FeagiAgentConnector
from feagi_connector.capabilities.manager import CapabilitiesManager
from feagi_connector.motor.processor import MotorProcessor
from feagi_connector.state.connection import ConnectionState
from feagi_connector.agent_logging.setup import setup_agent_logging

# New modular utilities
from feagi_connector.agent_logging.diagnostics import log_sensor_area_counts
from feagi_connector.utils.shm import SharedFrameWriter, ShmBytesWriter, ShmBytesReader
from feagi_connector.utils.rest_helpers import (
    get_cortical_dimensions,
    get_segmented_3x3_dimensions,
    set_simulation_timestep,
)
from feagi_connector.vision.processor import SegmentedVisionProcessor, bgr_to_rgb_uint8, numpy_to_image_frame
from feagi_connector.vision.visualize import build_segmented_mosaic, build_neural_image
from feagi_connector.utils.zero_serialization import build_zero_serialized_xyzp
from feagi_connector.media.source import MediaSource, MediaInfo
from feagi_connector.motor.shm_poll import poll_motor_shm
from feagi_connector.video.stream import stream_segmented_camera

# Export main classes and functions
__all__ = [
    # High-level agent connector
    "FeagiAgentConnector",
    "FeagiClient",
    "CapabilitiesManager", 
    "MotorProcessor",
    "ConnectionState",
    "setup_agent_logging",
    # Diagnostics
    "log_sensor_area_counts",
    # SHM
    "SharedFrameWriter",
    "ShmBytesWriter",
    "ShmBytesReader",
    # REST helpers
    "get_cortical_dimensions",
    "get_segmented_3x3_dimensions",
    "set_simulation_timestep",
    # Vision
    "SegmentedVisionProcessor",
    "bgr_to_rgb_uint8",
    "numpy_to_image_frame",
    "build_segmented_mosaic",
    "build_neural_image",
    "build_zero_serialized_xyzp",
    "MediaSource",
    "MediaInfo",
    "poll_motor_shm",
    "stream_segmented_camera",
]

__version__ = "1.0.0" 