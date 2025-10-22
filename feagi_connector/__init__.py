"""
FEAGI Connector SDK

Complete SDK library for connecting to FEAGI (Fractal Evolutionary Adaptive General Intelligence).

New in 2.0:
- FeagiAgentClient: Rust-backed high-performance client (RECOMMENDED)
- Automatic heartbeat and reconnection
- 10-100x faster message processing
- Simpler API with fewer manual steps

Legacy:
- FeagiClient: Legacy ZMQ client (DEPRECATED - use FeagiAgentClient)
- FeagiAgentConnector: Legacy connector (DEPRECATED - use FeagiAgentClient)
"""

# New Rust-backed client (RECOMMENDED)
from feagi_connector.agent_client import FeagiAgentClient, AgentType, create_agent

# Legacy clients (DEPRECATED)
from feagi_connector.client import FeagiClient
from feagi_connector.agent_connector import FeagiAgentConnector
from feagi_connector.capabilities.manager import CapabilitiesManager
from feagi_connector.motor.processor import MotorProcessor
from feagi_connector.state.connection import ConnectionState
from feagi_connector.agent_logging.setup import setup_agent_logging

# Convenience function for creating test/example connectors
def create_dummy_connector():
    """Create a dummy connector for testing and examples without actual server connection.
    
    This is a convenience wrapper around FeagiAgentConnector.create_dummy_connector().
    
    Returns:
        FeagiAgentConnector: A connector instance with dummy server backend
    """
    return FeagiAgentConnector.create_dummy_connector()

# New modular utilities
from feagi_connector.agent_logging.diagnostics import log_sensor_area_counts
from feagi_connector.utils.config_loader import (
    load_agent_config,
    validate_feagi_config,
    get_config_template,
    merge_cli_args,
)
from feagi_connector.utils.shm import SharedFrameWriter, ShmBytesWriter, ShmBytesReader
from feagi_connector.utils.latest_only_writer import LatestOnlyWriter
from feagi_connector.utils.rest_helpers import (
    get_cortical_dimensions,
    get_segmented_3x3_dimensions,
    set_simulation_timestep,
)
from feagi_connector.vision.processor import SegmentedVisionProcessor, GazeMotorProcessor, bgr_to_rgb_uint8, numpy_to_image_frame, create_gaze_control_neurons
from feagi_connector.vision.visualize import build_segmented_mosaic, build_neural_image
from feagi_connector.utils.zero_serialization import build_zero_serialized_xyzp
from feagi_connector.media.source import MediaSource, MediaInfo
from feagi_connector.motor.shm_poll import poll_motor_shm
from feagi_connector.video.stream import stream_segmented_camera

# Export main classes and functions
__all__ = [
    # New Rust-backed client (RECOMMENDED)
    "FeagiAgentClient",
    "AgentType",
    "create_agent",
    # Legacy clients (DEPRECATED)
    "FeagiAgentConnector",
    "create_dummy_connector",
    "FeagiClient",
    "CapabilitiesManager", 
    "MotorProcessor",
    "ConnectionState",
    "setup_agent_logging",
    # Diagnostics
    "log_sensor_area_counts",
    # Configuration (FEAGI 2.0 standard)
    "load_agent_config",
    "validate_feagi_config",
    "get_config_template",
    "merge_cli_args",
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
    "GazeMotorProcessor", 
    "bgr_to_rgb_uint8",
    "numpy_to_image_frame",
    "create_gaze_control_neurons",
    "build_segmented_mosaic",
    "build_neural_image",
    "build_zero_serialized_xyzp",
    "MediaSource",
    "MediaInfo",
    "poll_motor_shm",
    "stream_segmented_camera",
]

__version__ = "2.0.0"

# Print version info on import to help debug version mismatches
import sys
import os

def _print_version_info():
    """Print version information for debugging"""
    print(f"📦 [feagi-connector] Version {__version__}")
    
    # Try to get Rust SDK version
    try:
        import feagi_agent_sdk_py
        sdk_file = feagi_agent_sdk_py.__file__
        # Get the .so file
        sdk_dir = os.path.dirname(sdk_file)
        so_files = [f for f in os.listdir(sdk_dir) if f.endswith('.so')]
        if so_files:
            so_path = os.path.join(sdk_dir, so_files[0])
            import time
            mtime = os.path.getmtime(so_path)
            mtime_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))
            print(f"🦀 [feagi-agent-sdk-py] Built: {mtime_str}")
        else:
            print(f"🦀 [feagi-agent-sdk-py] Installed (no .so found)")
    except ImportError:
        print(f"⚠️  [feagi-agent-sdk-py] Not installed")
    except Exception as e:
        print(f"⚠️  [feagi-agent-sdk-py] Error checking version: {e}")

# Only print on first import (not on reload)
if not hasattr(sys, '_feagi_connector_version_printed'):
    _print_version_info()
    sys._feagi_connector_version_printed = True 