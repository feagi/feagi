"""
FEAGI Agent Framework

Base classes and templates for creating FEAGI agents.

Provides:
- BaseAgent: Abstract base class for all agents
- SDKRobotAgent: Template for SDK-based robots (Cozmo, NAO, etc.)
- SimulatorAgent: Template for physics simulators (Webots, Gazebo)
- EmbeddedAgent: Template for embedded devices (ESP32, Arduino)
- VirtualAgent: Template for game engines (Unity, Unreal)

Example:
    from feagi.agent import SDKRobotAgent
    
    class CozmoAgent(SDKRobotAgent):
        sdk_package = "cozmo"
        
        def initialize_hardware(self):
            import cozmo
            self.robot = cozmo.connect()
        
        def map_sensors(self, hw_data):
            return {"camera": hw_data.camera_image}
"""

from typing import TYPE_CHECKING

from feagi.agent.base import BaseAgent
from feagi.agent.video import VideoStreamAgent

# Optional dependency: pyserial. Keep import lazy; SDK can be used without
# embedded support.
if TYPE_CHECKING:
    from feagi.agent.esp32 import Esp32SerialController

__all__ = ["BaseAgent", "VideoStreamAgent", "Esp32SerialController"]

# Optional Bluetooth support
try:
    from feagi.agent.bluetooth import BluetoothRobot
    __all__.append("BluetoothRobot")
except ImportError:
    BluetoothRobot = None

def __getattr__(name: str):
    """Lazy attribute access for optional agent helpers."""
    if name == "Esp32SerialController":
        from feagi.agent.esp32 import Esp32SerialController

        return Esp32SerialController
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

