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

from feagi.agent.base import BaseAgent
from feagi.agent.video import VideoStreamAgent
from feagi.agent.esp32 import Esp32SerialController

__all__ = ["BaseAgent", "VideoStreamAgent", "Esp32SerialController"]

# Optional Bluetooth support
try:
    from feagi.agent.bluetooth import BluetoothRobot
    __all__.append("BluetoothRobot")
except ImportError:
    BluetoothRobot = None

