"""
FEAGI Peripheral Nervous System (PNS)

Simple, intuitive API for sensorimotor operations.

The PNS layer provides:
- brain_input: Global input manager (send sensory data to FEAGI)
- brain_output: Global output manager (receive motor commands from FEAGI)
- inputs: All data sources (Camera, Infrared, NumericStream, etc.)
- outputs: All data targets (ServoMotor, RotaryMotor, NumericStream, etc.)

Example - Robotics:
    from feagi.pns.inputs import Camera, Infrared
    from feagi.pns.outputs import ServoMotor, RotaryMotor
    from feagi.pns import brain_input, brain_output
    
    # Register devices
    camera = Camera.register(resolution=(1920, 1080))
    infrared = Infrared.register()
    servo = ServoMotor.register(range=(0, 180))
    motor = RotaryMotor.register()
    
    # Configure
    brain_input.configure(feagi_host="localhost")
    brain_output.configure(feagi_host="localhost")
    brain_input.connect()
    brain_output.connect()
    
    # Main loop
    while True:
        camera.set_frame(frame)
        infrared.set_distance(distance)
        brain_input.send()
        
        brain_output.receive()
        servo_angle = servo.get_angle()
        motor_speed = motor.get_speed()

Example - Language Learning:
    from feagi.pns.inputs import TextStream as TextInput
    from feagi.pns.outputs import TextStream as TextOutput
    from feagi.pns import brain_input, brain_output
    
    text_in = TextInput.register(max_length=100)
    text_out = TextOutput.register(max_length=100)
    
    brain_input.configure()
    brain_output.configure()
    brain_input.connect()
    brain_output.connect()
    
    while True:
        text_in.set_text(input_text)
        brain_input.send()
        
        brain_output.receive()
        generated_text = text_out.get_text()
"""

# Global singletons
from feagi.pns.brain_input import brain_input
from feagi.pns.brain_output import brain_output

# Import submodules for convenient access
from feagi.pns import inputs
from feagi.pns import outputs

# Client for agent integration
from feagi.pns.client import FeagiAgentClient, AgentType

# Also export common classes directly
from feagi.pns.inputs import Camera, NumericStream as NumericInput, Infrared
from feagi.pns.outputs import ServoMotor, RotaryMotor, NumericStream as NumericOutput

__all__ = [
    # Global singletons
    "brain_input",
    "brain_output",
    # Client
    "FeagiAgentClient",
    "AgentType",
    # Submodules
    "inputs",
    "outputs",
    # Common inputs
    "Camera",
    "NumericInput",
    "Infrared",
    # Common outputs
    "ServoMotor",
    "RotaryMotor",
    "NumericOutput",
]
