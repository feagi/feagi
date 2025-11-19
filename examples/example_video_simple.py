"""
Simple Video Streaming Example - Using VideoStreamAgent

Clean separation of concerns:
- User manages FEAGI engine lifecycle
- VideoStreamAgent focuses ONLY on video streaming

Usage:
    python examples/example_video_simple.py
"""

from feagi.engine import FeagiEngine
from feagi.agent import VideoStreamAgent

print("=" * 60)
print("FEAGI SDK 3.0 - Video Streaming Examples")
print("=" * 60)

# ==============================================================================
# Example 1: Simplest - User controls FEAGI, agent handles video
# ==============================================================================
print("\nExample 1: Basic video streaming")
print("-" * 60)

# 1. User starts FEAGI
engine = FeagiEngine()
engine.load_config("feagi_configuration.toml")
engine.start()

# 2. Agent handles ONLY video streaming
with VideoStreamAgent("examples/vt_all.mov") as agent:
    frames_sent = agent.run(max_frames=50)

# 3. User stops FEAGI
engine.stop()

print(f"✅ Sent {frames_sent} frames!\n")


# ==============================================================================
# Example 2: Generator pattern for motor control
# ==============================================================================
print("\nExample 2: Generator pattern with motor control")
print("-" * 60)

# Start FEAGI
engine = FeagiEngine()
engine.load_config("feagi_configuration.toml")
engine.start()

# Use agent as generator for frame-by-frame control
with VideoStreamAgent("examples/vt_all.mov") as agent:
    for frame_num, frame_data in agent.stream(max_frames=50):
        # Process vision
        if frame_num % 10 == 0:
            print(f"  Frame {frame_num}: shape={frame_data.shape}")
        
        # Here you would send motor commands based on vision
        # Example: brain_output.receive() and process motor commands

engine.stop()

print(f"✅ Processed frames with custom logic!\n")


# ==============================================================================
# Example 3: Advanced - Manual control
# ==============================================================================
print("\nExample 3: Manual control (no context manager)")
print("-" * 60)

# Start FEAGI
engine = FeagiEngine()
engine.load_config("feagi_configuration.toml")
engine.start()

# Create agent
agent = VideoStreamAgent(
    video_path="examples/vt_all.mov",
    camera_position="center",
    camera_encoding="absolute"
)

# Manually start agent
agent.start(
    feagi_host="localhost",
    feagi_port=5558,
    transport="zmq"
)

# Stream with custom settings
frame_count = 0
for frame_num, frame in agent.stream(
    max_frames=50,
    progress_interval=10,
    pace_by_fps=True
):
    frame_count += 1

# Manually stop agent
agent.stop()

# Stop FEAGI
engine.stop()

print(f"✅ Manual control: {frame_count} frames!\n")


# ==============================================================================
# Example 4: Sensorimotor loop (vision + motor control)
# ==============================================================================
print("\nExample 4: Complete sensorimotor loop")
print("-" * 60)

from feagi.pns import brain_output

# Start FEAGI
engine = FeagiEngine()
engine.load_config("feagi_configuration.toml")
engine.start()

# VideoStreamAgent for vision input
with VideoStreamAgent("examples/vt_all.mov") as agent:
    for frame_num, frame_data in agent.stream(max_frames=30):
        # Vision is already sent by agent.stream()
        
        # Receive motor commands from FEAGI
        # motor_data = brain_output.receive()
        
        # Process motor commands
        # Example: control_robot(motor_data)
        
        if frame_num % 10 == 0:
            print(f"  Sensorimotor loop: frame {frame_num}")

engine.stop()

print(f"✅ Sensorimotor loop complete!\n")


print("=" * 60)
print("All examples complete!")
print("=" * 60)
print("\nKey points:")
print("  ✅ User controls FEAGI lifecycle (engine.start/stop)")
print("  ✅ Agent focuses ONLY on video streaming")
print("  ✅ Generator pattern enables motor control")
print("  ✅ Clean separation of concerns")
print("=" * 60)
