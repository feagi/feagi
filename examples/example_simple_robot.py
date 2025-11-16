"""
Simple Robot Example using FEAGI SDK 3.0

Demonstrates the new inputs/outputs API for a simple robot with:
- Camera
- Infrared sensor
- Servo motor
- Rotary motors (left/right wheels)
"""

from feagi.pns.inputs import Camera, Infrared
from feagi.pns.outputs import ServoMotor, RotaryMotor
from feagi.pns import brain_input, brain_output
import numpy as np
import time


def main():
    print("=" * 60)
    print("FEAGI 3.0 - Simple Robot Example")
    print("=" * 60)
    
    # === Register Inputs ===
    print("\n📥 Registering inputs...")
    
    camera = Camera.register(
        resolution=(640, 480),
        encoding="absolute",
        position="center"
    )
    print(f"  ✓ Camera registered (640x480)")
    
    infrared_front = Infrared.register(
        encoding="absolute",
        inverted=False,
        min_distance=0.0,
        max_distance=400.0
    )
    print(f"  ✓ Infrared sensor registered (0-400cm)")
    
    # === Register Outputs ===
    print("\n📤 Registering outputs...")
    
    servo_head = ServoMotor.register(range=(0, 180))
    print(f"  ✓ Servo motor registered (0-180°)")
    
    motor_left = RotaryMotor.register(bidirectional=True)
    motor_right = RotaryMotor.register(bidirectional=True)
    print(f"  ✓ Rotary motors registered (left, right)")
    
    # === Configure Connection ===
    print("\n🔧 Configuring connection...")
    
    brain_input.configure(
        feagi_host="localhost",
        feagi_port=5558,
        transport="zmq"
    )
    
    brain_output.configure(
        feagi_host="localhost",
        feagi_port=5564,
        transport="zmq"
    )
    
    print(f"  ✓ Configured: localhost:5558 (input), localhost:5564 (output)")
    
    # === Connect ===
    print("\n🔗 Connecting to FEAGI...")
    
    try:
        brain_input.connect()
        brain_output.connect()
        print("  ✓ Connected successfully!")
    except Exception as e:
        print(f"  ✗ Connection failed: {e}")
        print("\n⚠️  Make sure:")
        print("    1. FEAGI is running on localhost")
        print("    2. feagi_rust_py_libs is installed")
        return
    
    # === Main Agent Loop ===
    print("\n🤖 Starting agent loop...")
    print("  (Press Ctrl+C to stop)\n")
    
    try:
        loop_count = 0
        while True:
            loop_count += 1
            
            # === Simulate sensor readings ===
            # In real code, replace with actual hardware API calls
            
            # Generate fake camera frame (random noise)
            camera_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            
            # Simulate infrared reading (oscillating distance)
            ir_distance = 200.0 + 100.0 * np.sin(loop_count * 0.1)
            
            # === Update FEAGI inputs ===
            camera.set_frame(camera_frame)
            infrared_front.set_distance(ir_distance)
            
            # === Send to FEAGI ===
            brain_input.send()
            
            # === Receive from FEAGI ===
            brain_output.receive()
            
            # === Read motor commands ===
            head_angle = servo_head.get_angle()
            left_speed = motor_left.get_speed()
            right_speed = motor_right.get_speed()
            
            # === Apply to hardware ===
            # In real code, replace with actual hardware API calls
            # set_servo_angle(head_angle)
            # set_motor_speeds(left_speed, right_speed)
            
            # Print status every 10 loops
            if loop_count % 10 == 0:
                print(f"[Loop {loop_count:04d}] "
                      f"IR={ir_distance:6.1f}cm | "
                      f"Head={head_angle:5.1f}° | "
                      f"Motors=({left_speed:+.2f}, {right_speed:+.2f})")
            
            # Small delay
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Stopping agent...")
    
    # === Cleanup ===
    brain_input.disconnect()
    brain_output.disconnect()
    print("✓ Disconnected")
    
    print("\n" + "=" * 60)
    print("Agent stopped successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()

