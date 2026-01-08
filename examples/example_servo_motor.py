"""
Servo Motor Example using FEAGI SDK 3.0

Demonstrates how to use ServoMotor with FEAGI for positional control.
This example shows:
- Registering multiple servos with different ranges
- Connecting to FEAGI
- Reading servo angles in the main loop
- Applying angles to hardware
"""

from feagi.pns.outputs import ServoMotor
from feagi.pns import brain_output
import time


def main():
    print("=" * 60)
    print("FEAGI 3.0 - Servo Motor Example")
    print("=" * 60)
    
    # === Register Servos ===
    print("\n📤 Registering servo motors...")
    
    # Standard servo (0-180°)
    servo_head = ServoMotor.register(range=(0, 180), encoding="absolute")
    print(f"  ✓ Head servo registered (0-180°)")
    
    # Wide-range servo (0-270°)
    servo_arm = ServoMotor.register(range=(0, 270), encoding="absolute")
    print(f"  ✓ Arm servo registered (0-270°)")
    
    # Bidirectional servo (-90 to +90°)
    servo_tilt = ServoMotor.register(range=(-90, 90), encoding="absolute")
    print(f"  ✓ Tilt servo registered (-90° to +90°)")
    
    # === Configure Connection ===
    print("\n🔧 Configuring connection...")
    
    brain_output.configure(
        feagi_host="localhost",
        feagi_port=5564,
        transport="zmq"
    )
    
    print(f"  ✓ Configured: localhost:5564 (ZMQ)")
    
    # === Connect ===
    print("\n🔗 Connecting to FEAGI...")
    
    try:
        brain_output.connect()
        print("  ✓ Connected successfully!")
    except Exception as e:
        print(f"  ✗ Connection failed: {e}")
        print("\n⚠️  Make sure:")
        print("    1. FEAGI is running on localhost")
        print("    2. feagi_rust_py_libs is installed")
        print("    3. Motor output port 5564 is accessible")
        return
    
    # === Main Loop ===
    print("\n🤖 Starting motor control loop...")
    print("  (Press Ctrl+C to stop)\n")
    
    try:
        loop_count = 0
        while True:
            loop_count += 1
            
            # === Receive motor commands from FEAGI ===
            brain_output.receive()
            
            # === Read servo angles ===
            head_angle = servo_head.get_angle()
            arm_angle = servo_arm.get_angle()
            tilt_angle = servo_tilt.get_angle()
            
            # === Apply to hardware ===
            # In real code, replace with actual hardware API calls:
            # 
            # import RPi.GPIO as GPIO  # Raspberry Pi
            # head_pwm.ChangeDutyCycle(angle_to_duty_cycle(head_angle))
            # 
            # import Adafruit_PCA9685  # PCA9685 servo driver
            # pwm.set_pwm(0, 0, angle_to_pulse(head_angle))
            # 
            # import serial  # Arduino over serial
            # arduino.write(f"S0:{int(head_angle)}\n".encode())
            
            # Simulated hardware calls
            apply_servo_angle("head", head_angle)
            apply_servo_angle("arm", arm_angle)
            apply_servo_angle("tilt", tilt_angle)
            
            # Print status every 10 loops
            if loop_count % 10 == 0:
                print(f"[Loop {loop_count:04d}] "
                      f"Head={head_angle:6.1f}° | "
                      f"Arm={arm_angle:6.1f}° | "
                      f"Tilt={tilt_angle:+6.1f}°")
            
            # Small delay (100ms = 10Hz control loop)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Stopping motor control...")
    
    # === Cleanup ===
    brain_output.disconnect()
    print("✓ Disconnected")
    
    print("\n" + "=" * 60)
    print("Motor control stopped successfully!")
    print("=" * 60)


def apply_servo_angle(name: str, angle: float):
    """
    Simulated hardware function.
    
    In a real embodiment controller, this would call your
    actual servo control library (e.g., pigpio, Adafruit, etc.)
    """
    # Example implementations:
    
    # Raspberry Pi (pigpio):
    # import pigpio
    # pi = pigpio.pi()
    # pulse_width = int(500 + (angle / 180.0) * 2000)  # 500-2500 μs
    # pi.set_servo_pulsewidth(pin, pulse_width)
    
    # PCA9685 (Adafruit):
    # import Adafruit_PCA9685
    # pwm = Adafruit_PCA9685.PCA9685()
    # pulse = int((angle / 180.0) * (600 - 150) + 150)  # 150-600 range
    # pwm.set_pwm(channel, 0, pulse)
    
    # Arduino (pyserial):
    # import serial
    # ser = serial.Serial('/dev/ttyUSB0', 9600)
    # ser.write(f"SERVO:{name}:{int(angle)}\n".encode())
    
    pass  # Placeholder for simulation


if __name__ == "__main__":
    main()

