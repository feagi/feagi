"""
ESP32 Serial Controller Example

Demonstrates how to use the ESP32SerialController to bridge ESP32 firmware
communication with FEAGI.

This example:
1. Auto-detects ESP32 serial port (or uses specified port)
2. Connects to FEAGI via ZMQ
3. Bridges sensory data from ESP32 → FEAGI
4. Bridges motor commands from FEAGI → ESP32

Prerequisites:
- ESP32 connected via USB
- ESP32 firmware flashed with FEAGI controller mode
- FEAGI running on localhost (or specify --feagi-host)

Usage:
    # Auto-detect ESP32 port
    python esp32_controller_example.py --agent-id esp32-01
    
    # Specify port
    python esp32_controller_example.py --agent-id esp32-01 --serial-port /dev/ttyUSB0
    
    # Custom FEAGI host
    python esp32_controller_example.py --agent-id esp32-01 --feagi-host 192.168.1.100
"""

import argparse
import logging
from feagi.agent.esp32 import Esp32SerialController


def main():
    parser = argparse.ArgumentParser(
        description="ESP32 Serial-to-FEAGI Bridge",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    
    parser.add_argument(
        "--agent-id",
        required=True,
        help="Unique identifier for this controller"
    )
    parser.add_argument(
        "--serial-port",
        default=None,
        help="Serial port path (auto-detects if not specified)"
    )
    parser.add_argument(
        "--baud-rate",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--feagi-host",
        default="localhost",
        help="FEAGI server hostname or IP (default: localhost)"
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)"
    )
    
    args = parser.parse_args()
    
    # Configure logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create and run controller
    controller = Esp32SerialController(
        agent_id=args.agent_id,
        serial_port=args.serial_port,
        baud_rate=args.baud_rate,
        feagi_host=args.feagi_host,
        auto_detect_port=(args.serial_port is None),
    )
    
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        raise


if __name__ == "__main__":
    main()

