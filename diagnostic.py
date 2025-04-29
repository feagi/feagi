#!/usr/bin/env python3
"""Diagnostic script for checking Python and ZMQ configuration.

This script helps diagnose issues with Python execution paths and ZMQ availability.
Run it with both your system Python and virtual environment Python to compare.
"""
import sys
import os
import platform

def main():
    print("\n===== FEAGI Diagnostic Tool =====\n")
    
    # Print system information
    print("System Information:")
    print(f"- OS: {platform.system()} {platform.release()}")
    print(f"- Python version: {platform.python_version()}")
    print(f"- Python executable: {sys.executable}")
    print(f"- Current working directory: {os.getcwd()}")
    print(f"- Virtual env active: {'VIRTUAL_ENV' in os.environ}")
    if 'VIRTUAL_ENV' in os.environ:
        print(f"- Virtual env path: {os.environ['VIRTUAL_ENV']}")
    
    # Check if we're in the FEAGI directory
    print("\nFEAGI Check:")
    feagi_dir = os.path.join(os.getcwd(), "feagi")
    if os.path.isdir(feagi_dir):
        print(f"- FEAGI directory found at: {feagi_dir}")
    else:
        print("- FEAGI directory not found in current location")
    
    # Check for ZMQ
    print("\nZMQ Check:")
    try:
        import zmq
        print(f"- PyZMQ imported successfully from: {getattr(zmq, '__file__', 'unknown')}")
        print(f"- PyZMQ version: {getattr(zmq, '__version__', 'unknown')}")
        
        if hasattr(zmq, 'Context') and callable(zmq.Context):
            print("- ZMQ Context attribute is available and callable")
            # Test creating a context
            try:
                context = zmq.Context()
                print("- Successfully created ZMQ Context")
                # Test creating a socket
                try:
                    socket = context.socket(zmq.PUB)
                    print("- Successfully created ZMQ Socket")
                    socket.close()
                except Exception as e:
                    print(f"- Error creating ZMQ Socket: {e}")
                finally:
                    context.term()
            except Exception as e:
                print(f"- Error creating ZMQ Context: {e}")
        else:
            print("- ZMQ Context attribute is MISSING or NOT CALLABLE")
            print("  This is likely why FEAGI is failing to initialize ZMQ")
            
        # Check zmq module attributes
        print("\nZMQ Module Attributes:")
        for attr in dir(zmq):
            if not attr.startswith('_'):  # Skip private attributes
                print(f"- {attr}: {type(getattr(zmq, attr))}")
    except ImportError as e:
        print(f"- PyZMQ import failed: {e}")
    except Exception as e:
        print(f"- Unexpected error checking ZMQ: {e}")
    
    print("\nRecommendations:")
    print("1. Always run FEAGI from your virtual environment using:")
    print("   source .venv/bin/activate && python -m feagi")
    print("   or directly with:")
    print("   ./.venv/bin/feagi")
    print("2. If you're still seeing Context issues, try reinstalling PyZMQ:")
    print("   pip uninstall -y pyzmq && pip install pyzmq==24.0.1")
    print("\n===== End of Diagnostic =====\n")

if __name__ == "__main__":
    main() 