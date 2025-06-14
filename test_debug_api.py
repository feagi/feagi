#!/usr/bin/env python3
"""
Test script to demonstrate the enhanced --debug-api functionality.

This script shows how the improved API debug logging works by making
sample API requests and showing the detailed logging output.

Usage:
    python test_debug_api.py

The script will:
1. Start FEAGI with --debug-api enabled
2. Make sample API requests
3. Show the enhanced logging output
"""

import asyncio
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

import requests

# Add the current directory to Python path
sys.path.insert(0, str(Path(__file__).parent))


def test_enhanced_debug_api():
    """Test the enhanced --debug-api functionality."""

    print("🧪 Testing Enhanced --debug-api Functionality")
    print("=" * 50)

    # Start FEAGI with debug-api enabled in background
    print("🚀 Starting FEAGI with --debug-api enabled...")

    feagi_process = None
    try:
        # Start FEAGI process with debug-api
        feagi_process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "feagi.main",
                "--debug-api",
                "--api-port",
                "8001",  # Use different port to avoid conflicts
                "--log-level",
                "INFO",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True,
        )

        print("⏳ Waiting for FEAGI to start up...")

        # Wait for startup (look for API ready message)
        startup_timeout = 30
        start_time = time.time()
        api_ready = False

        while time.time() - start_time < startup_timeout:
            if feagi_process.poll() is not None:
                print("❌ FEAGI process exited unexpectedly")
                return False

            time.sleep(1)

            # Try to connect to API
            try:
                response = requests.get("http://localhost:8001/", timeout=2)
                if response.status_code in [200, 307]:  # 307 is redirect to /docs
                    api_ready = True
                    break
            except requests.exceptions.RequestException:
                continue

        if not api_ready:
            print("❌ FEAGI API did not start within timeout")
            return False

        print("✅ FEAGI API is ready!")
        print()

        # Now make test API requests to demonstrate debug logging
        print("📡 Making test API requests to demonstrate enhanced debug logging...")
        print("   (Check the FEAGI console output for detailed debug logs)")
        print()

        base_url = "http://localhost:8001"

        # Test 1: Simple GET request
        print("🔵 Test 1: GET /v1/system/status")
        try:
            response = requests.get(f"{base_url}/v1/system/status", timeout=5)
            print(f"   Response: {response.status_code}")
        except Exception as e:
            print(f"   Error: {e}")

        time.sleep(1)

        # Test 2: GET request with query parameters
        print("🔵 Test 2: GET /v1/connectome/cortical_areas with query params")
        try:
            response = requests.get(
                f"{base_url}/v1/connectome/cortical_areas?limit=10&offset=0", timeout=5
            )
            print(f"   Response: {response.status_code}")
        except Exception as e:
            print(f"   Error: {e}")

        time.sleep(1)

        # Test 3: POST request with JSON body
        print("🔵 Test 3: POST request with JSON body")
        test_data = {
            "test_field": "test_value",
            "nested_object": {"key1": "value1", "key2": 42, "array": [1, 2, 3]},
        }

        try:
            response = requests.post(
                f"{base_url}/v1/system/test",  # This might not exist, but will show debug logging
                json=test_data,
                timeout=5,
            )
            print(f"   Response: {response.status_code}")
        except Exception as e:
            print(f"   Error: {e}")

        time.sleep(1)

        # Test 4: Request that will cause an error (to test error logging)
        print("🔵 Test 4: Request to non-existent endpoint (to test error logging)")
        try:
            response = requests.get(f"{base_url}/v1/nonexistent/endpoint", timeout=5)
            print(f"   Response: {response.status_code}")
        except Exception as e:
            print(f"   Error: {e}")

        print()
        print("✅ Test requests completed!")
        print()
        print("🔍 Enhanced Debug Features Demonstrated:")
        print("   ✅ Unique request IDs for correlation")
        print("   ✅ Detailed request headers (with sensitive data masked)")
        print("   ✅ Query and path parameters logging")
        print("   ✅ Pretty-printed JSON request/response bodies")
        print("   ✅ Response timing information")
        print("   ✅ Color-coded logging with emojis")
        print("   ✅ Error handling and logging")
        print("   ✅ Request/response correlation")
        print()
        print("📋 Check the FEAGI console output above to see the detailed debug logs!")

        return True

    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
        return False

    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False

    finally:
        # Clean up: terminate FEAGI process
        if feagi_process:
            print("\n🛑 Stopping FEAGI...")
            feagi_process.terminate()
            try:
                feagi_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print("⚠️  Force killing FEAGI process...")
                feagi_process.kill()
                feagi_process.wait()
            print("✅ FEAGI stopped")


def show_usage_examples():
    """Show usage examples for the enhanced --debug-api flag."""

    print("\n📚 Enhanced --debug-api Usage Examples:")
    print("=" * 50)
    print()

    print("🔧 Basic usage:")
    print("   python -m feagi.main --debug-api")
    print()

    print("🔧 With custom port and log level:")
    print("   python -m feagi.main --debug-api --api-port 8001 --log-level DEBUG")
    print()

    print("🔧 Combined with other debug flags:")
    print("   python -m feagi.main --debug-api --debug-npu --debug-zmq-outbound")
    print()

    print("🔧 Environment variable (alternative):")
    print("   export FEAGI_DEBUG_API=1")
    print("   python -m feagi.main")
    print()

    print("📋 What you'll see with --debug-api enabled:")
    print("   🔵 Detailed request logging (method, URL, headers, body)")
    print("   🟢 Detailed response logging (status, headers, body)")
    print("   🔴 Error logging with stack traces")
    print("   ⏱️  Request timing information")
    print("   🆔 Unique request IDs for correlation")
    print("   🎨 Color-coded and formatted output")
    print("   🔒 Automatic masking of sensitive headers")
    print("   📄 Pretty-printed JSON formatting")
    print()


if __name__ == "__main__":
    print("🧪 FEAGI Enhanced --debug-api Test Suite")
    print("=" * 50)

    if len(sys.argv) > 1 and sys.argv[1] == "--examples":
        show_usage_examples()
    else:
        print("This script will test the enhanced --debug-api functionality.")
        print("It will start FEAGI with debug logging and make test API requests.")
        print()

        response = input("Continue? (y/N): ").strip().lower()
        if response in ["y", "yes"]:
            success = test_enhanced_debug_api()
            if success:
                show_usage_examples()
                print("\n🎉 Enhanced --debug-api test completed successfully!")
            else:
                print("\n❌ Test failed. Check the output above for details.")
                sys.exit(1)
        else:
            print("Test cancelled.")
            show_usage_examples()
