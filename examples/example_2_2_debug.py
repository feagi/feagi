
from feagi_connector_2 import FeagiAgent
import feagi_rust_py_libs as frpl
import numpy as np
import asyncio
import sys

if sys.platform == 'win32':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())


async def main():
    print("=" * 60)
    print("FEAGI Agent Example - Debug Version")
    print("=" * 60)
    
    input_image_resolution = (128, 128, 3)
    print(f"\n[1/6] Setting up image properties: {input_image_resolution}")

    input_image_properties = frpl.connector_core.data.descriptors.ImageFrameProperties(
        frpl.connector_core.data.descriptors.ImageXYResolution(input_image_resolution[0], input_image_resolution[1]),
        frpl.connector_core.data.descriptors.ColorSpace.Linear,
        frpl.connector_core.data.descriptors.ColorChannelLayout.RGB
    )
    print("      ✓ Image properties created")

    print("\n[2/6] Creating FEAGI agent...")
    feagi_agent = FeagiAgent()
    print("      ✓ Agent created")
    
    print("\n[3/6] Registering camera...")
    feagi_agent.brain_input.image_camera_center.register(0, 1, input_image_properties)
    print("      ✓ Camera registered (device_index=0, channel=1)")

    # Connection with timeout
    print("\n[4/6] Connecting to FEAGI...")
    print("      Host: tcp://localhost")
    print("      Registration port: 30000")
    print("      Brain input port: 30001")
    print("      Brain output port: 30002")
    print("\n      Waiting for FEAGI response (timeout: 5 seconds)...")
    
    try:
        registration_response = await asyncio.wait_for(
            feagi_agent.feagi.connect_via_zmq(
                "tcp://localhost", 
                input_image_resolution, 
                registration_port=30000, 
                brain_input_port=30001, 
                brain_output_port=30002
            ),
            timeout=5.0
        )
        print("      ✓ Connected successfully!")
        print(f"      Response: {registration_response}")
    except asyncio.TimeoutError:
        print("\n      ✗ CONNECTION FAILED: Timeout")
        print("\n" + "=" * 60)
        print("ERROR: Could not connect to FEAGI")
        print("=" * 60)
        print("\nPossible causes:")
        print("  1. FEAGI is not running")
        print("  2. FEAGI is running on a different host/port")
        print("  3. Firewall is blocking the connection")
        print("\nTo fix:")
        print("  1. Make sure FEAGI is running")
        print("  2. Check FEAGI is listening on localhost:30000")
        print("  3. Verify the port configuration in your FEAGI setup")
        print("=" * 60)
        return
    except Exception as e:
        print(f"\n      ✗ CONNECTION FAILED: {type(e).__name__}")
        print(f"      Error: {e}")
        return

    # Create and send image data
    print("\n[5/6] Creating test image...")
    image_arr: np.ndarray = np.ones(input_image_resolution).astype(np.uint8) * 100
    print(f"      ✓ Created {input_image_resolution} image with value 100")
    
    image_frame = frpl.connector_core.data.ImageFrame.new_from_array(
        image_arr, 
        input_image_properties.color_space, 
        frpl.connector_core.data.descriptors.MemoryOrderLayout.WidthsHeightsChannels
    )
    print("      ✓ Image frame created")
    
    feagi_agent.brain_input.image_camera_center.write(0, 0, image_frame)
    print("      ✓ Image written to cache (device=0, channel=0)")

    print("\n[6/6] Sending data to FEAGI...")
    try:
        await asyncio.wait_for(
            feagi_agent.brain_input_cache.send_brain_input_to_feagi(),
            timeout=2.0
        )
        print("      ✓ Data sent successfully!")
    except asyncio.TimeoutError:
        print("      ✗ Timeout while sending data")
        return
    except Exception as e:
        print(f"      ✗ Error sending data: {e}")
        return

    print("\n" + "=" * 60)
    print("SUCCESS: Example completed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()

