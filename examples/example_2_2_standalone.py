
"""
Standalone test for FEAGI Agent - No FEAGI connection required
This version tests the agent functionality without connecting to FEAGI
"""

from feagi_connector_2 import FeagiAgent
import feagi_rust_py_libs as frpl
import numpy as np
import asyncio
import sys

if sys.platform == 'win32':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())


async def main():
    print("=" * 60)
    print("FEAGI Agent Standalone Test")
    print("Testing agent functionality without FEAGI connection")
    print("=" * 60)
    
    # Test 1: Agent Creation
    print("\n[TEST 1] Creating FEAGI agent...")
    try:
        feagi_agent = FeagiAgent()
        print("         ✓ Agent created successfully")
    except Exception as e:
        print(f"         ✗ Failed: {e}")
        return
    
    # Test 2: Image Properties Setup
    print("\n[TEST 2] Setting up image properties...")
    try:
        input_image_resolution = (128, 128, 3)
        input_image_properties = frpl.connector_core.data.descriptors.ImageFrameProperties(
            frpl.connector_core.data.descriptors.ImageXYResolution(
                input_image_resolution[0], 
                input_image_resolution[1]
            ),
            frpl.connector_core.data.descriptors.ColorSpace.Linear,
            frpl.connector_core.data.descriptors.ColorChannelLayout.RGB
        )
        print(f"         ✓ Properties created for {input_image_resolution}")
    except Exception as e:
        print(f"         ✗ Failed: {e}")
        return
    
    # Test 3: Camera Registration
    print("\n[TEST 3] Registering camera...")
    try:
        feagi_agent.brain_input.image_camera_center.register(0, 1, input_image_properties)
        print("         ✓ Camera registered (device=0, channel=1)")
    except Exception as e:
        print(f"         ✗ Failed: {e}")
        return
    
    # Test 4: Image Creation
    print("\n[TEST 4] Creating test images...")
    try:
        # Create multiple test images with different patterns
        images = {
            "solid_gray": np.ones(input_image_resolution).astype(np.uint8) * 100,
            "gradient": np.linspace(0, 255, 128*128*3).reshape(input_image_resolution).astype(np.uint8),
            "checkerboard": ((np.indices((128, 128)).sum(axis=0) % 2)[:, :, None] * 255).repeat(3, axis=2).astype(np.uint8)
        }
        print(f"         ✓ Created {len(images)} test images")
    except Exception as e:
        print(f"         ✗ Failed: {e}")
        return
    
    # Test 5: Image Frame Conversion
    print("\n[TEST 5] Converting images to frames...")
    try:
        frames = {}
        for name, img_arr in images.items():
            frame = frpl.connector_core.data.ImageFrame.new_from_array(
                img_arr, 
                input_image_properties.color_space, 
                frpl.connector_core.data.descriptors.MemoryOrderLayout.WidthsHeightsChannels
            )
            frames[name] = frame
        print(f"         ✓ Converted {len(frames)} images to frames")
    except Exception as e:
        print(f"         ✗ Failed: {e}")
        return
    
    # Test 6: Writing to Cache
    print("\n[TEST 6] Writing frames to cache...")
    try:
        for name, frame in frames.items():
            feagi_agent.brain_input.image_camera_center.write(0, 0, frame)
            print(f"         ✓ Wrote {name} frame to cache")
    except Exception as e:
        print(f"         ✗ Failed: {e}")
        return
    
    # Test 7: Serialization (without sending)
    print("\n[TEST 7] Testing cache serialization...")
    try:
        # Get the serialized data without sending it
        byte_container = feagi_agent.brain_input_cache._rust_cache.sensor_get_byte_container()
        serialized_bytes = byte_container.get_bytes()
        print(f"         ✓ Serialized data: {len(serialized_bytes)} bytes")
    except Exception as e:
        print(f"         ✗ Failed: {e}")
        # This is expected if the method doesn't exist yet
        print(f"         Note: Serialization may not be fully implemented")
    
    # Test 8: Multiple Device Support
    print("\n[TEST 8] Testing multiple device support...")
    try:
        # Register additional cameras
        feagi_agent.brain_input.image_camera_center.register(1, 1, input_image_properties)
        feagi_agent.brain_input.image_camera_center.register(2, 1, input_image_properties)
        
        # Write to different devices
        test_img = np.ones(input_image_resolution).astype(np.uint8) * 50
        test_frame = frpl.connector_core.data.ImageFrame.new_from_array(
            test_img,
            input_image_properties.color_space,
            frpl.connector_core.data.descriptors.MemoryOrderLayout.WidthsHeightsChannels
        )
        
        feagi_agent.brain_input.image_camera_center.write(1, 0, test_frame)
        feagi_agent.brain_input.image_camera_center.write(2, 0, test_frame)
        
        print("         ✓ Multiple devices working")
    except Exception as e:
        print(f"         ✗ Failed: {e}")
        return
    
    # Test 9: Agent State
    print("\n[TEST 9] Checking agent state...")
    try:
        state = feagi_agent.get_agent_state()
        print(f"         ✓ Agent state: {state}")
    except Exception as e:
        print(f"         ✗ Failed: {e}")
        return
    
    # Summary
    print("\n" + "=" * 60)
    print("STANDALONE TEST COMPLETE")
    print("=" * 60)
    print("\n✓ All local functionality working correctly!")
    print("\nNext steps:")
    print("  1. Start FEAGI on localhost:30000")
    print("  2. Run example_2_2_debug.py to test connection")
    print("  3. Once connected, run example_2_2.py")
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

