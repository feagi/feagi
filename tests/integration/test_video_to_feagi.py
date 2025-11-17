"""
Integration Test: Video Streaming to FEAGI

This test demonstrates the complete flow:
1. Start FEAGI (TODO: Phase 2 - feagi.engine)
2. Load a genome
3. Read video frames from examples/vt_all.mov
4. Send frames to FEAGI using new SDK

Requirements:
- FEAGI running on localhost (or use feagi.engine when Phase 2 is complete)
- feagi_rust_py_libs installed
- opencv-python installed
- Video file at examples/vt_all.mov

Usage:
    pytest tests/integration/test_video_to_feagi.py -v
    
    # Or run standalone:
    python tests/integration/test_video_to_feagi.py
"""

try:
    import pytest
    PYTEST_AVAILABLE = True
except ImportError:
    PYTEST_AVAILABLE = False
    pytest = None

import time
import cv2
import numpy as np
import logging
from pathlib import Path
from typing import Optional

# Import new FEAGI SDK
from feagi.pns.inputs import Camera
from feagi.pns import brain_input

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')


class TestVideoToFEAGI:
    """Integration test for video streaming to FEAGI"""
    
    def feagi_config(self):
        """FEAGI connection configuration"""
        return {
            "host": "localhost",
            "sensory_port": 5558,
            "motor_port": 5564,
            "rest_port": 8000,
        }
    
    def video_path(self):
        """Path to test video file"""
        # Try multiple possible locations
        possible_paths = [
            Path(__file__).parent.parent.parent / "examples" / "vt_all.mov",
            Path.cwd() / "examples" / "vt_all.mov",
            Path("/Users/nadji/code/FEAGI-2.0/feagi-python-sdk/examples/vt_all.mov"),
        ]
        
        for path in possible_paths:
            if path.exists():
                return path
        
        pytest.skip(f"Video file not found. Tried: {[str(p) for p in possible_paths]}")
    
    def feagi_engine(self, feagi_config):
        """Start FEAGI engine for testing"""
        from feagi.engine import FeagiEngine
        
        # Try to find config and genome
        config_path = self._find_file("feagi_configuration.toml")
        genome_path = self._find_file("genome.json") or self._find_file("test_genome.json")
        
        if config_path is None:
            pytest.skip("No feagi_configuration.toml found. Please provide config file.")
        
        logger.info(f"Using config: {config_path}")
        if genome_path:
            logger.info(f"Using genome: {genome_path}")
        
        # Create engine
        engine = FeagiEngine()
        engine.load_config(str(config_path))
        
        if genome_path:
            engine.load_genome(str(genome_path))
        
        # Start FEAGI
        logger.info("Starting FEAGI engine...")
        if not engine.start(wait_for_ready=True, timeout=60.0):
            pytest.skip("Failed to start FEAGI engine")
        
        # Yield engine to test
        yield engine
        
        # Cleanup after test
        logger.info("Stopping FEAGI engine...")
        engine.stop()
    
    def _find_file(self, filename: str) -> Optional[Path]:
        """Find a file in common locations"""
        possible_dirs = [
            Path.cwd(),
            Path(__file__).parent.parent.parent,
            Path(__file__).parent.parent.parent / "feagi-core",
            Path("/Users/nadji/code/FEAGI-2.0/feagi-core"),
        ]
        
        for directory in possible_dirs:
            path = directory / filename
            if path.exists():
                return path
        
        return None
    
    def test_video_stream_to_feagi(self, feagi_config, video_path, feagi_engine):
        """
        Complete integration test: Read video frames and send to FEAGI
        
        This test:
        1. Opens a video file
        2. Registers a Camera input with the new SDK
        3. Configures connection to FEAGI
        4. Streams frames to FEAGI
        5. Verifies data is sent successfully
        """
        print(f"\n{'='*60}")
        print("FEAGI SDK 3.0 - Video Streaming Integration Test")
        print(f"{'='*60}\n")
        
        # Step 1: Open video file
        print(f"📹 Opening video: {video_path}")
        cap = cv2.VideoCapture(str(video_path))
        
        if not cap.isOpened():
            pytest.fail(f"Failed to open video file: {video_path}")
        
        # Get video properties
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        print(f"   Resolution: {width}x{height}")
        print(f"   FPS: {fps}")
        print(f"   Total frames: {total_frames}")
        
        # Step 2: Register Camera input using new SDK
        print(f"\n📥 Registering Camera input with FEAGI SDK 3.0...")
        
        camera = Camera.register(
            resolution=(width, height),
            encoding="absolute",
            position="center"
        )
        
        print(f"   ✓ Camera registered: {width}x{height}, encoding=absolute, position=center")
        
        # Step 3: Configure connection to FEAGI
        print(f"\n🔧 Configuring connection to FEAGI...")
        
        brain_input.configure(
            feagi_host=feagi_config["host"],
            feagi_port=feagi_config["sensory_port"],
            transport="zmq"
        )
        
        print(f"   Host: {feagi_config['host']}")
        print(f"   Port: {feagi_config['sensory_port']}")
        print(f"   Transport: ZMQ")
        
        # Step 4: Connect to FEAGI
        print(f"\n🔗 Connecting to FEAGI...")
        
        try:
            brain_input.connect()
            print(f"   ✓ Connected successfully!")
        except Exception as e:
            pytest.fail(f"Failed to connect to FEAGI: {e}")
        
        # Step 5: Stream frames to FEAGI
        print(f"\n🎬 Streaming frames to FEAGI...")
        print(f"   (Sending first 100 frames for testing)\n")
        
        frames_sent = 0
        frames_to_send = min(100, total_frames)  # Limit for testing
        start_time = time.time()
        
        try:
            while frames_sent < frames_to_send:
                # Read frame
                ret, frame_bgr = cap.read()
                if not ret:
                    break
                
                # Convert BGR to RGB (NumPy convention)
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                
                # Set frame in camera input
                camera.set_frame(frame_rgb)
                
                # Send to FEAGI (encodes and transmits)
                brain_input.send()
                
                frames_sent += 1
                
                # Progress update every 10 frames
                if frames_sent % 10 == 0:
                    elapsed = time.time() - start_time
                    current_fps = frames_sent / elapsed if elapsed > 0 else 0
                    print(f"   Frame {frames_sent}/{frames_to_send} "
                          f"({current_fps:.1f} fps)")
                
                # Pace by video FPS
                time.sleep(1.0 / fps)
            
        except KeyboardInterrupt:
            print("\n\n⚠️  Test interrupted by user")
        
        except Exception as e:
            pytest.fail(f"Error during frame streaming: {e}")
        
        finally:
            # Cleanup
            cap.release()
            brain_input.disconnect()
        
        # Step 6: Verify results
        elapsed_time = time.time() - start_time
        actual_fps = frames_sent / elapsed_time if elapsed_time > 0 else 0
        
        print(f"\n{'='*60}")
        print("📊 Test Results:")
        print(f"{'='*60}")
        print(f"   Frames sent: {frames_sent}/{frames_to_send}")
        print(f"   Time elapsed: {elapsed_time:.2f}s")
        print(f"   Actual FPS: {actual_fps:.1f}")
        print(f"   Target FPS: {fps:.1f}")
        print(f"   Status: {'✅ PASS' if frames_sent == frames_to_send else '⚠️  INCOMPLETE'}")
        print(f"{'='*60}\n")
        
        # Assert success
        assert frames_sent > 0, "No frames were sent"
        assert frames_sent == frames_to_send, f"Only sent {frames_sent}/{frames_to_send} frames"


# Standalone execution
if __name__ == "__main__":
    print("\n" + "="*60)
    print("FEAGI SDK 3.0 - Video Streaming Integration Test")
    print("="*60)
    print("\nRunning standalone (not using pytest)...\n")
    
    # Create test instance
    test = TestVideoToFEAGI()
    
    # Setup fixtures manually
    feagi_config = {
        "host": "localhost",
        "sensory_port": 5558,
        "motor_port": 5564,
        "rest_port": 8000,
    }
    
    # Find video file
    video_path = None
    possible_paths = [
        Path(__file__).parent.parent.parent / "examples" / "vt_all.mov",
        Path.cwd() / "examples" / "vt_all.mov",
    ]
    
    for path in possible_paths:
        if path.exists():
            video_path = path
            break
    
    if not video_path:
        print("❌ Error: Video file not found!")
        print(f"   Tried: {[str(p) for p in possible_paths]}")
        print("\n   Please place vt_all.mov in examples/ directory")
        exit(1)
    
    # Start FEAGI
    from feagi.engine import FeagiEngine
    
    config_path = None
    genome_path = None
    
    # Find config
    for d in [Path.cwd(), Path(__file__).parent.parent.parent]:
        p = d / "feagi_configuration.toml"
        if p.exists():
            config_path = p
            break
    
    if not config_path:
        print("❌ No feagi_configuration.toml found!")
        exit(1)
    
    print(f"\n📝 Using config: {config_path}")
    
    # Create and start engine
    engine = FeagiEngine()
    engine.load_config(str(config_path))
    
    # Find and load genome if available
    for d in [Path.cwd(), Path(__file__).parent.parent.parent]:
        for gname in ["genome.json", "test_genome.json"]:
            p = d / gname
            if p.exists():
                genome_path = p
                break
        if genome_path:
            break
    
    if genome_path:
        print(f"📝 Using genome: {genome_path}")
        engine.load_genome(str(genome_path))
    
    print("\n🚀 Starting FEAGI engine...")
    # Start without health check for now (REST API might take time to initialize)
    if not engine.start(wait_for_ready=False):
        print("❌ Failed to start FEAGI!")
        exit(1)
    
    # Give FEAGI time to initialize
    print("⏳ Waiting for FEAGI to initialize (10s)...")
    time.sleep(10)
    
    print("✅ FEAGI is running!\n")
    
    feagi_engine = engine
    
    # Run test
    try:
        test.test_video_stream_to_feagi(feagi_config, video_path, feagi_engine)
        print("\n✅ Test completed successfully!\n")
    except Exception as e:
        print(f"\n❌ Test failed: {e}\n")
        import traceback
        traceback.print_exc()
        exit(1)
    finally:
        # Stop FEAGI
        print("\n🛑 Stopping FEAGI...")
        engine.stop()
        print("✅ FEAGI stopped\n")

