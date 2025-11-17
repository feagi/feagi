"""
Integration Test: VideoStreamAgent

Tests the high-level VideoStreamAgent class for video streaming.
VideoStreamAgent focuses ONLY on video streaming - FEAGI lifecycle is managed separately.

Usage:
    pytest tests/integration/test_video_agent.py -v
    
    # Or run standalone:
    python tests/integration/test_video_agent.py
"""

try:
    import pytest
    PYTEST_AVAILABLE = True
except ImportError:
    PYTEST_AVAILABLE = False
    pytest = None

import logging
from pathlib import Path

from feagi.engine import FeagiEngine
from feagi.agent import VideoStreamAgent

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')


class TestVideoStreamAgent:
    """Integration test for VideoStreamAgent"""
    
    def video_path(self):
        """Find test video file"""
        possible_paths = [
            Path(__file__).parent.parent.parent / "examples" / "vt_all.mov",
            Path.cwd() / "examples" / "vt_all.mov",
        ]
        
        for path in possible_paths:
            if path.exists():
                return str(path)
        
        if PYTEST_AVAILABLE:
            pytest.skip(f"Video file not found. Tried: {[str(p) for p in possible_paths]}")
        else:
            raise FileNotFoundError("Video file not found")
    
    def config_path(self):
        """Find config file"""
        possible_paths = [
            Path.cwd() / "feagi_configuration.toml",
            Path(__file__).parent.parent.parent / "feagi_configuration.toml",
        ]
        
        for path in possible_paths:
            if path.exists():
                return str(path)
        
        if PYTEST_AVAILABLE:
            pytest.skip("No feagi_configuration.toml found")
        else:
            raise FileNotFoundError("No feagi_configuration.toml found")
    
    def test_video_stream_basic(self, video_path, config_path):
        """
        Test basic video streaming with clean separation of concerns.
        
        User manages FEAGI lifecycle, agent handles ONLY video streaming.
        """
        print(f"\n{'='*60}")
        print("FEAGI SDK 3.0 - VideoStreamAgent Basic Test")
        print(f"{'='*60}\n")
        
        # User starts FEAGI
        engine = FeagiEngine()
        engine.load_config(config_path)
        engine.start(wait_for_ready=False)
        
        try:
            # Agent handles ONLY video streaming
            with VideoStreamAgent(video_path) as agent:
                frames_sent = agent.run(
                    max_frames=100,
                    progress_interval=10,
                    pace_by_fps=True
                )
            
            # Verify results
            print(f"\n{'='*60}")
            print("📊 Test Results:")
            print(f"{'='*60}")
            print(f"   Frames sent: {frames_sent}")
            print(f"   Status: {'✅ PASS' if frames_sent > 0 else '❌ FAIL'}")
            print(f"{'='*60}\n")
            
            # Assert success
            assert frames_sent > 0, "No frames were sent"
            assert frames_sent <= 100, f"Sent too many frames: {frames_sent}"
        
        finally:
            # User stops FEAGI
            engine.stop()
    
    def test_video_agent_generator(self, video_path, config_path):
        """
        Test VideoStreamAgent as generator for sensorimotor loop.
        """
        print(f"\n{'='*60}")
        print("VideoStreamAgent - Generator Pattern Test")
        print(f"{'='*60}\n")
        
        # User starts FEAGI
        engine = FeagiEngine()
        engine.load_config(config_path)
        engine.start(wait_for_ready=False)
        
        try:
            # Create agent
            agent = VideoStreamAgent(video_path)
            agent.start()
            
            try:
                frame_count = 0
                for frame_num, frame_data in agent.stream(max_frames=50):
                    frame_count += 1
                    
                    # Verify frame data
                    assert frame_data is not None
                    assert len(frame_data.shape) == 3  # (height, width, channels)
                    assert frame_data.shape[2] == 3     # RGB channels
                    
                    # Here you would process motor commands
                    # motor_data = brain_output.receive()
                    
                    if frame_num % 10 == 0:
                        print(f"   Frame {frame_num}: shape={frame_data.shape}")
                
                print(f"\n   ✅ Processed {frame_count} frames in sensorimotor loop")
                
                # Assert success
                assert frame_count == 50, f"Expected 50 frames, got {frame_count}"
            
            finally:
                agent.stop()
        
        finally:
            # User stops FEAGI
            engine.stop()


# Standalone execution
if __name__ == "__main__":
    print("\n" + "="*60)
    print("FEAGI SDK 3.0 - VideoStreamAgent Integration Test")
    print("="*60)
    print("\nRunning standalone (not using pytest)...\n")
    
    # Create test instance
    test = TestVideoStreamAgent()
    
    # Find files
    try:
        video_path = test.video_path()
        config_path = test.config_path()
    except FileNotFoundError as e:
        print(f"❌ {e}")
        exit(1)
    
    # Run tests
    try:
        print("\nTest 1: Basic video streaming")
        print("-" * 60)
        test.test_video_stream_basic(video_path, config_path)
        
        print("\nTest 2: Generator pattern (sensorimotor loop)")
        print("-" * 60)
        test.test_video_agent_generator(video_path, config_path)
        
        print("\n" + "="*60)
        print("✅ All tests passed!")
        print("="*60)
        print("\nKey points:")
        print("  ✅ User controls FEAGI lifecycle")
        print("  ✅ Agent focuses ONLY on video streaming")
        print("  ✅ Generator pattern works for sensorimotor loops")
        print("  ✅ Clean separation of concerns")
        print("="*60 + "\n")
    
    except Exception as e:
        print(f"\n❌ Test failed: {e}\n")
        import traceback
        traceback.print_exc()
        exit(1)
