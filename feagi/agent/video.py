"""
Video Stream Agent - High-level helper for streaming video to FEAGI

This module provides VideoStreamAgent, which handles video file streaming
to FEAGI. The agent focuses ONLY on video streaming - FEAGI engine lifecycle
is managed separately by the user.

Example:
    from feagi.engine import FeagiEngine
    from feagi.agent import VideoStreamAgent
    
    # User manages FEAGI lifecycle
    engine = FeagiEngine()
    engine.load_config("feagi_configuration.toml")
    engine.start()
    
    # Agent handles ONLY video streaming
    with VideoStreamAgent("video.mp4") as agent:
        frames_sent = agent.run()
    
    engine.stop()
    
    # Or use as generator for motor control
    with VideoStreamAgent("video.mp4") as agent:
        for frame_num, frame_data in agent.stream():
            # Process vision
            # Send motor commands
            # Full control
            pass
"""

import time
import logging
from pathlib import Path
from typing import Optional, Generator, Tuple, Any
import numpy as np

from feagi.agent.base import BaseAgent
from feagi.pns.inputs import Camera
from feagi.pns import brain_input

logger = logging.getLogger(__name__)


class VideoStreamAgent(BaseAgent):
    """
    High-level agent for streaming video files to FEAGI.
    
    Focuses ONLY on video streaming. FEAGI engine lifecycle is managed
    separately by the user using feagi.engine.FeagiEngine.
    
    Handles:
    - Video file opening and property detection
    - Camera input registration
    - Connection to FEAGI (assumes FEAGI is already running)
    - Frame loop with FPS timing
    - BGR to RGB conversion
    - Progress reporting
    - Cleanup
    
    Attributes:
        video_path: Path to video file
        width: Video width in pixels
        height: Video height in pixels
        fps: Video frames per second
        total_frames: Total number of frames in video
        camera: Registered Camera input
    """
    
    def __init__(
        self,
        video_path: str,
        camera_position: str = "center",
        camera_encoding: str = "absolute",
        auto_detect: bool = True,
        agent_id: str = "video_stream_agent",
        feagi_host: str = "localhost"
    ):
        """
        Initialize VideoStreamAgent.
        
        Note: FEAGI engine must be started separately before using this agent.
        
        Args:
            video_path: Path to video file
            camera_position: Camera position for registration (default: "center")
            camera_encoding: Encoding type (default: "absolute")
            auto_detect: Auto-detect video properties on init (default: True)
            agent_id: Unique agent identifier (default: "video_stream_agent")
            feagi_host: FEAGI hostname (default: "localhost")
        """
        super().__init__(agent_id=agent_id, feagi_host=feagi_host)
        
        self.video_path = Path(video_path)
        if not self.video_path.exists():
            raise FileNotFoundError(f"Video file not found: {video_path}")
        
        # Video properties (will be set by _detect_video_properties)
        self.width = None
        self.height = None
        self.fps = None
        self.total_frames = None
        
        # Camera settings
        self.camera_position = camera_position
        self.camera_encoding = camera_encoding
        
        # Agent state
        self.camera = None
        self._cap = None
        self._started = False
        
        # Auto-detect video properties
        if auto_detect:
            self._detect_video_properties()
            logger.info(
                f"Video detected: {self.width}x{self.height} @ {self.fps:.1f} fps, "
                f"{self.total_frames} frames"
            )
    
    def _detect_video_properties(self):
        """Detect video properties (resolution, FPS, frame count)"""
        try:
            import cv2
        except ImportError:
            raise ImportError(
                "opencv-python is required for VideoStreamAgent. "
                "Install with: pip install opencv-python"
            )
        
        cap = cv2.VideoCapture(str(self.video_path))
        
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open video file: {self.video_path}")
        
        try:
            self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
            self.total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        finally:
            cap.release()
    
    def start(
        self,
        feagi_host: str = "localhost",
        feagi_port: int = 5558,
        transport: str = "zmq"
    ):
        """
        Connect to FEAGI and register camera.
        
        Note: FEAGI engine must already be running before calling this.
        
        Args:
            feagi_host: FEAGI hostname (default: "localhost")
            feagi_port: FEAGI sensory port (default: 5558)
            transport: Transport protocol (default: "zmq")
        """
        if self._started:
            logger.warning("VideoStreamAgent already started")
            return
        
        logger.info("Starting VideoStreamAgent...")
        
        # 1. Register camera input
        logger.info(f"Registering camera: {self.width}x{self.height}")
        self.camera = Camera.register(
            resolution=(self.width, self.height),
            encoding=self.camera_encoding,
            position=self.camera_position
        )
        
        # 2. Configure and connect to FEAGI (assumes FEAGI is already running)
        logger.info(f"Connecting to FEAGI at {feagi_host}:{feagi_port}")
        brain_input.configure(
            feagi_host=feagi_host,
            feagi_port=feagi_port,
            transport=transport
        )
        brain_input.connect()
        
        self._started = True
        logger.info("VideoStreamAgent ready!")
    
    def stream(
        self,
        max_frames: Optional[int] = None,
        progress_interval: int = 10,
        pace_by_fps: bool = True
    ) -> Generator[Tuple[int, np.ndarray], None, None]:
        """
        Stream video frames to FEAGI.
        
        This is a generator that yields each frame after sending it to FEAGI.
        Automatically handles BGR→RGB conversion, FPS pacing, and progress reporting.
        
        Args:
            max_frames: Maximum number of frames to stream (None = all)
            progress_interval: Print progress every N frames (0 = no progress)
            pace_by_fps: Sleep between frames to match video FPS (default: True)
        
        Yields:
            Tuple of (frame_number, frame_rgb_array)
        
        Example:
            for frame_num, frame in agent.stream(max_frames=100):
                # Custom logic here
                if frame_num % 10 == 0:
                    print(f"Processed {frame_num} frames")
        """
        if not self._started:
            raise RuntimeError("Agent not started. Call start() first.")
        
        try:
            import cv2
        except ImportError:
            raise ImportError("opencv-python required")
        
        # Open video
        self._cap = cv2.VideoCapture(str(self.video_path))
        if not self._cap.isOpened():
            raise RuntimeError(f"Failed to open video: {self.video_path}")
        
        frame_count = 0
        start_time = time.time()
        frames_to_stream = max_frames or self.total_frames
        
        try:
            while True:
                # Check limits
                if max_frames and frame_count >= max_frames:
                    break
                
                # Read frame
                ret, frame_bgr = self._cap.read()
                if not ret:
                    break
                
                # Convert BGR → RGB (FEAGI/NumPy convention)
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                
                # Set frame in camera input
                self.camera.set_frame(frame_rgb)
                
                # Send to FEAGI (encodes and transmits)
                brain_input.send()
                
                frame_count += 1
                
                # Progress reporting
                if progress_interval > 0 and frame_count % progress_interval == 0:
                    elapsed = time.time() - start_time
                    current_fps = frame_count / elapsed if elapsed > 0 else 0
                    logger.info(
                        f"Frame {frame_count}/{frames_to_stream} "
                        f"({current_fps:.1f} fps)"
                    )
                
                # Yield frame for custom processing
                yield frame_count, frame_rgb
                
                # Pace by video FPS
                if pace_by_fps:
                    time.sleep(1.0 / self.fps)
        
        finally:
            if self._cap:
                self._cap.release()
                self._cap = None
    
    def run(
        self,
        max_frames: Optional[int] = None,
        progress_interval: int = 10,
        pace_by_fps: bool = True
    ) -> int:
        """
        One-liner: Stream entire video to FEAGI.
        
        This is the simplest way to use VideoStreamAgent. It handles everything:
        - Opens video
        - Streams all frames
        - Reports progress
        - Returns frame count
        
        Args:
            max_frames: Maximum frames to stream (None = all)
            progress_interval: Print progress every N frames (0 = no progress)
            pace_by_fps: Sleep between frames to match video FPS
        
        Returns:
            Number of frames streamed
        
        Example:
            with VideoStreamAgent("video.mp4") as agent:
                frames_sent = agent.run()
        """
        if not self._started:
            raise RuntimeError("Agent not started. Call start() first or use context manager.")
        
        logger.info(f"Streaming video: {self.video_path}")
        
        frame_count = 0
        start_time = time.time()
        
        try:
            for frame_count, _ in self.stream(
                max_frames=max_frames,
                progress_interval=progress_interval,
                pace_by_fps=pace_by_fps
            ):
                pass  # Just stream, no custom processing
        
        except KeyboardInterrupt:
            logger.warning("Streaming interrupted by user")
        
        # Report results
        elapsed = time.time() - start_time
        actual_fps = frame_count / elapsed if elapsed > 0 else 0
        
        logger.info(
            f"Streaming complete: {frame_count} frames in {elapsed:.2f}s "
            f"({actual_fps:.1f} fps)"
        )
        
        return frame_count
    
    def stop(self):
        """Disconnect from FEAGI and cleanup resources"""
        if not self._started:
            return
        
        logger.info("Stopping VideoStreamAgent...")
        
        # Disconnect from FEAGI
        try:
            brain_input.disconnect()
        except Exception as e:
            logger.warning(f"Error disconnecting: {e}")
        
        # Release video capture if still open
        if self._cap:
            self._cap.release()
        
        self._started = False
        logger.info("VideoStreamAgent stopped")
    
    def __enter__(self):
        """Context manager entry: auto-start"""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit: auto-cleanup"""
        self.stop()
        return False
    
    # BaseAgent abstract method implementations
    # (VideoStreamAgent handles video input differently than typical agents)
    
    def initialize_hardware(self):
        """
        Initialize video hardware (already handled in __init__).
        
        For VideoStreamAgent, hardware initialization means opening
        the video file and detecting its properties, which is done
        in __init__ via _detect_video_properties().
        """
        pass  # Already handled in __init__
    
    def map_sensors(self, hardware_data):
        """
        Map video frame to FEAGI sensor format.
        
        For VideoStreamAgent, this is handled automatically by the
        Camera input class during stream().
        
        Args:
            hardware_data: Video frame (not used directly)
        
        Returns:
            Empty dict (mapping handled by Camera input)
        """
        return {}  # Camera input handles encoding
    
    def map_motors(self, feagi_output):
        """
        Map FEAGI motor output (not applicable for video streaming).
        
        VideoStreamAgent is input-only and doesn't process motor commands.
        
        Args:
            feagi_output: FEAGI motor commands (ignored)
        
        Returns:
            Empty dict (no motors)
        """
        return {}  # No motor output for video streaming

