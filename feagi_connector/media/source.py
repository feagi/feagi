"""
Media source utilities for video frames using OpenCV.

Encapsulates:
- Webcam probing (0,1,2)
- File capture with loop-on-end
- Optional mirroring for webcam
- FPS/size discovery
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple
import time

import cv2
import numpy as np


@dataclass
class MediaInfo:
    width: int
    height: int
    fps: float
    total_frames: int
    live: bool


class MediaSource:
    def __init__(self, use_webcam: bool, path: Optional[str], mirror: bool = True) -> None:
        self.use_webcam = bool(use_webcam)
        self.path = path
        self.mirror = bool(mirror)
        self.cap: Optional[cv2.VideoCapture] = None
        self.info: Optional[MediaInfo] = None

    def open(self) -> bool:
        if self.use_webcam:
            for idx in [0, 1, 2]:
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    ok, frame = cap.read()
                    if ok and frame is not None:
                        self.cap = cap
                        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        fps = self.cap.get(cv2.CAP_PROP_FPS)
                        if fps <= 0:
                            fps = 30.0
                        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        self.info = MediaInfo(width=w, height=h, fps=float(fps), total_frames=2**31-1, live=True)
                        return True
                    cap.release()
            return False
        else:
            cap = cv2.VideoCapture(str(self.path))
            if not cap.isOpened():
                return False
            self.cap = cap
            total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            fps = float(self.cap.get(cv2.CAP_PROP_FPS))
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.info = MediaInfo(width=w, height=h, fps=fps, total_frames=total, live=False)
            return True

    def read(self) -> Optional[np.ndarray]:
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        if not ok or frame is None:
            if self.info and not self.info.live:
                # loop file
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
                if not ok or frame is None:
                    return None
            else:
                return None
        if self.use_webcam and self.mirror:
            frame = cv2.flip(frame, 1)
        return frame

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None
