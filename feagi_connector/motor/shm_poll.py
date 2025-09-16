"""
Motor polling helper using ShmBytesReader.

Provides a simple loop function for background polling of motor bytes from SHM
and calling a user callback with the payload.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Callable, Optional

from feagi_connector.utils.shm import ShmBytesReader


def poll_motor_shm(path: Path, stop_flag: Callable[[], bool], on_payload: Callable[[bytes], None], interval_sec: float = 0.02) -> None:
    reader: Optional[ShmBytesReader] = None
    try:
        reader = ShmBytesReader(Path(path))
        while not stop_flag():
            try:
                data = reader.read_latest()
                if data:
                    on_payload(data)
            except Exception:
                pass
            time.sleep(interval_sec)
    finally:
        try:
            if reader is not None:
                reader.close()
        except Exception:
            pass


