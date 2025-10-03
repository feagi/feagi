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
    """Continuously read latest motor bytes from SHM with resilient startup.

    Handles the case where the SHM file exists but header is not yet initialized
    by the writer (size < header). In that case it will retry opening until ready.
    """
    reader: Optional[ShmBytesReader] = None
    try:
        while not stop_flag():
            # Ensure reader is open and header is initialized
            if reader is None:
                try:
                    reader = ShmBytesReader(Path(path))
                except (FileNotFoundError, ValueError):
                    # File not ready or header not initialized yet; retry shortly
                    time.sleep(max(0.05, interval_sec))
                    continue
                except Exception:
                    time.sleep(max(0.05, interval_sec))
                    continue

            try:
                data = reader.read_latest()
                if data:
                    on_payload(data)
                time.sleep(interval_sec)
            except Exception:
                # If read fails (e.g., writer rotated or header changed), reopen
                try:
                    reader.close()
                except Exception:
                    pass
                reader = None
                time.sleep(max(0.05, interval_sec))
                continue
    finally:
        try:
            if reader is not None:
                reader.close()
        except Exception:
            pass


