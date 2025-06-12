"""
FEAGI LZ4 Compression Utility

LZ4 compression service for ZMQ streams (visualization, motor, etc.)
optimized for real-time neural data streaming with TOML configuration support.

Features:
- LZ4 fast compression with TOML config control
- Performance monitoring
- Configurable compression by stream type
- Graceful handling when compression disabled or LZ4 unavailable
"""

import threading
import time
from typing import Any, Dict, Optional, Tuple

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class LZ4Compressor:
    """
    LZ4 compression service for FEAGI ZMQ streams with TOML configuration support.

    Respects compression settings from feagi_configuration.toml and handles cases
    where compression is disabled or LZ4 is unavailable.
    """

    def __init__(
        self,
        enabled: bool = True,
        min_size_threshold: int = 100,
        enable_stats: bool = True,
        require_lz4: bool = True,
    ):
        """
        Initialize LZ4 compressor with configuration settings.

        Args:
            enabled: Whether compression is enabled via TOML config
            min_size_threshold: Minimum payload size to attempt compression
            enable_stats: Whether to collect performance statistics
            require_lz4: Whether to require LZ4 availability (True = fail if unavailable)
        """
        self.config_enabled = enabled
        self.min_size_threshold = min_size_threshold
        self.enable_stats = enable_stats
        self.require_lz4 = require_lz4

        # LZ4 module availability
        self._lz4_module = None
        self._lz4_available = self._initialize_lz4()

        # Final operational state
        self.operational = self.config_enabled and self._lz4_available

        # Statistics
        self.stats = {
            "total_compressions": 0,
            "total_bytes_in": 0,
            "total_bytes_out": 0,
            "total_time_ms": 0.0,
            "failed_compressions": 0,
            "skipped_compressions": 0,
            "config_disabled_count": 0,
        }
        self._stats_lock = threading.Lock()

        # Log initialization status
        if not self.config_enabled:
            logger.info("[COMPRESSION] LZ4 compression DISABLED by configuration")
        elif not self._lz4_available:
            if self.require_lz4:
                logger.error("[COMPRESSION] LZ4 compression REQUIRED but not available")
                raise ImportError(
                    "LZ4 compression is required but lz4 package is not installed"
                )
            else:
                logger.warning("[COMPRESSION] LZ4 not available - compression disabled")
        else:
            logger.info("[COMPRESSION] LZ4 compression initialized and ready")

    def _initialize_lz4(self) -> bool:
        """
        Initialize LZ4 compression module.

        Returns:
            True if LZ4 is available, False otherwise
        """
        try:
            import lz4.frame

            self._lz4_module = lz4.frame
            return True
        except ImportError as e:
            logger.debug(f"[COMPRESSION] LZ4 not available: {e}")
            return False

    def is_enabled(self) -> bool:
        """Check if compression is enabled and operational."""
        return self.operational

    def compress(self, data: bytes) -> Tuple[bytes, Dict[str, Any]]:
        """
        Compress data using LZ4 based on configuration.

        Args:
            data: Raw bytes to compress

        Returns:
            Tuple of (data, compression_info)
            - If compression disabled/unavailable: returns original data
            - If compression enabled: returns compressed data (or original if no benefit)
        """
        # Handle disabled compression
        if not self.config_enabled:
            if self.enable_stats:
                with self._stats_lock:
                    self.stats["config_disabled_count"] += 1

            return data, {
                "compressed": False,
                "ratio": 1.0,
                "time_ms": 0.0,
                "bytes_saved": 0,
                "reason": "compression_disabled_by_config",
            }

        # Handle LZ4 unavailable
        if not self._lz4_available:
            return data, {
                "compressed": False,
                "ratio": 1.0,
                "time_ms": 0.0,
                "bytes_saved": 0,
                "reason": "lz4_not_available",
            }

        # Handle small payloads
        if len(data) < self.min_size_threshold:
            if self.enable_stats:
                with self._stats_lock:
                    self.stats["skipped_compressions"] += 1

            return data, {
                "compressed": False,
                "ratio": 1.0,
                "time_ms": 0.0,
                "bytes_saved": 0,
                "reason": "payload_too_small",
            }

        # Perform compression
        start_time = time.perf_counter()

        try:
            compressed_data = self._lz4_module.compress(data)
            compression_time_ms = (time.perf_counter() - start_time) * 1000

            # Only use compressed data if it's actually smaller
            if len(compressed_data) < len(data):
                final_data = compressed_data
                compressed = True
                bytes_saved = len(data) - len(compressed_data)
                ratio = len(compressed_data) / len(data)
                reason = "compressed"
            else:
                final_data = data
                compressed = False
                bytes_saved = 0
                ratio = 1.0
                reason = "no_compression_benefit"
                if self.enable_stats:
                    with self._stats_lock:
                        self.stats["skipped_compressions"] += 1

            # Update statistics
            if self.enable_stats:
                with self._stats_lock:
                    self.stats["total_compressions"] += 1
                    self.stats["total_bytes_in"] += len(data)
                    self.stats["total_bytes_out"] += len(final_data)
                    self.stats["total_time_ms"] += compression_time_ms

            return final_data, {
                "compressed": compressed,
                "ratio": ratio,
                "time_ms": compression_time_ms,
                "bytes_saved": bytes_saved,
                "reason": reason,
            }

        except Exception as e:
            logger.warning(f"[COMPRESSION] LZ4 compression failed: {e}")

            if self.enable_stats:
                with self._stats_lock:
                    self.stats["failed_compressions"] += 1

            return data, {
                "compressed": False,
                "ratio": 1.0,
                "time_ms": 0.0,
                "bytes_saved": 0,
                "reason": f"compression_error: {e}",
            }

    def get_stats(self) -> Dict[str, Any]:
        """Get compression statistics."""
        with self._stats_lock:
            total_compressions = max(self.stats["total_compressions"], 1)
            bytes_saved = max(
                0, self.stats["total_bytes_in"] - self.stats["total_bytes_out"]
            )

            return {
                "compression_enabled": self.config_enabled,
                "lz4_available": self._lz4_available,
                "operational": self.operational,
                "compression_type": "lz4",
                "total_compressions": self.stats["total_compressions"],
                "total_bytes_in": self.stats["total_bytes_in"],
                "total_bytes_out": self.stats["total_bytes_out"],
                "bytes_saved": bytes_saved,
                "compression_ratio": self.stats["total_bytes_out"]
                / max(self.stats["total_bytes_in"], 1),
                "bandwidth_savings_percent": (
                    bytes_saved / max(self.stats["total_bytes_in"], 1)
                )
                * 100,
                "avg_compression_time_ms": self.stats["total_time_ms"]
                / total_compressions,
                "failed_compressions": self.stats["failed_compressions"],
                "skipped_compressions": self.stats["skipped_compressions"],
                "config_disabled_count": self.stats["config_disabled_count"],
                "success_rate": (
                    (
                        self.stats["total_compressions"]
                        - self.stats["failed_compressions"]
                    )
                    / total_compressions
                )
                * 100,
            }

    def reset_stats(self) -> None:
        """Reset compression statistics."""
        with self._stats_lock:
            self.stats = {
                "total_compressions": 0,
                "total_bytes_in": 0,
                "total_bytes_out": 0,
                "total_time_ms": 0.0,
                "failed_compressions": 0,
                "skipped_compressions": 0,
                "config_disabled_count": 0,
            }


def create_lz4_compressor(
    enabled: bool = True,
    min_size_threshold: int = 100,
    enable_stats: bool = True,
    require_lz4: bool = False,
) -> LZ4Compressor:
    """
    Create LZ4 compressor for FEAGI streams with TOML configuration support.

    Args:
        enabled: Whether compression is enabled via configuration
        min_size_threshold: Minimum payload size for compression
        enable_stats: Whether to collect statistics
        require_lz4: Whether to require LZ4 (True = raise error if unavailable)

    Returns:
        LZ4Compressor instance

    Raises:
        ImportError: If require_lz4=True and LZ4 is not available
    """
    return LZ4Compressor(
        enabled=enabled,
        min_size_threshold=min_size_threshold,
        enable_stats=enable_stats,
        require_lz4=require_lz4,
    )
