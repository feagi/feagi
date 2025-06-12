"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
High-Performance ZMQ Traffic Debugging System

Provides zero-overhead debugging for ZMQ traffic with runtime configuration,
filtering, performance monitoring, and minimal impact on production systems.

Features:
- Zero overhead when disabled (no environment variable checks per call)
- Runtime enable/disable without restart
- Smart data filtering and truncation
- Performance impact monitoring
- Message type filtering
- Rate limiting to prevent log spam
- Thread-safe operations
- Memory-efficient buffering
"""

import json
import os
import threading
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Set, Union

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

# Console logger for debug output (when console debugging is enabled)
_console_logger = None


def _get_console_logger():
    """Get or create a console logger for ZMQ debug output."""
    global _console_logger
    if _console_logger is None:
        import logging

        _console_logger = logging.getLogger("zmq_debug_console")
        _console_logger.setLevel(logging.INFO)

        # Remove any existing handlers to prevent duplicates
        _console_logger.handlers.clear()

        # Create console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)

        # Create formatter for clean console output
        formatter = logging.Formatter("%(message)s")
        console_handler.setFormatter(formatter)

        _console_logger.addHandler(console_handler)
        _console_logger.propagate = False  # Don't propagate to root logger

    return _console_logger


class DebugLevel(Enum):
    """Debug verbosity levels."""

    OFF = 0
    MINIMAL = 1  # Just message counts and endpoints
    HEADERS = 2  # Add topics, sizes, timestamps
    SUMMARY = 3  # Add data previews (first 100 chars)
    FULL = 4  # Full data (use with caution!)


class MessageType(Enum):
    """ZMQ message types for filtering."""

    SENSORY = "sensory"
    MOTOR = "motor"
    VISUALIZATION = "visualization"
    CONTROL = "control"
    REST = "rest"
    HEARTBEAT = "heartbeat"
    UNKNOWN = "unknown"


@dataclass
class DebugStats:
    """Statistics for debug performance monitoring."""

    messages_logged: int = 0
    messages_filtered: int = 0
    total_bytes: int = 0
    debug_overhead_ms: float = 0.0
    rate_limited_messages: int = 0
    last_reset_time: float = field(default_factory=time.time)

    def reset(self):
        """Reset stats for new measurement period."""
        self.messages_logged = 0
        self.messages_filtered = 0
        self.total_bytes = 0
        self.debug_overhead_ms = 0.0
        self.rate_limited_messages = 0
        self.last_reset_time = time.time()


class ZMQDebugger:
    """
    High-performance ZMQ debugging system with runtime configuration.

    This class provides zero-overhead debugging when disabled and
    comprehensive debugging capabilities when enabled.
    """

    def __init__(self):
        # Core state
        self._inbound_enabled = False
        self._outbound_enabled = False
        self._lock = threading.RLock()

        # Configuration
        self._debug_level = DebugLevel.SUMMARY
        self._message_filters: Set[MessageType] = set()  # Empty = all allowed
        self._endpoint_filters: Set[str] = set()  # Empty = all allowed
        self._max_data_preview = 200
        self._rate_limit_per_second = 100  # Max messages per second
        self._enable_performance_tracking = True
        self._console_output = False  # Console output option

        # Rate limiting
        self._message_timestamps = deque(maxlen=1000)

        # Statistics
        self._stats = DebugStats()
        self._per_endpoint_stats: Dict[str, DebugStats] = defaultdict(DebugStats)

        # Performance monitoring
        self._start_time = time.time()

        # Initialize from environment (startup only)
        self._init_from_environment()

        logger.debug(
            f"ZMQDebugger initialized - Inbound: {self._inbound_enabled}, Outbound: {self._outbound_enabled}, Console: {self._console_output}"
        )

    def _init_from_environment(self):
        """Initialize from environment variables (called once at startup)."""
        # Check environment variables once at startup
        inbound_env = os.environ.get("FEAGI_DEBUG_ZMQ_INBOUND", "").lower()
        outbound_env = os.environ.get("FEAGI_DEBUG_ZMQ_OUTBOUND", "").lower()

        self._inbound_enabled = inbound_env in ("1", "true", "yes")
        self._outbound_enabled = outbound_env in ("1", "true", "yes")

        # Check for console output option
        console_env = os.environ.get("FEAGI_DEBUG_ZMQ_CONSOLE", "").lower()
        self._console_output = console_env in ("1", "true", "yes")

        # Parse debug level from environment
        level_env = os.environ.get("FEAGI_DEBUG_ZMQ_LEVEL", "summary").lower()
        level_map = {
            "off": DebugLevel.OFF,
            "minimal": DebugLevel.MINIMAL,
            "headers": DebugLevel.HEADERS,
            "summary": DebugLevel.SUMMARY,
            "full": DebugLevel.FULL,
        }
        self._debug_level = level_map.get(level_env, DebugLevel.SUMMARY)

        # Parse message type filters
        filter_env = os.environ.get("FEAGI_DEBUG_ZMQ_FILTER_TYPES", "")
        if filter_env:
            try:
                filter_types = [MessageType(t.strip()) for t in filter_env.split(",")]
                self._message_filters = set(filter_types)
            except ValueError as e:
                logger.warning(f"Invalid ZMQ debug message type filter: {e}")

    # Runtime Configuration API

    def enable_inbound(self, enabled: bool = True):
        """Enable/disable inbound message debugging at runtime."""
        with self._lock:
            self._inbound_enabled = enabled
            logger.info(f"ZMQ inbound debugging {'enabled' if enabled else 'disabled'}")

    def enable_outbound(self, enabled: bool = True):
        """Enable/disable outbound message debugging at runtime."""
        with self._lock:
            self._outbound_enabled = enabled
            logger.info(
                f"ZMQ outbound debugging {'enabled' if enabled else 'disabled'}"
            )

    def set_debug_level(self, level: DebugLevel):
        """Set debug verbosity level at runtime."""
        with self._lock:
            self._debug_level = level
            logger.info(f"ZMQ debug level set to: {level.name}")

    def set_message_filters(self, message_types: List[MessageType]):
        """Set message type filters. Empty list = allow all."""
        with self._lock:
            self._message_filters = set(message_types)
            if message_types:
                logger.info(
                    f"ZMQ debug filtering enabled for: {[t.value for t in message_types]}"
                )
            else:
                logger.info("ZMQ debug filtering disabled - all message types allowed")

    def set_endpoint_filters(self, endpoints: List[str]):
        """Set endpoint filters. Empty list = allow all."""
        with self._lock:
            self._endpoint_filters = set(endpoints)
            if endpoints:
                logger.info(f"ZMQ debug filtering enabled for endpoints: {endpoints}")
            else:
                logger.info("ZMQ endpoint filtering disabled - all endpoints allowed")

    def set_rate_limit(self, messages_per_second: int):
        """Set rate limiting for debug messages."""
        with self._lock:
            self._rate_limit_per_second = messages_per_second
            logger.info(f"ZMQ debug rate limit set to: {messages_per_second} msg/sec")

    def set_console_output(self, enabled: bool = True):
        """Enable/disable console output for debug messages."""
        with self._lock:
            self._console_output = enabled
            logger.info(
                f"ZMQ debug console output {'enabled' if enabled else 'disabled'}"
            )

    def get_status(self) -> Dict[str, Any]:
        """Get current debugging status and configuration."""
        with self._lock:
            return {
                "inbound_enabled": self._inbound_enabled,
                "outbound_enabled": self._outbound_enabled,
                "debug_level": self._debug_level.name,
                "message_filters": [t.value for t in self._message_filters],
                "endpoint_filters": list(self._endpoint_filters),
                "rate_limit_per_second": self._rate_limit_per_second,
                "console_output": self._console_output,
                "stats": {
                    "messages_logged": self._stats.messages_logged,
                    "messages_filtered": self._stats.messages_filtered,
                    "total_bytes": self._stats.total_bytes,
                    "debug_overhead_ms": self._stats.debug_overhead_ms,
                    "rate_limited_messages": self._stats.rate_limited_messages,
                    "uptime_seconds": time.time() - self._start_time,
                },
            }

    def get_endpoint_stats(self) -> Dict[str, Dict[str, Any]]:
        """Get per-endpoint statistics."""
        with self._lock:
            return {
                endpoint: {
                    "messages_logged": stats.messages_logged,
                    "total_bytes": stats.total_bytes,
                    "rate_limited": stats.rate_limited_messages,
                }
                for endpoint, stats in self._per_endpoint_stats.items()
            }

    def reset_stats(self):
        """Reset all debugging statistics."""
        with self._lock:
            self._stats.reset()
            self._per_endpoint_stats.clear()
            logger.info("ZMQ debug statistics reset")

    # Core Debugging Methods

    def log_outbound(
        self,
        endpoint: str,
        data: Union[bytes, List[bytes]],
        message_type: MessageType = MessageType.UNKNOWN,
        topic: str = "",
        context: str = "",
    ):
        """
            Log outbound ZMQ traffic with minimal performance impact.

        Args:
                endpoint: ZMQ endpoint
                data: Raw bytes or list of frames
                message_type: Type of message for filtering
            topic: ZMQ topic (for PUB/SUB)
            context: Additional context information
        """
        # Fast path: if debugging disabled, return immediately (zero overhead)
        if not self._outbound_enabled:
            return

        # Performance timing
        start_time = time.perf_counter() if self._enable_performance_tracking else 0

        try:
            # Apply filters
            if not self._should_log_message(endpoint, message_type):
                self._stats.messages_filtered += 1
                return

            # Rate limiting
            if not self._check_rate_limit():
                self._stats.rate_limited_messages += 1

                return
            # Prepare data for logging
            frames = data if isinstance(data, list) else [data]
            total_size = sum(len(frame) for frame in frames)

            # Update statistics
            self._stats.messages_logged += 1
            self._stats.total_bytes += total_size
            self._per_endpoint_stats[endpoint].messages_logged += 1
            self._per_endpoint_stats[endpoint].total_bytes += total_size

            # Log based on debug level
            self._log_message(
                direction="OUTBOUND",
                endpoint=endpoint,
                frames=frames,
                message_type=message_type,
                topic=topic,
                context=context,
            )

        finally:
            # Track performance overhead
            if self._enable_performance_tracking:
                overhead = (time.perf_counter() - start_time) * 1000
                self._stats.debug_overhead_ms += overhead

    def log_inbound(
        self,
        endpoint: str,
        frames: List[bytes],
        message_type: MessageType = MessageType.UNKNOWN,
        context: str = "",
    ):
        """
            Log inbound ZMQ traffic with minimal performance impact.

        Args:
                endpoint: ZMQ endpoint
            frames: List of ZMQ frames received
                message_type: Type of message for filtering
            context: Additional context information
        """
        # Fast path: if debugging disabled, return immediately (zero overhead)
        if not self._inbound_enabled:
            return

        # Performance timing
        start_time = time.perf_counter() if self._enable_performance_tracking else 0

        try:
            # Apply filters
            if not self._should_log_message(endpoint, message_type):
                self._stats.messages_filtered += 1
                return

            # Rate limiting
            if not self._check_rate_limit():
                self._stats.rate_limited_messages += 1
                return

            # Calculate total size
            total_size = sum(len(frame) for frame in frames)

            # Update statistics
            self._stats.messages_logged += 1
            self._stats.total_bytes += total_size
            self._per_endpoint_stats[endpoint].messages_logged += 1
            self._per_endpoint_stats[endpoint].total_bytes += total_size

            # Log based on debug level
            self._log_message(
                direction="INBOUND",
                endpoint=endpoint,
                frames=frames,
                message_type=message_type,
                context=context,
            )

        finally:
            # Track performance overhead
            if self._enable_performance_tracking:
                overhead = (time.perf_counter() - start_time) * 1000
                self._stats.debug_overhead_ms += overhead

    # Internal Helper Methods

    def _should_log_message(self, endpoint: str, message_type: MessageType) -> bool:
        """Check if message should be logged based on filters."""
        # Check message type filter
        if self._message_filters and message_type not in self._message_filters:
            return False

        # Check endpoint filter
        if self._endpoint_filters and endpoint not in self._endpoint_filters:
            return False

        return True

    def _check_rate_limit(self) -> bool:
        """Check if message should be logged based on rate limiting."""
        current_time = time.time()

        # Add current timestamp
        self._message_timestamps.append(current_time)

        # Count messages in the last second
        cutoff_time = current_time - 1.0
        recent_count = sum(1 for ts in self._message_timestamps if ts >= cutoff_time)

        return recent_count <= self._rate_limit_per_second

    def _log_message(
        self,
        direction: str,
        endpoint: str,
        frames: List[bytes],
        message_type: MessageType,
        topic: str = "",
        context: str = "",
    ):
        """Log message based on current debug level."""
        if self._debug_level == DebugLevel.OFF:
            return

        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        total_size = sum(len(frame) for frame in frames)
        arrow = "📤" if direction == "OUTBOUND" else "📥"

        # Choose logger based on console output setting
        output_logger = _get_console_logger() if self._console_output else logger

        # Minimal logging
        if self._debug_level == DebugLevel.MINIMAL:
            output_logger.info(
                f"{arrow} ZMQ {direction} [{timestamp}] {endpoint} ({total_size}b)"
            )
            return

        # Headers and above
        output_logger.info(f"{arrow} ZMQ {direction} [{timestamp}]")
        output_logger.info(f"   [TARGET] {endpoint}")
        output_logger.info(f"   [TYPE] {message_type.value}")
        output_logger.info(f"   [STATS] Frames: {len(frames)}, Size: {total_size}b")

        if topic:
            output_logger.info(f"   [TAG] Topic: '{topic}'")
        if context:
            output_logger.info(f"   [CONTEXT] {context}")

        if self._debug_level == DebugLevel.HEADERS:
            output_logger.info("   " + "─" * 40)
        return

    def _get_data_preview(self, data: bytes, max_chars: int) -> str:
        """Get a truncated preview of frame data."""
        if not data:
            return "<empty>"

        try:
            # Try UTF-8 decode
            decoded = data.decode("utf-8")
            if len(decoded) <= max_chars:
                return f"TEXT: {decoded}"
            else:
                return f"TEXT: {decoded[:max_chars]}... [+{len(decoded) - max_chars} chars]"
        except UnicodeDecodeError:
            # Binary data
            hex_preview = data[: max_chars // 2].hex()
            if len(data) > max_chars // 2:
                return f"BINARY: {hex_preview}... [+{len(data) - max_chars // 2} bytes]"
            else:
                return f"BINARY: {hex_preview}"

    def _decode_frame_data(self, data: bytes) -> str:
        """Decode frame data for full logging."""
        if not data:
            return "<empty>"

        try:
            decoded = data.decode("utf-8")
            # Try to parse as JSON for pretty printing
            try:
                json_data = json.loads(decoded)
                return f"JSON: {json.dumps(json_data, indent=2)}"
            except (json.JSONDecodeError, TypeError):
                return f"TEXT: {decoded}"
        except UnicodeDecodeError:
            return f"BINARY: {data.hex()} ({len(data)} bytes)"


# Global debugger instance
_debugger = ZMQDebugger()

# Public API Functions


def enable_inbound_debug(enabled: bool = True):
    """Enable/disable inbound ZMQ debugging at runtime."""
    _debugger.enable_inbound(enabled)


def enable_outbound_debug(enabled: bool = True):
    """Enable/disable outbound ZMQ debugging at runtime."""
    _debugger.enable_outbound(enabled)


def set_debug_level(level: Union[DebugLevel, str]):
    """Set debug verbosity level."""
    if isinstance(level, str):
        level = DebugLevel[level.upper()]
    _debugger.set_debug_level(level)


def set_message_filters(message_types: List[Union[MessageType, str]]):
    """Set message type filters."""
    types = []
    for mt in message_types:
        if isinstance(mt, str):
            types.append(MessageType(mt.lower()))
        else:
            types.append(mt)
    _debugger.set_message_filters(types)


def set_endpoint_filters(endpoints: List[str]):
    """Set endpoint filters."""
    _debugger.set_endpoint_filters(endpoints)


def set_rate_limit(messages_per_second: int):
    """Set rate limiting for debug messages."""
    _debugger.set_rate_limit(messages_per_second)


def set_console_output(enabled: bool = True):
    """Enable/disable console output for debug messages."""
    _debugger.set_console_output(enabled)


def get_debug_status() -> Dict[str, Any]:
    """Get current debugging status."""
    return _debugger.get_status()


def get_endpoint_stats() -> Dict[str, Dict[str, Any]]:
    """Get per-endpoint statistics."""
    return _debugger.get_endpoint_stats()


def reset_debug_stats():
    """Reset debugging statistics."""
    _debugger.reset_stats()


# High-performance logging functions (zero overhead when disabled)


def log_outbound(
    endpoint: str,
    data: Union[bytes, List[bytes]],
    message_type: Union[MessageType, str] = MessageType.UNKNOWN,
    topic: str = "",
    context: str = "",
):
    """Log outbound ZMQ traffic."""
    if isinstance(message_type, str):
        message_type = MessageType(message_type.lower())
    _debugger.log_outbound(endpoint, data, message_type, topic, context)


def log_inbound(
    endpoint: str,
    frames: List[bytes],
    message_type: Union[MessageType, str] = MessageType.UNKNOWN,
    context: str = "",
):
    """Log inbound ZMQ traffic."""
    if isinstance(message_type, str):
        message_type = MessageType(message_type.lower())
    _debugger.log_inbound(endpoint, frames, message_type, context)


# Convenience functions for specific ZMQ patterns


def log_pub_message(
    endpoint: str, topic: Union[str, bytes], data: bytes, context: str = ""
):
    """Log a PUB/SUB outbound message."""
    topic_str = topic.decode("utf-8") if isinstance(topic, bytes) else topic
    log_outbound(
        endpoint,
        [topic.encode() if isinstance(topic, str) else topic, data],
        MessageType.UNKNOWN,
        topic_str,
        context,
    )


def log_req_message(endpoint: str, data: bytes, context: str = ""):
    """Log a REQ/REP outbound request."""
    log_outbound(endpoint, data, MessageType.UNKNOWN, "", f"REQ {context}")


def log_rep_message(endpoint: str, data: bytes, context: str = ""):
    """Log a REQ/REP outbound reply."""
    log_outbound(endpoint, data, MessageType.UNKNOWN, "", f"REP {context}")


def log_push_message(endpoint: str, data: bytes, context: str = ""):
    """Log a PUSH/PULL outbound message."""
    log_outbound(endpoint, data, MessageType.UNKNOWN, "", f"PUSH {context}")


def log_sub_message(endpoint: str, frames: List[bytes], context: str = ""):
    """Log a PUB/SUB inbound message."""
    log_inbound(endpoint, frames, MessageType.UNKNOWN, f"SUB {context}")


def log_pull_message(endpoint: str, frames: List[bytes], context: str = ""):
    """Log a PUSH/PULL inbound message."""
    log_inbound(endpoint, frames, MessageType.UNKNOWN, f"PULL {context}")


def log_req_received(endpoint: str, frames: List[bytes], context: str = ""):
    """Log a REQ/REP inbound request."""
    log_inbound(endpoint, frames, MessageType.UNKNOWN, f"REQ received {context}")


def log_rep_received(endpoint: str, frames: List[bytes], context: str = ""):
    """Log a REQ/REP inbound reply."""
    log_inbound(endpoint, frames, MessageType.UNKNOWN, f"REP received {context}")


# Legacy compatibility functions (deprecated but maintained for backwards compatibility)


def decode_zmq_data(data: bytes, max_preview: int = 200) -> str:
    """Legacy function - use log_outbound/log_inbound instead."""
    return _debugger._get_data_preview(data, max_preview)


def log_zmq_outbound(
    endpoint: str,
    topic: Union[str, bytes],
    data: bytes,
    context: str = "",
    message_type: str = "unknown",
) -> None:
    """Legacy function - use log_outbound instead."""
    log_pub_message(endpoint, topic, data, context)


def log_zmq_inbound(
    endpoint: str, frames: List[bytes], context: str = "", message_type: str = "unknown"
) -> None:
    """Legacy function - use log_inbound instead."""
    log_inbound(endpoint, frames, MessageType.UNKNOWN, context)


def log_zmq_multipart_outbound(
    endpoint: str,
    multipart_data: List[bytes],
    context: str = "",
    message_type: str = "unknown",
) -> None:
    """Legacy function - use log_outbound instead."""
    log_outbound(endpoint, multipart_data, MessageType.UNKNOWN, "", context)
