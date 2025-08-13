"""
Platform-specific optimizations for ZMQ sockets handling neural data.

This module provides platform-aware socket configuration to maximize
performance for neural data transmission.
"""

import sys
from typing import Any, Dict

import zmq

__all__ = [
    "PlatformOptimizer",
    "get_platform_optimizer",
    "optimize_socket_for_neural_data",
]


class PlatformOptimizer:
    """Base class for platform-specific optimizations."""

    def __init__(self):
        self.platform = sys.platform
        self.zmq_version = zmq.zmq_version()
        self.pyzmq_version = zmq.pyzmq_version()

    def optimize_for_neural_data(
        self, socket: zmq.Socket, socket_type: int, is_sender: bool = False
    ) -> None:
        """
        Optimize socket for neural data transmission.

        Args:
            socket: ZMQ socket to optimize
            socket_type: ZMQ socket type (PUB, SUB, PUSH, PULL, etc.)
            is_sender: Whether this socket sends data
        """
        # Common optimizations for all platforms
        self._apply_common_optimizations(socket, socket_type, is_sender)

        # Platform-specific optimizations
        self._apply_platform_optimizations(socket, socket_type, is_sender)

    def _apply_common_optimizations(
        self, socket: zmq.Socket, socket_type: int, is_sender: bool
    ) -> None:
        """Apply optimizations common to all platforms."""
        # Set high water marks to prevent memory bloat
        if is_sender:
            socket.setsockopt(zmq.SNDHWM, 1000)  # 1000 messages max
            socket.setsockopt(zmq.SNDBUF, 8388608)  # 8MB send buffer
        else:
            socket.setsockopt(zmq.RCVHWM, 1000)  # 1000 messages max
            socket.setsockopt(zmq.RCVBUF, 8388608)  # 8MB receive buffer

        # Set linger to avoid hanging on shutdown
        socket.setsockopt(zmq.LINGER, 1000)  # 1 second linger

        # Enable TCP keepalive for reliability
        socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
        socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 120)  # 2 minutes
        socket.setsockopt(zmq.TCP_KEEPALIVE_CNT, 3)
        socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 10)

        # Immediate mode for low latency
        if socket_type in [zmq.PUB, zmq.PUSH]:
            socket.setsockopt(zmq.IMMEDIATE, 1)

    def _apply_platform_optimizations(
        self, socket: zmq.Socket, socket_type: int, is_sender: bool
    ) -> None:
        """Override in platform-specific subclasses."""
        pass

    def get_optimal_io_threads(self) -> int:
        """Get optimal number of I/O threads for this platform."""
        import multiprocessing

        cpu_count = multiprocessing.cpu_count()

        # Use 1 I/O thread per 4 CPUs, min 1, max 4
        return max(1, min(4, cpu_count // 4))

    def get_socket_monitor_events(self) -> int:
        """Get events to monitor for socket diagnostics."""
        return (
            zmq.EVENT_CONNECTED
            | zmq.EVENT_DISCONNECTED
            | zmq.EVENT_CONNECT_RETRIED
            | zmq.EVENT_BIND_FAILED
        )


class LinuxOptimizer(PlatformOptimizer):
    """Linux-specific optimizations."""

    def _apply_platform_optimizations(
        self, socket: zmq.Socket, socket_type: int, is_sender: bool
    ) -> None:
        """Apply Linux-specific optimizations."""
        try:
            # Enable zero-copy on Linux (requires ZMQ 4.2+)
            if is_sender and hasattr(zmq, "ZEROCOPY"):
                socket.setsockopt(zmq.ZEROCOPY, 1)

            # Set socket priority for neural data
            import socket as pysocket

            if hasattr(pysocket, "SO_PRIORITY"):
                # Higher priority for real-time neural data
                socket.setsockopt(pysocket.SOL_SOCKET, pysocket.SO_PRIORITY, 6)

            # Enable SO_REUSEPORT for load balancing
            if hasattr(pysocket, "SO_REUSEPORT"):
                socket.setsockopt(
                    pysocket.SOL_SOCKET, pysocket.SO_REUSEPORT, 1
                )

            # CPU affinity hints (requires recent kernels)
            if hasattr(pysocket, "SO_INCOMING_CPU"):
                # Hint to process on specific CPU
                socket.setsockopt(
                    pysocket.SOL_SOCKET,
                    pysocket.SO_INCOMING_CPU,
                    self._get_neural_processor_cpu(),
                )

            # Optimize TCP settings for low latency
            if socket_type in [zmq.DEALER, zmq.ROUTER, zmq.REQ, zmq.REP]:
                socket.setsockopt(zmq.TCP_NODELAY, 1)  # Disable Nagle

        except (AttributeError, zmq.ZMQError):
            # Optimizations not available on this system
            pass

    def _get_neural_processor_cpu(self) -> int:
        """Get CPU best suited for neural processing."""
        # In production, this would check NUMA topology
        # For now, use CPU 0
        return 0


class DarwinOptimizer(PlatformOptimizer):
    """macOS-specific optimizations."""

    def _apply_platform_optimizations(
        self, socket: zmq.Socket, socket_type: int, is_sender: bool
    ) -> None:
        """Apply macOS-specific optimizations."""
        try:
            import socket as pysocket

            # Prevent SIGPIPE on macOS
            if hasattr(pysocket, "SO_NOSIGPIPE"):
                socket.setsockopt(
                    pysocket.SOL_SOCKET, pysocket.SO_NOSIGPIPE, 1
                )

            # macOS-specific buffer tuning
            # macOS has different buffer scaling behavior
            if is_sender:
                socket.setsockopt(zmq.SNDBUF, 4194304)  # 4MB on macOS
            else:
                socket.setsockopt(zmq.RCVBUF, 4194304)  # 4MB on macOS

        except (AttributeError, zmq.ZMQError):
            pass


class WindowsOptimizer(PlatformOptimizer):
    """Windows-specific optimizations."""

    def _apply_platform_optimizations(
        self, socket: zmq.Socket, socket_type: int, is_sender: bool
    ) -> None:
        """Apply Windows-specific optimizations."""
        try:
            # Windows-specific socket options

            # Set socket to non-blocking mode explicitly
            if hasattr(socket, "FD"):
                # Get underlying socket FD
                fd = socket.getsockopt(zmq.FD)
                if fd > 0:
                    # Windows socket tuning would go here
                    pass

            # Increase socket buffer sizes on Windows
            # Windows has different buffer semantics
            if is_sender:
                socket.setsockopt(zmq.SNDBUF, 16777216)  # 16MB on Windows
            else:
                socket.setsockopt(zmq.RCVBUF, 16777216)  # 16MB on Windows

        except (AttributeError, zmq.ZMQError):
            pass

    def get_optimal_io_threads(self) -> int:
        """Windows benefits from more I/O threads."""
        import multiprocessing

        cpu_count = multiprocessing.cpu_count()

        # Windows: 1 thread per 2 CPUs, min 2, max 8
        return max(2, min(8, cpu_count // 2))


# Platform optimizer registry
_PLATFORM_OPTIMIZERS = {
    "linux": LinuxOptimizer,
    "linux2": LinuxOptimizer,
    "darwin": DarwinOptimizer,
    "win32": WindowsOptimizer,
    "cygwin": WindowsOptimizer,
}


def get_platform_optimizer() -> PlatformOptimizer:
    """Get optimizer for current platform."""
    platform = sys.platform
    optimizer_class = _PLATFORM_OPTIMIZERS.get(platform, PlatformOptimizer)
    return optimizer_class()


def optimize_socket_for_neural_data(
    socket: zmq.Socket, socket_type: int, is_sender: bool = False
) -> None:
    """
    Optimize a ZMQ socket for neural data transmission.

    Args:
        socket: ZMQ socket to optimize
        socket_type: ZMQ socket type
        is_sender: Whether this socket sends data
    """
    optimizer = get_platform_optimizer()
    optimizer.optimize_for_neural_data(socket, socket_type, is_sender)


def get_platform_info() -> Dict[str, Any]:
    """Get platform information for diagnostics."""
    optimizer = get_platform_optimizer()

    return {
        "platform": sys.platform,
        "zmq_version": optimizer.zmq_version,
        "pyzmq_version": optimizer.pyzmq_version,
        "optimal_io_threads": optimizer.get_optimal_io_threads(),
        "optimizer_class": optimizer.__class__.__name__,
    }


# Example usage
if __name__ == "__main__":
    import zmq

    # Get platform info
    info = get_platform_info()
    print(f"Platform info: {info}")

    # Create and optimize a socket
    context = zmq.Context()
    socket = context.socket(zmq.PUB)

    # Apply optimizations
    optimize_socket_for_neural_data(socket, zmq.PUB, is_sender=True)

    print("Socket optimized for neural data transmission")
