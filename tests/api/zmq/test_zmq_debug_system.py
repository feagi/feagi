"""
Tests for the Enhanced ZMQ Debugging System

These tests verify the high-performance debugging features including:
- Zero-overhead operation when disabled
- Runtime configuration changes
- Filtering and rate limiting
- Performance monitoring
- REST API control endpoints
"""

import threading
import time
from unittest.mock import Mock, patch

import pytest

from feagi.utils.zmq_debug import (
    DebugLevel,
    MessageType,
    ZMQDebugger,
    enable_inbound_debug,
    enable_outbound_debug,
    get_debug_status,
    get_endpoint_stats,
    log_inbound,
    log_outbound,
    reset_debug_stats,
    set_debug_level,
    set_endpoint_filters,
    set_message_filters,
    set_rate_limit,
)


@pytest.fixture(autouse=True)
def reset_global_debugger():
    """Reset global debugger state before each test."""
    # Reset all global state
    reset_debug_stats()
    enable_inbound_debug(False)
    enable_outbound_debug(False)
    set_debug_level(DebugLevel.SUMMARY)
    set_message_filters([])
    set_endpoint_filters([])
    set_rate_limit(100)
    yield
    # Clean up after test
    reset_debug_stats()


class TestZMQDebuggerCore:
    """Test the core ZMQDebugger functionality."""

    def test_debugger_initialization(self):
        """Test that debugger initializes with correct default values."""
        debugger = ZMQDebugger()

        assert not debugger._inbound_enabled
        assert not debugger._outbound_enabled
        assert debugger._debug_level == DebugLevel.SUMMARY
        assert len(debugger._message_filters) == 0  # No filters by default
        assert len(debugger._endpoint_filters) == 0
        assert debugger._rate_limit_per_second == 100

    def test_runtime_configuration(self):
        """Test runtime configuration changes."""
        debugger = ZMQDebugger()

        # Test enabling debugging
        debugger.enable_inbound(True)
        debugger.enable_outbound(True)

        assert debugger._inbound_enabled
        assert debugger._outbound_enabled

        # Test debug level changes
        debugger.set_debug_level(DebugLevel.MINIMAL)
        assert debugger._debug_level == DebugLevel.MINIMAL

        # Test filter configuration
        filters = [MessageType.SENSORY, MessageType.MOTOR]
        debugger.set_message_filters(filters)
        assert debugger._message_filters == set(filters)

        # Test endpoint filters
        endpoints = ["tcp://localhost:5562", "tcp://*:5564"]
        debugger.set_endpoint_filters(endpoints)
        assert debugger._endpoint_filters == set(endpoints)

        # Test rate limiting
        debugger.set_rate_limit(50)
        assert debugger._rate_limit_per_second == 50

    def test_message_filtering(self):
        """Test message type filtering logic."""
        debugger = ZMQDebugger()

        # No filters = allow all
        assert debugger._should_log_message("tcp://test:5562", MessageType.SENSORY)
        assert debugger._should_log_message("tcp://test:5562", MessageType.MOTOR)

        # With filters = only allow specified types
        debugger.set_message_filters([MessageType.SENSORY])
        assert debugger._should_log_message("tcp://test:5562", MessageType.SENSORY)
        assert not debugger._should_log_message("tcp://test:5562", MessageType.MOTOR)

    def test_endpoint_filtering(self):
        """Test endpoint filtering logic."""
        debugger = ZMQDebugger()

        # No filters = allow all
        assert debugger._should_log_message("tcp://test:5562", MessageType.SENSORY)
        assert debugger._should_log_message("tcp://other:5564", MessageType.SENSORY)

        # With filters = only allow specified endpoints
        debugger.set_endpoint_filters(["tcp://test:5562"])
        assert debugger._should_log_message("tcp://test:5562", MessageType.SENSORY)
        assert not debugger._should_log_message("tcp://other:5564", MessageType.SENSORY)

    def test_rate_limiting(self):
        """Test rate limiting functionality."""
        debugger = ZMQDebugger()
        debugger.set_rate_limit(2)  # Very low limit for testing

        # Clear any existing timestamps
        debugger._message_timestamps.clear()

        # First few messages should pass
        assert debugger._check_rate_limit()
        assert debugger._check_rate_limit()

        # After limit, should be blocked
        assert not debugger._check_rate_limit()

        # After time passes, should be allowed again
        time.sleep(1.1)  # Wait for rate limit window to reset
        assert debugger._check_rate_limit()

    def test_performance_tracking(self):
        """Test performance overhead tracking."""
        debugger = ZMQDebugger()
        debugger.enable_outbound(True)
        debugger.reset_stats()

        initial_overhead = debugger._stats.debug_overhead_ms

        # Simulate some debug operations
        with patch("feagi.utils.zmq_debug.logger"):
            debugger.log_outbound("tcp://test:5562", b"test data", MessageType.SENSORY)
            debugger.log_outbound("tcp://test:5562", b"test data", MessageType.SENSORY)

        # Should have tracked some overhead
        assert debugger._stats.debug_overhead_ms > initial_overhead
        assert debugger._stats.messages_logged == 2

    def test_statistics_reset(self):
        """Test statistics reset functionality."""
        debugger = ZMQDebugger()
        debugger.enable_outbound(True)

        # Generate some statistics
        with patch("feagi.utils.zmq_debug.logger"):
            debugger.log_outbound("tcp://test:5562", b"test data", MessageType.SENSORY)

        assert debugger._stats.messages_logged > 0

        # Reset and verify
        debugger.reset_stats()
        assert debugger._stats.messages_logged == 0
        assert debugger._stats.debug_overhead_ms == 0.0


class TestZMQDebuggerPerformance:
    """Test performance characteristics of the debugging system."""

    def test_zero_overhead_when_disabled(self):
        """Test that there's truly zero overhead when debugging is disabled."""
        debugger = ZMQDebugger()
        # Ensure debugging is disabled
        debugger.enable_inbound(False)
        debugger.enable_outbound(False)

        start_time = time.perf_counter()

        # Make many calls with debugging disabled
        for i in range(1000):
            debugger.log_outbound("tcp://test:5562", b"test data", MessageType.SENSORY)
            debugger.log_inbound("tcp://test:5562", [b"test data"], MessageType.SENSORY)

        elapsed = time.perf_counter() - start_time

        # Should be extremely fast (less than 10ms for 1000 calls on slower systems)
        assert elapsed < 0.01, f"Disabled debugging took too long: {elapsed:.3f}s"

        # No statistics should be updated
        assert debugger._stats.messages_logged == 0
        assert debugger._stats.debug_overhead_ms == 0.0

    def test_minimal_overhead_when_enabled(self):
        """Test that overhead is minimal when debugging is enabled."""
        debugger = ZMQDebugger()
        debugger.enable_outbound(True)
        debugger.set_debug_level(DebugLevel.MINIMAL)  # Least verbose level
        debugger.set_rate_limit(1000)  # High limit to avoid rate limiting

        start_time = time.perf_counter()

        with patch("feagi.utils.zmq_debug.logger"):
            # Make moderate number of calls with debugging enabled
            for i in range(100):
                debugger.log_outbound(
                    "tcp://test:5562", b"test data", MessageType.SENSORY
                )

        elapsed = time.perf_counter() - start_time

        # Should still be reasonably fast (less than 1s for 100 calls)
        assert elapsed < 1.0, f"Enabled debugging took too long: {elapsed:.3f}s"

        # Statistics should be updated
        assert debugger._stats.messages_logged == 100
        assert debugger._stats.debug_overhead_ms > 0

    def test_thread_safety(self):
        """Test that the debugger is thread-safe."""
        debugger = ZMQDebugger()
        debugger.enable_outbound(True)
        debugger.set_rate_limit(
            1000
        )  # High limit to avoid rate limiting during threading
        debugger.reset_stats()

        results = []

        def debug_worker():
            with patch("feagi.utils.zmq_debug.logger"):
                for i in range(25):  # Reduced to avoid rate limiting
                    debugger.log_outbound(
                        "tcp://test:5562", b"test data", MessageType.SENSORY
                    )
            results.append(True)

        # Start multiple threads
        threads = [threading.Thread(target=debug_worker) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # All threads should complete successfully
        assert len(results) == 4
        assert all(results)

        # Should have logged most messages (some may be rate limited)
        assert debugger._stats.messages_logged >= 80  # Allow for some rate limiting


class TestGlobalDebugAPI:
    """Test the global debugging API functions."""

    def test_global_enable_disable(self):
        """Test global enable/disable functions."""
        # Start with disabled state
        enable_inbound_debug(False)
        enable_outbound_debug(False)

        status = get_debug_status()
        assert not status["inbound_enabled"]
        assert not status["outbound_enabled"]

        # Enable debugging
        enable_inbound_debug(True)
        enable_outbound_debug(True)

        status = get_debug_status()
        assert status["inbound_enabled"]
        assert status["outbound_enabled"]

    def test_global_configuration(self):
        """Test global configuration functions."""
        # Set debug level
        set_debug_level(DebugLevel.HEADERS)
        status = get_debug_status()
        assert status["debug_level"] == "HEADERS"

        # Set message filters
        set_message_filters([MessageType.MOTOR, MessageType.VISUALIZATION])
        status = get_debug_status()
        assert set(status["message_filters"]) == {"motor", "visualization"}

        # Set endpoint filters
        set_endpoint_filters(["tcp://localhost:5562"])
        status = get_debug_status()
        assert status["endpoint_filters"] == ["tcp://localhost:5562"]

        # Set rate limit
        set_rate_limit(200)
        status = get_debug_status()
        assert status["rate_limit_per_second"] == 200

    def test_global_logging_functions(self):
        """Test global logging functions."""
        enable_outbound_debug(True)
        enable_inbound_debug(True)
        set_rate_limit(1000)  # High limit to avoid rate limiting
        reset_debug_stats()

        with patch("feagi.utils.zmq_debug.logger"):
            # Test outbound logging
            log_outbound("tcp://test:5562", b"test data", MessageType.SENSORY)
            log_outbound("tcp://test:5562", [b"topic", b"data"], MessageType.MOTOR)

            # Test inbound logging
            log_inbound("tcp://test:5563", [b"frame1", b"frame2"], MessageType.CONTROL)

        status = get_debug_status()
        assert status["stats"]["messages_logged"] == 3

        # Test endpoint stats
        endpoint_stats = get_endpoint_stats()
        assert "tcp://test:5562" in endpoint_stats
        assert "tcp://test:5563" in endpoint_stats
        assert endpoint_stats["tcp://test:5562"]["messages_logged"] == 2
        assert endpoint_stats["tcp://test:5563"]["messages_logged"] == 1

    def test_statistics_reset(self):
        """Test global statistics reset."""
        enable_outbound_debug(True)
        set_rate_limit(1000)  # High limit to avoid rate limiting

        with patch("feagi.utils.zmq_debug.logger"):
            log_outbound("tcp://test:5562", b"test data", MessageType.SENSORY)

        status = get_debug_status()
        assert status["stats"]["messages_logged"] > 0

        reset_debug_stats()

        status = get_debug_status()
        assert status["stats"]["messages_logged"] == 0

        endpoint_stats = get_endpoint_stats()
        assert len(endpoint_stats) == 0


class TestDebugLevels:
    """Test different debug verbosity levels."""

    def test_off_level(self):
        """Test that OFF level produces no output."""
        debugger = ZMQDebugger()
        debugger.enable_outbound(True)
        debugger.set_debug_level(DebugLevel.OFF)

        with patch("feagi.utils.zmq_debug.logger") as mock_logger:
            debugger.log_outbound("tcp://test:5562", b"test data", MessageType.SENSORY)

            # Should not call logger at all
            mock_logger.info.assert_not_called()

    def test_minimal_level(self):
        """Test that MINIMAL level produces basic output."""
        debugger = ZMQDebugger()
        debugger.enable_outbound(True)
        debugger.set_debug_level(DebugLevel.MINIMAL)

        with patch("feagi.utils.zmq_debug.logger") as mock_logger:
            debugger.log_outbound("tcp://test:5562", b"test data", MessageType.SENSORY)

            # Should call logger once for minimal output
            assert mock_logger.info.call_count == 1
            call_args = mock_logger.info.call_args[0][0]
            assert "OUTBOUND" in call_args
            assert "tcp://test:5562" in call_args

    def test_summary_level(self):
        """Test that SUMMARY level includes data previews."""
        debugger = ZMQDebugger()
        debugger.enable_outbound(True)
        debugger.set_debug_level(DebugLevel.SUMMARY)

        with patch("feagi.utils.zmq_debug.logger") as mock_logger:
            debugger.log_outbound("tcp://test:5562", b"test data", MessageType.SENSORY)

            # Should call logger multiple times for headers + data
            assert mock_logger.info.call_count > 3

            # Should include data preview
            all_calls = [call[0][0] for call in mock_logger.info.call_args_list]
            data_call = next((call for call in all_calls if "Frame 0:" in call), None)
            assert data_call is not None
            assert "TEXT: test data" in data_call


class TestDebugIntegrationWithStreams:
    """Test integration with actual ZMQ streams."""

    def test_sensory_stream_debug_integration(self):
        """Test that sensory stream debug hooks work."""
        enable_inbound_debug(True)
        set_rate_limit(1000)  # High limit to avoid rate limiting
        reset_debug_stats()

        # Import and test the actual function used in streams
        from feagi.utils.zmq_debug import log_inbound

        with patch("feagi.utils.zmq_debug.logger"):
            log_inbound(
                endpoint="tcp://*:5558",
                frames=[b"neural_data_frame"],
                message_type=MessageType.SENSORY,
                context="test_neural_data",
            )

        status = get_debug_status()
        assert status["stats"]["messages_logged"] == 1

        endpoint_stats = get_endpoint_stats()
        assert "tcp://*:5558" in endpoint_stats

    def test_visualization_stream_debug_integration(self):
        """Test that visualization stream debug hooks work."""
        enable_outbound_debug(True)
        set_rate_limit(1000)  # High limit to avoid rate limiting
        reset_debug_stats()

        # Import and test the actual function used in streams
        from feagi.utils.zmq_debug import log_outbound

        with patch("feagi.utils.zmq_debug.logger"):
            log_outbound(
                endpoint="tcp://*:5562",
                data=[b"activity", b"visualization_data"],
                message_type=MessageType.VISUALIZATION,
                topic="activity",
                context="test_vis_data",
            )

        status = get_debug_status()
        assert status["stats"]["messages_logged"] == 1

        endpoint_stats = get_endpoint_stats()
        assert "tcp://*:5562" in endpoint_stats

    def test_motor_stream_debug_integration(self):
        """Test that motor stream debug hooks work."""
        enable_outbound_debug(True)
        set_rate_limit(1000)  # High limit to avoid rate limiting
        reset_debug_stats()

        # Import and test the actual function used in streams
        from feagi.utils.zmq_debug import log_outbound

        with patch("feagi.utils.zmq_debug.logger"):
            log_outbound(
                endpoint="tcp://*:5564",
                data=[b"motor_channel", b"motor_command_data"],
                message_type=MessageType.MOTOR,
                topic="motor_channel",
                context="test_motor_cmd",
            )

        status = get_debug_status()
        assert status["stats"]["messages_logged"] == 1

        endpoint_stats = get_endpoint_stats()
        assert "tcp://*:5564" in endpoint_stats


class TestDebugRegressionPrevention:
    """Tests specifically designed to prevent regression of debugging features."""

    def test_environment_variable_initialization(self):
        """Test that environment variables are correctly read at startup."""
        # Mock environment variables
        with patch.dict(
            "os.environ",
            {
                "FEAGI_DEBUG_ZMQ_INBOUND": "1",
                "FEAGI_DEBUG_ZMQ_OUTBOUND": "1",
                "FEAGI_DEBUG_ZMQ_LEVEL": "headers",
            },
        ):
            debugger = ZMQDebugger()

            assert debugger._inbound_enabled
            assert debugger._outbound_enabled
            assert debugger._debug_level == DebugLevel.HEADERS

    def test_legacy_compatibility(self):
        """Test that legacy debug function still work."""
        enable_outbound_debug(True)
        enable_inbound_debug(True)  # Enable inbound as well for legacy test
        set_rate_limit(1000)  # High limit to avoid rate limiting
        reset_debug_stats()

        # Test legacy functions
        from feagi.utils.zmq_debug import (
            decode_zmq_data,
            log_zmq_inbound,
            log_zmq_outbound,
        )

        with patch("feagi.utils.zmq_debug.logger"):
            # These should not crash and should still log
            log_zmq_outbound("tcp://test:5562", b"topic", b"data", "test context")
            log_zmq_inbound("tcp://test:5563", [b"frame1", b"frame2"], "test context")

            # decode_zmq_data should still work
            result = decode_zmq_data(b"test data")
            assert "TEXT: test data" in result

        # Should have logged messages
        status = get_debug_status()
        assert status["stats"]["messages_logged"] >= 2

    def test_no_performance_regression(self):
        """Test that there's no performance regression in the fast path."""
        debugger = ZMQDebugger()
        # Ensure debugging is disabled for fast path test
        debugger.enable_inbound(False)
        debugger.enable_outbound(False)

        import timeit

        # Time the fast path (disabled debugging)
        def fast_path_test():
            debugger.log_outbound("tcp://test:5562", b"data", MessageType.SENSORY)

        # Should be extremely fast (less than 10 microseconds per call on modern systems)
        time_per_call = timeit.timeit(fast_path_test, number=10000) / 10000
        assert time_per_call < 0.00001, (
            f"Fast path too slow: {time_per_call:.6f}s per call"
        )

    def test_memory_efficiency(self):
        """Test that debugging doesn't cause memory leaks or excessive usage."""
        debugger = ZMQDebugger()
        debugger.enable_outbound(True)
        debugger.set_rate_limit(10000)  # High limit to test memory usage

        with patch("feagi.utils.zmq_debug.logger"):
            # Generate many debug messages
            for i in range(1000):
                debugger.log_outbound(
                    f"tcp://test:{5562 + i % 10}", b"data", MessageType.SENSORY
                )

        # Check that timestamp deque doesn't grow unbounded
        assert len(debugger._message_timestamps) <= 1000

        # Check that per-endpoint stats don't accumulate too many entries
        assert len(debugger._per_endpoint_stats) <= 10
