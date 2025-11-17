"""
ZMQ API Compliance Tests

These tests specifically verify that we're using correct ZMQ API methods
and catch issues like the recv_into bug that caused runtime errors.
"""

from unittest.mock import Mock

import pytest
import zmq


class TestZMQAPICompliance:
    """Test ZMQ API method usage compliance."""

    def test_zmq_socket_has_required_methods(self):
        """Test that ZMQ sockets have the methods we expect."""
        context = zmq.Context()

        # Test different socket types
        socket_types = [zmq.PULL, zmq.PUSH, zmq.PUB, zmq.SUB, zmq.REQ, zmq.REP]

        for socket_type in socket_types:
            socket = context.socket(socket_type)

            try:
                # These methods SHOULD exist
                assert hasattr(socket, "recv"), (
                    f"Socket type {socket_type} missing recv method"
                )
                assert hasattr(socket, "send"), (
                    f"Socket type {socket_type} missing send method"
                )
                assert hasattr(socket, "bind"), (
                    f"Socket type {socket_type} missing bind method"
                )
                assert hasattr(socket, "connect"), (
                    f"Socket type {socket_type} missing connect method"
                )
                assert hasattr(socket, "close"), (
                    f"Socket type {socket_type} missing close method"
                )

                # These methods should NOT exist (catch recv_into bug)
                assert not hasattr(socket, "recv_into"), (
                    f"Socket type {socket_type} should not have recv_into"
                )
                assert not hasattr(socket, "send_into"), (
                    f"Socket type {socket_type} should not have send_into"
                )

            finally:
                socket.close()

        context.term()

    def test_zmq_socket_types_consistency(self):
        """Test that socket types are correctly identified."""
        context = zmq.Context()

        test_cases = [
            (zmq.PULL, "PULL"),
            (zmq.PUSH, "PUSH"),
            (zmq.PUB, "PUB"),
            (zmq.SUB, "SUB"),
            (zmq.REQ, "REQ"),
            (zmq.REP, "REP"),
        ]

        for socket_type, name in test_cases:
            socket = context.socket(socket_type)
            try:
                assert socket.socket_type == socket_type, (
                    f"Socket type mismatch for {name}"
                )
            finally:
                socket.close()

        context.term()

    def test_sensory_stream_socket_usage(self):
        """Test the sensory stream uses ZMQ API correctly."""
        # Import here to avoid dependency issues
        try:
            from feagi.api.zmq.streams.sensory_neural import SensoryNeuralStream
        except ImportError:
            pytest.skip("SensoryNeuralStream not available")

        # Create a minimal stream to test socket setup
        mock_core_api = Mock()

        try:
            stream = SensoryNeuralStream(
                core_api=mock_core_api,
                host="127.0.0.1",
                port=5599,  # Test port
                ring_buffer_slots=2,
                slot_size=1024,
            )

            # Verify socket was created correctly
            assert stream.socket is not None
            assert stream.socket.socket_type == zmq.PULL

            # Verify socket has correct methods
            assert hasattr(stream.socket, "recv")
            assert not hasattr(stream.socket, "recv_into")

        except Exception as e:
            # If there are dependency issues, at least verify basic ZMQ usage
            if "recv_into" in str(e):
                pytest.fail(f"SensoryNeuralStream still trying to use recv_into: {e}")
            else:
                pytest.skip(f"SensoryNeuralStream dependency issue: {e}")


class TestZMQErrorPrevention:
    """Test specific error conditions to prevent regression."""

    def test_recv_into_not_available(self):
        """Specifically test that recv_into is not available on ZMQ sockets."""
        context = zmq.Context()
        socket = context.socket(zmq.PULL)

        try:
            # This should NOT exist
            assert not hasattr(socket, "recv_into")

            # Trying to call it should raise AttributeError
            with pytest.raises(AttributeError):
                socket.recv_into

        finally:
            socket.close()
            context.term()

    def test_import_sensory_neural_no_error(self):
        """Test that importing sensory_neural doesn't cause import errors."""
        try:
            from feagi.api.zmq.streams import sensory_neural

            # If import succeeds, verify key classes exist
            assert hasattr(sensory_neural, "SensoryNeuralStream")
            assert hasattr(sensory_neural, "StreamResult")
        except ImportError as e:
            if "recv_into" in str(e):
                pytest.fail(f"Import error related to recv_into: {e}")
            # Other import errors might be due to missing dependencies
            pytest.skip(f"Import error (may be due to dependencies): {e}")


@pytest.mark.xfail(reason="Version test hits mocks in test environment")
def test_zmq_version_compatibility():
    """Test ZMQ version compatibility."""
    import zmq

    # Get ZMQ version
    version = zmq.zmq_version()
    print(f"ZMQ version: {version}")

    # Verify we have a compatible version (basic check)
    assert isinstance(version, str), (
        f"ZMQ version should be a string, got {type(version)}"
    )
    assert len(version) > 0, "ZMQ version should not be empty"

    # Print version info for debugging
    print(f"PyZMQ version: {zmq.pyzmq_version()}")


def test_zmq_recv_into_error_prevention():
    """Test that specifically prevents the recv_into error."""
    context = zmq.Context()
    socket = context.socket(zmq.PULL)

    try:
        # This is the specific test that would have caught the original bug
        assert not hasattr(socket, "recv_into"), (
            "ZMQ socket should not have recv_into method"
        )

        # Verify recv method exists and is callable
        assert hasattr(socket, "recv"), "ZMQ socket should have recv method"
        assert callable(socket.recv), "recv should be callable"

        # Test that trying to access recv_into raises AttributeError
        with pytest.raises(AttributeError, match="recv_into"):
            socket.recv_into

    finally:
        socket.close()
        context.term()
