"""A transport whose optional dependency is missing fails clearly at use time."""

import pytest

from feagi.transports import _missing_transport


def test_missing_transport_names_the_extra_and_the_module():
    cls = _missing_transport(
        "WebSocketClientTransport", "bluetooth", ImportError("x", name="websockets")
    )
    with pytest.raises(ImportError) as exc:
        cls("ws://127.0.0.1:1")
    assert "WebSocketClientTransport needs feagi-core[bluetooth]" in str(exc.value)
    assert "missing: websockets" in str(exc.value)
