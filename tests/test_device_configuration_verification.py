"""Tests for deterministic device configuration verification."""

import pytest

from feagi.pns.client import FeagiAgentClient


class _DummyRustClient:
    """Minimal stub for Rust client interaction."""

    def __init__(self) -> None:
        self.sent_payloads: list[str] = []

    def send_device_configuration(self, payload: str) -> None:
        """Record outbound device configuration payload."""
        self.sent_payloads.append(payload)

    def disconnect(self) -> None:
        """Mirror Rust client disconnect API for test cleanup."""
        return None


def _make_client() -> FeagiAgentClient:
    """Create a client instance without running Rust-backed __init__."""
    client = FeagiAgentClient.__new__(FeagiAgentClient)
    client.agent_id = "test-agent"
    client._connected = True
    client._client = _DummyRustClient()
    client._motor_registration_retries = 3
    client._motor_registration_retry_interval_s = 0.01
    client._feagi_host = "127.0.0.1"
    client._feagi_api_port = 8000
    client._feagi_http_timeout_s = 1.0
    return client


def test_send_device_configuration_without_expectations() -> None:
    """Controller-compatible path should send once when no checks requested."""
    client = _make_client()
    client.send_device_configuration('{"foo":"bar"}')
    assert client._client.sent_payloads == ['{"foo":"bar"}']


def test_send_device_configuration_empty_list_uses_retry_path(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Explicit empty list must not skip the retry loop (regression: falsy list vs None)."""
    client = _make_client()
    monkeypatch.setattr(client, "_fetch_existing_cortical_areas", lambda _ids: set())
    monkeypatch.setattr("feagi.pns.client.time.sleep", lambda _s: None)
    client.send_device_configuration('{"foo":"bar"}', expected_cortical_ids=[])
    assert len(client._client.sent_payloads) == 1


def test_send_device_configuration_retries_until_areas_ready(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Verification path should retry until all expected cortical IDs exist."""
    client = _make_client()
    responses = [set(), {"opseAAAAAAA=", "omotAAAAAAA="}]

    def _fake_fetch(_cortical_ids: list[str]) -> set[str]:
        return responses.pop(0)

    monkeypatch.setattr(client, "_fetch_existing_cortical_areas", _fake_fetch)
    monkeypatch.setattr("feagi.pns.client.time.sleep", lambda _s: None)

    client.send_device_configuration(
        '{"foo":"bar"}',
        expected_cortical_ids=["opseAAAAAAA=", "omotAAAAAAA="],
    )
    assert len(client._client.sent_payloads) == 2


def test_send_device_configuration_raises_when_areas_never_ready(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Verification path must fail when cortical areas never appear."""
    client = _make_client()
    client._motor_registration_retries = 2

    monkeypatch.setattr(
        client,
        "_fetch_existing_cortical_areas",
        lambda _ids: set(),
    )
    monkeypatch.setattr("feagi.pns.client.time.sleep", lambda _s: None)

    with pytest.raises(RuntimeError, match="Device registration incomplete"):
        client.send_device_configuration(
            '{"foo":"bar"}',
            expected_cortical_ids=["opseAAAAAAA="],
        )
    assert len(client._client.sent_payloads) == 2
