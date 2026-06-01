"""Tests for FeagiAgentClient duplicate-registration retry on connect."""

import pytest

import feagi.pns.client as client_mod
from feagi.pns.client import AgentType, FeagiAgentClient


class _FlakyPyAgentClient:
    """Stub PyAgentClient: fail connect with AlreadyRegistered once, then ok."""

    instances: list["_FlakyPyAgentClient"] = []
    connect_calls = 0

    def __init__(self, _config: object) -> None:
        _FlakyPyAgentClient.instances.append(self)

    def connect(self) -> None:
        _FlakyPyAgentClient.connect_calls += 1
        if _FlakyPyAgentClient.connect_calls == 1:
            raise RuntimeError(
                "FeagiAgentError: Connection failed: Client already registered!",
            )

    def disconnect(self) -> None:
        return None


@pytest.fixture(autouse=True)
def _reset_flaky() -> None:
    _FlakyPyAgentClient.instances.clear()
    _FlakyPyAgentClient.connect_calls = 0
    yield
    _FlakyPyAgentClient.instances.clear()
    _FlakyPyAgentClient.connect_calls = 0


def _minimal_configured_client() -> FeagiAgentClient:
    c = FeagiAgentClient.__new__(FeagiAgentClient)
    c.agent_id = "test"
    c.agent_type = AgentType.MOTOR
    c._config = object()
    c._client = None
    c._connected = False
    c._feagi_host = "127.0.0.1"
    c._registration_port = 30001
    c._sensory_port = 5563
    c._motor_registration_retry_interval_s = 1.0
    return c


def test_connect_retries_once_on_already_registered(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(client_mod, "PyAgentClient", _FlakyPyAgentClient)
    monkeypatch.setattr(client_mod.time, "sleep", lambda _s: None)

    c = _minimal_configured_client()
    assert c.connect() is True
    assert c._connected is True
    assert _FlakyPyAgentClient.connect_calls == 2
    assert len(_FlakyPyAgentClient.instances) == 2


def test_connect_fails_after_retry_exhausted(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class _AlwaysDup(_FlakyPyAgentClient):
        def connect(self) -> None:
            raise RuntimeError(
                "FeagiAgentError: Connection failed: Client already registered!",
            )

    monkeypatch.setattr(client_mod, "PyAgentClient", _AlwaysDup)
    monkeypatch.setattr(client_mod.time, "sleep", lambda _s: None)

    c = _minimal_configured_client()
    with pytest.raises(RuntimeError, match="already registered"):
        c.connect()


def test_duplicate_retry_delay_from_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("FEAGI_AGENT_DUPLICATE_REGISTRATION_RETRY_S", "3.5")
    c = _minimal_configured_client()
    assert c._duplicate_registration_retry_delay_s() == 3.5


def test_duplicate_retry_delay_derived_from_heartbeat(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("FEAGI_AGENT_DUPLICATE_REGISTRATION_RETRY_S", raising=False)
    c = _minimal_configured_client()
    c._motor_registration_retry_interval_s = 1.0
    assert c._duplicate_registration_retry_delay_s() == 2.25


def test_duplicate_retry_delay_minimum_for_short_heartbeat(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("FEAGI_AGENT_DUPLICATE_REGISTRATION_RETRY_S", raising=False)
    c = _minimal_configured_client()
    c._motor_registration_retry_interval_s = 0.25
    assert c._duplicate_registration_retry_delay_s() == 2.25


def test_duplicate_retry_delay_caps_long_heartbeat(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("FEAGI_AGENT_DUPLICATE_REGISTRATION_RETRY_S", raising=False)
    c = _minimal_configured_client()
    c._motor_registration_retry_interval_s = 5.0
    assert c._duplicate_registration_retry_delay_s() == 4.0
