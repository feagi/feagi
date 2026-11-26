"""A rejected registration is reported once; ``connect`` never sleeps and retries."""

import pytest

import feagi.pns.client as client_mod
from feagi.pns.client import AgentType, FeagiAgentClient

ALREADY_REGISTERED = "FeagiAgentError: Connection failed: Client already registered!"


class _RejectingPyAgentClient:
    """Stub PyAgentClient whose registration is always rejected as a duplicate."""

    instances: list["_RejectingPyAgentClient"] = []

    def __init__(self, _config: object) -> None:
        self.connect_calls = 0
        _RejectingPyAgentClient.instances.append(self)

    def connect(self) -> None:
        self.connect_calls += 1
        raise RuntimeError(ALREADY_REGISTERED)


@pytest.fixture(autouse=True)
def _reset_instances() -> None:
    _RejectingPyAgentClient.instances.clear()
    yield
    _RejectingPyAgentClient.instances.clear()


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


def test_connect_raises_already_registered_after_a_single_attempt(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(client_mod, "PyAgentClient", _RejectingPyAgentClient)

    def _no_sleep(_seconds: float) -> None:
        raise AssertionError("connect() must not sleep and retry a rejected registration")

    monkeypatch.setattr(client_mod.time, "sleep", _no_sleep)

    c = _minimal_configured_client()
    with pytest.raises(RuntimeError, match="already registered"):
        c.connect()

    assert len(_RejectingPyAgentClient.instances) == 1
    assert _RejectingPyAgentClient.instances[0].connect_calls == 1
    assert c._connected is False


def test_connect_graceful_returns_false_on_already_registered(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(client_mod, "PyAgentClient", _RejectingPyAgentClient)
    monkeypatch.setattr(client_mod.time, "sleep", lambda _s: None)

    c = _minimal_configured_client()
    assert c.connect(graceful=True) is False
    assert _RejectingPyAgentClient.instances[0].connect_calls == 1
