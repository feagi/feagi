"""
Unit tests for ``feagi.pns.recovery_supervisor``.

The Rust health monitor and reconnect policy are real. Only what is outside the SDK is
faked: the FEAGI HTTP health fetch and the FEAGI session (a fake transport that can be
force-deregistered, taken down and brought back). Time is a value the test passes in.
"""

from __future__ import annotations

import threading

import pytest

import feagi.pns.health_monitor as health_monitor_module
from feagi.pns.health_monitor import (
    FeagiHealthMonitor,
    FeagiHealthSnapshot,
    FeagiRecoveryTrigger,
)
from feagi.pns.recovery_supervisor import (
    DEFAULT_SLOW_RETRY_INTERVAL_S,
    BackgroundSessionSupervisor,
    SessionRecoveryState,
    SessionSupervisor,
)

pytestmark = pytest.mark.skipif(
    not hasattr(FeagiRecoveryTrigger, "not_registered"),
    reason="installed feagi_rust_py_libs wheel predates the NotRegistered recovery trigger",
)

COOLDOWN_MS = 1_000
MAX_FAILURES = 3
WAIT_LIMIT_S = 10.0


class FakeFeagi:
    """Fake transport + health endpoint. ``deregister`` mimics a genome load."""

    def __init__(self) -> None:
        self.up = True
        self.registered = True
        self.genome_num = 1
        self.session = 1
        self.device_registrations = {"servo": 4, "proprioception": 4}
        self.reconnect_times: list[float] = []
        self.registrations_sent: list[dict] = []
        self.now_s = 0.0
        self.block: threading.Event | None = None
        self.entered = threading.Event()

    def deregister(self) -> None:
        self.registered = False
        self.genome_num += 1

    def snapshot(self) -> FeagiHealthSnapshot:
        if not self.up:
            raise RuntimeError("FEAGI unreachable")
        return FeagiHealthSnapshot(self.session, self.genome_num, 100, False, True, True)

    def reconnect(self, reason: str) -> None:
        self.reconnect_times.append(self.now_s)
        self.entered.set()
        if self.block is not None:
            assert self.block.wait(WAIT_LIMIT_S), "test never released the reconnect"
        if not self.up:
            raise RuntimeError("FEAGI unreachable")
        self.registrations_sent.append(dict(self.device_registrations))
        self.registered = True

    def is_registered(self) -> bool:
        return self.registered


@pytest.fixture
def feagi(monkeypatch: pytest.MonkeyPatch) -> FakeFeagi:
    fake = FakeFeagi()
    monkeypatch.setattr(
        health_monitor_module, "_rust_fetch_health_snapshot", lambda _config: fake.snapshot()
    )
    return fake


def _monitor() -> FeagiHealthMonitor:
    return FeagiHealthMonitor(
        feagi_api_host="127.0.0.1",
        feagi_api_port=8000,
        fetch_timeout_ms=500,
        cooldown_ms=COOLDOWN_MS,
        max_consecutive_failures=MAX_FAILURES,
        trigger_on_session_changed=True,
        trigger_on_genome_changed=True,
        trigger_on_genome_load_completed=True,
        trigger_on_back_online=True,
        trigger_on_brain_ready=False,
        trigger_on_transport_send_failed=True,
    )


def _supervisor(feagi: FakeFeagi, states: list | None = None) -> SessionSupervisor:
    return SessionSupervisor(
        monitor=_monitor(),
        reconnect=feagi.reconnect,
        is_registered=feagi.is_registered,
        on_state_change=(states.append if states is not None else None),
    )


def _run(sup: SessionSupervisor, feagi: FakeFeagi, start_s: float, end_s: float, step_s: float):
    steps = round((end_s - start_s) / step_s)
    for i in range(steps + 1):
        feagi.now_s = start_s + i * step_s
        sup.tick(feagi.now_s)


def test_forced_deregistration_causes_exactly_one_reregistration_with_same_registrations(feagi):
    states: list = []
    sup = _supervisor(feagi, states)
    _run(sup, feagi, 0.0, 5.0, 0.5)  # baseline, registered
    assert feagi.reconnect_times == []
    assert sup.state is SessionRecoveryState.REGISTERED

    feagi.deregister()
    _run(sup, feagi, 5.5, 60.0, 0.5)

    assert len(feagi.reconnect_times) == 1
    assert feagi.registrations_sent == [{"servo": 4, "proprioception": 4}]
    assert sup.state is SessionRecoveryState.REGISTERED
    assert states == [SessionRecoveryState.RECONNECTING, SessionRecoveryState.REGISTERED]


def test_feagi_down_attempts_at_0_1_2_then_every_30s_then_recovers(feagi):
    sup = _supervisor(feagi)
    _run(sup, feagi, 0.0, 2.0, 0.5)
    feagi.up = False
    feagi.deregister()
    base = 2.5
    _run(sup, feagi, base, 100.0, 0.5)

    rel = [round(t - base, 3) for t in feagi.reconnect_times]
    # Three fast attempts 1 s apart, then one slow attempt every 30 s after the last fast one.
    assert rel == [0.0, 1.0, 2.0, 32.0, 62.0, 92.0]
    assert sup.state is SessionRecoveryState.CANNOT_REACH
    assert sup.next_slow_attempt_in_s(100.0) == pytest.approx(base + 92.0 + 30.0 - 100.0)

    feagi.up = True
    _run(sup, feagi, 100.5, 200.0, 0.5)
    assert len(feagi.reconnect_times) == 7
    assert feagi.reconnect_times[-1] == pytest.approx(base + 122.0)
    assert sup.state is SessionRecoveryState.REGISTERED
    assert feagi.registrations_sent == [{"servo": 4, "proprioception": 4}]
    assert DEFAULT_SLOW_RETRY_INTERVAL_S == 30.0


def test_no_tight_loop_when_ticked_far_faster_than_the_cooldown(feagi):
    sup = _supervisor(feagi)
    _run(sup, feagi, 0.0, 1.0, 0.1)
    feagi.up = False
    feagi.deregister()
    _run(sup, feagi, 1.01, 31.0, 0.01)  # 3000 ticks in 30 s
    assert len(feagi.reconnect_times) == 3  # 3 fast attempts, next slow one is at 33.01 s


def test_stopping_during_reconnecting_ends_cleanly(feagi):
    # Sync core: stop() while Reconnecting prevents any further attempt.
    sup = _supervisor(feagi)
    _run(sup, feagi, 0.0, 1.0, 0.5)
    feagi.up = False
    feagi.deregister()
    _run(sup, feagi, 1.5, 1.5, 0.5)
    assert sup.state is SessionRecoveryState.RECONNECTING
    sup.stop()
    count = len(feagi.reconnect_times)
    _run(sup, feagi, 2.0, 60.0, 0.5)
    assert len(feagi.reconnect_times) == count

    # Background driver: close() while a rebuild is blocked; the worker exits once it ends.
    fake2 = FakeFeagi()
    fake2.block = threading.Event()
    core = SessionSupervisor(
        monitor=_monitor(), reconnect=fake2.reconnect, is_registered=fake2.is_registered
    )
    bg = BackgroundSessionSupervisor(core=core, poll_interval_s=0.1)
    fake2.registered = False
    bg.tick(0.0)
    _wait_until(lambda: not bg.poll_in_flight)
    bg.tick(0.1)  # applies the poll result, starts the attempt
    assert fake2.entered.wait(WAIT_LIMIT_S)
    assert bg.state is SessionRecoveryState.RECONNECTING and bg.attempt_in_flight
    assert bg.close(wait_timeout_s=0.05) is False  # still blocked in the rebuild
    fake2.block.set()
    assert bg.close(wait_timeout_s=WAIT_LIMIT_S) is True
    bg.tick(5.0)  # no-op after close
    assert len(fake2.reconnect_times) == 1


def test_background_supervisor_never_blocks_tick_and_reconnects_once(feagi):
    feagi.block = threading.Event()
    core = SessionSupervisor(
        monitor=_monitor(), reconnect=feagi.reconnect, is_registered=feagi.is_registered
    )
    bg = BackgroundSessionSupervisor(core=core, poll_interval_s=0.1)
    bg.tick(0.0)
    _wait_until(lambda: not bg.poll_in_flight)
    bg.tick(0.1)
    feagi.deregister()
    for i in range(1, 30):  # baseline poll done above; now poll sees the deregistration
        bg.tick(0.1 * (i + 1))
        _wait_until(lambda: not bg.poll_in_flight)
    assert feagi.entered.wait(WAIT_LIMIT_S)
    # Rebuild is blocked yet tick() keeps returning immediately.
    for i in range(100):
        bg.tick(10.0 + i)
    feagi.block.set()
    _wait_until(lambda: not bg.attempt_in_flight)
    bg.tick(200.0)
    assert len(feagi.reconnect_times) == 1
    assert bg.state is SessionRecoveryState.REGISTERED
    assert bg.close(wait_timeout_s=WAIT_LIMIT_S) is True


def test_invalid_intervals_are_rejected(feagi):
    with pytest.raises(ValueError, match="slow_retry_interval_s"):
        SessionSupervisor(
            monitor=None, reconnect=lambda r: None, is_registered=lambda: True,
            slow_retry_interval_s=0.0,
        )
    with pytest.raises(ValueError, match="poll_interval_s"):
        BackgroundSessionSupervisor(core=_supervisor(feagi), poll_interval_s=0.0)


def _wait_until(predicate, limit_s: float = WAIT_LIMIT_S) -> None:
    gate = threading.Event()
    for _ in range(int(limit_s / 0.001)):
        if predicate():
            return
        gate.wait(0.001)
    raise AssertionError("condition not reached")
