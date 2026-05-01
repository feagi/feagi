"""
Unit tests for ``feagi.pns.health_monitor``.

These tests exercise the Python facade that wraps the Rust recovery
primitives. The decision logic itself is tested at the Rust level (see
``feagi-core/crates/feagi-agent/src/clients/recovery``) - here we verify
the binding surface and the ``FeagiHealthMonitor`` composition behave
exactly as documented and that no Python-side defaults sneak in.
"""

from __future__ import annotations

import pytest

from feagi.pns.health_monitor import (
    FeagiHealthEvent,
    FeagiHealthFetchConfig,
    FeagiHealthMonitor,
    FeagiHealthSnapshot,
    FeagiHealthWatcher,
    FeagiReconnectPolicy,
    FeagiReconnectPolicyConfig,
    FeagiRecoveryTrigger,
    fetch_health_snapshot_blocking,
)


def _snapshot(
    session: int | None = 1,
    genome_num: int | None = 1,
    timestamp: int | None = 100,
    loading: bool = False,
    ready: bool = True,
) -> FeagiHealthSnapshot:
    return FeagiHealthSnapshot(session, genome_num, timestamp, loading, True, ready)


def _policy_config(**overrides) -> FeagiReconnectPolicyConfig:
    base = dict(
        cooldown_ms=1_000,
        max_consecutive_failures=3,
        trigger_on_session_changed=True,
        trigger_on_genome_changed=True,
        trigger_on_genome_load_completed=True,
        trigger_on_back_online=True,
        trigger_on_brain_ready=False,
        trigger_on_transport_send_failed=True,
    )
    base.update(overrides)
    return FeagiReconnectPolicyConfig(**base)


def test_health_watcher_first_observation_yields_no_events():
    watcher = FeagiHealthWatcher()
    assert watcher.has_baseline() is False
    events = watcher.observe(_snapshot())
    assert events == []
    assert watcher.has_baseline() is True
    assert watcher.is_reachable() is True


def test_health_watcher_detects_genome_change():
    watcher = FeagiHealthWatcher()
    watcher.observe(_snapshot(genome_num=1, timestamp=100))
    events = watcher.observe(_snapshot(genome_num=2, timestamp=200))
    kinds = [e.kind for e in events]
    assert "genome_changed" in kinds
    target = next(e for e in events if e.kind == "genome_changed")
    assert target.old_genome_num == 1
    assert target.new_genome_num == 2
    assert target.old_genome_timestamp == 100
    assert target.new_genome_timestamp == 200


def test_health_watcher_unreachable_then_back_online():
    watcher = FeagiHealthWatcher()
    watcher.observe(_snapshot())
    drop = watcher.observe_unreachable()
    assert [e.kind for e in drop] == ["feagi_unreachable"]
    repeat = watcher.observe_unreachable()
    assert repeat == []
    recovered = watcher.observe(_snapshot())
    assert [e.kind for e in recovered] == ["feagi_back_online"]


def test_reconnect_policy_attempts_then_cools_down():
    policy = FeagiReconnectPolicy(_policy_config())
    decision = policy.decide(
        [FeagiRecoveryTrigger.transport_send_failed()], 0
    )
    assert decision.kind == "attempt_now"
    assert decision.reason == "transport send failed"

    cooldown = policy.decide(
        [FeagiRecoveryTrigger.transport_send_failed()], 250
    )
    assert cooldown.kind == "retry_after"
    assert cooldown.wait_ms == 750

    later = policy.decide(
        [FeagiRecoveryTrigger.transport_send_failed()], 1_500
    )
    assert later.kind == "attempt_now"


def test_reconnect_policy_give_up_after_max_failures():
    policy = FeagiReconnectPolicy(_policy_config(max_consecutive_failures=2))
    policy.record_attempt_failed()
    policy.record_attempt_failed()
    decision = policy.decide(
        [FeagiRecoveryTrigger.transport_send_failed()], 10_000
    )
    assert decision.kind == "give_up"
    assert decision.consecutive_failures == 2


def test_health_monitor_validates_inputs():
    with pytest.raises(ValueError):
        FeagiHealthMonitor(
            feagi_api_host="x",
            feagi_api_port=1,
            fetch_timeout_ms=0,
            cooldown_ms=10,
            max_consecutive_failures=1,
            trigger_on_session_changed=True,
            trigger_on_genome_changed=True,
            trigger_on_genome_load_completed=True,
            trigger_on_back_online=True,
            trigger_on_brain_ready=False,
            trigger_on_transport_send_failed=True,
        )


def test_health_monitor_tick_against_unreachable_endpoint():
    """
    Pointing the monitor at a closed port must yield ``snapshot_fetched=False``
    and propagate the fetch error string. Subsequent ticks with
    ``transport_send_failed=True`` should produce ``attempt_now`` decisions.
    """
    monitor = FeagiHealthMonitor(
        feagi_api_host="127.0.0.1",
        feagi_api_port=1,
        fetch_timeout_ms=50,
        cooldown_ms=100,
        max_consecutive_failures=5,
        trigger_on_session_changed=True,
        trigger_on_genome_changed=True,
        trigger_on_genome_load_completed=True,
        trigger_on_back_online=True,
        trigger_on_brain_ready=False,
        trigger_on_transport_send_failed=True,
    )
    first = monitor.tick(now_ms=0, transport_send_failed=False)
    assert first.snapshot_fetched is False
    assert first.fetch_error is not None
    assert first.decision.kind == "skip"

    second = monitor.tick(now_ms=200, transport_send_failed=True)
    assert second.decision.kind == "attempt_now"
    assert second.decision.reason == "transport send failed"
    monitor.record_attempt_succeeded()
    assert monitor.policy.consecutive_failures == 0


def test_fetch_health_snapshot_blocking_raises_on_unreachable():
    with pytest.raises(RuntimeError):
        fetch_health_snapshot_blocking(
            feagi_api_host="127.0.0.1",
            feagi_api_port=1,
            timeout_ms=50,
        )


def test_health_fetch_config_constructs():
    cfg = FeagiHealthFetchConfig("h.example", 8000, 250)
    assert "h.example" in repr(cfg)


def test_reconnect_decision_kinds_exhaustive():
    policy = FeagiReconnectPolicy(_policy_config())
    assert policy.decide([], 0).kind == "skip"

    snap = FeagiHealthSnapshot(1, 1, 100, False, True, True)
    snap2 = FeagiHealthSnapshot(2, 1, 100, False, True, True)
    watcher = FeagiHealthWatcher()
    watcher.observe(snap)
    events = watcher.observe(snap2)
    assert any(e.kind == "session_changed" for e in events)
    decision = policy.decide(
        [FeagiRecoveryTrigger.health(e) for e in events], 5
    )
    assert decision.kind == "attempt_now"
    assert "feagi session changed" in decision.reason


def test_health_event_repr_is_stable():
    snap1 = FeagiHealthSnapshot(1, 1, 100, False, True, True)
    snap2 = FeagiHealthSnapshot(1, 2, 200, False, True, True)
    watcher = FeagiHealthWatcher()
    watcher.observe(snap1)
    events = watcher.observe(snap2)
    assert any("GenomeChanged" in repr(e) for e in events)
    assert isinstance(events[0], FeagiHealthEvent)
    assert isinstance(events[0].kind, str)
    assert isinstance(events[0].new_genome_num, int)
