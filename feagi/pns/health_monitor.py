"""
FEAGI agent health monitor (Python facade for the Rust recovery primitives).

This module is a thin Python wrapper around the recovery types defined in
``feagi-core/crates/feagi-agent/src/clients/recovery``. The decision logic
for when to reconnect (which health events count as triggers, cooldown
between attempts, max-failures give-up behaviour) lives entirely in Rust
and is exposed unchanged through the PyO3 ``feagi_rust_py_libs.feagi_agent``
module. This guarantees that the Python, Rust, and (future) Java SDKs all
follow the same observable behaviour without re-implementing any of it.

Design summary
--------------
- :func:`fetch_health_snapshot_blocking` is a Rust HTTP fetch returning a
  ``FeagiHealthSnapshot`` (or raising ``RuntimeError``).
- :class:`FeagiHealthMonitor` composes a Rust ``HealthWatcher`` and a Rust
  ``ReconnectPolicy`` so a controller's main loop can call ``tick()`` once
  per cycle to learn whether it should reconnect now.
- The monitor never performs the reconnect itself: it returns a decision
  object and the controller invokes ``brain_output.reconnect()`` (or the
  agent client's ``reconnect()``) on its own. This keeps ownership of the
  session lifecycle with the controller while keeping the *decision* in
  the Rust source of truth.

@cursor:critical-path
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger("feagi.pns.health_monitor")


# ---------------------------------------------------------------------------
# Rust binding import (single source of truth)
# ---------------------------------------------------------------------------

try:
    from feagi_rust_py_libs.feagi_agent import (  # type: ignore[attr-defined]
        FeagiHealthEvent as _RustHealthEvent,
        FeagiHealthFetchConfig as _RustHealthFetchConfig,
        FeagiHealthSnapshot as _RustHealthSnapshot,
        FeagiHealthWatcher as _RustHealthWatcher,
        FeagiReconnectDecision as _RustReconnectDecision,
        FeagiReconnectPolicy as _RustReconnectPolicy,
        FeagiReconnectPolicyConfig as _RustReconnectPolicyConfig,
        FeagiRecoveryTrigger as _RustRecoveryTrigger,
        fetch_health_snapshot as _rust_fetch_health_snapshot,
    )
except ImportError as exc:  # pragma: no cover - import failure surfaces cleanly
    raise ImportError(
        "feagi.pns.health_monitor requires feagi_rust_py_libs >= 0.0.97 "
        "with recovery primitives. Re-build the wheel via maturin.",
    ) from exc


# Public re-exports so callers can type-annotate without reaching into the
# Rust module directly.
FeagiHealthSnapshot = _RustHealthSnapshot
FeagiHealthEvent = _RustHealthEvent
FeagiHealthWatcher = _RustHealthWatcher
FeagiRecoveryTrigger = _RustRecoveryTrigger
FeagiReconnectPolicyConfig = _RustReconnectPolicyConfig
FeagiReconnectPolicy = _RustReconnectPolicy
FeagiReconnectDecision = _RustReconnectDecision
FeagiHealthFetchConfig = _RustHealthFetchConfig


def fetch_health_snapshot_blocking(
    feagi_api_host: str,
    feagi_api_port: int,
    timeout_ms: int,
) -> FeagiHealthSnapshot:
    """
    Blocking HTTP fetch of FEAGI's ``/v1/system/health_check`` endpoint.

    Returns a :class:`FeagiHealthSnapshot`. Raises ``RuntimeError`` on any
    fetch / parse failure; callers driving a recovery loop should catch
    and pass the failure to ``FeagiHealthWatcher.observe_unreachable()``.

    All three arguments are required: this function intentionally has no
    defaults so the central-configuration policy is honoured.
    """
    config = FeagiHealthFetchConfig(feagi_api_host, feagi_api_port, timeout_ms)
    return _rust_fetch_health_snapshot(config)


@dataclass(frozen=True)
class FeagiHealthMonitorTickResult:
    """
    Outcome of a single :meth:`FeagiHealthMonitor.tick` call.

    Attributes:
        decision: The :class:`FeagiReconnectDecision` returned by the Rust
            policy. Use ``decision.is_attempt_now()`` /
            ``decision.is_give_up()`` etc. to branch.
        events: List of :class:`FeagiHealthEvent` observed this tick. May
            be empty (no transitions, or first observation establishing a
            baseline). Useful for telemetry/logging.
        snapshot_fetched: ``True`` if the underlying HTTP fetch succeeded
            this tick. ``False`` if the watcher recorded an unreachable.
        fetch_error: Stringified exception if the fetch failed; ``None``
            otherwise.
    """

    decision: FeagiReconnectDecision
    events: list
    snapshot_fetched: bool
    fetch_error: Optional[str]


class FeagiHealthMonitor:
    """
    Compose a Rust ``HealthWatcher`` and ``ReconnectPolicy`` into a single
    Python-friendly object that a controller can call once per main-loop
    iteration.

    The class **does not** perform reconnects itself: a tick returns a
    decision object and the controller decides whether to invoke
    :meth:`feagi.pns.brain_output.BrainOutput.reconnect`. This keeps the
    session lifecycle in the controller's hands while the decision logic
    stays in Rust.

    Construction requires every threshold explicitly (no defaults), per
    the project-wide central-configuration policy.
    """

    def __init__(
        self,
        *,
        feagi_api_host: str,
        feagi_api_port: int,
        fetch_timeout_ms: int,
        cooldown_ms: int,
        max_consecutive_failures: int,
        trigger_on_session_changed: bool,
        trigger_on_genome_changed: bool,
        trigger_on_genome_load_completed: bool,
        trigger_on_back_online: bool,
        trigger_on_brain_ready: bool,
        trigger_on_transport_send_failed: bool,
    ) -> None:
        if cooldown_ms < 0:
            raise ValueError("cooldown_ms must be >= 0")
        if max_consecutive_failures < 0:
            raise ValueError("max_consecutive_failures must be >= 0")
        if fetch_timeout_ms <= 0:
            raise ValueError("fetch_timeout_ms must be > 0")

        self._fetch_config = FeagiHealthFetchConfig(
            feagi_api_host, feagi_api_port, fetch_timeout_ms
        )
        self._watcher = FeagiHealthWatcher()
        self._policy = FeagiReconnectPolicy(
            FeagiReconnectPolicyConfig(
                cooldown_ms,
                max_consecutive_failures,
                trigger_on_session_changed,
                trigger_on_genome_changed,
                trigger_on_genome_load_completed,
                trigger_on_back_online,
                trigger_on_brain_ready,
                trigger_on_transport_send_failed,
            )
        )

    @property
    def watcher(self) -> FeagiHealthWatcher:
        return self._watcher

    @property
    def policy(self) -> FeagiReconnectPolicy:
        return self._policy

    def tick(
        self,
        *,
        now_ms: int,
        transport_send_failed: bool = False,
    ) -> FeagiHealthMonitorTickResult:
        """
        Fetch a fresh FEAGI health snapshot, feed it to the watcher, and
        ask the policy whether the controller should reconnect now.

        Args:
            now_ms: Monotonic millisecond clock value supplied by the
                caller (the policy never reads time itself).
            transport_send_failed: ``True`` if the controller's IO loop
                observed a non-transient transport send failure since the
                last tick. Becomes a one-shot ``TransportSendFailed``
                trigger inside the Rust policy.

        Returns:
            A :class:`FeagiHealthMonitorTickResult` describing what was
            observed and what the policy decided.
        """
        fetch_error: Optional[str] = None
        snapshot_fetched = False
        try:
            snapshot = _rust_fetch_health_snapshot(self._fetch_config)
            events = list(self._watcher.observe(snapshot))
            snapshot_fetched = True
        except Exception as exc:  # noqa: BLE001 - want any fetch error
            fetch_error = str(exc)
            events = list(self._watcher.observe_unreachable())

        triggers = [FeagiRecoveryTrigger.health(event) for event in events]
        if transport_send_failed:
            triggers.append(FeagiRecoveryTrigger.transport_send_failed())

        decision = self._policy.decide(triggers, now_ms)
        return FeagiHealthMonitorTickResult(
            decision=decision,
            events=events,
            snapshot_fetched=snapshot_fetched,
            fetch_error=fetch_error,
        )

    def record_attempt_succeeded(self) -> None:
        """Inform the policy that the most recent reconnect attempt succeeded."""
        self._policy.record_attempt_succeeded()

    def record_attempt_failed(self) -> None:
        """Inform the policy that the most recent reconnect attempt failed."""
        self._policy.record_attempt_failed()

    def reset_policy(self) -> None:
        """Forget cooldown/failure state (e.g. after a manual reconnect)."""
        self._policy.reset()


__all__ = [
    "FeagiHealthEvent",
    "FeagiHealthFetchConfig",
    "FeagiHealthMonitor",
    "FeagiHealthMonitorTickResult",
    "FeagiHealthSnapshot",
    "FeagiHealthWatcher",
    "FeagiReconnectDecision",
    "FeagiReconnectPolicy",
    "FeagiReconnectPolicyConfig",
    "FeagiRecoveryTrigger",
    "fetch_health_snapshot_blocking",
]
