"""
SDK-owned recovery supervisor: keeps an agent session registered with FEAGI.

FEAGI ends every agent session on purpose when a genome is loaded, and a session can
also be lost with the link. The agent finds out only when its next heartbeat is refused.
This module is the Python counterpart of the Rust supervisor in
``feagi-agent/src/clients/recovery/supervisor.rs`` and uses the same names and policy:

* States (:class:`SessionRecoveryState`): ``REGISTERED``, ``RECONNECTING`` (the fast
  attempts are running) and ``CANNOT_REACH`` (the fast attempts are used up).
* The fast attempts come from the shared reconnect policy exposed by
  :class:`feagi.pns.health_monitor.FeagiHealthMonitor` (``cooldown_ms`` apart,
  ``max_consecutive_failures`` of them). When they are spent the supervisor does not give
  up: the state becomes ``CANNOT_REACH`` and it retries every ``slow_retry_interval_s``
  (default :data:`DEFAULT_SLOW_RETRY_INTERVAL_S`) forever.
* A rebuild re-registers the session with the same device registrations
  (``brain_output.reconnect``). One session end causes one rebuild; a failed rebuild is
  spaced by the policy cooldown or the slow interval, so there is no tight loop.
* The supervisor never reads a clock: the driver passes ``now_s`` (monotonic seconds).
* Not being registered is a level trigger: every poll tells the monitor, so a failed or
  deferred attempt is asked about again on the next poll.

Two drivers share one decision core:

* :class:`SessionSupervisor` is the synchronous core (``poll`` / ``begin_attempt`` /
  ``run_attempt``, or ``tick`` to do all three inline). Use it directly only where blocking
  for a health fetch and a rebuild is acceptable.
* :class:`BackgroundSessionSupervisor` runs the health fetch and every rebuild on one worker
  thread, so the caller's loop never blocks on either. The state-change callback for the
  start of an attempt runs on the caller's thread before the worker touches the client, so a
  caller that gates its FEAGI client calls on the state never overlaps a rebuild.

The monitor and the policy are touched from the worker thread only. This module has no
knowledge of any robot, model or experiment.

@cursor:critical-path
@cursor:cross-sdk-contract
"""

from __future__ import annotations

import enum
import logging
import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Optional

logger = logging.getLogger("feagi.pns.recovery_supervisor")

#: Slow retry interval used after the fast attempts are spent (product decision: ~30 s).
DEFAULT_SLOW_RETRY_INTERVAL_S: float = 30.0

_SLOW_RETRY_REASON = "slow retry: client not registered"


class SessionRecoveryState(str, enum.Enum):
    """Where an agent session stands (same names as the Rust ``SessionRecoveryState``)."""

    REGISTERED = "registered"
    RECONNECTING = "reconnecting"
    CANNOT_REACH = "cannot_reach"


class SupervisorAction(enum.Enum):
    """What a poll asks the driver to do."""

    NONE = "none"
    ATTEMPT = "attempt"


@dataclass(frozen=True)
class PollResult:
    """Outcome of one :meth:`SessionSupervisor.poll`."""

    action: SupervisorAction
    reason: Optional[str] = None
    #: State the supervisor should be in after this poll (applied by ``apply_poll``).
    state: Optional[SessionRecoveryState] = None
    #: True when the poll itself failed (the send-failure flag was not consumed).
    failed: bool = False


@dataclass(frozen=True)
class SupervisedTickReport:
    """What one synchronous :meth:`SessionSupervisor.tick` did."""

    state: SessionRecoveryState
    rebuild_attempted: bool
    rebuild_succeeded: bool
    reason: Optional[str] = None


class SessionSupervisor:
    """
    Decision core of the recovery supervisor (see the module documentation).

    Args:
        monitor: ``FeagiHealthMonitor``-shaped object: ``tick(now_ms=, transport_send_failed=,
            not_registered=)`` returning an object with ``decision`` and ``events``, plus
            ``record_attempt_succeeded``, ``record_attempt_failed`` and ``reset_policy``.
        reconnect: Rebuilds the session and re-registers the same device registrations;
            receives the reason. Raising means the attempt failed.
        is_registered: True while the client holds a live registration.
        slow_retry_interval_s: Interval between attempts once the fast ones are spent.
        on_state_change: Called with the new state after every change. May run on the
            caller's thread or the worker's thread; keep it short and thread-safe.
        initial_state: ``REGISTERED`` for a session that is connected when supervision starts;
            ``RECONNECTING`` for one that never connected (the Rust supervisor reports
            ``Reconnecting`` until the first registration).
    """

    def __init__(
        self,
        *,
        monitor: Any,
        reconnect: Callable[[str], None],
        is_registered: Callable[[], bool],
        slow_retry_interval_s: float = DEFAULT_SLOW_RETRY_INTERVAL_S,
        on_state_change: Optional[Callable[[SessionRecoveryState], None]] = None,
        initial_state: SessionRecoveryState = SessionRecoveryState.REGISTERED,
    ) -> None:
        if slow_retry_interval_s <= 0.0:
            raise ValueError(
                f"slow_retry_interval_s must be > 0, got {slow_retry_interval_s!r}"
            )
        self._monitor = monitor
        self._reconnect = reconnect
        self._is_registered = is_registered
        self._slow_retry_interval_s = slow_retry_interval_s
        self._on_state_change = on_state_change
        self._lock = threading.Lock()
        self._state = initial_state
        self._stopped = False
        # Seconds on the driver's clock.
        self._last_attempt_s: Optional[float] = None
        self._next_slow_attempt_s: Optional[float] = None

    # --- state ------------------------------------------------------------------------

    @property
    def state(self) -> SessionRecoveryState:
        with self._lock:
            return self._state

    @property
    def stopped(self) -> bool:
        with self._lock:
            return self._stopped

    def stop(self) -> None:
        """Stop deciding: no new attempt starts after this. An attempt in flight finishes."""
        with self._lock:
            self._stopped = True

    def next_slow_attempt_in_s(self, now_s: float) -> Optional[float]:
        """Seconds until the next slow attempt; ``Some`` only in ``CANNOT_REACH``."""
        with self._lock:
            if self._state is not SessionRecoveryState.CANNOT_REACH:
                return None
            if self._next_slow_attempt_s is None:
                return None
            return max(0.0, self._next_slow_attempt_s - now_s)

    def _set_state(self, new_state: SessionRecoveryState) -> None:
        with self._lock:
            if new_state is self._state:
                return
            self._state = new_state
        logger.info("[RECOVERY] Session state -> %s", new_state.value)
        if self._on_state_change is not None:
            try:
                self._on_state_change(new_state)
            except Exception as exc:  # noqa: BLE001 - a callback must not break recovery
                logger.warning("[RECOVERY] State-change callback failed: %s", exc)

    # --- decision ---------------------------------------------------------------------

    def poll(self, now_s: float, *, transport_send_failed: bool = False) -> PollResult:
        """
        One health fetch plus the policy decision. Changes no state and starts nothing.

        Returns what the driver should do and the state it should apply with
        :meth:`apply_poll`.
        """
        try:
            registered = bool(self._is_registered())
            result = self._monitor.tick(
                now_ms=int(now_s * 1000.0),
                transport_send_failed=transport_send_failed,
                not_registered=not registered,
            )
        except Exception as exc:  # noqa: BLE001 - a failed poll must never stop the controller
            logger.warning("[RECOVERY] Health monitor tick failed: %s", exc)
            return PollResult(SupervisorAction.NONE, failed=True)
        for event in result.events:
            logger.info("[RECOVERY] Observed health event: %s", getattr(event, "kind", event))

        current = self.state
        # Registered again (by us, or by the caller on its own): re-arm everything.
        if registered and current is not SessionRecoveryState.REGISTERED:
            self._monitor.reset_policy()
            with self._lock:
                self._next_slow_attempt_s = None
            return PollResult(SupervisorAction.NONE, state=SessionRecoveryState.REGISTERED)

        decision = result.decision
        if decision.is_attempt_now():
            return PollResult(
                SupervisorAction.ATTEMPT,
                reason=decision.reason,
                state=SessionRecoveryState.REGISTERED if registered else None,
            )
        if decision.is_give_up():
            if registered:
                return PollResult(SupervisorAction.NONE, state=SessionRecoveryState.REGISTERED)
            # Fast attempts are spent: keep trying slowly, forever.
            with self._lock:
                if self._next_slow_attempt_s is None:
                    base = self._last_attempt_s if self._last_attempt_s is not None else now_s
                    self._next_slow_attempt_s = base + self._slow_retry_interval_s
                due = self._next_slow_attempt_s
            if now_s >= due:
                return PollResult(
                    SupervisorAction.ATTEMPT,
                    reason=_SLOW_RETRY_REASON,
                    state=SessionRecoveryState.CANNOT_REACH,
                )
            return PollResult(SupervisorAction.NONE, state=SessionRecoveryState.CANNOT_REACH)
        if decision.is_retry_after():
            logger.debug(
                "[RECOVERY] Reconnect deferred %s ms (reason=%s)",
                decision.wait_ms,
                decision.reason,
            )
        if registered:
            return PollResult(SupervisorAction.NONE)
        if current is SessionRecoveryState.CANNOT_REACH:
            return PollResult(SupervisorAction.NONE)
        return PollResult(SupervisorAction.NONE, state=SessionRecoveryState.RECONNECTING)

    def apply_poll(self, result: PollResult) -> None:
        """Apply the state a poll decided on (call on the driver's thread)."""
        if result.state is not None:
            self._set_state(result.state)

    def begin_attempt(self, reason: str, now_s: float) -> None:
        """
        Mark the start of an attempt: the state leaves ``REGISTERED`` and the attempt time is
        recorded. Call on the driver's thread before the rebuild runs.
        """
        with self._lock:
            self._last_attempt_s = now_s
        if self.state is not SessionRecoveryState.CANNOT_REACH:
            self._set_state(SessionRecoveryState.RECONNECTING)
        logger.info("[RECOVERY] Re-registering the session (reason=%s)", reason)

    def run_attempt(self, reason: str, now_s: float) -> bool:
        """One rebuild; reports the outcome to the policy. Returns True on success."""
        try:
            self._reconnect(reason)
        except Exception as exc:  # noqa: BLE001 - any rebuild error is a failed attempt
            logger.warning("[RECOVERY] Reconnect attempt failed (reason=%s): %s", reason, exc)
            self._monitor.record_attempt_failed()
            with self._lock:
                if self._state is SessionRecoveryState.CANNOT_REACH:
                    self._next_slow_attempt_s = now_s + self._slow_retry_interval_s
            return False
        self._monitor.record_attempt_succeeded()
        self._monitor.reset_policy()
        with self._lock:
            self._next_slow_attempt_s = None
        logger.info("[RECOVERY] Reconnect succeeded (reason=%s)", reason)
        self._set_state(SessionRecoveryState.REGISTERED)
        return True

    # --- synchronous driver -----------------------------------------------------------

    def tick(
        self, now_s: float, *, transport_send_failed: bool = False
    ) -> SupervisedTickReport:
        """Poll, and run the attempt inline when the poll asks for one. Blocks for both."""
        if self.stopped:
            return SupervisedTickReport(self.state, False, False)
        result = self.poll(now_s, transport_send_failed=transport_send_failed)
        self.apply_poll(result)
        if result.action is SupervisorAction.ATTEMPT and result.reason is not None:
            self.begin_attempt(result.reason, now_s)
            ok = self.run_attempt(result.reason, now_s)
            return SupervisedTickReport(self.state, True, ok, result.reason)
        return SupervisedTickReport(self.state, False, False)


@dataclass(frozen=True)
class _PollJob:
    now_s: float
    transport_send_failed: bool


@dataclass(frozen=True)
class _AttemptJob:
    reason: str
    now_s: float


class BackgroundSessionSupervisor:
    """
    Run a :class:`SessionSupervisor` without blocking the caller's loop.

    Call :meth:`tick` once per loop pass from one thread (the loop thread). It only requests
    a health poll on the configured cadence and consumes the latest decision the worker
    handed back; the health fetch and every rebuild run on one long-lived worker thread (never
    two jobs at once). The state change for the start of an attempt happens on the loop thread
    before the worker starts the rebuild.
    """

    def __init__(self, *, core: SessionSupervisor, poll_interval_s: float) -> None:
        if poll_interval_s <= 0.0:
            raise ValueError(f"poll_interval_s must be > 0, got {poll_interval_s!r}")
        self._core = core
        self._poll_interval_s = poll_interval_s
        self._lock = threading.Condition()
        self._job: _PollJob | _AttemptJob | None = None
        self._in_flight = False
        self._poll_in_flight = False
        self._poll_result: Optional[PollResult] = None
        self._poll_now_s = 0.0
        self._transport_send_failed = False
        self._closed = False
        self._worker: Optional[threading.Thread] = None
        self._last_poll_at: Optional[float] = None

    @property
    def state(self) -> SessionRecoveryState:
        return self._core.state

    @property
    def io_allowed(self) -> bool:
        """True while the session is ``REGISTERED``: the only time FEAGI client calls are safe."""
        return self._core.state is SessionRecoveryState.REGISTERED

    @property
    def core(self) -> SessionSupervisor:
        return self._core

    @property
    def attempt_in_flight(self) -> bool:
        with self._lock:
            return self._in_flight

    @property
    def poll_in_flight(self) -> bool:
        with self._lock:
            return self._poll_in_flight

    def note_transport_send_failed(self) -> None:
        """Record that a sensory send failed; becomes a one-shot trigger at the next poll."""
        with self._lock:
            self._transport_send_failed = True

    def close(self, *, wait_timeout_s: Optional[float] = None) -> bool:
        """
        Stop supervising: no new attempt starts and the worker exits once its current job
        ends. With ``wait_timeout_s`` it waits that long for the worker; returns True when the
        worker is gone (or never started).
        """
        self._core.stop()
        with self._lock:
            self._closed = True
            self._lock.notify_all()
        worker = self._worker
        if worker is None:
            return True
        if wait_timeout_s is not None:
            worker.join(wait_timeout_s)
        return not worker.is_alive()

    def tick(self, now_s: float) -> None:
        """
        Advance recovery by one loop pass. Never blocks on a fetch or a reconnect. Call from
        the loop thread only, once per pass, before any FEAGI client call.
        """
        if self._closed:
            return
        self._ensure_worker()
        self._apply_poll_result()
        with self._lock:
            if self._in_flight or self._poll_in_flight or self._closed:
                return
        if (
            self._last_poll_at is not None
            and (now_s - self._last_poll_at) < self._poll_interval_s
        ):
            return
        self._last_poll_at = now_s
        with self._lock:
            send_failed = self._transport_send_failed
            self._transport_send_failed = False
            self._poll_in_flight = True
        self._submit(_PollJob(now_s, send_failed))

    def _ensure_worker(self) -> None:
        if self._worker is not None:
            return
        self._worker = threading.Thread(
            target=self._run_worker, name="feagi-session-supervisor", daemon=True
        )
        self._worker.start()

    def _submit(self, job: _PollJob | _AttemptJob) -> None:
        with self._lock:
            self._job = job
            self._lock.notify_all()

    def _apply_poll_result(self) -> None:
        with self._lock:
            result = self._poll_result
            now_s = self._poll_now_s
            self._poll_result = None
        if result is None:
            return
        self._core.apply_poll(result)
        if result.action is SupervisorAction.ATTEMPT and result.reason is not None:
            # State change (gate closes) on this thread, before the worker touches the client.
            self._core.begin_attempt(result.reason, now_s)
            with self._lock:
                self._in_flight = True
            self._submit(_AttemptJob(result.reason, now_s))

    def _run_worker(self) -> None:
        while True:
            with self._lock:
                while self._job is None and not self._closed:
                    self._lock.wait()
                if self._closed:
                    return
                job = self._job
                self._job = None
            if isinstance(job, _PollJob):
                result = self._core.poll(job.now_s, transport_send_failed=job.transport_send_failed)
                if result.failed and job.transport_send_failed:
                    self.note_transport_send_failed()
                with self._lock:
                    self._poll_result = result
                    self._poll_now_s = job.now_s
                    self._poll_in_flight = False
            else:
                try:
                    self._core.run_attempt(job.reason, job.now_s)
                finally:
                    with self._lock:
                        self._in_flight = False


def create_session_supervisor(
    *,
    brain_output: Any,
    feagi_api_host: str,
    feagi_api_port: int,
    health_poll_interval_s: float,
    health_fetch_timeout_ms: int,
    reconnect_cooldown_ms: int,
    reconnect_max_failures: int,
    slow_retry_interval_s: float = DEFAULT_SLOW_RETRY_INTERVAL_S,
    on_state_change: Optional[Callable[[SessionRecoveryState], None]] = None,
    after_reconnect: Optional[Callable[[], None]] = None,
    initial_state: SessionRecoveryState = SessionRecoveryState.REGISTERED,
) -> BackgroundSessionSupervisor:
    """
    Build the supervisor a controller drives: the Rust health monitor and reconnect policy,
    ``brain_output.reconnect`` as the rebuild and ``brain_output.is_registered`` as the probe.

    The four recovery values normally come from ``feagi.config.load_recovery_config()``
    (all required; no defaults here). ``after_reconnect`` runs on the worker thread after a
    successful rebuild (for example to refresh a status field that carries the session id).
    """
    from feagi.pns.health_monitor import FeagiHealthMonitor

    monitor = FeagiHealthMonitor(
        feagi_api_host=feagi_api_host,
        feagi_api_port=feagi_api_port,
        fetch_timeout_ms=health_fetch_timeout_ms,
        cooldown_ms=reconnect_cooldown_ms,
        max_consecutive_failures=reconnect_max_failures,
        trigger_on_session_changed=True,
        trigger_on_genome_changed=True,
        trigger_on_genome_load_completed=True,
        trigger_on_back_online=True,
        trigger_on_brain_ready=False,
        trigger_on_transport_send_failed=True,
    )

    def _rebuild(reason: str) -> None:
        brain_output.reconnect(reason=reason)
        if after_reconnect is not None:
            after_reconnect()

    core = SessionSupervisor(
        monitor=monitor,
        reconnect=_rebuild,
        is_registered=brain_output.is_registered,
        slow_retry_interval_s=slow_retry_interval_s,
        on_state_change=on_state_change,
        initial_state=initial_state,
    )
    return BackgroundSessionSupervisor(core=core, poll_interval_s=health_poll_interval_s)


def create_session_supervisor_from_config(
    *,
    brain_output: Any,
    feagi_api_host: str,
    feagi_api_port: int,
    slow_retry_interval_s: float = DEFAULT_SLOW_RETRY_INTERVAL_S,
    on_state_change: Optional[Callable[[SessionRecoveryState], None]] = None,
    after_reconnect: Optional[Callable[[], None]] = None,
    initial_state: SessionRecoveryState = SessionRecoveryState.REGISTERED,
) -> BackgroundSessionSupervisor:
    """
    Same as :func:`create_session_supervisor`, with the four recovery values read from the
    ``[recovery]`` section of the FEAGI config file (``feagi.config.load_recovery_config``).
    """
    from feagi.config import load_recovery_config

    recovery = load_recovery_config()
    return create_session_supervisor(
        brain_output=brain_output,
        feagi_api_host=feagi_api_host,
        feagi_api_port=feagi_api_port,
        health_poll_interval_s=recovery.health_poll_interval_s,
        health_fetch_timeout_ms=recovery.health_fetch_timeout_ms,
        reconnect_cooldown_ms=recovery.reconnect_cooldown_ms,
        reconnect_max_failures=recovery.reconnect_max_failures,
        slow_retry_interval_s=slow_retry_interval_s,
        on_state_change=on_state_change,
        after_reconnect=after_reconnect,
        initial_state=initial_state,
    )


__all__ = [
    "DEFAULT_SLOW_RETRY_INTERVAL_S",
    "BackgroundSessionSupervisor",
    "PollResult",
    "SessionRecoveryState",
    "SessionSupervisor",
    "SupervisedTickReport",
    "SupervisorAction",
    "create_session_supervisor",
    "create_session_supervisor_from_config",
]
