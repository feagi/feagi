"""
Unit tests for RawIMU / SmartIMU SDK helpers.

Covers two surfaces:

* :mod:`feagi.pns.brain_output` -- ``register_sensor_units`` dispatch for
  ``RawIMU``/``SmartIMU`` plus the controller-facing ``write_sensor_raw_imu``
  and ``write_sensor_smart_imu`` methods.
* :mod:`feagi.pns.brain_input` -- the cortical-subtype dispatch table and the
  auto-discovery branch in ``register_cortical_areas_with_cache``.

Tests deliberately avoid importing the Rust extension. Each fake reproduces
exactly the surface the SDK calls, so a contract change in the Rust layer
(e.g. a renamed register method) will surface as a focused failure here
rather than as a runtime error in a controller.
"""

from __future__ import annotations

import base64
import sys
from types import SimpleNamespace
from typing import List, Tuple

import pytest

# NOTE: ``feagi.pns.__init__`` re-exports a *singleton instance* named
# ``brain_input`` which rebinds the ``feagi.pns.brain_input`` attribute on the
# package to the instance, shadowing the submodule. We therefore obtain the
# real module from ``sys.modules`` after triggering its import.
import feagi.pns.brain_input  # noqa: F401  (registers in sys.modules)
from feagi.pns.brain_input import _SUBTYPE_TO_SENSOR_REGISTER_CANDIDATES
from feagi.pns.brain_output import BrainOutput

brain_input_module = sys.modules["feagi.pns.brain_input"]


# ---------------------------------------------------------------------------
# Shared fake cache (covers register_sensor_units + IMU write helpers)
# ---------------------------------------------------------------------------


class _FakeCache:
    """Captures all sensor cache calls performed by ``BrainOutput``."""

    def __init__(self) -> None:
        self.calls: List[Tuple] = []

    # --- register_* surfaces ----------------------------------------------
    def sensor_RawIMU_register(self, group, count, frame_mode, z_res, positioning):
        self.calls.append(
            ("RawIMU_register", group, count, frame_mode, z_res, positioning)
        )

    def sensor_SmartIMU_register(self, group, count, frame_mode, z_res, positioning):
        self.calls.append(
            ("SmartIMU_register", group, count, frame_mode, z_res, positioning)
        )

    def sensor_Shock_register(self, group, count, frame_mode, z_res, positioning):
        self.calls.append(
            ("Shock_register", group, count, frame_mode, z_res, positioning)
        )

    # --- write_* surfaces -------------------------------------------------
    def sensor_raw_i_m_u_write(self, *, group, channel_index, data):
        self.calls.append(("raw_imu_write", group, channel_index, data))

    def sensor_raw_i_m_u_write_accelerometer(
        self, *, group, channel_index, accelerometer
    ):
        self.calls.append(
            ("raw_imu_write_accelerometer", group, channel_index, accelerometer)
        )

    def sensor_raw_i_m_u_write_gyroscope(self, *, group, channel_index, gyroscope):
        self.calls.append(
            ("raw_imu_write_gyroscope", group, channel_index, gyroscope)
        )

    def sensor_raw_i_m_u_write_magnetometer(
        self, *, group, channel_index, magnetometer
    ):
        self.calls.append(
            ("raw_imu_write_magnetometer", group, channel_index, magnetometer)
        )

    def sensor_smart_i_m_u_write(self, *, group, channel_index, data):
        self.calls.append(("smart_imu_write", group, channel_index, data))

    # Method names the IMU paths must NEVER touch (legacy regression guard):
    # the fake intentionally raises so a call surfaces immediately.
    def sensor_Accelerometer_register(self, *_args, **_kwargs):
        raise AssertionError(
            "sensor_Accelerometer_register must not be invoked: "
            "Accelerometer was migrated into RawIMU."
        )

    def sensor_Gyroscope_register(self, *_args, **_kwargs):
        raise AssertionError(
            "sensor_Gyroscope_register must not be invoked: "
            "Gyroscope was migrated into RawIMU."
        )


def _install_fake_frpl(monkeypatch) -> None:
    """
    Install a ``feagi_rust_py_libs`` shim sufficient for
    ``register_sensor_units``.

    Only the minimum surface used by the IMU paths is provided. ``Absolute``
    and ``Linear`` are returned as opaque sentinel strings so equality checks
    can confirm they were forwarded into the cache call without depending on
    the real Rust enum identity.
    """

    class _FrameChangeHandling:
        @staticmethod
        def Absolute():
            return "ABS"

    class _Positioning:
        @staticmethod
        def Linear():
            return "LIN"

    class _Descriptors:
        class ImageXYResolution:
            def __init__(self, x, y):
                self.x = x
                self.y = y

        class ColorSpace:
            Gamma = "Gamma"

        class ColorChannelLayout:
            RGB = "RGB"

        class ImageFrameProperties:
            def __init__(self, resolution, color_space, channel_layout):
                self.resolution = resolution
                self.color_space = color_space
                self.channel_layout = channel_layout

        class MiscDataDimensions:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z

    fake = SimpleNamespace(
        data_structures=SimpleNamespace(
            genomic=SimpleNamespace(
                cortical_area=SimpleNamespace(
                    FrameChangeHandling=_FrameChangeHandling,
                    PercentageNeuronPositioning=_Positioning,
                )
            )
        ),
        connector_core=SimpleNamespace(
            data_types=SimpleNamespace(
                descriptors=_Descriptors,
            )
        ),
    )
    monkeypatch.setitem(sys.modules, "feagi_rust_py_libs", fake)


def _make_brain_output_with_fake_cache() -> Tuple[BrainOutput, _FakeCache]:
    cache = _FakeCache()
    bo = BrainOutput()
    bo._cache = cache
    bo._cache_available = True
    return bo, cache


# ---------------------------------------------------------------------------
# brain_input: cortical subtype dispatch table
# ---------------------------------------------------------------------------


def test_subtype_table_routes_rim_and_sim_only() -> None:
    """
    ``rim``/``sim`` map to RawIMU/SmartIMU candidate methods, and the legacy
    ``acc``/``gyq`` subtypes have been intentionally removed (silent-drop
    migration policy).
    """
    rim_candidates = _SUBTYPE_TO_SENSOR_REGISTER_CANDIDATES.get(b"rim")
    sim_candidates = _SUBTYPE_TO_SENSOR_REGISTER_CANDIDATES.get(b"sim")

    assert rim_candidates is not None, "RawIMU subtype b'rim' must dispatch"
    assert sim_candidates is not None, "SmartIMU subtype b'sim' must dispatch"

    # Rust macro emits both snake_case and PascalCase candidates; both must be
    # tried so the SDK keeps working across Rust binding rebuilds.
    assert "sensor_raw_i_m_u_register" in rim_candidates
    assert "sensor_RawIMU_register" in rim_candidates
    assert "sensor_smart_i_m_u_register" in sim_candidates
    assert "sensor_SmartIMU_register" in sim_candidates

    assert b"acc" not in _SUBTYPE_TO_SENSOR_REGISTER_CANDIDATES
    assert b"gyq" not in _SUBTYPE_TO_SENSOR_REGISTER_CANDIDATES


# ---------------------------------------------------------------------------
# brain_input: register_cortical_areas_with_cache for rim/sim
# ---------------------------------------------------------------------------


class _AutoDiscoveryFakeCache:
    """Fake cache for the auto-discovery (brain_input) IMU branch."""

    def __init__(self) -> None:
        self.calls: List[Tuple] = []

    def sensor_raw_i_m_u_register(
        self,
        *,
        group,
        number_channels,
        frame_change_handling,
        z_neuron_resolution,
        percentage_neuron_positioning,
    ):
        self.calls.append(
            (
                "rim_register",
                group,
                number_channels,
                frame_change_handling,
                z_neuron_resolution,
                percentage_neuron_positioning,
            )
        )

    def sensor_smart_i_m_u_register(
        self,
        *,
        group,
        number_channels,
        frame_change_handling,
        z_neuron_resolution,
        percentage_neuron_positioning,
    ):
        self.calls.append(
            (
                "sim_register",
                group,
                number_channels,
                frame_change_handling,
                z_neuron_resolution,
                percentage_neuron_positioning,
            )
        )


def _make_cortical_id(subtype_bytes: bytes) -> str:
    """
    Build a base64 cortical ID matching the layout assumed by
    ``decode_cortical_id_to_subtype``: byte 0 = category, bytes 1-3 = subtype,
    bytes 4-7 = unit index padding.
    """
    if len(subtype_bytes) != 3:
        raise ValueError("subtype_bytes must be exactly 3 bytes")
    raw = b"i" + subtype_bytes + b"\x00\x00\x00\x00"
    return base64.b64encode(raw).decode("ascii")


def test_register_cortical_areas_dispatches_rim_and_sim(monkeypatch) -> None:
    """
    Auto-discovery routes ``rim`` and ``sim`` cortical IDs to their respective
    snake_case Rust register methods using the template defaults
    (``number_channels=1``, ``z_neuron_resolution=10``).
    """
    _install_fake_frpl(monkeypatch)
    cache = _AutoDiscoveryFakeCache()
    rim_id = _make_cortical_id(b"rim")
    sim_id = _make_cortical_id(b"sim")

    brain_input_module.register_cortical_areas_with_cache(cache, [rim_id, sim_id])

    rim_calls = [c for c in cache.calls if c[0] == "rim_register"]
    sim_calls = [c for c in cache.calls if c[0] == "sim_register"]
    assert len(rim_calls) == 1
    assert len(sim_calls) == 1

    # Each call must mirror the template defaults (see
    # feagi-core/.../templates/sensor_cortical_units.rs). Group IDs are
    # assigned by enumeration order over the discovered subtypes; assert
    # only the values the auto-discovery contract guarantees.
    for tag, _group, channels, frame, z_res, positioning in rim_calls + sim_calls:
        assert channels == 1, f"{tag} must register exactly 1 channel by default"
        assert frame == "ABS", f"{tag} must use Absolute frame change handling"
        assert z_res == 10, f"{tag} must mirror template z=10 default"
        assert positioning == "LIN", f"{tag} must use Linear neuron positioning"


def test_register_cortical_areas_skips_legacy_acc_gyq(monkeypatch, caplog) -> None:
    """
    Legacy ``acc``/``gyq`` subtypes (Accelerometer/Gyroscope) are no longer
    auto-registered. They must be logged as unknown subtypes (matching the
    silent-drop migration policy) and must not raise.
    """
    _install_fake_frpl(monkeypatch)
    cache = _AutoDiscoveryFakeCache()
    acc_id = _make_cortical_id(b"acc")
    gyq_id = _make_cortical_id(b"gyq")

    with caplog.at_level("WARNING", logger="feagi.pns.brain_input"):
        brain_input_module.register_cortical_areas_with_cache(cache, [acc_id, gyq_id])

    assert cache.calls == [], (
        "Legacy acc/gyq subtypes must not invoke any sensor register method "
        "(they are dropped by the migrator and are no longer first-class)."
    )
    warning_messages = [r.getMessage() for r in caplog.records]
    assert any("Unknown cortical subtype" in m for m in warning_messages), (
        "Auto-discovery must log a warning for unsupported subtypes"
    )


# ---------------------------------------------------------------------------
# brain_output.register_sensor_units: RawIMU + SmartIMU dispatch
# ---------------------------------------------------------------------------


def test_register_sensor_units_dispatches_raw_and_smart_imu(monkeypatch) -> None:
    """
    Sorted dispatch from ``register_sensor_units`` calls
    ``sensor_RawIMU_register`` and ``sensor_SmartIMU_register`` with the
    expected ``(group, count, frame, z_res, positioning)`` payload.
    """
    _install_fake_frpl(monkeypatch)
    bo, cache = _make_brain_output_with_fake_cache()

    groups = bo.register_sensor_units(
        {"RawIMU": 2, "SmartIMU": 1},
        z_neuron_resolution=10,
    )

    # Sorted alphabetical: RawIMU -> 0, SmartIMU -> 1.
    assert groups == {"RawIMU": 0, "SmartIMU": 1}

    register_calls = [c for c in cache.calls if c[0].endswith("_register")]
    assert ("RawIMU_register", 0, 2, "ABS", 10, "LIN") in register_calls
    assert ("SmartIMU_register", 1, 1, "ABS", 10, "LIN") in register_calls


def test_register_sensor_units_does_not_couple_imu_with_shock(monkeypatch) -> None:
    """
    Regression: registering RawIMU alongside Shock must produce two
    independent registrations. Shock continues to use its own group; the IMU
    registration must NOT route through the Shock path under any circumstance.
    """
    _install_fake_frpl(monkeypatch)
    bo, cache = _make_brain_output_with_fake_cache()

    groups = bo.register_sensor_units(
        {"RawIMU": 1, "Shock": 1},
        z_neuron_resolution=10,
    )

    assert "RawIMU" in groups and "Shock" in groups
    assert groups["RawIMU"] != groups["Shock"], (
        "RawIMU and Shock must be assigned to distinct cache groups"
    )
    register_kinds = {c[0] for c in cache.calls if c[0].endswith("_register")}
    assert {"RawIMU_register", "Shock_register"}.issubset(register_kinds)


def test_register_sensor_units_rejects_legacy_unit_names(monkeypatch) -> None:
    """
    The legacy ``Accelerometer``/``Gyroscope`` keys are no longer accepted by
    the SDK -- supplying them must raise ``ValueError`` rather than silently
    fall through to a no-op.
    """
    _install_fake_frpl(monkeypatch)
    bo, _cache = _make_brain_output_with_fake_cache()

    for legacy_name in ("Accelerometer", "Gyroscope"):
        with pytest.raises(ValueError, match=legacy_name):
            bo.register_sensor_units(
                {legacy_name: 1},
                z_neuron_resolution=10,
            )


# ---------------------------------------------------------------------------
# brain_output write helpers: RawIMU + SmartIMU
# ---------------------------------------------------------------------------


class _FakeRawImuFactory:
    """Capture-only stand-in for the Rust ``RawIMU`` constructor."""

    @staticmethod
    def try_from_axis_triples(accel, gyro, mag):
        return ("RawIMU", tuple(accel), tuple(gyro), tuple(mag))


class _FakeSignedPercentage:
    @staticmethod
    def new_from_m1_1(value: float):
        return ("SP", float(value))


class _FakeSignedPercentage4D:
    def __init__(self, w, x, y, z):
        # Stored as a plain tuple so test assertions are deterministic.
        self.payload = ("SP4D", w, x, y, z)


class _FakeSignedPercentage3D:
    """
    Capture-only stand-in for the Rust ``SignedPercentage3D`` constructor.

    Stores its components as a tuple so partial-write tests can assert that
    the SDK forwarded the expected axis values without depending on the real
    Rust pyo3 wrapper class.
    """

    def __init__(self, a, b, c):
        self.payload = ("SP3D", a, b, c)


def _install_imu_factories(bo: BrainOutput) -> None:
    """
    Install fake IMU factories on a ``BrainOutput`` and short-circuit the
    lazy-init helper so the Rust extension is never imported during tests.
    """
    bo._sensory_signed_percentage_factory = _FakeSignedPercentage
    bo._sensory_signed_percentage_3d_factory = _FakeSignedPercentage3D
    bo._sensory_signed_percentage_4d_factory = _FakeSignedPercentage4D
    bo._sensory_raw_imu_factory = _FakeRawImuFactory
    bo._init_imu_write_helpers = lambda: None  # type: ignore[method-assign]


def test_write_sensor_raw_imu_routes_to_composite_write() -> None:
    """
    ``write_sensor_raw_imu`` must build a single ``RawIMU`` composite from
    three axis triples and forward it (unchanged) to the cache's
    ``sensor_raw_i_m_u_write`` keyword API.
    """
    bo, cache = _make_brain_output_with_fake_cache()
    _install_imu_factories(bo)

    bo.write_sensor_raw_imu(
        group=0,
        channel_index=2,
        accelerometer_xyz=(0.1, -0.2, 0.3),
        gyroscope_xyz=(0.4, -0.5, 0.6),
        magnetometer_xyz=(-0.7, 0.8, -0.9),
    )

    raw_writes = [c for c in cache.calls if c[0] == "raw_imu_write"]
    assert len(raw_writes) == 1, "RawIMU write must call the composite cache method exactly once"
    _, group, channel, data = raw_writes[0]
    assert group == 0
    assert channel == 2
    assert data == (
        "RawIMU",
        (0.1, -0.2, 0.3),
        (0.4, -0.5, 0.6),
        (-0.7, 0.8, -0.9),
    )

    # The Rust template encodes accel/gyro/mag into three sub-areas at burst
    # time, NOT through three separate cache writes. A regression where the
    # SDK splits the composite back into three writes would silently break the
    # encoder and is guarded against here.
    assert all(c[0] != "smart_imu_write" for c in cache.calls)


def test_write_sensor_raw_imu_requires_initialised_cache() -> None:
    """Calling write helpers before ``configure()`` must raise, not no-op."""
    bo = BrainOutput()
    with pytest.raises(RuntimeError, match="ConnectorAgent cache"):
        bo.write_sensor_raw_imu(
            group=0,
            channel_index=0,
            accelerometer_xyz=(0.0, 0.0, 0.0),
            gyroscope_xyz=(0.0, 0.0, 0.0),
            magnetometer_xyz=(0.0, 0.0, 0.0),
        )


def test_write_sensor_smart_imu_routes_to_quaternion_write() -> None:
    """
    ``write_sensor_smart_imu`` wraps each quaternion component as
    ``SignedPercentage`` and constructs a ``SignedPercentage4D``, then
    forwards via ``sensor_smart_i_m_u_write``.

    Convention asserted: ``(w, x, y, z)`` -> ``SignedPercentage4D(w, x, y, z)``.
    """
    bo, cache = _make_brain_output_with_fake_cache()
    _install_imu_factories(bo)

    bo.write_sensor_smart_imu(
        group=1,
        channel_index=0,
        quaternion_wxyz=(0.7071, 0.0, 0.7071, 0.0),
    )

    quat_writes = [c for c in cache.calls if c[0] == "smart_imu_write"]
    assert len(quat_writes) == 1
    _, group, channel, data = quat_writes[0]
    assert group == 1
    assert channel == 0
    assert isinstance(data, _FakeSignedPercentage4D)
    assert data.payload == (
        "SP4D",
        ("SP", 0.7071),
        ("SP", 0.0),
        ("SP", 0.7071),
        ("SP", 0.0),
    )

    # SmartIMU is a single sub-area unit; it must never accidentally use the
    # RawIMU composite write API.
    assert all(c[0] != "raw_imu_write" for c in cache.calls)


def test_write_sensor_smart_imu_requires_initialised_cache() -> None:
    bo = BrainOutput()
    with pytest.raises(RuntimeError, match="ConnectorAgent cache"):
        bo.write_sensor_smart_imu(
            group=0,
            channel_index=0,
            quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        )


# ---------------------------------------------------------------------------
# brain_output partial-write helpers: per-axis RawIMU writes
# ---------------------------------------------------------------------------
#
# These tests cover the SDK side of the cache's read-modify-write API. They
# are the SDK-level guard for the design decision: a controller exposing
# only some IMU axes (e.g. accel + gyro, no magnetometer) must be able to
# update the present axes without forcing the missing one to zero each
# tick. The composite ``write_sensor_raw_imu`` would impose that implicit
# fallback, which is forbidden by project policy without explicit
# permission. The dedicated cache method names asserted here MUST match the
# Rust pyo3 binding output (``sensor_raw_i_m_u_write_<axis>``); a rename on
# the Rust side would surface as a focused failure here.


def test_write_sensor_raw_imu_accelerometer_routes_to_partial_write() -> None:
    """
    ``write_sensor_raw_imu_accelerometer`` constructs a SignedPercentage3D
    from the supplied triple and forwards it via the dedicated partial-write
    cache method, leaving gyro/mag completely untouched.
    """
    bo, cache = _make_brain_output_with_fake_cache()
    _install_imu_factories(bo)

    bo.write_sensor_raw_imu_accelerometer(
        group=0,
        channel_index=3,
        accelerometer_xyz=(0.1, -0.2, 0.3),
    )

    accel_writes = [c for c in cache.calls if c[0] == "raw_imu_write_accelerometer"]
    assert len(accel_writes) == 1
    _, group, channel, triple = accel_writes[0]
    assert group == 0
    assert channel == 3
    assert isinstance(triple, _FakeSignedPercentage3D)
    assert triple.payload == (
        "SP3D",
        ("SP", 0.1),
        ("SP", -0.2),
        ("SP", 0.3),
    )

    # Partial accelerometer write must NOT also fan out to the gyro or mag
    # cache methods, and must NOT issue a whole-composite write either; that
    # would defeat the read-modify-write contract.
    other_kinds = {c[0] for c in cache.calls} - {"raw_imu_write_accelerometer"}
    assert other_kinds == set(), (
        f"partial accel write must not touch other axes, got: {other_kinds}"
    )


def test_write_sensor_raw_imu_gyroscope_routes_to_partial_write() -> None:
    bo, cache = _make_brain_output_with_fake_cache()
    _install_imu_factories(bo)

    bo.write_sensor_raw_imu_gyroscope(
        group=2,
        channel_index=0,
        gyroscope_xyz=(0.4, 0.5, 0.6),
    )

    writes = [c for c in cache.calls if c[0] == "raw_imu_write_gyroscope"]
    assert len(writes) == 1
    _, group, channel, triple = writes[0]
    assert group == 2
    assert channel == 0
    assert triple.payload == (
        "SP3D",
        ("SP", 0.4),
        ("SP", 0.5),
        ("SP", 0.6),
    )
    assert {c[0] for c in cache.calls} == {"raw_imu_write_gyroscope"}


def test_write_sensor_raw_imu_magnetometer_routes_to_partial_write() -> None:
    bo, cache = _make_brain_output_with_fake_cache()
    _install_imu_factories(bo)

    bo.write_sensor_raw_imu_magnetometer(
        group=2,
        channel_index=1,
        magnetometer_xyz=(-0.7, 0.8, -0.9),
    )

    writes = [c for c in cache.calls if c[0] == "raw_imu_write_magnetometer"]
    assert len(writes) == 1
    _, group, channel, triple = writes[0]
    assert group == 2
    assert channel == 1
    assert triple.payload == (
        "SP3D",
        ("SP", -0.7),
        ("SP", 0.8),
        ("SP", -0.9),
    )
    assert {c[0] for c in cache.calls} == {"raw_imu_write_magnetometer"}


def test_partial_write_helpers_require_initialised_cache() -> None:
    """
    Every partial-write helper must raise the same ``RuntimeError`` as the
    composite write when the cache is not configured. Silent acceptance
    would let a controller silently drop IMU samples.
    """
    bo = BrainOutput()

    expected_msg = "ConnectorAgent cache"
    with pytest.raises(RuntimeError, match=expected_msg):
        bo.write_sensor_raw_imu_accelerometer(
            group=0,
            channel_index=0,
            accelerometer_xyz=(0.0, 0.0, 0.0),
        )
    with pytest.raises(RuntimeError, match=expected_msg):
        bo.write_sensor_raw_imu_gyroscope(
            group=0,
            channel_index=0,
            gyroscope_xyz=(0.0, 0.0, 0.0),
        )
    with pytest.raises(RuntimeError, match=expected_msg):
        bo.write_sensor_raw_imu_magnetometer(
            group=0,
            channel_index=0,
            magnetometer_xyz=(0.0, 0.0, 0.0),
        )


def test_partial_write_helpers_do_not_mutate_other_axes_in_sequence() -> None:
    """
    Sequencing accel -> gyro -> mag partial writes must produce three
    independent cache calls with their respective axis payloads. This
    locks in that the SDK does not accidentally batch updates or pollute
    the next call with state from the previous one.
    """
    bo, cache = _make_brain_output_with_fake_cache()
    _install_imu_factories(bo)

    bo.write_sensor_raw_imu_accelerometer(
        group=0, channel_index=0, accelerometer_xyz=(0.1, 0.2, 0.3)
    )
    bo.write_sensor_raw_imu_gyroscope(
        group=0, channel_index=0, gyroscope_xyz=(-0.4, -0.5, -0.6)
    )
    bo.write_sensor_raw_imu_magnetometer(
        group=0, channel_index=0, magnetometer_xyz=(0.7, -0.8, 0.9)
    )

    kinds = [c[0] for c in cache.calls]
    assert kinds == [
        "raw_imu_write_accelerometer",
        "raw_imu_write_gyroscope",
        "raw_imu_write_magnetometer",
    ]
    accel_payload = cache.calls[0][3].payload
    gyro_payload = cache.calls[1][3].payload
    mag_payload = cache.calls[2][3].payload
    assert accel_payload[1:] == (("SP", 0.1), ("SP", 0.2), ("SP", 0.3))
    assert gyro_payload[1:] == (("SP", -0.4), ("SP", -0.5), ("SP", -0.6))
    assert mag_payload[1:] == (("SP", 0.7), ("SP", -0.8), ("SP", 0.9))
