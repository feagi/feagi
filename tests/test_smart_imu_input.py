"""Unit tests for SmartImu brain_input wiring."""

from __future__ import annotations

from typing import Any, List, Tuple

import pytest

pytest.importorskip("feagi_rust_py_libs")

from feagi.pns.inputs.smart_imu import SmartImu, _normalize_quaternion_xyzw


class _FakeCacheRegister:
    """Captures SmartIMU register + write calls."""

    def __init__(self) -> None:
        self.registers: List[Tuple[str, dict[str, Any]]] = []
        self.writes: List[Tuple[str, dict[str, Any]]] = []

    def sensor_SmartIMU_register(self, **kwargs: Any) -> None:
        self.registers.append(("sensor_SmartIMU_register", dict(kwargs)))

    def sensor_smart_i_m_u_write(self, **kwargs: Any) -> None:
        self.writes.append(("sensor_smart_i_m_u_write", dict(kwargs)))


@pytest.mark.parametrize(
    ("xyzw", "expected_wxyz"),
    [
        ((0.0, 0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0)),
        ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0)),
    ],
)
def test_normalize_quaternion_xyzw(
    xyzw: tuple[float, float, float, float],
    expected_wxyz: tuple[float, float, float, float],
) -> None:
    x, y, z, w = xyzw
    got = _normalize_quaternion_xyzw(x, y, z, w)
    assert got is not None
    assert got == pytest.approx(expected_wxyz)


def test_normalize_quaternion_rejects_zero_norm() -> None:
    assert _normalize_quaternion_xyzw(0.0, 0.0, 0.0, 0.0) is None


def test_smart_imu_register_calls_cache(monkeypatch: pytest.MonkeyPatch) -> None:
    fake = _FakeCacheRegister()

    def fake_register_input(inp: SmartImu, group_id: int | None = None) -> None:
        gid = 7 if group_id is None else int(group_id)
        inp._register_with_cache(fake, gid)
        inp._mark_registered(gid)

    monkeypatch.setattr(
        "feagi.pns.brain_input.brain_input.register_input",
        fake_register_input,
    )

    SmartImu.register(group_id=7, channel_index=0)
    assert len(fake.registers) == 1
    assert fake.registers[0][1]["group"] == 7
    assert fake.registers[0][1]["number_channels"] == 1


def test_smart_imu_channel_index_expands_number_channels(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake = _FakeCacheRegister()

    def fake_register_input(inp: SmartImu, group_id: int | None = None) -> None:
        gid = int(group_id) if group_id is not None else 0
        inp._register_with_cache(fake, gid)
        inp._mark_registered(gid)

    monkeypatch.setattr(
        "feagi.pns.brain_input.brain_input.register_input",
        fake_register_input,
    )
    SmartImu.register(group_id=3, channel_index=2)
    assert fake.registers[0][1]["number_channels"] == 3
