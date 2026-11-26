"""Unit tests for the signed ``Force`` sensory unit in ``BrainOutput``."""

from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

from feagi.pns.brain_output import BrainOutput


class _FakeSigned:
    def __init__(self, value: float) -> None:
        self.value = value

    @staticmethod
    def new_from_m1_1(value: float) -> "_FakeSigned":
        return _FakeSigned(value)


class _FakeCache:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def sensor_Force_register(self, group, count, frame_mode, z_res, positioning):
        self.calls.append(("register", group, count, frame_mode, z_res, positioning))

    def sensor_force_write(self, *, group, channel_index, data):
        self.calls.append(("write", group, channel_index, data.value))


def _install_fake_frpl(monkeypatch) -> None:
    class _Frame:
        @staticmethod
        def Absolute():
            return "ABS"

        @staticmethod
        def Incremental():
            return "INC"

    class _Positioning:
        @staticmethod
        def Linear():
            return "LIN"

        @staticmethod
        def Fractional():
            return "FRAC"

    class _Descriptors:
        class ImageXYResolution:
            def __init__(self, x, y):
                pass

        class ColorSpace:
            Gamma = "Gamma"

        class ColorChannelLayout:
            RGB = "RGB"

        class ImageFrameProperties:
            def __init__(self, *args):
                pass

        class MiscDataDimensions:
            def __init__(self, x, y, z):
                pass

    fake = SimpleNamespace(
        data_structures=SimpleNamespace(
            genomic=SimpleNamespace(
                cortical_area=SimpleNamespace(
                    FrameChangeHandling=_Frame,
                    PercentageNeuronPositioning=_Positioning,
                )
            )
        ),
        connector_core=SimpleNamespace(
            data_types=SimpleNamespace(
                descriptors=_Descriptors,
                SignedPercentage=_FakeSigned,
                SignedPercentage3D=object,
                SignedPercentage4D=object,
                RawIMU=object,
            )
        ),
    )
    monkeypatch.setitem(sys.modules, "feagi_rust_py_libs", fake)


def _make():
    cache = _FakeCache()
    bo = BrainOutput()
    bo._cache = cache
    bo._cache_available = True
    return bo, cache


def test_register_force_defaults_to_linear(monkeypatch) -> None:
    _install_fake_frpl(monkeypatch)
    bo, cache = _make()
    groups = bo.register_sensor_units({"Force": 3}, z_neuron_resolution=20)
    assert groups == {"Force": 0}
    assert cache.calls == [("register", 0, 3, "ABS", 20, "LIN")]


def test_register_force_fractional(monkeypatch) -> None:
    _install_fake_frpl(monkeypatch)
    bo, cache = _make()
    bo.register_sensor_units(
        {"Force": 2},
        z_neuron_resolution=10,
        percentage_neuron_positioning="fractional",
    )
    assert cache.calls[0][-1] == "FRAC"


def test_register_rejects_unknown_positioning(monkeypatch) -> None:
    _install_fake_frpl(monkeypatch)
    bo, _ = _make()
    with pytest.raises(ValueError):
        bo.register_sensor_units(
            {"Force": 1},
            z_neuron_resolution=10,
            percentage_neuron_positioning="diagonal",
        )


@pytest.mark.parametrize("value", [-1.0, 0.0, 0.5, 1.0])
def test_signed_write_passes_value_through(monkeypatch, value) -> None:
    _install_fake_frpl(monkeypatch)
    bo, cache = _make()
    bo.write_sensor_signed_scalar(
        unit_key="Force", group=0, channel_index=2, scalar_neg1_1=value
    )
    assert cache.calls == [("write", 0, 2, value)]


@pytest.mark.parametrize("value", [1.5, -1.5, float("nan")])
def test_signed_write_rejects_out_of_range(monkeypatch, value) -> None:
    _install_fake_frpl(monkeypatch)
    bo, cache = _make()
    with pytest.raises(ValueError):
        bo.write_sensor_signed_scalar(
            unit_key="Force", group=0, channel_index=0, scalar_neg1_1=value
        )
    assert cache.calls == []


def test_signed_write_rejects_unknown_unit(monkeypatch) -> None:
    _install_fake_frpl(monkeypatch)
    bo, _ = _make()
    with pytest.raises(ValueError):
        bo.write_sensor_signed_scalar(
            unit_key="Servo", group=0, channel_index=0, scalar_neg1_1=0.1
        )
