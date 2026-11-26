"""Tests for BrainOutput.read_rotary_motor_targets (held and tick modes)."""

import pytest

from feagi.pns.brain_output import BrainOutput

_DESCRIPTOR_B64 = "A" * 64


@pytest.fixture
def bo(monkeypatch):
    monkeypatch.setenv("FEAGI_AGENT_DESCRIPTOR_B64", _DESCRIPTOR_B64)
    out = BrainOutput()
    out.register_motor_groups(
        {0: {"rotary_motor": ["a", "b", "c"]}}, z_neuron_resolution=10
    )
    return out


def _set_cache_values(bo, values):
    """Make the real cache report given signed values (wraps the read binding only)."""
    real = bo._cache
    real_read = real.motor_rotary_motor_read_postprocessed_cache_value

    class _V:
        def __init__(self, v):
            self.v = v

        def get_as_m1_1(self):
            return self.v

    class _Cache:
        def __getattr__(self, name):
            return getattr(real, name)

        def motor_rotary_motor_read_postprocessed_cache_value(self, g, c):
            real_read(g, c)  # real registration and bounds checks
            return _V(values[c])

    bo._cache = _Cache()


def test_held_returns_signed_values(bo):
    _set_cache_values(bo, {0: -0.5, 1: 0.75, 2: 0.0})
    assert bo.read_rotary_motor_targets(0) == {0: -0.5, 1: 0.75, 2: 0.0}


def test_held_default_on_real_cache(bo):
    assert bo.read_rotary_motor_targets(0) == {0: 0.0, 1: 0.0, 2: 0.0}


def test_tick_zeroes_channels_absent_from_frame(bo):
    _set_cache_values(bo, {0: -0.5, 1: 0.75, 2: 0.3})
    bo._decoded_motor_values = {
        ("rotary_motor", 0, 1, "absolute"): 0.75,
        ("rotary_motor", 5, 0, "absolute"): 1.0,
        ("positional_servo", 0, 0, "absolute"): 0.9,
    }
    assert bo.read_rotary_motor_targets(0, mode="tick") == {0: 0.0, 1: 0.75, 2: 0.0}
    # held still reports the cache
    assert bo.read_rotary_motor_targets(0)[0] == -0.5


def test_tick_after_empty_receive_is_all_zero(bo):
    class _Client:
        def receive_motor_data_raw(self):
            return b""

    _set_cache_values(bo, {0: -0.5, 1: 0.75, 2: 0.3})
    bo._connected = True
    bo._sensory_only_mode = False
    bo._client = _Client()
    bo._decoded_motor_values = {("rotary_motor", 0, 0, "absolute"): -0.5}
    bo.receive()
    assert bo.read_rotary_motor_targets(0, mode="tick") == {0: 0.0, 1: 0.0, 2: 0.0}


def test_unregistered_group_raises(bo):
    with pytest.raises(ValueError):
        bo.read_rotary_motor_targets(7)
    with pytest.raises(ValueError):
        bo.read_rotary_motor_targets(7, mode="tick")


def test_bad_mode_raises(bo):
    with pytest.raises(ValueError, match="mode"):
        bo.read_rotary_motor_targets(0, mode="latest")


def test_tick_ignores_servo_sharing_the_group(monkeypatch):
    monkeypatch.setenv("FEAGI_AGENT_DESCRIPTOR_B64", _DESCRIPTOR_B64)
    out = BrainOutput()
    out.register_motor_groups(
        {0: {"rotary_motor": ["a"], "positional_servo": ["s"]}},
        z_neuron_resolution=10,
    )
    assert out.read_rotary_motor_targets(0) == {0: 0.0}
    out._decoded_motor_values = {
        ("positional_servo", 0, 0, "absolute"): 0.9,
        ("rotary_motor", 0, 0, "absolute"): -0.4,
    }
    assert out.read_rotary_motor_targets(0, mode="tick") == {0: -0.4}
