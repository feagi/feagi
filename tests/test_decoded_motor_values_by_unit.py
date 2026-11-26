"""Decoded motor values are keyed by unit, so units sharing a group do not collide."""

import pytest

from feagi.pns.brain_output import BrainOutput

_DESCRIPTOR_B64 = "A" * 64


class _Client:
    def __init__(self, frames):
        self._frames = list(frames)

    def receive_motor_data_raw(self):
        return self._frames.pop(0) if self._frames else b""


class _SnapshotCache:
    """Real registered cache; only the byte decode and snapshot read are scripted."""

    def __init__(self, real, snapshots):
        self._real = real
        self._snapshots = list(snapshots)
        self._current = []

    def __getattr__(self, name):
        return getattr(self._real, name)

    def motors_load_in_bytes_and_verify(self, raw):
        pass

    def motors_decode_cached_byte_data_to_motor(self):
        self._current = self._snapshots.pop(0)

    def motors_read_decoded_snapshot(self, updated_only=False):
        assert updated_only is True
        return list(self._current)


@pytest.fixture
def bo(monkeypatch):
    monkeypatch.setenv("FEAGI_AGENT_DESCRIPTOR_B64", _DESCRIPTOR_B64)
    out = BrainOutput()
    out.register_motor_groups(
        {0: {"positional_servo": ["s0", "s1"], "rotary_motor": ["r0", "r1"]}},
        z_neuron_resolution=10,
    )
    out._connected = True
    out._sensory_only_mode = False
    return out


def _wire(bo, frames, snapshots):
    bo._client = _Client(frames)
    bo._cache = _SnapshotCache(bo._cache, snapshots)


def test_servo_and_rotary_in_same_frame_do_not_overwrite(bo):
    _wire(
        bo,
        [b"f"],
        [
            [
                ("positional_servo", 0, 0, "absolute", 0.9),
                ("positional_servo", 0, 1, "absolute", 0.1),
                ("rotary_motor", 0, 0, "absolute", -0.5),
                ("rotary_motor", 0, 1, "absolute", 0.25),
            ]
        ],
    )
    bo.receive()
    assert bo.read_decoded_motor_values("positional_servo", 0) == {
        (0, 0, "absolute"): 0.9,
        (0, 1, "absolute"): 0.1,
    }
    assert bo.read_decoded_motor_values("rotary_motor", 0) == {
        (0, 0, "absolute"): -0.5,
        (0, 1, "absolute"): 0.25,
    }
    assert bo.read_rotary_motor_targets(0, mode="tick") == {0: -0.5, 1: 0.25}
    assert bo.read_decoded_motor_values("rotary_motor", 3) == {}


def test_empty_tick_clears_values(bo):
    _wire(bo, [b"f"], [[("rotary_motor", 0, 0, "absolute", 0.5)]])
    bo.receive()
    assert bo.read_decoded_motor_values("rotary_motor")
    bo.receive()
    assert bo.read_decoded_motor_values("rotary_motor") == {}


def test_drain_merges_frames_and_sums_incremental(bo):
    seen = []
    _wire(
        bo,
        [b"a", b"b"],
        [
            [
                ("rotary_motor", 0, 0, "absolute", 0.1),
                ("spatial_pointer", 1, 0, "incremental", 0.1),
            ],
            [
                ("positional_servo", 0, 0, "absolute", 0.7),
                ("spatial_pointer", 1, 0, "incremental", 0.2),
            ],
        ],
    )
    frames = bo.drain_motor_frames(10, on_frame=seen.append)
    assert frames == 2
    assert seen == [b"a", b"b"]
    assert bo._last_motor_frame == b"b"
    assert bo.read_decoded_motor_values("rotary_motor") == {(0, 0, "absolute"): 0.1}
    assert bo.read_decoded_motor_values("positional_servo") == {(0, 0, "absolute"): 0.7}
    pointer = bo.read_decoded_motor_values("spatial_pointer")
    assert pointer[(1, 0, "incremental")] == pytest.approx(0.3)


def test_drain_with_no_frames_returns_zero(bo):
    _wire(bo, [], [])
    assert bo.drain_motor_frames(5) == 0
    assert bo._last_motor_frame is None
