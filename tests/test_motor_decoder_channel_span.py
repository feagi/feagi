"""Motor decoder registration must allocate max(channel_index)+1 slots per group."""

import pytest

from feagi.pns.brain_output import BrainOutput
from feagi.pns.outputs.motor import RotaryMotor


def test_rotary_decoder_uses_max_channel_index_plus_one() -> None:
    """One output on I/O channel 1 must register 2 decoder channels (0 and 1)."""
    recorded: list[tuple[int, int]] = []

    class _FakeCache:
        def motor_positional_servo_register(self, *_a, **_kw) -> None:
            raise AssertionError("unexpected servo register")

        def motor_rotary_motor_register(self, group_id: int, count: int, *_rest, **_kw) -> None:
            recorded.append((int(group_id), int(count)))

        def register_callback(self, *_a, **_kw) -> None:
            pass

    bo = BrainOutput()
    bo._cache = _FakeCache()
    motor = RotaryMotor(unit_id=0, channel_index=1)
    motor.group_id = 0
    motor.channel = 1
    bo._outputs = [motor]
    bo._register_motor_decoder()
    assert recorded == [(0, 2)]


def test_two_rotaries_same_group_still_two_channels_when_zero_and_one() -> None:
    recorded: list[tuple[int, int]] = []

    class _FakeCache:
        def motor_positional_servo_register(self, *_a, **_kw) -> None:
            raise AssertionError("unexpected servo register")

        def motor_rotary_motor_register(self, group_id: int, count: int, *_rest, **_kw) -> None:
            recorded.append((int(group_id), int(count)))

        def register_callback(self, *_a, **_kw) -> None:
            pass

    bo = BrainOutput()
    bo._cache = _FakeCache()
    left = RotaryMotor(unit_id=0, channel_index=0)
    right = RotaryMotor(unit_id=0, channel_index=1)
    left.group_id = right.group_id = 0
    left.channel = 0
    right.channel = 1
    bo._outputs = [left, right]
    bo._register_motor_decoder()
    assert recorded == [(0, 2)]


def test_feagi_registration_output_count_uses_channel_span() -> None:
    """FEAGI output_count must not be only the number of motor objects (see ROS bridge ch 1)."""
    bo = BrainOutput()
    motor = RotaryMotor(unit_id=0, channel_index=1)
    motor.group_id = 0
    motor.channel = 1
    bo._outputs = [motor]
    assert bo._motor_feagi_output_count() == 2


def test_rotary_incremental_registers_incremental_frame() -> None:
    """Rust motor_rotary_motor_register must receive Incremental when encoding is incremental."""
    pytest.importorskip("feagi_rust_py_libs")
    recorded: list[object] = []

    class _FakeCache:
        def motor_positional_servo_register(self, *_a, **_kw) -> None:
            raise AssertionError("unexpected servo register")

        def motor_rotary_motor_register(
            self,
            _group_id: int,
            _count: int,
            frame_mode: object,
            *_rest: object,
            **_kw: object,
        ) -> None:
            recorded.append(frame_mode)

        def register_callback(self, *_a, **_kw) -> None:
            pass

    bo = BrainOutput()
    bo._cache = _FakeCache()
    motor = RotaryMotor(encoding="incremental", unit_id=0, channel_index=0)
    motor.group_id = 0
    motor.channel = 0
    bo._outputs = [motor]
    bo._register_motor_decoder()
    assert len(recorded) == 1
    assert "Incremental" in type(recorded[0]).__name__


def test_mixed_rotary_absolute_incremental_same_group_raises() -> None:
    pytest.importorskip("feagi_rust_py_libs")

    class _FakeCache:
        def motor_positional_servo_register(self, *_a, **_kw) -> None:
            pass

        def motor_rotary_motor_register(self, *_a, **_kw) -> None:
            pass

        def register_callback(self, *_a, **_kw) -> None:
            pass

    bo = BrainOutput()
    bo._cache = _FakeCache()
    left = RotaryMotor(encoding="absolute", unit_id=0, channel_index=0)
    right = RotaryMotor(encoding="incremental", unit_id=0, channel_index=1)
    left.group_id = right.group_id = 0
    left.channel = 0
    right.channel = 1
    bo._outputs = [left, right]
    with pytest.raises(RuntimeError, match="mix absolute and incremental"):
        bo._register_motor_decoder()
