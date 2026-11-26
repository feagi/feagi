"""Motor decoder registration must allocate max(channel_index)+1 slots per group."""

import pytest

from feagi.pns.brain_output import BrainOutput
from feagi.pns.outputs.motor import (
    ABSOLUTE_TARGET_INCREMENTAL_SPEED,
    RotaryMotor,
    ServoMotor,
)


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
    assert str(recorded[0]) == "Incremental"


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


def test_positional_servo_registers_configured_z_neuron_resolution() -> None:
    """A positional-servo group must use the depth configured by its device."""
    pytest.importorskip("feagi_rust_py_libs")
    recorded: list[int] = []

    class _FakeCache:
        def motor_positional_servo_register(
            self,
            _group_id: int,
            _count: int,
            _frame_mode: object,
            z_neuron_resolution: int,
            _positioning: object,
        ) -> None:
            recorded.append(z_neuron_resolution)

        def motor_rotary_motor_register(self, *_a, **_kw) -> None:
            raise AssertionError("unexpected rotary register")

        def register_callback(self, *_a, **_kw) -> None:
            pass

    bo = BrainOutput()
    bo._cache = _FakeCache()
    servo = ServoMotor(unit_id=0, channel_index=0, z_neuron_resolution=13)
    servo.group_id = 0
    servo.channel = 0
    bo._outputs = [servo]
    bo._register_motor_decoder()
    assert recorded == [13]


def test_positional_servo_group_rejects_mixed_z_neuron_resolutions() -> None:
    """Every positional-servo channel in a motor group must share one depth."""
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
    first = ServoMotor(unit_id=0, channel_index=0, z_neuron_resolution=10)
    second = ServoMotor(unit_id=0, channel_index=1, z_neuron_resolution=11)
    first.group_id = second.group_id = 0
    first.channel = 0
    second.channel = 1
    bo._outputs = [first, second]
    with pytest.raises(RuntimeError, match="different .*z-neuron resolutions"):
        bo._register_motor_decoder()


def test_target_speed_decoder_registers_incremental_step_from_servo() -> None:
    """Target-speed registration must pass the configured incremental step."""
    pytest.importorskip("feagi_rust_py_libs")
    recorded: list[tuple[int, int, float, list[float]]] = []

    class _FakeCache:
        def motor_positional_servo_register(self, *_a, **_kw) -> None:
            raise AssertionError("unexpected legacy servo register")

        def motor_positional_servo_target_speed_register(
            self,
            group_id: int,
            count: int,
            _absolute_z: int,
            _incremental_z: int,
            _positioning: object,
            default_speeds: list[float],
            incremental_step_0_1: float,
        ) -> None:
            recorded.append(
                (int(group_id), int(count), float(incremental_step_0_1), list(default_speeds))
            )

        def motor_rotary_motor_register(self, *_a, **_kw) -> None:
            raise AssertionError("unexpected rotary register")

        def register_callback(self, *_a, **_kw) -> None:
            pass

    bo = BrainOutput()
    bo._cache = _FakeCache()
    servo = ServoMotor(unit_id=0, channel_index=0, z_neuron_resolution=10)
    servo.group_id = 0
    servo.channel = 0
    servo.control_semantics = ABSOLUTE_TARGET_INCREMENTAL_SPEED
    servo.default_speed_0_1 = 0.35
    servo.incremental_step_ratio = 0.006
    bo._outputs = [servo]
    bo._register_motor_decoder()
    assert recorded == [(0, 1, 0.006, [0.35])]


def test_target_speed_group_rejects_mixed_incremental_steps() -> None:
    """All target-speed servos in one group must share one incremental step."""
    pytest.importorskip("feagi_rust_py_libs")

    class _FakeCache:
        def motor_positional_servo_register(self, *_a, **_kw) -> None:
            raise AssertionError("unexpected legacy servo register")

        def motor_positional_servo_target_speed_register(self, *_a, **_kw) -> None:
            pass

        def motor_rotary_motor_register(self, *_a, **_kw) -> None:
            pass

        def register_callback(self, *_a, **_kw) -> None:
            pass

    bo = BrainOutput()
    bo._cache = _FakeCache()
    first = ServoMotor(unit_id=0, channel_index=0, z_neuron_resolution=10)
    second = ServoMotor(unit_id=0, channel_index=1, z_neuron_resolution=10)
    first.group_id = second.group_id = 0
    first.channel = 0
    second.channel = 1
    first.control_semantics = ABSOLUTE_TARGET_INCREMENTAL_SPEED
    second.control_semantics = ABSOLUTE_TARGET_INCREMENTAL_SPEED
    first.incremental_step_ratio = 0.004
    second.incremental_step_ratio = 0.007
    bo._outputs = [first, second]
    with pytest.raises(RuntimeError, match="different incremental_step_ratio"):
        bo._register_motor_decoder()


def test_target_speed_decoder_falls_back_to_legacy_cache_signature() -> None:
    """Registration remains functional with older Rust bindings (no step arg)."""
    pytest.importorskip("feagi_rust_py_libs")
    recorded: list[tuple[int, int, list[float]]] = []

    class _FakeCache:
        def motor_positional_servo_register(self, *_a, **_kw) -> None:
            raise AssertionError("unexpected legacy servo register")

        def motor_positional_servo_target_speed_register(self, *args: object, **_kw: object) -> None:
            if len(args) == 7:
                raise TypeError(
                    "ConnectorAgent.motor_positional_servo_target_speed_register() "
                    "takes 6 positional arguments but 7 were given"
                )
            if len(args) != 6:
                raise AssertionError(f"unexpected args count: {len(args)}")
            group_id, count, _absolute_z, _incremental_z, _positioning, default_speeds = args
            recorded.append((int(group_id), int(count), list(default_speeds)))

        def motor_rotary_motor_register(self, *_a, **_kw) -> None:
            raise AssertionError("unexpected rotary register")

        def register_callback(self, *_a, **_kw) -> None:
            pass

    bo = BrainOutput()
    bo._cache = _FakeCache()
    servo = ServoMotor(unit_id=0, channel_index=0, z_neuron_resolution=10)
    servo.group_id = 0
    servo.channel = 0
    servo.control_semantics = ABSOLUTE_TARGET_INCREMENTAL_SPEED
    servo.default_speed_0_1 = 0.4
    servo.incremental_step_ratio = 0.009
    bo._outputs = [servo]
    bo._register_motor_decoder()
    assert recorded == [(0, 1, [0.4])]


def _fake_cache_recording_legacy_and_target_speed():
    """Build a cache stub that records both PositionalServo registration paths."""
    legacy: list[tuple[int, int]] = []
    target_speed: list[tuple[int, int, float, list[float]]] = []

    class _FakeCache:
        def motor_positional_servo_register(
            self, group_id: int, count: int, *_a, **_kw
        ) -> None:
            legacy.append((int(group_id), int(count)))

        def motor_positional_servo_target_speed_register(
            self,
            group_id: int,
            count: int,
            _absolute_z: int,
            _incremental_z: int,
            _positioning: object,
            default_speeds: list[float],
            incremental_step_0_1: float,
        ) -> None:
            target_speed.append(
                (
                    int(group_id),
                    int(count),
                    float(incremental_step_0_1),
                    list(default_speeds),
                )
            )

        def motor_rotary_motor_register(self, *_a, **_kw) -> None:
            raise AssertionError("unexpected rotary register")

    return _FakeCache(), legacy, target_speed


def test_register_motor_groups_legacy_passes_incremental_step_none() -> None:
    """MuJoCo-style grouped registration must supply incremental_step_0_1."""
    pytest.importorskip("feagi_rust_py_libs")
    cache, legacy, target_speed = _fake_cache_recording_legacy_and_target_speed()
    captured: list[dict] = []
    bo = BrainOutput()
    bo._cache = cache
    original = bo._register_positional_servo_group_decoder

    def _capture(**kwargs):
        captured.append(kwargs)
        return original(**kwargs)

    bo._register_positional_servo_group_decoder = _capture
    bo.register_motor_groups(
        {0: {"positional_servo": ["act1", "act2"], "rotary_motor": []}},
        z_neuron_resolution=20,
    )
    assert legacy == [(0, 2)]
    assert target_speed == []
    assert captured[0]["incremental_step_0_1"] is None
    assert captured[0]["default_speeds_by_channel"] is None


def test_register_motor_groups_target_speed_uses_explicit_incremental_step() -> None:
    """Explicit per-group incremental step is forwarded to the Rust decoder."""
    pytest.importorskip("feagi_rust_py_libs")
    cache, legacy, target_speed = _fake_cache_recording_legacy_and_target_speed()
    bo = BrainOutput()
    bo._cache = cache
    bo.register_motor_groups(
        {0: {"positional_servo": ["act1", "act2"], "rotary_motor": []}},
        z_neuron_resolution=20,
        positional_servo_default_speed_0_1={0: [0.2, 0.4]},
        positional_servo_incremental_step_0_1={0: 0.006},
    )
    assert legacy == []
    assert target_speed == [(0, 2, 0.006, [0.2, 0.4])]


def test_register_motor_groups_target_speed_uses_registered_servo_outputs() -> None:
    """Omitted override dicts resolve decoder args from registered ServoMotors."""
    pytest.importorskip("feagi_rust_py_libs")
    cache, legacy, target_speed = _fake_cache_recording_legacy_and_target_speed()
    bo = BrainOutput()
    bo._cache = cache
    servo = ServoMotor(unit_id=0, channel_index=0, z_neuron_resolution=20)
    servo.group_id = 0
    servo.channel = 0
    servo.control_semantics = ABSOLUTE_TARGET_INCREMENTAL_SPEED
    servo.default_speed_0_1 = 0.33
    servo.incremental_step_ratio = 0.008
    bo._outputs = [servo]
    bo.register_motor_groups(
        {0: {"positional_servo": ["act1"], "rotary_motor": []}},
        z_neuron_resolution=20,
    )
    assert legacy == []
    assert target_speed == [(0, 1, 0.008, [0.33])]


def test_register_motor_groups_target_speed_without_step_raises() -> None:
    """Target-speed grouped registration cannot omit incremental_step_0_1."""
    pytest.importorskip("feagi_rust_py_libs")
    cache, _legacy, _target_speed = _fake_cache_recording_legacy_and_target_speed()
    bo = BrainOutput()
    bo._cache = cache
    with pytest.raises(RuntimeError, match="incremental_step_0_1"):
        bo.register_motor_groups(
            {0: {"positional_servo": ["act1"], "rotary_motor": []}},
            z_neuron_resolution=20,
            positional_servo_default_speed_0_1={0: [0.3]},
        )
