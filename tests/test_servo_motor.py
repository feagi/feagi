import pytest
from feagi.pns.outputs import ServoMotor


def test_servo_motor_registration():
    """Test servo motor registration with IOCache"""
    # Register servo motor with default settings
    servo = ServoMotor.register(range=(0, 180))
    
    # Verify registration
    assert servo.is_registered()
    assert servo.group_id is not None
    assert servo.min_angle == 0
    assert servo.max_angle == 180
    assert servo.encoding == "absolute"
    
    # Verify initial angle is at midpoint
    assert servo.get_angle() == 90.0


def test_servo_motor_custom_range():
    """Test servo motor with custom angle range"""
    servo = ServoMotor.register(range=(-90, 90))
    
    assert servo.is_registered()
    assert servo.min_angle == -90
    assert servo.max_angle == 90
    assert servo.get_angle() == 0.0  # Midpoint


def test_servo_motor_incremental():
    """Test servo motor with incremental encoding"""
    servo = ServoMotor.register(range=(0, 270), encoding="incremental")
    
    assert servo.is_registered()
    assert servo.encoding == "incremental"
    assert servo.min_angle == 0
    assert servo.max_angle == 270


def test_servo_absolute_endpoint_mapping():
    """Absolute mode should map -1/0/+1 to min/center/max."""
    servo = ServoMotor(range=(-1.0, 1.0), encoding="absolute")
    servo._on_motor_command(-1.0)
    assert servo.get_angle() == pytest.approx(-1.0, abs=1e-6)
    servo._on_motor_command(0.0)
    assert servo.get_angle() == pytest.approx(0.0, abs=1e-6)
    servo._on_motor_command(1.0)
    assert servo.get_angle() == pytest.approx(1.0, abs=1e-6)


def test_servo_incremental_applies_delta_and_clamps():
    """Incremental mode should step from current state and clamp to range."""
    servo = ServoMotor(range=(-1.0, 1.0), encoding="incremental")
    servo.incremental_step_ratio = 0.5

    # Starts at center (0.0)
    servo._on_motor_command(1.0)
    assert servo.get_angle() == pytest.approx(0.5, abs=1e-6)
    servo._on_motor_command(1.0)
    assert servo.get_angle() == pytest.approx(1.0, abs=1e-6)
    servo._on_motor_command(1.0)
    assert servo.get_angle() == pytest.approx(1.0, abs=1e-6)

    servo._on_motor_command(-1.0)
    assert servo.get_angle() == pytest.approx(0.5, abs=1e-6)
    servo._on_motor_command(-1.0)
    assert servo.get_angle() == pytest.approx(0.0, abs=1e-6)


def test_servo_motor_cache_methods():
    """Test that servo motor calls correct Rust cache methods"""
    try:
        import feagi_rust_py_libs as frpl
    except ImportError:
        pytest.skip("feagi_rust_py_libs not installed")
    
    # Create cache
    cache = frpl.connector_core.caching.IOCache()
    
    # Create servo
    servo = ServoMotor(range=(0, 180), encoding="absolute")
    
    # Verify cache has the expected methods
    assert hasattr(
        cache,
        "motor_positional_servo_absolute_linear_try_register",
    )
    assert hasattr(
        cache,
        "motor_positional_servo_absolute_linear_try_read_postprocessed_cached_value",
    )
    
    # Register with cache
    servo._register_with_cache(cache, group_id=0)
    servo._mark_registered(0)
    
    # Verify it doesn't crash when reading
    servo._read_from_cache(cache)
    angle = servo.get_angle()
    assert isinstance(angle, float)


def test_multiple_servos():
    """Test registering multiple servos"""
    servo1 = ServoMotor.register(range=(0, 180))
    servo2 = ServoMotor.register(range=(0, 180))
    servo3 = ServoMotor.register(range=(-90, 90))
    
    # Each should have unique group ID
    assert servo1.group_id != servo2.group_id
    assert servo2.group_id != servo3.group_id
    assert servo1.group_id != servo3.group_id
    
    # All should be registered
    assert servo1.is_registered()
    assert servo2.is_registered()
    assert servo3.is_registered()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

