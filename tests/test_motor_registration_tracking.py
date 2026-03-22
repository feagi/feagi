"""Test dynamic motor area registration tracking."""

import feagi_rust_py_libs as frpl
from feagi_connector_2 import FeagiAgent


def test_motor_registration_tracking():
    """Test that motor area registrations are tracked correctly."""
    
    feagi_agent = FeagiAgent()
    
    # Initially, no motor areas should be registered
    motor_areas = feagi_agent.brain_output.get_registered_cortical_areas()
    assert motor_areas == [], f"Expected empty list, got {motor_areas}"
    
    # Register miscellaneous motor (omot00)
    feagi_agent.brain_output.miscellaneous_absolute.register(
        cortical_group=0,
        number_of_channels=10,
        misc_dimensions=frpl.connector_core.data_types.descriptors.MiscDataDimensions(10, 1, 1)
    )
    
    motor_areas = feagi_agent.brain_output.get_registered_cortical_areas()
    assert "omot00" in motor_areas, f"Expected 'omot00' in {motor_areas}"
    assert len(motor_areas) == 1, f"Expected 1 area, got {len(motor_areas)}"
    
    # Register gaze motor (ogaz00)
    feagi_agent.brain_output.gaze_absolute_linear.register(
        cortical_group=0,
        number_of_channels=1,
        z_resolution=10
    )
    
    motor_areas = feagi_agent.brain_output.get_registered_cortical_areas()
    assert "omot00" in motor_areas, f"Expected 'omot00' in {motor_areas}"
    assert "ogaz00" in motor_areas, f"Expected 'ogaz00' in {motor_areas}"
    assert len(motor_areas) == 2, f"Expected 2 areas, got {len(motor_areas)}"
    
    # Verify sorted order
    assert motor_areas == ["ogaz00", "omot00"], f"Expected sorted order, got {motor_areas}"
    
    print("✅ All motor registration tracking tests passed!")


def test_multiple_cortical_groups():
    """Test registration with multiple cortical groups."""
    
    feagi_agent = FeagiAgent()
    
    # Register omot00, omot01, omot02
    feagi_agent.brain_output.miscellaneous_absolute.register(
        cortical_group=0,
        number_of_channels=10,
        misc_dimensions=frpl.connector_core.data_types.descriptors.MiscDataDimensions(10, 1, 1)
    )
    feagi_agent.brain_output.miscellaneous_absolute.register(
        cortical_group=1,
        number_of_channels=10,
        misc_dimensions=frpl.connector_core.data_types.descriptors.MiscDataDimensions(10, 1, 1)
    )
    feagi_agent.brain_output.miscellaneous_absolute.register(
        cortical_group=2,
        number_of_channels=10,
        misc_dimensions=frpl.connector_core.data_types.descriptors.MiscDataDimensions(10, 1, 1)
    )
    
    motor_areas = feagi_agent.brain_output.get_registered_cortical_areas()
    assert "omot00" in motor_areas
    assert "omot01" in motor_areas
    assert "omot02" in motor_areas
    assert len(motor_areas) == 3, f"Expected 3 areas, got {len(motor_areas)}: {motor_areas}"
    
    print("✅ Multiple cortical groups test passed!")


def test_servo_registration():
    """Test servo motor registration (oser prefix)."""
    
    feagi_agent = FeagiAgent()
    
    # Register servo motor (oser00)
    feagi_agent.brain_output.positional_servo_absolute_linear.register(
        cortical_group=0,
        number_of_channels=5,
        z_resolution=10
    )
    
    motor_areas = feagi_agent.brain_output.get_registered_cortical_areas()
    assert "oser00" in motor_areas, f"Expected 'oser00' in {motor_areas}"
    
    print("✅ Servo registration test passed!")


if __name__ == "__main__":
    test_motor_registration_tracking()
    test_multiple_cortical_groups()
    test_servo_registration()
    print("\n🎉 All tests passed!")


