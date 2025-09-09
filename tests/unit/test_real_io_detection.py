"""
Test the actual CoreAPIService._auto_assign_region_io method
to validate I/O detection works with real implementation.
"""

def test_real_core_api_io_detection():
    """Test the actual CoreAPIService I/O detection method."""
    from unittest.mock import MagicMock, patch
    
    # Test with the exact scenario from your logs
    with patch("feagi.core.state_manager.FeagiStateManager") as mock_state_manager_class:
        mock_state_manager = MagicMock()
        mock_state_manager_class.instance.return_value = mock_state_manager
        
        # Mock blueprint matching the connection structure
        test_blueprint = {
            'c1Xqqq': {
                'cortical_id': 'c1Xqqq',
                'cortical_group': 'CUSTOM',
                'brain_region_id': 'region_6a991f2c',
                'cortical_destinations': {}  # No outgoing connections
            },
            'cIHMot': {
                'cortical_id': 'cIHMot', 
                'cortical_group': 'IPU',
                'cortical_destinations': {}
            },
            'cRSMot': {
                'cortical_id': 'cRSMot',
                'cortical_group': 'OPU', 
                'cortical_destinations': {
                    'external_area': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
                }
            },
            # External area that connects TO c1Xqqq
            'external_sensor': {
                'cortical_id': 'external_sensor',
                'brain_region_id': 'root',
                'cortical_destinations': {
                    'c1Xqqq': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]  
                }
            },
            'external_area': {
                'cortical_id': 'external_area',
                'brain_region_id': 'root',
                'cortical_destinations': {}
            }
        }
        
        mock_state_manager.genome = {'blueprint': test_blueprint}
        
        # Import and test the real CoreAPIService method
        from feagi.api.core.services.core_api_service import CoreAPIService
        
        service = CoreAPIService()
        
        # Test the actual _auto_assign_region_io method
        areas = ['c1Xqqq', 'cIHMot', 'cRSMot']
        inputs, outputs = service._auto_assign_region_io(areas, test_blueprint)
        
        print("🔍 REAL METHOD RESULTS:")
        print(f"   Areas: {areas}")
        print(f"   Inputs: {inputs}")  
        print(f"   Outputs: {outputs}")
        
        # Validate results
        assert 'c1Xqqq' in inputs, (
            f"c1Xqqq should be detected as INPUT due to connection from external_sensor. "
            f"Got inputs: {inputs}"
        )
        
        assert 'cRSMot' in outputs, (
            f"cRSMot should be detected as OUTPUT due to connection to external_area. "
            f"Got outputs: {outputs}"
        )
        
        print("✅ REAL CORE API METHOD TEST PASSED!")
        print("   c1Xqqq correctly detected as INPUT")
        print("   cRSMot correctly detected as OUTPUT")
        
        return True

if __name__ == "__main__":
    print("🧪 Testing Real CoreAPIService I/O Detection...")
    print()
    
    try:
        test_real_core_api_io_detection()
        print()
        print("🎉 REAL METHOD VALIDATION PASSED!")
        
    except Exception as e:
        print(f"❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
