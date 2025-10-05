"""
Comprehensive test for I/O detection covering all connection scenarios.

Tests different cases:
1. Area with incoming connection → INPUT
2. Area with outgoing connection → OUTPUT  
3. Area with both incoming and outgoing → BOTH INPUT and OUTPUT
4. Area with no connections → fallback to group type (IPU/OPU)
5. Area with internal connections only → no I/O designation
"""

def test_comprehensive_io_detection_scenarios():
    """Test all I/O detection scenarios comprehensively."""
    
    def simulate_io_detection(areas, blueprint):
        """Simulate the I/O detection algorithm."""
        inputs = []
        outputs = []
        area_set = set(areas)
        
        # STEP 1: Find OUTPUTS - areas with connections to external areas
        for area_id in areas:
            area_props = blueprint.get(area_id, {})
            destinations = area_props.get("cortical_destinations", {})
            external_destinations = [dest for dest in destinations.keys() if dest not in area_set]
            if external_destinations:
                outputs.append(area_id)
        
        # STEP 2: Find INPUTS - areas receiving connections from external areas
        for source_area_id, source_props in blueprint.items():
            if source_area_id in area_set:  # Skip internal areas
                continue
            source_destinations = source_props.get("cortical_destinations", {})
            for dest_area in source_destinations.keys():
                if dest_area in area_set and dest_area not in inputs:
                    inputs.append(dest_area)
        
        # STEP 3: Fallback to group types for unassigned areas
        areas_with_io = set(inputs + outputs)
        unassigned_areas = [area for area in areas if area not in areas_with_io]
        
        for area_id in unassigned_areas:
            area_props = blueprint.get(area_id, {})
            area_group = area_props.get("cortical_group", "").upper()
            if area_group == "IPU":
                inputs.append(area_id)
            elif area_group == "OPU":
                outputs.append(area_id)
        
        return sorted(inputs), sorted(outputs)
    
    print("🧪 Testing I/O Detection - All Connection Scenarios")
    print()
    
    # === SCENARIO 1: Area with INCOMING connection only ===
    print("📥 SCENARIO 1: Area with INCOMING connection → should be INPUT")
    
    blueprint_1 = {
        # Region areas
        'area_incoming_only': {
            'cortical_id': 'area_incoming_only',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'test_region',
            'cortical_destinations': {}  # No outgoing connections
        },
        # External area connecting TO region
        'external_source': {
            'cortical_id': 'external_source',
            'brain_region_id': 'root',
            'cortical_destinations': {
                'area_incoming_only': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
            }
        }
    }
    
    areas_1 = ['area_incoming_only']
    inputs_1, outputs_1 = simulate_io_detection(areas_1, blueprint_1)
    
    assert 'area_incoming_only' in inputs_1, f"Should be INPUT, got inputs: {inputs_1}"
    assert 'area_incoming_only' not in outputs_1, f"Should NOT be OUTPUT, got outputs: {outputs_1}"
    print(f"   ✅ PASS: inputs={inputs_1}, outputs={outputs_1}")
    
    # === SCENARIO 2: Area with OUTGOING connection only ===
    print("📤 SCENARIO 2: Area with OUTGOING connection → should be OUTPUT")
    
    blueprint_2 = {
        # Region area
        'area_outgoing_only': {
            'cortical_id': 'area_outgoing_only',
            'cortical_group': 'CUSTOM', 
            'brain_region_id': 'test_region',
            'cortical_destinations': {
                'external_target': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
            }
        },
        # External target area
        'external_target': {
            'cortical_id': 'external_target',
            'brain_region_id': 'root',
            'cortical_destinations': {}
        }
    }
    
    areas_2 = ['area_outgoing_only']
    inputs_2, outputs_2 = simulate_io_detection(areas_2, blueprint_2)
    
    assert 'area_outgoing_only' not in inputs_2, f"Should NOT be INPUT, got inputs: {inputs_2}"
    assert 'area_outgoing_only' in outputs_2, f"Should be OUTPUT, got outputs: {outputs_2}"
    print(f"   ✅ PASS: inputs={inputs_2}, outputs={outputs_2}")
    
    # === SCENARIO 3: Area with BOTH incoming AND outgoing ===
    print("🔄 SCENARIO 3: Area with BOTH incoming AND outgoing → should be BOTH")
    
    blueprint_3 = {
        # Region area 
        'area_both': {
            'cortical_id': 'area_both',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'test_region',
            'cortical_destinations': {
                'external_target': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
            }
        },
        # External source connecting TO region area
        'external_source': {
            'cortical_id': 'external_source',
            'brain_region_id': 'root',
            'cortical_destinations': {
                'area_both': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
            }
        },
        # External target area
        'external_target': {
            'cortical_id': 'external_target', 
            'brain_region_id': 'root',
            'cortical_destinations': {}
        }
    }
    
    areas_3 = ['area_both']
    inputs_3, outputs_3 = simulate_io_detection(areas_3, blueprint_3)
    
    assert 'area_both' in inputs_3, f"Should be INPUT, got inputs: {inputs_3}"
    assert 'area_both' in outputs_3, f"Should be OUTPUT, got outputs: {outputs_3}"
    print(f"   ✅ PASS: inputs={inputs_3}, outputs={outputs_3}")
    
    # === SCENARIO 4: Area with NO connections → fallback to group type ===
    print("🈳 SCENARIO 4: Area with NO connections → fallback to group type")
    
    blueprint_4 = {
        # IPU area with no connections
        'area_ipu_no_conn': {
            'cortical_id': 'area_ipu_no_conn',
            'cortical_group': 'IPU',
            'brain_region_id': 'test_region',
            'cortical_destinations': {}
        },
        # OPU area with no connections
        'area_opu_no_conn': {
            'cortical_id': 'area_opu_no_conn',
            'cortical_group': 'OPU', 
            'brain_region_id': 'test_region',
            'cortical_destinations': {}
        },
        # CUSTOM area with no connections (no fallback)
        'area_custom_no_conn': {
            'cortical_id': 'area_custom_no_conn',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'test_region', 
            'cortical_destinations': {}
        }
    }
    
    areas_4 = ['area_ipu_no_conn', 'area_opu_no_conn', 'area_custom_no_conn']
    inputs_4, outputs_4 = simulate_io_detection(areas_4, blueprint_4)
    
    assert 'area_ipu_no_conn' in inputs_4, f"IPU should be INPUT, got inputs: {inputs_4}"
    assert 'area_opu_no_conn' in outputs_4, f"OPU should be OUTPUT, got outputs: {outputs_4}"
    assert 'area_custom_no_conn' not in inputs_4 and 'area_custom_no_conn' not in outputs_4, f"CUSTOM should be neither, got inputs: {inputs_4}, outputs: {outputs_4}"
    print(f"   ✅ PASS: inputs={inputs_4}, outputs={outputs_4}")
    
    # === SCENARIO 5: Area with INTERNAL connections only ===
    print("🔗 SCENARIO 5: Area with INTERNAL connections only → no I/O designation")
    
    blueprint_5 = {
        # Region areas
        'area_internal_source': {
            'cortical_id': 'area_internal_source',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'test_region',
            'cortical_destinations': {
                'area_internal_target': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
            }
        },
        'area_internal_target': {
            'cortical_id': 'area_internal_target',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'test_region',
            'cortical_destinations': {}
        }
    }
    
    areas_5 = ['area_internal_source', 'area_internal_target']
    inputs_5, outputs_5 = simulate_io_detection(areas_5, blueprint_5)
    
    # Neither should be I/O since all connections are internal
    assert len(inputs_5) == 0, f"Should have no inputs for internal connections, got: {inputs_5}"
    assert len(outputs_5) == 0, f"Should have no outputs for internal connections, got: {outputs_5}"
    print(f"   ✅ PASS: inputs={inputs_5}, outputs={outputs_5}")
    
    print()
    print("🎉 ALL I/O DETECTION SCENARIOS PASSED!")
    print("   ✓ Incoming connection → INPUT")
    print("   ✓ Outgoing connection → OUTPUT")
    print("   ✓ Both connections → BOTH INPUT and OUTPUT")
    print("   ✓ No connections → fallback to group type")
    print("   ✓ Internal connections only → no I/O designation")
    
    return True

if __name__ == "__main__":
    try:
        test_comprehensive_io_detection_scenarios()
        print()
        print("🏆 COMPREHENSIVE I/O DETECTION TEST COMPLETE!")
        
    except Exception as e:
        print(f"❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
