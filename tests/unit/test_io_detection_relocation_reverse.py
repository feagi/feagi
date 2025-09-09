"""
Comprehensive test for I/O detection during REVERSE relocation scenarios.

Tests moving areas OUT of regions in all connection scenarios:
1. Area with incoming connection moved OUT → should remove from region inputs
2. Area with outgoing connection moved OUT → should remove from region outputs  
3. Area with both connections moved OUT → should remove from both inputs and outputs
4. Area with no connections moved OUT → should handle group-based changes
5. Area with internal connections moved OUT → should update internal connection handling

Tests both source region (losing area) and destination region (gaining area).
"""

def test_comprehensive_reverse_relocation_io():
    """Test I/O detection when areas are moved OUT of regions."""
    
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
    
    print("🧪 Testing REVERSE Relocation - All I/O Scenarios")
    print("   (Moving areas OUT of regions)")
    print()
    
    # === REVERSE SCENARIO 1: Move area with INCOMING connection OUT of region ===
    print("📥➡️ REVERSE 1: Move area with INCOMING connection OUT → should remove from region inputs")
    
    blueprint_r1 = {
        # Region areas (before move)
        'area_with_incoming': {
            'cortical_id': 'area_with_incoming',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'source_region',  # Currently in source_region
            'cortical_destinations': {}
        },
        'other_area_in_region': {
            'cortical_id': 'other_area_in_region',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'source_region',
            'cortical_destinations': {}
        },
        # External area connecting TO region
        'external_source': {
            'cortical_id': 'external_source',
            'brain_region_id': 'root',
            'cortical_destinations': {
                'area_with_incoming': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
            }
        }
    }
    
    # BEFORE move: source_region has both areas
    areas_before = ['area_with_incoming', 'other_area_in_region']
    inputs_before, outputs_before = simulate_io_detection(areas_before, blueprint_r1)
    print(f"   Before move - Source region: inputs={inputs_before}, outputs={outputs_before}")
    assert 'area_with_incoming' in inputs_before, "Should have area_with_incoming as input"
    
    # SIMULATE MOVE: area_with_incoming moves to destination_region
    blueprint_after_r1 = blueprint_r1.copy()
    blueprint_after_r1['area_with_incoming']['brain_region_id'] = 'destination_region'
    
    # AFTER move: source_region only has other_area_in_region
    areas_source_after = ['other_area_in_region']
    inputs_source_after, outputs_source_after = simulate_io_detection(areas_source_after, blueprint_after_r1)
    print(f"   After move - Source region: inputs={inputs_source_after}, outputs={outputs_source_after}")
    assert 'area_with_incoming' not in inputs_source_after, "Should remove area_with_incoming from source inputs"
    
    # AFTER move: destination_region gains area_with_incoming
    areas_dest_after = ['area_with_incoming']
    inputs_dest_after, outputs_dest_after = simulate_io_detection(areas_dest_after, blueprint_after_r1)
    print(f"   After move - Destination region: inputs={inputs_dest_after}, outputs={outputs_dest_after}")
    assert 'area_with_incoming' in inputs_dest_after, "Should add area_with_incoming to destination inputs"
    
    print("   ✅ PASS: Incoming connection correctly handled in reverse relocation")
    print()
    
    # === REVERSE SCENARIO 2: Move area with OUTGOING connection OUT of region ===
    print("📤➡️ REVERSE 2: Move area with OUTGOING connection OUT → should remove from region outputs")
    
    blueprint_r2 = {
        # Region areas (before move)
        'area_with_outgoing': {
            'cortical_id': 'area_with_outgoing',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'source_region',
            'cortical_destinations': {
                'external_target': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
            }
        },
        'other_area_in_region': {
            'cortical_id': 'other_area_in_region', 
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'source_region',
            'cortical_destinations': {}
        },
        # External target
        'external_target': {
            'cortical_id': 'external_target',
            'brain_region_id': 'root',
            'cortical_destinations': {}
        }
    }
    
    # BEFORE move
    areas_before_r2 = ['area_with_outgoing', 'other_area_in_region']
    inputs_before_r2, outputs_before_r2 = simulate_io_detection(areas_before_r2, blueprint_r2)
    print(f"   Before move - Source region: inputs={inputs_before_r2}, outputs={outputs_before_r2}")
    assert 'area_with_outgoing' in outputs_before_r2, "Should have area_with_outgoing as output"
    
    # SIMULATE MOVE
    blueprint_after_r2 = blueprint_r2.copy()
    blueprint_after_r2['area_with_outgoing']['brain_region_id'] = 'destination_region'
    
    # AFTER move: source_region loses the output area
    areas_source_after_r2 = ['other_area_in_region']
    inputs_source_after_r2, outputs_source_after_r2 = simulate_io_detection(areas_source_after_r2, blueprint_after_r2)
    print(f"   After move - Source region: inputs={inputs_source_after_r2}, outputs={outputs_source_after_r2}")
    assert 'area_with_outgoing' not in outputs_source_after_r2, "Should remove area_with_outgoing from source outputs"
    
    # AFTER move: destination_region gains the output area
    areas_dest_after_r2 = ['area_with_outgoing']
    inputs_dest_after_r2, outputs_dest_after_r2 = simulate_io_detection(areas_dest_after_r2, blueprint_after_r2)
    print(f"   After move - Destination region: inputs={inputs_dest_after_r2}, outputs={outputs_dest_after_r2}")
    assert 'area_with_outgoing' in outputs_dest_after_r2, "Should add area_with_outgoing to destination outputs"
    
    print("   ✅ PASS: Outgoing connection correctly handled in reverse relocation")
    print()
    
    # === REVERSE SCENARIO 3: Move area with BOTH connections OUT ===
    print("🔄➡️ REVERSE 3: Move area with BOTH connections OUT → should remove from both inputs and outputs")
    
    blueprint_r3 = {
        # Region area with both connections
        'area_with_both': {
            'cortical_id': 'area_with_both',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'source_region',
            'cortical_destinations': {
                'external_target': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
            }
        },
        'other_area_in_region': {
            'cortical_id': 'other_area_in_region',
            'cortical_group': 'CUSTOM', 
            'brain_region_id': 'source_region',
            'cortical_destinations': {}
        },
        # External areas
        'external_source': {
            'cortical_id': 'external_source',
            'brain_region_id': 'root',
            'cortical_destinations': {
                'area_with_both': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
            }
        },
        'external_target': {
            'cortical_id': 'external_target',
            'brain_region_id': 'root',
            'cortical_destinations': {}
        }
    }
    
    # BEFORE move
    areas_before_r3 = ['area_with_both', 'other_area_in_region']
    inputs_before_r3, outputs_before_r3 = simulate_io_detection(areas_before_r3, blueprint_r3)
    print(f"   Before move - Source region: inputs={inputs_before_r3}, outputs={outputs_before_r3}")
    assert 'area_with_both' in inputs_before_r3, "Should have area_with_both as input"
    assert 'area_with_both' in outputs_before_r3, "Should have area_with_both as output"
    
    # SIMULATE MOVE
    blueprint_after_r3 = blueprint_r3.copy()
    blueprint_after_r3['area_with_both']['brain_region_id'] = 'destination_region'
    
    # AFTER move: source_region loses both input and output
    areas_source_after_r3 = ['other_area_in_region']
    inputs_source_after_r3, outputs_source_after_r3 = simulate_io_detection(areas_source_after_r3, blueprint_after_r3)
    print(f"   After move - Source region: inputs={inputs_source_after_r3}, outputs={outputs_source_after_r3}")
    assert 'area_with_both' not in inputs_source_after_r3, "Should remove from source inputs"
    assert 'area_with_both' not in outputs_source_after_r3, "Should remove from source outputs"
    
    # AFTER move: destination_region gains both input and output
    areas_dest_after_r3 = ['area_with_both']
    inputs_dest_after_r3, outputs_dest_after_r3 = simulate_io_detection(areas_dest_after_r3, blueprint_after_r3)
    print(f"   After move - Destination region: inputs={inputs_dest_after_r3}, outputs={outputs_dest_after_r3}")
    assert 'area_with_both' in inputs_dest_after_r3, "Should add to destination inputs"
    assert 'area_with_both' in outputs_dest_after_r3, "Should add to destination outputs"
    
    print("   ✅ PASS: Both connections correctly handled in reverse relocation")
    print()
    
    # === REVERSE SCENARIO 4: Move area with NO connections (group-based) OUT ===
    print("🈳➡️ REVERSE 4: Move area with NO connections OUT → should handle group-based fallbacks")
    
    blueprint_r4 = {
        # IPU area with no connections in source region
        'ipu_area_no_conn': {
            'cortical_id': 'ipu_area_no_conn',
            'cortical_group': 'IPU',
            'brain_region_id': 'source_region',
            'cortical_destinations': {}
        },
        'other_area': {
            'cortical_id': 'other_area',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'source_region',
            'cortical_destinations': {}
        }
    }
    
    # BEFORE move: IPU area should be input due to group
    areas_before_r4 = ['ipu_area_no_conn', 'other_area']
    inputs_before_r4, outputs_before_r4 = simulate_io_detection(areas_before_r4, blueprint_r4)
    print(f"   Before move - Source region: inputs={inputs_before_r4}, outputs={outputs_before_r4}")
    assert 'ipu_area_no_conn' in inputs_before_r4, "IPU should be input due to group type"
    
    # SIMULATE MOVE
    blueprint_after_r4 = blueprint_r4.copy()
    blueprint_after_r4['ipu_area_no_conn']['brain_region_id'] = 'destination_region'
    
    # AFTER move: source region loses IPU input
    areas_source_after_r4 = ['other_area']
    inputs_source_after_r4, outputs_source_after_r4 = simulate_io_detection(areas_source_after_r4, blueprint_after_r4)
    print(f"   After move - Source region: inputs={inputs_source_after_r4}, outputs={outputs_source_after_r4}")
    assert 'ipu_area_no_conn' not in inputs_source_after_r4, "Should remove IPU from source inputs"
    
    # AFTER move: destination region gains IPU input
    areas_dest_after_r4 = ['ipu_area_no_conn']
    inputs_dest_after_r4, outputs_dest_after_r4 = simulate_io_detection(areas_dest_after_r4, blueprint_after_r4)
    print(f"   After move - Destination region: inputs={inputs_dest_after_r4}, outputs={outputs_dest_after_r4}")
    assert 'ipu_area_no_conn' in inputs_dest_after_r4, "Should add IPU to destination inputs"
    
    print("   ✅ PASS: Group-based assignment correctly handled in reverse relocation")
    print()
    
    # === REVERSE SCENARIO 5: Move area with INTERNAL connections OUT ===
    print("🔗➡️ REVERSE 5: Move area with INTERNAL connections OUT → should break internal chain")
    
    blueprint_r5 = {
        # Internal connection chain in source region
        'area_internal_source': {
            'cortical_id': 'area_internal_source',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'source_region',
            'cortical_destinations': {
                'area_internal_target': [['0_0_0-1_0_0', [1, 1, 1], 1, False, 1, 1, 1]]
            }
        },
        'area_internal_target': {
            'cortical_id': 'area_internal_target',
            'cortical_group': 'CUSTOM',
            'brain_region_id': 'source_region',
            'cortical_destinations': {}
        }
    }
    
    # BEFORE move: both areas internal, no I/O
    areas_before_r5 = ['area_internal_source', 'area_internal_target']
    inputs_before_r5, outputs_before_r5 = simulate_io_detection(areas_before_r5, blueprint_r5)
    print(f"   Before move - Source region: inputs={inputs_before_r5}, outputs={outputs_before_r5}")
    assert len(inputs_before_r5) == 0, "Should have no inputs (internal only)"
    assert len(outputs_before_r5) == 0, "Should have no outputs (internal only)"
    
    # SIMULATE MOVE: move source area to destination
    blueprint_after_r5 = blueprint_r5.copy()
    blueprint_after_r5['area_internal_source']['brain_region_id'] = 'destination_region'
    
    # AFTER move: source region only has target area
    areas_source_after_r5 = ['area_internal_target']
    inputs_source_after_r5, outputs_source_after_r5 = simulate_io_detection(areas_source_after_r5, blueprint_after_r5)
    print(f"   After move - Source region: inputs={inputs_source_after_r5}, outputs={outputs_source_after_r5}")
    # area_internal_target should now be an INPUT (receives from external area_internal_source)
    assert 'area_internal_target' in inputs_source_after_r5, "Should become INPUT (receives from external area)"
    
    # AFTER move: destination region has source area  
    areas_dest_after_r5 = ['area_internal_source']
    inputs_dest_after_r5, outputs_dest_after_r5 = simulate_io_detection(areas_dest_after_r5, blueprint_after_r5)
    print(f"   After move - Destination region: inputs={inputs_dest_after_r5}, outputs={outputs_dest_after_r5}")
    # area_internal_source should now be an OUTPUT (connects to external area_internal_target)
    assert 'area_internal_source' in outputs_dest_after_r5, "Should become OUTPUT (connects to external area)"
    
    print("   ✅ PASS: Internal connection breakage correctly handled in reverse relocation")
    print()
    
    print("🎉 ALL REVERSE RELOCATION SCENARIOS PASSED!")
    print("   ✓ Incoming connection removal handled correctly")
    print("   ✓ Outgoing connection removal handled correctly")
    print("   ✓ Both connection types removal handled correctly")
    print("   ✓ Group-based fallback changes handled correctly")
    print("   ✓ Internal connection breakage creates new I/O correctly")
    
    return True

if __name__ == "__main__":
    try:
        test_comprehensive_reverse_relocation_io()
        print()
        print("🏆 COMPREHENSIVE REVERSE RELOCATION TEST COMPLETE!")
        print("   All scenarios for moving areas OUT of regions validated!")
        
    except Exception as e:
        print(f"❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
