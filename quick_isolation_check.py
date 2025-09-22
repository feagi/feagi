#!/usr/bin/env python3
"""
Quick diagnostic for isolated cortical area activation issue.

This script performs immediate checks that can be run in a live FEAGI environment
to identify the most likely causes of cross-area activation corruption.

Usage:
    python quick_isolation_check.py --area-id cS7aaa
"""

import sys
from typing import Dict, List, Optional, Set
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

def quick_diagnostic(cortical_id: str) -> Dict[str, any]:
    """Run quick diagnostic checks for isolated area activation."""
    
    print(f"\n🔍 QUICK DIAGNOSTIC for isolated area: {cortical_id}")
    print("=" * 60)
    
    results = {
        'area_exists': False,
        'cortical_mapping_ok': False,
        'has_synapses': False,
        'neuron_count': 0,
        'errors': []
    }
    
    try:
        # Import and get instances
        from feagi.bdu.connectome_manager import ConnectomeManager
        from feagi.npu.burst_engine import BurstEngine
        
        # Get ConnectomeManager instance
        try:
            cm = ConnectomeManager.instance()
            print(f"✅ ConnectomeManager instance acquired")
        except Exception as e:
            print(f"❌ Failed to get ConnectomeManager: {e}")
            results['errors'].append(f"ConnectomeManager access failed: {e}")
            return results
        
        # Check 1: Area exists
        print(f"\n[CHECK 1] Area existence...")
        if cortical_id in cm.cortical_areas:
            results['area_exists'] = True
            area = cm.cortical_areas[cortical_id]
            print(f"✅ Area '{cortical_id}' exists")
            print(f"   - Name: {area.name}")
            print(f"   - Dimensions: {area.dimensions}")
            print(f"   - Position: {area.position}")
            print(f"   - Type: {area.area_type}")
        else:
            print(f"❌ Area '{cortical_id}' not found in cortical_areas")
            results['errors'].append(f"Area {cortical_id} not found")
            return results
            
        # Check 2: Cortical mapping
        print(f"\n[CHECK 2] Cortical ID <-> Index mapping...")
        cortical_idx = cm.cortical_mapping.get_idx(cortical_id)
        if cortical_idx is not None:
            results['cortical_mapping_ok'] = True
            # Verify reverse mapping
            reverse_id = cm.cortical_mapping.get_id(cortical_idx)
            if reverse_id == cortical_id:
                print(f"✅ Cortical mapping OK: '{cortical_id}' <-> {cortical_idx}")
            else:
                print(f"❌ Cortical mapping BROKEN: '{cortical_id}' -> {cortical_idx} -> '{reverse_id}'")
                results['errors'].append(f"Bidirectional mapping broken")
        else:
            print(f"❌ No cortical_idx found for '{cortical_id}'")
            results['errors'].append(f"Missing cortical_idx mapping")
            
        # Check 3: Neuron count
        print(f"\n[CHECK 3] Neuron inventory...")
        try:
            neurons = cm.get_neurons_by_cortical_area(cortical_id)
            results['neuron_count'] = len(neurons)
            print(f"✅ Found {len(neurons)} neurons in area")
            if len(neurons) > 0:
                print(f"   - Sample neuron IDs: {neurons[:5]}")
        except Exception as e:
            print(f"❌ Failed to get neurons: {e}")
            results['errors'].append(f"Neuron retrieval failed: {e}")
            
        # Check 4: Synaptic connections (CRITICAL)
        print(f"\n[CHECK 4] Synaptic connections (CRITICAL)...")
        if results['neuron_count'] > 0:
            try:
                # Check for incoming synapses
                incoming_count = 0
                sample_synapses = []
                
                if hasattr(cm, '_npu_interface') and cm._npu_interface:
                    synapse_array = getattr(cm._npu_interface, 'synapse_array', None)
                    if synapse_array and hasattr(synapse_array, 'source_neuron_index'):
                        target_neurons = set(neurons)
                        
                        # Check all synapses for targets in our area
                        for src_neuron, synapse_indices in synapse_array.source_neuron_index.items():
                            for syn_idx in synapse_indices:
                                if (hasattr(synapse_array, 'target_neuron_ids') and 
                                    syn_idx < len(synapse_array.target_neuron_ids)):
                                    target_neuron = synapse_array.target_neuron_ids[syn_idx]
                                    
                                    if target_neuron in target_neurons:
                                        incoming_count += 1
                                        if len(sample_synapses) < 5:
                                            weight = synapse_array.weights[syn_idx] if hasattr(synapse_array, 'weights') else 'unknown'
                                            sample_synapses.append((src_neuron, target_neuron, weight))
                
                results['has_synapses'] = incoming_count > 0
                
                if incoming_count > 0:
                    print(f"🚨 FOUND {incoming_count} INCOMING SYNAPSES - THIS IS THE PROBLEM!")
                    print(f"   This isolated area should have ZERO incoming synapses!")
                    print(f"   Sample connections:")
                    for src, tgt, weight in sample_synapses:
                        src_area = "unknown"
                        try:
                            src_area = cm.get_cortical_area_for_neuron(src)
                        except:
                            pass
                        print(f"     - {src} ({src_area}) -> {tgt} (weight: {weight})")
                else:
                    print(f"✅ No incoming synapses found (as expected for isolated area)")
                    
            except Exception as e:
                print(f"❌ Failed to check synapses: {e}")
                results['errors'].append(f"Synapse check failed: {e}")
                
    except Exception as e:
        print(f"❌ Diagnostic failed: {e}")
        results['errors'].append(f"Diagnostic failure: {e}")
        
    # Summary
    print(f"\n{'=' * 60}")
    print("🏁 DIAGNOSTIC SUMMARY:")
    
    if results['has_synapses']:
        print("🚨 ROOT CAUSE IDENTIFIED: Isolated area has incoming synapses!")
        print("   This is definitely the source of spurious activations.")
        print("   NEXT STEPS:")
        print("   1. Investigate how these synapses were created")
        print("   2. Check connectome loading/modification code")
        print("   3. Verify genome integrity")
        
    elif results['errors']:
        print("❌ Issues found, but root cause unclear:")
        for error in results['errors']:
            print(f"   - {error}")
            
    else:
        print("⚠️  No obvious issues found in static analysis.")
        print("   The corruption may be dynamic (runtime FCL injection).")
        print("   NEXT STEPS:")
        print("   1. Run the full debug_cortical_mapping_isolation.py script")
        print("   2. Enable --debug-npu and monitor FCL injections")
        print("   3. Check for neuron ID mapping corruption")
        
    return results


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Quick diagnostic for isolated area activation')
    parser.add_argument('--area-id', required=True, help='Cortical area ID (e.g., cS7aaa)')
    
    args = parser.parse_args()
    
    try:
        results = quick_diagnostic(args.area_id)
        
        # Exit code based on findings
        if results['has_synapses']:
            sys.exit(2)  # Root cause found
        elif results['errors']:
            sys.exit(1)  # Issues found
        else:
            sys.exit(0)  # No immediate issues
            
    except Exception as e:
        logger.error(f"❌ Quick diagnostic failed: {e}")
        sys.exit(3)


if __name__ == "__main__":
    main()
