#!/usr/bin/env python3
"""
Debug script for isolated cortical area activation issue.

This script investigates potential corruption in cortical mapping system
that could cause cross-area activation leakage.

Usage:
    python debug_cortical_mapping_isolation.py --area-id cS7aaa --timesteps 10
"""

import sys
import numpy as np
from typing import Dict, List, Set, Optional, Tuple
from collections import defaultdict

from feagi.utils.logger import setup_logger
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.npu.burst_engine import BurstEngine
from feagi.core.state_manager import FeagiStateManager

logger = setup_logger(__name__)

class CorticalIsolationDebugger:
    """Debug tool for investigating isolated cortical area activation corruption."""
    
    def __init__(self, connectome_manager: ConnectomeManager, burst_engine: BurstEngine):
        self.cm = connectome_manager
        self.burst_engine = burst_engine
        self.target_area_id = None
        self.target_cortical_idx = None
        self.debug_data = defaultdict(list)
        
    def set_target_area(self, cortical_id: str):
        """Set the target isolated area to debug."""
        self.target_area_id = cortical_id
        
        # Verify area exists
        if cortical_id not in self.cm.cortical_areas:
            raise ValueError(f"Cortical area '{cortical_id}' not found")
            
        # Get cortical index
        self.target_cortical_idx = self.cm.cortical_mapping.get_idx(cortical_id)
        if self.target_cortical_idx is None:
            raise RuntimeError(f"No cortical_idx found for area '{cortical_id}' - mapping corruption detected")
            
        logger.info(f"🎯 Debugging area: {cortical_id} (idx={self.target_cortical_idx})")
        
    def verify_cortical_mapping_integrity(self) -> Dict[str, any]:
        """Verify the integrity of cortical mapping system."""
        logger.info("🔍 [STEP 1] Verifying cortical mapping integrity...")
        
        issues = []
        mapping_stats = {
            'total_areas': len(self.cm.cortical_areas),
            'mapping_issues': [],
            'target_area_status': 'unknown'
        }
        
        # Check bidirectional mapping consistency
        for cortical_id, area in self.cm.cortical_areas.items():
            # Forward mapping: ID -> IDX
            mapped_idx = self.cm.cortical_mapping.get_idx(cortical_id)
            
            if mapped_idx is None:
                issue = f"❌ Area '{cortical_id}' has no cortical_idx mapping"
                issues.append(issue)
                logger.error(issue)
                continue
                
            # Reverse mapping: IDX -> ID  
            reverse_id = self.cm.cortical_mapping.get_id(mapped_idx)
            
            if reverse_id != cortical_id:
                issue = f"❌ Bidirectional mapping broken: '{cortical_id}' -> {mapped_idx} -> '{reverse_id}'"
                issues.append(issue)
                logger.error(issue)
                
            # Check if this is our target area
            if cortical_id == self.target_area_id:
                mapping_stats['target_area_status'] = 'valid' if not issues else 'corrupted'
                logger.info(f"✅ Target area mapping: '{cortical_id}' <-> {mapped_idx}")
                
        mapping_stats['mapping_issues'] = issues
        return mapping_stats
        
    def verify_neuron_id_mappings(self) -> Dict[str, any]:
        """Verify neuron ID to array index mapping consistency."""
        logger.info("🔍 [STEP 2] Verifying neuron ID mapping integrity...")
        
        stats = {
            'total_neurons': 0,
            'mapping_inconsistencies': [],
            'target_area_neurons': 0,
            'orphaned_neurons': []
        }
        
        # Get all neurons in target area
        try:
            target_neurons = self.cm.get_neurons_by_cortical_area(self.target_area_id)
            stats['target_area_neurons'] = len(target_neurons)
            logger.info(f"📊 Target area '{self.target_area_id}' has {len(target_neurons)} neurons")
            
            # Verify each neuron's mapping consistency
            inconsistent_neurons = []
            
            for neuron_id in target_neurons:
                # Check ConnectomeManager mapping
                cm_index = self.cm._neuron_id_to_index_map.get(neuron_id)
                
                # Check NPU NeuronArray mapping
                npu_index = None
                if hasattr(self.cm, '_npu_interface') and self.cm._npu_interface:
                    na = getattr(self.cm._npu_interface, 'neuron_array', None)
                    if na:
                        npu_index = na.neuron_id_to_index.get(neuron_id)
                
                # Check consistency
                if cm_index != npu_index:
                    inconsistency = {
                        'neuron_id': neuron_id,
                        'cm_index': cm_index,
                        'npu_index': npu_index
                    }
                    inconsistent_neurons.append(inconsistency)
                    logger.error(f"❌ Neuron {neuron_id}: CM index={cm_index}, NPU index={npu_index}")
                    
                # Verify reverse mapping
                if cm_index is not None:
                    reverse_id = self.cm._index_to_neuron_id_map.get(cm_index)
                    if reverse_id != neuron_id:
                        inconsistent_neurons.append({
                            'neuron_id': neuron_id,
                            'index': cm_index,
                            'reverse_id': reverse_id,
                            'issue': 'reverse_mapping_broken'
                        })
                        
            stats['mapping_inconsistencies'] = inconsistent_neurons
            
        except Exception as e:
            logger.error(f"❌ Failed to verify neuron mappings: {e}")
            stats['error'] = str(e)
            
        return stats
        
    def trace_fcl_injection_sources(self, timesteps: int = 5) -> Dict[str, any]:
        """Trace FCL injection sources to identify spurious activation origins."""
        logger.info(f"🔍 [STEP 3] Tracing FCL injection sources over {timesteps} timesteps...")
        
        injection_data = {
            'timestep_data': [],
            'unexpected_injections': [],
            'injection_sources': defaultdict(int)
        }
        
        # Hook into FCL injection system
        original_inject_synaptic = None
        if hasattr(self.burst_engine, 'injection_service'):
            fcl_injector = self.burst_engine.injection_service
            
            # Store original method
            original_inject_synaptic = fcl_injector.inject_synaptic_propagation
            
            def debug_inject_synaptic_propagation(fcl, propagation_data):
                """Debug wrapper for synaptic injection."""
                timestep_injections = []
                
                for cortical_idx, connections in propagation_data.items():
                    if cortical_idx == self.target_cortical_idx:
                        # This is an injection into our isolated area!
                        injection_event = {
                            'timestep': self.burst_engine.current_timestep,
                            'cortical_idx': cortical_idx,
                            'connection_count': len(connections),
                            'connections': connections[:10] if connections else []  # Sample first 10
                        }
                        timestep_injections.append(injection_event)
                        injection_data['unexpected_injections'].append(injection_event)
                        
                        logger.error(f"🚨 UNEXPECTED INJECTION into isolated area {self.target_area_id}!")
                        logger.error(f"   Timestep: {self.burst_engine.current_timestep}")
                        logger.error(f"   Connections: {len(connections)}")
                        logger.error(f"   Sample connections: {connections[:5] if connections else 'None'}")
                        
                    # Track injection sources
                    if connections:
                        injection_data['injection_sources'][cortical_idx] += len(connections)
                
                injection_data['timestep_data'].append({
                    'timestep': self.burst_engine.current_timestep,
                    'injections': timestep_injections
                })
                
                # Call original method
                return original_inject_synaptic(fcl, propagation_data)
                
            # Replace method with debug wrapper
            fcl_injector.inject_synaptic_propagation = debug_inject_synaptic_propagation
            
        # Run simulation for specified timesteps
        logger.info(f"🏃 Running {timesteps} simulation timesteps...")
        try:
            for step in range(timesteps):
                logger.info(f"  Timestep {step + 1}/{timesteps}")
                fired_neurons = self.burst_engine.process_burst()
                
                # Check if any neurons fired in our target area
                if fired_neurons:
                    target_area_fires = []
                    for neuron_id in fired_neurons:
                        try:
                            area_id = self.cm.get_cortical_area_for_neuron(neuron_id)
                            if area_id == self.target_area_id:
                                target_area_fires.append(neuron_id)
                        except KeyError:
                            logger.error(f"❌ Fired neuron {neuron_id} has no cortical area mapping!")
                            
                    if target_area_fires:
                        logger.error(f"🚨 ISOLATED AREA FIRED: {len(target_area_fires)} neurons in {self.target_area_id}")
                        logger.error(f"   Fired neurons: {target_area_fires}")
                        
        finally:
            # Restore original method
            if original_inject_synaptic:
                fcl_injector.inject_synaptic_propagation = original_inject_synaptic
                
        return injection_data
        
    def analyze_synapse_array_integrity(self) -> Dict[str, any]:
        """Analyze synapse array for potential corruption."""
        logger.info("🔍 [STEP 4] Analyzing synapse array integrity...")
        
        analysis = {
            'synapse_count': 0,
            'target_area_incoming': 0,
            'target_area_outgoing': 0,
            'suspicious_synapses': [],
            'array_integrity_issues': []
        }
        
        try:
            if not hasattr(self.cm, '_npu_interface') or not self.cm._npu_interface:
                analysis['error'] = "No NPU interface available"
                return analysis
                
            synapse_array = getattr(self.cm._npu_interface, 'synapse_array', None)
            if not synapse_array:
                analysis['error'] = "No synapse array available"
                return analysis
                
            # Get target area neurons
            target_neurons = set(self.cm.get_neurons_by_cortical_area(self.target_area_id))
            
            # Check for incoming synapses (should be zero for isolated area)
            if hasattr(synapse_array, 'source_neuron_index') and hasattr(synapse_array, 'target_neuron_ids'):
                for src_neuron, synapse_indices in synapse_array.source_neuron_index.items():
                    for syn_idx in synapse_indices:
                        if syn_idx < len(synapse_array.target_neuron_ids):
                            target_neuron = synapse_array.target_neuron_ids[syn_idx]
                            
                            if target_neuron in target_neurons:
                                # Found incoming synapse to isolated area!
                                suspicious_synapse = {
                                    'source_neuron': src_neuron,
                                    'target_neuron': int(target_neuron),
                                    'synapse_index': syn_idx,
                                    'weight': synapse_array.weights[syn_idx] if hasattr(synapse_array, 'weights') else 'unknown'
                                }
                                analysis['suspicious_synapses'].append(suspicious_synapse)
                                analysis['target_area_incoming'] += 1
                                
                                logger.error(f"🚨 SUSPICIOUS SYNAPSE: {src_neuron} -> {target_neuron}")
                                logger.error(f"   Weight: {suspicious_synapse['weight']}")
                                
            analysis['synapse_count'] = getattr(synapse_array, 'total_synapses', 0)
            
        except Exception as e:
            logger.error(f"❌ Failed to analyze synapse array: {e}")
            analysis['error'] = str(e)
            
        return analysis
        
    def generate_debug_report(self, all_results: Dict) -> str:
        """Generate comprehensive debug report."""
        report_lines = [
            "=" * 80,
            f"🚨 CORTICAL ISOLATION DEBUG REPORT - Area: {self.target_area_id}",
            "=" * 80,
            "",
            "SUMMARY OF FINDINGS:",
        ]
        
        # Mapping integrity
        mapping_result = all_results.get('mapping_integrity', {})
        issues = mapping_result.get('mapping_issues', [])
        if issues:
            report_lines.extend([
                "❌ CORTICAL MAPPING ISSUES DETECTED:",
                *[f"   - {issue}" for issue in issues],
                ""
            ])
        else:
            report_lines.append("✅ Cortical mapping integrity: OK")
            
        # Neuron ID mappings
        neuron_result = all_results.get('neuron_mappings', {})
        inconsistencies = neuron_result.get('mapping_inconsistencies', [])
        if inconsistencies:
            report_lines.extend([
                f"❌ NEURON MAPPING INCONSISTENCIES: {len(inconsistencies)} found",
                *[f"   - Neuron {inc['neuron_id']}: CM={inc.get('cm_index')}, NPU={inc.get('npu_index')}" 
                  for inc in inconsistencies[:10]],  # Show first 10
                ""
            ])
        else:
            report_lines.append("✅ Neuron ID mappings: OK")
            
        # FCL injections
        injection_result = all_results.get('fcl_injections', {})
        unexpected = injection_result.get('unexpected_injections', [])
        if unexpected:
            report_lines.extend([
                f"🚨 UNEXPECTED FCL INJECTIONS: {len(unexpected)} detected",
                "   This is the most likely cause of the activation corruption!",
                *[f"   - Timestep {inj['timestep']}: {inj['connection_count']} connections" 
                  for inj in unexpected[:5]],
                ""
            ])
        else:
            report_lines.append("✅ FCL injections: No unexpected injections detected")
            
        # Synapse array
        synapse_result = all_results.get('synapse_analysis', {})
        suspicious = synapse_result.get('suspicious_synapses', [])
        if suspicious:
            report_lines.extend([
                f"🚨 SUSPICIOUS SYNAPSES: {len(suspicious)} found",
                "   These synapses should not exist in an isolated area!",
                *[f"   - {syn['source_neuron']} -> {syn['target_neuron']} (weight: {syn['weight']})" 
                  for syn in suspicious[:10]],
                ""
            ])
        else:
            report_lines.append("✅ Synapse array: No suspicious connections found")
            
        report_lines.extend([
            "",
            "RECOMMENDED ACTIONS:",
            "1. If FCL injection issues found: Investigate synaptic propagation logic",
            "2. If synapse array issues found: Check connectome creation/modification code",
            "3. If mapping issues found: Investigate cortical area management",
            "4. Enable --debug-npu for detailed NPU logging",
            "",
            "=" * 80
        ])
        
        return "\n".join(report_lines)


def main():
    """Main debug entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Debug isolated cortical area activation')
    parser.add_argument('--area-id', required=True, help='Cortical area ID (e.g., cS7aaa)')
    parser.add_argument('--timesteps', type=int, default=5, help='Number of timesteps to simulate')
    parser.add_argument('--output', help='Output file for debug report')
    
    args = parser.parse_args()
    
    try:
        # Initialize FEAGI components
        logger.info("🚀 Initializing FEAGI debug session...")
        state_manager = FeagiStateManager.instance()
        state_manager.set_debug_npu_enabled(True)  # Enable detailed NPU logging
        
        # Get connectome manager and burst engine
        # NOTE: This would need to be adapted to your specific initialization
        # connectome_manager = ConnectomeManager.instance()
        # burst_engine = BurstEngine(connectome_manager, state_manager)
        
        # For now, show the usage pattern
        logger.info("To use this debugger:")
        logger.info("1. Initialize your FEAGI system")
        logger.info("2. Create debugger: debugger = CorticalIsolationDebugger(cm, burst_engine)")
        logger.info("3. Set target area: debugger.set_target_area('cS7aaa')")
        logger.info("4. Run investigation methods")
        
        print("\n" + "="*60)
        print("🔧 USAGE EXAMPLE:")
        print("="*60)
        print(f"""
# In your FEAGI environment:
from debug_cortical_mapping_isolation import CorticalIsolationDebugger

# Initialize debugger
debugger = CorticalIsolationDebugger(connectome_manager, burst_engine)
debugger.set_target_area('{args.area_id}')

# Run comprehensive investigation
results = {{}}
results['mapping_integrity'] = debugger.verify_cortical_mapping_integrity()
results['neuron_mappings'] = debugger.verify_neuron_id_mappings()
results['fcl_injections'] = debugger.trace_fcl_injection_sources({args.timesteps})
results['synapse_analysis'] = debugger.analyze_synapse_array_integrity()

# Generate report
report = debugger.generate_debug_report(results)
print(report)
""")
        
    except Exception as e:
        logger.error(f"❌ Debug session failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
