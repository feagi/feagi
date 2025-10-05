"""
Example usage of Clean FEAGI NPU Architecture

This example shows how all NPU components work together:
- FCL Injector: Injects candidates into Fire Candidate List
- Burst Engine: Orchestrates neural processing
- FQ Sampler: Samples firing neurons from Fire Queue
- Fire Ledger: Manages historical data

Clean data flow: FCL → Fire Queue → Fire Ledger
"""

import numpy as np
from typing import Dict, Any

# Import clean NPU components
from feagi.npu import (
    BurstEngine,
    FCLInjector,
    FQSampler,
    FireQueue,
    FireLedgerInterface,
    CoordinateConverter
)

def example_usage():
    """Complete example of clean NPU usage."""
    
    print("🧠 FEAGI NPU - Clean Architecture Example")
    print("=" * 50)
    
    # Mock connectome manager for example
    class MockConnectomeManager:
        def __init__(self):
            self.cortical_areas = {
                'visual': type('Area', (), {'cortical_idx': 1})(),
                'motor': type('Area', (), {'cortical_idx': 2})(),
            }
    
    connectome_manager = MockConnectomeManager()
    
    # === Step 1: Initialize Burst Engine ===
    print("1. Initializing Burst Engine...")
    burst_engine = BurstEngine(
        connectome_manager=connectome_manager,
        fire_ledger_window_size=20
    )
    
    # === Step 2: Initialize FQ Sampler ===
    print("2. Initializing FQ Sampler...")
    fq_sampler = burst_engine.initialize_fq_sampler(
        sample_frequency_hz=10.0,
        sampling_mode="visualization"
    )
    
    # === Step 3: Demonstrate Clean Data Flow ===
    print("3. Demonstrating clean data flow...")
    
    # Phase A: External injection (sensory data)
    print("   → Injecting sensory data...")
    sensory_result = inject_sensory_example(burst_engine)
    
    # Phase B: Burst processing
    print("   → Processing burst...")
    fired_neurons = burst_engine.process_burst()
    print(f"     • {len(fired_neurons)} neurons fired")
    
    # Phase C: FQ Sampling
    print("   → Sampling Fire Queue...")
    sample_result = fq_sampler.sample()
    if sample_result:
        total_sampled = sum(len(data.get('neuron_ids', [])) for data in sample_result.values())
        print(f"     • Sampled {total_sampled} neurons from {len(sample_result)} areas")
    
    # Phase D: Historical access
    print("   → Accessing Fire Ledger...")
    fire_ledger = burst_engine.get_fire_ledger()
    stats = fire_ledger.get_statistics()
    print(f"     • Fire Ledger: {stats['tracked_areas']} areas tracked")
    
    print("\n✅ Clean NPU architecture demonstration complete!")
    print_architecture_benefits()

def inject_sensory_example(burst_engine: BurstEngine) -> int:
    """Example of sensory data injection using SoA format."""
    
    # Example: Visual cortex stimulation
    # SoA format: [x,y,z,p] → [i,p] conversion
    x_coords = np.array([10, 11, 12], dtype=np.int32)
    y_coords = np.array([20, 21, 22], dtype=np.int32)  
    z_coords = np.array([0, 0, 0], dtype=np.int32)
    potentials = np.array([1.2, 1.5, 0.8], dtype=np.float32)
    
    # Inject via burst engine interface
    return burst_engine.inject_sensory_data(
        cortical_id='visual',
        x_coords=x_coords,
        y_coords=y_coords,
        z_coords=z_coords,
        potentials=potentials
    )

def print_architecture_benefits():
    """Print the benefits of the clean architecture."""
    
    print("\n🎯 Clean Architecture Benefits:")
    print("   • Single responsibility per component")
    print("   • No timing conflicts or mixed data")
    print("   • Rust-friendly SoA format")
    print("   • Clear separation: FCL → Fire Queue → Fire Ledger")
    print("   • FQ Sampler reads from Fire Queue (not historical)")
    print("   • FCL Injector handles all external stimuli")
    print("   • Fire Ledger manages STDP and memory formation")

def demonstrate_component_isolation():
    """Show how components can be tested independently."""
    
    print("\n🔧 Component Isolation Example:")
    
    # Example 1: FCL Injector standalone
    from feagi.npu import FCLInjector, CoordinateConverter, FireCandidateList
    
    mock_cm = type('CM', (), {})()
    converter = CoordinateConverter(mock_cm)
    injector = FCLInjector(converter)
    fcl = FireCandidateList()
    
    print("   • FCL Injector: ✅ Standalone testable")
    
    # Example 2: FQ Sampler standalone  
    from feagi.npu import FQSampler, FireQueue
    
    mock_fire_queue = FireQueue()
    sampler = FQSampler(mock_fire_queue)
    
    print("   • FQ Sampler: ✅ Standalone testable")
    
    # Example 3: Fire Ledger standalone
    from feagi.npu import FireLedgerInterface
    
    ledger = FireLedgerInterface(window_size=10)
    
    print("   • Fire Ledger: ✅ Standalone testable")
    print("   • Each component has SINGLE responsibility!")

if __name__ == "__main__":
    example_usage()
    demonstrate_component_isolation()
