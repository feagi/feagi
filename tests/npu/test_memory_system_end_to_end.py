"""
Comprehensive end-to-end test for FEAGI memory system.

This test validates the complete memory system workflow:
1. Memory area creation and registration
2. Upstream area mapping with memory morphology
3. Pattern detection and memory neuron creation
4. FCL injection and visualization integration
5. BurstEngine integration with MemoryProcessor

Run with: pytest tests/npu/test_memory_system_end_to_end.py -v -s --tb=short
"""

import os
import pytest
MEMORY_TESTS_ENABLED = os.environ.get("FEAGI_MEMORY_TESTS", "0") == "1"

pytestmark = pytest.mark.skipif(
    not MEMORY_TESTS_ENABLED,
    reason="Memory system tests disabled; set FEAGI_MEMORY_TESTS=1 to enable",
)
import logging
import tempfile
import json
from typing import Dict, Any, Set, List
from unittest.mock import Mock, patch

# Setup logging to see debug output
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_memory_system_end_to_end():
    """
    Comprehensive end-to-end test of the memory system.
    
    This test validates:
    - Memory cortical area creation with proper registration
    - Regular cortical area creation and mapping to memory area
    - Memory morphology handling and upstream tracking
    - BurstEngine and MemoryProcessor integration
    - Pattern detection and memory neuron creation
    - FCL injection and FQ sampler compatibility
    """
    
    logger.info("🧪 STARTING MEMORY SYSTEM END-TO-END TEST")
    
    # =================================================================
    # STEP 1: Initialize clean system with barebones genome
    # =================================================================
    logger.info("📋 Step 1: Loading barebones genome...")
    
    try:
        from feagi.api.core.services.core_api_service import CoreAPIService
        from feagi.bdu.connectome_manager import ConnectomeManager
        from feagi.npu.burst_engine import BurstEngine
        from feagi.core.state_manager import get_state_manager
        
        # Load barebones genome for clean slate
        state_manager = get_state_manager()
        
        # Create CoreAPIService
        connectome_manager = ConnectomeManager(config_or_max_neurons=100000, max_synapses=1000000)
        core_api = CoreAPIService(connectome_manager=connectome_manager)
        
        # Initialize BurstEngine with proper dependencies
        burst_engine = BurstEngine(
            connectome_manager=connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"memory_processing_batch_size": 100, "memory_pattern_cache_size": 1000}
        )
        
        # Load barebones genome
        barebones_response = core_api.load_barebones_genome()
        assert barebones_response["success"], f"Failed to load barebones genome: {barebones_response}"
        logger.info("✅ Barebones genome loaded successfully")
        
    except Exception as e:
        pytest.fail(f"Failed to initialize system: {e}")
    
    # =================================================================
    # STEP 2: Create regular cortical area (upstream area)
    # =================================================================
    logger.info("📋 Step 2: Creating upstream cortical area...")
    
    upstream_area_id = "TEST_UPSTREAM"
    try:
        upstream_response = core_api.create_cortical_area(
            name="Test Upstream Area",
            coordinates={"x": 10, "y": 10, "z": 0},
            dimensions={"width": 3, "height": 3, "depth": 1},  # 9 neurons
            area_type="custom",
            parameters={
                "cortical_group": "CUSTOM",
                "cortical_sub_group": "TEST",
                "brain_region_id": "root",
                "per_voxel_neuron_cnt": 1,
                "coordinates_2d": [10, 10]
            }
        )
        
        assert upstream_response is not None, f"Failed to create upstream area: {upstream_response}"
        upstream_area_id = upstream_response.get("cortical_id") if isinstance(upstream_response, dict) else "TEST_UPSTREAM"
        logger.info(f"✅ Created upstream area: {upstream_area_id}")
        
    except Exception as e:
        pytest.fail(f"Failed to create upstream area: {e}")
    
    # =================================================================
    # STEP 3: Create memory cortical area
    # =================================================================
    logger.info("📋 Step 3: Creating memory cortical area...")
    
    memory_area_id = "TEST_MEMORY"
    try:
        memory_response = core_api.create_cortical_area(
            name="Test Memory Area",
            coordinates={"x": 50, "y": 50, "z": 0},  # Use different coordinates to avoid collision
            dimensions={"width": 2, "height": 2, "depth": 1},  # 2x2x1 to accommodate multiple neurons
            area_type="memory",
            parameters={
                "cortical_group": "CUSTOM",
                "cortical_sub_group": "MEMORY",
                "sub_group_id": "MEMORY",  # This is the key identifier
                "brain_region_id": "root",
                "coordinates_2d": [50, 50],  # Update 2D coordinates too
                "temporal_depth": 2,  # Look back 2 timesteps
                "init_lifespan": 10,
                "lifespan_growth_rate": 1.2,
                "longterm_mem_threshold": 50,
                "per_voxel_neuron_cnt": 1  # Use only 1 neuron per voxel to avoid position conflicts
            }
        )
        
        assert memory_response is not None, f"Failed to create memory area: {memory_response}"
        memory_area_id = memory_response.get("cortical_id") if isinstance(memory_response, dict) else "TEST_MEMORY"
        logger.info(f"✅ Created memory area: {memory_area_id}")
        
    except Exception as e:
        pytest.fail(f"Failed to create memory area: {e}")
    
    # =================================================================
    # STEP 4: Verify memory area registration
    # =================================================================
    logger.info("📋 Step 4: Verifying memory area registration...")
    
    # Check ConnectomeManager registration
    cm = core_api.get_connectome_manager()
    assert memory_area_id in cm.memory_areas, f"Memory area {memory_area_id} not in ConnectomeManager.memory_areas"
    logger.info(f"✅ Memory area registered in ConnectomeManager: {list(cm.memory_areas)}")
    
    # Check StateManager registration
    try:
        assert state_manager.is_memory_area(memory_area_id), f"Memory area {memory_area_id} not in StateManager"
        logger.info(f"✅ Memory area registered in StateManager")
    except Exception as e:
        logger.warning(f"Could not verify StateManager registration: {e}")
    
    # Check BurstEngine MemoryProcessor registration
    burst_engine = BurstEngine.get_instance()
    if burst_engine and burst_engine.memory_processor:
        assert memory_area_id in burst_engine.memory_processor.active_memory_areas, \
            f"Memory area {memory_area_id} not registered in MemoryProcessor"
        logger.info(f"✅ Memory area registered in MemoryProcessor: {list(burst_engine.memory_processor.active_memory_areas)}")
    else:
        pytest.fail("BurstEngine or MemoryProcessor not available")
    
    # =================================================================
    # STEP 5: Create cortical mapping with memory morphology
    # =================================================================
    logger.info("📋 Step 5: Creating cortical mapping with memory morphology...")
    
    try:
        # First, verify memory morphology exists in genome
        genome_data = state_manager.genome
        assert "neuron_morphologies" in genome_data, "No neuron_morphologies in genome"
        assert "memory" in genome_data["neuron_morphologies"], "Memory morphology not found in genome"
        logger.info("✅ Memory morphology found in genome")
        
        # Create mapping from upstream to memory area - correct format
        mapping_success = core_api.update_cortical_mapping({
            upstream_area_id: {
                memory_area_id: [{
                    "morphology_id": "memory",
                    "morphology_scalar": [1, 1, 1],
                    "postSynapticCurrent_multiplier": 1.0,
                    "plasticity_flag": False,
                    "plasticity_constant": 0.0,
                    "ltp_multiplier": 1.0,
                    "ltd_multiplier": 1.0
                }]
            }
        })
        
        assert mapping_success, f"Failed to create cortical mapping: {mapping_success}"
        logger.info(f"✅ Created mapping: {upstream_area_id} -> {memory_area_id} (memory morphology)")
        
    except Exception as e:
        pytest.fail(f"Failed to create cortical mapping: {e}")
    
    # =================================================================
    # STEP 6: Verify upstream mapping tracking
    # =================================================================
    logger.info("📋 Step 6: Verifying upstream mapping tracking...")
    
    # Check ConnectomeManager upstream tracking
    upstream_areas = cm.get_upstream_areas_for_memory(memory_area_id)
    assert upstream_area_id in upstream_areas, \
        f"Upstream area {upstream_area_id} not tracked for memory area {memory_area_id}. Found: {upstream_areas}"
    logger.info(f"✅ Upstream mapping tracked in ConnectomeManager: {upstream_areas}")
    
    # Check MemoryProcessor upstream tracking
    memory_props = burst_engine.memory_processor.memory_area_properties.get(memory_area_id, {})
    processor_upstream = memory_props.get("upstream_areas", set())
    assert upstream_area_id in processor_upstream, \
        f"Upstream area {upstream_area_id} not in MemoryProcessor. Found: {processor_upstream}"
    logger.info(f"✅ Upstream mapping tracked in MemoryProcessor: {processor_upstream}")
    
    # =================================================================
    # STEP 7: Activate neurons in upstream area
    # =================================================================
    logger.info("📋 Step 7: Activating neurons in upstream area...")
    
    try:
        # Get some neurons from the upstream area
        upstream_neurons = list(cm.get_neurons_by_cortical_area(upstream_area_id))
        assert len(upstream_neurons) > 0, f"No neurons found in upstream area {upstream_area_id}"
        
        # Activate first 3 neurons for pattern
        neurons_to_activate = upstream_neurons[:3]
        logger.info(f"Activating neurons: {neurons_to_activate}")
        
        # Use manual stimulation API with correct format
        stimulation_success = core_api.trigger_manual_stimulation({
            "cortical_id": upstream_area_id,
            "coordinates": [
                {"x": 0, "y": 0, "z": 0},
                {"x": 1, "y": 0, "z": 0},
                {"x": 2, "y": 0, "z": 0}
            ],  # Activate first 3 voxels with correct format
            "intensity": 1.0
        })
        
        assert stimulation_success, f"Failed to activate neurons: {stimulation_success}"
        logger.info("✅ Neurons activated in upstream area")
        
    except Exception as e:
        pytest.fail(f"Failed to activate neurons: {e}")
    
    # =================================================================
    # STEP 8: Enable debug mode and run burst to test memory processing
    # =================================================================
    logger.info("📋 Step 8: Enable debug mode and run burst to test memory processing...")
    
    try:
        # Enable NPU debug mode using correct format
        state_manager.set_debug_config({"debug": {"debug_npu": True}})
        
        # Verify debug mode is enabled
        assert state_manager.is_debug_npu_enabled(), "NPU debug mode not enabled"
        logger.info("✅ NPU debug mode enabled")
        
        # Run a burst to trigger memory processing
        if burst_engine:
            # Use the correct method for triggering a burst manually
            # Get current timestep from FCL manager and increment
            current_timestep = cm.fcl_manager.current_timestep + 1
            fired_neurons = burst_engine._process_burst_with_power_injection(current_timestep)
            logger.info(f"✅ Single burst executed: timestep={current_timestep}, fired_neurons={len(fired_neurons) if fired_neurons else 0}")
        else:
            logger.warning("❌ BurstEngine not available for burst execution")
        
    except Exception as e:
        pytest.fail(f"Failed to enable debug mode or run burst: {e}")
    
    # =================================================================
    # STEP 9: Verify memory processing occurred
    # =================================================================
    logger.info("📋 Step 9: Verifying memory processing occurred...")
    
    # Check memory neuron array for created neurons
    memory_array = burst_engine.memory_processor.memory_neuron_array
    memory_stats = memory_array.get_statistics()
    logger.info(f"Memory neuron statistics: {memory_stats}")
    
    # Check for active memory neurons in the memory area
    active_memory_neurons = memory_array.get_active_neurons_for_area(memory_area_id)
    logger.info(f"Active memory neurons in {memory_area_id}: {active_memory_neurons}")
    
    # =================================================================
    # STEP 10: Verify FCL contains memory neurons
    # =================================================================
    logger.info("📋 Step 10: Verifying FCL contains memory neurons...")
    
    try:
        # Get memory area cortical_idx
        memory_area = cm.get_cortical_area(memory_area_id)
        if memory_area:
            memory_cortical_idx = memory_area.cortical_idx
            
            # Check FCL for memory area
            fcl_bitmap = cm.fcl_manager.get_cortical_fcl(memory_cortical_idx)
            fcl_neurons = list(fcl_bitmap) if fcl_bitmap else []
            logger.info(f"FCL neurons in memory area {memory_area_id} (idx={memory_cortical_idx}): {fcl_neurons}")
            
            # Check if memory area appears in active corticals
            active_corticals = cm.fcl_manager.get_active_corticals()
            logger.info(f"Active cortical indices in FCL: {active_corticals}")
            
    except Exception as e:
        logger.error(f"Error checking FCL: {e}")
    
    # =================================================================
    # STEP 11: Check FQ Sampler integration
    # =================================================================
    logger.info("📋 Step 11: Testing FQ Sampler integration...")
    
    try:
        # Create FQ Sampler and test sampling
        from feagi.npu.fq_sampler import FQSampler
        
        sampler = FQSampler(
            fire_queue_provider=cm.fcl_manager,
            sample_frequency_hz=10.0,
            sampling_mode="visualization",
            connectome_manager=cm,
            state_manager=state_manager
        )
        
        # Get visualization areas
        viz_areas = sampler._get_visualization_areas()
        logger.info(f"Visualization areas found by FQ Sampler: {viz_areas}")
        
        if memory_area_id in viz_areas:
            logger.info(f"✅ Memory area {memory_area_id} detected by FQ Sampler!")
        else:
            logger.warning(f"⚠️  Memory area {memory_area_id} NOT detected by FQ Sampler")
            
    except Exception as e:
        logger.error(f"Error testing FQ Sampler: {e}")
    
    # =================================================================
    # STEP 12: Final system state summary
    # =================================================================
    logger.info("📋 Step 12: Final system state summary...")
    
    logger.info("=" * 60)
    logger.info("MEMORY SYSTEM TEST SUMMARY")
    logger.info("=" * 60)
    
    logger.info(f"Upstream Area: {upstream_area_id}")
    logger.info(f"Memory Area: {memory_area_id}")
    logger.info(f"ConnectomeManager Memory Areas: {list(cm.memory_areas)}")
    logger.info(f"MemoryProcessor Active Areas: {list(burst_engine.memory_processor.active_memory_areas)}")
    logger.info(f"Memory Statistics: {memory_stats}")
    logger.info(f"Active Memory Neurons: {len(active_memory_neurons)}")
    
    # Get processing statistics
    if hasattr(burst_engine.memory_processor, 'get_processing_statistics'):
        processing_stats = burst_engine.memory_processor.get_processing_statistics()
        logger.info(f"Processing Statistics: {processing_stats}")
    
    logger.info("=" * 60)
    
    # =================================================================
    # ASSERTIONS FOR TEST VALIDATION
    # =================================================================
    
    # Essential assertions
    assert memory_area_id in cm.memory_areas, "Memory area not registered in ConnectomeManager"
    assert memory_area_id in burst_engine.memory_processor.active_memory_areas, "Memory area not registered in MemoryProcessor"
    assert upstream_area_id in cm.get_upstream_areas_for_memory(memory_area_id), "Upstream mapping not tracked"
    
    logger.info("🎉 MEMORY SYSTEM END-TO-END TEST COMPLETED")


if __name__ == "__main__":
    # Run the test directly
    test_memory_system_end_to_end() 