"""
PlasticityService - independent thread that computes plasticity every burst.

RTOS-friendly design:
- No sleeps/timeouts; uses a condition variable signaled by BurstEngine
- Read-only access to FireLedger; compute-only here
- Mutations are enqueued to NPUInterface as bounded commands (drop-on-full)

Enhanced with memory formation:
- RoaringBitmap-based pattern detection
- Memory neuron lifecycle management
- Deterministic pattern hashing with SHA-256
- CPU-optimized with minimal GPU interaction
"""

from typing import Any, Dict, List, Optional, Set
from dataclasses import dataclass
import threading
import numpy as np

from .orchestrator import PlasticityOrchestrator
from .stdp import STDPConfig
from .memory import MemoryConfig
from .pattern_detector import BatchPatternDetector, PatternConfig
from .memory_neuron_array import MemoryNeuronArray, MemoryNeuronLifecycleConfig


@dataclass
class PlasticityConfig:
    queue_capacity: int
    max_ops_per_burst: int
    stdp: Dict[str, Any]
    memory: Dict[str, Any]


class PlasticityService:
    def __init__(
        self,
        fire_ledger,
        npu_interface,
        plasticity_config: PlasticityConfig,
        state_manager=None,
        memory_neuron_array: Optional[MemoryNeuronArray] = None,
    ) -> None:
        self._ledger = fire_ledger
        self._npu = npu_interface
        self._orchestrator = PlasticityOrchestrator(fire_ledger)
        self._cfg = plasticity_config
        self._state_manager = state_manager

        self._cv = threading.Condition()
        self._latest_timestep = -1
        self._running = False

        # Configure NPU bounded queue
        self._npu.configure_plasticity_queue(self._cfg.queue_capacity)

        # Prebuild configs (strict; if missing, computations will be skipped)
        self._stdp_cfg = STDPConfig(
            lookback_steps=int(self._cfg.stdp["lookback_steps"]),
            tau_pre=float(self._cfg.stdp["tau_pre"]),
            tau_post=float(self._cfg.stdp["tau_post"]),
            a_plus=float(self._cfg.stdp["a_plus"]),
            a_minus=float(self._cfg.stdp["a_minus"]),
            max_pairs_per_synapse=8,
        ) if self._cfg.stdp else None

        self._mem_cfg = MemoryConfig(
            lookback_steps=int(self._cfg.memory["lookback_steps"]),
            pattern_duration=int(self._cfg.memory["pattern_duration"]),
            min_activation_count=int(self._cfg.memory["min_activation_count"]),
        ) if self._cfg.memory else None

        # Memory formation components
        self._memory_neuron_array = memory_neuron_array or MemoryNeuronArray()
        
        # Pattern detection system
        pattern_config = PatternConfig(
            default_temporal_depth=self._cfg.memory.get("default_temporal_depth", 3) if self._cfg.memory else 3,
            min_activity_threshold=self._cfg.memory.get("min_activation_count", 1) if self._cfg.memory else 1,
            max_pattern_cache_size=self._cfg.memory.get("pattern_cache_size", 10000) if self._cfg.memory else 10000,
        )
        self._pattern_detector = BatchPatternDetector(pattern_config)
        
        # Memory area tracking
        self._memory_areas: Dict[int, Dict] = {}  # area_idx -> config
        self._memory_lifecycle_configs: Dict[int, MemoryNeuronLifecycleConfig] = {}
        
        # Statistics
        self._stats = {
            'memory_patterns_detected': 0,
            'memory_neurons_created': 0,
            'memory_neurons_reactivated': 0,
            'memory_neurons_aged': 0,
            'memory_neurons_converted_ltm': 0,
            'plasticity_commands_enqueued': 0,
            'plasticity_commands_dropped': 0,
        }

    # ==== BurstEngine hook ====
    def notify_burst(self, timestep: int) -> None:
        with self._cv:
            self._latest_timestep = int(timestep)
            self._cv.notify()

    def start(self) -> threading.Thread:
        self._running = True
        t = threading.Thread(target=self._run, daemon=True)
        t.start()
        return t

    def stop(self) -> None:
        self._running = False
        with self._cv:
            self._cv.notify_all()

    def _run(self) -> None:
        while self._running:
            with self._cv:
                # Wait for a burst notification
                self._cv.wait()
                ts = self._latest_timestep

            if not self._running:
                break

            # Compute and enqueue STDP commands every burst
            try:
                self._compute_enqueue_stdp()
            except Exception:
                pass

            # Compute and enqueue memory commands every burst
            try:
                self._compute_enqueue_memory()
            except Exception:
                pass

    def _compute_enqueue_stdp(self) -> None:
        if self._stdp_cfg is None:
            return
        syn = getattr(self._npu, 'synapse_array', None)
        if syn is None or getattr(syn, 'count', 0) <= 0:
            return

        count = int(syn.count)
        # Only plastic synapses
        mask = syn.is_plastic_flags[:count]
        if not np.any(mask):
            return
        idxs = np.nonzero(mask)[0].astype(np.int32)
        src = syn.source_neuron_ids[idxs].astype(np.int32, copy=False)
        tgt = syn.target_neuron_ids[idxs].astype(np.int32, copy=False)

        # Group by (area_src, area_tgt)
        groups = PlasticityOrchestrator.group_synapses_by_area_pairs(src, tgt, getattr(self._npu, 'neuron_to_area', {}))
        if not groups:
            return

        # Accumulate deltas deterministically by group key order
        all_indices: List[int] = []
        all_deltas: List[float] = []

        for key in sorted(groups.keys()):
            s_area, t_area = key
            gidx = groups[key]
            gsrc = src[gidx]
            gtgt = tgt[gidx]
            timing = self._orchestrator.stdp_activity(
                inputs=type('PI', (), {
                    'syn_source_ids': gsrc,
                    'syn_target_ids': gtgt,
                    'source_cortical_idx': s_area,
                    'target_cortical_idx': t_area,
                })(),
                stdp_cfg=self._stdp_cfg,
            )[0]  # use activity factors (or switch to timing_factors if desired)

            coeffs = syn.plasticity_coeffs[idxs[gidx]].astype(np.float32, copy=False)
            deltas = (timing.astype(np.float32, copy=False)) * coeffs

            all_indices.extend(idxs[gidx].tolist())
            all_deltas.extend(deltas.tolist())

        if not all_indices:
            return

        # Single command (counts as 1 op in queue) with coalesced deltas
        cmd = {
            'type': 'update_weights_delta',
            'indices': np.asarray(all_indices, dtype=np.int32),
            'deltas': np.asarray(all_deltas, dtype=np.float32),
        }
        self._npu.enqueue_plasticity_commands([cmd])

    def _compute_enqueue_memory(self) -> None:
        """
        Compute memory formation commands using advanced pattern detection.
        
        This method:
        1. Ages all existing memory neurons
        2. Detects temporal patterns in configured memory areas
        3. Creates new memory neurons for novel patterns
        4. Reactivates existing neurons for known patterns
        5. Converts eligible neurons to long-term memory
        """
        import sys
        debug_mem = '--debug-mem' in sys.argv
        
        if debug_mem:
            print(f"[DEBUG-MEM] _compute_enqueue_memory called - mem_cfg: {self._mem_cfg is not None}, memory_areas: {len(self._memory_areas)}")
            if self._memory_areas:
                print(f"[DEBUG-MEM] Registered memory areas: {list(self._memory_areas.keys())}")
                for area_idx, config in self._memory_areas.items():
                    print(f"[DEBUG-MEM]   Area {area_idx}: temporal_depth={config['temporal_depth']}, upstream_areas={config['upstream_areas']}")
            else:
                print("[DEBUG-MEM] ❌ NO MEMORY AREAS REGISTERED - this is why no memory neurons are created!")
        
        if self._mem_cfg is None or not self._memory_areas:
            if debug_mem:
                print(f"[DEBUG-MEM] Early return - mem_cfg is None: {self._mem_cfg is None}, no memory areas: {not self._memory_areas}")
            return
        
        current_timestep = self._latest_timestep
        if current_timestep < 0:
            if debug_mem:
                print(f"[DEBUG-MEM] Invalid timestep: {current_timestep}")
            return
        
        if debug_mem:
            print(f"[DEBUG-MEM] Processing memory at timestep {current_timestep}")
        
        try:
            # Step 1: Age all memory neurons (vectorized operation)
            died_neurons = self._memory_neuron_array.age_memory_neurons(current_timestep)
            if died_neurons:
                self._stats['memory_neurons_aged'] += len(died_neurons)
            
            # Step 2: Check for long-term memory conversion
            converted_neurons = self._memory_neuron_array.check_longterm_conversion()
            if converted_neurons:
                self._stats['memory_neurons_converted_ltm'] += len(converted_neurons)
            
            # Step 3: Detect patterns for all memory areas in batch
            patterns = self._pattern_detector.detect_patterns_batch(
                self._ledger, self._memory_areas, current_timestep
            )
            
            if debug_mem:
                print(f"[DEBUG-MEM] Pattern detection results: {len(patterns)} areas processed")
                for area_idx, pattern in patterns.items():
                    if pattern:
                        print(f"[DEBUG-MEM]   Area {area_idx}: Pattern detected (hash: {pattern.pattern_hash.hex()[:8]}...)")
                        print(f"[DEBUG-MEM]     Pattern details: temporal_depth={pattern.temporal_depth}, total_activity={pattern.total_activity}")
                        
                        # Show the actual pattern data for debugging
                        upstream_areas = self._memory_areas.get(area_idx, {}).get('upstream_areas', [])
                        print(f"[DEBUG-MEM]     Upstream areas: {upstream_areas}")
                        
                        # Get recent activity from fire ledger to show what pattern was detected
                        for i in range(pattern.temporal_depth):
                            timestep_to_check = current_timestep - i
                            if timestep_to_check >= 0:
                                activity_summary = []
                                for upstream_area in upstream_areas:
                                    activity = self._ledger.get_area_activity(upstream_area, timestep_to_check)
                                    if activity and len(activity) > 0:
                                        activity_summary.append(f"area_{upstream_area}:{len(activity)}neurons")
                                if activity_summary:
                                    print(f"[DEBUG-MEM]       T-{i} (timestep {timestep_to_check}): {', '.join(activity_summary)}")
                                else:
                                    print(f"[DEBUG-MEM]       T-{i} (timestep {timestep_to_check}): no activity")
                    else:
                        print(f"[DEBUG-MEM]   Area {area_idx}: No pattern detected")
            
            # Step 4: Process detected patterns
            commands = []
            for memory_area_idx, pattern in patterns.items():
                if pattern is None:
                    if debug_mem:
                        print(f"[DEBUG-MEM]   Area {memory_area_idx}: Skipping - no pattern")
                    continue
                
                if debug_mem:
                    print(f"[DEBUG-MEM]   Area {memory_area_idx}: Processing pattern (hash: {pattern.pattern_hash.hex()[:8]}...)")
                
                self._stats['memory_patterns_detected'] += 1
                
                # Check if pattern already has a memory neuron
                existing_neuron_idx = self._memory_neuron_array.find_neuron_by_pattern(
                    pattern.pattern_hash
                )
                
                if debug_mem:
                    if existing_neuron_idx is not None:
                        neuron_id = self._memory_neuron_array.neuron_ids[existing_neuron_idx]
                        is_active = self._memory_neuron_array.is_active[existing_neuron_idx]
                        lifespan = self._memory_neuron_array.lifespan_current[existing_neuron_idx]
                        activation_count = self._memory_neuron_array.activation_count[existing_neuron_idx]
                        print(f"[DEBUG-MEM]     Found existing neuron at index {existing_neuron_idx} (ID: {neuron_id})")
                        print(f"[DEBUG-MEM]       Active: {is_active}, Lifespan: {lifespan}, Activations: {activation_count}")
                    else:
                        print(f"[DEBUG-MEM]     No existing neuron found - will create new one")
                        
                        # Show current memory neuron array state
                        total_memory_neurons = sum(1 for i in range(self._memory_neuron_array.next_available_index) 
                                                 if self._memory_neuron_array.is_active[i])
                        print(f"[DEBUG-MEM]     Current memory array: {total_memory_neurons} active neurons, next_index: {self._memory_neuron_array.next_available_index}")
                
                if existing_neuron_idx is not None:
                    # Reactivate existing memory neuron
                    success = self._memory_neuron_array.reactivate_memory_neuron(
                        existing_neuron_idx, current_timestep
                    )
                    if success:
                        self._stats['memory_neurons_reactivated'] += 1
                        
                        # Get the neuron ID for FCL injection
                        neuron_id = self._memory_neuron_array.neuron_ids[existing_neuron_idx]
                        
                        # CRITICAL: Add reactivated memory neuron to FCL so it can fire
                        commands.append({
                            'type': 'inject_memory_neuron_to_fcl',
                            'neuron_id': int(neuron_id),
                            'area_idx': memory_area_idx,
                            'membrane_potential': 1.5,  # Above threshold to ensure firing
                            'pattern_hash': pattern.pattern_hash.hex(),
                            'is_reactivation': True,
                            'timestep': current_timestep,
                        })
                else:
                    # Create new memory neuron for novel pattern
                    lifecycle_config = self._memory_lifecycle_configs.get(
                        memory_area_idx, 
                        MemoryNeuronLifecycleConfig()
                    )
                    
                    neuron_idx = self._memory_neuron_array.create_memory_neuron(
                        pattern_hash=pattern.pattern_hash,
                        cortical_area_id=memory_area_idx,
                        current_burst=current_timestep,
                        config=lifecycle_config
                    )
                    
                    if neuron_idx is not None:
                        self._stats['memory_neurons_created'] += 1
                        
                        # Get the neuron ID for FCL injection and state updates
                        neuron_id = self._memory_neuron_array.neuron_ids[neuron_idx]
                        
                        # CRITICAL: Register memory neuron in regular neuron array for neural dynamics
                        commands.append({
                            'type': 'register_memory_neuron_in_regular_array',
                            'neuron_id': int(neuron_id),
                            'area_idx': memory_area_idx,
                            'threshold': 1.0,  # Firing threshold
                            'membrane_potential': 0.0,  # Initial membrane potential
                            'coordinates': [0, 0, 0],  # Default coordinates
                        })
                        
                        # CRITICAL: Add memory neuron to FCL so it can fire in next burst
                        commands.append({
                            'type': 'inject_memory_neuron_to_fcl',
                            'neuron_id': int(neuron_id),
                            'area_idx': memory_area_idx,
                            'membrane_potential': 1.5,  # Above threshold to ensure firing
                            'pattern_hash': pattern.pattern_hash.hex(),
                            'temporal_depth': pattern.temporal_depth,
                            'total_activity': pattern.total_activity,
                            'timestep': current_timestep,
                        })
                        
                        # CRITICAL: Update state manager neuron counters
                        commands.append({
                            'type': 'update_state_counters',
                            'memory_neurons_created': 1,
                            'total_neurons_created': 1,
                            'area_idx': memory_area_idx,
                            'neuron_id': int(neuron_id),
                        })
            
            # Step 5: Enqueue commands (with drop-on-full policy)
            if commands:
                if debug_mem:
                    print(f"[DEBUG-MEM] Enqueueing {len(commands)} plasticity commands:")
                    for i, cmd in enumerate(commands):
                        print(f"[DEBUG-MEM]   Command {i+1}: {cmd.get('type', 'unknown')} (neuron_id: {cmd.get('neuron_id', 'N/A')})")
                
                try:
                    self._npu.enqueue_plasticity_commands(commands)
                    self._stats['plasticity_commands_enqueued'] += len(commands)
                    if debug_mem:
                        print(f"[DEBUG-MEM] ✅ Successfully enqueued {len(commands)} commands")
                except Exception as e:
                    # Commands dropped due to queue saturation
                    self._stats['plasticity_commands_dropped'] += len(commands)
                    if self._state_manager:
                        self._state_manager.increment_plasticity_dropped_ops(len(commands))
                    if debug_mem:
                        print(f"[DEBUG-MEM] ❌ Failed to enqueue commands: {e}")
            else:
                if debug_mem:
                    print(f"[DEBUG-MEM] No commands to enqueue")
        
        except Exception as e:
            # Robust error handling - don't crash the service
            if self._state_manager:
                self._state_manager.increment_plasticity_dropped_ops(1)
    
    def register_memory_area(
        self,
        area_idx: int,
        temporal_depth: int,
        upstream_areas: List[int],
        lifecycle_config: Optional[MemoryNeuronLifecycleConfig] = None
    ) -> bool:
        """
        Register a memory area for pattern detection.
        
        Args:
            area_idx: Memory area cortical index
            temporal_depth: Number of timesteps for pattern detection
            upstream_areas: List of upstream cortical areas
            lifecycle_config: Memory neuron lifecycle configuration
            
        Returns:
            True if successfully registered
        """
        try:
            # Configure fire ledger for this memory area
            self._ledger.configure_memory_area(area_idx, temporal_depth, upstream_areas)
            
            # Register with pattern detector
            self._memory_areas[area_idx] = {
                'temporal_depth': temporal_depth,
                'upstream_areas': upstream_areas,
            }
            
            # Store lifecycle configuration
            self._memory_lifecycle_configs[area_idx] = (
                lifecycle_config or MemoryNeuronLifecycleConfig()
            )
            
            return True
            
        except Exception:
            return False
    
    def get_memory_stats(self) -> Dict:
        """Get comprehensive memory formation statistics."""
        memory_array_stats = self._memory_neuron_array.get_stats()
        pattern_detector_stats = self._pattern_detector.get_batch_stats()
        
        return {
            'service_stats': self._stats.copy(),
            'memory_array_stats': {
                'total_capacity': memory_array_stats.total_capacity,
                'active_neurons': memory_array_stats.active_neurons,
                'longterm_neurons': memory_array_stats.longterm_neurons,
                'dead_neurons': memory_array_stats.dead_neurons,
                'avg_lifespan': memory_array_stats.avg_lifespan,
                'avg_activation_count': memory_array_stats.avg_activation_count,
                'memory_usage_bytes': memory_array_stats.memory_usage_bytes,
            },
            'pattern_detector_stats': pattern_detector_stats,
            'registered_memory_areas': len(self._memory_areas),
            'memory_area_configs': {
                area_idx: config.copy() 
                for area_idx, config in self._memory_areas.items()
            },
        }
    
    def get_memory_neuron_array(self) -> MemoryNeuronArray:
        """Get direct access to memory neuron array."""
        return self._memory_neuron_array


