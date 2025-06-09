"""
Embedded-Optimized Burst Engine

This module provides a burst engine specifically optimized for embedded
single-core operation targeting 10M neuron operations at 15Hz. It integrates
with the embedded-optimized connectome manager and addresses all critical
performance bottlenecks.

Key Optimizations:
1. Single-threaded operation for embedded systems
2. Minimal overhead burst processing
3. SIMD-optimized neural computation
4. Cache-friendly memory access patterns
5. Zero-allocation operation paths
"""

import os
import time
import threading
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import logging

from feagi.bdu.embedded_optimized_connectome import (
    EmbeddedOptimizedConnectomeManager,
    EmbeddedConfig
)
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

@dataclass
class EmbeddedBurstConfig:
    """Configuration for embedded burst engine."""
    target_frequency_hz: float = 15.0
    max_burst_time_ms: float = 66.7  # 1000/15 Hz
    embedded_mode: bool = True
    single_threaded: bool = True
    cpu_affinity: Optional[int] = 0  # Pin to core 0 for embedded
    debug_performance: bool = False
    performance_logging_interval: int = 100  # Log every N bursts

class EmbeddedBurstEngine:
    """
    Ultra-high-performance burst engine for embedded single-core operation.
    
    Designed to achieve 10M neuron operations at 15Hz by:
    - Minimal overhead burst processing
    - Single-threaded operation with CPU affinity
    - SIMD-optimized neural computation
    - Zero-allocation operation paths
    - Real-time performance monitoring
    """
    
    def __init__(self, burst_config: Optional[EmbeddedBurstConfig] = None,
                 connectome_config: Optional[EmbeddedConfig] = None):
        """Initialize embedded burst engine."""
        self.burst_config = burst_config or EmbeddedBurstConfig()
        self.connectome_config = connectome_config or EmbeddedConfig()
        
        # Ensure consistency between configs
        self.connectome_config.target_frequency_hz = self.burst_config.target_frequency_hz
        self.connectome_config.max_burst_time_ms = self.burst_config.max_burst_time_ms
        self.connectome_config.single_threaded = self.burst_config.single_threaded
        
        # Initialize embedded-optimized connectome manager
        self.connectome_manager = EmbeddedOptimizedConnectomeManager(self.connectome_config)
        
        # Burst timing and control
        self.burst_count = 0
        self.total_burst_time = 0.0
        self.running = False
        self.burst_thread = None
        
        # Performance tracking
        self.performance_history = []
        self.target_burst_interval = 1.0 / self.burst_config.target_frequency_hz
        
        # CPU affinity for embedded systems
        if self.burst_config.embedded_mode and self.burst_config.cpu_affinity is not None:
            self._set_cpu_affinity()
        
        # Environment variables for embedded optimization
        if self.burst_config.embedded_mode:
            self._configure_embedded_environment()
        
        logger.info(f"EmbeddedBurstEngine initialized: "
                   f"{self.burst_config.target_frequency_hz}Hz target, "
                   f"embedded mode: {self.burst_config.embedded_mode}")
    
    def _set_cpu_affinity(self):
        """Set CPU affinity to single core for embedded operation."""
        try:
            if hasattr(os, 'sched_setaffinity'):
                # Linux
                os.sched_setaffinity(0, {self.burst_config.cpu_affinity})
                logger.info(f"CPU affinity set to core {self.burst_config.cpu_affinity}")
            else:
                logger.warning("CPU affinity not supported on this platform")
        except Exception as e:
            logger.warning(f"Failed to set CPU affinity: {e}")
    
    def _configure_embedded_environment(self):
        """Configure environment variables for embedded optimization."""
        # Disable threading in various libraries
        os.environ['OMP_NUM_THREADS'] = '1'
        os.environ['MKL_NUM_THREADS'] = '1'
        os.environ['NUMEXPR_NUM_THREADS'] = '1'
        os.environ['OPENBLAS_NUM_THREADS'] = '1'
        
        # Set embedded mode flags
        os.environ['FEAGI_EMBEDDED_MODE'] = 'true'
        os.environ['FEAGI_DISABLE_ZMQ'] = 'true'
        os.environ['FEAGI_SINGLE_THREADED'] = 'true'
        
        # Memory optimization
        os.environ['MALLOC_TRIM_THRESHOLD_'] = '128000'  # More aggressive memory trimming
        
        logger.info("Embedded environment configured for single-core operation")
    
    def create_neuron(self, cortical_id: str, position: tuple, **kwargs) -> int:
        """Create a neuron using the embedded-optimized connectome manager."""
        return self.connectome_manager.create_neuron(cortical_id, position, **kwargs)
    
    def add_connection(self, source_id: int, target_id: int, weight: float):
        """Add synaptic connection between neurons."""
        self.connectome_manager.add_connection(source_id, target_id, weight)
    
    def create_cortical_area(self, name: str, dimensions: tuple, **kwargs):
        """Create a cortical area."""
        return self.connectome_manager.create_cortical_area(name, dimensions, **kwargs)
    
    def batch_create_neurons(self, cortical_id: str, positions: List[tuple], **kwargs) -> List[int]:
        """Create multiple neurons efficiently in batch."""
        return self.connectome_manager.batch_create_neurons(cortical_id, positions, **kwargs)
    
    def process_single_burst(self) -> Dict[str, Any]:
        """
        Process a single burst optimized for embedded operation.
        
        This is the critical path that must execute in <66.7ms for 10M neurons at 15Hz.
        
        Returns:
            Dictionary with burst results and performance metrics
        """
        burst_start = time.perf_counter()
        
        # Execute embedded-optimized membrane potential updates
        fired_neuron_ids = self.connectome_manager.update_membrane_potentials()
        
        # Calculate timing
        burst_time = time.perf_counter() - burst_start
        
        # Update statistics
        self.burst_count += 1
        self.total_burst_time += burst_time
        
        # Build results
        results = {
            'burst_id': self.burst_count,
            'fired_neurons': fired_neuron_ids,
            'neuron_count': self.connectome_manager.get_neuron_count(),
            'burst_time_ms': burst_time * 1000,
            'target_time_ms': self.burst_config.max_burst_time_ms,
            'target_achieved': burst_time < (self.burst_config.max_burst_time_ms / 1000),
            'frequency_hz': 1.0 / burst_time if burst_time > 0 else 0,
            'target_frequency_hz': self.burst_config.target_frequency_hz,
            'performance_metrics': self.connectome_manager.performance_metrics.__dict__.copy()
        }
        
        # Store performance history
        self.performance_history.append({
            'burst_id': self.burst_count,
            'burst_time_ms': burst_time * 1000,
            'neurons_fired': len(fired_neuron_ids),
            'target_achieved': results['target_achieved']
        })
        
        # Periodic performance logging
        if (self.burst_config.debug_performance and 
            self.burst_count % self.burst_config.performance_logging_interval == 0):
            self._log_performance_summary()
        
        return results
    
    def start_burst_loop(self, duration_seconds: Optional[float] = None):
        """
        Start the embedded burst loop in a separate thread.
        
        Args:
            duration_seconds: Duration to run (None for indefinite)
        """
        if self.running:
            logger.warning("Burst engine is already running")
            return
        
        self.running = True
        
        def burst_loop():
            """Main burst processing loop optimized for embedded operation."""
            start_time = time.perf_counter()
            next_burst_time = start_time
            
            while self.running:
                # Check if we should stop based on duration
                if duration_seconds and (time.perf_counter() - start_time) >= duration_seconds:
                    break
                
                # Calculate timing for next burst
                current_time = time.perf_counter()
                
                if current_time >= next_burst_time:
                    # Process burst
                    self.process_single_burst()
                    
                    # Schedule next burst
                    next_burst_time += self.target_burst_interval
                    
                    # Handle overruns - don't let schedule drift
                    if next_burst_time <= current_time:
                        next_burst_time = current_time + self.target_burst_interval
                else:
                    # Precise timing wait for embedded systems
                    sleep_time = next_burst_time - current_time
                    if sleep_time > 0.001:  # Only sleep if >1ms
                        time.sleep(sleep_time)
            
            self.running = False
            logger.info("Burst loop stopped")
        
        # Start burst thread
        self.burst_thread = threading.Thread(target=burst_loop, name="EmbeddedBurstEngine")
        self.burst_thread.daemon = True
        self.burst_thread.start()
        
        logger.info(f"Burst engine started: {self.burst_config.target_frequency_hz}Hz target")
    
    def stop_burst_loop(self):
        """Stop the burst processing loop."""
        if not self.running:
            logger.warning("Burst engine is not running")
            return
        
        self.running = False
        
        if self.burst_thread and self.burst_thread.is_alive():
            self.burst_thread.join(timeout=2.0)
        
        logger.info("Burst engine stopped")
    
    def _log_performance_summary(self):
        """Log comprehensive performance summary."""
        if not self.performance_history:
            return
        
        recent_history = self.performance_history[-self.burst_config.performance_logging_interval:]
        
        # Calculate statistics
        avg_burst_time = sum(p['burst_time_ms'] for p in recent_history) / len(recent_history)
        max_burst_time = max(p['burst_time_ms'] for p in recent_history)
        min_burst_time = min(p['burst_time_ms'] for p in recent_history)
        target_achieved_rate = sum(1 for p in recent_history if p['target_achieved']) / len(recent_history)
        
        # Get connectome performance summary
        connectome_perf = self.connectome_manager.get_performance_summary()
        
        logger.info(f"EMBEDDED PERFORMANCE SUMMARY [Burst {self.burst_count}]:")
        logger.info(f"  Burst Timing: avg={avg_burst_time:.2f}ms, "
                   f"min={min_burst_time:.2f}ms, max={max_burst_time:.2f}ms")
        logger.info(f"  Target Achievement: {target_achieved_rate*100:.1f}% "
                   f"({connectome_perf['avg_burst_time_ms']:.2f}ms avg overall)")
        logger.info(f"  Neural Processing: {connectome_perf['total_neurons']} neurons, "
                   f"{connectome_perf['megaops_per_second']:.1f} MOps/sec")
        logger.info(f"  SIMD Optimization: {connectome_perf['simd_type']}, "
                   f"width={connectome_perf['simd_width']}, "
                   f"efficiency={connectome_perf['latest_simd_efficiency']:.2f}")
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get comprehensive performance summary."""
        if not self.performance_history:
            return {}
        
        # Calculate overall statistics
        all_burst_times = [p['burst_time_ms'] for p in self.performance_history]
        avg_burst_time = sum(all_burst_times) / len(all_burst_times)
        target_achieved_count = sum(1 for p in self.performance_history if p['target_achieved'])
        target_achieved_rate = target_achieved_count / len(self.performance_history)
        
        # Get connectome performance
        connectome_perf = self.connectome_manager.get_performance_summary()
        
        # Calculate 10M neuron progress
        current_neurons = connectome_perf['total_neurons']
        target_neurons = 10_000_000
        progress_percent = (current_neurons / target_neurons) * 100
        
        # Estimate performance at 10M neurons
        if avg_burst_time > 0 and current_neurons > 0:
            estimated_10m_time = avg_burst_time * (target_neurons / current_neurons)
        else:
            estimated_10m_time = 0
        
        summary = {
            # Burst engine metrics
            'burst_count': self.burst_count,
            'total_runtime_s': self.total_burst_time,
            'avg_burst_time_ms': avg_burst_time,
            'target_frequency_hz': self.burst_config.target_frequency_hz,
            'target_time_ms': self.burst_config.max_burst_time_ms,
            'target_achieved_rate': target_achieved_rate,
            'embedded_mode': self.burst_config.embedded_mode,
            
            # Neural processing metrics
            'current_neurons': current_neurons,
            'target_neurons': target_neurons,
            'progress_percent': progress_percent,
            'operations_per_second': connectome_perf.get('operations_per_second', 0),
            'megaops_per_second': connectome_perf.get('megaops_per_second', 0),
            
            # 10M neuron projection
            'estimated_10m_time_ms': estimated_10m_time,
            'projected_10m_achievable': estimated_10m_time < self.burst_config.max_burst_time_ms,
            
            # Optimization details
            'simd_type': connectome_perf.get('simd_type', 'Unknown'),
            'simd_width': connectome_perf.get('simd_width', 0),
            'block_sparse_size': connectome_perf.get('block_sparse_size', 0),
            'memory_pools_enabled': connectome_perf.get('memory_pools_enabled', False),
            
            # Latest performance
            'latest_burst_time_ms': connectome_perf.get('latest_burst_time_ms', 0),
            'latest_neurons_fired': connectome_perf.get('latest_neurons_fired', 0),
            'latest_simd_efficiency': connectome_perf.get('latest_simd_efficiency', 0)
        }
        
        return summary
    
    def run_performance_test(self, duration_seconds: float = 10.0) -> Dict[str, Any]:
        """
        Run a performance test for the specified duration.
        
        Args:
            duration_seconds: Duration to run the test
            
        Returns:
            Performance test results
        """
        logger.info(f"Starting embedded performance test: {duration_seconds}s duration")
        
        # Clear previous performance history
        self.performance_history.clear()
        self.burst_count = 0
        self.total_burst_time = 0.0
        
        # Run burst loop for specified duration
        self.start_burst_loop(duration_seconds)
        
        # Wait for completion
        if self.burst_thread:
            self.burst_thread.join()
        
        # Generate test results
        results = self.get_performance_summary()
        results['test_duration_s'] = duration_seconds
        results['bursts_completed'] = self.burst_count
        results['actual_frequency_hz'] = self.burst_count / duration_seconds if duration_seconds > 0 else 0
        results['frequency_accuracy'] = results['actual_frequency_hz'] / self.burst_config.target_frequency_hz
        
        logger.info(f"Performance test completed: {self.burst_count} bursts, "
                   f"{results['avg_burst_time_ms']:.2f}ms avg, "
                   f"{results['target_achieved_rate']*100:.1f}% target achievement")
        
        return results
    
    def clear_brain_data(self):
        """Clear all brain data for new genome loading."""
        self.connectome_manager.clear_brain_data()
        
        # Reset performance tracking
        self.performance_history.clear()
        self.burst_count = 0
        self.total_burst_time = 0.0
        
        logger.info("Brain data cleared, ready for new genome")
    
    def get_neuron_count(self) -> int:
        """Get total number of neurons."""
        return self.connectome_manager.get_neuron_count()
    
    def get_neuron(self, neuron_id: int) -> Dict[str, Any]:
        """Get neuron information."""
        return self.connectome_manager.get_neuron(neuron_id)
    
    def get_cortical_area(self, area_id: str):
        """Get cortical area information."""
        return self.connectome_manager.get_cortical_area(area_id)
    
    # Compatibility properties and methods
    
    @property
    def neuron_count(self) -> int:
        """Compatibility property for neuron count."""
        return self.get_neuron_count()
    
    @property 
    def debug_npu(self) -> bool:
        """Compatibility property for debug mode."""
        return self.burst_config.debug_performance
    
    def burst(self) -> Dict[str, Any]:
        """Single burst execution for compatibility."""
        return self.process_single_burst()