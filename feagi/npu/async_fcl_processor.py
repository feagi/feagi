"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Asynchronous FCL Processor for FEAGI 2.0

This module implements high-performance asynchronous Fire Candidate List (FCL) 
processing that runs in parallel with BDU membrane potential updates.

Key Features:
- Non-blocking FCL updates using asyncio
- Event-driven communication with BDU
- Optimized for embedded systems and Rust migration
- Thread-safe queue-based architecture
"""

import asyncio
import threading
import time
from collections import deque
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

from feagi.utils.logger import setup_logger
from feagi.npu.interfaces import FiredNeuronEvent, NPUFCLInterface
from feagi.npu.rust_optimized_fcl import (
    RustOptimizedFCLProcessor, 
    RustCompatibleFCLInterface,
    RustCompatibleFCLEvent
)


@dataclass
class FCLProcessingStats:
    """Statistics for FCL processing performance."""
    total_events_processed: int = 0
    average_processing_time_ms: float = 0.0
    queue_depth_max: int = 0
    queue_depth_current: int = 0
    last_update_timestamp: float = 0.0


class AsyncFCLProcessor(NPUFCLInterface):
    """
    Asynchronous FCL processor that handles fired neuron events in parallel
    with BDU membrane potential updates.
    
    This design enables:
    1. BDU to continue processing without waiting for FCL updates
    2. NPU to batch and optimize FCL operations
    3. Clean separation for future Rust migration
    """
    
    def __init__(self, fcl_manager, max_queue_size: int = 10000, enable_rust_optimization: bool = True):
        """
        Initialize async FCL processor.
        
        Args:
            fcl_manager: The FCL manager instance to delegate operations to
            max_queue_size: Maximum number of events in the processing queue
            enable_rust_optimization: Enable Rust-compatible optimizations
        """
        self.logger = setup_logger(__name__)
        self.fcl_manager = fcl_manager
        self.max_queue_size = max_queue_size
        self.enable_rust_optimization = enable_rust_optimization
        
        # Event queue for BDU-NPU communication
        self._event_queue: deque = deque(maxlen=max_queue_size)
        self._queue_lock = threading.Lock()
        
        # Async processing control
        self._processing_task: Optional[asyncio.Task] = None
        self._stop_event = asyncio.Event()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        
        # Performance tracking
        self.stats = FCLProcessingStats()
        
        # Processing thread
        self._processing_thread: Optional[threading.Thread] = None
        self._thread_stop_event = threading.Event()
        
        # Rust optimization components
        if self.enable_rust_optimization:
            self.rust_processor = RustOptimizedFCLProcessor(buffer_capacity=max_queue_size)
            self.rust_interface = RustCompatibleFCLInterface(self.rust_processor)
            self.logger.info("[NPU] Rust optimization enabled with cache-aligned buffers")
        else:
            self.rust_processor = None
            self.rust_interface = None
        
        self.logger.info("[NPU] AsyncFCLProcessor initialized with queue size: %d, rust_opt: %s", 
                        max_queue_size, enable_rust_optimization)
    
    def start(self) -> None:
        """Start the asynchronous FCL processing."""
        if self._processing_thread is not None:
            self.logger.warning("[NPU] AsyncFCLProcessor already running")
            return
        
        self._thread_stop_event.clear()
        self._processing_thread = threading.Thread(
            target=self._run_async_loop,
            name="AsyncFCLProcessor",
            daemon=True
        )
        self._processing_thread.start()
        self.logger.info("[NPU] AsyncFCLProcessor started")
    
    def stop(self) -> None:
        """Stop the asynchronous FCL processing."""
        if self._processing_thread is None:
            return
        
        self.logger.info("[NPU] Stopping AsyncFCLProcessor...")
        self._thread_stop_event.set()
        
        if self._loop and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(self._stop_event.set)
        
        self._processing_thread.join(timeout=2.0)
        if self._processing_thread.is_alive():
            self.logger.warning("[NPU] AsyncFCLProcessor thread did not stop gracefully")
        
        self._processing_thread = None
        self.logger.info("[NPU] AsyncFCLProcessor stopped")
    
    def _run_async_loop(self) -> None:
        """Run the asyncio event loop in a separate thread."""
        try:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            
            self._processing_task = self._loop.create_task(self._process_events())
            self._loop.run_until_complete(self._processing_task)
            
        except Exception as e:
            self.logger.error("[NPU] Error in async FCL processing loop: %s", e)
        finally:
            if self._loop and not self._loop.is_closed():
                self._loop.close()
    
    async def _process_events(self) -> None:
        """Main event processing loop."""
        self.logger.info("[NPU] FCL event processing loop started")
        
        while not self._stop_event.is_set():
            try:
                # Process events in batches for efficiency
                events = self._get_event_batch()
                
                if events:
                    start_time = time.perf_counter()
                    await self._process_event_batch(events)
                    processing_time = (time.perf_counter() - start_time) * 1000
                    
                    # Update statistics
                    self._update_stats(len(events), processing_time)
                    
                    if self.logger.isEnabledFor(10):  # DEBUG level
                        self.logger.debug(
                            "[NPU-DEBUG] Processed %d FCL events in %.2fms",
                            len(events), processing_time
                        )
                else:
                    # No events to process, yield control
                    await asyncio.sleep(0.001)  # 1ms sleep
                    
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error("[NPU] Error processing FCL events: %s", e)
                await asyncio.sleep(0.01)  # Brief pause on error
        
        self.logger.info("[NPU] FCL event processing loop stopped")
    
    def _get_event_batch(self, max_batch_size: int = 100) -> List[FiredNeuronEvent]:
        """Get a batch of events from the queue."""
        events = []
        
        with self._queue_lock:
            batch_size = min(max_batch_size, len(self._event_queue))
            for _ in range(batch_size):
                if self._event_queue:
                    events.append(self._event_queue.popleft())
                else:
                    break
        
        return events
    
    async def _process_event_batch(self, events: List[FiredNeuronEvent]) -> None:
        """Process a batch of fired neuron events."""
        if not events:
            return
        
        # Group events by timestep for efficient processing
        events_by_timestep: Dict[int, List[FiredNeuronEvent]] = {}
        for event in events:
            if event.timestep not in events_by_timestep:
                events_by_timestep[event.timestep] = []
            events_by_timestep[event.timestep].append(event)
        
        # Process each timestep
        for timestep, timestep_events in events_by_timestep.items():
            # Merge all neurons_by_cortical from events in this timestep
            merged_neurons_by_cortical: Dict[int, List[int]] = {}
            for event in timestep_events:
                for cortical_id, neuron_ids in event.neurons_by_cortical.items():
                    if cortical_id not in merged_neurons_by_cortical:
                        merged_neurons_by_cortical[cortical_id] = []
                    merged_neurons_by_cortical[cortical_id].extend(neuron_ids)
            
            # Delegate to FCL manager (this is the actual FCL update)
            if self.fcl_manager and merged_neurons_by_cortical:
                if self.rust_interface:
                    # Use Rust-optimized processing
                    await asyncio.get_event_loop().run_in_executor(
                        None,
                        self._process_with_rust_optimization,
                        timestep,
                        merged_neurons_by_cortical
                    )
                else:
                    # Standard FCL processing
                    await asyncio.get_event_loop().run_in_executor(
                        None,
                        self.fcl_manager.update_fcl,
                        timestep,
                        merged_neurons_by_cortical
                    )
    
    def _update_stats(self, events_processed: int, processing_time_ms: float) -> None:
        """Update processing statistics."""
        self.stats.total_events_processed += events_processed
        
        # Update rolling average processing time
        alpha = 0.1  # Smoothing factor
        if self.stats.average_processing_time_ms == 0:
            self.stats.average_processing_time_ms = processing_time_ms
        else:
            self.stats.average_processing_time_ms = (
                alpha * processing_time_ms + 
                (1 - alpha) * self.stats.average_processing_time_ms
            )
        
        with self._queue_lock:
            self.stats.queue_depth_current = len(self._event_queue)
            self.stats.queue_depth_max = max(
                self.stats.queue_depth_max, 
                self.stats.queue_depth_current
            )
        
        self.stats.last_update_timestamp = time.time()
    
    def _process_with_rust_optimization(self, timestep: int, neurons_by_cortical: Dict[int, List[int]]) -> None:
        """Process FCL update using Rust optimizations."""
        if not self.rust_interface:
            # Fallback to standard processing
            self.fcl_manager.update_fcl(timestep, neurons_by_cortical)
            return
        
        # Use Rust-compatible interface
        rust_events = self.rust_interface.process_fired_neurons_rust_compatible(
            timestep, neurons_by_cortical
        )
        
        # Still need to update the original FCL manager for compatibility
        if rust_events:
            self.fcl_manager.update_fcl(timestep, neurons_by_cortical)
            
            # Log Rust optimization stats if debug enabled
            if self.logger.isEnabledFor(10):  # DEBUG level
                rust_stats = self.rust_processor.get_performance_stats()
                self.logger.debug(
                    "[NPU-DEBUG] Rust optimization: %d events, %.3fms avg, %.1f%% cache hit",
                    len(rust_events),
                    rust_stats.get('avg_processing_time_ms', 0),
                    rust_stats.get('cache_hit_ratio', 0) * 100
                )
    
    # NPUFCLInterface implementation
    def process_fired_neurons(self, event: FiredNeuronEvent) -> None:
        """
        Process fired neurons asynchronously.
        
        This method is called by BDU and immediately returns, allowing
        BDU to continue processing while NPU handles FCL updates.
        """
        if not event.neurons_by_cortical:
            return
        
        # Add event to queue for async processing
        with self._queue_lock:
            if len(self._event_queue) >= self.max_queue_size:
                # Queue is full, drop oldest event (backpressure handling)
                dropped_event = self._event_queue.popleft()
                dropped_neurons = sum(len(neurons) for neurons in dropped_event.neurons_by_cortical.values())
                self.logger.warning(
                    "[NPU] FCL queue full, dropped event with %d neurons",
                    dropped_neurons
                )
            
            self._event_queue.append(event)
        
        # Log debug info if enabled
        if self.logger.isEnabledFor(10):  # DEBUG level
            total_neurons = sum(len(neurons) for neurons in event.neurons_by_cortical.values())
            cortical_areas = list(event.neurons_by_cortical.keys())
            self.logger.debug(
                "[NPU-DEBUG] Queued FCL event: %d neurons from cortical areas %s",
                total_neurons, cortical_areas
            )
    
    def get_processing_stats(self) -> FCLProcessingStats:
        """Get current processing statistics."""
        with self._queue_lock:
            self.stats.queue_depth_current = len(self._event_queue)
        return self.stats
    
    def is_queue_healthy(self) -> bool:
        """Check if the processing queue is healthy (not overloaded)."""
        with self._queue_lock:
            queue_utilization = len(self._event_queue) / self.max_queue_size
            return queue_utilization < 0.8  # Healthy if less than 80% full
    
    def get_rust_optimization_stats(self) -> Optional[Dict[str, any]]:
        """Get Rust optimization statistics if enabled."""
        if not self.rust_processor:
            return None
        
        return {
            'enabled': True,
            'performance_stats': self.rust_processor.get_performance_stats(),
            'buffer_utilization': self.rust_processor.get_buffer_utilization(),
            'migration_info': self.rust_processor.prepare_for_rust_migration()
        }
    
    def generate_rust_code(self) -> Optional[str]:
        """Generate Rust code template for migration."""
        if not self.rust_interface:
            return None
        
        return self.rust_interface.get_rust_migration_info()
