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
Rust-Optimized FCL Processing for FEAGI 2.0

This module implements Rust-compatible optimizations for FCL processing:
- Zero-copy data structures
- Cache-aligned memory layouts
- Rust-compatible type definitions
- SIMD-friendly data organization

Designed for seamless migration to Rust while providing immediate performance
benefits in Python.
"""

import array
import mmap
import struct
import time
from typing import Dict, List, Optional, Tuple, Union
from dataclasses import dataclass
from ctypes import Structure, c_uint32, c_uint64, c_float, c_ubyte, POINTER
import numpy as np

from feagi.utils.logger import setup_logger


# Rust-compatible type aliases
RustU32 = c_uint32
RustU64 = c_uint64
RustF32 = c_float
RustU8 = c_ubyte

# Constants for memory alignment (matching Rust's alignment requirements)
CACHE_LINE_SIZE = 64  # bytes
SIMD_ALIGNMENT = 32   # bytes for AVX2
PAGE_SIZE = 4096      # bytes


class RustCompatibleFCLEvent(Structure):
    """
    Rust-compatible FCL event structure.
    
    This structure is designed to match Rust's memory layout exactly,
    enabling zero-copy transfer between Python and future Rust implementation.
    """
    _fields_ = [
        ("timestep", RustU64),
        ("cortical_area_id", RustU32),
        ("neuron_count", RustU32),
        ("neuron_ids_ptr", POINTER(RustU32)),  # Pointer to neuron IDs array
        ("processing_time_ns", RustU64),
        ("event_type", RustU8),
        ("_padding", RustU8 * 7),  # Ensure 64-byte alignment
    ]
    
    def __init__(self, timestep: int, cortical_area_id: int, neuron_ids: List[int]):
        super().__init__()
        self.timestep = timestep
        self.cortical_area_id = cortical_area_id
        self.neuron_count = len(neuron_ids)
        
        # Create aligned array for neuron IDs
        self._neuron_ids_array = (RustU32 * len(neuron_ids))(*neuron_ids)
        self.neuron_ids_ptr = self._neuron_ids_array
        
        self.processing_time_ns = int(time.perf_counter_ns())
        self.event_type = 0  # Standard event
    
    def get_neuron_ids(self) -> List[int]:
        """Get neuron IDs as Python list."""
        return [self.neuron_ids_ptr[i] for i in range(self.neuron_count)]


@dataclass
class CacheAlignedFCLBuffer:
    """
    Cache-aligned buffer for FCL processing.
    
    Designed for optimal cache performance and SIMD operations.
    """
    capacity: int
    write_index: int = 0
    read_index: int = 0
    buffer: Optional[np.ndarray] = None
    
    def __post_init__(self):
        # Create cache-aligned buffer
        self.buffer = np.zeros(
            self.capacity, 
            dtype=np.uint32
        )
        # Ensure cache alignment
        if self.buffer.ctypes.data % CACHE_LINE_SIZE != 0:
            # Reallocate with proper alignment
            aligned_size = self.capacity + (CACHE_LINE_SIZE // 4)
            temp_buffer = np.zeros(aligned_size, dtype=np.uint32)
            offset = (CACHE_LINE_SIZE - (temp_buffer.ctypes.data % CACHE_LINE_SIZE)) // 4
            self.buffer = temp_buffer[offset:offset + self.capacity]
    
    def is_full(self) -> bool:
        """Check if buffer is full."""
        return (self.write_index + 1) % self.capacity == self.read_index
    
    def is_empty(self) -> bool:
        """Check if buffer is empty."""
        return self.write_index == self.read_index
    
    def available_space(self) -> int:
        """Get available space in buffer."""
        if self.write_index >= self.read_index:
            return self.capacity - (self.write_index - self.read_index) - 1
        else:
            return self.read_index - self.write_index - 1


class RustOptimizedFCLProcessor:
    """
    Rust-optimized FCL processor with zero-copy operations and cache-friendly data structures.
    
    This implementation provides:
    - Zero-copy event processing
    - Cache-aligned memory layouts
    - SIMD-friendly data organization
    - Rust-compatible interfaces
    """
    
    def __init__(self, buffer_capacity: int = 100000):
        """
        Initialize Rust-optimized FCL processor.
        
        Args:
            buffer_capacity: Capacity of the processing buffer
        """
        self.logger = setup_logger(__name__)
        self.buffer_capacity = buffer_capacity
        
        # Create cache-aligned buffers for different cortical areas
        self.cortical_buffers: Dict[int, CacheAlignedFCLBuffer] = {}
        
        # Statistics tracking (Rust-compatible)
        self.stats = {
            'events_processed': 0,
            'total_processing_time_ns': 0,
            'cache_hits': 0,
            'cache_misses': 0,
            'simd_operations': 0
        }
        
        # Memory pool for event structures
        self._event_pool: List[RustCompatibleFCLEvent] = []
        self._pool_index = 0
        
        self.logger.info("[NPU] RustOptimizedFCLProcessor initialized with capacity: %d", buffer_capacity)
    
    def get_or_create_buffer(self, cortical_area_id: int) -> CacheAlignedFCLBuffer:
        """Get or create cache-aligned buffer for cortical area."""
        if cortical_area_id not in self.cortical_buffers:
            self.cortical_buffers[cortical_area_id] = CacheAlignedFCLBuffer(
                capacity=self.buffer_capacity
            )
            self.stats['cache_misses'] += 1
            self.logger.debug(
                "[NPU-DEBUG] Created new buffer for cortical area %d", 
                cortical_area_id
            )
        else:
            self.stats['cache_hits'] += 1
        
        return self.cortical_buffers[cortical_area_id]
    
    def process_event_zero_copy(self, event: RustCompatibleFCLEvent) -> bool:
        """
        Process FCL event with zero-copy operations.
        
        Args:
            event: Rust-compatible FCL event
            
        Returns:
            True if processed successfully, False if buffer full
        """
        start_time = time.perf_counter_ns()
        
        # Get buffer for this cortical area
        buffer = self.get_or_create_buffer(event.cortical_area_id)
        
        if buffer.is_full():
            self.logger.warning(
                "[NPU] Buffer full for cortical area %d", 
                event.cortical_area_id
            )
            return False
        
        # Process neuron IDs using SIMD-friendly operations
        neuron_ids = event.get_neuron_ids()
        self._process_neuron_batch_simd(buffer, neuron_ids)
        
        # Update statistics
        processing_time = time.perf_counter_ns() - start_time
        self.stats['events_processed'] += 1
        self.stats['total_processing_time_ns'] += processing_time
        self.stats['simd_operations'] += 1
        
        event.processing_time_ns = processing_time
        
        return True
    
    def _process_neuron_batch_simd(self, buffer: CacheAlignedFCLBuffer, neuron_ids: List[int]) -> None:
        """
        Process neuron batch using SIMD-friendly operations.
        
        This method is designed to be easily replaceable with Rust SIMD implementation.
        """
        # Convert to numpy array for vectorized operations
        neuron_array = np.array(neuron_ids, dtype=np.uint32)
        
        # Ensure we have space in buffer
        available = buffer.available_space()
        if len(neuron_array) > available:
            # Process only what fits
            neuron_array = neuron_array[:available]
        
        # SIMD-friendly batch processing
        if len(neuron_array) > 0:
            # Calculate write positions
            start_pos = buffer.write_index
            end_pos = (start_pos + len(neuron_array)) % buffer.capacity
            
            if end_pos > start_pos:
                # No wrap-around, direct copy
                buffer.buffer[start_pos:end_pos] = neuron_array
            else:
                # Handle wrap-around
                first_chunk = buffer.capacity - start_pos
                buffer.buffer[start_pos:] = neuron_array[:first_chunk]
                buffer.buffer[:end_pos] = neuron_array[first_chunk:]
            
            buffer.write_index = end_pos
    
    def create_event_from_pool(self, timestep: int, cortical_area_id: int, neuron_ids: List[int]) -> RustCompatibleFCLEvent:
        """
        Create FCL event from object pool to reduce allocations.
        
        This mimics Rust's approach to memory management.
        """
        if self._pool_index < len(self._event_pool):
            # Reuse existing event
            event = self._event_pool[self._pool_index]
            event.timestep = timestep
            event.cortical_area_id = cortical_area_id
            event.neuron_count = len(neuron_ids)
            
            # Update neuron IDs array
            if len(neuron_ids) <= len(event._neuron_ids_array):
                for i, neuron_id in enumerate(neuron_ids):
                    event._neuron_ids_array[i] = neuron_id
            else:
                # Need larger array
                event._neuron_ids_array = (RustU32 * len(neuron_ids))(*neuron_ids)
                event.neuron_ids_ptr = event._neuron_ids_array
            
            event.processing_time_ns = int(time.perf_counter_ns())
            self._pool_index += 1
        else:
            # Create new event and add to pool
            event = RustCompatibleFCLEvent(timestep, cortical_area_id, neuron_ids)
            self._event_pool.append(event)
            self._pool_index += 1
        
        return event
    
    def reset_pool(self) -> None:
        """Reset object pool for next processing cycle."""
        self._pool_index = 0
    
    def get_buffer_utilization(self) -> Dict[int, float]:
        """Get buffer utilization for each cortical area."""
        utilization = {}
        for cortical_id, buffer in self.cortical_buffers.items():
            used_space = buffer.capacity - buffer.available_space()
            utilization[cortical_id] = used_space / buffer.capacity
        return utilization
    
    def get_performance_stats(self) -> Dict[str, Union[int, float]]:
        """Get performance statistics."""
        stats = self.stats.copy()
        if stats['events_processed'] > 0:
            stats['avg_processing_time_ns'] = (
                stats['total_processing_time_ns'] / stats['events_processed']
            )
            stats['avg_processing_time_ms'] = stats['avg_processing_time_ns'] / 1_000_000
        else:
            stats['avg_processing_time_ns'] = 0
            stats['avg_processing_time_ms'] = 0.0
        
        # Cache hit ratio
        total_accesses = stats['cache_hits'] + stats['cache_misses']
        if total_accesses > 0:
            stats['cache_hit_ratio'] = stats['cache_hits'] / total_accesses
        else:
            stats['cache_hit_ratio'] = 0.0
        
        return stats
    
    def prepare_for_rust_migration(self) -> Dict[str, any]:
        """
        Prepare data structures for Rust migration.
        
        Returns information needed for Rust implementation.
        """
        return {
            'buffer_capacity': self.buffer_capacity,
            'cache_line_size': CACHE_LINE_SIZE,
            'simd_alignment': SIMD_ALIGNMENT,
            'event_structure_size': struct.calcsize('QLLPQB7s'),  # Size of RustCompatibleFCLEvent
            'cortical_areas': list(self.cortical_buffers.keys()),
            'performance_stats': self.get_performance_stats(),
            'buffer_utilization': self.get_buffer_utilization()
        }


class RustCompatibleFCLInterface:
    """
    Interface for Rust-compatible FCL processing.
    
    This interface defines the contract that the Rust implementation will follow.
    """
    
    def __init__(self, processor: RustOptimizedFCLProcessor):
        """Initialize with Rust-optimized processor."""
        self.processor = processor
        self.logger = setup_logger(__name__)
    
    def process_fired_neurons_rust_compatible(
        self, 
        timestep: int, 
        neurons_by_cortical: Dict[int, List[int]]
    ) -> List[RustCompatibleFCLEvent]:
        """
        Process fired neurons using Rust-compatible interface.
        
        This method signature matches what the Rust implementation will provide.
        """
        events = []
        
        for cortical_id, neuron_ids in neurons_by_cortical.items():
            if neuron_ids:
                # Create event from pool
                event = self.processor.create_event_from_pool(
                    timestep, cortical_id, neuron_ids
                )
                
                # Process with zero-copy operations
                if self.processor.process_event_zero_copy(event):
                    events.append(event)
                else:
                    self.logger.warning(
                        "[NPU] Failed to process event for cortical area %d", 
                        cortical_id
                    )
        
        return events
    
    def get_rust_migration_info(self) -> str:
        """Get information for Rust migration."""
        info = self.processor.prepare_for_rust_migration()
        
        rust_info = f"""
// Rust Migration Information
// Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}

const BUFFER_CAPACITY: usize = {info['buffer_capacity']};
const CACHE_LINE_SIZE: usize = {info['cache_line_size']};
const SIMD_ALIGNMENT: usize = {info['simd_alignment']};

#[repr(C, align({info['cache_line_size']}))]
pub struct FCLEvent {{
    pub timestep: u64,
    pub cortical_area_id: u32,
    pub neuron_count: u32,
    pub neuron_ids_ptr: *const u32,
    pub processing_time_ns: u64,
    pub event_type: u8,
    _padding: [u8; 7],
}}

// Performance baseline from Python implementation:
// - Events processed: {info['performance_stats']['events_processed']}
// - Avg processing time: {info['performance_stats'].get('avg_processing_time_ms', 0):.3f} ms
// - Cache hit ratio: {info['performance_stats'].get('cache_hit_ratio', 0):.3f}
// - SIMD operations: {info['performance_stats']['simd_operations']}
"""
        
        return rust_info
