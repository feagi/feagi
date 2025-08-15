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
NPU Interface Definitions for FEAGI 2.0

This module defines the interfaces and data structures for communication
between the BDU (Brain Development Unit) and NPU (Neural Processing Unit).

The primary purpose is to establish clean architectural boundaries that
support parallel processing and Rust migration.
"""

import asyncio
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Optional, Protocol, runtime_checkable
from enum import Enum

# Type aliases for clarity and Rust migration compatibility
NeuronId = int
CorticalIdx = int
Timestep = int


class FCLEventType(Enum):
    """Types of FCL events for different processing modes."""
    STANDARD = "standard"  # Normal synaptic propagation
    INJECTION = "injection"  # Special area injection (power, modulator, etc.)
    MEMORY = "memory"  # Memory cortical area processing


@dataclass
class FiredNeuronEvent:
    """Event containing fired neuron data for NPU processing.
    
    This data structure is designed for:
    - Zero-copy transfer (in future Rust implementation)
    - Efficient serialization across process boundaries
    - Clear temporal ordering and cortical area grouping
    """
    timestep: Timestep
    neurons_by_cortical: Dict[CorticalIdx, List[NeuronId]]
    event_type: FCLEventType = FCLEventType.STANDARD
    source_module: str = "bdu"
    
    def __post_init__(self):
        """Validate event data after initialization."""
        if self.timestep < 0:
            raise ValueError(f"Invalid timestep: {self.timestep}")
        
        if not self.neurons_by_cortical:
            # Empty events are valid (no neurons fired)
            self.neurons_by_cortical = {}
        
        # Validate cortical indices and neuron IDs
        for cortical_idx, neuron_ids in self.neurons_by_cortical.items():
            if cortical_idx < 0:
                raise ValueError(f"Invalid cortical_idx: {cortical_idx}")
            if not isinstance(neuron_ids, list):
                raise ValueError(f"neuron_ids must be list, got {type(neuron_ids)}")
            for neuron_id in neuron_ids:
                if neuron_id < 0:
                    raise ValueError(f"Invalid neuron_id: {neuron_id}")

    @property
    def total_neurons(self) -> int:
        """Total number of neurons in this event."""
        return sum(len(neurons) for neurons in self.neurons_by_cortical.values())
    
    @property
    def cortical_areas(self) -> List[CorticalIdx]:
        """List of cortical areas involved in this event."""
        return list(self.neurons_by_cortical.keys())


@runtime_checkable
class NPUFCLInterface(Protocol):
    """Protocol defining the interface between BDU and NPU for FCL processing.
    
    This interface supports:
    - Async communication for parallel processing
    - Clean separation between neural computation (BDU) and activity tracking (NPU)
    - Future Rust migration with zero-copy semantics
    """
    
    async def process_fired_neurons(self, event: FiredNeuronEvent) -> None:
        """Process fired neurons from BDU membrane potential updates.
        
        Args:
            event: FiredNeuronEvent containing timestep and neuron data
            
        This method should:
        1. Update FCL data structures with fired neurons
        2. Group neurons by cortical area for efficient processing
        3. Maintain temporal history for visualization and analysis
        4. Handle special area processing (power, memory, etc.)
        """
        ...
    
    async def advance_timestep(self, new_timestep: Timestep) -> None:
        """Advance the NPU to the next timestep.
        
        Args:
            new_timestep: The new current timestep
            
        This method should:
        1. Advance FCL temporal windows
        2. Clean up old history data
        3. Prepare for next burst cycle
        """
        ...
    
    async def get_fcl_status(self) -> Dict[str, any]:
        """Get current FCL status for monitoring and debugging.
        
        Returns:
            Dictionary containing FCL statistics and status information
        """
        ...


class FCLEventProcessor:
    """Async processor for FCL events from BDU.
    
    This class implements the NPU side of the BDU → NPU communication,
    providing parallel processing of fired neuron events while maintaining
    temporal consistency.
    """
    
    def __init__(self, fcl_manager, max_queue_size: int = 1000):
        """Initialize the FCL event processor.
        
        Args:
            fcl_manager: The FCL manager instance to update
            max_queue_size: Maximum number of events to queue
        """
        self.fcl_manager = fcl_manager
        self.event_queue: asyncio.Queue[FiredNeuronEvent] = asyncio.Queue(maxsize=max_queue_size)
        self.current_timestep: Timestep = 0
        self.processing_task: Optional[asyncio.Task] = None
        self.is_running = False
        
        # Performance metrics
        self.events_processed = 0
        self.total_neurons_processed = 0
        self.processing_times = []
        
        # Error handling
        self.max_retries = 3
        self.retry_delay = 0.001  # 1ms
    
    async def start(self) -> None:
        """Start the async event processing loop."""
        if self.is_running:
            return
            
        self.is_running = True
        self.processing_task = asyncio.create_task(self._processing_loop())
    
    async def stop(self) -> None:
        """Stop the async event processing loop."""
        self.is_running = False
        if self.processing_task:
            self.processing_task.cancel()
            try:
                await self.processing_task
            except asyncio.CancelledError:
                pass
    
    async def submit_event(self, event: FiredNeuronEvent) -> None:
        """Submit a fired neuron event for processing.
        
        Args:
            event: FiredNeuronEvent to process
            
        Raises:
            asyncio.QueueFull: If event queue is full
        """
        if not self.is_running:
            raise RuntimeError("FCLEventProcessor is not running")
        
        # Validate event timestep ordering
        if event.timestep < self.current_timestep:
            raise ValueError(f"Event timestep {event.timestep} is behind current timestep {self.current_timestep}")
        
        await self.event_queue.put(event)
    
    async def _processing_loop(self) -> None:
        """Main async processing loop for FCL events."""
        while self.is_running:
            try:
                # Wait for next event with timeout
                event = await asyncio.wait_for(self.event_queue.get(), timeout=0.1)
                
                # Process the event
                start_time = asyncio.get_event_loop().time()
                await self._process_event(event)
                processing_time = asyncio.get_event_loop().time() - start_time
                
                # Update metrics
                self.events_processed += 1
                self.total_neurons_processed += event.total_neurons
                self.processing_times.append(processing_time)
                
                # Keep only recent processing times for performance monitoring
                if len(self.processing_times) > 1000:
                    self.processing_times = self.processing_times[-500:]
                
                # Mark task as done
                self.event_queue.task_done()
                
            except asyncio.TimeoutError:
                # No events to process, continue loop
                continue
            except Exception as e:
                # Log error but continue processing
                import logging
                logger = logging.getLogger(__name__)
                logger.error(f"Error processing FCL event: {e}")
                continue
    
    async def _process_event(self, event: FiredNeuronEvent) -> None:
        """Process a single FCL event with retry logic.
        
        Args:
            event: FiredNeuronEvent to process
        """
        for attempt in range(self.max_retries):
            try:
                # Update current timestep if event is newer
                if event.timestep > self.current_timestep:
                    self.current_timestep = event.timestep
                
                # Process based on event type
                if event.event_type == FCLEventType.STANDARD:
                    await self._process_standard_event(event)
                elif event.event_type == FCLEventType.INJECTION:
                    await self._process_injection_event(event)
                elif event.event_type == FCLEventType.MEMORY:
                    await self._process_memory_event(event)
                else:
                    raise ValueError(f"Unknown event type: {event.event_type}")
                
                return  # Success, exit retry loop
                
            except Exception as e:
                if attempt == self.max_retries - 1:
                    raise  # Final attempt failed, re-raise
                
                # Wait before retry
                await asyncio.sleep(self.retry_delay * (2 ** attempt))
    
    async def _process_standard_event(self, event: FiredNeuronEvent) -> None:
        """Process standard synaptic propagation event."""
        # Update FCL manager with fired neurons
        self.fcl_manager.update_fcl(event.timestep, event.neurons_by_cortical)
    
    async def _process_injection_event(self, event: FiredNeuronEvent) -> None:
        """Process special area injection event."""
        # Handle injection events (power areas, modulators, etc.)
        self.fcl_manager.update_fcl(event.timestep, event.neurons_by_cortical)
    
    async def _process_memory_event(self, event: FiredNeuronEvent) -> None:
        """Process memory cortical area event."""
        # Handle memory cortical areas with extended temporal windows
        self.fcl_manager.update_fcl(event.timestep, event.neurons_by_cortical)
    
    def get_performance_stats(self) -> Dict[str, any]:
        """Get performance statistics for monitoring."""
        if not self.processing_times:
            return {
                "events_processed": self.events_processed,
                "total_neurons_processed": self.total_neurons_processed,
                "avg_processing_time_ms": 0.0,
                "queue_size": self.event_queue.qsize(),
            }
        
        avg_time = sum(self.processing_times) / len(self.processing_times)
        return {
            "events_processed": self.events_processed,
            "total_neurons_processed": self.total_neurons_processed,
            "avg_processing_time_ms": avg_time * 1000,
            "min_processing_time_ms": min(self.processing_times) * 1000,
            "max_processing_time_ms": max(self.processing_times) * 1000,
            "queue_size": self.event_queue.qsize(),
            "current_timestep": self.current_timestep,
        }


# Factory function for creating FCL event processors
def create_fcl_event_processor(fcl_manager, config: Optional[Dict] = None) -> FCLEventProcessor:
    """Create an FCL event processor with configuration.
    
    Args:
        fcl_manager: FCL manager instance
        config: Optional configuration dictionary
        
    Returns:
        Configured FCLEventProcessor instance
    """
    if config is None:
        config = {}
    
    max_queue_size = config.get("max_queue_size", 1000)
    processor = FCLEventProcessor(fcl_manager, max_queue_size)
    
    return processor
