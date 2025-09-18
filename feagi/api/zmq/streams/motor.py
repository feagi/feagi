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
FEAGI Motor Stream - For Robot/Agent Motor Control ONLY

[WARN] IMPORTANT: This stream is for MOTOR CONTROL, NOT brain visualization!
   - Motor data uses Type 10 (NEURON_FLAT) format and should stay that way
   - Do NOT change this to Type 11 for "DPR compatibility"
   - DPR (Direct Point Rendering) is ONLY for the visualization stream
   - Motor commands are sent to robots/agents for movement control
   - Completely separate from brain visualization data

This stream handles:
- Real-time motor commands to robotic agents
- Low-latency control signals
- Motor cortex output (OPU areas)
- Agent/robot movement commands

This stream does NOT handle:
- Brain visualization data (that's the visualization stream)
- Neural activity rendering
- Brain monitoring/analysis
"""

import asyncio
import time
from typing import Any, Dict, Optional
from pathlib import Path

# CRITICAL FIX: Import numpy at module level to prevent scoping issues
import numpy as np
import zmq
import zmq.asyncio
import feagi_data_processing as fdp

from feagi.core.state_manager import GenomeState
from feagi.utils.logger import setup_logger
from feagi.utils.zmq_debug import MessageType, log_outbound

# Import the unified CoreAPIService
from ...core.services.core_api_service import CoreAPIService
from ...utils.rate_limit import RateLimiter

import mmap as _mmap
import os as _os
import struct as _struct

logger = setup_logger(__name__)


class MotorStream:
    """ZeroMQ Motor Stream implementation.

    This implementation uses a PUB socket for sending motor data (FEAGI → agents).
    The stream automatically adjusts to the genome availability state:
    - When no genome is loaded, it operates in standby mode
    - When a genome is loaded, it transitions to active mode

    Motor Subscriber Management:
    - Automatically detects motor stream subscribers via heartbeat tracking
    - Controls FQ sampler to sample OPU cortical areas at burst frequency
    - Provides efficient motor data delivery for real-time control applications
    """

    def __init__(
        self,
        core_api: CoreAPIService,
        host: str = "*",
        port: int = 5564,
        context: Optional[zmq.asyncio.Context] = None,
        fq_sampler: Optional[Any] = None,
        fire_queue_provider=None,
        stream_config: Optional[Dict[str, Any]] = None,
        connectome_manager=None,
    ):
        """Initialize the Motor Stream.

        Args:
            core_api: The CoreAPIService instance to delegate calls to
            host: Host address to bind to
            port: Port for sending motor data
            context: Optional existing ZMQ context to use
            fq_sampler: Optional FQ sampler instance for motor data sampling
            fire_queue_provider: Fire queue provider for creating UnifiedFQSampler
            stream_config: Optional stream configuration
            connectome_manager: Optional connectome manager for area info
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()

        # State tracking
        self._active_mode = False  # True when genome is loaded and ready

        # PUB socket for sending motor data (FEAGI → agents)
        self.socket = self._setup_socket()

        # Rate limiter for throttling high-frequency data
        self.rate_limiter = RateLimiter()

        # Motor subscriber management and FQ Sampler integration - ONLY use FQ
        # sampler from Process Manager
        if fq_sampler:
            self.fq_sampler = fq_sampler
            logger.info(
                "MotorStream using FQ sampler from Process Manager "
                "(created on-demand when motor agents connect)"
            )
        else:
            self.fq_sampler = None
            logger.info(
                "No motor FQ sampler available - will be created on-demand "
                "when motor agents connect"
            )

        self.client_last_heartbeat: Dict[str, float] = {}
        self.client_heartbeat_timeout = 30.0  # 30 seconds timeout
        self.subscriber_check_interval = 2.0  # Check every 2 seconds
        self._last_subscriber_count = 0
        self._subscriber_count = 0

        # Motor stream processing task
        self._motor_data_task: Optional[asyncio.Task] = None
        self._subscriber_monitor_task: Optional[asyncio.Task] = None

        # Optional SHM writer for core motor data and per-agent motor writers
        self._shm_writer = None
        self._agent_shm_writers: Dict[str, _ShmRingWriter] = {}
        try:
            from feagi.core.state_manager import FeagiStateManager

            sm = FeagiStateManager.instance()
            shm = sm.get_shared_memory_registry() if hasattr(sm, "get_shared_memory_registry") else {}
            motor_path = shm.get("motor_stream", "")
            if motor_path:
                self._shm_writer = _ShmRingWriter(Path(motor_path))
                logger.info(f"[SHM] Motor stream writing to: {motor_path}")
            else:
                logger.info("[SHM] Motor shared memory not configured; using ZMQ PUB only")
        except Exception as e:
            logger.info(f"[SHM] Motor SHM registry unavailable; using ZMQ PUB only ({e})")

        # Register for genome state change notifications
        if hasattr(core_api, "register_genome_change_listener"):
            core_api.register_genome_change_listener(
                self._on_genome_state_change
            )

        # Initialize state based on current genome availability
        self._update_active_mode()

    def _setup_socket(self):
        """Set up the motor (PUB) socket.

        Returns:
            Configured ZMQ socket
        """
        socket = self.context.socket(zmq.PUB)

        # Configure socket for real-time data with no queuing
        socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
        socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent message
        socket.setsockopt(
            zmq.LINGER, 0
        )  # Don't wait for messages to be sent when closing

        bind_addr = f"tcp://{self.host}:{self.port}"
        logger.info(f"Binding motor PUB socket to {bind_addr}")
        socket.bind(bind_addr)
        return socket

    def _update_active_mode(self):
        """Update active mode based on genome availability."""
        old_mode = self._active_mode

        # Safely check genome loaded state with defensive programming
        try:
            self._active_mode = (
                self.core_api.genome_is_loaded() if self.core_api else False
            )
        except Exception as e:
            #  If there's any error accessing genome state, default to standby
            #  mode
            logger.warning(
                f"Error checking genome state: {e}, defaulting to standby mode"
            )
            self._active_mode = False

        if old_mode != self._active_mode:
            if self._active_mode:
                logger.info("MotorStream entering ACTIVE mode (genome loaded)")
            else:
                logger.info(
                    "MotorStream entering STANDBY mode (no genome loaded)"
                )

    def _on_genome_state_change(self, old_state, new_state):
        """Handle genome state changes.

        Args:
            old_state: Previous genome state
            new_state: New genome state
        """
        logger.debug(
            f"Received genome state change: {old_state} → {new_state}"
        )

        try:
            # Only care about LOADED vs other states
            if new_state == GenomeState.LOADED:
                # Transition to active mode when genome is loaded
                self._active_mode = True
                if self.running:
                    logger.info(
                        "MotorStream entering ACTIVE mode (genome loaded)"
                    )
            else:
                # Any other state means genome not fully loaded
                self._active_mode = False
                if self.running:
                    logger.info(
                        "MotorStream entering STANDBY mode (no genome loaded)"
                    )
        except Exception as e:
            logger.error(f"Error handling genome state change: {e}")
            # Default to standby mode on error
            self._active_mode = False

    def _has_shm_consumers(self) -> bool:
        """Return True if there is any SHM consumer configured for motor data.

        This includes either a core motor SHM writer or at least one
        per-agent motor SHM mapping registered in the state manager.
        """
        try:
            if self._shm_writer is not None:
                return True
            # Check cached per-agent writers first
            if getattr(self, "_agent_shm_writers", None):
                if len(self._agent_shm_writers) > 0:
                    return True
            # Check registry for any agent with a motor SHM path
            from feagi.core.state_manager import FeagiStateManager
            sm = FeagiStateManager.instance()
            agent_map = getattr(sm, "_agent_shared_memory", {})
            for _aid, mapping in agent_map.items():
                if mapping.get("motor") or mapping.get("motor_stream"):
                    return True
            return False
        except Exception:
            # Be conservative on error
            return False

    async def start(self) -> None:
        """Start the motor stream server."""
        if self.running:
            return

        logger.info(f"Starting Motor Stream server on {self.host}:{self.port}")
        self.running = True

        # Start motor data processing if FQ sampler queue is available
        if self.fq_sampler:
            self._motor_data_task = asyncio.create_task(
                self._process_motor_data()
            )

        # Start subscriber monitoring
        self._subscriber_monitor_task = asyncio.create_task(
            self._monitor_subscribers()
        )

        logger.info("Motor Stream server started")

    async def stop(self) -> None:
        """Stop the motor stream server."""
        if not self.running:
            return

        logger.info("Stopping Motor Stream server")
        self.running = False

        # RTOS-friendly: Simple cancellation with bounded wait
        if self._motor_data_task:
            self._motor_data_task.cancel()
            # RTOS-friendly: Simple cancellation, no complex timeout handling
            try:
                await self._motor_data_task
            except asyncio.CancelledError:
                pass  # Expected during cancellation
            self._motor_data_task = None

        if self._subscriber_monitor_task:
            self._subscriber_monitor_task.cancel()
            # RTOS-friendly: Simple cancellation, no complex timeout handling
            try:
                await self._subscriber_monitor_task
            except asyncio.CancelledError:
                pass  # Expected during cancellation
            self._subscriber_monitor_task = None

        #  NOTE: FQ sampler control is handled by Registration Manager, not by
        #  streams

        # RTOS-friendly: Simple socket cleanup
        if self.socket:
            self.socket.close()
            self.socket = None

        # RTOS-friendly: Simple SHM cleanup
        if self._shm_writer:
            self._shm_writer.close()
            self._shm_writer = None

        logger.info("Motor Stream server stopped")

    async def _process_motor_data(self) -> None:
        """Process motor data using only the new UnifiedFQSampler format."""
        if not self.fq_sampler:
            logger.warning("No FQ sampler available for motor data processing")
            return

        logger.debug("Starting motor data processing")

        while self.running:
            try:
                # Get motor data from UnifiedFQSampler only
                motor_data = None
                try:
                    motor_data = self.fq_sampler.sample()
                    if motor_data:
                        logger.debug(
                            f"Got motor data from UnifiedFQSampler: "
                            f"{len(motor_data)} cortical areas"
                        )
                except Exception as e:
                    logger.debug(f"UnifiedFQSampler motor sampling error: {e}")
                    await asyncio.sleep(0.01)
                    continue

                if motor_data is None:
                    await asyncio.sleep(0.01)
                    continue

                # Handle cortical area format data
                if isinstance(motor_data, dict):
                    logger.debug(
                        f"Processing cortical area format: {len(motor_data)} areas"
                    )
                    await self._process_cortical_area_motor_data(motor_data)
                else:
                    logger.warning(
                        f"Unexpected data type from UnifiedFQSampler: "
                        f"{type(motor_data)}"
                    )

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in motor data processing: {e}")
                await asyncio.sleep(0.1)

    async def _process_cortical_area_motor_data(
        self, cortical_data: Dict[str, Any]
    ) -> None:
        """Process motor data in the cortical area format from
        UnifiedFQSampler."""
        try:
            # Check if we have active consumers
            client_count = self.get_connected_client_count()
            has_shm = self._has_shm_consumers()

            if client_count == 0 and not has_shm:
                logger.debug(
                    "No motor consumers (no ZMQ clients, no SHM), skipping cortical area data"
                )
                return
            elif client_count == 0 and has_shm:
                logger.info("𒓉 [MOTOR] ZMQ has no subscribers, but SHM consumers detected → proceeding with SHM fan-out")

            logger.debug(
                f"Processing new cortical area format for motor: "
                f"{len(cortical_data)} areas"
            )

            # Process each cortical area separately for motor control
            for area_key, area_data in cortical_data.items():
                # Resolve cortical ID string from sampler key (int index or str)
                cortical_id_str = None
                try:
                    if isinstance(area_key, int):
                        cortical_id_str = self.core_api.get_cortical_id_for_idx(area_key)
                    elif isinstance(area_key, str) and area_key.isdigit():
                        cortical_id_str = self.core_api.get_cortical_id_for_idx(int(area_key))
                    elif isinstance(area_key, str):
                        cortical_id_str = area_key
                except Exception:
                    cortical_id_str = None

                if not cortical_id_str:
                    continue

                # Filter to OPU (OU) areas only
                try:
                    cm = self.core_api.get_connectome_manager() if hasattr(self.core_api, "get_connectome_manager") else None
                    if cm and hasattr(cm, "get_area_info"):
                        info = cm.get_area_info(cortical_id_str) or {}
                        area_type = str(info.get("type", "")).upper()
                        if area_type != "OPU":
                            continue
                except Exception:
                    # If metadata unavailable, heuristic: require 'o' prefix
                    if not cortical_id_str.startswith("o"):
                        continue

                if not area_data or not area_data.get("neuron_ids"):
                    continue

                # Extract data from area
                neuron_ids = area_data["neuron_ids"]
                membrane_potentials = area_data.get("membrane_potentials", [])
                coordinates = area_data.get("coordinates", [])

                #  Use membrane potentials if available, otherwise default to
                #  1.0
                if membrane_potentials and len(membrane_potentials) == len(
                    neuron_ids
                ):
                    potentials = membrane_potentials
                else:
                    potentials = [1.0] * len(neuron_ids)

                # Encode using feagi_data_processing for motor data - USE
                # HIGH-PERFORMANCE NUMPY APPROACH
                try:
                    # Create the main mapped neuron data container
                    generated_mapped_neuron_data = (
                        fdp.neuron_data.xyzp.CorticalMappedXYZPNeuronData()
                    )

                    # Generate coordinates if not available
                    if coordinates and len(coordinates) == len(neuron_ids):
                        x_values = [coord[0] for coord in coordinates]
                        y_values = [coord[1] for coord in coordinates]
                        z_values = [coord[2] for coord in coordinates]
                    else:
                        # Fallback to ID-based coordinates
                        x_values = [nid % 100 for nid in neuron_ids]
                        y_values = [(nid // 100) % 100 for nid in neuron_ids]
                        z_values = [nid // 10000 for nid in neuron_ids]

                    # Ensure all arrays are the same length
                    max_len = len(neuron_ids)
                    if max_len == 0:
                        continue

                    # Pad potentials if needed
                    if len(potentials) < max_len:
                        potentials.extend([0.0] * (max_len - len(potentials)))
                    elif len(potentials) > max_len:
                        potentials = potentials[:max_len]

                    # Create NumPy arrays with proper dtypes for performance
                    # (neuron_c pattern)
                    neurons_x = np.asarray(x_values[:max_len], dtype=np.uint32)
                    neurons_y = np.asarray(y_values[:max_len], dtype=np.uint32)
                    neurons_z = np.asarray(z_values[:max_len], dtype=np.uint32)
                    neurons_p = np.asarray(
                        potentials[:max_len], dtype=np.float32
                    )

                    #  Create cortical ID using modern feagi-rust-py-libs
                    #  approach
                    area_str = str(cortical_id_str)

                    try:
                        #  Try to create cortical ID directly from string -
                        #  handles all modern format IDs
                        cortical_id_obj = (
                            fdp.genome.CorticalID.try_new_from_string(area_str)
                        )
                    except ValueError:
                        # Fallback for areas that can't be parsed directly
                        if area_str == "_power":
                            cortical_id_obj = fdp.genome.CorticalID.new_core_cortical_area_id(
                                fdp.genome.CoreCorticalType.Power
                            )
                        elif area_str == "_death":
                            cortical_id_obj = fdp.genome.CorticalID.new_core_cortical_area_id(
                                fdp.genome.CoreCorticalType.Death
                            )
                        else:
                            # For unknown areas, use custom cortical ID
                            # Custom cortical IDs must start with lowercase 'c' (fdp requirement)
                            if len(area_str) == 6:
                                # If starts with 'C', convert to 'c'; if already starts with 'c', keep as is
                                if area_str.startswith('C'):
                                    custom_id = 'c' + area_str[1:]  # Replace 'C' with 'c'
                                elif area_str.startswith('c'):
                                    custom_id = area_str  # Already correct
                                else:
                                    custom_id = f"c{area_str[:-1]}"  # Add 'c' prefix, truncate to 6 chars
                                cortical_id_obj = fdp.genome.CorticalID.new_custom_cortical_area_id(
                                    custom_id
                                )
                            else:
                                # Only add 'c' prefix if less than 6 characters
                                custom_id = f"c{area_str}"[:6]  # Ensure max 6 characters
                                cortical_id_obj = fdp.genome.CorticalID.new_custom_cortical_area_id(
                                    custom_id
                                )

                    # Use high-performance NumPy approach (neuron_c pattern)
                    neurons_array = (
                        fdp.neuron_data.xyzp.NeuronXYZPArrays.new_from_numpy(
                            neurons_x, neurons_y, neurons_z, neurons_p
                        )
                    )

                    #  Insert the neuron array into the mapped data with its
                    #  cortical ID
                    generated_mapped_neuron_data.insert(
                        cortical_id_obj, neurons_array
                    )

                    # Create the final byte structure from the mapped data
                    byte_structure = (
                        generated_mapped_neuron_data.as_new_feagi_byte_structure()
                    )
                    binary_data = byte_structure.copy_out_as_byte_vector()

                    # DEBUG: Log the structure ID being generated
                    if binary_data and len(binary_data) > 0:
                        logger.debug(
                            f"MOTOR STREAM DEBUG: Generated {len(binary_data)} "
                            "bytes via optimized path"
                        )
                        logger.debug(
                            f"   Structure ID (bytes[0]): {binary_data[0]} "
                            f"(0x{binary_data[0]:02X})"
                        )
                        logger.debug(
                            f"   First 8 bytes: {list(binary_data[:8])}"
                        )
                        logger.debug(
                            "   Generated Type 11 (NEURON_CATEGORIES) - "
                            "optimized motor path!"
                        )

                    await self._send_motor_binary_data(
                        binary_data, channel=cortical_id_str
                    )

                except Exception as e:
                    logger.error(
                        f"Error encoding motor data for area {cortical_id_str}: {e}"
                    )

        except Exception as e:
            logger.error(f"Error processing cortical area motor data: {e}")

    async def _send_motor_binary_data(
        self, binary_data: bytes, channel: str = "motor"
    ):
        """Send binary motor data to motor clients."""
        try:
            # Skip if in standby mode
            if not self._active_mode:
                logger.debug(
                    "Motor stream in STANDBY mode, skipping data send"
                )
                return

            #  Debug logging for outbound motor data (gated)
            try:
                from feagi.core.state_manager import FeagiStateManager
                if FeagiStateManager.instance().is_debug_zmq_outbound_enabled():
                    debug_endpoint = f"tcp://{self.host}:{self.port}"
                    log_outbound(
                        endpoint=debug_endpoint,
                        data=[channel.encode("utf-8"), binary_data],
                        message_type=MessageType.MOTOR,
                        topic=channel,
                        context="motor_cmd",
                    )
            except Exception:
                pass

            # Write to SHM if configured
            if self._shm_writer:
                try:
                    self._shm_writer.write_payload(binary_data)
                except Exception as e:
                    logger.debug(f"[SHM] Motor write failed: {e}")
            # Also fan-out to per-agent motor SHM files if available
            try:
                from feagi.core.state_manager import FeagiStateManager
                sm = FeagiStateManager.instance()
                agent_map = getattr(sm, "_agent_shared_memory", {})
                for aid, mapping in agent_map.items():
                    path = mapping.get("motor") or mapping.get("motor_stream")
                    if not path:
                        continue
                    writer = self._agent_shm_writers.get(aid)
                    if writer is None:
                        try:
                            writer = _ShmRingWriter(Path(path))
                            self._agent_shm_writers[aid] = writer
                            logger.info(f"[SHM] Motor fan-out enabled for agent {aid}: {path}")
                        except Exception as we:
                            logger.debug(f"[SHM] Failed to open agent motor SHM for {aid}: {we}")
                            continue
                    try:
                        writer.write_payload(binary_data)
                    except Exception as wre:
                        logger.debug(f"[SHM] Agent motor write failed for {aid}: {wre}")
            except Exception:
                pass

            # Send data on specified motor channel (retain ZMQ path)
            await self.socket.send_multipart(
                [channel.encode("utf-8"), binary_data]
            )

            try:
                from feagi.core.state_manager import FeagiStateManager
                if FeagiStateManager.instance().is_debug_zmq_outbound_enabled():
                    logger.debug(
                        f"[ZMQ-OUT-DEBUG] Sent {len(binary_data)} bytes on channel {channel}"
                    )
            except Exception:
                pass

        except Exception as e:
            logger.error(f"Error sending motor binary data: {e}")

    async def send_motor_data(self, channel_id: str, data: bytes) -> None:
        """Send motor data to agents.

        Args:
            channel_id: Motor channel ID
            data: Binary motor data
        """
        if not self.running or not self.socket:
            logger.warning("Cannot send motor data: server not running")
            return

        # Skip if in standby mode
        if not self._active_mode:
            logger.debug(
                f"Suppressing motor output (channel {channel_id}) in standby mode"
            )
            return

        try:
            # Apply rate limiting if needed
            if not self.rate_limiter.check_rate(
                f"motor_{channel_id}", 0.01
            ):  # Max 100Hz per channel
                logger.debug(
                    f"Rate limiting motor data on channel {channel_id}"
                )
                return

            #  Debug logging for outbound motor data (gated)
            try:
                from feagi.core.state_manager import FeagiStateManager
                if FeagiStateManager.instance().is_debug_zmq_outbound_enabled():
                    debug_endpoint = f"tcp://{self.host}:{self.port}"
                    log_outbound(
                        endpoint=debug_endpoint,
                        data=[channel_id.encode("utf-8"), data],
                        message_type=MessageType.MOTOR,
                        topic=channel_id,
                        context="external_motor_cmd",
                    )
            except Exception:
                pass

            # Send multipart message with topic (channel_id) and data
            await self.socket.send_multipart(
                [
                    channel_id.encode("utf-8"),  # Topic (channel ID)
                    data,  # Binary data
                ]
            )

            try:
                from feagi.core.state_manager import FeagiStateManager
                if FeagiStateManager.instance().is_debug_zmq_outbound_enabled():
                    logger.debug(
                        f"[ZMQ-OUT-DEBUG] Sent {len(data)} bytes on channel {channel_id}"
                    )
            except Exception:
                pass

        except Exception as e:
            logger.error(
                f"Error sending motor data on channel {channel_id}: {e}"
            )

    async def broadcast_system_message(self, message: str) -> None:
        """Broadcast a system message to all connected agents.

        Args:
            message: System message to broadcast
        """
        try:
            # Send on system channel
            await self.socket.send_multipart(
                [
                    b"system",  # System channel
                    message.encode("utf-8"),  # Message
                ]
            )

            logger.debug(f"Broadcast system message: {message}")

        except Exception as e:
            logger.error(f"Error broadcasting system message: {e}")

    def get_connected_client_count(self) -> int:
        """Get the current number of connected motor clients."""
        try:
            now = time.time()
            active_clients = 0

            for (
                _client_id,
                last_heartbeat,
            ) in self.client_last_heartbeat.items():
                if now - last_heartbeat < self.client_heartbeat_timeout:
                    active_clients += 1

            return active_clients
        except Exception as e:
            logger.warning(f"Error getting motor client count: {e}")
            return 0

    async def record_client_heartbeat(self, client_id: str) -> None:
        """Record a heartbeat from a motor client."""
        current_time = time.time()
        self.client_last_heartbeat[client_id] = current_time

    async def _monitor_subscribers(self) -> None:
        """Monitor ZMQ motor subscribers - removed FQ sampler control
        (handled by Registration Manager)."""
        logger.info(
            "Starting motor subscriber monitoring for logging/statistics only"
        )

        # RTOS-friendly: Simple loop with small, bounded sleep intervals
        while self.running:
            try:
                # Check current subscriber count for logging/statistics only
                current_count = self.get_connected_client_count()

                # Update subscriber count for logging only
                if current_count != self._last_subscriber_count:
                    logger.info(
                        f"Motor subscriber count changed: "
                        f"{self._last_subscriber_count} -> {current_count}"
                    )
                    self._last_subscriber_count = current_count
                    #  NOTE: FQ sampler control is handled by Registration
                    #  Manager
                    # when agents register/deregister

                #  RTOS-friendly: Use small bounded intervals for responsive
                #  shutdown
                #  Check running flag more frequently for deterministic
                #  cancellation
                remaining_sleep = self.subscriber_check_interval
                while remaining_sleep > 0 and self.running:
                    sleep_chunk = min(
                        0.1, remaining_sleep
                    )  # 100ms chunks maximum
                    await asyncio.sleep(sleep_chunk)
                    remaining_sleep -= sleep_chunk

            except asyncio.CancelledError:
                # RTOS-friendly: Simple, deterministic cancellation
                logger.debug(
                    "Motor subscriber monitoring cancelled during shutdown"
                )
                break
            except Exception as e:
                logger.error(f"Error in motor subscriber monitoring: {e}")
                # RTOS-friendly: Fixed, bounded error recovery delay
                await asyncio.sleep(0.1)  # Fixed 100ms delay

        logger.info("Motor subscriber monitoring stopped")

    async def register_motor_client(self, client_id: str) -> None:
        """Register a motor client and update heartbeat."""
        current_time = time.time()
        self.client_last_heartbeat[client_id] = current_time
        logger.info(f"🚗 Motor client registered: {client_id}")

        # Update subscriber count for logging only
        current_count = self.get_connected_client_count()
        if current_count != self._last_subscriber_count:
            self._last_subscriber_count = current_count
            # NOTE: FQ sampler control is handled by Registration Manager,
            # not by streams

    async def unregister_motor_client(self, client_id: str) -> None:
        """Unregister a motor client."""
        if client_id in self.client_last_heartbeat:
            del self.client_last_heartbeat[client_id]
            logger.info(f"🚗 Motor client unregistered: {client_id}")

            # Update subscriber count for logging only
            current_count = self.get_connected_client_count()
            if current_count != self._last_subscriber_count:
                self._last_subscriber_count = current_count
                # NOTE: FQ sampler control is handled by Registration Manager,
                # not by streams

    async def heartbeat_motor_client(self, client_id: str) -> None:
        """Update heartbeat for a motor client."""
        self.client_last_heartbeat[client_id] = time.time()
        # Don't log every heartbeat to avoid spam, just update the timestamp


def _process_tuple_data(
    cortical_id: str, data_dict: Dict[str, Any]
) -> Dict[str, Any]:
    """Process tuple-format FQ data into encoder format."""
    try:
        # Extract neuron data from the data dictionary
        neuron_ids = data_dict.get("neuron_ids", [])
        x_coords = data_dict.get("x", [])
        y_coords = data_dict.get("y", [])
        z_coords = data_dict.get("z", [])
        membrane_potentials = data_dict.get("membrane_potentials", [])

        # Create cortical_ids list (same cortical_id for all neurons)
        cortical_ids = [cortical_id] * len(neuron_ids)

        return {
            "cortical_ids": cortical_ids,
            "x_coords": x_coords,
            "y_coords": y_coords,
            "z_coords": z_coords,
            "membrane_potentials": membrane_potentials,
        }
    except Exception as e:
        logger.error(f"Error processing tuple data: {e}")
        return {}


def _process_dict_data(fq_data: Dict[str, Any]) -> Dict[str, Any]:
    """Process dictionary-format FQ data into encoder format."""
    try:
        cortical_ids = []
        x_coords = []
        y_coords = []
        z_coords = []
        membrane_potentials = []

        # Process each cortical area in the dictionary
        for cortical_id, area_data in fq_data.items():
            if not isinstance(area_data, dict):
                continue

            area_neuron_ids = area_data.get("neuron_ids", [])
            area_x = area_data.get("x", [])
            area_y = area_data.get("y", [])
            area_z = area_data.get("z", [])
            area_potentials = area_data.get("membrane_potentials", [])

            # Add data for this area
            cortical_ids.extend([cortical_id] * len(area_neuron_ids))
            x_coords.extend(area_x)
            y_coords.extend(area_y)
            z_coords.extend(area_z)
            membrane_potentials.extend(area_potentials)

        return {
            "cortical_ids": cortical_ids,
            "x_coords": x_coords,
            "y_coords": y_coords,
            "z_coords": z_coords,
            "membrane_potentials": membrane_potentials,
        }
    except Exception as e:
        logger.error(f"Error processing dict data: {e}")
        return {}


def handle_motor_stream(
    burst_engine, subscriber_count: int
) -> Optional[bytes]:
    """Handle motor stream with optimized performance path.

    This function uses the optimized FQ sampler if available, otherwise falls
    back to legacy processing for compatibility.
    """
    if subscriber_count <= 0:
        return None

    # Check for optimized sampler
    if hasattr(burst_engine, "optimized_fq_sampler"):
        try:
            # Get OPU areas efficiently
            opu_areas = []
            if (
                hasattr(burst_engine, "connectome_manager")
                and burst_engine.connectome_manager
            ):
                cm = burst_engine.connectome_manager
                if hasattr(cm, "cortical_areas"):
                    for area_id, area in cm.cortical_areas.items():
                        area_type = getattr(area, "cortical_type", "").upper()
                        if (
                            "OPU" in area_type
                            or "OUTPUT" in area_type
                            or "MOTOR" in area_type
                            or area_id.startswith(
                                ("opu_", "motor_", "output_")
                            )
                        ):
                            opu_areas.append(area_id)

            # Direct binary output from optimized sampler
            if opu_areas:
                binary_data = burst_engine.optimized_fq_sampler.sample_motor_areas_direct(
                    opu_areas
                )
                if binary_data:
                    logger.debug(
                        f"MOTOR STREAM DEBUG: Generated {len(binary_data)} "
                        "bytes via optimized path"
                    )
                    logger.debug(
                        f"   Structure ID (bytes[0]): {binary_data[0]} "
                        f"(0x{binary_data[0]:02X})"
                    )
                    logger.debug(f"   First 8 bytes: {list(binary_data[:8])}")
                    logger.debug(
                        "   Generated Type 11 (NEURON_CATEGORIES) - "
                        "optimized motor path!"
                    )
                    return binary_data
        except Exception as e:
            logger.error(f"Error in optimized motor stream: {e}")
            # Fall through to legacy processing

    # Legacy processing path
    if not hasattr(burst_engine, "fq_sampler") or not burst_engine.fq_sampler:
        logger.warning("No FQ sampler available for motor stream")
        return None

    try:
        # Get data from legacy sampler
        fq_data = burst_engine.fq_sampler.get_latest_data(target="motor")

        if not fq_data:
            return None

        # Process the data appropriately
        if isinstance(fq_data, tuple) and len(fq_data) == 2:
            cortical_id, data_dict = fq_data
            processed_data = _process_tuple_data(cortical_id, data_dict)
        elif isinstance(fq_data, dict):
            processed_data = _process_dict_data(fq_data)
        else:
            logger.warning(f"Unexpected motor fq_data format: {type(fq_data)}")
            return None

        if not processed_data:
            return None

        # Encode using the encoder
        from feagi.protocols.feagi_data_codec import FeagiCodec

        encoder = FeagiCodec()

        binary_data = encoder.encode_neuron_categories(
            cortical_ids=processed_data.get("cortical_ids", []),
            x_coords=processed_data.get("x_coords", []),
            y_coords=processed_data.get("y_coords", []),
            z_coords=processed_data.get("z_coords", []),
            membrane_potentials=processed_data.get("membrane_potentials", []),
        )

        logger.debug(
            f"MOTOR STREAM DEBUG: Generated {len(binary_data)} bytes via legacy path"
        )
        logger.debug(
            f"   Structure ID (bytes[0]): {binary_data[0]} (0x{binary_data[0]:02X})"
        )
        logger.debug(f"   First 8 bytes: {list(binary_data[:8])}")
        logger.debug(
            "   Generated Type 11 (NEURON_CATEGORIES) - legacy motor path!"
        )

        return binary_data

    except Exception as e:
        logger.error(f"Error in motor stream processing: {e}")
        return None


class _ShmRingWriter:
    MAGIC = b"FEAGIMOT"
    VERSION = 1
    HEADER_SIZE = 256
    HEADER_FMT = "<8sIIIQI"

    def __init__(self, path: Path, num_slots: int = 64, slot_size: int = 1 * 1024 * 1024):
        self.path = Path(path)
        self.num_slots = int(max(2, num_slots))
        self.slot_size = int(max(1024, slot_size))
        self._mm = None
        self._fd = None
        self._frame_seq = 0
        self._write_index = 0
        self._open()

    def _open(self) -> None:
        total_size = self.HEADER_SIZE + self.num_slots * self.slot_size
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fd = _os.open(str(self.path), _os.O_CREAT | _os.O_RDWR)
        _os.ftruncate(self._fd, total_size)
        self._mm = _mmap.mmap(self._fd, total_size, access=_mmap.ACCESS_WRITE)
        header = _struct.pack(
            self.HEADER_FMT,
            self.MAGIC,
            self.VERSION,
            self.num_slots,
            self.slot_size,
            0,
            0,
        )
        self._mm.seek(0)
        self._mm.write(header)
        if self.HEADER_SIZE > len(header):
            self._mm.write(b"\x00" * (self.HEADER_SIZE - len(header)))

    def write_payload(self, payload: bytes) -> None:
        if not self._mm:
            return
        if len(payload) + 4 > self.slot_size:
            payload = payload[: self.slot_size - 4]
        slot_off = self.HEADER_SIZE + self._write_index * self.slot_size
        self._mm.seek(slot_off)
        self._mm.write(_struct.pack("<I", len(payload)))
        self._mm.write(payload)
        rem = self.slot_size - 4 - len(payload)
        if rem > 0:
            self._mm.write(b"\x00" * rem)
        self._frame_seq += 1
        self._write_index = (self._write_index + 1) % self.num_slots
        # Update header
        self._mm.seek(0)
        header = _struct.pack(
            self.HEADER_FMT,
            self.MAGIC,
            self.VERSION,
            self.num_slots,
            self.slot_size,
            self._frame_seq,
            self._write_index,
        )
        self._mm.write(header)

    def close(self) -> None:
        try:
            if self._mm:
                self._mm.flush()
                self._mm.close()
        except Exception:
            pass
        if self._fd is not None:
            try:
                _os.close(self._fd)
            except Exception:
                pass
        self._mm = None
        self._fd = None
