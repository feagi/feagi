"""
ZeroMQ Visualization Stream Implementation for FEAGI API

This module implements the specialized streaming pattern for brain visualization data.
It provides:
- Efficient brain activity visualization streaming
- Level-of-detail mechanisms for performance optimization
- Client view control and filtering
- Genome-dependent state management (standby when no genome loaded)

Performance Optimization:
- All sockets are configured for real-time operation with minimal latency
- Messages are treated as ephemeral - no queueing is performed
- ZMQ_CONFLATE ensures only the latest message is kept, preventing stale data processing
- High water marks (HWM) are set to minimal values to prevent buffer buildup
"""

import asyncio
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import time
import uuid
from typing import Dict, Any, List, Optional, Set, Tuple, Union

import zmq
import zmq.asyncio
import numpy as np

from ...core.service import CoreApiService
from ..serialization import serialize_message, deserialize_message
from ...utils.rate_limit import RateLimiter
from feagi.core.state_manager import GenomeState


class VisualizationStream:
    """
    ZeroMQ Visualization Stream implementation.
    
    This specialized stream efficiently broadcasts neural activity and structural data
    to visualization clients, with support for level-of-detail streaming.
    
    The stream automatically adjusts to the genome availability state:
    - When no genome is loaded, it operates in standby mode, sending status updates but no data
    - When a genome is loaded, it transitions to active mode with full functionality
    """
    
    def __init__(
        self, 
        core_api: CoreApiService,
        host: str = "*", 
        structure_port: int = 5560,
        activity_port: int = 5561,
        control_port: int = 5562,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize a new Visualization Stream.
        
        Args:
            core_api: The CoreApiService instance to delegate calls to
            host: Host address to bind to
            structure_port: Port for structural data
            activity_port: Port for activity data
            control_port: Port for client control messages
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self._structure_port = structure_port
        self._activity_port = activity_port
        self._control_port = control_port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        
        # State tracking
        self._active_mode = False  # True when genome is loaded and ready
        
        # Socket for structural data (changes infrequently)
        self.structure_socket = self._setup_socket(structure_port)
        
        # Socket for real-time activity data (high frequency)
        self.activity_socket = self._setup_socket(activity_port)
        
        # Socket for client requests (view changes, filters)
        self.control_socket = self.context.socket(zmq.ROUTER)
        self.control_socket.bind(f"tcp://{host}:{control_port}")
        
        # Connected clients and their view settings
        self.clients = {}
        
        # Previous state for delta encoding
        self.previous_state = {}
        
        # Rate limiter for different detail levels
        self.rate_limiter = RateLimiter()
        
        # Periodic task references
        self.periodic_tasks = {}
        
        # Register for genome state change notifications
        if hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
        
        # Initialize state based on current genome availability
        self._update_active_mode()

    def _setup_socket(self, port: int) -> zmq.asyncio.Socket:
        """
        Set up a visualization socket with real-time optimization.
        
        Args:
            port: Port to bind to
            
        Returns:
            Configured ZMQ socket
        """
        socket = self.context.socket(zmq.PUB)
        
        # Configure for real-time with no queuing
        socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
        socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent message
        socket.setsockopt(zmq.LINGER, 0)  # Don't wait when closing
        
        bind_addr = f"tcp://{self.host}:{port}"
        logger.info(f"Binding visualization PUB socket to {bind_addr}")
        socket.bind(bind_addr)
        return socket
        
    def _update_active_mode(self):
        """Update whether stream is active based on genome state."""
        if not hasattr(self.core_api, 'get_state_manager'):
            self._active_mode = True  # Default to active for testing
            return
            
        state_manager = self.core_api.get_state_manager()
        if not state_manager:
            self._active_mode = False
            return
            
        genome_state = state_manager.genome_state
        
        if genome_state == GenomeState.LOADED:
            if not self._active_mode:
                logger.info("Visualization Stream activating (genome loaded)")
            self._active_mode = True
        else:
            if self._active_mode:
                logger.info("Visualization Stream entering standby mode (no genome loaded)")
            self._active_mode = False

    async def _broadcast_state_change(self, state: str):
        """Broadcast state change to all connected clients.
        
        Args:
            state: New state ("active" or "standby")
        """
        try:
            # Send on system channel
            await self.activity_socket.send_multipart([
                b"system",
                b"application/json",
                serialize_message({
                    "type": "state_change",
                    "state": state,
                    "timestamp": int(time.time() * 1000)
                }, "application/json")
            ])
            logger.debug(f"Broadcasted visualization state change to {state}")
        except Exception as e:
            logger.error(f"Error broadcasting state change: {e}")
    
    def _on_genome_state_change(self, old_state, new_state):
        """Handle genome state changes.
        
        Args:
            old_state: Previous genome state
            new_state: New genome state
        """
        logger.debug(f"Visualization received genome state change: {old_state} → {new_state}")
        
        try:
            # Only care about LOADED vs other states
            if new_state == GenomeState.LOADED:
                # Transition to active mode when genome is loaded
                self._active_mode = True
                if self.running:
                    logger.info("VisualizationStream entering ACTIVE mode (genome loaded)")
                    asyncio.create_task(self._broadcast_state_change("active"))
                    # Force a full structure update to all clients
                    asyncio.create_task(self._send_brain_structure_to_all_force())
            else:
                # Any other state means genome not fully loaded
                self._active_mode = False 
                if self.running:
                    logger.info("VisualizationStream entering STANDBY mode (genome not loaded)")
                    asyncio.create_task(self._broadcast_state_change("standby"))
        except Exception as e:
            logger.error(f"Error handling genome state change: {e}")
            # Default to standby mode on error
            self._active_mode = False
            if self.running:
                asyncio.create_task(self._broadcast_state_change("standby"))

    @property
    def structure_port(self) -> int:
        """Get the port used for structural data."""
        return self._structure_port
        
    @property
    def activity_port(self) -> int:
        """Get the port used for activity data."""
        return self._activity_port
        
    @property
    def control_port(self) -> int:
        """Get the port used for control messages."""
        return self._control_port

    async def start(self) -> None:
        """Start the visualization stream server."""
        logger.info(f"Starting Visualization Stream server")
        self.running = True
        
        # Store the current event loop for this method
        self._event_loop = asyncio.get_event_loop()
        
        # Start tasks in the current loop
        self.periodic_tasks["control_handler"] = self._event_loop.create_task(
            self._handle_control_messages()
        )
        
        self.periodic_tasks["activity_stream"] = self._event_loop.create_task(
            self._stream_activity()
        )
        
        self.periodic_tasks["structure_updates"] = self._event_loop.create_task(
            self._stream_structure_updates()
        )
        
        # Determine initial state (active or standby)
        self._update_active_mode()
        
        # Broadcast initial state to clients
        if self._active_mode:
            await self._broadcast_state_change("active")
        else:
            await self._broadcast_state_change("standby")

        # Start FCL collector with default interval
        await self.start_fcl_collector(interval=0.1)

    async def stop(self) -> None:
        """Stop the visualization stream server."""
        logger.info("Stopping Visualization Stream server")
        self.running = False
        
        # Cancel all periodic tasks
        for task_name, task in self.periodic_tasks.items():
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    logger.debug(f"Cancelled periodic task: {task_name}")
        
        # Close all sockets
        self.structure_socket.close()
        self.activity_socket.close()
        self.control_socket.close()

    async def _handle_control_messages(self) -> None:
        """Handle client control messages for view settings."""
        while self.running:
            try:
                # Receive client message (client_id, empty delimiter, message)
                multipart = await self.control_socket.recv_multipart()
                
                if len(multipart) < 3:
                    logger.error(f"Received malformed control message: {multipart}")
                    continue
                
                client_id = multipart[0].decode()
                content_type = multipart[2].decode()
                message = deserialize_message(multipart[3], content_type)
                
                logger.debug(f"Received control message from client {client_id}")
                
                # Special handling for system message type
                if message.get("type") == "system":
                    await self._handle_system_message(client_id, message)
                    continue
                
                # Process message based on type
                if message["type"] == "view_settings":
                    await self._handle_view_settings(client_id, message["settings"])
                elif message["type"] == "register":
                    await self._handle_client_registration(client_id, message.get("settings", {}))
                elif message["type"] == "unregister":
                    await self._handle_client_unregistration(client_id)
                else:
                    logger.warning(f"Unknown control message type: {message['type']}")
                
                # Acknowledge receipt
                await self.control_socket.send_multipart([
                    client_id.encode(),
                    b"",
                    b"application/json",
                    serialize_message({"status": "ok"}, "application/json")
                ])
                
            except asyncio.CancelledError:
                logger.debug("Control message handler cancelled")
                break
            except Exception as e:
                logger.error(f"Error handling control message: {e}")
                await asyncio.sleep(1)  # Avoid tight loop on errors
                
    async def _handle_system_message(self, client_id: str, message: Dict):
        """Handle system messages from clients.
        
        Args:
            client_id: Client ID
            message: Message content
        """
        command = message.get("command")
        
        if command == "status_check":
            # Check genome status safely
            try:
                genome_loaded = self.core_api.genome_is_loaded() if self.core_api else False
            except Exception as e:
                logger.warning(f"Error checking genome state during status check: {e}")
                genome_loaded = False
                
            # Reply with current system state
            await self.control_socket.send_multipart([
                client_id.encode(),
                b"",
                b"application/json",
                serialize_message({
                    "type": "system_status",
                    "active_mode": self._active_mode,
                    "genome_loaded": genome_loaded,
                    "timestamp": int(time.time() * 1000)
                }, "application/json")
            ])
            logger.debug(f"Sent system status to client {client_id}")

    async def _handle_view_settings(self, client_id: str, settings: Dict) -> None:
        """
        Update a client's view settings.
        
        Args:
            client_id: Client identifier
            settings: View settings dictionary
        """
        if client_id not in self.clients:
            logger.warning(f"Received view settings for unknown client: {client_id}")
            self.clients[client_id] = settings
        else:
            # Update existing settings
            self.clients[client_id].update(settings)
        
        logger.debug(f"Updated view settings for client {client_id}: {settings}")

    async def _handle_client_registration(self, client_id: str, settings: Dict) -> None:
        """
        Register a new visualization client.
        
        Args:
            client_id: Client identifier
            settings: Initial view settings
        """
        self.clients[client_id] = settings
        logger.info(f"Registered new visualization client: {client_id}")
        
        # Send current state information
        await self.control_socket.send_multipart([
            client_id.encode(),
            b"",
            b"application/json",
            serialize_message({
                "type": "system_status",
                "active_mode": self._active_mode,
                "genome_loaded": self.core_api.genome_is_loaded(),
                "timestamp": int(time.time() * 1000)
            }, "application/json")
        ])
        
        # Send current brain structure to new client if in active mode
        if self._active_mode:
            await self._send_brain_structure(client_id)

    async def _handle_client_unregistration(self, client_id: str) -> None:
        """
        Unregister a visualization client.
        
        Args:
            client_id: Client identifier
        """
        if client_id in self.clients:
            del self.clients[client_id]
            logger.info(f"Unregistered visualization client: {client_id}")
        else:
            logger.warning(f"Attempt to unregister unknown client: {client_id}")

    async def _stream_activity(self) -> None:
        """Periodically stream brain activity updates."""
        update_interval = 0.05  # 50ms default (20 Hz)
        
        while self.running:
            try:
                if not self.clients:
                    # No connected clients, sleep and check again
                    await asyncio.sleep(0.5)
                    continue
                
                # Skip processing if system is in standby mode
                if not self._active_mode:
                    # Send standby heartbeat every second to keep clients informed
                    if int(time.time()) % 5 == 0:  # Every 5 seconds
                        await self._send_standby_heartbeat()
                    await asyncio.sleep(1.0)
                    continue
                
                # Get current brain activity
                brain_state = await self.core_api.get_brain_activity()
                
                # Send activity to clients
                await self._send_activity_update(brain_state)
                
                # Dynamic sleep based on client needs
                min_interval = min(
                    client.get("update_interval", update_interval) 
                    for client in self.clients.values()
                )
                await asyncio.sleep(min_interval)
                
            except asyncio.CancelledError:
                logger.debug("Activity stream task cancelled")
                break
            except Exception as e:
                logger.error(f"Error streaming activity: {e}")
                await asyncio.sleep(1)  # Avoid tight loop on errors
                
    async def _send_standby_heartbeat(self):
        """Send standby heartbeat to clients."""
        try:
            await self.activity_socket.send_multipart([
                b"system",
                b"application/json",
                serialize_message({
                    "type": "heartbeat",
                    "state": "standby",
                    "timestamp": int(time.time() * 1000)
                }, "application/json")
            ])
            logger.debug("Sent standby heartbeat")
        except Exception as e:
            logger.error(f"Error sending standby heartbeat: {e}")

    async def _stream_structure_updates(self) -> None:
        """Periodically send brain structure updates."""
        while self.running:
            try:
                # Skip processing if system is in standby mode
                if not self._active_mode:
                    await asyncio.sleep(5.0)
                    continue
                    
                # Get current brain structure
                structure = await self.core_api.get_brain_structure()
                
                # Send to all clients
                await self._send_brain_structure_to_all(structure)
                
                # Check for structural changes every 5 seconds
                await asyncio.sleep(5.0)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error streaming structure updates: {e}")
                await asyncio.sleep(5.0)  # Avoid tight loop on errors

    async def _send_activity_update(self, brain_state: Dict) -> None:
        """
        Send activity updates to clients with appropriate level of detail.
        
        Args:
            brain_state: Current brain state with activity data
        """
        timestamp = int(time.time() * 1000)
        
        # Create base update with minimal data (low detail)
        base_update = {
            "timestamp": timestamp,
            "summary": self._create_activity_summary(brain_state)
        }
        
        # Send the base update to all clients
        await self.activity_socket.send_multipart([
            b"activity.base",
            b"application/json",
            serialize_message(base_update, "application/json")
        ])
        
        # For each detail level, send additional data if there are clients at that level
        for detail_level in range(1, 4):  # LOD levels 1-3
            clients_at_level = [cid for cid, settings in self.clients.items() 
                               if settings.get("detail_level", 1) >= detail_level]
            
            if not clients_at_level:
                continue
                
            # Create detail level specific data
            detail_data = self._create_detail_level(brain_state, detail_level)
            
            await self.activity_socket.send_multipart([
                f"activity.detail.{detail_level}".encode(),
                b"application/octet-stream",  # Binary for efficiency
                serialize_message(detail_data, "application/octet-stream")
            ])
        
        # Send ROI-specific high detail data
        for client_id, settings in self.clients.items():
            roi = settings.get("roi")
            if roi:
                roi_data = self._extract_roi_data(brain_state, roi)
                
                await self.activity_socket.send_multipart([
                    f"activity.roi.{client_id}".encode(),
                    b"application/octet-stream",
                    serialize_message(roi_data, "application/octet-stream")
                ])

    def _create_activity_summary(self, brain_state: Dict) -> Dict:
        """
        Create a summary of brain activity data.
        
        Args:
            brain_state: Full brain state data
            
        Returns:
            Dictionary with summarized activity data
        """
        summary = {}
        
        for area_id, area_data in brain_state.items():
            # Count active neurons
            if hasattr(area_data, "nonzero"):
                # NumPy array
                active_count = np.count_nonzero(area_data)
                total_count = area_data.size
            elif isinstance(area_data, dict) and "active_indices" in area_data:
                # Sparse representation
                active_count = len(area_data["active_indices"])
                total_count = area_data.get("total_count", 0)
            else:
                # Unknown format
                active_count = 0
                total_count = 0
            
            # Create area summary
            summary[area_id] = {
                "active_count": active_count,
                "total_count": total_count,
                "activity_ratio": active_count / max(1, total_count)
            }
        
        return summary

    def _create_detail_level(self, brain_state: Dict, detail_level: int) -> Dict:
        """
        Create a detail level specific view of brain activity.
        
        Args:
            brain_state: Full brain state data
            detail_level: Detail level (1-3)
            
        Returns:
            Dictionary with appropriate level of detail
        """
        result = {}
        
        # Tailor detail based on level
        if detail_level == 1:
            # Level 1: Region-level activity summaries
            for area_id, area_data in brain_state.items():
                # Simple downsampling
                result[area_id] = self._downsample_area(area_data, factor=16)
                
        elif detail_level == 2:
            # Level 2: Medium resolution data
            for area_id, area_data in brain_state.items():
                result[area_id] = self._downsample_area(area_data, factor=4)
                
        elif detail_level == 3:
            # Level 3: High resolution data
            for area_id, area_data in brain_state.items():
                # No downsampling, but still use delta encoding if possible
                result[area_id] = self._delta_encode_area(area_id, area_data)
        
        return result

    def _downsample_area(self, area_data: Any, factor: int) -> Dict:
        """
        Downsample area data by a given factor.
        
        Args:
            area_data: Original area data
            factor: Downsampling factor
            
        Returns:
            Downsampled data
        """
        # Implement downsampling based on data type
        # This is a simple placeholder implementation
        return {
            "downsampled": True,
            "factor": factor,
            "data": "downsampled_data_placeholder"
        }

    def _delta_encode_area(self, area_id: str, current_data: Any) -> Dict:
        """
        Create delta-encoded data based on previous state.
        
        Args:
            area_id: Area identifier
            current_data: Current area data
            
        Returns:
            Delta-encoded data or full data if no previous state exists
        """
        # Simple placeholder for delta encoding
        return {
            "delta_encoded": True,
            "data": "delta_encoded_data_placeholder"
        }

    def _extract_roi_data(self, brain_state: Dict, roi: Dict) -> Dict:
        """
        Extract data for a specific region of interest.
        
        Args:
            brain_state: Full brain state
            roi: Region of interest specification
            
        Returns:
            Data for the specified ROI
        """
        # Extract data for the specified ROI
        # This is a simple placeholder implementation
        return {
            "roi": roi,
            "data": "roi_specific_data_placeholder"
        }

    async def _send_brain_structure(self, client_id: str) -> None:
        """
        Send brain structure information to a specific client.
        
        Args:
            client_id: Client identifier
        """
        # Skip if in standby mode
        if not self._active_mode:
            await self.control_socket.send_multipart([
                client_id.encode(),
                b"",
                b"application/json",
                serialize_message({
                    "type": "structure_unavailable",
                    "reason": "standby_mode",
                    "timestamp": int(time.time() * 1000)
                }, "application/json")
            ])
            return
            
        structure = await self.core_api.get_brain_structure()
        
        # Send structure directly to specified client
        await self.control_socket.send_multipart([
            client_id.encode(),
            b"",
            b"application/json",
            serialize_message({
                "type": "structure_update",
                "timestamp": int(time.time() * 1000),
                "data": structure
            }, "application/json")
        ])

    async def _send_brain_structure_to_all(self, structure: Dict) -> None:
        """
        Send brain structure information to all connected clients.
        
        Args:
            structure: Brain structure data
        """
        # Skip if in standby mode
        if not self._active_mode:
            return
            
        # Publish the structure to all subscribers
        await self.structure_socket.send_multipart([
            b"structure",
            b"application/json",
            serialize_message({
                "timestamp": int(time.time() * 1000),
                "data": structure
            }, "application/json")
        ])
        
    async def _send_brain_structure_to_all_force(self) -> None:
        """Force sending brain structure to all clients even after state changes."""
        try:
            # Skip if still in standby mode
            if not self._active_mode:
                return
                
            # Add small delay to ensure genome is fully loaded
            await asyncio.sleep(0.5)
                
            # Get current brain structure
            structure = await self.core_api.get_brain_structure()
            
            # Send to all clients
            await self._send_brain_structure_to_all(structure)
            
        except Exception as e:
            logger.error(f"Error sending forced structure update: {e}")

    async def send_fcl_visualization_data(self):
        """
        Collect FCL data from FCL manager and send it through the visualization stream.
        
        This method:
        1. Gets the current FCL data by area
        2. Gathers neuron positions from the connectome manager
        3. Formats data for visualization clients
        4. Sends the prepared data through the stream
        """
        # Check if we're in active mode
        if not self._active_mode:
            await self.send_status_update({"status": "standby", "message": "No genome loaded"})
            return
            
        # Get state manager
        if not hasattr(self.core_api, 'get_state_manager'):
            logger.warning("Core API missing get_state_manager method, visualization data unavailable")
            return
            
        state_manager = self.core_api.get_state_manager()
        if not state_manager:
            logger.warning("State manager not available, visualization data unavailable")
            return
        
        # Get FCL manager
        fcl_manager = state_manager.get_fcl_manager()
        if not fcl_manager:
            logger.warning("FCL manager not available, visualization data unavailable")
            return
            
        # Get connectome manager
        connectome_manager = state_manager.get_connectome()
        if not connectome_manager:
            logger.warning("Connectome manager not available, visualization data unavailable")
            return
            
        # Get current FCL data by area
        fcl_by_area = fcl_manager.get_fcl_by_area()
        if not fcl_by_area:
            # No activity, just send empty data
            await self.send_activity_update({
                "timestamp": int(time.time() * 1000),
                "fcl_by_area": {}
            })
            return
            
        # Convert FCL data to serializable format
        fcl_data = {}
        for area_id, neuron_bitmap in fcl_by_area.items():
            # Convert to regular list for serialization
            fcl_data[area_id] = list(neuron_bitmap)
            
        # Prepare visualization data
        activity_data = {
            "timestamp": int(time.time() * 1000),
            "fcl_by_area": fcl_data
        }
        
        # Send activity data
        await self.send_activity_update(activity_data)
        
        # Periodically send structure data (less frequently)
        current_time = time.time()
        if not hasattr(self, '_last_structure_update') or current_time - self._last_structure_update > 5.0:
            await self.send_structure_update()
            self._last_structure_update = current_time
    
    async def send_structure_update(self):
        """Send brain structure data to visualization clients."""
        # Check if we're in active mode
        if not self._active_mode:
            return
            
        # Get state manager
        state_manager = self.core_api.get_state_manager()
        if not state_manager:
            return
            
        # Get connectome manager
        connectome_manager = state_manager.get_connectome()
        if not connectome_manager:
            return
            
        # Build structure data
        structure_data = {
            "timestamp": int(time.time() * 1000),
            "cortical_areas": {}
        }
        
        # Get all cortical areas
        try:
            # Add cortical area information
            for area_id, area in connectome_manager.cortical_areas.items():
                area_data = {
                    "name": area.name,
                    "dimensions": area.dimensions,
                    "position": area.position,
                    "area_type": area.area_type,
                    # We don't send all neurons to avoid huge messages
                    # Clients can request specific neuron data if needed
                    "neuron_count": len(connectome_manager.get_neurons_by_area(area_id))
                }
                structure_data["cortical_areas"][area_id] = area_data
                
            # Add genome information
            structure_data["genome"] = {
                "timestamp": state_manager.genome_timestamp if hasattr(state_manager, 'genome_timestamp') else None
            }
            
            # Send structure data
            await self.send_structure_message(structure_data)
            
        except Exception as e:
            logger.exception(f"Error preparing structure data: {e}")
            
    async def start_fcl_collector(self, interval=0.1):
        """
        Start periodic collection of FCL data for visualization.
        
        Args:
            interval: Time between FCL data collections in seconds
        """
        logger.info(f"Starting FCL data collector with interval {interval}s")
        
        # Create periodic task for FCL collection
        self.periodic_tasks['fcl_collector'] = asyncio.create_task(
            self._fcl_collector_loop(interval)
        )
    
    async def _fcl_collector_loop(self, interval):
        """
        Periodic loop to collect FCL data and send visualization updates.
        
        Args:
            interval: Time between updates in seconds
        """
        while self.running:
            try:
                await self.send_fcl_visualization_data()
            except Exception as e:
                logger.exception(f"Error in FCL collector: {e}")
            
            await asyncio.sleep(interval)

    async def send_activity_update(self, data):
        """
        Send activity update to clients.
        
        Args:
            data: Activity data to send
        """
        if not self.running:
            return
            
        try:
            # Send activity update
            await self.activity_socket.send_multipart([
                b"activity.base",
                b"application/json",
                serialize_message(data, "application/json")
            ])
            logger.debug("Sent activity update")
        except Exception as e:
            logger.error(f"Error sending activity update: {e}")
            
    async def send_structure_message(self, data):
        """
        Send structure data to clients.
        
        Args:
            data: Structure data to send
        """
        if not self.running:
            return
            
        try:
            # Send structure message
            await self.structure_socket.send_multipart([
                b"structure",
                b"application/json",
                serialize_message(data, "application/json")
            ])
            logger.debug("Sent structure update")
        except Exception as e:
            logger.error(f"Error sending structure update: {e}")
            
    async def send_status_update(self, status_data):
        """
        Send status update to clients.
        
        Args:
            status_data: Status data to send
        """
        if not self.running:
            return
            
        try:
            # Send system status message
            await self.activity_socket.send_multipart([
                b"system",
                b"application/json",
                serialize_message(status_data, "application/json")
            ])
            logger.debug("Sent status update")
        except Exception as e:
            logger.error(f"Error sending status update: {e}")


class VisualizationClient:
    """
    ZeroMQ Visualization Client implementation.
    
    This client connects to a Visualization Stream server and receives
    brain structure and activity data.
    """
    
    def __init__(
        self, 
        host: str = "localhost", 
        structure_port: int = 5560,
        activity_port: int = 5561,
        control_port: int = 5562,
        detail_level: int = 1,
        update_interval: float = 0.05,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize a new Visualization Client.
        
        Args:
            host: Visualization server host address
            structure_port: Port for structural data
            activity_port: Port for activity data
            control_port: Port for control messages
            detail_level: Initial detail level (1-3)
            update_interval: Desired update interval in seconds
            context: Optional existing ZMQ context to use
        """
        self.host = host
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        self.client_id = str(uuid.uuid4())
        
        # Socket for structural data
        self.structure_socket = self.context.socket(zmq.SUB)
        self.structure_socket.connect(f"tcp://{host}:{structure_port}")
        self.structure_socket.setsockopt(zmq.SUBSCRIBE, b"structure")
        
        # Socket for activity data
        self.activity_socket = self.context.socket(zmq.SUB)
        self.activity_socket.connect(f"tcp://{host}:{activity_port}")
        self.activity_socket.setsockopt(zmq.SUBSCRIBE, b"activity.base")
        
        # Socket for control messages
        self.control_socket = self.context.socket(zmq.DEALER)
        self.control_socket.setsockopt(zmq.IDENTITY, self.client_id.encode())
        self.control_socket.connect(f"tcp://{host}:{control_port}")
        
        # Initial settings
        self.settings = {
            "detail_level": detail_level,
            "update_interval": update_interval,
            "roi": None
        }
        
        # Callback registry
        self.structure_callback = None
        self.activity_callback = None
        
        # Set up subscriptions based on detail level
        self._update_subscriptions()

    def _update_subscriptions(self) -> None:
        """Update activity subscriptions based on current detail level."""
        detail_level = self.settings["detail_level"]
        
        # Subscribe to appropriate detail levels
        for level in range(1, 4):  # Levels 1-3
            if level <= detail_level:
                self.activity_socket.setsockopt(zmq.SUBSCRIBE, 
                                              f"activity.detail.{level}".encode())
            else:
                self.activity_socket.setsockopt(zmq.UNSUBSCRIBE, 
                                              f"activity.detail.{level}".encode())
        
        # Subscribe to ROI-specific messages if an ROI is set
        if self.settings["roi"]:
            self.activity_socket.setsockopt(zmq.SUBSCRIBE, 
                                          f"activity.roi.{self.client_id}".encode())
        else:
            self.activity_socket.setsockopt(zmq.UNSUBSCRIBE, 
                                          f"activity.roi.{self.client_id}".encode())

    async def start(self) -> None:
        """Start the visualization client."""
        logger.info(f"Starting Visualization client to {self.host}")
        self.running = True
        
        # Register with the server
        await self._register()
        
        # Start receivers
        asyncio.create_task(self._receive_structure_updates())
        asyncio.create_task(self._receive_activity_updates())
        asyncio.create_task(self._receive_control_responses())

    async def stop(self) -> None:
        """Stop the visualization client."""
        logger.info("Stopping Visualization client")
        
        # Unregister from the server
        await self._unregister()
        
        self.running = False
        self.structure_socket.close()
        self.activity_socket.close()
        self.control_socket.close()

    async def _register(self) -> None:
        """Register with the visualization server."""
        await self.control_socket.send_multipart([
            b"",
            b"application/json",
            serialize_message({
                "type": "register",
                "settings": self.settings
            }, "application/json")
        ])

    async def _unregister(self) -> None:
        """Unregister from the visualization server."""
        await self.control_socket.send_multipart([
            b"",
            b"application/json",
            serialize_message({
                "type": "unregister"
            }, "application/json")
        ])

    async def set_detail_level(self, level: int) -> None:
        """
        Set the visualization detail level.
        
        Args:
            level: Detail level (1-3)
        """
        if level < 1 or level > 3:
            raise ValueError("Detail level must be between 1 and 3")
            
        self.settings["detail_level"] = level
        self._update_subscriptions()
        
        # Notify the server
        await self.control_socket.send_multipart([
            b"",
            b"application/json",
            serialize_message({
                "type": "view_settings",
                "settings": {"detail_level": level}
            }, "application/json")
        ])

    async def set_region_of_interest(self, roi: Optional[Dict]) -> None:
        """
        Set a region of interest for focused visualization.
        
        Args:
            roi: Region specification or None to clear
        """
        self.settings["roi"] = roi
        self._update_subscriptions()
        
        # Notify the server
        await self.control_socket.send_multipart([
            b"",
            b"application/json",
            serialize_message({
                "type": "view_settings",
                "settings": {"roi": roi}
            }, "application/json")
        ])

    async def set_update_interval(self, interval: float) -> None:
        """
        Set the desired update interval.
        
        Args:
            interval: Update interval in seconds
        """
        if interval <= 0:
            raise ValueError("Update interval must be positive")
            
        self.settings["update_interval"] = interval
        
        # Notify the server
        await self.control_socket.send_multipart([
            b"",
            b"application/json",
            serialize_message({
                "type": "view_settings",
                "settings": {"update_interval": interval}
            }, "application/json")
        ])

    def register_structure_callback(self, callback) -> None:
        """
        Register a callback for brain structure updates.
        
        Args:
            callback: Function to call with structure data
        """
        self.structure_callback = callback

    def register_activity_callback(self, callback) -> None:
        """
        Register a callback for brain activity updates.
        
        Args:
            callback: Function to call with activity data
        """
        self.activity_callback = callback

    async def _receive_structure_updates(self) -> None:
        """Receive and process brain structure updates."""
        while self.running:
            try:
                multipart = await self.structure_socket.recv_multipart()
                
                # Expecting [topic, content_type, data]
                if len(multipart) < 3:
                    logger.error(f"Received malformed structure message: {multipart}")
                    continue
                
                topic = multipart[0].decode()
                content_type = multipart[1].decode()
                data = deserialize_message(multipart[2], content_type)
                
                logger.debug(f"Received structure update")
                
                # Call the registered callback if any
                if self.structure_callback:
                    try:
                        await self.structure_callback(data)
                    except Exception as e:
                        logger.error(f"Error in structure callback: {e}")
            
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error receiving structure update: {e}")
                await asyncio.sleep(1)

    async def _receive_activity_updates(self) -> None:
        """Receive and process brain activity updates."""
        while self.running:
            try:
                multipart = await self.activity_socket.recv_multipart()
                
                # Expecting [topic, content_type, data]
                if len(multipart) < 3:
                    logger.error(f"Received malformed activity message: {multipart}")
                    continue
                
                topic = multipart[0].decode()
                content_type = multipart[1].decode()
                data = deserialize_message(multipart[2], content_type)
                
                logger.debug(f"Received activity update: {topic}")
                
                # Call the registered callback if any
                if self.activity_callback:
                    try:
                        await self.activity_callback(topic, data)
                    except Exception as e:
                        logger.error(f"Error in activity callback: {e}")
            
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error receiving activity update: {e}")
                await asyncio.sleep(1)

    async def _receive_control_responses(self) -> None:
        """Receive responses to control messages."""
        while self.running:
            try:
                multipart = await self.control_socket.recv_multipart()
                
                # Expecting [empty, content_type, data]
                if len(multipart) < 3:
                    logger.error(f"Received malformed control response: {multipart}")
                    continue
                
                content_type = multipart[1].decode()
                data = deserialize_message(multipart[2], content_type)
                
                logger.debug(f"Received control response: {data}")
                
                # Process specific responses if needed
            
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error receiving control response: {e}")
                await asyncio.sleep(1) 