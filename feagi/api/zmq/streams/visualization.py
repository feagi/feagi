"""
ZeroMQ Visualization Stream Implementation for FEAGI API

This module implements the specialized streaming pattern for brain visualization data.
It provides:
- Efficient brain activity visualization streaming
- Level-of-detail mechanisms for performance optimization
- Client view control and filtering
"""

import asyncio
import logging
import time
import uuid
from typing import Dict, Any, List, Optional, Set, Tuple, Union

import zmq
import zmq.asyncio
import numpy as np

from ...core.service import CoreApiService
from ..serialization import serialize_message, deserialize_message
from ...utils.rate_limit import RateLimiter

logger = logging.getLogger(__name__)


class VisualizationStream:
    """
    ZeroMQ Visualization Stream implementation.
    
    This specialized stream efficiently broadcasts neural activity and structural data
    to visualization clients, with support for level-of-detail streaming.
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
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        
        # Socket for structural data (changes infrequently)
        self.structure_socket = self.context.socket(zmq.PUB)
        self.structure_socket.bind(f"tcp://{host}:{structure_port}")
        
        # Socket for real-time activity data (high frequency)
        self.activity_socket = self.context.socket(zmq.PUB)
        self.activity_socket.bind(f"tcp://{host}:{activity_port}")
        
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

    async def start(self) -> None:
        """Start the visualization stream server."""
        logger.info(f"Starting Visualization Stream server")
        self.running = True
        
        # Start control socket handler
        self.periodic_tasks["control_handler"] = asyncio.create_task(
            self._handle_control_messages()
        )
        
        # Start activity streaming task
        self.periodic_tasks["activity_stream"] = asyncio.create_task(
            self._stream_activity()
        )
        
        # Start structure update task (lower frequency)
        self.periodic_tasks["structure_updates"] = asyncio.create_task(
            self._stream_structure_updates()
        )

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
        
        # Send current brain structure to new client
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

    async def _stream_structure_updates(self) -> None:
        """Periodically check for and stream brain structure updates."""
        while self.running:
            try:
                # Check if brain structure has changed
                current_structure = await self.core_api.get_brain_structure()
                
                # Compare with last known structure and send if different
                structure_hash = hash(str(current_structure))
                if getattr(self, '_last_structure_hash', None) != structure_hash:
                    await self._send_brain_structure_to_all(current_structure)
                    self._last_structure_hash = structure_hash
                
                # Structure updates are less frequent
                await asyncio.sleep(5.0)  # Check every 5 seconds
                
            except asyncio.CancelledError:
                logger.debug("Structure update task cancelled")
                break
            except Exception as e:
                logger.error(f"Error streaming structure updates: {e}")
                await asyncio.sleep(5)  # Longer sleep on errors

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
        # Publish the structure to all subscribers
        await self.structure_socket.send_multipart([
            b"structure",
            b"application/json",
            serialize_message({
                "timestamp": int(time.time() * 1000),
                "data": structure
            }, "application/json")
        ])


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