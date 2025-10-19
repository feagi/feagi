"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
FEAGI Client

Main client interface for connecting to FEAGI and exchanging data through four stream types:
1. Control Stream: Bidirectional JSON-based messaging for control commands and health status
2. Sensory Stream: One-directional binary data flow from agent to FEAGI
3. Motor Stream: One-directional binary data flow from FEAGI to agent
4. Visualization Stream: One-directional binary data flow from FEAGI to agent for neuron activity
"""

import asyncio
import json
import logging
import uuid
import time
from typing import Dict, Any, Optional, List, Union, Tuple, Callable, Awaitable

import zmq
import zmq.asyncio
# from feagi_bytes import ByteStructureEncoder, ByteStructureDecoder, ByteStructureID

from feagi_connector.api.command_client import FeagiControlClient
from feagi_connector.api.sensory_client import FeagiSensoryClient
from feagi_connector.api.motor_client import FeagiMotorClient
from feagi_connector.api.viz_client import FeagiVizClient
from feagi_connector.utils import NEURON_POTENTIAL_CATEGORICAL_XYZ
from feagi_connector.utils.processing import (
    infer_byte_structure_type_python as infer_byte_structure_type,
    extract_sub_structures_python as extract_sub_structures
)

# Configure logging
logger = logging.getLogger("feagi_connector")


class FeagiClient:
    """
    High-level client for connecting to FEAGI.
    
    This client provides a unified interface for all FEAGI communication streams:
    1. Control Stream: Bidirectional JSON-based messaging for control commands and health status
    2. Sensory Stream: One-directional binary data flow from agent to FEAGI
    3. Motor Stream: One-directional binary data flow from FEAGI to agent
    4. Visualization Stream: One-directional binary data flow from FEAGI to agent for neuron activity
    
    It handles connection management, agent registration, and data exchange.
    """
    
    def __init__(
        self,
        host: str = "127.0.0.1",
        control_port: int = 5555,
        rest_port: int = 5563,
        sensory_port: int = 5558,
        motor_port: int = 5564,
        visualization_port: int = 5562,
        agent_id: Optional[str] = None,
        agent_type: str = "external",
        timeout: int = 5000,
    ):
        """
        Initialize the FEAGI client.
        
        Args:
            host: FEAGI hostname or IP
            control_port: Control stream port (default 5555)
            rest_port: REST Stream API port (default 5563)
            sensory_port: Sensory stream port (default 5558)
            motor_port: Motor stream port (default 5564)
            visualization_port: Visualization stream port (default 5562)
            agent_id: Agent ID for FEAGI registration (default: auto-generated)
            agent_type: Agent type for categorization
            timeout: Socket timeout in milliseconds
        """
        self.host = host
        self.agent_id = agent_id or f"agent-{uuid.uuid4().hex[:8]}"
        self.agent_type = agent_type
        self.timeout = timeout
        
        # Initialize ZMQ context
        self.context = zmq.asyncio.Context.instance()
        
        # Initialize individual clients for different FEAGI streams
        self.command_client = FeagiControlClient(
            host=host, 
            port=control_port,
            timeout=timeout
        )
        
        # REST client for agent registration
        self.rest_client = FeagiControlClient(host=host, port=rest_port, timeout=timeout)
        
        self.sensory_client = FeagiSensoryClient(
            host=host,
            port=sensory_port,
            timeout=timeout
        )
        
        self.motor_client = FeagiMotorClient(
            host=host,
            port=motor_port,
            agent_id=agent_id,
            socket_timeout=timeout
        )
        
        self.viz_client = FeagiVizClient(
            host=host,
            port=visualization_port,
            agent_id=agent_id,
            socket_timeout=timeout
        )
        
        # State
        self.connected = False
        self.registered = False
        self.motor_callback = None
        self.visualization_callback = None
        self.shared_memory_paths: Dict[str, str] = {}
        
        # Tasks
        self.heartbeat_task = None
        self.motor_listen_task = None
        self.viz_listen_task = None
        # Reused HTTP session for heartbeats (best-effort)
        self._http_session = None
    
    async def register_with_capabilities(
        self,
        capabilities_file: str = None,
        capabilities_data: Dict[str, Any] = None
    ) -> bool:
        """
        Register agent with FEAGI using capabilities from file or data.
        
        Args:
            capabilities_file: Path to capabilities.json file
            capabilities_data: Capabilities data dictionary (alternative to file)
            
        Returns:
            True if registration was successful
        """
        import json
        import os
        
        # Load capabilities
        if capabilities_data:
            full_capabilities = capabilities_data
        elif capabilities_file and os.path.exists(capabilities_file):
            try:
                with open(capabilities_file, 'r') as f:
                    full_capabilities = json.load(f)
            except Exception as e:
                logger.error(f"Failed to load capabilities file {capabilities_file}: {e}")
                return False
        else:
            logger.warning("No capabilities file or data provided, using defaults")
            full_capabilities = None
        
        # Extract simple capabilities from full structure
        simple_capabilities = {
            "sensory": False,
            "motor": False,
            "visualization": bool(self.visualization_callback)
        }
        
        if full_capabilities and "capabilities" in full_capabilities:
            caps = full_capabilities["capabilities"]
            simple_capabilities["sensory"] = bool(caps.get("input", {}))
            simple_capabilities["motor"] = bool(caps.get("output", {}))
        
        # Register via REST Stream API
        registration_result = await self.rest_client.register_agent(
            agent_id=self.agent_id,
            agent_type=self.agent_type,
            capabilities=simple_capabilities,
            full_capabilities=full_capabilities
        )
        
        if "error" in registration_result:
            logger.error(f"Failed to register agent: {registration_result['error']}")
            return False
        
        # Check if registration was successful
        response_status = registration_result.get("status", 500)
        if response_status == 200:
            response_body = registration_result.get("body", {})
            logger.info(f"✅ Agent registered successfully: {response_body.get('message', 'OK')}")
            # Parse SHM paths if included in message (JSON)
            try:
                msg = response_body.get("message", "")
                if isinstance(msg, str) and msg:
                    import json as _json
                    parsed = _json.loads(msg)
                    shm = parsed.get("shared_memory") or {}
                    if isinstance(shm, dict):
                        self.shared_memory_paths = {str(k): str(v) for k, v in shm.items()}
                        logger.info(f"[SHM] Received shared memory paths: {self.shared_memory_paths}")
            except Exception as e:
                logger.debug(f"[SHM] Unable to parse shared memory from registration: {e}")
            
            # Log FQ sampler coordination info if available
            fq_info = response_body.get("fq_samplers_enabled", {})
            if fq_info:
                logger.info(f"🔄 FQ Samplers enabled: {fq_info}")
            
            self.registered = True
            return True
        else:
            logger.error(f"❌ Agent registration failed with status {response_status}: {registration_result}")
            return False

    def get_shared_memory_paths(self) -> Dict[str, str]:
        """Return shared memory paths provided by FEAGI (if any)."""
        return dict(self.shared_memory_paths)

    async def check_feagi_readiness(self) -> Dict[str, Any]:
        """
        Check if FEAGI is ready to accept agent connections.
        
        Returns:
            Dict containing readiness status from health_check endpoint
        """
        try:
            # Use HTTP REST API for health check (not ZMQ)
            import aiohttp
            import asyncio
            
            # Calculate HTTP port - usually 8000 for FEAGI HTTP REST API
            # The rest_port parameter refers to ZMQ REST Stream, but we need HTTP
            http_port = 8000  # Standard FEAGI HTTP port
            
            async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=5)) as session:
                url = f"http://{self.host}:{http_port}/v1/system/health_check"
                async with session.get(url) as response:
                    if response.status == 200:
                        health_data = await response.json()
                        
                        # FEAGI is ready if genome is available and burst engine is running
                        genome_available = health_data.get("genome_availability", False)
                        burst_engine_ready = health_data.get("burst_engine", False)
                        brain_ready = health_data.get("brain_readiness", False)
                        
                        ready = genome_available and burst_engine_ready and brain_ready
                        
                        # Determine reason and required actions
                        required_actions = []
                        reason = None
                        
                        if not genome_available:
                            required_actions.append("Load a genome via /v1/genome/upload/file")
                            reason = "genome_not_loaded"
                        
                        if not burst_engine_ready:
                            required_actions.append("Start burst engine via /v1/burst_engine/start")
                            if reason:
                                reason = "genome_not_loaded_and_burst_engine_not_ready"
                            else:
                                reason = "burst_engine_not_ready"
                        
                        return {
                            "ready": ready,
                            "reason": reason,
                            "details": {
                                "genome_available": genome_available,
                                "burst_engine_ready": burst_engine_ready,
                                "brain_ready": brain_ready
                            },
                            "required_actions": required_actions
                        }
                    else:
                        return {
                            "ready": False,
                            "reason": "health_check_failed",
                            "details": {"error": f"HTTP {response.status}"},
                            "required_actions": ["Check if FEAGI is running"]
                        }
                
        except ImportError:
            # Fallback to requests if aiohttp not available
            try:
                import requests
                http_port = 8000
                url = f"http://{self.host}:{http_port}/v1/system/health_check"
                response = requests.get(url, timeout=5)
                
                if response.status_code == 200:
                    health_data = response.json()
                    
                    # FEAGI is ready if genome is available and burst engine is running
                    genome_available = health_data.get("genome_availability", False)
                    burst_engine_ready = health_data.get("burst_engine", False)
                    brain_ready = health_data.get("brain_readiness", False)
                    
                    ready = genome_available and burst_engine_ready and brain_ready
                    
                    # Determine reason and required actions
                    required_actions = []
                    reason = None
                    
                    if not genome_available:
                        required_actions.append("Load a genome via /v1/genome/upload/file")
                        reason = "genome_not_loaded"
                    
                    if not burst_engine_ready:
                        required_actions.append("Start burst engine via /v1/burst_engine/start")
                        if reason:
                            reason = "genome_not_loaded_and_burst_engine_not_ready"
                        else:
                            reason = "burst_engine_not_ready"
                    
                    return {
                        "ready": ready,
                        "reason": reason,
                        "details": {
                            "genome_available": genome_available,
                            "burst_engine_ready": burst_engine_ready,
                            "brain_ready": brain_ready
                        },
                        "required_actions": required_actions
                    }
                else:
                    return {
                        "ready": False,
                        "reason": "health_check_failed", 
                        "details": {"error": f"HTTP {response.status_code}"},
                        "required_actions": ["Check if FEAGI is running"]
                    }
            except ImportError:
                return {
                    "ready": False,
                    "reason": "missing_http_client",
                    "details": {"error": "Neither aiohttp nor requests available"},
                    "required_actions": ["Install aiohttp or requests: pip install aiohttp"]
                }
        except Exception as e:
            return {
                "ready": False,
                "reason": "connection_failed",
                "details": {"error": str(e)},
                "required_actions": ["Check if FEAGI is running and accessible"]
            }

    async def connect_with_readiness_check(self, timeout: float = 60.0, show_guidance: bool = True) -> bool:
        """
        Connect to FEAGI with readiness validation and user guidance.
        
        Args:
            timeout: Maximum time to wait for FEAGI to become ready
            show_guidance: Whether to show user guidance messages
            
        Returns:
            True if connected successfully, False otherwise
        """
        start_time = time.time()
        attempt = 0
        guidance_shown = False
        
        logger.info("Connecting to FEAGI...")
        
        while time.time() - start_time < timeout:
            attempt += 1
            
            # Check if FEAGI is ready
            readiness = await self.check_feagi_readiness()
            
            if readiness["ready"]:
                # FEAGI is ready, proceed with normal connection
                logger.info("✅ FEAGI is ready - establishing connection...")
                return await self.connect()
            else:
                # Not ready, provide user guidance
                reason = readiness.get("reason", "unknown")
                actions = readiness.get("required_actions", [])
                
                if attempt == 1:
                    logger.warning(f"🔒 FEAGI not ready: {reason}")
                    
                    if show_guidance and not guidance_shown:
                        guidance_shown = True
                        self._show_readiness_guidance(reason, actions)
                
                # Calculate delay with exponential backoff (max 10s)
                delay = min(2.0 * (1.5 ** (attempt - 1)), 10.0)
                logger.info(f"Retrying in {delay:.1f}s... (attempt {attempt})")
                await asyncio.sleep(delay)
        
        logger.error(f"❌ FEAGI did not become ready within {timeout:.1f}s timeout")
        return False

    def _show_readiness_guidance(self, reason: str, required_actions: List[str]) -> None:
        """Show user-friendly guidance for making FEAGI ready."""
        if reason == "genome_not_loaded":
            logger.info("""
🧠 FEAGI needs a genome to process neural data.

To load a genome:
1. Open FEAGI web interface at http://localhost:8000
2. Go to Genome → Upload
3. Select a .json genome file
4. Or use API: POST /v1/genome/upload/file

Once loaded, your agent will automatically connect.
            """.strip())
        elif reason == "burst_engine_not_ready":
            logger.info("""
⚡ FEAGI's burst engine needs to be started.

To start the burst engine:
1. Ensure a genome is loaded first
2. Use API: POST /v1/burst_engine/start
3. Or restart FEAGI with a genome

Once started, your agent will automatically connect.
            """.strip())
        elif reason == "genome_not_loaded_and_burst_engine_not_ready":
            logger.info("""
🧠⚡ FEAGI needs both a genome and the burst engine to be ready.

To get FEAGI ready:
1. Load a genome:
   - Open FEAGI web interface at http://localhost:8000
   - Go to Genome → Upload and select a .json file
   - Or use API: POST /v1/genome/upload/file

2. Start the burst engine:
   - Use API: POST /v1/burst_engine/start
   - Or restart FEAGI (it will auto-start with a loaded genome)

Once both are ready, your agent will automatically connect.
            """.strip())
        else:
            logger.info(f"Required actions: {', '.join(required_actions)}")

    async def connect(self) -> bool:
        """Connect to FEAGI with proper registration."""
        return await self.connect_full()

    async def connect_full(self) -> bool:
        """Connect to FEAGI with all streams (full functionality)."""
        try:
            # Step 1: Connect all clients
            if not await self.command_client.connect():
                logger.error("Failed to connect to FEAGI command stream")
                return False
                
            if not self.sensory_client.connect():
                logger.error("Failed to connect to FEAGI sensory stream")
                return False
                
            # Step 2: Register agent if not already registered
            if not self.registered:
                try:
                    registration_result = await self.rest_client.register_agent(self.agent_id, self.agent_type)
                    if isinstance(registration_result, dict) and "error" in registration_result:
                        logger.warning(
                            f"⚠️ Agent registration error: {registration_result.get('error')} — continuing..."
                        )
                    else:
                        response_status = registration_result.get("status", 500) if isinstance(registration_result, dict) else 500
                        if response_status == 200:
                            self.registered = True
                            logger.info("✅ Agent registered successfully")
                        else:
                            logger.warning(
                                f"⚠️ Agent registration failed with status {response_status}, continuing..."
                            )
                except Exception as e:
                    logger.warning(f"⚠️ Agent registration exception: {e}, continuing...")
            
            # Step 3: Start motor and visualization listeners ONLY if callbacks are registered
            if self.motor_callback and hasattr(self.motor_client, 'register_motor_callback'):
                self.motor_client.register_motor_callback(self.motor_callback)
                if hasattr(self.motor_client, 'start'):
                    await self.motor_client.start()
                logger.info("✅ Motor client started with callback")
            else:
                logger.info("ℹ️ Motor client not started (no callback registered)")
                
            self.connected = True
            logger.info("✅ Successfully connected to FEAGI")
            # Start heartbeat after successful connect
            if not self.heartbeat_task:
                self.heartbeat_task = asyncio.create_task(self._heartbeat_loop())
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to FEAGI: {e}")
            return False

    async def connect_sensory_only(self) -> bool:
        """Connect to FEAGI sensory stream only (for data-sending agents like video)."""
        try:
            logger.info("🎯 Connecting in sensory-only mode (skipping control stream)...")
            
            # Step 1: Connect only sensory stream
            if not self.sensory_client.connect():
                logger.error("Failed to connect to FEAGI sensory stream")
                return False
                
            # Step 2: SKIP automatic registration - let caller handle registration with proper capabilities
            # (Automatic registration used default capabilities, causing issues with rate_hz)
            if False:  # Disabled - caller should register explicitly with correct capabilities
                try:
                    registration_result = await self.rest_client.register_agent(self.agent_id, self.agent_type)
                    if isinstance(registration_result, dict) and "error" in registration_result:
                        logger.warning(
                            f"⚠️ Agent registration error: {registration_result.get('error')} — continuing in sensory-only mode..."
                        )
                    else:
                        response_status = registration_result.get("status", 500) if isinstance(registration_result, dict) else 500
                        if response_status == 200:
                            self.registered = True
                            logger.info("✅ Agent registered successfully")
                        else:
                            logger.warning(
                                f"⚠️ Agent registration failed with status {response_status}, continuing in sensory-only mode..."
                            )
                except Exception as e:
                    logger.warning(f"⚠️ Agent registration exception: {e}, continuing in sensory-only mode...")
            
            self.connected = True
            logger.info("✅ Successfully connected to FEAGI")
            # Start heartbeat after successful connect
            if not self.heartbeat_task:
                self.heartbeat_task = asyncio.create_task(self._heartbeat_loop())
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to FEAGI: {e}")
            return False
    
    async def disconnect(self) -> None:
        """
        Disconnect from FEAGI.
        """
        self.connected = False
        
        # Send goodbye message
        try:
            await self.command_client.send_goodbye(agent_id=self.agent_id, agent_type=self.agent_type)
        except Exception as e:
            logger.warning(f"Error sending goodbye message: {e}")
        
        # Cancel background tasks
        if self.heartbeat_task:
            self.heartbeat_task.cancel()
            try:
                await self.heartbeat_task
            except asyncio.CancelledError:
                pass
            self.heartbeat_task = None
        
        if self.motor_listen_task:
            self.motor_listen_task.cancel()
            try:
                await self.motor_listen_task
            except asyncio.CancelledError:
                pass
            self.motor_listen_task = None
        
        if self.viz_listen_task:
            self.viz_listen_task.cancel()
            try:
                await self.viz_listen_task
            except asyncio.CancelledError:
                pass
            self.viz_listen_task = None
        
        # REST deregistration (best-effort) — only if we registered
        if self.registered:
            try:
                # REST Stream wrapper: route and method recognized by FEAGI transport
                await self.rest_client.make_rest_request({
                    "route": "/v1/agent/deregister",
                    "method": "DELETE",
                    "body": {"agent_id": self.agent_id},
                })
            except Exception:
                pass
            finally:
                self.registered = False

        # Close clients
        await self.command_client.close()
        self.sensory_client.close()  # This is now a synchronous method
        await self.motor_client.close()
        await self.viz_client.close()
        # Close shared HTTP session
        try:
            if self._http_session is not None:
                await self._http_session.close()
        except Exception:
            pass
        self._http_session = None
        
        logger.info("Disconnected from FEAGI")
    
    def send_sensory_data(
        self, cortical_area: str, neuron_data: Dict[Tuple[int, int, int], float]
    ) -> bool:
        """Send sensory data to FEAGI (synchronous)."""
        if not self.connected:
            logger.error("Not connected to FEAGI")
            return False
            
        return self.sensory_client.send_sensory_data(cortical_area, neuron_data)
    
    def register_motor_callback(
        self, 
        callback: Callable[[str, bytes], None]
    ) -> None:
        """
        Register a callback for motor data.
        
        Args:
            callback: Function to call when motor data is received
                     (parameters: channel_id, data)
        """
        self.motor_callback = callback
        if self.connected and not self.motor_listen_task:
            self.motor_listen_task = asyncio.create_task(self._motor_listen_loop())
    
    def register_visualization_callback(
        self,
        callback: Callable[[bytes], None]
    ) -> None:
        """
        Register a callback for visualization data.
        
        Args:
            callback: Function to call when visualization data is received
                     (parameter: data)
        """
        logger.info(f"Registering visualization callback function: {callback.__name__ if hasattr(callback, '__name__') else 'anonymous'}")
        self.visualization_callback = callback
        if self.connected and not self.viz_listen_task:
            logger.debug("Already connected, starting visualization listener task")
            self.viz_listen_task = asyncio.create_task(self._viz_listen_loop())
        elif not self.connected:
            logger.debug("Not connected yet, visualization listener will start on connection")
    
    async def get_status(self) -> Dict[str, Any]:
        """
        Get FEAGI system status.
        
        Returns:
            Dictionary with system status information
        """
        if not self.connected:
            logger.error("Not connected to FEAGI")
            return {"error": "Not connected"}
        
        return await self.command_client.get_status()
    
    async def _heartbeat_loop(self) -> None:
        """Send periodic heartbeats to keep the connection alive."""
        interval_sec = 10.0
        heartbeat_count = 0
        try:
            while self.connected:
                try:
                    # Send heartbeat over ZMQ control path
                    await self.command_client.send_heartbeat(
                        agent_id=self.agent_id,
                        agent_type=self.agent_type,
                    )
                    # Also notify FEAGI over HTTP API to keep registration fresh
                    try:
                        import aiohttp  # type: ignore
                        http_port = 8000  # Align with readiness check usage
                        url = f"http://{self.host}:{http_port}/v1/agent/heartbeat"
                        if self._http_session is None:
                            self._http_session = aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=3))
                        await self._http_session.post(url, json={"agent_id": self.agent_id})
                        heartbeat_count += 1
                        if heartbeat_count == 1 or heartbeat_count % 6 == 0:
                            logger.info(f"💓 Heartbeat #{heartbeat_count} sent to FEAGI")
                    except Exception as e:
                        # Best-effort HTTP heartbeat; continue even if unavailable
                        if heartbeat_count == 0:
                            logger.warning(f"⚠️ HTTP heartbeat failed: {e}")
                except Exception as e:
                    # Non-fatal; keep connection state and retry later
                    if heartbeat_count == 0:
                        logger.warning(f"⚠️ ZMQ heartbeat failed: {e}")
                await asyncio.sleep(interval_sec)
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Error in heartbeat loop: {e}")
    
    async def _motor_listen_loop(self) -> None:
        """Listen for motor data."""
        try:
            # Register callback and start motor client
            self.motor_client.register_motor_callback(self.motor_callback)
            await self.motor_client.start()
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Error in motor listen loop: {e}")
            self.connected = False
    
    async def _viz_listen_loop(self) -> None:
        """Listen for visualization data."""
        logger.info("Starting visualization listen loop")
        try:
            start_time = time.time()
            logger.debug(f"Calling viz_client.start_visualization_listener with callback: {self.visualization_callback.__name__ if hasattr(self.visualization_callback, '__name__') else 'anonymous'}")
            await self.viz_client.start_visualization_listener(self.visualization_callback)
            total_time = time.time() - start_time
            logger.info(f"Visualization listener exited after {total_time:.1f} seconds")
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            logger.info("Visualization listen loop cancelled")
            pass
        except Exception as e:
            logger.error(f"Error in visualization listen loop: {e}", exc_info=True)
            self.connected = False
        logger.debug("Visualization listen loop exited") 