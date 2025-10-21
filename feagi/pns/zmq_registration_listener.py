"""
ZMQ Registration Listener - Agent registration via ZMQ REQ/REP

This module provides ZMQ-based agent registration for Python FEAGI,
enabling agents to register via ZMQ (like video_agent_rust) in addition
to REST API registration.

Protocol: ZMQ REQ/REP pattern
Port: 5000 (default, configurable)
Format: JSON messages
"""

import json
import logging
import threading
import zmq
from typing import Optional

logger = logging.getLogger(__name__)


class ZmqRegistrationListener:
    """
    ZMQ-based agent registration listener
    
    Listens for registration requests via ZMQ REQ/REP and delegates
    to the RegistrationManager (Rust-backed).
    """
    
    def __init__(self, registration_manager, host="*", port=5000):
        """
        Initialize ZMQ registration listener
        
        Args:
            registration_manager: RegistrationManager instance
            host: Host to bind to (default "*" for all interfaces)
            port: Port to bind to (default 5000)
        """
        self.registration_manager = registration_manager
        self.host = host
        self.port = port
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # ZMQ context and socket (created in start())
        self.context: Optional[zmq.Context] = None
        self.socket: Optional[zmq.Socket] = None
    
    def start(self):
        """Start the ZMQ registration listener in a background thread"""
        if self.running:
            logger.warning("ZMQ registration listener already running")
            return
        
        self.running = True
        self.thread = threading.Thread(
            target=self._listener_loop,
            name="ZMQ-Registration-Listener",
            daemon=True
        )
        self.thread.start()
        logger.info(f"🦀 ZMQ registration listener started on tcp://{self.host}:{self.port}")
    
    def stop(self):
        """Stop the ZMQ registration listener"""
        if not self.running:
            return
        
        logger.info("Stopping ZMQ registration listener...")
        self.running = False
        
        # Close socket and context
        if self.socket:
            try:
                self.socket.close()
            except Exception as e:
                logger.warning(f"Error closing socket: {e}")
        
        if self.context:
            try:
                self.context.term()
            except Exception as e:
                logger.warning(f"Error terminating context: {e}")
        
        # Wait for thread to finish
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        logger.info("ZMQ registration listener stopped")
    
    def _listener_loop(self):
        """Main listener loop (runs in background thread)"""
        try:
            # Create ZMQ context and socket
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.REP)
            
            # Bind to address
            bind_address = f"tcp://{self.host}:{self.port}"
            self.socket.bind(bind_address)
            logger.info(f"🦀 ZMQ registration listener bound to {bind_address}")
            
            # Set receive timeout so we can check self.running periodically
            self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
            
            while self.running:
                try:
                    # Receive registration request (with timeout)
                    message = self.socket.recv_string()
                    
                    # Process request
                    response = self._handle_registration_request(message)
                    
                    # Send response
                    self.socket.send_string(response)
                    
                except zmq.Again:
                    # Timeout - no message received, continue loop
                    continue
                except Exception as e:
                    logger.error(f"Error processing registration request: {e}", exc_info=True)
                    # Send error response
                    try:
                        error_response = json.dumps({
                            "status": "error",
                            "message": f"Internal error: {str(e)}"
                        })
                        self.socket.send_string(error_response)
                    except Exception:
                        pass
        
        except Exception as e:
            logger.error(f"Fatal error in ZMQ registration listener: {e}", exc_info=True)
        finally:
            logger.info("ZMQ registration listener loop exited")
    
    def _handle_registration_request(self, message: str) -> str:
        """
        Handle a registration request message
        
        Args:
            message: JSON string with registration request
        
        Returns:
            JSON string with registration response
        """
        try:
            # Parse request
            request_data = json.loads(message)
            request_type = request_data.get("type", "")
            
            logger.info(f"📥 ZMQ registration request: type={request_type}, agent_id={request_data.get('agent_id')}")
            
            if request_type == "register":
                return self._handle_register(request_data)
            elif request_type == "deregister":
                return self._handle_deregister(request_data)
            elif request_type == "heartbeat":
                return self._handle_heartbeat(request_data)
            else:
                return json.dumps({
                    "status": "error",
                    "message": f"Unknown request type: {request_type}"
                })
        
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON in registration request: {e}")
            return json.dumps({
                "status": "error",
                "message": f"Invalid JSON: {str(e)}"
            })
        except Exception as e:
            logger.error(f"Error handling registration request: {e}", exc_info=True)
            return json.dumps({
                "status": "error",
                "message": f"Internal error: {str(e)}"
            })
    
    def _handle_register(self, request_data: dict) -> str:
        """Handle agent registration request"""
        from feagi.pns.registration_manager import AgentRegistrationRequest
        
        try:
            # Extract request fields
            agent_id = request_data.get("agent_id", "")
            agent_type = request_data.get("agent_type", "")
            capabilities = request_data.get("capabilities", {})
            agent_ip = request_data.get("agent_ip", "unknown")
            metadata = request_data.get("metadata", {})
            
            # Create registration request
            request = AgentRegistrationRequest(
                agent_id=agent_id,
                agent_type=agent_type,
                agent_ip=agent_ip,
                capabilities=capabilities,
                metadata=metadata
            )
            
            # Register with RegistrationManager
            response = self.registration_manager.register_agent(request)
            
            # Build response
            if response.success:
                result = {
                    "status": "success",
                    "agent_id": response.agent_id,
                    "message": response.message,
                    "endpoints": response.transport_info
                }
                logger.info(f"✅ Agent registered via ZMQ: {agent_id}")
            else:
                result = {
                    "status": "error",
                    "agent_id": response.agent_id,
                    "message": response.message,
                    "error_code": response.error_code
                }
                logger.warning(f"❌ Agent registration failed via ZMQ: {agent_id} - {response.message}")
            
            return json.dumps(result)
        
        except Exception as e:
            logger.error(f"Error in register handler: {e}", exc_info=True)
            return json.dumps({
                "status": "error",
                "message": f"Registration failed: {str(e)}"
            })
    
    def _handle_deregister(self, request_data: dict) -> str:
        """Handle agent deregistration request"""
        try:
            agent_id = request_data.get("agent_id", "")
            
            # Deregister with RegistrationManager
            response = self.registration_manager.deregister_agent(agent_id)
            
            # Build response
            if response.success:
                result = {
                    "status": "success",
                    "agent_id": response.agent_id,
                    "message": response.message
                }
                logger.info(f"✅ Agent deregistered via ZMQ: {agent_id}")
            else:
                result = {
                    "status": "error",
                    "agent_id": response.agent_id,
                    "message": response.message
                }
                logger.warning(f"❌ Agent deregistration failed via ZMQ: {agent_id}")
            
            return json.dumps(result)
        
        except Exception as e:
            logger.error(f"Error in deregister handler: {e}", exc_info=True)
            return json.dumps({
                "status": "error",
                "message": f"Deregistration failed: {str(e)}"
            })
    
    def _handle_heartbeat(self, request_data: dict) -> str:
        """Handle agent heartbeat request"""
        try:
            agent_id = request_data.get("agent_id", "")
            
            # Update activity
            success = self.registration_manager.heartbeat_agent(agent_id)
            
            if success:
                return json.dumps({
                    "status": "success",
                    "agent_id": agent_id,
                    "message": "Heartbeat acknowledged"
                })
            else:
                return json.dumps({
                    "status": "error",
                    "agent_id": agent_id,
                    "message": "Agent not found"
                })
        
        except Exception as e:
            logger.error(f"Error in heartbeat handler: {e}", exc_info=True)
            return json.dumps({
                "status": "error",
                "message": f"Heartbeat failed: {str(e)}"
            })

