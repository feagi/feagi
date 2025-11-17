"""
Example Brain Visualizer Client with Robust Heartbeat Implementation

This example demonstrates how to implement a robust brain visualizer agent that:
1. Registers with FEAGI with proper capabilities  
2. Sends regular heartbeats to maintain registration
3. Handles network failures gracefully
4. Implements proper cleanup on shutdown

Usage:
    python brain_visualizer_example.py --feagi-host localhost --feagi-port 8000
"""

import asyncio
import logging
import signal
import sys
import time
from typing import Optional, Dict, Any
import json
import requests
from datetime import datetime

logger = logging.getLogger(__name__)


class RobustBrainVisualizerClient:
    """
    Example brain visualizer client with robust heartbeat functionality.
    
    This client demonstrates best practices for FEAGI agent implementation:
    - Proper registration with capability declaration
    - Regular heartbeat sending with failure recovery  
    - Graceful shutdown handling
    - Network error resilience
    """
    
    def __init__(
        self, 
        feagi_host: str = "localhost",
        feagi_port: int = 8000,
        agent_id: str = "brain_visualizer",
        heartbeat_interval: float = 15.0  # Send heartbeat every 15 seconds
    ):
        self.feagi_host = feagi_host
        self.feagi_port = feagi_port
        self.agent_id = agent_id
        self.heartbeat_interval = heartbeat_interval
        
        self.base_url = f"http://{feagi_host}:{feagi_port}"
        self.registered = False
        self.running = False
        self.heartbeat_task: Optional[asyncio.Task] = None
        
        # Track heartbeat statistics
        self.heartbeat_count = 0
        self.heartbeat_failures = 0
        self.last_heartbeat_success = None
        
        logger.info(f"💗 Initialized BrainVisualizerClient for {self.base_url}")
    
    async def register(self) -> bool:
        """
        Register with FEAGI as a brain visualizer agent.
        
        Returns:
            True if registration successful, False otherwise
        """
        registration_data = {
            "agent_id": self.agent_id,
            "agent_type": "brain_visualizer",
            "capabilities": {
                "visualization": True,
                "sensory": False,
                "motor": False,
                "output": False
            },
            "metadata": {
                "version": "2.0.1",
                "platform": "python_client",
                "description": "Brain visualization client with robust heartbeat",
                "heartbeat_interval": self.heartbeat_interval
            }
        }
        
        try:
            logger.info(f"🔌 Registering agent '{self.agent_id}' with FEAGI...")
            
            response = requests.post(
                f"{self.base_url}/v1/agent/register",
                json=registration_data,
                timeout=10.0
            )
            
            if response.status_code == 200:
                result = response.json()
                self.registered = True
                logger.info(f"✅ Successfully registered agent '{self.agent_id}': {result.get('message')}")
                return True
            else:
                logger.error(f"❌ Registration failed: {response.status_code} - {response.text}")
                return False
                
        except Exception as e:
            logger.error(f"❌ Registration error: {e}")
            return False
    
    async def send_heartbeat(self) -> bool:
        """
        Send a heartbeat to FEAGI to maintain registration.
        
        Returns:
            True if heartbeat successful, False otherwise
        """
        if not self.registered:
            logger.warning("💔 Attempted to send heartbeat but not registered")
            return False
        
        heartbeat_data = {
            "agent_id": self.agent_id
        }
        
        try:
            response = requests.post(
                f"{self.base_url}/v1/agent/heartbeat",
                json=heartbeat_data,
                timeout=5.0
            )
            
            if response.status_code == 200:
                self.heartbeat_count += 1
                self.last_heartbeat_success = datetime.now()
                logger.debug(f"💗 Heartbeat #{self.heartbeat_count} sent successfully")
                return True
            else:
                self.heartbeat_failures += 1
                logger.warning(f"💔 Heartbeat failed: {response.status_code} - {response.text}")
                return False
                
        except Exception as e:
            self.heartbeat_failures += 1
            logger.warning(f"💔 Heartbeat error: {e}")
            return False
    
    async def heartbeat_loop(self) -> None:
        """
        Main heartbeat loop that runs continuously while the client is active.
        """
        logger.info(f"💗 Starting heartbeat loop (interval={self.heartbeat_interval}s)")
        
        consecutive_failures = 0
        max_consecutive_failures = 5
        
        while self.running:
            try:
                success = await self.send_heartbeat()
                
                if success:
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    
                    if consecutive_failures >= max_consecutive_failures:
                        logger.error(f"💔 {consecutive_failures} consecutive heartbeat failures - may have been deregistered")
                        # Attempt to re-register
                        logger.info("🔄 Attempting to re-register due to heartbeat failures...")
                        if await self.register():
                            consecutive_failures = 0
                            logger.info("✅ Successfully re-registered after heartbeat failures")
                        else:
                            logger.error("❌ Re-registration failed - continuing to retry")
                
                # Wait for next heartbeat interval
                await asyncio.sleep(self.heartbeat_interval)
                
            except asyncio.CancelledError:
                logger.info("💔 Heartbeat loop cancelled")
                break
            except Exception as e:
                logger.error(f"💔 Unexpected error in heartbeat loop: {e}")
                await asyncio.sleep(5.0)  # Brief pause on error
    
    async def start(self) -> bool:
        """
        Start the brain visualizer client.
        
        Returns:
            True if startup successful, False otherwise
        """
        # Register with FEAGI
        if not await self.register():
            return False
        
        # Start heartbeat loop
        self.running = True
        self.heartbeat_task = asyncio.create_task(self.heartbeat_loop())
        
        logger.info(f"🚀 Brain visualizer client started successfully")
        return True
    
    async def stop(self) -> None:
        """
        Stop the brain visualizer client and cleanup resources.
        """
        logger.info("🛑 Stopping brain visualizer client...")
        
        self.running = False
        
        # Cancel heartbeat task
        if self.heartbeat_task:
            self.heartbeat_task.cancel()
            try:
                await self.heartbeat_task
            except asyncio.CancelledError:
                pass
        
        # Deregister from FEAGI
        if self.registered:
            await self.deregister()
        
        logger.info("✅ Brain visualizer client stopped successfully")
    
    async def deregister(self) -> bool:
        """
        Deregister from FEAGI.
        
        Returns:
            True if deregistration successful, False otherwise
        """
        if not self.registered:
            return True
        
        deregistration_data = {
            "agent_id": self.agent_id
        }
        
        try:
            logger.info(f"🔌 Deregistering agent '{self.agent_id}' from FEAGI...")
            
            response = requests.delete(
                f"{self.base_url}/v1/agent/deregister",
                json=deregistration_data,
                timeout=10.0
            )
            
            if response.status_code == 200:
                result = response.json()
                self.registered = False
                logger.info(f"✅ Successfully deregistered agent '{self.agent_id}': {result.get('message')}")
                return True
            else:
                logger.warning(f"⚠️ Deregistration warning: {response.status_code} - {response.text}")
                return False
                
        except Exception as e:
            logger.warning(f"⚠️ Deregistration error: {e}")
            return False
    
    def print_stats(self) -> None:
        """Print heartbeat statistics."""
        logger.info(f"📊 Heartbeat Stats - Total: {self.heartbeat_count}, "
                   f"Failures: {self.heartbeat_failures}, "
                   f"Last Success: {self.last_heartbeat_success}")


async def main():
    """Main function demonstrating robust brain visualizer client."""
    
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s'
    )
    
    # Create client
    client = RobustBrainVisualizerClient(
        feagi_host="localhost",
        feagi_port=8000,
        agent_id="brain_visualizer",
        heartbeat_interval=15.0
    )
    
    # Setup signal handlers for graceful shutdown
    shutdown_event = asyncio.Event()
    
    def signal_handler(signum, frame):
        logger.info(f"📡 Received signal {signum} - initiating graceful shutdown")
        shutdown_event.set()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Start the client
        if not await client.start():
            logger.error("❌ Failed to start brain visualizer client")
            return 1
        
        logger.info("🧠 Brain visualizer client running - press Ctrl+C to stop")
        
        # Main application loop - in a real client this would do visualization work
        while not shutdown_event.is_set():
            # Simulate visualization work
            await asyncio.sleep(1.0)
            
            # Periodically print stats
            if client.heartbeat_count > 0 and client.heartbeat_count % 10 == 0:
                client.print_stats()
        
        return 0
        
    except Exception as e:
        logger.error(f"💥 Unexpected error: {e}")
        return 1
        
    finally:
        # Cleanup
        await client.stop()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))




