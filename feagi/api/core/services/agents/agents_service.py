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

"""Agents service for managing FEAGI agent operations."""

from typing import Any, Dict, List, Optional

from ..shared.base_service import BaseService


class AgentsService(BaseService):
    """
    Agents service handles agent registration, monitoring,
    and communication operations.
    """

    def get_connected_agents(self) -> List[Dict[str, Any]]:
        """Get list of currently connected agents."""
        try:
            if not self.state_manager:
                return []

            connected_agents = getattr(self.state_manager, "connected_agents", {})

            # Ensure connected_agents is a dictionary, not an integer or other type
            if not isinstance(connected_agents, dict):
                self.logger.warning(
                    f"connected_agents is not a dictionary, got type {type(connected_agents)}. Initializing as empty dict."
                )
                connected_agents = {}
                # Fix the state manager's connected_agents
                self.state_manager.connected_agents = {}

            # Convert to list format for API
            agent_list = []
            for agent_id, agent_info in connected_agents.items():
                # Ensure agent_info is also a dictionary
                if not isinstance(agent_info, dict):
                    self.logger.warning(
                        f"Agent info for {agent_id} is not a dictionary, got type {type(agent_info)}. Skipping."
                    )
                    continue

                agent_data = {
                    "agent_id": agent_id,
                    "agent_type": agent_info.get("type", "unknown"),
                    "status": agent_info.get("status", "unknown"),
                    "last_seen": agent_info.get("last_seen", None),
                    "capabilities": agent_info.get("capabilities", []),
                    "address": agent_info.get("address", "unknown"),
                }
                agent_list.append(agent_data)

            return agent_list
        except Exception as e:
            self.logger.error(f"Error getting connected agents: {str(e)}")
            return []

    def register_agent(self, agent_data: Dict[str, Any]) -> Dict[str, Any]:
        """Register a new agent."""
        try:
            if not self.state_manager:
                return {"success": False, "error": "State manager not available"}

            # Extract agent information
            agent_id = agent_data.get("agent_id")
            if not agent_id:
                return {"success": False, "error": "Agent ID required"}

            # Initialize connected_agents if it doesn't exist or is wrong type
            if not hasattr(self.state_manager, "connected_agents") or not isinstance(
                getattr(self.state_manager, "connected_agents", None), dict
            ):
                self.logger.warning("Initializing connected_agents as empty dictionary")
                self.state_manager.connected_agents = {}

            # Register the agent
            self.state_manager.connected_agents[agent_id] = {
                "type": agent_data.get("type", "unknown"),
                "status": "connected",
                "capabilities": agent_data.get("capabilities", []),
                "address": agent_data.get("address", "unknown"),
                "last_seen": agent_data.get("timestamp", None),
                "metadata": agent_data.get("metadata", {}),
            }

            self.logger.info(f"Agent {agent_id} registered successfully")

            return {
                "success": True,
                "agent_id": agent_id,
                "message": "Agent registered successfully",
            }
        except Exception as e:
            self.logger.error(f"Error registering agent: {str(e)}")
            return {"success": False, "error": str(e)}

    def unregister_agent(self, agent_id: str) -> Dict[str, Any]:
        """Unregister an agent."""
        try:
            if not self.state_manager:
                return {"success": False, "error": "State manager not available"}

            # Ensure connected_agents is a dictionary
            connected_agents = getattr(self.state_manager, "connected_agents", {})
            if not isinstance(connected_agents, dict):
                self.logger.warning(
                    f"connected_agents is not a dictionary in unregister, got type {type(connected_agents)}. Initializing as empty dict."
                )
                connected_agents = {}
                self.state_manager.connected_agents = {}

            if agent_id in connected_agents:
                del connected_agents[agent_id]
                self.logger.info(f"Agent {agent_id} unregistered successfully")
                return {"success": True, "message": "Agent unregistered successfully"}
            else:
                return {"success": False, "error": "Agent not found"}
        except Exception as e:
            self.logger.error(f"Error unregistering agent: {str(e)}")
            return {"success": False, "error": str(e)}

    def update_agent_status(
        self, agent_id: str, status: str, metadata: Dict[str, Any] = None
    ) -> bool:
        """Update agent status and metadata."""
        try:
            if not self.state_manager:
                return False

            # Ensure connected_agents is a dictionary
            connected_agents = getattr(self.state_manager, "connected_agents", {})
            if not isinstance(connected_agents, dict):
                self.logger.warning(
                    f"connected_agents is not a dictionary in update_agent_status, got type {type(connected_agents)}. Initializing as empty dict."
                )
                connected_agents = {}
                self.state_manager.connected_agents = {}
                return False  # Can't update if we had to reset

            if agent_id in connected_agents:
                # Ensure agent_info is also a dictionary
                if not isinstance(connected_agents[agent_id], dict):
                    self.logger.warning(
                        f"Agent info for {agent_id} is not a dictionary in update_agent_status, got type {type(connected_agents[agent_id])}. Skipping update."
                    )
                    return False

                connected_agents[agent_id]["status"] = status
                connected_agents[agent_id]["last_seen"] = (
                    metadata.get("timestamp") if metadata else None
                )

                if metadata:
                    if "metadata" not in connected_agents[agent_id]:
                        connected_agents[agent_id]["metadata"] = {}
                    connected_agents[agent_id]["metadata"].update(metadata)

                return True
            else:
                return False
        except Exception as e:
            self.logger.error(f"Error updating agent status: {str(e)}")
            return False

    def get_agent_details(self, agent_id: str) -> Optional[Dict[str, Any]]:
        """Get detailed information about a specific agent."""
        try:
            if not self.state_manager:
                return None

            # Ensure connected_agents is a dictionary
            connected_agents = getattr(self.state_manager, "connected_agents", {})
            if not isinstance(connected_agents, dict):
                self.logger.warning(
                    f"connected_agents is not a dictionary in get_agent_details, got type {type(connected_agents)}. Initializing as empty dict."
                )
                connected_agents = {}
                self.state_manager.connected_agents = {}
                return None

            if agent_id in connected_agents:
                agent_info = connected_agents[agent_id]
                # Ensure agent_info is also a dictionary
                if not isinstance(agent_info, dict):
                    self.logger.warning(
                        f"Agent info for {agent_id} is not a dictionary in get_agent_details, got type {type(agent_info)}. Returning None."
                    )
                    return None

                return {
                    "agent_id": agent_id,
                    "type": agent_info.get("type", "unknown"),
                    "status": agent_info.get("status", "unknown"),
                    "last_seen": agent_info.get("last_seen", None),
                    "capabilities": agent_info.get("capabilities", []),
                    "address": agent_info.get("address", "unknown"),
                    "metadata": agent_info.get("metadata", {}),
                }
            else:
                return None
        except Exception as e:
            self.logger.error(f"Error getting agent details: {str(e)}")
            return None

    def send_message_to_agent(
        self, agent_id: str, message: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Send a message to a specific agent."""
        try:
            # This is a placeholder implementation
            # In a real system, this would involve the actual messaging infrastructure

            if not self.state_manager:
                return {"success": False, "error": "State manager not available"}

            connected_agents = getattr(self.state_manager, "connected_agents", {})

            if agent_id not in connected_agents:
                return {"success": False, "error": "Agent not found"}

            # Log the message attempt
            self.logger.info(f"Sending message to agent {agent_id}: {message}")

            # In a real implementation, this would use the messaging system
            # For now, we just return success
            return {
                "success": True,
                "agent_id": agent_id,
                "message_id": f"msg_{agent_id}_{hash(str(message))}",
                "status": "sent",
            }
        except Exception as e:
            self.logger.error(f"Error sending message to agent: {str(e)}")
            return {"success": False, "error": str(e)}

    def broadcast_message(
        self, message: Dict[str, Any], agent_filter: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Broadcast a message to all connected agents or filtered subset."""
        try:
            if not self.state_manager:
                return {"success": False, "error": "State manager not available"}

            connected_agents = getattr(self.state_manager, "connected_agents", {})

            # Apply filter if provided
            target_agents = []
            for agent_id, agent_info in connected_agents.items():
                if agent_filter:
                    # Simple filtering based on agent type or status
                    if (
                        "type" in agent_filter
                        and agent_info.get("type") != agent_filter["type"]
                    ):
                        continue
                    if (
                        "status" in agent_filter
                        and agent_info.get("status") != agent_filter["status"]
                    ):
                        continue

                target_agents.append(agent_id)

            # Log the broadcast
            self.logger.info(
                f"Broadcasting message to {len(target_agents)} agents: {message}"
            )

            return {
                "success": True,
                "message_id": f"broadcast_{hash(str(message))}",
                "target_agents": target_agents,
                "status": "sent",
            }
        except Exception as e:
            self.logger.error(f"Error broadcasting message: {str(e)}")
            return {"success": False, "error": str(e)}

    def get_agent_statistics(self) -> Dict[str, Any]:
        """Get statistics about connected agents."""
        try:
            if not self.state_manager:
                return {}

            connected_agents = getattr(self.state_manager, "connected_agents", {})

            # Count agents by type and status
            type_counts = {}
            status_counts = {}

            for agent_info in connected_agents.values():
                agent_type = agent_info.get("type", "unknown")
                agent_status = agent_info.get("status", "unknown")

                type_counts[agent_type] = type_counts.get(agent_type, 0) + 1
                status_counts[agent_status] = status_counts.get(agent_status, 0) + 1

            return {
                "total_agents": len(connected_agents),
                "agents_by_type": type_counts,
                "agents_by_status": status_counts,
            }
        except Exception as e:
            self.logger.error(f"Error getting agent statistics: {str(e)}")
            return {}
