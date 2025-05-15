#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
FEAGI Visualization Protocol (FVP) implementation.

This module provides the implementation of the FEAGI Visualization Protocol, which
is used for neural activity visualization data between FEAGI and monitoring agents.
"""

import struct
import time
import numpy as np
from enum import IntEnum
from typing import Dict, Any, Optional, Tuple, ClassVar, List, Union

from feagi.utils.logger import setup_logger
from feagi.api.protocols.base import VersionedProtocol, ProtocolID, ProtocolRegistry

logger = setup_logger()


class FVPFrameType(IntEnum):
    """Frame types for FVP."""
    NEURON_ACTIVATIONS = 0x01
    CONNECTION_STRENGTHS = 0x02
    AREA_SUMMARY = 0x03
    GLOBAL_STATS = 0x04
    STRUCTURE_DATA = 0x05
    ERROR = 0xFF


class FVPMessageFormat:
    """
    FVP message format utilities.
    
    Format:
    +-------------+-------------+-------------+----------------+-------------+------------------+
    | Protocol ID | Version     | Frame       | Timestamp      | Data        | Data Payload     |
    | (1 byte)    | (1 byte)    | Type        | (8 bytes)      | Length      | (variable)       |
    |             |             | (1 byte)    |                | (4 bytes)   |                  |
    +-------------+-------------+-------------+----------------+-------------+------------------+
    """
    
    HEADER_FORMAT = "!BQI"  # Frame type (1 byte) + timestamp (8 bytes) + data length (4 bytes)
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    
    @staticmethod
    def pack_header(frame_type: FVPFrameType, timestamp_ms: int, payload_length: int) -> bytes:
        """Pack FVP header."""
        return struct.pack(FVPMessageFormat.HEADER_FORMAT, frame_type, timestamp_ms, payload_length)
    
    @staticmethod
    def unpack_header(data: bytes) -> Tuple[FVPFrameType, int, int]:
        """Unpack FVP header."""
        frame_type, timestamp_ms, payload_length = struct.unpack(FVPMessageFormat.HEADER_FORMAT, data)
        return FVPFrameType(frame_type), timestamp_ms, payload_length


class FVPv1(VersionedProtocol):
    """FEAGI Visualization Protocol version 1."""
    
    PROTOCOL_ID: ClassVar[ProtocolID] = ProtocolID.FVP
    VERSION: ClassVar[int] = 1
    
    @classmethod
    def encode(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode FVP data to binary format.
        
        Args:
            data: Dictionary containing:
                - frame_type: FVPFrameType
                - timestamp_ms: int (milliseconds since epoch)
                - payload: Dict or bytes containing frame-specific data
                
        Returns:
            Binary FVP data
        """
        frame_type = FVPFrameType(data.get("frame_type", FVPFrameType.ERROR))
        timestamp_ms = data.get("timestamp_ms", int(time.time() * 1000))
        payload = data.get("payload", {})
        
        # Convert payload to binary based on frame type
        if frame_type == FVPFrameType.NEURON_ACTIVATIONS:
            binary_payload = cls._encode_neuron_activations(payload)
        elif frame_type == FVPFrameType.CONNECTION_STRENGTHS:
            binary_payload = cls._encode_connection_strengths(payload)
        elif frame_type == FVPFrameType.AREA_SUMMARY:
            binary_payload = cls._encode_area_summary(payload)
        elif frame_type == FVPFrameType.GLOBAL_STATS:
            binary_payload = cls._encode_global_stats(payload)
        elif frame_type == FVPFrameType.STRUCTURE_DATA:
            binary_payload = cls._encode_structure_data(payload)
        else:
            raise ValueError(f"Unsupported frame type: {frame_type}")
        
        payload_length = len(binary_payload)
        
        # Pack header and payload
        header = FVPMessageFormat.pack_header(frame_type, timestamp_ms, payload_length)
        return header + binary_payload
    
    @classmethod
    def decode(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode binary FVP data.
        
        Args:
            data: Binary FVP data
                
        Returns:
            Dictionary containing:
                - frame_type: FVPFrameType
                - timestamp_ms: int (milliseconds since epoch)
                - payload: Dict containing frame-specific data
                
        Raises:
            ValueError: If the data format is invalid
        """
        if len(data) < FVPMessageFormat.HEADER_SIZE:
            raise ValueError("FVP data too short")
        
        # Unpack header
        header_data = data[:FVPMessageFormat.HEADER_SIZE]
        payload_data = data[FVPMessageFormat.HEADER_SIZE:]
        
        frame_type, timestamp_ms, payload_length = FVPMessageFormat.unpack_header(header_data)
        
        # Verify payload length
        if len(payload_data) != payload_length:
            raise ValueError(f"FVP payload length mismatch: expected {payload_length}, got {len(payload_data)}")
        
        # Decode payload based on frame type
        if frame_type == FVPFrameType.NEURON_ACTIVATIONS:
            payload = cls._decode_neuron_activations(payload_data)
        elif frame_type == FVPFrameType.CONNECTION_STRENGTHS:
            payload = cls._decode_connection_strengths(payload_data)
        elif frame_type == FVPFrameType.AREA_SUMMARY:
            payload = cls._decode_area_summary(payload_data)
        elif frame_type == FVPFrameType.GLOBAL_STATS:
            payload = cls._decode_global_stats(payload_data)
        elif frame_type == FVPFrameType.STRUCTURE_DATA:
            payload = cls._decode_structure_data(payload_data)
        else:
            raise ValueError(f"Unsupported frame type: {frame_type}")
        
        return {
            "frame_type": frame_type,
            "timestamp_ms": timestamp_ms,
            "payload": payload
        }
    
    @classmethod
    def _encode_neuron_activations(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode neuron activations data.
        
        Format:
        +-------------+-------------------+---------------+---------------+-------------------+
        | Cortical    | Number of         | Neuron        | Activation    | ... (repeating)   |
        | Area ID     | Neurons           | ID            | Value         |                   |
        | (4 bytes)   | (4 bytes)         | (4 bytes)     | (4 bytes)     |                   |
        +-------------+-------------------+---------------+---------------+-------------------+
        
        Args:
            data: Dictionary containing:
                - cortical_area_id: int
                - activations: Dict mapping neuron IDs to activation values
                
        Returns:
            Binary neuron activations data
        """
        cortical_area_id = data.get("cortical_area_id", 0)
        activations = data.get("activations", {})
        
        # Pack cortical area ID and number of neurons
        num_neurons = len(activations)
        result = struct.pack("!II", cortical_area_id, num_neurons)
        
        # Pack each neuron ID and activation value
        for neuron_id, activation in activations.items():
            result += struct.pack("!If", int(neuron_id), float(activation))
            
        return result
    
    @classmethod
    def _decode_neuron_activations(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode neuron activations data.
        
        Args:
            data: Binary neuron activations data
                
        Returns:
            Dictionary containing:
                - cortical_area_id: int
                - activations: Dict mapping neuron IDs to activation values
        """
        if len(data) < 8:  # At least cortical area ID and number of neurons
            raise ValueError("Neuron activations data too short")
            
        # Unpack cortical area ID and number of neurons
        cortical_area_id, num_neurons = struct.unpack("!II", data[:8])
        
        # Unpack activations
        activations = {}
        offset = 8
        for _ in range(num_neurons):
            if offset + 8 > len(data):
                raise ValueError("Neuron activations data truncated")
                
            neuron_id, activation = struct.unpack("!If", data[offset:offset+8])
            activations[str(neuron_id)] = activation
            offset += 8
            
        return {
            "cortical_area_id": cortical_area_id,
            "activations": activations
        }
    
    @classmethod
    def _encode_connection_strengths(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode connection strengths data.
        
        Format:
        +-------------+-------------------+-------------------+-------------------+-------------------+-------------------+
        | Source      | Target            | Number of         | Source            | Target            | Strength          |
        | Area ID     | Area ID           | Connections       | Neuron ID         | Neuron ID         | Value             |
        | (4 bytes)   | (4 bytes)         | (4 bytes)         | (4 bytes)         | (4 bytes)         | (4 bytes)         |
        +-------------+-------------------+-------------------+-------------------+-------------------+-------------------+
        
        Args:
            data: Dictionary containing:
                - source_area_id: int
                - target_area_id: int
                - connections: List of dicts with source_neuron_id, target_neuron_id, strength
                
        Returns:
            Binary connection strengths data
        """
        source_area_id = data.get("source_area_id", 0)
        target_area_id = data.get("target_area_id", 0)
        connections = data.get("connections", [])
        
        # Pack area IDs and number of connections
        num_connections = len(connections)
        result = struct.pack("!III", source_area_id, target_area_id, num_connections)
        
        # Pack each connection
        for conn in connections:
            source_neuron_id = conn.get("source_neuron_id", 0)
            target_neuron_id = conn.get("target_neuron_id", 0)
            strength = conn.get("strength", 0.0)
            result += struct.pack("!IIf", source_neuron_id, target_neuron_id, strength)
            
        return result
    
    @classmethod
    def _decode_connection_strengths(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode connection strengths data.
        
        Args:
            data: Binary connection strengths data
                
        Returns:
            Dictionary containing:
                - source_area_id: int
                - target_area_id: int
                - connections: List of dicts with source_neuron_id, target_neuron_id, strength
        """
        if len(data) < 12:  # At least source area ID, target area ID and number of connections
            raise ValueError("Connection strengths data too short")
            
        # Unpack area IDs and number of connections
        source_area_id, target_area_id, num_connections = struct.unpack("!III", data[:12])
        
        # Unpack connections
        connections = []
        offset = 12
        for _ in range(num_connections):
            if offset + 12 > len(data):
                raise ValueError("Connection strengths data truncated")
                
            source_neuron_id, target_neuron_id, strength = struct.unpack("!IIf", data[offset:offset+12])
            connections.append({
                "source_neuron_id": source_neuron_id,
                "target_neuron_id": target_neuron_id,
                "strength": strength
            })
            offset += 12
            
        return {
            "source_area_id": source_area_id,
            "target_area_id": target_area_id,
            "connections": connections
        }
    
    @classmethod
    def _encode_area_summary(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode cortical area summary data.
        
        Format:
        +-------------+-------------------+-------------------+-------------------+-------------------+
        | Cortical    | Number of         | Average           | Max               | Number of         |
        | Area ID     | Neurons           | Activity          | Activity          | Active Neurons    |
        | (4 bytes)   | (4 bytes)         | (4 bytes)         | (4 bytes)         | (4 bytes)         |
        +-------------+-------------------+-------------------+-------------------+-------------------+
        
        Args:
            data: Dictionary containing area summary info
                
        Returns:
            Binary area summary data
        """
        cortical_area_id = data.get("cortical_area_id", 0)
        num_neurons = data.get("num_neurons", 0)
        avg_activity = data.get("avg_activity", 0.0)
        max_activity = data.get("max_activity", 0.0)
        num_active = data.get("num_active", 0)
        
        return struct.pack("!IIffi", cortical_area_id, num_neurons, avg_activity, max_activity, num_active)
    
    @classmethod
    def _decode_area_summary(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode cortical area summary data.
        
        Args:
            data: Binary area summary data
                
        Returns:
            Dictionary containing area summary info
        """
        if len(data) < 20:  # Fixed size for area summary
            raise ValueError("Area summary data too short")
            
        cortical_area_id, num_neurons, avg_activity, max_activity, num_active = struct.unpack("!IIffi", data[:20])
            
        return {
            "cortical_area_id": cortical_area_id,
            "num_neurons": num_neurons,
            "avg_activity": avg_activity,
            "max_activity": max_activity,
            "num_active": num_active
        }
    
    @classmethod
    def _encode_global_stats(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode global brain statistics.
        
        Format:
        +-------------+-------------------+-------------------+-------------------+-------------------+
        | Total       | Total             | Total             | Current Burst     | Average Burst     |
        | Neurons     | Synapses          | Active Neurons    | Duration (ms)     | Frequency (Hz)    |
        | (4 bytes)   | (4 bytes)         | (4 bytes)         | (4 bytes)         | (4 bytes)         |
        +-------------+-------------------+-------------------+-------------------+-------------------+
        
        Args:
            data: Dictionary containing global statistics
                
        Returns:
            Binary global statistics data
        """
        total_neurons = data.get("total_neurons", 0)
        total_synapses = data.get("total_synapses", 0)
        total_active = data.get("total_active", 0)
        burst_duration = data.get("burst_duration", 0)
        burst_frequency = data.get("burst_frequency", 0.0)
        
        return struct.pack("!IIIIf", total_neurons, total_synapses, total_active, burst_duration, burst_frequency)
    
    @classmethod
    def _decode_global_stats(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode global brain statistics.
        
        Args:
            data: Binary global statistics data
                
        Returns:
            Dictionary containing global statistics
        """
        if len(data) < 20:  # Fixed size for global stats
            raise ValueError("Global stats data too short")
            
        total_neurons, total_synapses, total_active, burst_duration, burst_frequency = struct.unpack("!IIIIf", data[:20])
            
        return {
            "total_neurons": total_neurons,
            "total_synapses": total_synapses,
            "total_active": total_active,
            "burst_duration": burst_duration,
            "burst_frequency": burst_frequency
        }
    
    @classmethod
    def _encode_structure_data(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode brain structure data.
        
        Args:
            data: Dictionary containing structure data
                
        Returns:
            Binary structure data
        """
        # This is a placeholder for a more complex structure encoding
        # For now, we'll implement a simple version with basic properties
        area_count = len(data.get("areas", {}))
        connection_count = len(data.get("connections", []))
        
        # Pack counts
        result = struct.pack("!II", area_count, connection_count)
        
        # Pack each area (ID, name length, name, neuron count)
        for area_id, area_info in data.get("areas", {}).items():
            area_name = area_info.get("name", "").encode('utf-8')
            neuron_count = area_info.get("neuron_count", 0)
            result += struct.pack("!II", int(area_id), len(area_name))
            result += area_name
            result += struct.pack("!I", neuron_count)
            
        # Pack each connection (source area, target area, synapse count)
        for connection in data.get("connections", []):
            source_id = connection.get("source_id", 0)
            target_id = connection.get("target_id", 0)
            synapse_count = connection.get("synapse_count", 0)
            result += struct.pack("!III", source_id, target_id, synapse_count)
            
        return result
    
    @classmethod
    def _decode_structure_data(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode brain structure data.
        
        Args:
            data: Binary structure data
                
        Returns:
            Dictionary containing structure data
        """
        if len(data) < 8:  # At least area count and connection count
            raise ValueError("Structure data too short")
            
        # Unpack counts
        offset = 0
        area_count, connection_count = struct.unpack("!II", data[offset:offset+8])
        offset += 8
        
        # Unpack areas
        areas = {}
        for _ in range(area_count):
            if offset + 8 > len(data):
                raise ValueError("Structure data truncated")
                
            area_id, name_length = struct.unpack("!II", data[offset:offset+8])
            offset += 8
            
            if offset + name_length > len(data):
                raise ValueError("Structure data truncated")
                
            area_name = data[offset:offset+name_length].decode('utf-8')
            offset += name_length
            
            if offset + 4 > len(data):
                raise ValueError("Structure data truncated")
                
            neuron_count = struct.unpack("!I", data[offset:offset+4])[0]
            offset += 4
            
            areas[str(area_id)] = {
                "name": area_name,
                "neuron_count": neuron_count
            }
            
        # Unpack connections
        connections = []
        for _ in range(connection_count):
            if offset + 12 > len(data):
                raise ValueError("Structure data truncated")
                
            source_id, target_id, synapse_count = struct.unpack("!III", data[offset:offset+12])
            offset += 12
            
            connections.append({
                "source_id": source_id,
                "target_id": target_id,
                "synapse_count": synapse_count
            })
            
        return {
            "areas": areas,
            "connections": connections
        }


# Register the protocol with the registry
def register_protocols():
    """Register FVP protocol versions with the registry."""
    from feagi.api.protocols.base import ProtocolRegistry
    registry = ProtocolRegistry()
    registry.register(FVPv1)


# Helper functions for creating common FVP messages

def create_neuron_activations_message(cortical_area_id: int, activations: Dict[str, float]) -> Dict[str, Any]:
    """
    Create a neuron activations message.
    
    Args:
        cortical_area_id: ID of the cortical area
        activations: Dictionary mapping neuron IDs to activation values
        
    Returns:
        FVP message data
    """
    return {
        "frame_type": FVPFrameType.NEURON_ACTIVATIONS,
        "timestamp_ms": int(time.time() * 1000),
        "payload": {
            "cortical_area_id": cortical_area_id,
            "activations": activations
        }
    }


def create_global_stats_message(stats: Dict[str, Any]) -> Dict[str, Any]:
    """
    Create a global statistics message.
    
    Args:
        stats: Dictionary containing global brain statistics
        
    Returns:
        FVP message data
    """
    return {
        "frame_type": FVPFrameType.GLOBAL_STATS,
        "timestamp_ms": int(time.time() * 1000),
        "payload": stats
    }


def create_structure_data_message(areas: Dict[str, Dict[str, Any]], connections: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Create a brain structure data message.
    
    Args:
        areas: Dictionary mapping area IDs to area information
        connections: List of dictionaries containing connection information
        
    Returns:
        FVP message data
    """
    return {
        "frame_type": FVPFrameType.STRUCTURE_DATA,
        "timestamp_ms": int(time.time() * 1000),
        "payload": {
            "areas": areas,
            "connections": connections
        }
    } 