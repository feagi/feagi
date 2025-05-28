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
ZMQ Traffic Debugging Utilities

Provides logging functions for debugging ZMQ inbound and outbound traffic
with data decoding capabilities.
"""

import os
import json
import time
from typing import Any, List, Optional, Union
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

# Check if debugging is enabled via environment variables
DEBUG_ZMQ_OUTBOUND = os.environ.get('FEAGI_DEBUG_ZMQ_OUTBOUND', '').lower() in ('1', 'true', 'yes')
DEBUG_ZMQ_INBOUND = os.environ.get('FEAGI_DEBUG_ZMQ_INBOUND', '').lower() in ('1', 'true', 'yes')

def decode_zmq_data(data: bytes, max_preview: int = 200) -> str:
    """
    Decode ZMQ byte data for human-readable logging.
    
    Args:
        data: Raw byte data
        max_preview: Maximum number of characters to show in preview (DISABLED - shows full data)
        
    Returns:
        Human-readable string representation of the data
    """
    if not data:
        return "<empty>"
    
    # Try to decode as UTF-8 first
    try:
        decoded = data.decode('utf-8')
        
        # Try to parse as JSON for pretty printing
        try:
            json_data = json.loads(decoded)
            pretty_json = json.dumps(json_data, indent=2)
            # REMOVED TRUNCATION - always show full data
            return f"JSON: {pretty_json}"
        except (json.JSONDecodeError, TypeError):
            # Not JSON, return as plain text
            # REMOVED TRUNCATION - always show full data
            return f"TEXT: {decoded}"
                
    except UnicodeDecodeError:
        # Binary data - show hex dump
        hex_data = data.hex()
        # REMOVED TRUNCATION - always show full hex data
        return f"BINARY: {hex_data} (total: {len(data)} bytes)"

def log_zmq_outbound(endpoint: str, topic: Union[str, bytes], data: bytes, 
                    context: str = "", message_type: str = "unknown") -> None:
    """
    Log outbound ZMQ traffic if debugging is enabled.
    
    Args:
        endpoint: ZMQ endpoint (e.g., "tcp://localhost:5562")
        topic: ZMQ topic (for PUB/SUB)
        data: Raw byte data being sent
        context: Additional context information
        message_type: Type of message (e.g., "activity", "structure", "control")
    """
    if not DEBUG_ZMQ_OUTBOUND:
        return
    
    topic_str = topic.decode('utf-8') if isinstance(topic, bytes) else str(topic)
    decoded_data = decode_zmq_data(data)
    timestamp = time.strftime("%H:%M:%S.%f")[:-3]  # Include milliseconds
    
    logger.info(f"📤 ZMQ OUTBOUND [{timestamp}]")
    logger.info(f"   [TARGET] Endpoint: {endpoint}")
    logger.info(f"   [TAG]  Topic: '{topic_str}'")
    logger.info(f"   📦 Type: {message_type}")
    logger.info(f"   [STATS] Size: {len(data)} bytes")
    if context:
        logger.info(f"   [SEARCH] Context: {context}")
    logger.info(f"   📄 Data: {decoded_data}")
    logger.info("   " + "─" * 50)

def log_zmq_inbound(endpoint: str, frames: List[bytes], 
                   context: str = "", message_type: str = "unknown") -> None:
    """
    Log inbound ZMQ traffic if debugging is enabled.
    
    Args:
        endpoint: ZMQ endpoint (e.g., "tcp://localhost:5563")
        frames: List of ZMQ frames received
        context: Additional context information  
        message_type: Type of message (e.g., "request", "sensory", "control")
    """
    if not DEBUG_ZMQ_INBOUND:
        return
        
    timestamp = time.strftime("%H:%M:%S.%f")[:-3]  # Include milliseconds
    total_size = sum(len(frame) for frame in frames)
    
    logger.info(f"📥 ZMQ INBOUND [{timestamp}]")
    logger.info(f"   [TARGET] Endpoint: {endpoint}")
    logger.info(f"   📦 Type: {message_type}")
    logger.info(f"   [STATS] Frames: {len(frames)}, Total size: {total_size} bytes")
    if context:
        logger.info(f"   [SEARCH] Context: {context}")
    
    # Log each frame
    for i, frame in enumerate(frames):
        decoded_frame = decode_zmq_data(frame)
        logger.info(f"   📄 Frame {i}: {decoded_frame}")
    
    logger.info("   " + "─" * 50)

def log_zmq_multipart_outbound(endpoint: str, multipart_data: List[bytes],
                              context: str = "", message_type: str = "unknown") -> None:
    """
    Log outbound multipart ZMQ message if debugging is enabled.
    
    Args:
        endpoint: ZMQ endpoint
        multipart_data: List of byte frames being sent
        context: Additional context information
        message_type: Type of message
    """
    if not DEBUG_ZMQ_OUTBOUND:
        return
    
    # Extract topic if this looks like a PUB/SUB message
    topic_str = "N/A"
    data_frames = multipart_data
    
    if len(multipart_data) >= 2:
        # Assume first frame is topic for PUB/SUB
        try:
            topic_str = multipart_data[0].decode('utf-8')
            data_frames = multipart_data[1:]
        except UnicodeDecodeError:
            topic_str = f"<binary: {multipart_data[0][:20].hex()}>"
    
    timestamp = time.strftime("%H:%M:%S.%f")[:-3]
    total_size = sum(len(frame) for frame in multipart_data)
    
    logger.info(f"📤 ZMQ MULTIPART OUTBOUND [{timestamp}]")
    logger.info(f"   [TARGET] Endpoint: {endpoint}")
    logger.info(f"   [TAG]  Topic: '{topic_str}'")
    logger.info(f"   📦 Type: {message_type}")
    logger.info(f"   [STATS] Frames: {len(multipart_data)}, Total size: {total_size} bytes")
    if context:
        logger.info(f"   [SEARCH] Context: {context}")
    
    # Log each frame
    for i, frame in enumerate(multipart_data):
        frame_label = "Topic" if i == 0 and len(multipart_data) >= 2 else f"Data {i}"
        decoded_frame = decode_zmq_data(frame)
        logger.info(f"   📄 {frame_label}: {decoded_frame}")
    
    logger.info("   " + "─" * 50)

# Convenience functions for common ZMQ patterns
def log_pub_message(endpoint: str, topic: Union[str, bytes], data: bytes, context: str = "") -> None:
    """Log a PUB/SUB outbound message."""
    log_zmq_outbound(endpoint, topic, data, context, "PUB/SUB")

def log_req_message(endpoint: str, data: bytes, context: str = "") -> None:
    """Log a REQ/REP outbound request."""
    log_zmq_outbound(endpoint, "", data, context, "REQ")

def log_rep_message(endpoint: str, data: bytes, context: str = "") -> None:
    """Log a REQ/REP outbound reply."""
    log_zmq_outbound(endpoint, "", data, context, "REP")

def log_push_message(endpoint: str, data: bytes, context: str = "") -> None:
    """Log a PUSH/PULL outbound message."""
    log_zmq_outbound(endpoint, "", data, context, "PUSH")

def log_sub_message(endpoint: str, frames: List[bytes], context: str = "") -> None:
    """Log a PUB/SUB inbound message."""
    log_zmq_inbound(endpoint, frames, context, "SUB")

def log_pull_message(endpoint: str, frames: List[bytes], context: str = "") -> None:
    """Log a PUSH/PULL inbound message."""
    log_zmq_inbound(endpoint, frames, context, "PULL")

def log_req_received(endpoint: str, frames: List[bytes], context: str = "") -> None:
    """Log a REQ/REP inbound request."""
    log_zmq_inbound(endpoint, frames, context, "REQ received")

def log_rep_received(endpoint: str, frames: List[bytes], context: str = "") -> None:
    """Log a REQ/REP inbound reply."""
    log_zmq_inbound(endpoint, frames, context, "REP received") 