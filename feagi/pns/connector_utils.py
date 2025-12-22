"""
Connector Agent Utilities - JSON Import/Export

Provides helper functions for loading and saving agent capabilities in the new JSON format.
All parsing and validation is done in Rust - Python just passes strings.
"""

import json
import logging
from pathlib import Path
from typing import Union

logger = logging.getLogger(__name__)

try:
    from feagi_rust_py_libs import connector_core
    PyConnectorAgent = connector_core.ConnectorAgent
    RUST_AVAILABLE = True
except ImportError:
    PyConnectorAgent = None
    RUST_AVAILABLE = False
    logger.warning("feagi_rust_py_libs not available - connector utilities disabled")


def load_capabilities_from_file(agent: 'PyConnectorAgent', filepath: Union[str, Path]) -> None:
    """
    Load capabilities JSON into agent (Rust does ALL parsing/validation)
    
    Devices must be registered first using the appropriate register functions
    (e.g., agent.sensor_simple_vision_register()). This function only updates
    pipeline stages and friendly names for already-registered devices.
    
    Args:
        agent: PyConnectorAgent instance with devices already registered
        filepath: Path to capabilities.json file in new format
        
    Raises:
        FileNotFoundError: If file doesn't exist
        FeagiError: If JSON is malformed or references unregistered devices
        
    Example:
        >>> agent = PyConnectorAgent()
        >>> agent.sensor_simple_vision_register(group=0, number_channels=1, ...)
        >>> load_capabilities_from_file(agent, "capabilities.json")
    """
    if not RUST_AVAILABLE:
        raise RuntimeError("feagi_rust_py_libs not available")
    
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"Capabilities file not found: {filepath}")
    
    logger.info(f"📂 Loading capabilities from: {filepath}")
    
    with open(filepath, 'r') as f:
        json_str = f.read()
    
    # Single source of truth: Rust parses and validates
    try:
        agent.import_capabilities_json(json_str)
        logger.info("✅ Capabilities loaded successfully")
    except Exception as e:
        logger.error(f"❌ Failed to load capabilities: {e}")
        
        # Check if it's an old format
        try:
            data = json.loads(json_str)
            if _is_old_format(data):
                logger.error(
                    "\n⚠️  OLD FORMAT DETECTED!\n"
                    "   This appears to be an old capabilities.json format.\n"
                    "   Please use the migration script:\n\n"
                    "   python feagi-core/scripts/migrate_capabilities_json.py \\\n"
                    f"     --input {filepath} \\\n"
                    f"     --output {filepath.parent / 'capabilities_new.json'}\n"
                )
        except:
            pass
        
        raise


def export_capabilities_to_file(agent: 'PyConnectorAgent', filepath: Union[str, Path]) -> None:
    """
    Export current agent capabilities to JSON file
    
    Exports all registered devices with their current pipeline configurations
    and friendly names in the new JSON format.
    
    Args:
        agent: PyConnectorAgent instance
        filepath: Path where to save capabilities.json
        
    Example:
        >>> agent = PyConnectorAgent()
        >>> # ... register and configure devices ...
        >>> export_capabilities_to_file(agent, "capabilities.json")
    """
    if not RUST_AVAILABLE:
        raise RuntimeError("feagi_rust_py_libs not available")
    
    filepath = Path(filepath)
    logger.info(f"💾 Exporting capabilities to: {filepath}")
    
    # Rust generates the JSON
    json_str = agent.export_capabilities_json()
    
    # Write to file
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with open(filepath, 'w') as f:
        f.write(json_str)
    
    logger.info("✅ Capabilities exported successfully")


def _is_old_format(data: dict) -> bool:
    """
    Detect if JSON is in old format (heuristic check)
    
    Old format has:
    - capabilities.input.<type>.<group>.feagi_index
    - capabilities.input.<type>.<group>.custom_name
    - No "channels" array
    
    New format has:
    - capabilities.input.<type>.<group>.friendly_name
    - capabilities.input.<type>.<group>.channels (array)
    """
    try:
        caps = data.get('capabilities', {})
        input_data = caps.get('input', {})
        
        for sensor_type, groups in input_data.items():
            for group_id, config in groups.items():
                # Old format indicators
                if 'feagi_index' in config:
                    return True
                if 'custom_name' in config and 'channels' not in config:
                    return True
                    
        return False
    except:
        return False

