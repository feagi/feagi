"""
Capabilities Management

Handles loading and managing agent capabilities from JSON configuration files.
"""

import json
import logging
from pathlib import Path
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)


class CapabilitiesManager:
    """Manage agent capabilities and device configurations."""
    
    def __init__(self, capabilities_file: str = "capabilities.json"):
        """Initialize capabilities manager."""
        self.capabilities_file = capabilities_file
        self.capabilities = {}
        self.has_input_capabilities = False
        self.has_output_capabilities = False
    
    def load_capabilities(self) -> bool:
        """Load agent capabilities from JSON file."""
        try:
            capabilities_path = Path(self.capabilities_file)
            if not capabilities_path.exists():
                logger.error(f"❌ Capabilities file not found: {self.capabilities_file}")
                return False
                
            with open(capabilities_path, 'r') as f:
                self.capabilities = json.load(f)
                
            # Analyze capabilities
            input_caps = self.capabilities.get("capabilities", {}).get("input", {})
            output_caps = self.capabilities.get("capabilities", {}).get("output", {})
            
            self.has_input_capabilities = bool(input_caps)
            self.has_output_capabilities = bool(output_caps)
            
            logger.info(f"📋 Loaded capabilities:")
            logger.info(f"   📥 Input devices: {list(input_caps.keys()) if input_caps else 'None'}")
            logger.info(f"   📤 Output devices: {list(output_caps.keys()) if output_caps else 'None'}")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Error loading capabilities: {e}")
            return False
    
    def get_input_capabilities(self) -> Dict[str, Any]:
        """Get input device capabilities."""
        return self.capabilities.get("capabilities", {}).get("input", {})
    
    def get_output_capabilities(self) -> Dict[str, Any]:
        """Get output device capabilities."""
        return self.capabilities.get("capabilities", {}).get("output", {})
    
    def map_cortical_id_to_device(self, cortical_id: str, output_caps: Dict) -> Optional[tuple]:
        """Map cortical area ID to device configuration."""
        for device_type, devices in output_caps.items():
            for device_id, config in devices.items():
                if not config.get("disabled", False):
                    # Match based on cortical area naming convention
                    expected_cortical = f"{device_type}_{config.get('feagi_index', device_id)}"
                    if cortical_id.lower().startswith(expected_cortical.lower()):
                        return device_type, device_id, config
        return None 