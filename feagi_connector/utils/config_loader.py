"""
FEAGI Agent Configuration Loader

Standard configuration loader for all FEAGI agents following FEAGI 2.0 architecture:
- Single source of truth (config.toml)
- No hardcoded defaults in agent code
- Fail-fast if configuration is missing or invalid
- CLI arguments are OVERRIDES only

Usage:
    from feagi_connector import load_agent_config
    
    # Load config.toml from agent's directory
    config = load_agent_config()
    
    # Or specify custom path
    config = load_agent_config("/path/to/custom_config.toml")
    
    # Access configuration
    feagi_host = config["feagi"]["host"]
    registration_port = config["feagi"]["registration_port"]
"""

import logging
import sys
from pathlib import Path
from typing import Optional, Dict, Any

try:
    import toml
except ImportError:
    print("ERROR: 'toml' package is required for configuration loading")
    print("Install it with: pip install toml")
    sys.exit(1)

logger = logging.getLogger(__name__)


def load_agent_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load agent configuration from TOML file (REQUIRED - no fallback)
    
    Following FEAGI 2.0 architecture principles:
    - Configuration file is the single source of truth
    - No hardcoded defaults in code
    - Fail-fast if config is missing or invalid
    - Deterministic behavior across all platforms
    
    Args:
        config_path: Optional path to config file. If None, looks for 
                    'config.toml' in the calling script's directory.
    
    Returns:
        Dict containing configuration structure
    
    Raises:
        SystemExit: If config file is missing or invalid
    
    Example config.toml structure:
        [feagi]
        host = "127.0.0.1"
        registration_port = 5563
        sensory_port = 5558
        
        [agent]
        id = "my-agent-1"
        log_level = "INFO"
        
        [capabilities]
        # Agent-specific capabilities...
    """
    # Determine config file path
    if config_path is None:
        # Look in caller's directory
        import inspect
        caller_frame = inspect.stack()[1]
        caller_file = Path(caller_frame.filename)
        config_path = caller_file.parent / "config.toml"
    else:
        config_path = Path(config_path)
    
    # Validate config file exists
    if not config_path.exists():
        logger.error(f"Configuration file not found: {config_path}")
        logger.error("")
        logger.error("FEAGI 2.0 Architecture Requirement:")
        logger.error("  - All agents must have a config.toml file")
        logger.error("  - No hardcoded defaults are allowed")
        logger.error("  - Configuration is the single source of truth")
        logger.error("")
        logger.error("To create a config file, copy the template:")
        logger.error("  cp agent_config.toml.template config.toml")
        sys.exit(1)
    
    # Load and parse TOML
    try:
        config = toml.load(config_path)
        logger.debug(f"✓ Loaded configuration from {config_path}")
        return config
    except Exception as e:
        logger.error(f"Failed to parse configuration file: {config_path}")
        logger.error(f"Error: {e}")
        logger.error("")
        logger.error("Check your TOML syntax:")
        logger.error("  - Ensure proper key = value format")
        logger.error("  - Check for missing quotes around strings")
        logger.error("  - Verify section headers like [feagi]")
        sys.exit(1)


def validate_feagi_config(config: Dict[str, Any]) -> bool:
    """
    Validate that configuration contains required FEAGI connection fields
    
    Args:
        config: Configuration dictionary
    
    Returns:
        True if valid
    
    Raises:
        ValueError: If required fields are missing
    """
    required_fields = {
        "feagi": ["host", "registration_port", "sensory_port"],
    }
    
    for section, fields in required_fields.items():
        if section not in config:
            raise ValueError(f"Missing required configuration section: [{section}]")
        
        for field in fields:
            if field not in config[section]:
                raise ValueError(f"Missing required field: {section}.{field}")
    
    return True


def get_config_template() -> str:
    """
    Return a standard FEAGI agent configuration template
    
    Returns:
        String containing template TOML configuration
    """
    return """# FEAGI Agent Configuration Template
#
# This is the single source of truth for agent configuration.
# Following FEAGI 2.0 architecture - NO hardcoded defaults in code!
#
# Copy this file to your agent directory as 'config.toml' and customize.

[feagi]
# FEAGI connection settings
host = "127.0.0.1"

# Rust PNS ports (FEAGI 2.0)
registration_port = 5563  # Agent registration/heartbeat (ZMQ REQ/REP)
sensory_port = 5558       # Sensory data input (ZMQ PUSH/PULL)

[agent]
# Agent identification
# id = "my-agent-1"  # Uncomment to specify agent ID (default: auto-generated)

# Logging configuration
log_level = "INFO"  # DEBUG, INFO, WARNING, ERROR

[capabilities]
# Define your agent's capabilities here
# This section is agent-specific and depends on your use case

# Example for vision agent:
# [capabilities.vision]
# cortical_area = "iic400"
# width = 64
# height = 64

# Example for motor agent:
# [capabilities.motor]
# dof = 6  # degrees of freedom
"""


# Convenience function for CLI override pattern
def merge_cli_args(config: Dict[str, Any], args: Any) -> Dict[str, Any]:
    """
    Merge CLI arguments into configuration (CLI overrides config.toml)
    
    Args:
        config: Base configuration from TOML
        args: Parsed argparse.Namespace with CLI arguments
    
    Returns:
        Updated configuration dictionary with CLI overrides applied
    
    Note:
        This modifies config in-place and returns it for convenience
    """
    # Common FEAGI connection overrides
    if hasattr(args, 'feagi_host') and args.feagi_host is not None:
        config.setdefault('feagi', {})['host'] = args.feagi_host
    
    if hasattr(args, 'registration_port') and args.registration_port is not None:
        config.setdefault('feagi', {})['registration_port'] = args.registration_port
    
    if hasattr(args, 'sensory_port') and args.sensory_port is not None:
        config.setdefault('feagi', {})['sensory_port'] = args.sensory_port
    
    # Agent ID override
    if hasattr(args, 'agent_id') and args.agent_id is not None:
        config.setdefault('agent', {})['id'] = args.agent_id
    
    # Log level override
    if hasattr(args, 'log_level') and args.log_level is not None:
        config.setdefault('agent', {})['log_level'] = args.log_level
    
    return config

