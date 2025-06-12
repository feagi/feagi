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
Default configuration values for FEAGI.

This module defines default configuration values to ensure FEAGI
can operate with reasonable defaults when no specific configuration is provided.
"""

import multiprocessing as mp
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class DefaultConfig:
    """
    Default configuration dataclass to provide type hints and documentation.

    This class doesn't contain the actual values but defines the structure
    and documentation for configuration options.
    """

    # System configuration
    system: Dict[str, Any] = field(default_factory=dict)
    """System-level configuration options"""

    # Resource configuration
    resources: Dict[str, Any] = field(default_factory=dict)
    """Resource allocation and management configuration"""

    # Neural Processing Unit configuration
    npu: Dict[str, Any] = field(default_factory=dict)
    """Neural Processing Unit configuration"""

    # API configuration
    api: Dict[str, Any] = field(default_factory=dict)
    """API server configuration"""

    # ZMQ configuration
    zmq: Dict[str, Any] = field(default_factory=dict)
    """ZeroMQ communication configuration"""

    # Peripheral Nervous System configuration
    pns: Dict[str, Any] = field(default_factory=dict)
    """Peripheral Nervous System configuration"""

    # Brain Developmental Unit configuration
    bdu: Dict[str, Any] = field(default_factory=dict)
    """Brain Developmental Unit configuration"""

    # Evolutionary Unit configuration
    evo: Dict[str, Any] = field(default_factory=dict)
    """Evolutionary Unit configuration"""

    # Visualization configuration
    viz: Dict[str, Any] = field(default_factory=dict)
    """Visualization configuration"""

    # Logging configuration
    logging: Dict[str, Any] = field(default_factory=dict)
    """Logging configuration"""


# Default CPU cores calculation
def _get_default_cpu_cores() -> int:
    """Get default CPU core count for configuration"""
    try:
        return mp.cpu_count()
    except NotImplementedError:
        return 2  # Fallback to a reasonable default


# Detect if running in a container
def _is_containerized() -> bool:
    """Check if running in a container environment"""
    # Check for container indicators
    if os.path.exists("/.dockerenv"):
        return True

    try:
        with open("/proc/1/cgroup", "r") as f:
            if "docker" in f.read() or "kubepods" in f.read():
                return True
    except (IOError, FileNotFoundError):
        pass

    return False


# Default configuration values
DEFAULT_CONFIG = {
    "system": {
        "mode": "development",  # Options: 'development', 'production', 'test'
        "debug": True,
        "log_level": "INFO",
        "data_dir": os.environ.get(
            "FEAGI_DATA_DIR", os.path.expanduser("~/.feagi/data")
        ),
        "container_mode": _is_containerized(),
    },
    "resources": {
        "cpu_cores": _get_default_cpu_cores(),
        "use_gpu": True,  # Whether to use GPU if available
        "gpu_memory_fraction": 0.8,  # Fraction of GPU memory to use
        "process_management": {
            "enable_affinity": False,  # CPU affinity setting
            "monitor_interval": 5.0,  # Process monitoring interval in seconds
            "enable_health_check": True,  # Enable process health monitoring
        },
    },
    "npu": {
        "backend": "auto",  # Options: 'auto', 'cpu', 'gpu', 'webgpu', 'cuda', 'metal'
        "burst_engine": {
            "timestep": 0.1,  # ms
            "batch_size": 1000,  # Number of neurons to process in a batch
            "use_sparse_computation": True,  # Enable sparse computation optimization
        },
        "fcl_manager": {
            "default_window_size": 10,  # Default FCL window size
            "memory_window_size": 100,  # FCL window size for memory cortical areas
        },
        "learning": {
            "enable_plasticity": True,  # Enable learning mechanisms
            "stdp_time_window": 20,  # ms, STDP time window
        },
    },
    "api": {
        "enabled": True,
        "host": os.environ.get("FEAGI_API_HOST", "0.0.0.0"),
        "port": int(os.environ.get("FEAGI_API_PORT", "8000")),
        "workers": int(os.environ.get("FEAGI_API_WORKERS", "1")),
        "reload": os.environ.get("FEAGI_API_RELOAD", "0").lower()
        in ["1", "true", "yes"],
        "cors": {
            "enabled": True,
            "origins": ["*"],  # Default to allowing all origins in development
        },
        "security": {
            "enable_auth": False,  # Enable authentication
            "token_expiration": 86400,  # 24 hours in seconds
        },
    },
    "zmq": {
        "enabled": True,
        "host": os.environ.get("FEAGI_ZMQ_HOST", "0.0.0.0"),
        "pub_port": int(os.environ.get("FEAGI_ZMQ_PUB_PORT", "5557")),
        "sub_port": int(os.environ.get("FEAGI_ZMQ_SUB_PORT", "5558")),
        "topics": ["neural", "metrics", "heartbeat"],
        "polling_timeout": 100,  # ms
        "message_buffer_size": 100,  # Maximum messages to buffer
        # Stream-specific enable/disable configuration
        "streams": {
            "visualization": {
                "enabled": True,  # Enable visualization stream
                "auto_enable_on_subscribers": True,  # Auto-enable FQ sampler when clients connect
                "subscriber_check_interval": 1.0,  # Seconds between subscriber checks
            },
            "sensory": {
                "enabled": True,  # Enable sensory stream
            },
            "motor": {
                "enabled": True,  # Enable motor stream
            },
            "rest": {
                "enabled": True,  # Enable REST API stream (primary API interface)
            },
        },
    },
    "pns": {
        "enabled": True,
        "adapters": {
            "vision": {
                "enabled": True,
                "input_dimensions": [28, 28],  # Default for MNIST-like input
            },
            "motor": {
                "enabled": True,
                "output_dimensions": [10],  # Default for simple classification
            },
        },
    },
    "bdu": {
        "enabled": True,
        "stem_cell_manager": {
            "enabled": True,
            "development_interval": 100,  # Timesteps between development cycles
        },
    },
    "evo": {
        "enabled": False,  # Disabled by default
        "population_size": 100,
        "mutation_rate": 0.01,
        "crossover_rate": 0.7,
    },
    "viz": {
        "enabled": True,
        "update_interval": 50,  # ms between visualization updates
    },
    "logging": {
        "level": "INFO",
        "file": {
            "enabled": True,
            "path": os.environ.get("FEAGI_LOG_PATH", "feagi.log"),
            "max_size": 10 * 1024 * 1024,  # 10 MB
            "backup_count": 5,
        },
        "console": {
            "enabled": True,
            "format": "%(asctime)s - %(levelname)s - %(message)s",
        },
    },
}
