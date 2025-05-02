#!/usr/bin/env python3
"""
Shared utilities for FEAGI performance tests.
"""

import os
import sys
import platform
import logging
from typing import Dict, List, Optional, Any, Tuple

logger = logging.getLogger(__name__)

def is_webgpu_available() -> bool:
    """
    Check if WebGPU backend is available on this system.
    
    Returns:
        True if WebGPU is available, False otherwise
    """
    try:
        from feagi.core.backends.webgpu.webgpu_backend import WebGPUBackend
        backend = WebGPUBackend()
        return backend.is_available()
    except (ImportError, Exception) as e:
        logger.debug(f"WebGPU unavailable: {str(e)}")
        return False

def is_cuda_available() -> bool:
    """
    Check if CUDA backend is available on this system.
    
    Returns:
        True if CUDA is available, False otherwise
    """
    try:
        from feagi.core.backends.cuda.cuda_backend import CUDABackend
        backend = CUDABackend()
        return backend.is_available()
    except (ImportError, Exception) as e:
        logger.debug(f"CUDA unavailable: {str(e)}")
        return False

def get_available_backends() -> List[str]:
    """
    Get a list of all available backends on this system.
    
    Returns:
        List of available backend names
    """
    backends = ["cpu"]  # CPU always available
    
    if is_webgpu_available():
        backends.append("webgpu")
        
    if is_cuda_available():
        backends.append("cuda")
        
    return backends

def get_system_info() -> Dict[str, Any]:
    """
    Get detailed information about the current system.
    
    Returns:
        Dictionary containing system information
    """
    import platform
    import os
    
    info = {
        "os": platform.system(),
        "os_version": platform.version(),
        "os_release": platform.release(),
        "python_version": platform.python_version(),
        "cpu": platform.processor(),
        "cpu_count": os.cpu_count(),
        "python_implementation": platform.python_implementation(),
        "available_backends": get_available_backends()
    }
    
    # Try to get GPU information if available
    try:
        import GPUtil
        gpus = GPUtil.getGPUs()
        if gpus:
            gpu_info = []
            for gpu in gpus:
                gpu_info.append({
                    "name": gpu.name,
                    "memory_total": gpu.memoryTotal,
                    "driver": gpu.driver,
                })
            info["gpus"] = gpu_info
    except (ImportError, Exception) as e:
        logger.debug(f"Could not get GPU info: {str(e)}")
        info["gpus"] = "Unknown"
    
    return info 