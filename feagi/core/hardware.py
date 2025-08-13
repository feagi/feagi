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
Hardware detection module for FEAGI.

This module handles detection of various hardware components including:
- CPU information and capabilities
- Memory statistics
- GPU detection (CUDA, Metal, WebGPU)
- Specialized accelerators (Apple Neural Engine, TPUs, etc.)
"""
import os
import platform

from feagi.utils.logger import setup_logger

logger = setup_logger()
import logging
from typing import Any, Dict, Tuple

logger = logging.getLogger("feagi.hardware")


def get_cpu_info() -> Dict[str, Any]:
    """
    Get detailed CPU information.

    Returns:
        Dictionary containing CPU information including:
        - count: Number of logical cores
        - physical_count: Number of physical cores (if available)
        - architecture: CPU architecture
        - model: CPU model name
        - features: Available CPU features (AVX, SSE, etc.)
    """
    info = {
        "count": os.cpu_count() or 1,
        "architecture": platform.machine(),
        "model": "Unknown",
        "features": [],
        "physical_count": None,
    }

    try:
        system = platform.system().lower()

        if system == "linux":
            try:
                # Try to get CPU model
                with open("/proc/cpuinfo", "r") as f:
                    cpuinfo = f.read()

                for line in cpuinfo.split("\n"):
                    if "model name" in line:
                        info["model"] = line.split(":")[1].strip()
                        break

                # Try to get CPU features
                for line in cpuinfo.split("\n"):
                    if "flags" in line or "Features" in line:
                        info["features"] = line.split(":")[1].strip().split()
                        break

                # Try to get physical core count
                physical_ids = set()
                for line in cpuinfo.split("\n"):
                    if "physical id" in line:
                        physical_ids.add(line.split(":")[1].strip())
                if physical_ids:
                    info["physical_count"] = len(physical_ids)
            except Exception as e:
                logger.warning(
                    f"Error getting detailed CPU info on Linux: {e}"
                )

        elif system == "darwin":
            try:
                import subprocess

                # Get CPU model
                result = subprocess.run(
                    ["sysctl", "-n", "machdep.cpu.brand_string"],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                info["model"] = result.stdout.strip()

                # Get CPU features
                result = subprocess.run(
                    ["sysctl", "-n", "machdep.cpu.features"],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                info["features"] = result.stdout.strip().split()

                # Get physical core count
                result = subprocess.run(
                    ["sysctl", "-n", "hw.physicalcpu"],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                info["physical_count"] = int(result.stdout.strip())
            except Exception as e:
                logger.warning(
                    f"Error getting detailed CPU info on macOS: {e}"
                )

        elif system == "windows":
            try:
                import subprocess

                # Try to get CPU model and physical core count using PowerShell
                result = subprocess.run(
                    [
                        "powershell",
                        "-Command",
                        "(Get-CimInstance -ClassName Win32_Processor).Name",
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                info["model"] = result.stdout.strip()

                result = subprocess.run(
                    [
                        "powershell",
                        "-Command",
                        "(Get-CimInstance -ClassName Win32_Processor).NumberOfCores",
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                info["physical_count"] = int(result.stdout.strip())
            except Exception as e:
                logger.warning(
                    f"Error getting detailed CPU info on Windows: {e}"
                )

    except Exception as e:
        logger.warning(f"Error getting detailed CPU info: {e}")

    # Check for SIMD support
    info["has_avx"] = "avx" in info["features"] or "AVX" in info["features"]
    info["has_avx2"] = "avx2" in info["features"] or "AVX2" in info["features"]
    info["has_sse4"] = any(
        f in info["features"]
        for f in ["sse4", "sse4.1", "sse4.2", "SSE4", "SSE4.1", "SSE4.2"]
    )

    return info


def get_memory_info() -> Dict[str, int]:
    """
    Get system memory information.

    Returns:
        Dictionary containing memory information in bytes:
        - total: Total physical memory
        - available: Available memory
        - used: Used memory
    """
    memory_info = {"total": 0, "available": 0, "used": 0}

    try:
        import psutil

        vm = psutil.virtual_memory()
        memory_info["total"] = vm.total
        memory_info["available"] = vm.available
        memory_info["used"] = vm.used
    except ImportError:
        logger.warning("psutil not available. Using limited memory detection.")
        try:
            if platform.system().lower() == "linux":
                with open("/proc/meminfo", "r") as f:
                    meminfo = f.read()

                for line in meminfo.split("\n"):
                    if "MemTotal" in line:
                        memory_info["total"] = int(line.split()[1]) * 1024
                    elif "MemAvailable" in line:
                        memory_info["available"] = int(line.split()[1]) * 1024

                memory_info["used"] = (
                    memory_info["total"] - memory_info["available"]
                )
        except Exception as e:
            logger.warning(f"Error getting memory info: {e}")

    return memory_info


def get_cuda_info() -> Dict[str, Any]:
    """
    Get CUDA GPU information if available.

    Returns:
        Dictionary containing CUDA information:
        - available: Whether CUDA is available
        - version: CUDA version
        - devices: List of CUDA devices with their properties
    """
    cuda_info = {"available": False, "version": None, "devices": []}

    try:
        import torch

        if torch.cuda.is_available():
            cuda_info["available"] = True
            cuda_info["version"] = torch.version.cuda

            for i in range(torch.cuda.device_count()):
                props = torch.cuda.get_device_properties(i)
                device_info = {
                    "id": i,
                    "name": props.name,
                    "total_memory": props.total_memory,
                    "compute_capability": f"{props.major}.{props.minor}",
                    "multi_processor_count": props.multi_processor_count,
                }
                cuda_info["devices"].append(device_info)
    except ImportError:
        logger.warning("PyTorch not available. CUDA detection skipped.")
    except Exception as e:
        logger.warning(f"Error detecting CUDA: {e}")

    return cuda_info


def get_metal_info() -> Dict[str, Any]:
    """
    Get Metal GPU information if available (macOS only).

    Returns:
        Dictionary containing Metal information:
        - available: Whether Metal is available
        - devices: List of Metal devices with their properties
    """
    metal_info = {"available": False, "devices": []}

    if platform.system().lower() != "darwin":
        return metal_info

    try:
        import subprocess

        # Check if Metal is available
        result = subprocess.run(
            ["system_profiler", "SPDisplaysDataType"],
            capture_output=True,
            text=True,
            check=True,
        )
        output = result.stdout

        if "Metal" in output:
            metal_info["available"] = True

            # Parse the output to extract GPU information
            current_device = None
            for line in output.split("\n"):
                line = line.strip()

                if "Chipset Model" in line:
                    if current_device:
                        metal_info["devices"].append(current_device)

                    current_device = {
                        "name": line.split(":")[1].strip(),
                        "memory": None,
                    }

                if current_device and "VRAM" in line:
                    memory_str = line.split(":")[1].strip()
                    if "MB" in memory_str:
                        current_device["memory"] = (
                            int(memory_str.split()[0]) * 1024 * 1024
                        )
                    elif "GB" in memory_str:
                        current_device["memory"] = (
                            int(memory_str.split()[0]) * 1024 * 1024 * 1024
                        )

            if current_device:
                metal_info["devices"].append(current_device)
    except Exception as e:
        logger.warning(f"Error detecting Metal: {e}")

    return metal_info


def get_specialized_accelerators() -> Dict[str, Any]:
    """
    Detect specialized hardware accelerators (Apple Neural Engine, TPUs, etc.).

    Returns:
        Dictionary containing information about specialized accelerators
    """
    accelerators = {
        "apple_neural_engine": {"available": False},
        "tpu": {"available": False},
    }

    # Detect Apple Neural Engine on Apple Silicon Macs
    if platform.system().lower() == "darwin":
        try:
            import subprocess

            result = subprocess.run(
                ["sysctl", "-n", "hw.optional.arm64"],
                capture_output=True,
                text=True,
            )

            # If running on Apple Silicon
            if result.returncode == 0 and result.stdout.strip() == "1":
                # Check for ANE
                result = subprocess.run(
                    ["system_profiler", "SPiBridgeDataType"],
                    capture_output=True,
                    text=True,
                )

                if "Apple Neural Engine" in result.stdout:
                    accelerators["apple_neural_engine"]["available"] = True
        except Exception as e:
            logger.warning(f"Error detecting Apple Neural Engine: {e}")

    # Detect TPUs
    try:
        import tensorflow as tf

        tpu_devices = tf.config.list_physical_devices("TPU")
        if tpu_devices:
            accelerators["tpu"]["available"] = True
            accelerators["tpu"]["count"] = len(tpu_devices)
    except ImportError:
        logger.debug("TensorFlow not available. TPU detection skipped.")
    except Exception as e:
        logger.warning(f"Error detecting TPUs: {e}")

    return accelerators


def get_webgpu_info() -> Dict[str, Any]:
    """
    Detect WebGPU availability and capabilities.

    Returns:
        Dictionary containing WebGPU information
    """
    webgpu_info = {"available": False}

    # Check for wgpu library (Python binding for WebGPU)
    try:
        import wgpu

        webgpu_info["available"] = True
        webgpu_info["backend"] = wgpu.BackendType.Undefined

        try:
            adapter = wgpu.request_adapter(
                power_preference=wgpu.PowerPreference.HighPerformance
            )
            if adapter:
                webgpu_info["adapter"] = {
                    "name": adapter.name,
                    "backend": adapter.backend,
                    "device_id": adapter.device_id,
                }

                # Get adapter features and limits
                features = []
                for feature in dir(wgpu.FeatureName):
                    if not feature.startswith("_") and adapter.has_feature(
                        getattr(wgpu.FeatureName, feature)
                    ):
                        features.append(feature)

                webgpu_info["features"] = features
        except Exception as e:
            logger.warning(f"Error getting WebGPU adapter info: {e}")
    except ImportError:
        logger.debug("wgpu library not available. WebGPU detection skipped.")
    except Exception as e:
        logger.warning(f"Error detecting WebGPU: {e}")

    return webgpu_info


def get_all_hardware_info() -> Dict[str, Any]:
    """
    Get comprehensive information about all available hardware.

    Returns:
        Dictionary containing all hardware information
    """
    return {
        "cpu": get_cpu_info(),
        "memory": get_memory_info(),
        "cuda": get_cuda_info(),
        "metal": get_metal_info(),
        "webgpu": get_webgpu_info(),
        "specialized_accelerators": get_specialized_accelerators(),
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "version": platform.version(),
            "machine": platform.machine(),
            "processor": platform.processor(),
        },
    }


def has_hardware_acceleration() -> Tuple[bool, str]:
    """
    Check if any hardware acceleration is available.

    Returns:
        Tuple of (has_acceleration, acceleration_type)
    """
    # Check for CUDA
    cuda_info = get_cuda_info()
    if cuda_info["available"]:
        return True, "cuda"

    # Check for Metal
    metal_info = get_metal_info()
    if metal_info["available"]:
        return True, "metal"

    # Check for WebGPU
    webgpu_info = get_webgpu_info()
    if webgpu_info["available"]:
        return True, "webgpu"

    # Check for specialized accelerators
    accelerators = get_specialized_accelerators()
    if accelerators["apple_neural_engine"]["available"]:
        return True, "apple_neural_engine"
    if accelerators["tpu"]["available"]:
        return True, "tpu"

    # Check for SIMD support
    cpu_info = get_cpu_info()
    if cpu_info.get("has_avx2"):
        return True, "avx2"
    if cpu_info.get("has_avx"):
        return True, "avx"
    if cpu_info.get("has_sse4"):
        return True, "sse4"

    return False, "none"


def get_optimal_backend() -> str:
    """
    Determine the optimal backend based on available hardware.

    Returns:
        String identifier for the optimal backend
    """
    has_accel, accel_type = has_hardware_acceleration()

    if accel_type == "cuda":
        return "cuda"
    elif accel_type == "metal":
        return "metal"
    elif accel_type == "webgpu":
        return "webgpu"
    elif accel_type == "apple_neural_engine":
        return "coreml"
    elif accel_type == "tpu":
        return "tpu"
    elif accel_type in ["avx2", "avx", "sse4"]:
        return "cpu_simd"
    else:
        return "cpu"
