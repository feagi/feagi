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
Runtime SIMD Detection and Backend Selection for FEAGI NPU.

This module provides runtime detection of CPU SIMD capabilities and automatic
selection of optimal computational backends for neural processing operations.

@cursor:simd-detection
@cursor:critical-path
"""

import platform
import subprocess
import os
import logging
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)

class SIMDBackend(Enum):
    """Available SIMD computational backends."""
    SCALAR = "scalar"
    SSE = "sse"
    SSE2 = "sse2"
    AVX = "avx"
    AVX2 = "avx2"
    AVX512 = "avx512"
    NEON = "neon"
    SVE = "sve"
    GPU_CUDA = "gpu_cuda"
    GPU_WEBGPU = "gpu_webgpu"

@dataclass
class SIMDCapabilities:
    """Container for detected SIMD capabilities."""
    platform: str
    architecture: str
    
    # x86_64 features
    sse: bool = False
    sse2: bool = False
    sse3: bool = False
    ssse3: bool = False
    sse4_1: bool = False
    sse4_2: bool = False
    avx: bool = False
    avx2: bool = False
    avx512f: bool = False
    avx512dq: bool = False
    fma: bool = False
    
    # ARM features
    neon: bool = False
    asimd: bool = False
    sve: bool = False
    sve2: bool = False
    
    # GPU features
    cuda_available: bool = False
    webgpu_available: bool = False
    
    # Performance characteristics
    vector_width: int = 1
    preferred_dtype: str = "float32"
    cache_line_size: int = 64
    
    def __post_init__(self):
        """Calculate derived properties after initialization."""
        if self.avx512f:
            self.vector_width = 16  # 512 bits / 32 bits = 16 float32s
        elif self.avx2 or self.avx:
            self.vector_width = 8   # 256 bits / 32 bits = 8 float32s
        elif self.sse2 or self.sse:
            self.vector_width = 4   # 128 bits / 32 bits = 4 float32s
        elif self.neon or self.asimd:
            self.vector_width = 4   # 128 bits / 32 bits = 4 float32s
        elif self.sve:
            self.vector_width = 8   # Variable, but assume 256-bit for now
        else:
            self.vector_width = 1   # Scalar

class SIMDDetector:
    """Runtime SIMD capability detection."""
    
    def __init__(self):
        self.capabilities: Optional[SIMDCapabilities] = None
        self._cache_capabilities()
    
    def _cache_capabilities(self):
        """Cache detected capabilities to avoid repeated detection."""
        if self.capabilities is None:
            self.capabilities = self._detect_capabilities()
    
    def _detect_capabilities(self) -> SIMDCapabilities:
        """Detect available SIMD capabilities."""
        system = platform.system()
        machine = platform.machine()
        
        capabilities = SIMDCapabilities(
            platform=system,
            architecture=machine
        )
        
        # Detect x86_64 features
        if machine.lower() in ["x86_64", "amd64"]:
            self._detect_x86_features(capabilities)
        # Detect ARM features
        elif machine.lower() in ["arm64", "aarch64"]:
            self._detect_arm_features(capabilities)
        
        # Detect GPU capabilities
        self._detect_gpu_capabilities(capabilities)
        
        # Detect cache characteristics
        self._detect_cache_characteristics(capabilities)
        
        logger.info(f"[TARGET] SIMD Detection: {machine} with vector width {capabilities.vector_width}")
        
        return capabilities
    
    def _detect_x86_features(self, capabilities: SIMDCapabilities):
        """Detect x86_64 SIMD features."""
        
        # Try cpufeature library first
        try:
            import cpufeature
            capabilities.sse = getattr(cpufeature, 'CPUFeature')["SSE"]
            capabilities.sse2 = getattr(cpufeature, 'CPUFeature')["SSE2"]
            capabilities.sse3 = getattr(cpufeature, 'CPUFeature')["SSE3"]
            capabilities.ssse3 = getattr(cpufeature, 'CPUFeature')["SSSE3"]
            capabilities.sse4_1 = getattr(cpufeature, 'CPUFeature')["SSE4_1"]
            capabilities.sse4_2 = getattr(cpufeature, 'CPUFeature')["SSE4_2"]
            capabilities.avx = getattr(cpufeature, 'CPUFeature')["AVX"]
            capabilities.avx2 = getattr(cpufeature, 'CPUFeature')["AVX2"]
            capabilities.avx512f = getattr(cpufeature, 'CPUFeature')["AVX512F"]
            capabilities.fma = getattr(cpufeature, 'CPUFeature')["FMA"]
            return
        except (ImportError, KeyError, AttributeError):
            pass
        
        # Fallback to platform-specific detection
        if capabilities.platform == "Linux":
            self._detect_x86_linux(capabilities)
        elif capabilities.platform == "Darwin":
            self._detect_x86_macos(capabilities)
        elif capabilities.platform == "Windows":
            self._detect_x86_windows(capabilities)
        else:
            # Conservative fallback - assume SSE2 for x86_64
            capabilities.sse2 = True
    
    def _detect_x86_linux(self, capabilities: SIMDCapabilities):
        """Detect x86 features on Linux via /proc/cpuinfo."""
        try:
            with open("/proc/cpuinfo", "r") as f:
                cpuinfo = f.read().lower()
                
            capabilities.sse = "sse" in cpuinfo
            capabilities.sse2 = "sse2" in cpuinfo
            capabilities.sse3 = "sse3" in cpuinfo or "pni" in cpuinfo
            capabilities.ssse3 = "ssse3" in cpuinfo
            capabilities.sse4_1 = "sse4_1" in cpuinfo
            capabilities.sse4_2 = "sse4_2" in cpuinfo
            capabilities.avx = "avx" in cpuinfo
            capabilities.avx2 = "avx2" in cpuinfo
            capabilities.avx512f = "avx512f" in cpuinfo
            capabilities.fma = "fma" in cpuinfo
            
        except Exception as e:
            logger.warning(f"Failed to read /proc/cpuinfo: {e}")
            capabilities.sse2 = True  # Conservative fallback
    
    def _detect_x86_macos(self, capabilities: SIMDCapabilities):
        """Detect x86 features on macOS via sysctl."""
        try:
            result = subprocess.run(
                ["sysctl", "-a"], capture_output=True, text=True, timeout=5
            )
            sysctl_output = result.stdout.lower()
            
            # macOS sysctl uses different naming
            capabilities.sse = "sse" in sysctl_output
            capabilities.sse2 = "sse2" in sysctl_output
            capabilities.sse3 = "sse3" in sysctl_output
            capabilities.ssse3 = "ssse3" in sysctl_output
            capabilities.sse4_1 = "sse4.1" in sysctl_output
            capabilities.sse4_2 = "sse4.2" in sysctl_output
            capabilities.avx = "avx1.0" in sysctl_output or "avx" in sysctl_output
            capabilities.avx2 = "avx2.0" in sysctl_output or "avx2" in sysctl_output
            capabilities.avx512f = "avx512f" in sysctl_output
            capabilities.fma = "fma" in sysctl_output
            
        except Exception as e:
            logger.warning(f"Failed to run sysctl: {e}")
            capabilities.sse2 = True  # Conservative fallback
    
    def _detect_x86_windows(self, capabilities: SIMDCapabilities):
        """Detect x86 features on Windows via wmic."""
        try:
            result = subprocess.run(
                ["wmic", "cpu", "get", "Name"], capture_output=True, text=True, timeout=5
            )
            cpu_info = result.stdout.lower()
            
            # Basic detection for Windows - could be enhanced
            capabilities.sse2 = True  # x86_64 always has SSE2
            capabilities.avx = "avx" in cpu_info
            capabilities.avx2 = "avx2" in cpu_info
            
        except Exception as e:
            logger.warning(f"Failed to run wmic: {e}")
            capabilities.sse2 = True  # Conservative fallback
    
    def _detect_arm_features(self, capabilities: SIMDCapabilities):
        """Detect ARM SIMD features."""
        
        # ARM64 always has NEON/ASIMD
        if capabilities.architecture.lower() in ["arm64", "aarch64"]:
            capabilities.neon = True
            capabilities.asimd = True
        
        # Try to detect SVE
        if capabilities.platform == "Linux":
            try:
                with open("/proc/cpuinfo", "r") as f:
                    cpuinfo = f.read().lower()
                
                capabilities.sve = "sve" in cpuinfo
                capabilities.sve2 = "sve2" in cpuinfo
                
            except Exception:
                pass
        
        elif capabilities.platform == "Darwin":
            # macOS ARM (Apple Silicon) always has NEON/ASIMD
            capabilities.neon = True
            capabilities.asimd = True
    
    def _detect_gpu_capabilities(self, capabilities: SIMDCapabilities):
        """Detect GPU acceleration capabilities."""
        
        # Check for CUDA
        try:
            import torch
            capabilities.cuda_available = torch.cuda.is_available()
        except ImportError:
            pass
        
        # Check for WebGPU
        try:
            # This would need actual WebGPU detection
            # For now, assume it's available if we're not on a server
            capabilities.webgpu_available = not self._is_headless_server()
        except Exception:
            pass
    
    def _detect_cache_characteristics(self, capabilities: SIMDCapabilities):
        """Detect CPU cache characteristics."""
        
        # Try to get cache line size
        if capabilities.platform == "Linux":
            try:
                with open("/sys/devices/system/cpu/cpu0/cache/index0/coherency_line_size", "r") as f:
                    capabilities.cache_line_size = int(f.read().strip())
            except Exception:
                pass
        
        # Platform-specific defaults
        if capabilities.architecture.lower() in ["x86_64", "amd64"]:
            capabilities.cache_line_size = 64  # Intel/AMD default
        elif capabilities.architecture.lower() in ["arm64", "aarch64"]:
            capabilities.cache_line_size = 64  # ARM64 default
    
    def _is_headless_server(self) -> bool:
        """Check if running on a headless server."""
        return (
            os.environ.get("DISPLAY") is None and 
            os.environ.get("SSH_CONNECTION") is not None
        )
    
    def get_optimal_backend(self, operation_type: str = "general") -> SIMDBackend:
        """Select optimal SIMD backend for given operation type."""
        caps = self.capabilities
        
        # GPU first if available
        if caps.cuda_available:
            return SIMDBackend.GPU_CUDA
        elif caps.webgpu_available and operation_type in ["membrane_update", "large_matrix"]:
            return SIMDBackend.GPU_WEBGPU
        
        # CPU SIMD selection
        if caps.avx512f:
            return SIMDBackend.AVX512
        elif caps.avx2:
            return SIMDBackend.AVX2
        elif caps.avx:
            return SIMDBackend.AVX
        elif caps.sve:
            return SIMDBackend.SVE
        elif caps.neon or caps.asimd:
            return SIMDBackend.NEON
        elif caps.sse2:
            return SIMDBackend.SSE2
        else:
            return SIMDBackend.SCALAR
    
    def get_aligned_size(self, size: int) -> int:
        """Get size aligned to SIMD vector boundaries."""
        vector_width = self.capabilities.vector_width
        return (size + vector_width - 1) & ~(vector_width - 1)
    
    def get_memory_alignment(self) -> int:
        """Get optimal memory alignment for SIMD operations."""
        caps = self.capabilities
        
        if caps.avx512f:
            return 64  # 512-bit alignment
        elif caps.avx2 or caps.avx:
            return 32  # 256-bit alignment
        elif caps.sse2 or caps.neon:
            return 16  # 128-bit alignment
        else:
            return 8   # Basic alignment

class SIMDBackendSelector:
    """Selects optimal SIMD backend configuration for NPU operations."""
    
    def __init__(self):
        self.detector = SIMDDetector()
        self.backend_config = self._create_backend_config()
    
    def _create_backend_config(self) -> Dict[str, Any]:
        """Create backend configuration based on detected capabilities."""
        caps = self.detector.capabilities
        backend = self.detector.get_optimal_backend()
        
        config = {
            "backend": backend,
            "vector_width": caps.vector_width,
            "alignment": self.detector.get_memory_alignment(),
            "use_fma": caps.fma,
            "cache_line_size": caps.cache_line_size,
            "parallel_threshold": self._get_parallel_threshold()
        }
        
        # Backend-specific optimizations
        if backend == SIMDBackend.AVX512:
            config.update({
                "chunk_size": 16,
                "unroll_factor": 4,
                "prefetch_distance": 64
            })
        elif backend == SIMDBackend.AVX2:
            config.update({
                "chunk_size": 8,
                "unroll_factor": 4,
                "prefetch_distance": 32
            })
        elif backend == SIMDBackend.NEON:
            config.update({
                "chunk_size": 4,
                "unroll_factor": 2,
                "prefetch_distance": 16
            })
        
        return config
    
    def _get_parallel_threshold(self) -> int:
        """Get threshold above which parallel processing is beneficial."""
        caps = self.detector.capabilities
        
        # Larger thresholds for more powerful SIMD
        if caps.avx512f:
            return 1024
        elif caps.avx2:
            return 512
        elif caps.neon or caps.sse2:
            return 256
        else:
            return 128
    
    def should_use_parallel(self, operation_size: int) -> bool:
        """Determine if operation should use parallel processing."""
        return operation_size >= self.backend_config["parallel_threshold"]
    
    def get_chunk_size(self, total_size: int) -> int:
        """Get optimal chunk size for processing."""
        base_chunk = self.backend_config["chunk_size"]
        
        # Adjust chunk size based on total size
        if total_size < base_chunk * 4:
            return total_size // 4 if total_size > 4 else 1
        else:
            return base_chunk

# Global instance for easy access
_simd_detector = None
_backend_selector = None

def get_simd_detector() -> SIMDDetector:
    """Get global SIMD detector instance."""
    global _simd_detector
    if _simd_detector is None:
        _simd_detector = SIMDDetector()
    return _simd_detector

def get_backend_selector() -> SIMDBackendSelector:
    """Get global backend selector instance."""
    global _backend_selector
    if _backend_selector is None:
        _backend_selector = SIMDBackendSelector()
    return _backend_selector

def detect_optimal_backend(operation_type: str = "general") -> SIMDBackend:
    """Convenience function to detect optimal backend."""
    return get_simd_detector().get_optimal_backend(operation_type)

def get_simd_config() -> Dict[str, Any]:
    """Get complete SIMD configuration for NPU."""
    selector = get_backend_selector()
    detector = get_simd_detector()
    
    return {
        "capabilities": detector.capabilities.__dict__,
        "backend_config": selector.backend_config,
        "recommended_backend": selector.backend_config["backend"].value
    } 