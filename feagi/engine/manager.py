"""
FEAGI Engine Manager

Manages the lifecycle of a FEAGI engine instance.
"""

import subprocess
import time
import logging
import os
from pathlib import Path
from typing import Optional

logger = logging.getLogger("feagi.engine")


class FeagiEngine:
    """
    FEAGI Engine Manager
    
    Controls the FEAGI neural engine lifecycle from Python.
    
    Example:
        # Start with a fresh genome
        engine = FeagiEngine()
        engine.load_config("feagi_configuration.toml")
        engine.load_genome("my_genome.json")
        engine.start()
        
        # Do work...
        
        engine.stop()
        
        # Or resume from a trained connectome
        engine = FeagiEngine()
        engine.load_config("feagi_configuration.toml")
        engine.load_connectome("trained_brain.connectome")
        engine.start()
        
        # Do work...
        
        engine.stop()
    """
    
    def __init__(
        self,
        feagi_path: Optional[str] = None,
        working_dir: Optional[str] = None
    ):
        """
        Initialize FEAGI engine manager
        
        Args:
            feagi_path: Path to FEAGI executable (auto-detected if None)
            working_dir: Working directory for FEAGI (default: current dir)
        """
        self.feagi_path = feagi_path
        self.working_dir = Path(working_dir) if working_dir else Path.cwd()
        self.process: Optional[subprocess.Popen] = None
        self.config_path: Optional[Path] = None
        self.genome_path: Optional[Path] = None
        self.connectome_path: Optional[Path] = None
        
        # Runtime info
        self.host = "localhost"
        self.rest_port = 8000
        self.sensory_port = 5558
        self.motor_port = 5564
        
        # Auto-detect FEAGI executable if not provided
        if self.feagi_path is None:
            self.feagi_path = self._find_feagi_executable()
    
    def _find_feagi_executable(self) -> Optional[str]:
        """
        Find FEAGI executable in common locations
        
        Priority order:
        1. Bundled binary (in package)
        2. Development build (local source)
        3. System installation (PATH)
        
        Returns:
            Path to FEAGI executable or None
        """
        import shutil
        
        # 1. Check for bundled binary (highest priority)
        bundled_binary = self._find_bundled_binary()
        if bundled_binary:
            logger.info(f"Found bundled FEAGI binary: {bundled_binary}")
            return str(bundled_binary)
        
        # 2. Check development/source builds
        possible_paths = [
            # Relative to SDK (feagi folder, not feagi-core)
            Path(__file__).parent.parent.parent.parent / "feagi" / "target" / "release" / "feagi",
            Path(__file__).parent.parent.parent.parent / "feagi" / "target" / "debug" / "feagi",
            # Also check feagi-core for backward compatibility
            Path(__file__).parent.parent.parent.parent / "feagi-core" / "target" / "release" / "feagi",
            Path(__file__).parent.parent.parent.parent / "feagi-core" / "target" / "debug" / "feagi",
        ]
        
        for path in possible_paths:
            if path.exists():
                logger.info(f"Found FEAGI at: {path}")
                return str(path)
        
        # 3. Check system PATH
        if shutil.which("feagi"):
            logger.info("Found FEAGI in system PATH")
            return "feagi"
        
        # 4. Check common install locations
        system_paths = [
            Path("/usr/local/bin/feagi"),
            Path("/usr/bin/feagi"),
            Path.home() / ".cargo" / "bin" / "feagi",
        ]
        
        for path in system_paths:
            if path.exists():
                logger.info(f"Found FEAGI at: {path}")
                return str(path)
        
        logger.warning("FEAGI executable not found. Please specify feagi_path.")
        return None
    
    def _find_bundled_binary(self) -> Optional[Path]:
        """
        Find bundled FEAGI binary for current platform
        
        Returns:
            Path to bundled binary or None if not found
        """
        import platform
        
        # Determine platform
        system = platform.system().lower()
        machine = platform.machine().lower()
        
        # Map to our directory structure
        if system == "linux":
            if machine in ["x86_64", "amd64"]:
                platform_dir = "linux-x86_64"
                binary_name = "feagi"
            elif machine in ["aarch64", "arm64"]:
                platform_dir = "linux-aarch64"
                binary_name = "feagi"
            else:
                return None
        elif system == "darwin":
            if machine in ["x86_64", "amd64"]:
                platform_dir = "darwin-x86_64"
                binary_name = "feagi"
            elif machine in ["arm64", "aarch64"]:
                platform_dir = "darwin-aarch64"
                binary_name = "feagi"
            else:
                return None
        elif system == "windows":
            if machine in ["x86_64", "amd64"]:
                platform_dir = "windows-x86_64"
                binary_name = "feagi.exe"
            else:
                return None
        else:
            return None
        
        # Look for bundled binary
        binary_path = Path(__file__).parent.parent / "bin" / platform_dir / binary_name
        
        if binary_path.exists():
            return binary_path
        
        return None
    
    def load_config(self, config_path: str | None = None) -> 'FeagiEngine':
        """
        Load FEAGI configuration file
        
        Args:
            config_path: Path to feagi_configuration.toml. If None, uses default config.
        
        Returns:
            Self for chaining
        """
        from feagi.config import ensure_default_config
        
        # Use default config if none specified
        if config_path is None:
            self.config_path = ensure_default_config()
        else:
            self.config_path = Path(config_path)
        
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        logger.info(f"Loaded config: {self.config_path}")
        
        # Parse config to extract ports (optional, for convenience)
        try:
            import toml
            config = toml.load(self.config_path)
            
            # Extract ports if available
            if "api" in config:
                self.rest_port = config["api"].get("port", 8000)
            
            if "pns" in config:
                self.sensory_port = config["pns"].get("sensory_port", 5558)
                self.motor_port = config["pns"].get("motor_port", 5564)
        except Exception as e:
            logger.warning(f"Could not parse config for ports: {e}")
        
        return self
    
    def load_genome(self, genome_path: str) -> 'FeagiEngine':
        """
        Load genome file (initial neural structure)
        
        Args:
            genome_path: Path to genome JSON file. Can be:
                - Absolute path: /path/to/genome.json
                - Relative path: ./genome.json (uses current directory)
                - Filename only: genome.json (searches in default genomes directory)
        
        Returns:
            Self for chaining
        
        Note:
            Use load_genome() for starting with a fresh neural structure.
            Use load_connectome() for loading a trained/saved state.
        """
        from feagi.paths import get_feagi_paths
        
        paths = get_feagi_paths()
        
        # Resolve path: if just a filename, look in genomes directory
        genome_path_obj = Path(genome_path)
        if not genome_path_obj.is_absolute() and "/" not in genome_path and "\\" not in genome_path:
            # Just a filename, try genomes directory
            self.genome_path = paths.resolve_path(genome_path, "genome")
        else:
            self.genome_path = Path(genome_path)
        
        if not self.genome_path.exists():
            raise FileNotFoundError(f"Genome file not found: {genome_path}")
        
        # Clear connectome if genome is loaded (mutually exclusive)
        if self.connectome_path:
            logger.warning("Genome loaded - clearing previously loaded connectome")
            self.connectome_path = None
        
        logger.info(f"Loaded genome: {self.genome_path}")
        return self
    
    def load_connectome(self, connectome_path: str) -> 'FeagiEngine':
        """
        Load connectome file (trained neural state)
        
        Args:
            connectome_path: Path to connectome file. Can be:
                - Absolute path: /path/to/trained.connectome
                - Relative path: ./trained.connectome (uses current directory)
                - Filename only: trained.connectome (searches in default connectomes directory)
        
        Returns:
            Self for chaining
        
        Note:
            Connectomes represent trained/learned neural states.
            Use load_connectome() to resume from a saved state.
            Use load_genome() to start fresh with initial structure.
        """
        from feagi.paths import get_feagi_paths
        
        paths = get_feagi_paths()
        
        # Resolve path: if just a filename, look in connectomes directory
        connectome_path_obj = Path(connectome_path)
        if not connectome_path_obj.is_absolute() and "/" not in connectome_path and "\\" not in connectome_path:
            # Just a filename, try connectomes directory
            self.connectome_path = paths.resolve_path(connectome_path, "connectome")
        else:
            self.connectome_path = Path(connectome_path)
        
        if not self.connectome_path.exists():
            raise FileNotFoundError(f"Connectome file not found: {connectome_path}")
        
        # Clear genome if connectome is loaded (mutually exclusive)
        if self.genome_path:
            logger.warning("Connectome loaded - clearing previously loaded genome")
            self.genome_path = None
        
        logger.info(f"Loaded connectome: {self.connectome_path}")
        return self
    
    def start(self, wait_for_ready: bool = True, timeout: float = 60.0) -> bool:
        """
        Start FEAGI engine
        
        Args:
            wait_for_ready: Wait for FEAGI to be ready before returning
            timeout: Maximum time to wait for ready (seconds)
        
        Returns:
            True if started successfully
        """
        if self.process is not None:
            logger.warning("FEAGI is already running")
            return True
        
        if self.feagi_path is None:
            raise RuntimeError(
                "FEAGI executable not found. Please specify feagi_path in constructor."
            )
        
        logger.info(f"Starting FEAGI engine: {self.feagi_path}")
        
        # Build command
        cmd = [self.feagi_path]
        
        if self.config_path:
            cmd.extend(["--config", str(self.config_path)])
        
        # Genome and connectome are mutually exclusive
        if self.genome_path:
            cmd.extend(["--genome", str(self.genome_path)])
        elif self.connectome_path:
            cmd.extend(["--connectome", str(self.connectome_path)])
        
        # Environment
        env = os.environ.copy()
        env["RUST_LOG"] = env.get("RUST_LOG", "info")
        
        logger.info(f"Command: {' '.join(cmd)}")
        logger.info(f"Working dir: {self.working_dir}")
        
        # Start process
        try:
            self.process = subprocess.Popen(
                cmd,
                cwd=self.working_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,  # Line buffered
            )
            
            logger.info(f"✓ FEAGI started (PID: {self.process.pid})")
            
        except Exception as e:
            logger.error(f"Failed to start FEAGI: {e}")
            return False
        
        # Wait for ready
        if wait_for_ready:
            logger.info(f"Waiting for FEAGI to be ready (timeout: {timeout}s)...")
            logger.info(f"Checking REST API at: http://{self.host}:{self.rest_port}/v1/feagi/status")
            
            if self._wait_for_ready(timeout):
                logger.info("✓ FEAGI is ready!")
                return True
            else:
                logger.error("✗ FEAGI failed to become ready")
                self.stop()
                return False
        
        return True
    
    def _wait_for_ready(self, timeout: float) -> bool:
        """
        Wait for FEAGI to be ready by polling REST API
        
        Args:
            timeout: Maximum time to wait (seconds)
        
        Returns:
            True if ready, False if timeout
        """
        start_time = time.time()
        attempts = 0
        
        while time.time() - start_time < timeout:
            attempts += 1
            elapsed = time.time() - start_time
            
            if attempts % 10 == 0:  # Log every 10 attempts (5 seconds)
                logger.info(f"  Still waiting... ({elapsed:.1f}s / {timeout:.0f}s)")
            # Check if process died
            if self.process and self.process.poll() is not None:
                logger.error("FEAGI process terminated unexpectedly")
                # Print stderr
                if self.process.stderr:
                    stderr = self.process.stderr.read()
                    if stderr:
                        logger.error(f"FEAGI stderr:\n{stderr}")
                return False
            
            # Try REST API
            try:
                import requests
                response = requests.get(
                    f"http://{self.host}:{self.rest_port}/v1/feagi/status",
                    timeout=2
                )
                if response.status_code == 200:
                    logger.info("✓ REST API is ready")
                    return True
            except requests.exceptions.ConnectionError:
                # Still starting up
                pass
            except Exception as e:
                logger.debug(f"Health check attempt failed: {e}")
            
            time.sleep(0.5)
        
        return False
    
    def stop(self, timeout: float = 10.0) -> bool:
        """
        Stop FEAGI engine gracefully
        
        Args:
            timeout: Maximum time to wait for graceful shutdown (seconds)
        
        Returns:
            True if stopped successfully
        """
        if self.process is None:
            logger.warning("FEAGI is not running")
            return True
        
        logger.info("Stopping FEAGI engine...")
        
        # Try graceful shutdown (SIGTERM)
        try:
            self.process.terminate()
            
            # Wait for process to exit
            try:
                self.process.wait(timeout=timeout)
                logger.info("✓ FEAGI stopped gracefully")
                self.process = None
                return True
            except subprocess.TimeoutExpired:
                logger.warning("FEAGI did not stop gracefully, forcing...")
                
                # Force kill (SIGKILL)
                self.process.kill()
                self.process.wait(timeout=5)
                logger.info("✓ FEAGI stopped (forced)")
                self.process = None
                return True
        
        except Exception as e:
            logger.error(f"Error stopping FEAGI: {e}")
            return False
    
    def is_running(self) -> bool:
        """Check if FEAGI is currently running"""
        return self.process is not None and self.process.poll() is None
    
    def get_logs(self, lines: int = 50) -> str:
        """
        Get recent FEAGI logs
        
        Args:
            lines: Number of lines to retrieve
        
        Returns:
            Log output as string
        """
        if self.process is None or self.process.stdout is None:
            return ""
        
        # TODO: Implement log tailing
        return "Log retrieval not yet implemented"
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - auto-stop"""
        self.stop()
    
    def __del__(self):
        """Destructor - auto-stop"""
        if self.process is not None:
            self.stop()

