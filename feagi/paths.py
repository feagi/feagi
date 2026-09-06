"""
FEAGI Path Management

Cross-platform path utilities for FEAGI configuration, data, logs, and cache.

Directory Structure:
    - Config, logs, cache: ~/.feagi/ (hidden, system-managed)
    - User content: ~/Documents/FEAGI/ (macOS/Windows) or ~/FEAGI/ (Linux)

Platform-specific behavior:
    Linux:
        Config:      ~/.feagi/config/
        Logs:        ~/.feagi/logs/
        Cache:       ~/.feagi/cache/
        Genomes:     ~/FEAGI/genomes/
        Connectomes: ~/FEAGI/connectomes/
    
    macOS:
        Config:      ~/.feagi/config/
        Logs:        ~/.feagi/logs/
        Cache:       ~/.feagi/cache/
        Genomes:     ~/Documents/FEAGI/Genomes/
        Connectomes: ~/Documents/FEAGI/Connectomes/
    
    Windows:
        Config:      ~/.feagi/config/
        Logs:        ~/.feagi/logs/
        Cache:       ~/.feagi/cache/
        Genomes:     ~/Documents/FEAGI/Genomes/
        Connectomes: ~/Documents/FEAGI/Connectomes/

Example:
    from feagi.paths import FeagiPaths
    
    paths = FeagiPaths()
    
    # Get default config path
    config_file = paths.get_default_config()
    
    # Ensure directories exist
    paths.ensure_all()
    
    # Get genome directory
    genome_dir = paths.genomes_dir
"""

import platform
import shutil
from datetime import datetime
from pathlib import Path
from typing import Optional


class FeagiPaths:
    """
    Cross-platform path manager for FEAGI directories.
    
    Provides consistent access to configuration, data, logs, and cache
    directories across Linux, macOS, and Windows.
    """
    
    def __init__(self):
        """Initialize path manager with platform-specific defaults."""
        self._home = Path.home()
        self._system = platform.system().lower()
        
        # Hidden system directories (always in ~/.feagi/)
        self._feagi_home = self._home / ".feagi"
        self.config_dir = self._feagi_home / "config"
        self.logs_dir = self._feagi_home / "logs"
        self.cache_dir = self._feagi_home / "cache"
        
        # Visible user content directories (platform-specific)
        if self._system in ("darwin", "windows", "win32"):
            # macOS and Windows: Use Documents folder
            user_content_root = self._home / "Documents" / "FEAGI"
            self.genomes_dir = user_content_root / "Genomes"
            self.connectomes_dir = user_content_root / "Connectomes"
        else:
            # Linux: Use home directory directly
            user_content_root = self._home / "FEAGI"
            self.genomes_dir = user_content_root / "genomes"
            self.connectomes_dir = user_content_root / "connectomes"
    
    def get_default_config(self) -> Path:
        """
        Get the default configuration file path.
        
        Returns:
            Path to feagi_configuration.toml in config directory
        """
        return self.config_dir / "feagi_configuration.toml"
    
    def ensure_config_dir(self) -> Path:
        """
        Ensure config directory exists.
        
        Returns:
            Path to config directory
        """
        self.config_dir.mkdir(parents=True, exist_ok=True)
        return self.config_dir
    
    def ensure_logs_dir(self) -> Path:
        """
        Ensure logs directory exists.
        
        Returns:
            Path to logs directory
        """
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        return self.logs_dir

    def create_log_run_dir(self, component: str, retention: int) -> Path:
        """
        Create a per-run log directory with component subfolder.

        Args:
            component: Component name (e.g., "feagi", "bv").
            retention: Number of most recent runs to keep.

        Returns:
            Path to the component-specific run directory.
        """
        self.ensure_logs_dir()
        component_dir = self.logs_dir / component
        component_dir.mkdir(parents=True, exist_ok=True)
        self._prune_log_runs(component_dir, retention)
        return self._create_unique_run_dir(component_dir)

    def _create_unique_run_dir(self, parent_dir: Path) -> Path:
        """Create a unique run directory under a parent folder."""
        timestamp = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        candidate = parent_dir / timestamp
        if not candidate.exists():
            candidate.mkdir(parents=True, exist_ok=True)
            return candidate
        counter = 1
        while True:
            candidate = parent_dir / f"{timestamp}_{counter}"
            if not candidate.exists():
                candidate.mkdir(parents=True, exist_ok=True)
                return candidate
            counter += 1

    def _prune_log_runs(self, parent_dir: Path, retention: int) -> None:
        """Remove log run directories beyond retention count."""
        run_dirs = sorted(
            [
                path
                for path in parent_dir.iterdir()
                if path.is_dir() and path.name.startswith("run_")
            ],
            key=lambda path: path.name,
        )
        excess = len(run_dirs) - retention
        if excess <= 0:
            return
        for path in run_dirs[:excess]:
            shutil.rmtree(path, ignore_errors=True)
    
    def ensure_cache_dir(self) -> Path:
        """
        Ensure cache directory exists.
        
        Returns:
            Path to cache directory
        """
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        return self.cache_dir
    
    def ensure_genomes_dir(self) -> Path:
        """
        Ensure genomes directory exists.
        
        Returns:
            Path to genomes directory
        """
        self.genomes_dir.mkdir(parents=True, exist_ok=True)
        return self.genomes_dir
    
    def ensure_connectomes_dir(self) -> Path:
        """
        Ensure connectomes directory exists.
        
        Returns:
            Path to connectomes directory
        """
        self.connectomes_dir.mkdir(parents=True, exist_ok=True)
        return self.connectomes_dir
    
    def ensure_all(self) -> None:
        """
        Ensure all FEAGI directories exist.
        
        Creates all standard directories if they don't exist:
        - config, logs, cache (hidden)
        - genomes, connectomes (visible)
        """
        self.ensure_config_dir()
        self.ensure_logs_dir()
        self.ensure_cache_dir()
        self.ensure_genomes_dir()
        self.ensure_connectomes_dir()
    
    def get_genome_path(self, filename: str) -> Path:
        """
        Get full path to a genome file in the genomes directory.
        
        Args:
            filename: Genome filename (e.g., "my_brain.genome")
        
        Returns:
            Full path to genome file
        """
        return self.genomes_dir / filename
    
    def get_connectome_path(self, filename: str) -> Path:
        """
        Get full path to a connectome file in the connectomes directory.
        
        Args:
            filename: Connectome filename (e.g., "trained_brain.connectome")
        
        Returns:
            Full path to connectome file
        """
        return self.connectomes_dir / filename
    
    def get_log_path(self, filename: str) -> Path:
        """
        Get full path to a log file in the logs directory.
        
        Args:
            filename: Log filename (e.g., "feagi.log")
        
        Returns:
            Full path to log file
        """
        return self.logs_dir / filename
    
    def resolve_path(
        self, path: str | Path, category: Optional[str] = None
    ) -> Path:
        """
        Resolve a path, handling relative paths based on category.
        
        If path is absolute, returns as-is.
        If path is relative and category is specified, resolves it relative to
        the category directory.
        Otherwise resolves relative to current working directory.
        
        Args:
            path: Path to resolve (string or Path object)
            category: Optional path category such as genome, connectome,
                configuration, or log.
        
        Returns:
            Resolved absolute Path
        
        Example:
            # Absolute path - returns as-is
            paths.resolve_path("/absolute/path/brain.genome")
            
            # Relative path with category
            paths.resolve_path("my_brain.genome", "genome")
            # -> ~/Documents/FEAGI/Genomes/my_brain.genome
            
            # Relative path without category
            paths.resolve_path("my_brain.genome")
            # -> ./my_brain.genome (current directory)
        """
        p = Path(path)
        
        if p.is_absolute():
            return p
        
        if category:
            category_lower = category.lower()
            if category_lower == "genome":
                return self.genomes_dir / p
            elif category_lower == "connectome":
                return self.connectomes_dir / p
            elif category_lower == "config":
                return self.config_dir / p
            elif category_lower == "log":
                return self.logs_dir / p
        
        # Default to current working directory for relative paths
        return Path.cwd() / p
    
    def __repr__(self) -> str:
        """String representation showing all directory paths."""
        return (
            f"FeagiPaths(\n"
            f"  platform={self._system}\n"
            f"  config={self.config_dir}\n"
            f"  logs={self.logs_dir}\n"
            f"  cache={self.cache_dir}\n"
            f"  genomes={self.genomes_dir}\n"
            f"  connectomes={self.connectomes_dir}\n"
            f")"
        )


# Global singleton instance
_paths_instance: Optional[FeagiPaths] = None


def get_feagi_paths() -> FeagiPaths:
    """
    Get the global FeagiPaths singleton instance.
    
    Returns:
        Global FeagiPaths instance
    """
    global _paths_instance
    if _paths_instance is None:
        _paths_instance = FeagiPaths()
    return _paths_instance
