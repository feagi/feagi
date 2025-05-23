"""Base service class for all FEAGI domain services."""

from typing import Optional, Any
from feagi.utils.logger import setup_logger

logger = setup_logger()


class BaseService:
    """
    Base class for all FEAGI domain services.
    
    Provides common functionality and patterns used across all services.
    """
    
    def __init__(self, connectome_manager, state_manager=None):
        """
        Initialize base service.
        
        Args:
            connectome_manager: ConnectomeManager instance
            state_manager: FeagiStateManager instance (optional)
        """
        self._connectome_manager = connectome_manager
        self.state_manager = state_manager
        self.logger = logger
        
    def _validate_connectome_ready(self) -> bool:
        """Check if connectome manager is ready for operations."""
        if not self._connectome_manager:
            self.logger.warning("Connectome manager not available")
            return False
            
        if not hasattr(self._connectome_manager, 'fcl_manager') or not self._connectome_manager.fcl_manager:
            self.logger.warning("FCL manager not initialized")
            return False
            
        return True
        
    def _validate_genome_loaded(self) -> bool:
        """Check if a genome is currently loaded."""
        if not self.state_manager:
            return False
            
        return self.state_manager.is_genome_loaded()
        
    def _get_current_genome(self) -> Optional[dict]:
        """Get the currently loaded genome data."""
        if not self.state_manager:
            return None
            
        return getattr(self.state_manager, 'genome', None)
        
    def _safe_execute(self, operation, error_message: str, default_return=None):
        """
        Safely execute an operation with error handling.
        
        Args:
            operation: Function to execute
            error_message: Error message prefix
            default_return: Default value to return on error
            
        Returns:
            Operation result or default_return on error
        """
        try:
            return operation()
        except Exception as e:
            self.logger.error(f"{error_message}: {str(e)}")
            return default_return 