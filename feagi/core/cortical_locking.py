"""
Cortical Area Locking for NPU-BDU Coordination

This module provides high-performance cortical area locking mechanisms for coordinating
between NPU (Neural Processing Unit) and BDU (Brain Development Unit) operations.

Key Design Principles:
- BDU operations take precedence over NPU processing
- In-memory only (no persistence) - locks are transient
- Thread-safe with atomic operations
- RTOS/Rust compatible design
- No lock timeouts - robust error handling for unlock
- Cortical area-level granular control

Architecture:
- NPU checks locks before processing cortical areas
- BDU locks areas during neurogenesis/synaptogenesis operations
- Sleep Manager coordinates global locking operations
- State Manager stores locking state for cross-component visibility
"""

import threading
import time
from typing import Set, Dict, Optional, List
from enum import IntEnum
from dataclasses import dataclass
import logging

from feagi.utils.logger import setup_logger

logger = setup_logger()


class LockResult(IntEnum):
    """Result codes for cortical area locking operations."""
    SUCCESS = 0
    ALREADY_LOCKED = 1
    INVALID_CORTICAL_IDX = 2
    LOCK_CONFLICT = 3
    UNLOCK_ERROR = 4
    GLOBAL_LOCK_ACTIVE = 5  # Cannot lock individual areas when global lock is active


@dataclass
class LockInfo:
    """Information about a cortical area lock."""
    cortical_idx: int
    locked_by: str  # Component name (e.g., "BDU", "SleepManager")
    lock_time: float  # Timestamp when locked
    operation: str  # Description of operation (e.g., "neurogenesis", "synaptogenesis")


@dataclass
class GlobalLockInfo:
    """Information about a global brain lock."""
    locked_by: str  # Component name (e.g., "SleepManager", "BDU")
    lock_time: float  # Timestamp when locked
    operation: str  # Description of operation (e.g., "global_maintenance", "full_brain_restructure")
    affected_areas: Optional[List[int]] = None  # Specific areas affected, None means all areas


class CorticalAreaLockManager:
    """
    High-performance cortical area locking manager.
    
    Provides atomic locking operations for cortical areas to coordinate between
    NPU and BDU operations. Designed for minimal overhead and RTOS compatibility.
    
    Features:
    - Atomic lock/unlock operations
    - Thread-safe with minimal contention
    - In-memory only (no persistence)
    - Granular cortical area-level control
    - BDU operations take precedence
    """
    
    def __init__(self):
        """Initialize cortical area lock manager."""
        # Thread-safe lock for atomic operations
        self._lock = threading.RLock()
        
        # Active locks: cortical_idx -> LockInfo
        self._locked_areas: Dict[int, LockInfo] = {}
        
        # Global brain lock
        self._global_lock: Optional[GlobalLockInfo] = None
        
        # Lock statistics for monitoring
        self._lock_count = 0
        self._unlock_count = 0
        self._conflict_count = 0
        self._global_lock_count = 0
        self._global_unlock_count = 0
        
        logger.info("🔒 CorticalAreaLockManager initialized with global locking support")
    
    def lock_area(self, cortical_idx: int, locked_by: str, operation: str = "unknown") -> LockResult:
        """Lock a cortical area for exclusive BDU operations.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            locked_by: Component requesting the lock (e.g., "BDU", "SleepManager")
            operation: Description of the operation requiring the lock
            
        Returns:
            LockResult indicating success or failure reason
        """
        if cortical_idx < 0:
            return LockResult.INVALID_CORTICAL_IDX
            
        with self._lock:
            # Check if global lock is active
            if self._global_lock is not None:
                logger.debug(f"🔒 Cannot lock area {cortical_idx}: global lock active by {self._global_lock.locked_by}")
                self._conflict_count += 1
                return LockResult.GLOBAL_LOCK_ACTIVE
            
            # Check if already locked
            if cortical_idx in self._locked_areas:
                existing_lock = self._locked_areas[cortical_idx]
                logger.debug(f"🔒 Area {cortical_idx} already locked by {existing_lock.locked_by}")
                self._conflict_count += 1
                return LockResult.ALREADY_LOCKED
            
            # Create new lock
            lock_info = LockInfo(
                cortical_idx=cortical_idx,
                locked_by=locked_by,
                lock_time=time.time(),
                operation=operation
            )
            
            self._locked_areas[cortical_idx] = lock_info
            self._lock_count += 1
            
            logger.debug(f"🔒 Locked area {cortical_idx} for {locked_by} ({operation})")
            return LockResult.SUCCESS
    
    def unlock_area(self, cortical_idx: int, locked_by: str) -> LockResult:
        """Unlock a cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            locked_by: Component that originally locked the area
            
        Returns:
            LockResult indicating success or failure reason
        """
        if cortical_idx < 0:
            return LockResult.INVALID_CORTICAL_IDX
            
        with self._lock:
            # Check if area is locked
            if cortical_idx not in self._locked_areas:
                logger.warning(f"🔓 Attempted to unlock area {cortical_idx} but it's not locked")
                return LockResult.UNLOCK_ERROR
            
            existing_lock = self._locked_areas[cortical_idx]
            
            # Verify the same component is unlocking
            if existing_lock.locked_by != locked_by:
                logger.warning(f"🔓 Lock ownership mismatch: area {cortical_idx} locked by {existing_lock.locked_by}, unlock attempted by {locked_by}")
                return LockResult.UNLOCK_ERROR
            
            # Remove lock
            del self._locked_areas[cortical_idx]
            self._unlock_count += 1
            
            lock_duration = time.time() - existing_lock.lock_time
            logger.debug(f"🔓 Unlocked area {cortical_idx} from {locked_by} (held for {lock_duration:.3f}s)")
            return LockResult.SUCCESS
    
    def is_area_locked(self, cortical_idx: int) -> bool:
        """Check if a cortical area is currently locked.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            True if the area is locked, False otherwise
        """
        if cortical_idx < 0:
            return False
            
        with self._lock:
            return cortical_idx in self._locked_areas
    
    def get_lock_info(self, cortical_idx: int) -> Optional[LockInfo]:
        """Get information about a cortical area lock.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            LockInfo if area is locked, None otherwise
        """
        if cortical_idx < 0:
            return None
            
        with self._lock:
            return self._locked_areas.get(cortical_idx)
    
    def get_locked_areas(self) -> List[int]:
        """Get list of all currently locked cortical area indices.
        
        Returns:
            List of cortical_idx values that are currently locked
        """
        with self._lock:
            return list(self._locked_areas.keys())
    
    def lock_multiple_areas(self, cortical_indices: List[int], locked_by: str, operation: str = "batch") -> Dict[int, LockResult]:
        """Lock multiple cortical areas atomically.
        
        This is useful for operations that span multiple areas (e.g., inter-area synaptogenesis).
        Either all areas are locked successfully, or none are locked.
        
        Args:
            cortical_indices: List of cortical area indices to lock
            locked_by: Component requesting the locks
            operation: Description of the operation requiring the locks
            
        Returns:
            Dictionary mapping cortical_idx to LockResult for each area
        """
        if not cortical_indices:
            return {}
        
        results = {}
        locked_areas = []
        
        with self._lock:
            # First pass: check if all areas can be locked
            for cortical_idx in cortical_indices:
                if cortical_idx < 0:
                    results[cortical_idx] = LockResult.INVALID_CORTICAL_IDX
                elif cortical_idx in self._locked_areas:
                    results[cortical_idx] = LockResult.ALREADY_LOCKED
                else:
                    results[cortical_idx] = LockResult.SUCCESS
            
            # Check if any locks failed
            failed_locks = [idx for idx, result in results.items() if result != LockResult.SUCCESS]
            if failed_locks:
                logger.debug(f"🔒 Batch lock failed for areas {failed_locks}, no areas locked")
                return results
            
            # Second pass: lock all areas
            for cortical_idx in cortical_indices:
                lock_info = LockInfo(
                    cortical_idx=cortical_idx,
                    locked_by=locked_by,
                    lock_time=time.time(),
                    operation=operation
                )
                self._locked_areas[cortical_idx] = lock_info
                locked_areas.append(cortical_idx)
                self._lock_count += 1
            
            logger.debug(f"🔒 Batch locked {len(locked_areas)} areas for {locked_by} ({operation})")
            return results
    
    def unlock_multiple_areas(self, cortical_indices: List[int], locked_by: str) -> Dict[int, LockResult]:
        """Unlock multiple cortical areas.
        
        Args:
            cortical_indices: List of cortical area indices to unlock
            locked_by: Component that originally locked the areas
            
        Returns:
            Dictionary mapping cortical_idx to LockResult for each area
        """
        if not cortical_indices:
            return {}
        
        results = {}
        
        with self._lock:
            for cortical_idx in cortical_indices:
                if cortical_idx < 0:
                    results[cortical_idx] = LockResult.INVALID_CORTICAL_IDX
                elif cortical_idx not in self._locked_areas:
                    results[cortical_idx] = LockResult.UNLOCK_ERROR
                else:
                    existing_lock = self._locked_areas[cortical_idx]
                    if existing_lock.locked_by != locked_by:
                        results[cortical_idx] = LockResult.UNLOCK_ERROR
                    else:
                        del self._locked_areas[cortical_idx]
                        self._unlock_count += 1
                        results[cortical_idx] = LockResult.SUCCESS
            
            successful_unlocks = [idx for idx, result in results.items() if result == LockResult.SUCCESS]
            logger.debug(f"🔓 Batch unlocked {len(successful_unlocks)} areas from {locked_by}")
            
            return results
    
    def force_unlock_all(self, locked_by: str) -> int:
        """Force unlock all areas locked by a specific component.
        
        This is useful for cleanup when a component shuts down unexpectedly.
        
        Args:
            locked_by: Component to unlock all areas for
            
        Returns:
            Number of areas that were unlocked
        """
        unlocked_count = 0
        
        with self._lock:
            areas_to_unlock = []
            for cortical_idx, lock_info in self._locked_areas.items():
                if lock_info.locked_by == locked_by:
                    areas_to_unlock.append(cortical_idx)
            
            for cortical_idx in areas_to_unlock:
                del self._locked_areas[cortical_idx]
                unlocked_count += 1
                self._unlock_count += 1
            
            if unlocked_count > 0:
                logger.info(f"🔓 Force unlocked {unlocked_count} areas from {locked_by}")
            
            return unlocked_count
    
    # ===== GLOBAL BRAIN LOCKING METHODS =====
    
    def lock_global_brain(self, locked_by: str, operation: str = "global_operation", 
                         affected_areas: Optional[List[int]] = None) -> LockResult:
        """Lock the entire brain for global operations.
        
        This is much more efficient than locking individual cortical areas and provides
        atomic global operations. When a global lock is active, no individual area
        locks can be acquired.
        
        Args:
            locked_by: Component requesting the global lock (e.g., "SleepManager")
            operation: Description of the global operation
            affected_areas: Optional list of specific areas affected (None = all areas)
            
        Returns:
            LockResult indicating success or failure reason
        """
        with self._lock:
            # Check if global lock already exists
            if self._global_lock is not None:
                existing_lock = self._global_lock
                logger.debug(f"🌍 Global brain already locked by {existing_lock.locked_by}")
                self._conflict_count += 1
                return LockResult.ALREADY_LOCKED
            
            # Check if any individual areas are locked
            if self._locked_areas:
                locked_by_others = set(lock.locked_by for lock in self._locked_areas.values())
                logger.debug(f"🌍 Cannot acquire global lock: {len(self._locked_areas)} areas locked by {locked_by_others}")
                self._conflict_count += 1
                return LockResult.LOCK_CONFLICT
            
            # Create global lock
            self._global_lock = GlobalLockInfo(
                locked_by=locked_by,
                lock_time=time.time(),
                operation=operation,
                affected_areas=affected_areas
            )
            
            self._global_lock_count += 1
            
            area_info = f" (affecting {len(affected_areas)} areas)" if affected_areas else " (affecting all areas)"
            logger.info(f"🌍 Global brain locked by {locked_by} for {operation}{area_info}")
            
            return LockResult.SUCCESS
    
    def unlock_global_brain(self, locked_by: str) -> LockResult:
        """Unlock the global brain lock.
        
        Args:
            locked_by: Component that originally locked the brain
            
        Returns:
            LockResult indicating success or failure reason
        """
        with self._lock:
            # Check if global lock exists
            if self._global_lock is None:
                logger.warning(f"🌍 Attempted to unlock global brain but no global lock exists")
                return LockResult.UNLOCK_ERROR
            
            # Verify the same component is unlocking
            if self._global_lock.locked_by != locked_by:
                logger.warning(f"🌍 Global lock ownership mismatch: locked by {self._global_lock.locked_by}, unlock attempted by {locked_by}")
                return LockResult.UNLOCK_ERROR
            
            # Calculate lock duration
            lock_duration = time.time() - self._global_lock.lock_time
            operation = self._global_lock.operation
            
            # Remove global lock
            self._global_lock = None
            self._global_unlock_count += 1
            
            logger.info(f"🌍 Global brain unlocked from {locked_by} (held for {lock_duration:.3f}s, operation: {operation})")
            return LockResult.SUCCESS
    
    def is_global_brain_locked(self) -> bool:
        """Check if the global brain lock is active.
        
        Returns:
            True if global brain is locked, False otherwise
        """
        with self._lock:
            return self._global_lock is not None
    
    def get_global_lock_info(self) -> Optional[GlobalLockInfo]:
        """Get information about the global brain lock.
        
        Returns:
            GlobalLockInfo if global lock is active, None otherwise
        """
        with self._lock:
            return self._global_lock
    
    def force_unlock_global_brain(self, locked_by: str) -> bool:
        """Force unlock the global brain (emergency cleanup).
        
        Args:
            locked_by: Component to force unlock for
            
        Returns:
            True if global lock was cleared, False if no lock or wrong owner
        """
        with self._lock:
            if self._global_lock is None:
                return False
            
            if self._global_lock.locked_by != locked_by:
                return False
            
            operation = self._global_lock.operation
            self._global_lock = None
            self._global_unlock_count += 1
            
            logger.warning(f"🚨 Emergency: force unlocked global brain from {locked_by} (operation: {operation})")
            return True
    
    def get_statistics(self) -> Dict[str, int]:
        """Get locking statistics for monitoring.
        
        Returns:
            Dictionary with lock statistics
        """
        with self._lock:
            return {
                "currently_locked": len(self._locked_areas),
                "total_locks": self._lock_count,
                "total_unlocks": self._unlock_count,
                "conflicts": self._conflict_count,
                "global_lock_active": self._global_lock is not None,
                "total_global_locks": self._global_lock_count,
                "total_global_unlocks": self._global_unlock_count
            }
    
    def clear_all_locks(self) -> int:
        """Clear all locks (emergency cleanup).
        
        Returns:
            Number of locks that were cleared
        """
        with self._lock:
            cleared_count = len(self._locked_areas)
            global_cleared = self._global_lock is not None
            
            self._locked_areas.clear()
            self._global_lock = None
            
            if cleared_count > 0 or global_cleared:
                logger.warning(f"🚨 Emergency: cleared {cleared_count} area locks and {'1' if global_cleared else '0'} global lock")
            
            return cleared_count + (1 if global_cleared else 0)


# Global singleton instance
_cortical_lock_manager: Optional[CorticalAreaLockManager] = None
_manager_lock = threading.Lock()


def get_cortical_lock_manager() -> CorticalAreaLockManager:
    """Get the global cortical area lock manager instance.
    
    Returns:
        CorticalAreaLockManager singleton instance
    """
    global _cortical_lock_manager
    
    if _cortical_lock_manager is None:
        with _manager_lock:
            if _cortical_lock_manager is None:
                _cortical_lock_manager = CorticalAreaLockManager()
    
    return _cortical_lock_manager


# Convenience functions for common operations
def lock_cortical_area(cortical_idx: int, locked_by: str, operation: str = "unknown") -> LockResult:
    """Convenience function to lock a cortical area."""
    return get_cortical_lock_manager().lock_area(cortical_idx, locked_by, operation)


def unlock_cortical_area(cortical_idx: int, locked_by: str) -> LockResult:
    """Convenience function to unlock a cortical area."""
    return get_cortical_lock_manager().unlock_area(cortical_idx, locked_by)


def is_cortical_area_locked(cortical_idx: int) -> bool:
    """Convenience function to check if a cortical area is locked."""
    return get_cortical_lock_manager().is_area_locked(cortical_idx)


# Global brain locking convenience functions
def lock_global_brain(locked_by: str, operation: str = "global_operation", 
                     affected_areas: Optional[List[int]] = None) -> LockResult:
    """Convenience function to lock the entire brain."""
    return get_cortical_lock_manager().lock_global_brain(locked_by, operation, affected_areas)


def unlock_global_brain(locked_by: str) -> LockResult:
    """Convenience function to unlock the global brain."""
    return get_cortical_lock_manager().unlock_global_brain(locked_by)


def is_global_brain_locked() -> bool:
    """Convenience function to check if the global brain is locked."""
    return get_cortical_lock_manager().is_global_brain_locked()
