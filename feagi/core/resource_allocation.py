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
Resource allocation module for FEAGI.

This module implements advanced CPU allocation strategies and load balancing
for optimal performance across different process priorities.
"""
import os

from feagi.utils.logger import setup_logger

logger = setup_logger()
import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Set

logger = logging.getLogger("feagi.resource_allocation")


class AllocationStrategy(Enum):
    """CPU allocation strategies."""

    STATIC = auto()  # Fixed allocation based on process priority
    DYNAMIC = auto()  # Dynamic allocation based on actual usage
    ADAPTIVE = auto()  # Adaptive allocation based on workload patterns
    PRIORITY_BOOST = auto()  # Temporarily boost priority of specific processes


@dataclass
class CoreAllocation:
    """CPU core allocation for a process."""

    process_name: str
    core_ids: List[int]
    strategy: AllocationStrategy = AllocationStrategy.STATIC
    priority: int = 1
    utilization: float = 0.0
    last_updated: float = field(default_factory=time.time)


class CPUAllocator:
    """
    Advanced CPU allocation with load balancing.

    This class manages CPU cores allocation to different processes,
    implements load balancing, and adjusts allocations based on
    process priorities and resource usage patterns.
    """

    def __init__(self, total_cores: Optional[int] = None):
        """
        Initialize the CPU allocator.

        Args:
            total_cores: Total number of CPU cores to manage.
                         If None, will use os.cpu_count()
        """
        self._lock = threading.RLock()
        self.total_cores = total_cores or os.cpu_count() or 1
        self.core_ids = list(range(self.total_cores))
        self.allocations: Dict[str, CoreAllocation] = {}
        self.reserved_cores: Set[int] = set()

        # Keep track of physical cores vs logical cores if possible
        self.physical_cores = self._detect_physical_cores()

        # Core pools for different priority levels
        self.priority_core_pools: Dict[int, List[int]] = (
            self._initialize_priority_pools()
        )

    def _detect_physical_cores(self) -> List[int]:
        """
        Attempt to detect physical cores for better allocation.

        Returns:
            List of physical core IDs (may be duplicated for logical cores)
        """
        physical_core_map = {}  # Maps logical cores to physical cores

        try:
            import psutil

            # Try to get physical core information
            logical_to_physical = {}

            # On Linux, use cpu_info() to map logical cores to physical ones
            if hasattr(psutil, "cpu_info") and hasattr(
                psutil.cpu_info(), "physical_core"
            ):
                for i, info in enumerate(psutil.cpu_info()):
                    logical_to_physical[i] = info.physical_core

            # If psutil method doesn't work, fallback to using all cores
            if not logical_to_physical:
                return self.core_ids

            # Group logical cores by physical core
            physical_core_map = {}
            for logical, physical in logical_to_physical.items():
                if physical not in physical_core_map:
                    physical_core_map[physical] = []
                physical_core_map[physical].append(logical)

        except (ImportError, AttributeError, Exception) as e:
            logger.warning(f"Error detecting physical cores: {e}")
            return self.core_ids

        # Create a flattened list with physical cores first, then additional logical cores
        physical_cores = []
        for _physical_id, logical_ids in physical_core_map.items():
            # Take one logical core from each physical core
            physical_cores.append(logical_ids[0])

            # Add remaining logical cores from same physical core
            for logical_id in logical_ids[1:]:
                physical_cores.append(logical_id)

        return physical_cores if physical_cores else self.core_ids

    def _initialize_priority_pools(self) -> Dict[int, List[int]]:
        """
        Initialize core pools for each priority level.

        Returns:
            Dictionary mapping priority levels to lists of core IDs
        """
        pools = {}

        # Priority 1 (critical) gets 50% of cores
        # Use physical cores first if detected
        p1_count = max(1, self.total_cores // 2)

        if self.physical_cores:
            pools[1] = self.physical_cores[:p1_count]
        else:
            pools[1] = self.core_ids[:p1_count]

        # Priority 2 (important) gets 30% of cores
        p2_count = max(1, int(self.total_cores * 0.3))
        remaining_cores = [c for c in self.core_ids if c not in pools[1]]
        pools[2] = remaining_cores[:p2_count]

        # Priority 3 (background) gets remaining cores
        pools[3] = [c for c in self.core_ids if c not in pools[1] and c not in pools[2]]

        return pools

    def allocate_cores(
        self,
        process_name: str,
        priority: int,
        num_cores: Optional[int] = None,
        strategy: AllocationStrategy = AllocationStrategy.STATIC,
    ) -> List[int]:
        """
        Allocate CPU cores to a process based on priority.

        Args:
            process_name: Name of the process
            priority: Priority level (1-3)
            num_cores: Number of cores to allocate. If None, determined by priority
            strategy: Allocation strategy to use

        Returns:
            List of allocated core IDs
        """
        with self._lock:
            # Check if process already has an allocation
            if process_name in self.allocations:
                return self.allocations[process_name].core_ids

            # Validate priority
            priority = max(1, min(3, priority))

            # Determine number of cores based on priority if not specified
            if num_cores is None:
                if priority == 1:
                    # Critical processes get more cores
                    num_cores = max(1, len(self.priority_core_pools[1]) // 2)
                elif priority == 2:
                    # Important processes get a moderate amount
                    num_cores = max(1, len(self.priority_core_pools[2]) // 2)
                else:
                    # Background processes get minimal cores
                    num_cores = 1

            # Make sure num_cores is at least 1
            num_cores = max(1, num_cores)

            # Get available cores from the appropriate priority pool
            available_cores = [
                c
                for c in self.priority_core_pools[priority]
                if c not in self.reserved_cores
            ]

            # If not enough cores in priority pool, take from lower priority
            if len(available_cores) < num_cores:
                lower_priority = priority + 1
                while lower_priority <= 3 and len(available_cores) < num_cores:
                    additional_cores = [
                        c
                        for c in self.priority_core_pools[lower_priority]
                        if c not in self.reserved_cores
                    ]
                    available_cores.extend(additional_cores)
                    lower_priority += 1

            # If still not enough, take from higher priority (sacrifice)
            if len(available_cores) < num_cores and priority > 1:
                higher_priority = priority - 1
                while higher_priority >= 1 and len(available_cores) < num_cores:
                    additional_cores = [
                        c
                        for c in self.priority_core_pools[higher_priority]
                        if c not in self.reserved_cores
                    ]
                    available_cores.extend(additional_cores)
                    higher_priority -= 1

            # Limit to what's available
            num_cores = min(num_cores, len(available_cores))

            # Allocate cores
            allocated_cores = available_cores[:num_cores]
            self.reserved_cores.update(allocated_cores)

            # Record allocation
            allocation = CoreAllocation(
                process_name=process_name,
                core_ids=allocated_cores,
                strategy=strategy,
                priority=priority,
            )
            self.allocations[process_name] = allocation

            logger.info(
                f"Allocated {len(allocated_cores)} cores to {process_name} (priority {priority})"
            )
            return allocated_cores

    def release_cores(self, process_name: str) -> None:
        """
        Release cores allocated to a process.

        Args:
            process_name: Name of the process
        """
        with self._lock:
            if process_name not in self.allocations:
                return

            # Get core IDs to release
            allocation = self.allocations[process_name]
            core_ids = allocation.core_ids

            # Remove from reserved cores
            self.reserved_cores.difference_update(core_ids)

            # Remove allocation
            del self.allocations[process_name]

            logger.info(f"Released {len(core_ids)} cores from {process_name}")

    def rebalance_allocations(self) -> Dict[str, List[int]]:
        """
        Rebalance core allocations based on current usage.

        Returns:
            Dictionary mapping process names to new core allocations
        """
        with self._lock:
            # Track changes to make
            changes = {}

            # Group allocations by priority
            by_priority = {1: [], 2: [], 3: []}

            for _process_name, allocation in self.allocations.items():
                by_priority[allocation.priority].append(allocation)

            # Start with highest priority processes
            for priority in [1, 2, 3]:
                processes = by_priority[priority]

                # Skip if no processes at this priority
                if not processes:
                    continue

                # Sort by utilization (highest first)
                processes.sort(key=lambda a: a.utilization, reverse=True)

                # Identify processes with high and low utilization
                high_util_threshold = 0.7  # 70%
                low_util_threshold = 0.3  # 30%

                high_util = [
                    p for p in processes if p.utilization >= high_util_threshold
                ]
                low_util = [
                    p
                    for p in processes
                    if p.utilization <= low_util_threshold and len(p.core_ids) > 1
                ]

                # If we have high utilization processes and low utilization processes,
                # redistribute cores from low to high
                if high_util and low_util:
                    # For each high utilization process, take one core from a low util process
                    for high_process in high_util:
                        if not low_util:
                            break

                        low_process = low_util.pop(0)

                        # Take one core from low process
                        core_to_move = low_process.core_ids.pop()
                        self.reserved_cores.remove(core_to_move)

                        # Add to high process
                        high_process.core_ids.append(core_to_move)
                        self.reserved_cores.add(core_to_move)

                        # Record changes
                        changes[high_process.process_name] = (
                            high_process.core_ids.copy()
                        )
                        changes[low_process.process_name] = low_process.core_ids.copy()

                        logger.info(
                            f"Rebalanced: moved core {core_to_move} from {low_process.process_name} to {high_process.process_name}"
                        )

                        # If low process has only one core left, remove from candidates
                        if len(low_process.core_ids) <= 1:
                            try:
                                low_util.remove(low_process)
                            except ValueError:
                                pass

            return changes

    def update_utilization(self, process_name: str, utilization: float) -> None:
        """
        Update CPU utilization for a process.

        Args:
            process_name: Name of the process
            utilization: CPU utilization as a fraction (0.0-1.0)
        """
        with self._lock:
            if process_name not in self.allocations:
                return

            allocation = self.allocations[process_name]
            allocation.utilization = utilization
            allocation.last_updated = time.time()

    def get_allocation(self, process_name: str) -> Optional[CoreAllocation]:
        """
        Get core allocation for a process.

        Args:
            process_name: Name of the process

        Returns:
            CoreAllocation object or None if not found
        """
        with self._lock:
            return self.allocations.get(process_name)

    def get_all_allocations(self) -> Dict[str, CoreAllocation]:
        """
        Get all core allocations.

        Returns:
            Dictionary mapping process names to their CoreAllocation
        """
        with self._lock:
            return self.allocations.copy()

    def set_allocation_strategy(
        self, process_name: str, strategy: AllocationStrategy
    ) -> bool:
        """
        Set allocation strategy for a process.

        Args:
            process_name: Name of the process
            strategy: New allocation strategy

        Returns:
            True if successful, False if process not found
        """
        with self._lock:
            if process_name not in self.allocations:
                return False

            self.allocations[process_name].strategy = strategy
            return True

    def temporarily_boost_priority(
        self, process_name: str, duration_seconds: float = 30.0
    ) -> bool:
        """
        Temporarily boost priority for a process.

        Args:
            process_name: Name of the process
            duration_seconds: Duration of priority boost in seconds

        Returns:
            True if successful, False if process not found
        """
        with self._lock:
            if process_name not in self.allocations:
                return False

            allocation = self.allocations[process_name]
            original_strategy = allocation.strategy

            # Set to priority boost strategy
            allocation.strategy = AllocationStrategy.PRIORITY_BOOST

            # Schedule restoration of original strategy
            def restore_strategy():
                time.sleep(duration_seconds)
                with self._lock:
                    if process_name in self.allocations:
                        self.allocations[process_name].strategy = original_strategy

            # Start a thread to restore the strategy after duration
            restore_thread = threading.Thread(target=restore_strategy, daemon=True)
            restore_thread.start()

            return True
