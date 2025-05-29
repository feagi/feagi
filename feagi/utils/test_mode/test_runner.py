"""
FEAGI Test Runner

This module provides the main TestRunner class for executing FEAGI test modes.
Supports both mode 1 (JSON-based predictable testing) and mode 2 (numpy-based scalable testing).
"""

import os
import logging
import time
import threading
import random
from typing import Dict, Any, Optional, List, Set
from pathlib import Path

from feagi.core.state_manager import FeagiStateManager, ServiceState, GenomeState

logger = logging.getLogger(__name__)


class TestRunner:
    """
    Main test runner for FEAGI test modes.
    
    Coordinates genome loading, test execution, and result validation
    for both test mode 1 and test mode 2.
    """
    
    def __init__(
        self,
        core_api,
        test_mode: str,
        test_duration: int = 10,
        test_frequency: int = 10
    ):
        """
        Initialize test runner.
        
        Args:
            core_api: FEAGI core API service instance
            test_mode: Test mode ("mode_1" or "mode_2")
            test_duration: Duration of test in seconds
            test_frequency: Frequency of stimulation in Hz
        """
        self.core_api = core_api
        self.test_mode = test_mode
        self.test_duration = test_duration
        self.test_frequency = test_frequency
        
        # Test state
        self.test_active = False
        self.start_time = None
        self.stimulation_count = 0
        
        logger.info(f"[TEST] Initialized TestRunner for {test_mode}")
        logger.info(f"[TEST] Duration: {test_duration}s, Frequency: {test_frequency}Hz")
    
    def load_genome(self):
        """
        Load the essential genome using the core API.
        
        Returns:
            bool: True if genome was loaded successfully, False otherwise
        """
        try:
            if self.test_mode == "mode_2":
                logger.info("Loading test genome for test mode 2")
                # Use the specific test genome for mode 2
                test_genome_path = Path(__file__).parent.parent.parent / "evo" / "defaults" / "genome" / "test_genome.json"
                if not test_genome_path.exists():
                    logger.error(f"Test genome not found at: {test_genome_path}")
                    return False
                # Use deploy_genome method for loading from file
                result = self.core_api.deploy_genome(str(test_genome_path))
                if result:
                    logger.info(f"[OK] Test genome loaded successfully from: {test_genome_path}")
                    return True
                else:
                    logger.error("Failed to load test genome")
                    return False
            else:
                # Mode 1: Load essential genome (default behavior)
                logger.info("Loading essential genome for test mode 1")
                result = self.core_api.load_essential_genome()
                if result:
                    logger.info("[OK] Essential genome loaded successfully")
                    return True
                else:
                    logger.error("Failed to load essential genome")
                    return False
                    
        except Exception as e:
            logger.error(f"Error loading genome: {e}")
            return False
    
    def wait_for_brain_readiness(self, timeout: int = 30) -> bool:
        """
        Wait for the brain to be ready for testing.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            bool: True if brain is ready, False if timeout
        """
        logger.info("[TEST] Waiting for brain to be ready...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                state_manager = FeagiStateManager.instance()
                genome_state = state_manager.get_genome_state()
                
                if (genome_state == GenomeState.LOADED and 
                    state_manager.get_service_state() == ServiceState.RUNNING):
                    logger.info("[OK] Brain is ready for testing")
                    return True
                    
                logger.debug(f"[WAIT] Brain not ready yet: genome={genome_state}, service={state_manager.get_service_state()}")
                time.sleep(1)
                
            except Exception as e:
                logger.warning(f"Error checking brain readiness: {e}")
                time.sleep(1)
        
        logger.error(f"[ERR] Brain not ready after {timeout}s timeout")
        return False
    
    def run_test_mode_1(self) -> bool:
        """
        Execute test mode 1: JSON-based predictable testing.
        
        Returns:
            bool: True if test passed, False otherwise
        """
        logger.info("[TEST] Starting test mode 1: JSON-based predictable testing")
        
        try:
            from .test_mode_1 import TestMode1Handler
            
            handler = TestMode1Handler(
                core_api=self.core_api,
                test_duration=self.test_duration,
                test_frequency=self.test_frequency
            )
            
            return handler.run()
            
        except ImportError as e:
            logger.error(f"Failed to import TestMode1Handler: {e}")
            return False
        except Exception as e:
            logger.error(f"Error in test mode 1: {e}")
            return False
    
    def run_test_mode_2(self) -> bool:
        """
        Execute test mode 2: numpy-based scalable testing.
        
        Returns:
            bool: True if test passed, False otherwise
        """
        logger.info("[TEST] Starting test mode 2: numpy-based scalable testing")
        
        try:
            from .test_mode_2 import TestMode2Handler
            
            handler = TestMode2Handler(
                core_api=self.core_api,
                test_duration=self.test_duration,
                test_frequency=self.test_frequency
            )
            
            return handler.run()
            
        except ImportError as e:
            logger.error(f"Failed to import TestMode2Handler: {e}")
            return False
        except Exception as e:
            logger.error(f"Error in test mode 2: {e}")
            return False
    
    def run(self) -> bool:
        """
        Execute the test runner.
        
        Returns:
            bool: True if test passed, False otherwise
        """
        logger.info(f"[TEST] Starting FEAGI test runner: {self.test_mode}")
        
        try:
            # Step 1: Load appropriate genome
            if not self.load_genome():
                logger.error("[ERR] Failed to load genome")
                return False
            
            # Step 2: Wait for brain readiness
            if not self.wait_for_brain_readiness():
                logger.error("[ERR] Brain not ready for testing")
                return False
            
            # Step 3: Execute the appropriate test mode
            if self.test_mode == "mode_1":
                result = self.run_test_mode_1()
            elif self.test_mode == "mode_2":
                result = self.run_test_mode_2()
            else:
                logger.error(f"[ERR] Unknown test mode: {self.test_mode}")
                return False
            
            # Step 4: Report results
            if result:
                logger.info(f"[OK] Test {self.test_mode} completed successfully")
                return True
            else:
                logger.error(f"[ERR] Test {self.test_mode} failed")
                return False
                
        except Exception as e:
            logger.error(f"[ERR] Test runner failed: {e}")
            return False 