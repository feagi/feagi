#!/usr/bin/env python3
"""
FEAGI Session Manager

Helper script to manage a FEAGI session including:
- Loading genomes
- Starting/stopping simulation
- Managing runtime parameters
"""

import os
import sys
import time
import logging
import json
from typing import Dict, Optional, Any, List, Union

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_session")

# Import feagi client
from feagi_client import FeagiClient

class FeagiSession:
    """Manager for a FEAGI session with genome and simulation management"""
    
    def __init__(self, host: str = "127.0.0.1", port: int = 5555, timeout: float = 5.0):
        """
        Initialize the FEAGI session.
        
        Args:
            host: FEAGI hostname or IP
            port: Command API port (default 5555)
            timeout: Command timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.client = FeagiClient(host=host, port=port)
        
        # Cache for status information
        self.is_running = False
        self.is_genome_loaded = False
        self.current_genome = None
        self.cortical_areas = {}
    
    def check_connection(self) -> bool:
        """
        Check if FEAGI is running and we can connect.
        
        Returns:
            True if connected, False otherwise
        """
        try:
            self.is_running = self.client.is_running()
            if not self.is_running:
                logger.error(f"FEAGI is not running at {self.host}:{self.port}")
            return self.is_running
        except Exception as e:
            logger.error(f"Failed to connect to FEAGI: {e}")
            self.is_running = False
            return False
    
    def update_status(self) -> Dict[str, Any]:
        """
        Update cached status information.
        
        Returns:
            Current status dictionary
        """
        if not self.check_connection():
            return {}
        
        try:
            # Get status information
            status = self.client.get_status()
            
            # Update cached values
            if "status" in status:
                self.is_genome_loaded = status["status"].get("genome_loaded", False)
                
                # If genome is loaded, get cortical areas
                if self.is_genome_loaded and not self.cortical_areas:
                    cortical_response = self.client.command("get_cortical_areas")
                    if "cortical_areas" in cortical_response:
                        self.cortical_areas = cortical_response["cortical_areas"]
            
            # Get simulation state
            sim_state = self.client.get_simulation_state()
            self.is_running = sim_state.get("running", False)
            
            # Return combined status
            return {
                "is_running": self.is_running,
                "is_genome_loaded": self.is_genome_loaded,
                "cortical_areas": self.cortical_areas
            }
        
        except Exception as e:
            logger.error(f"Failed to update status: {e}")
            return {}
    
    def list_available_genomes(self) -> List[str]:
        """
        Get list of available genomes.
        
        Returns:
            List of genome names
        """
        if not self.check_connection():
            return []
        
        try:
            response = self.client.command("list_genomes")
            if "genomes" in response:
                return response["genomes"]
            return []
        except Exception as e:
            logger.error(f"Failed to list genomes: {e}")
            return []
    
    def load_genome(self, genome_name: str) -> bool:
        """
        Load a genome by name.
        
        Args:
            genome_name: Name of the genome to load
            
        Returns:
            True if successful, False otherwise
        """
        if not self.check_connection():
            return False
        
        try:
            logger.info(f"Loading genome: {genome_name}")
            response = self.client.command("load_genome", {"name": genome_name})
            
            if response.get("status") == "success":
                logger.info(f"Successfully loaded genome: {genome_name}")
                self.current_genome = genome_name
                self.is_genome_loaded = True
                
                # Update cortical areas
                self.update_status()
                return True
            else:
                logger.error(f"Failed to load genome: {response.get('message', 'Unknown error')}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to load genome: {e}")
            return False
    
    def start_simulation(self) -> bool:
        """
        Start the FEAGI simulation.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.check_connection():
            return False
            
        # Check if genome is loaded first
        if not self.is_genome_loaded:
            logger.error("Cannot start simulation: No genome loaded")
            return False
        
        try:
            logger.info("Starting simulation")
            response = self.client.command("start_simulation")
            
            if response.get("status") == "success":
                logger.info("Simulation started successfully")
                self.is_running = True
                return True
            else:
                logger.error(f"Failed to start simulation: {response.get('message', 'Unknown error')}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to start simulation: {e}")
            return False
    
    def stop_simulation(self) -> bool:
        """
        Stop the FEAGI simulation.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.check_connection():
            return False
        
        try:
            logger.info("Stopping simulation")
            response = self.client.command("stop_simulation")
            
            if response.get("status") == "success":
                logger.info("Simulation stopped successfully")
                self.is_running = False
                return True
            else:
                logger.error(f"Failed to stop simulation: {response.get('message', 'Unknown error')}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to stop simulation: {e}")
            return False
    
    def ensure_ready_state(self, genome_name: Optional[str] = None) -> bool:
        """
        Ensure FEAGI is in a ready state with a genome loaded and simulation running.
        
        Args:
            genome_name: Optional name of genome to load if none is loaded
            
        Returns:
            True if ready, False otherwise
        """
        # Check connection and update status
        if not self.update_status():
            return False
        
        # Check if a genome is loaded
        if not self.is_genome_loaded:
            if genome_name:
                # Try to load the specified genome
                if not self.load_genome(genome_name):
                    # If that fails, try to load any available genome
                    available_genomes = self.list_available_genomes()
                    if available_genomes:
                        if not self.load_genome(available_genomes[0]):
                            return False
                    else:
                        logger.error("No genomes available to load")
                        return False
            else:
                # No genome specified, try to load any available genome
                available_genomes = self.list_available_genomes()
                if available_genomes:
                    if not self.load_genome(available_genomes[0]):
                        return False
                else:
                    logger.error("No genomes available to load")
                    return False
        
        # If we get here, a genome should be loaded
        # Now check if simulation is running
        if not self.is_running:
            if not self.start_simulation():
                return False
        
        # Final check to make sure everything is ready
        self.update_status()
        return self.is_genome_loaded and self.is_running
    
    def upload_genome(self, genome_file: str) -> bool:
        """
        Upload a genome from a file.
        
        Args:
            genome_file: Path to genome file
            
        Returns:
            True if successful, False otherwise
        """
        if not self.check_connection():
            return False
            
        if not os.path.exists(genome_file):
            logger.error(f"Genome file not found: {genome_file}")
            return False
        
        try:
            # Load the genome file
            with open(genome_file, 'r') as f:
                genome_data = json.load(f)
            
            # Get the name from the file
            genome_name = os.path.basename(genome_file).split('.')[0]
            
            # Upload the genome
            logger.info(f"Uploading genome from file: {genome_file}")
            response = self.client.command("upload_genome", {
                "name": genome_name,
                "genome": genome_data
            })
            
            if response.get("status") == "success":
                logger.info(f"Successfully uploaded genome: {genome_name}")
                return True
            else:
                logger.error(f"Failed to upload genome: {response.get('message', 'Unknown error')}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to upload genome: {e}")
            return False


def main():
    """Example usage of the FEAGI session manager"""
    import argparse
    
    parser = argparse.ArgumentParser(description="FEAGI Session Manager")
    parser.add_argument("--host", default="127.0.0.1", help="FEAGI host")
    parser.add_argument("--port", type=int, default=5555, help="FEAGI command port")
    parser.add_argument("--load", help="Load a specific genome")
    parser.add_argument("--upload", help="Upload a genome from file")
    parser.add_argument("--start", action="store_true", help="Start simulation")
    parser.add_argument("--stop", action="store_true", help="Stop simulation")
    parser.add_argument("--ensure-ready", action="store_true", help="Ensure FEAGI is ready (load genome and start sim)")
    args = parser.parse_args()
    
    # Create session
    session = FeagiSession(host=args.host, port=args.port)
    
    # Check connection
    if not session.check_connection():
        logger.error("Failed to connect to FEAGI")
        return 1
    
    # Update status
    status = session.update_status()
    logger.info(f"FEAGI Status: Running={status.get('is_running', False)}, Genome Loaded={status.get('is_genome_loaded', False)}")
    
    # Handle commands
    if args.upload:
        if session.upload_genome(args.upload):
            logger.info(f"Successfully uploaded genome from {args.upload}")
        else:
            logger.error(f"Failed to upload genome from {args.upload}")
            return 1
    
    if args.load:
        if session.load_genome(args.load):
            logger.info(f"Successfully loaded genome: {args.load}")
        else:
            logger.error(f"Failed to load genome: {args.load}")
            return 1
    
    if args.start:
        if session.start_simulation():
            logger.info("Successfully started simulation")
        else:
            logger.error("Failed to start simulation")
            return 1
    
    if args.stop:
        if session.stop_simulation():
            logger.info("Successfully stopped simulation")
        else:
            logger.error("Failed to stop simulation")
            return 1
    
    if args.ensure_ready:
        if session.ensure_ready_state(args.load):
            logger.info("FEAGI is in ready state with genome loaded and simulation running")
        else:
            logger.error("Failed to ensure FEAGI ready state")
            return 1
    
    # Final status update
    status = session.update_status()
    logger.info(f"Final FEAGI Status: Running={status.get('is_running', False)}, Genome Loaded={status.get('is_genome_loaded', False)}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main()) 