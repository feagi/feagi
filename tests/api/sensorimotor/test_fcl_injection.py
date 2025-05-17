#!/usr/bin/env python3
"""
Test script to verify that sensory data is properly injected into FCL.

This script tests sending sensory data to FEAGI and validates that it's 
properly injected into the FCL by using the visualization API to observe
neuron activations.
"""

import asyncio
import logging
import sys
import time
import random
import numpy as np
import zmq
import zmq.asyncio
from feagi_bytes import ByteStructureTranslator
from feagi_connector.zmq.client import ZmqFeagiClient

# Set up logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('feagi_fcl_injection_test.log')
    ]
)
logger = logging.getLogger("feagi_fcl_injection_test")

class FCLInjectionTester:
    """
    Test harness for validating FCL injection of sensory data.
    """
    
    def __init__(self, host="localhost"):
        """Initialize the test harness."""
        self.host = host
        self.client = None
        self.agent_id = f"fcl-test-{int(time.time())}"
        self.translator = ByteStructureTranslator()
        
    async def setup(self):
        """Set up the test environment."""
        # Create FEAGI client
        self.client = ZmqFeagiClient(
            host=self.host,
            control_port=5555,
            sensorimotor_port=5558,
            visualization_port=5560
        )
        
        # Connect to FEAGI
        logger.info("Connecting to FEAGI...")
        connected = await self.client.connect()
        if not connected:
            logger.error("Failed to connect to FEAGI")
            return False
        
        # Register agent
        logger.info(f"Registering agent {self.agent_id}...")
        response = await self.client.register_agent(self.agent_id, "fcl_test")
        if response.get("status") != "success":
            logger.error(f"Failed to register agent: {response}")
            await self.teardown()
            return False
        
        logger.info("Successfully connected and registered with FEAGI")
        return True
    
    async def teardown(self):
        """Clean up the test environment."""
        if self.client:
            # Deregister agent
            if hasattr(self, 'agent_id'):
                try:
                    await self.client.deregister_agent(self.agent_id)
                except Exception as e:
                    logger.warning(f"Error deregistering agent: {e}")
            
            # Disconnect
            await self.client.disconnect()
            self.client = None
        
        logger.info("Test environment cleaned up")
    
    async def send_test_data(self, cortical_area="iv00_C"):
        """
        Send test data to a cortical area and return the data that was sent.
        
        Args:
            cortical_area: ID of the cortical area to send data to
            
        Returns:
            Dictionary mapping (x,y,z) coordinates to activation values
        """
        # Generate test data - random activations on a 5x5 grid in the cortical area
        test_data = {}
        for x in range(5):
            for y in range(5):
                # Generate a random activation value between 0.7 and 1.0
                activation = random.uniform(0.7, 1.0)
                coord = f"{x},{y},0"
                test_data[coord] = activation
        
        # Convert to categorized format expected by FEAGI
        categorized_data = {
            cortical_area: {
                "x": [],
                "y": [],
                "z": [],
                "potentials": []
            }
        }
        
        for coord, potential in test_data.items():
            x, y, z = map(int, coord.split(","))
            categorized_data[cortical_area]["x"].append(x)
            categorized_data[cortical_area]["y"].append(y)
            categorized_data[cortical_area]["z"].append(z)
            categorized_data[cortical_area]["potentials"].append(potential)
        
        # Send the data
        logger.info(f"Sending test data to {cortical_area}...")
        success = await self.client.send_sensory_data(cortical_area, categorized_data)
        if not success:
            logger.error("Failed to send data")
            return None
        
        logger.info(f"Successfully sent {len(test_data)} activations to {cortical_area}")
        return test_data
    
    async def verify_fcl_injection(self, sent_data, cortical_area="iv00_C"):
        """
        Verify that data was properly injected into FCL by checking the neuron potentials.
        
        Args:
            sent_data: Dictionary mapping coordinates to activation values
            cortical_area: ID of the cortical area to check
            
        Returns:
            True if verification succeeded, False otherwise
        """
        # Let FCL process the data
        logger.info("Waiting for FCL to process data...")
        await asyncio.sleep(1)
        
        # Get current visualization data
        logger.info("Requesting visualization data...")
        try:
            # Send visualization request for specific cortical area
            vis_request = {
                "type": "visualization_request",
                "request_type": "neuron_potentials",
                "cortical_area": cortical_area
            }
            
            # Send request
            binary_request = self.translator.create_message(vis_request)
            response_data = await self.client.send_visualization_command(binary_request)
            
            # Try to decode the response
            if not response_data:
                logger.error("No response from visualization request")
                return False
            
            # Decode the response
            response = self.translator.decode_message(response_data)
            
            # Extract neuron potentials from response
            if "neuron_data" not in response:
                logger.error(f"No neuron data in response: {response}")
                return False
            
            # Verify that each activated neuron appears in the response
            verification_results = []
            for coord, sent_potential in sent_data.items():
                x, y, z = map(int, coord.split(","))
                
                # Look for matching neuron in response
                found = False
                for i in range(len(response["neuron_data"]["x"])):
                    if (response["neuron_data"]["x"][i] == x and 
                        response["neuron_data"]["y"][i] == y and 
                        response["neuron_data"]["z"][i] == z):
                        
                        # Found matching neuron, check potential
                        received_potential = response["neuron_data"]["potentials"][i]
                        
                        # Allow for some numerical difference due to FCL processing
                        potential_diff = abs(sent_potential - received_potential)
                        max_allowed_diff = 0.1  # 10% difference allowed
                        
                        if potential_diff > max_allowed_diff:
                            logger.warning(f"Potential at {coord} differs too much: sent={sent_potential}, received={received_potential}")
                            verification_results.append(False)
                        else:
                            logger.info(f"Verified neuron at {coord}: sent={sent_potential}, received={received_potential}")
                            verification_results.append(True)
                        
                        found = True
                        break
                
                if not found:
                    logger.warning(f"Neuron at {coord} not found in visualization data")
                    verification_results.append(False)
            
            # Success if all neurons were verified
            success_rate = sum(verification_results) / len(verification_results) if verification_results else 0
            logger.info(f"Verification success rate: {success_rate:.1%}")
            
            # Consider success if at least 80% of neurons were verified
            return success_rate >= 0.8
            
        except Exception as e:
            logger.error(f"Error verifying FCL injection: {e}")
            import traceback
            traceback.print_exc()
            return False

async def run_test():
    """Run the full FCL injection test."""
    tester = FCLInjectionTester()
    
    try:
        # Set up the test environment
        setup_success = await tester.setup()
        if not setup_success:
            logger.error("Test setup failed")
            return False
        
        # Send test data
        sent_data = await tester.send_test_data()
        if not sent_data:
            logger.error("Failed to send test data")
            return False
        
        # Verify FCL injection
        verification_result = await tester.verify_fcl_injection(sent_data)
        
        if verification_result:
            logger.info("✅ FCL INJECTION TEST PASSED")
        else:
            logger.error("❌ FCL INJECTION TEST FAILED")
        
        return verification_result
        
    except Exception as e:
        logger.error(f"Error during test: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Clean up
        await tester.teardown()

if __name__ == "__main__":
    logger.info("=== Starting FEAGI FCL Injection Test ===")
    success = asyncio.run(run_test())
    exit_code = 0 if success else 1
    logger.info("=== Test completed ===")
    sys.exit(exit_code) 