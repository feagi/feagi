#!/usr/bin/env python3
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
FEAGI Sensorimotor Binary Data Test

This script tests sending binary sensory data to FEAGI using the ZMQ sensorimotor
stream and verifies the data is correctly injected into the FCL.

This test demonstrates high-performance real-time binary data exchange with
optimized socket configurations for minimal latency.
"""

import asyncio
import logging
import random
import sys
import uuid

import numpy as np
import zmq
import zmq.asyncio

# Import feagi_bytes for binary serialization
from feagi_bytes import ByteStructureEncoder, ByteStructureTranslator

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("feagi_sensorimotor_test")


class SensorimotorClient:
    """Client for interacting with FEAGI's sensorimotor ZMQ streams."""

    def __init__(
        self, feagi_host="localhost", sensory_port=5558, motor_port=5564, agent_id=None
    ):
        """Initialize sensorimotor client.

        Args:
            feagi_host: Hostname/IP of FEAGI server
            sensory_port: Port for sensory data (PUSH)
            motor_port: Port for motor data (SUB)
            agent_id: Unique ID for this agent (generated if None)
        """
        self.feagi_host = feagi_host
        self.sensory_port = sensory_port
        self.motor_port = motor_port
        self.agent_id = agent_id or str(uuid.uuid4())

        # Connection status
        self.connected = False

        # State tracking
        self.feagi_state = "unknown"  # "active" or "standby"

        # Initialize ZMQ
        self._context = zmq.asyncio.Context()

        # Sensory socket (PUSH)
        self._sensory_socket = None

        # Motor socket (SUB)
        self._motor_socket = None

        # Binary encoder/decoder
        self.encoder = ByteStructureEncoder()
        self.translator = ByteStructureTranslator()

        # Callbacks for system events
        self._state_change_callbacks = []

    async def connect(self):
        """Connect to FEAGI."""
        try:
            # Set up ZMQ connection with optimized socket settings
            self._setup_connection()

            # Register for system messages
            self._motor_socket.setsockopt(zmq.SUBSCRIBE, b"system")

            # Start motor data receiver
            self._motor_receiver_task = asyncio.create_task(self._motor_data_receiver())

            # Check FEAGI state
            await self.check_feagi_state()

            return True
        except Exception as e:
            logger.error(f"Failed to connect to FEAGI: {e}")
            self.connected = False
            return False

    async def disconnect(self):
        """Disconnect from FEAGI streams."""
        logger.info("Disconnecting from FEAGI...")
        if (
            hasattr(self, "_motor_receiver_task")
            and not self._motor_receiver_task.done()
        ):
            self._motor_receiver_task.cancel()
            try:
                await self._motor_receiver_task
            except asyncio.CancelledError:
                pass

        if hasattr(self, "_sensory_socket"):
            self._sensory_socket.close()
        if hasattr(self, "_motor_socket"):
            self._motor_socket.close()
        if hasattr(self, "_context"):
            self._context.term()
        self.connected = False
        logger.info("Disconnected from FEAGI")

    async def send_sensory_data(
        self, channel_id: str, data, data_type="neurons"
    ) -> bool:
        """
        Send sensory data to FEAGI.

        Args:
            channel_id: Identifier for the sensory channel (e.g., "vision_input")
            data: Data to send (depends on data_type)
            data_type: Type of data to send ("neurons", "image", "raw")

        Returns:
            True if data was sent successfully, False otherwise
        """
        if not self.connected:
            logger.error("Not connected to FEAGI. Call connect() first.")
            return False

        if self.feagi_state != "active":
            logger.warning(
                f"FEAGI is in {self.feagi_state} mode. Sensory data will be ignored."
            )
            # Send anyway in case FEAGI transitions to active mode

        try:
            # Different types of data need different encoding
            if data_type == "neurons":
                # Encode neuron data in categories format
                # Convert to the structure expected by encoder: {cortical_id: {"x": [], "y": [], "z": [], "potentials": []}}
                categorized_data = {
                    channel_id: {"x": [], "y": [], "z": [], "potentials": []}
                }

                # Populate from the coordinate-based dictionary
                for coord, potential in data.items():
                    # Parse coordinate string "x,y,z"
                    x, y, z = map(int, coord.split(","))

                    # Add to appropriate arrays
                    categorized_data[channel_id]["x"].append(x)
                    categorized_data[channel_id]["y"].append(y)
                    categorized_data[channel_id]["z"].append(z)
                    categorized_data[channel_id]["potentials"].append(potential)

                binary_data = self.encoder.encode_neuron_categories(categorized_data)
            elif data_type == "flat_neurons":
                # Encode neuron data in flat format
                binary_data = self.encoder.encode_neuron_flat(
                    data["cortical_ids"],
                    data["x_coords"],
                    data["y_coords"],
                    data["z_coords"],
                    data["potentials"],
                )
            elif data_type == "image":
                # Encode image data
                binary_data = self.encoder.encode_raw_image(image=data)
            elif data_type == "raw":
                # Already binary data, send as-is
                binary_data = data
            else:
                logger.error(f"Unknown data type: {data_type}")
                return False

            # Send as multipart message with channel ID and binary data
            await self._sensory_socket.send_multipart(
                [channel_id.encode(), binary_data]
            )

            logger.info(
                f"Sent {len(binary_data)} bytes of {data_type} data to channel {channel_id}"
            )
            return True

        except Exception as e:
            logger.error(f"Error sending sensory data: {e}")
            return False

    async def check_feagi_state(self) -> str:
        """Check FEAGI's current state.

        Returns:
            Current state of FEAGI ('active', 'standby', or 'unknown')
        """
        try:
            # Send status check message
            await self._sensory_socket.send_multipart([b"system", b"STATUS_CHECK"])
            logger.info("Sent status check to FEAGI")

            # State will be updated asynchronously by motor_data_receiver
            return self.feagi_state
        except Exception as e:
            logger.error(f"Error checking FEAGI state: {e}")
            return "unknown"

    def register_state_change_callback(self, callback):
        """Register callback for state change events.

        Args:
            callback: Function(new_state) to call when state changes
        """
        self._state_change_callbacks.append(callback)

    def _update_state(self, new_state: str):
        """Update internal state tracking.

        Args:
            new_state: New FEAGI state
        """
        if self.feagi_state != new_state:
            old_state = self.feagi_state
            self.feagi_state = new_state
            logger.info(f"FEAGI state changed: {old_state} → {new_state}")

            # Call registered callbacks
            for callback in self._state_change_callbacks:
                try:
                    callback(new_state)
                except Exception as e:
                    logger.error(f"Error in state change callback: {e}")

    async def receive_motor_data(self, timeout=1.0):
        """Receive motor data from FEAGI.

        Args:
            timeout: Timeout in seconds to wait for data

        Returns:
            Decoded motor data or None if no data received
        """
        if not self.connected:
            logger.error("Not connected to FEAGI. Call connect() first.")
            return None

        try:
            # Wait for data with timeout
            multipart = await asyncio.wait_for(
                self._motor_socket.recv_multipart(), timeout=timeout
            )

            if len(multipart) < 2:
                logger.error(f"Received malformed motor data: {multipart}")
                return None

            # First part is topic, second part is data
            topic = multipart[0].decode()
            data = multipart[1]

            # Handle system messages differently
            if topic == "system":
                await self._handle_system_message(data)
                return None

            # Regular motor data
            logger.info(f"Received motor data on topic {topic}: {len(data)} bytes")

            # Decode the binary data
            decoded_data = self.translator.decode_message(data)
            return decoded_data
        except asyncio.TimeoutError:
            logger.debug("No motor data received (timeout)")
            return None
        except Exception as e:
            logger.error(f"Error receiving motor data: {e}")
            return None

    async def _motor_data_receiver(self):
        """Background task to receive motor data from FEAGI."""
        logger.info("Motor data receiver task started")

        while self.connected:
            try:
                # Non-blocking receive
                multipart = await self._motor_socket.recv_multipart()

                if len(multipart) < 2:
                    logger.warning(f"Received malformed data: {multipart}")
                    continue

                topic = multipart[0].decode()
                data = multipart[1]

                # Handle system messages
                if topic == "system":
                    await self._handle_system_message(data)
                else:
                    # Regular motor data
                    logger.debug(
                        f"Received motor data on topic {topic}: {len(data)} bytes"
                    )

            except asyncio.CancelledError:
                logger.info("Motor data receiver task cancelled")
                break
            except Exception as e:
                logger.error(f"Error in motor data receiver: {e}")
                await asyncio.sleep(0.1)

    async def _handle_system_message(self, data):
        """Handle system messages from FEAGI.

        Args:
            data: System message data
        """
        try:
            message = data.decode()

            # Handle state information
            if message.startswith("FEAGI_STATE:"):
                new_state = message.split(":")[1]
                self._update_state(new_state)

            # Handle state change notification
            elif message.startswith("FEAGI_STATE_CHANGE:"):
                new_state = message.split(":")[1]
                self._update_state(new_state)

            # Handle other system messages
            else:
                logger.debug(f"Received system message: {message}")

        except Exception as e:
            logger.error(f"Error handling system message: {e}")

    def _setup_connection(self):
        """Set up ZMQ connection to FEAGI with optimized real-time settings."""
        # Sensory socket (PUSH for agent → FEAGI)
        self._sensory_socket = self._context.socket(zmq.PUSH)

        # Configure for real-time with no queuing
        self._sensory_socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
        self._sensory_socket.setsockopt(zmq.LINGER, 0)  # Don't wait when closing

        self._sensory_socket.connect(f"tcp://{self.feagi_host}:{self.sensory_port}")

        # Motor socket (SUB for FEAGI → agent)
        self._motor_socket = self._context.socket(zmq.SUB)

        # Configure for real-time with no queuing
        self._motor_socket.setsockopt(zmq.RCVHWM, 1)  # Minimal receive queue
        self._motor_socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent
        self._motor_socket.setsockopt(zmq.LINGER, 0)  # Don't wait when closing

        # Subscribe to all messages
        self._motor_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self._motor_socket.connect(f"tcp://{self.feagi_host}:{self.motor_port}")

        logger.info(
            f"Connected to FEAGI at {self.feagi_host}:{self.sensory_port}/{self.motor_port}"
        )
        self.connected = True


# Helper function to generate test neuron data
def generate_test_neuron_data(
    cortical_id, width=64, height=64, depth=1, activity_percentage=20
):
    """
    Generate test neuron data for a cortical area.

    Args:
        cortical_id: Cortical area ID
        width: Width of the cortical area
        height: Height of the cortical area
        depth: Depth of the cortical area
        activity_percentage: Percentage of neurons to activate

    Returns:
        Dictionary mapping coordinates to activation values
    """
    # Create empty dictionary for neuron data
    neuron_data = {}

    # Calculate number of neurons to activate
    total_neurons = width * height * depth
    active_count = int(total_neurons * activity_percentage / 100)

    # Generate random activations
    for _ in range(active_count):
        # Generate random coordinates within the cortical area bounds
        x = random.randint(0, width - 1)  # 0 to 63 for width=64
        y = random.randint(0, height - 1)  # 0 to 63 for height=64
        z = 0  # Keep z at 0 as specified

        # Generate random activation between 0.5 and 1.0
        activation = random.uniform(0.5, 1.0)

        # Add to neuron data dictionary
        coord = f"{x},{y},{z}"
        neuron_data[coord] = activation

    logger.info(f"Generated neuron data with {len(neuron_data)} active neurons")
    return neuron_data


# Helper function to generate flat neuron data format
def generate_flat_neuron_data(neurons_per_area=20, num_areas=3):
    """
    Generate flat format neuron data for testing.

    Returns:
        Dictionary with flat neuron data
    """
    cortical_ids = []
    x_coords = []
    y_coords = []
    z_coords = []
    potentials = []

    for area_idx in range(num_areas):
        # This would normally be different cortical areas, but for testing
        # we'll use the same one with different coordinates
        area_id = "iic400"  # Use correct cortical ID

        for _ in range(neurons_per_area):
            # Generate coordinates within bounds
            x = random.randint(0, 63)  # 0 to 63
            y = random.randint(0, 63)  # 0 to 63
            z = 0  # Keep z at 0

            # Add to flat arrays
            cortical_ids.append(area_id)
            x_coords.append(x)
            y_coords.append(y)
            z_coords.append(z)
            potentials.append(random.uniform(0.5, 1.0))

    # Create the flat format dictionary
    flat_data = {
        "cortical_ids": cortical_ids,
        "x_coords": x_coords,
        "y_coords": y_coords,
        "z_coords": z_coords,
        "potentials": potentials,
    }

    logger.info(
        f"Generated flat neuron data with {len(cortical_ids)} neurons across {num_areas} areas"
    )
    return flat_data


# Helper function to generate test image data
def generate_test_image(width=64, height=64, channels=3, pattern="random"):
    """
    Generate a test image for sensory input.

    Args:
        width: Image width
        height: Image height
        channels: Number of channels (1=grayscale, 3=RGB)
        pattern: Pattern to generate ("random", "checker", "gradient")

    Returns:
        NumPy array of shape (height, width, channels)
    """
    # Create image with correct dimensions for the iic400 cortical area
    if pattern == "random":
        # Random noise
        if channels == 1:
            img = np.random.randint(0, 256, (height, width), dtype=np.uint8)
        else:
            img = np.random.randint(0, 256, (height, width, channels), dtype=np.uint8)

    elif pattern == "checker":
        # Checkerboard pattern
        x, y = np.indices((height, width))
        checker = (x + y) % 16 < 8
        if channels == 1:
            img = checker.astype(np.uint8) * 255
        else:
            img = np.zeros((height, width, channels), dtype=np.uint8)
            for c in range(channels):
                img[:, :, c] = checker.astype(np.uint8) * 255

    elif pattern == "gradient":
        # Horizontal gradient
        if channels == 1:
            img = np.tile(np.linspace(0, 255, width, dtype=np.uint8), (height, 1))
        else:
            img = np.zeros((height, width, channels), dtype=np.uint8)
            for c in range(channels):
                img[:, :, c] = np.tile(
                    np.linspace(0, 255, width, dtype=np.uint8), (height, 1)
                )

    else:
        raise ValueError(f"Unknown pattern: {pattern}")

    logger.info(f"Generated image data with shape {img.shape}")
    return img


async def handle_state_change(new_state):
    """Handle FEAGI state changes.

    Args:
        new_state: New FEAGI state ("active" or "standby")
    """
    logger.info(f"State change callback triggered: FEAGI is now in {new_state} mode")

    if new_state == "active":
        logger.info("FEAGI is ready to process sensory data!")
    else:
        logger.info("FEAGI is in standby mode. Sensory data will be ignored.")


async def main():
    """Main test routine."""
    logger.info("-------- STARTING TESTS --------")

    # Create client
    client = SensorimotorClient()

    # Connect to FEAGI
    try:
        await client.connect()

        # Register callback for state changes
        client.register_state_change_callback(handle_state_change)

        # Check initial state
        await client.check_feagi_state()
        logger.info(f"Current FEAGI state: {client.feagi_state}")

        # TEST 1: Send neuron category data
        logger.info("\n----- TEST 1: NEURON CATEGORY DATA -----")
        neuron_data = generate_test_neuron_data("iic400", width=64, height=64)
        logger.info(f"Sending data with FEAGI in {client.feagi_state} mode")
        await client.send_sensory_data("iic400", neuron_data, data_type="neurons")

        # Wait a bit to allow processing
        await asyncio.sleep(1)

        # TEST 2: Send flat neuron data
        logger.info("\n----- TEST 2: FLAT NEURON DATA -----")
        flat_data = generate_flat_neuron_data(neurons_per_area=20, num_areas=3)
        logger.info(f"Sending data with FEAGI in {client.feagi_state} mode")
        await client.send_sensory_data("iic400", flat_data, data_type="flat_neurons")

        # Wait a bit to allow processing
        await asyncio.sleep(1)

        # TEST 3: Send image data
        logger.info("\n----- TEST 3: IMAGE DATA -----")
        image_data = generate_test_image(
            width=64, height=64, channels=3, pattern="checker"
        )
        logger.info(f"Sending data with FEAGI in {client.feagi_state} mode")
        await client.send_sensory_data("iic400", image_data, data_type="image")

        # Wait a bit to allow processing
        await asyncio.sleep(1)

        # TEST 4: Check state handling
        logger.info("\n----- TEST 4: STATE HANDLING -----")
        logger.info("Sending state check request")
        await client.check_feagi_state()
        await asyncio.sleep(1)  # Give time for response
        logger.info(f"Final FEAGI state: {client.feagi_state}")

        # Wait for potential motor responses
        logger.info("\nWaiting for potential motor responses...")
        await asyncio.sleep(5)

        logger.info("-------- TESTS COMPLETED --------")

    except Exception as e:
        logger.error(f"Test error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # Clean up
        logger.info("Disconnecting from FEAGI...")
        await client.disconnect()


if __name__ == "__main__":
    # Run the test
    asyncio.run(main())
