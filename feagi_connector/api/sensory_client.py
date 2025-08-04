"""
FEAGI Sensory Client

Simple client for sending sensory data to FEAGI using feagi_data_processing library.
"""

import logging
from typing import Dict, Tuple, Optional
import zmq
import numpy as np

# Import the correct feagi_data_processing library
try:
    import feagi_data_processing as fdp
    HAS_FEAGI_DATA_PROCESSING = True
except ImportError:
    HAS_FEAGI_DATA_PROCESSING = False

logger = logging.getLogger(__name__)


class FeagiSensoryClient:
    """FEAGI sensory client that sends feagi_data_processing binary format."""
    
    def __init__(self, host: str = "localhost", port: int = 5558, timeout: int = 5):
        """Initialize sensory client with PUSH socket for feagi_data_processing format."""
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.context = None

    def connect(self) -> bool:
        """Connect to FEAGI sensory stream."""
        try:
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.PUSH)
            self.socket.setsockopt(zmq.LINGER, 1000)
            self.socket.setsockopt(zmq.SNDHWM, 100)  # Prevent buffer overflow
            
            address = f"tcp://{self.host}:{self.port}"
            self.socket.connect(address)
            
            logger.info(f"Connected to FEAGI sensory stream at {address}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to sensory stream: {e}")
            return False

    def send_sensory_data(
        self, cortical_area: str, neuron_data: Dict[Tuple[int, int, int], float]
    ) -> bool:
        """Send sensory data using feagi_data_processing format."""
        try:
            if not self.socket:
                logger.error("Not connected to sensory stream")
                return False
                
            if not neuron_data:
                logger.debug(f"No neuron data to send for {cortical_area}")
                return True
            
            if not HAS_FEAGI_DATA_PROCESSING:
                logger.error("feagi_data_processing library not available")
                return False
            
            # Encode using feagi_data_processing
            binary_data = self._encode_with_feagi_data_processing(cortical_area, neuron_data)
            if binary_data:
                self.socket.send(binary_data, zmq.NOBLOCK)
                logger.debug(f"Sent {len(neuron_data)} neurons for '{cortical_area}' as feagi_data_processing ({len(binary_data)} bytes)")
                return True
            else:
                logger.error("Failed to encode neuron data")
                return False
            
        except Exception as e:
            logger.error(f"Failed to send sensory data: {e}")
            return False

    def _encode_with_feagi_data_processing(
        self, cortical_area: str, neuron_data: Dict[Tuple[int, int, int], float]
    ) -> Optional[bytes]:
        """Encode neuron data using feagi_data_processing library."""
        try:
            if not neuron_data:
                return None
                
            # Convert to lists
            coords = list(neuron_data.keys())
            potentials = list(neuron_data.values())
            
            x_coords = [coord[0] for coord in coords]
            y_coords = [coord[1] for coord in coords]
            z_coords = [coord[2] for coord in coords]
            
            logger.info(f"Step 1: Converted to coordinate lists - {len(x_coords)} neurons")
            
            # Create NumPy arrays with proper dtypes
            neurons_x = np.asarray(x_coords, dtype=np.uint32)
            neurons_y = np.asarray(y_coords, dtype=np.uint32)
            neurons_z = np.asarray(z_coords, dtype=np.uint32)
            neurons_p = np.asarray(potentials, dtype=np.float32)
            
            logger.info(f"Step 2: Created numpy arrays - x={len(neurons_x)}, y={len(neurons_y)}, z={len(neurons_z)}, p={len(neurons_p)}")
            
            # Create cortical ID
            cortical_id_obj = fdp.cortical_data.CorticalID(str(cortical_area))
            logger.info(f"Step 3: Created cortical ID object")
            
            # Use NumPy approach
            neurons_array = fdp.neuron_data.neuron_arrays.NeuronXYZPArrays.new_from_numpy(
                neurons_x, neurons_y, neurons_z, neurons_p
            )
            logger.info(f"Step 4: Created neuron arrays object")
            
            # Create mapped neuron data container
            generated_mapped_neuron_data = fdp.neuron_data.neuron_mappings.CorticalMappedXYZPNeuronData()
            logger.info(f"Step 5: Created mapped neuron data container")
            
            generated_mapped_neuron_data.insert(cortical_id_obj, neurons_array)
            logger.info(f"Step 6: Inserted data into container")
            
            # Create the final byte structure from the mapped data
            byte_structure = generated_mapped_neuron_data.as_new_feagi_byte_structure()
            logger.info(f"Step 7: Created byte structure")
            
            # Get binary data - USE STANDARD METHOD - NO FALLBACKS
            binary_data = byte_structure.copy_as_bytes()
            logger.info(f"Step 8: Extracted binary data - {len(binary_data)} bytes")
            
            logger.info(f"📊 Agent encoded {len(neuron_data)} neurons into {len(binary_data)} bytes")
            
            logger.debug(f"Encoded {len(neuron_data)} neurons for '{cortical_area}' into {len(binary_data)} bytes using feagi_data_processing")
            return bytes(binary_data)
            
        except Exception as e:
            logger.error(f"Failed to encode with feagi_data_processing: {e}")
            import traceback
            logger.error(f"Encoding traceback: {traceback.format_exc()}")
            return None

    def close(self):
        """Close the sensory client connection."""
        try:
            if self.socket:
                self.socket.close()
            if self.context:
                self.context.term()
            logger.info("Sensory client closed")
        except Exception as e:
            logger.error(f"Error closing sensory client: {e}") 