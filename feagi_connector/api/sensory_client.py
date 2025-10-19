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
            # Bounded-send behavior: avoid indefinite blocks and silent drops
            # - IMMEDIATE: do not queue when no peer; fail fast to trigger reconnect
            # - SNDTIMEO: bounded blocking on backpressure (200 ms)
            try:
                self.socket.setsockopt(zmq.IMMEDIATE, 1)
            except Exception:
                pass
            try:
                self.socket.setsockopt(zmq.SNDTIMEO, 200)
            except Exception:
                pass
            
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
            
            # 🔍 DEBUG: Track what's being sent (for debugging self-stimulation)
            num_neurons = len(neuron_data)
            potentials = list(neuron_data.values())
            avg_potential = sum(potentials) / len(potentials) if potentials else 0.0
            unique_coords = len(set(neuron_data.keys()))
            
            # Log summary every 30 frames to avoid spam
            if not hasattr(self, '_frame_count'):
                self._frame_count = 0
                self._last_neuron_data = {}
            
            self._frame_count += 1
            
            # Check if we're re-sending the same data
            data_changed = neuron_data != self._last_neuron_data
            
            if self._frame_count % 30 == 0 or not data_changed:
                logger.info(f"🎥 [{cortical_area}] Frame {self._frame_count}: {num_neurons} neurons, "
                           f"avg_potential={avg_potential:.3f}, unique_coords={unique_coords}, "
                           f"data_changed={data_changed}")
                if not data_changed:
                    logger.warning(f"⚠️  [{cortical_area}] Sending IDENTICAL data as previous frame!")
            
            self._last_neuron_data = neuron_data.copy()
            
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
                
            logger.debug(f"FDP_INPUT: {cortical_area} -> {neuron_data}")
                
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
            
            logger.debug(f"FDP_ARRAYS: X={neurons_x.tolist()} Y={neurons_y.tolist()} Z={neurons_z.tolist()} P={neurons_p.tolist()}")
            
            logger.info(f"Step 2: Created numpy arrays - x={len(neurons_x)}, y={len(neurons_y)}, z={len(neurons_z)}, p={len(neurons_p)}")
            
            # Create cortical ID using the modern feagi-rust-py-libs approach
            cortical_area_str = str(cortical_area)
            
            try:
                # Try to create cortical ID directly from string - handles all modern format IDs
                cortical_id_obj = fdp.genome.CorticalID.try_new_from_string(cortical_area_str)
                logger.info(f"Step 3: Created cortical ID object from string: {cortical_area_str}")
            except ValueError as e:
                # Fallback for areas that can't be parsed directly
                logger.warning(f"Could not create cortical ID from '{cortical_area_str}': {e}")
                if len(cortical_area_str) == 6:
                    # If it's already 6 characters, it might be a custom area - try as is
                    try:
                        cortical_id_obj = fdp.genome.CorticalID.new_custom_cortical_area_id(cortical_area_str)
                        logger.info(f"Step 3b: Created as 6-char custom cortical ID: {cortical_area_str}")
                    except ValueError:
                        # If that fails too, something's wrong with this ID
                        logger.error(f"Could not create any cortical ID from '{cortical_area_str}' - invalid format")
                        return None
                else:
                    # For non-6-character areas, add 'c' prefix but ensure total length = 6
                    custom_name = f'c{cortical_area_str}'[:6]
                    cortical_id_obj = fdp.genome.CorticalID.new_custom_cortical_area_id(custom_name)
                    logger.info(f"Step 3c: Created as custom cortical ID: {custom_name}")
            
            # Use NumPy approach
            neurons_array = fdp.neuron_data.xyzp.NeuronXYZPArrays.new_from_numpy(
                neurons_x, neurons_y, neurons_z, neurons_p
            )
            logger.info(f"Step 4: Created neuron arrays object")
            
            # Create mapped neuron data container
            generated_mapped_neuron_data = fdp.neuron_data.xyzp.CorticalMappedXYZPNeuronData()
            logger.info(f"Step 5: Created mapped neuron data container")
            
            generated_mapped_neuron_data.insert(cortical_id_obj, neurons_array)
            logger.info(f"Step 6: Inserted data into container")
            
            # Create the final byte structure from the mapped data
            byte_structure = generated_mapped_neuron_data.as_new_feagi_byte_structure()
            logger.info(f"Step 7: Created byte structure")
            
            # Get binary data - USE STANDARD METHOD - NO FALLBACKS
            binary_data = byte_structure.copy_out_as_byte_vector()
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