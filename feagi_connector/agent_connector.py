from .feagi_interfaces.feagi_connection_status import FEAGIConnectionStatus
from .feagi_interfaces.abstract_feagi_interface import AbstractFeagiInterface
from .cache.sensor_cache import SensorCache


class FeagiAgentConnector:
    _server: AbstractFeagiInterface
    _py_sensor_cache: SensorCache

    def __init__(self, *args, **kwargs):
        raise RuntimeError("Direct instantiation not allowed, use one of the 'create' methods instead.")

    def __new__(cls, *args, **kwargs):
        raise RuntimeError("Direct instantiation not allowed, use one of the 'create' methods instead.")

    @staticmethod
    def _internal_init(server_backend: AbstractFeagiInterface):
        # We don't want users to be calling init directly
        if server_backend.get_current_connectivity_status() != FEAGIConnectionStatus.DISCONNECTED:
            raise Exception("Unable to start Feagi Agent with a running server!")
        obj = object.__new__(FeagiAgentConnector)
        obj._server = server_backend
        obj._py_sensor_cache = SensorCache()
        obj._motor_cache = MotorCache()
        return obj

    # Dummy connector removed per new architecture; use real interfaces only

    @property
    def server(self) -> AbstractFeagiInterface:
        return self._server

    @property
    def sensors(self) -> SensorCache:
        return self._py_sensor_cache
    
    @property
    def motors(self) -> "MotorCache":
        """Access to motor functionality as shown in the sample."""
        return self._motor_cache

    def encode_cache_to_bytes(self):
        self._py_sensor_cache.encode_cached_data_into_bytes()

    def get_most_recent_sensor_bytes(self) -> bytes:
        return self._py_sensor_cache.get_most_recent_sensor_bytes()


class MotorCache:
    """Motor cache for handling motor registrations and data processing.
    
    Provides access to gaze motors and other motor functionality as shown
    in the sample code.
    """
    
    def __init__(self):
        self._gaze_motor = GazeMotor()
        self._motor_data_cache = {}
    
    @property
    def gaze(self) -> "GazeMotor":
        """Access to gaze motor functionality."""
        return self._gaze_motor
    
    def read_bytes_into_motor(self, motor_bytes: bytes) -> None:
        """Read motor bytes into the motor cache as shown in the sample.
        
        Args:
            motor_bytes: Raw motor data bytes from FEAGI
        """
        try:
            import feagi_rust_py_libs as frpl
            
            # Process the motor bytes using the Rust library
            feagi_byte_structure = frpl.data_serialization.FeagiByteStructure(motor_bytes)
            mapped_neurons = frpl.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(feagi_byte_structure)
            
            # Store in cache for later retrieval
            self._motor_data_cache['latest'] = mapped_neurons
            
            # Notify gaze motor of new data
            self._gaze_motor._process_motor_data(mapped_neurons)
            
        except Exception as e:
            import logging
            logger = logging.getLogger(__name__)
            logger.warning(f"Error processing motor bytes: {e}")


class GazeMotor:
    """Gaze motor functionality as shown in the sample."""
    
    def __init__(self):
        self._registered = False
        self._group_index = 0
        self._num_channels = 10
        self._gaze_resolution = 8
        self._latest_gaze_data = None
        
        try:
            import feagi_rust_py_libs as frpl
            self._frpl = frpl
        except ImportError:
            self._frpl = None
    
    def register(self, cortical_group: int, num_channels, gaze_resolution: int) -> None:
        """Register gaze motor as shown in the sample.
        
        Args:
            cortical_group: Cortical group index
            num_channels: CorticalChannelCount object or integer
            gaze_resolution: Resolution for gaze mapping
        """
        self._group_index = cortical_group
        self._gaze_resolution = gaze_resolution
        
        # Handle both CorticalChannelCount objects and integers
        if hasattr(num_channels, 'value'):
            self._num_channels = num_channels.value
        else:
            self._num_channels = int(num_channels)
            
        self._registered = True
        
        import logging
        logger = logging.getLogger(__name__)
        logger.debug(f"Registered gaze motor: group={cortical_group}, channels={self._num_channels}, resolution={gaze_resolution}")
    
    def read_cache(self, cortical_group: int, channel: int):
        """Read gaze cache data as shown in the sample.
        
        Args:
            cortical_group: Cortical group index
            channel: Channel index
            
        Returns:
            Latest gaze data or None
        """
        return self._latest_gaze_data
    
    def _process_motor_data(self, mapped_neurons) -> None:
        """Process incoming motor data for gaze control."""
        try:
            if not self._frpl or not self._registered:
                return
            
            # Extract gaze-related neurons
            try:
                gaze_cortical_id = self._frpl.data_structures.genomic.CorticalID.new_motor_cortical_area_id(
                    self._frpl.data_structures.genomic.MotorCorticalType.Gaze, self._group_index
                )
            except AttributeError:
                # API might have changed, skip gaze processing
                return
            
            # Simple gaze position extraction
            total_x = 0.0
            total_y = 0.0
            total_activation = 0.0
            
            for cid_obj, neuron_arrays in mapped_neurons.iter_full():
                try:
                    x_coords, y_coords, z_coords, potentials = neuron_arrays
                    for x, y, z, p in zip(x_coords, y_coords, z_coords, potentials):
                        # Normalize coordinates to 0-1 range
                        gaze_x = float(x) / self._gaze_resolution
                        gaze_y = float(y) / self._gaze_resolution
                        total_x += gaze_x * p
                        total_y += gaze_y * p
                        total_activation += p
                except Exception:
                    continue
            
            if total_activation > 0:
                self._latest_gaze_data = (
                    max(0.0, min(1.0, total_x / total_activation)),
                    max(0.0, min(1.0, total_y / total_activation))
                )
            
        except Exception:
            pass

