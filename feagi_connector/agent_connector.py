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

    @staticmethod
    def create_dummy_connector():
        """Create a dummy connector for testing and examples without actual server connection.
        
        Returns:
            FeagiAgentConnector: A connector instance with dummy server backend
        """
        from .feagi_interfaces.abstract_feagi_interface import AbstractFeagiInterface
        
        class DummyFeagiInterface(AbstractFeagiInterface):
            """Minimal dummy interface for testing."""
            def __init__(self):
                super().__init__()
                self._status = FEAGIConnectionStatus.DISCONNECTED
            
            async def disconnect(self) -> None:
                pass
            
            async def _send_bytes(self, sending: bytes) -> None:
                pass
            
            async def connect(self) -> bool:
                """Dummy connect that just returns True."""
                self._status = FEAGIConnectionStatus.FEAGI_READY
                return True
        
        dummy_server = DummyFeagiInterface()
        return FeagiAgentConnector._internal_init(dummy_server)

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
    
    def sensor_get_byte_container(self):
        """Get the Rust FeagiByteContainer with encoded sensor data.
        
        Returns the underlying Rust byte container which provides access to:
        - number_contained_structures: Count of encoded structures
        - try_create_new_struct_from_index(index): Extract individual neuron data structures
        
        Example usage (from segmented_autogaze.py):
            byte_container = agent.sensor_get_byte_container()
            number_contained_structs = byte_container.number_contained_structures
            neuron_data = byte_container.try_create_new_struct_from_index(0)
        """
        if self._py_sensor_cache._rust_cache:
            try:
                # IOCache API: sensor_get_bytes() returns bytes, wrap in FeagiByteContainer
                if hasattr(self._py_sensor_cache._rust_cache, 'sensor_get_bytes'):
                    import feagi_rust_py_libs as frpl
                    raw_bytes = self._py_sensor_cache._rust_cache.sensor_get_bytes()
                    if raw_bytes:
                        container = frpl.data_serialization.FeagiByteContainer()
                        container.load_bytes_and_verify(raw_bytes)
                        return container
                return None
            except Exception as e:
                import logging
                logger = logging.getLogger(__name__)
                logger.debug(f"Byte container access failed: {e}")
                return None
        return None


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
            # Try new API first (FeagiByteContainer), skip if not available
            if not hasattr(frpl.data_serialization, 'FeagiByteContainer'):
                logger.warning("FeagiByteContainer not available in feagi-rust-py-libs, skipping motor decode")
                return
            if not motor_bytes or len(motor_bytes) == 0:
                return
            try:
                feagi_byte_structure = frpl.data_serialization.FeagiByteContainer()
                feagi_byte_structure.load_bytes_and_verify(motor_bytes)
                mapped_neurons = feagi_byte_structure.try_create_new_struct_from_index(0)
            except (KeyboardInterrupt, SystemExit):
                raise
            except:
                import sys
                logger.debug(f"Failed to decode motor bytes: {sys.exc_info()[1]}")
                return
            
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
        self._number_z_neurons = 10
        self._latest_gaze_data = None
        self._latest_percentage_4d = None
        
        try:
            import feagi_rust_py_libs as frpl
            self._frpl = frpl
        except ImportError:
            self._frpl = None
    
    def register(self, cortical_group: int, num_channels: int, number_z_neurons: int) -> None:
        """Register gaze motor as shown in the sample.
        
        Args:
            cortical_group: Cortical group index
            num_channels: Number of motor channels
            number_z_neurons: Number of Z-dimension neurons
        """
        self._group_index = cortical_group
        self._num_channels = int(num_channels)
        self._number_z_neurons = int(number_z_neurons)
        self._registered = True
        
        import logging
        logger = logging.getLogger(__name__)
        logger.debug(f"Registered gaze motor: group={cortical_group}, channels={self._num_channels}, z_neurons={number_z_neurons}")
    
    def read_cache(self, cortical_group: int, channel: int):
        """Read gaze cache data as shown in the sample.
        
        Args:
            cortical_group: Cortical group index
            channel: Channel index
            
        Returns:
            Latest gaze data or None
        """
        return self._latest_gaze_data
    
    def read_post_processed(self, cortical_group: int, device_channel: int):
        """Read post-processed gaze data as Percentage4D object.
        
        This method returns the gaze data in the format used by the sample code,
        where the Percentage4D object contains:
        - a, b: eccentricity (x, y)
        - c, d: modularity (x, y)
        
        Args:
            cortical_group: Cortical group index
            device_channel: Device channel index
            
        Returns:
            Percentage4D object or a default centered gaze if no data available
        """
        if not self._frpl:
            raise ImportError("feagi_rust_py_libs not available")
        
        # Return cached processed data or create default
        if self._latest_percentage_4d is not None:
            return self._latest_percentage_4d
        
        # Create default centered gaze (0.5, 0.5, 0.5, 0.5)
        try:
            return self._frpl.connector_core.data.Percentage4D(
                self._frpl.connector_core.data.Percentage.new_from_0_1(0.5),
                self._frpl.connector_core.data.Percentage.new_from_0_1(0.5),
                self._frpl.connector_core.data.Percentage.new_from_0_1(0.5),
                self._frpl.connector_core.data.Percentage.new_from_0_1(0.5),
            )
        except AttributeError:
            # Fallback if API path is different
            return None
    
    def _process_motor_data(self, mapped_neurons) -> None:
        """Process incoming motor data for gaze control."""
        try:
            if not self._frpl or not self._registered:
                return
            
            # Extract gaze-related neurons and process into Percentage4D
            # This extracts 4 values: eccentricity (x,y) and modularity (x,y)
            total_x = 0.0
            total_y = 0.0
            total_z = 0.0
            total_w = 0.0
            total_activation = 0.0
            
            for cid_obj, neuron_arrays in mapped_neurons.iter_full():
                try:
                    x_coords, y_coords, z_coords, potentials = neuron_arrays
                    for x, y, z, p in zip(x_coords, y_coords, z_coords, potentials):
                        # Map different channels to different parameters
                        # Channel 0-1: eccentricity x,y
                        # Channel 2-3: modularity x,y
                        if int(x) == 0:
                            total_x += float(z) / self._number_z_neurons * p
                        elif int(x) == 1:
                            total_y += float(z) / self._number_z_neurons * p
                        elif int(x) == 2:
                            total_z += float(z) / self._number_z_neurons * p
                        elif int(x) == 3:
                            total_w += float(z) / self._number_z_neurons * p
                        total_activation += p
                except Exception:
                    continue
            
            if total_activation > 0:
                # Normalize and create Percentage4D
                ecc_x = max(0.0, min(1.0, total_x / total_activation))
                ecc_y = max(0.0, min(1.0, total_y / total_activation))
                mod_x = max(0.0, min(1.0, total_z / total_activation))
                mod_y = max(0.0, min(1.0, total_w / total_activation))
                
                try:
                    self._latest_percentage_4d = self._frpl.connector_core.data.Percentage4D(
                        self._frpl.connector_core.data.Percentage.new_from_0_1(ecc_x),
                        self._frpl.connector_core.data.Percentage.new_from_0_1(ecc_y),
                        self._frpl.connector_core.data.Percentage.new_from_0_1(mod_x),
                        self._frpl.connector_core.data.Percentage.new_from_0_1(mod_y),
                    )
                except AttributeError:
                    # API path might be different
                    pass
                
                # Also store simple gaze data
                self._latest_gaze_data = (ecc_x, ecc_y, mod_x, mod_y)
            
        except Exception:
            pass

