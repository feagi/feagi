"""Tests for controller-facing BrainOutput helper APIs."""

import json
from types import SimpleNamespace

from feagi.pns.brain_output import BrainOutput


class _FakeCache:
    """Minimal cache stub for controller API tests."""

    def __init__(self) -> None:
        self.calls = []
        self._encoded = b"encoded"

    def sensor_Vision_register(self, group, count, frame_mode, image_props):
        self.calls.append(("Vision", group, count, frame_mode, image_props))

    def sensor_Proximity_register(self, group, count, frame_mode, z_res, positioning):
        self.calls.append(("Proximity", group, count, frame_mode, z_res, positioning))

    def sensor_Servo_register(self, group, count, frame_mode, z_res, positioning):
        self.calls.append(("Servo", group, count, frame_mode, z_res, positioning))

    def sensor_Shock_register(self, group, count, frame_mode, z_res, positioning):
        self.calls.append(("Shock", group, count, frame_mode, z_res, positioning))

    def sensor_MiscData_register(self, group, count, frame_mode, dims):
        self.calls.append(("MiscData", group, count, frame_mode, dims))

    def sensor_proximity_write(self, *, group, channel_index, data):
        self.calls.append(("write_proximity", group, channel_index, data))

    def sensor_servo_write(self, *, group, channel_index, data):
        self.calls.append(("write_servo", group, channel_index, data))

    def sensor_misc_data_write(self, *, group, channel_index, data):
        self.calls.append(("write_misc", group, channel_index, data))

    def sensors_encode_cached_sensor_data_to_bytes(self):
        self.calls.append(("encode",))

    def sensors_read_bytes(self):
        return self._encoded


class _FakeClient:
    """Minimal client stub for sensory send tests."""

    def __init__(self) -> None:
        self.sent = []

    def send_sensory_bytes(self, payload: bytes) -> None:
        self.sent.append(payload)


def _install_fake_frpl(monkeypatch):
    """Install a fake `feagi_rust_py_libs` module into sys.modules."""

    class _FrameChangeHandling:
        @staticmethod
        def Absolute():
            return "ABS"

        @staticmethod
        def Incremental():
            return "INC"

    class _Positioning:
        @staticmethod
        def Linear():
            return "LIN"

    class _Descriptors:
        class ImageXYResolution:
            def __init__(self, x, y):
                self.x = x
                self.y = y

        class ColorSpace:
            Gamma = "Gamma"

        class ColorChannelLayout:
            RGB = "RGB"

        class ImageFrameProperties:
            def __init__(self, resolution, color_space, channel_layout):
                self.resolution = resolution
                self.color_space = color_space
                self.channel_layout = channel_layout

        class MiscDataDimensions:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z

    fake = SimpleNamespace(
        data_structures=SimpleNamespace(
            genomic=SimpleNamespace(
                cortical_area=SimpleNamespace(
                    FrameChangeHandling=_FrameChangeHandling,
                    PercentageNeuronPositioning=_Positioning,
                )
            )
        ),
        connector_core=SimpleNamespace(
            data_types=SimpleNamespace(
                descriptors=_Descriptors,
            )
        ),
    )
    monkeypatch.setitem(__import__("sys").modules, "feagi_rust_py_libs", fake)


def test_register_sensor_units_deterministic_groups(monkeypatch):
    """Sensor units register through SDK without controller Rust imports."""
    _install_fake_frpl(monkeypatch)

    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._cache_available = True

    groups = bo.register_sensor_units(
        {
            "Shock": 2,
            "Servo": 1,
            "Proximity": 3,
        },
        z_neuron_resolution=10,
    )

    assert groups == {"Proximity": 0, "Servo": 1, "Shock": 2}
    assert ("Proximity", 0, 3, "ABS", 10, "LIN") in bo._cache.calls
    assert ("Servo", 1, 1, "ABS", 10, "LIN") in bo._cache.calls
    assert ("Shock", 2, 2, "ABS", 10, "LIN") in bo._cache.calls


def test_register_sensor_units_supports_incremental_frame_mode(monkeypatch):
    """Controllers can request incremental sensory cortical registrations."""
    _install_fake_frpl(monkeypatch)

    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._cache_available = True

    groups = bo.register_sensor_units(
        {"Servo": 1},
        z_neuron_resolution=10,
        group_index_start=9,
        frame_change_handling="incremental",
    )

    assert groups == {"Servo": 9}
    assert ("Servo", 9, 1, "INC", 10, "LIN") in bo._cache.calls


def test_write_and_flush_sensory_bytes_without_direct_rust_imports(monkeypatch):
    """Controller can write sensory scalars and flush using SDK wrappers."""
    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._cache_available = True
    bo._client = _FakeClient()

    class _PctFactory:
        @staticmethod
        def new_from_0_1(value):
            return ("pct", value)

    class _MiscFactory:
        @staticmethod
        def new_from_array(arr):
            return ("misc", float(arr[0, 0, 0]))

    def _fake_init_helpers():
        bo._sensory_percentage_factory = _PctFactory
        bo._sensory_misc_factory = _MiscFactory

    monkeypatch.setattr(bo, "_init_sensory_write_helpers", _fake_init_helpers)

    bo.write_sensor_scalar(unit_key="Proximity", group=0, channel_index=4, scalar_0_1=0.25)
    bo.write_sensor_scalar(unit_key="Servo", group=1, channel_index=0, scalar_0_1=0.5)
    bo.write_sensor_scalar(unit_key="MiscData", group=2, channel_index=1, scalar_0_1=0.75)
    sent_len = bo.flush_sensory_bytes()

    assert ("write_proximity", 0, 4, ("pct", 0.25)) in bo._cache.calls
    assert ("write_servo", 1, 0, ("pct", 0.5)) in bo._cache.calls
    assert ("write_misc", 2, 1, ("misc", 0.75)) in bo._cache.calls
    assert sent_len == len(b"encoded")
    assert bo._client.sent == [b"encoded"]


def test_receive_no_ops_when_sensory_only_mode():
    """MuJoCo sensors-only path sets _sensory_only_mode; receive must not touch motor ZMQ."""
    bo = BrainOutput()
    bo._connected = True
    bo._sensory_only_mode = True
    bo.receive()


def test_connect_uses_both_agent_type_with_motor_and_scalar_sensory(monkeypatch):
    """Motor + scalar sensory registrations should connect with BOTH agent type."""

    class _FakeAgentType:
        SENSORY = "sensory"
        BOTH = "both"
        MOTOR = "motor"

    class _FakeFeagiAgentClient:
        instances = []

        def __init__(self, _agent_id, agent_type):
            self.agent_type = agent_type
            self.configure_kwargs = None
            self.connected = False
            self.device_config_payload = None
            self.expected_cortical_ids = None
            _FakeFeagiAgentClient.instances.append(self)

        def configure(self, **kwargs):
            self.configure_kwargs = kwargs

        def connect(self):
            self.connected = True

        def set_motor_cortical_ids(self, _ids):
            return None

        def send_device_configuration(self, payload, expected_cortical_ids):
            self.device_config_payload = payload
            self.expected_cortical_ids = expected_cortical_ids

    fake_client_module = SimpleNamespace(
        AgentType=_FakeAgentType,
        FeagiAgentClient=_FakeFeagiAgentClient,
    )
    monkeypatch.setitem(__import__("sys").modules, "feagi.pns.client", fake_client_module)

    class _FakeConnectCache:
        def get_sensory_cortical_ids_for_verification(self):
            return ["sensory-id-1"]

        def export_capabilities_json(self):
            return json.dumps({})

    bo = BrainOutput()
    bo._cache = _FakeConnectCache()
    bo._cache_available = True
    bo._motor_total_channels = 1
    bo._motor_decoder_registered = True
    bo._agent_id = "test-agent"
    bo._feagi_host = "127.0.0.1"
    bo._feagi_registration_port = 8000
    bo._feagi_sensory_port = 8001
    bo._feagi_motor_port = 8002
    bo._transport_type = "zmq"
    bo._feagi_connection_timeout_ms = 1000
    bo._feagi_registration_retries = 1
    bo._feagi_heartbeat_interval_s = 1.0
    bo._feagi_api_port = 8003
    bo._feagi_http_timeout_s = 1.0
    bo._auth_token_b64 = "dGVzdA=="
    bo._vision_units = []
    bo._collect_motor_unit_specs = lambda: [("rotary_motor", 0)]  # type: ignore[method-assign]
    bo._collect_motor_cortical_ids = lambda: ["motor-id-1"]  # type: ignore[method-assign]
    bo._normalize_device_registration_properties = lambda payload: payload  # type: ignore[method-assign]
    bo._validate_device_registration_contract = lambda payload: payload  # type: ignore[method-assign]

    bo.connect()

    client = _FakeFeagiAgentClient.instances[-1]
    assert client.connected is True
    assert client.agent_type == _FakeAgentType.BOTH


def test_collect_motor_cortical_ids_empty_when_no_motor_outputs_registered():
    """
    Sensory-only agents must not get synthetic PositionalServo IDs; those inflated
    ``output_count`` and forced the motor registration path (ROS SmartIMU-only).
    """
    bo = BrainOutput()
    assert bo._outputs == []
    assert bo._motor_total_channels == 0
    assert bo._collect_motor_cortical_ids() == []
