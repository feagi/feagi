"""Tests for controller-facing BrainOutput helper APIs."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from feagi.pns.brain_output import BrainOutput


class _FakeCache:
    """Minimal cache stub for controller API tests."""

    def __init__(self) -> None:
        self.calls = []
        self._encoded = b"encoded"
        self.input_units: dict = {}

    def sensor_Vision_register(self, group, count, frame_mode, image_props):
        self.calls.append(("Vision", group, count, frame_mode, image_props))

    def sensor_vision_register(self, *, group, number_channels, frame_change_handling, image_properties):
        self.calls.append(
            ("simple_vision", group, number_channels, frame_change_handling, image_properties)
        )

    def sensor_vision_write(self, *, group, channel_index, data):
        self.calls.append(("write_simple_vision", group, channel_index, data))

    def sensor_segmented_vision_write(self, *, group, channel_index, data):
        self.calls.append(("write_segmented_vision", group, channel_index, data))

    def export_capabilities_json(self):
        return json.dumps(
            {
                "input_units_and_encoder_properties": dict(self.input_units),
                "output_units_and_decoder_properties": {},
            }
        )

    def import_capabilities_json(self, payload: str):
        parsed = json.loads(payload)
        inputs = parsed.get("input_units_and_encoder_properties") or {}
        self.input_units = dict(inputs)
        self.calls.append(("import_capabilities", sorted(self.input_units.keys())))

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
    assert 0 not in bo._vision_group_modes
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


def test_receive_persists_unmapped_motor_channels_in_snapshot():
    """Canonical _motor_data keeps decoded channels even without registered outputs.

    Models the Rust decode flow: receive() pulls raw bytes, decodes them through
    the Rust cache, then builds _motor_data from the cache's flat snapshot. The
    snapshot here returns three SpatialPointer axes (no registered Python output),
    which must still land in _motor_data.
    """

    class _FakeClient:
        def __init__(self):
            self.calls = []

        def receive_motor_data_raw(self):
            return b"raw-motor-bytes"

    class _FakeCache:
        def __init__(self):
            self.loaded = None
            self.decoded = False
            self.updated_only_calls = 0

        def motors_load_in_bytes_and_verify(self, raw):
            self.loaded = raw

        def motors_decode_cached_byte_data_to_motor(self):
            self.decoded = True

        def motors_read_decoded_snapshot(self, updated_only=False):
            self.updated_only_calls += int(bool(updated_only))
            # (group, channel, mode, value) tuples, as the Rust binding returns.
            return [
                (1, 0, "absolute", 0.9),
                (1, 1, "absolute", 0.2),
                (1, 2, "absolute", 0.0),
            ]

    bo = BrainOutput()
    bo._connected = True
    bo._sensory_only_mode = False
    bo._client = _FakeClient()
    bo._cache = _FakeCache()
    bo._motor_outputs_by_group_channel = {}
    bo._motor_outputs_by_channel = {}

    bo.receive()

    # Raw bytes were handed to the Rust decoder (not Python-decoded).
    assert bo._cache.loaded == b"raw-motor-bytes"
    assert bo._cache.decoded is True
    assert bo._cache.updated_only_calls == 1
    # Snapshot channels are persisted even without registered output objects.
    assert bo._motor_data["1:0:absolute"] == 0.9
    assert bo._motor_data["1:1:absolute"] == 0.2
    assert bo._motor_data["1:2:absolute"] == 0.0


def test_receive_clears_motor_data_when_no_raw_bytes():
    """Silent motor ticks must clear _motor_data so absolute commands do not latch."""

    class _FakeClient:
        def receive_motor_data_raw(self):
            return b""

    bo = BrainOutput()
    bo._connected = True
    bo._sensory_only_mode = False
    bo._client = _FakeClient()
    bo._cache = object()
    bo._motor_data = {"1:0:absolute": 1.0}

    bo.receive()

    assert bo._motor_data == {}


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


def _install_vision_write_helpers(brain_output, monkeypatch):
    class _ImageFrameFactory:
        @staticmethod
        def new_from_array(frame_array, _color_space, _memory_layout):
            return ("frame", frame_array.shape)

    def _fake_init_helpers():
        brain_output._vision_image_frame_factory = _ImageFrameFactory
        brain_output._vision_color_space = "Gamma"
        brain_output._vision_memory_layout = "HWC"

    monkeypatch.setattr(brain_output, "_init_vision_write_helpers", _fake_init_helpers)


def test_register_simple_vision_groups_records_simple_write_mode(monkeypatch):
    """Simple-vision registration must be remembered so writes do not use SegmentedVision."""
    _install_fake_frpl(monkeypatch)
    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._cache_available = True

    groups = bo.register_simple_vision_groups(
        [("camera", 128, 128, 3, "vision", 4)]
    )

    assert groups == [4]
    assert bo._vision_group_modes[4] == "simple"
    assert any(call[0] == "simple_vision" and call[1] == 4 for call in bo._cache.calls)


def test_write_sensor_vision_frame_routes_simple_when_segmented_method_exists(
    monkeypatch,
):
    """Mixed simple+segmented caches must not send simple frames through segmented write."""
    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._cache_available = True
    bo._vision_group_modes[0] = "simple"
    bo._vision_group_modes[1] = "segmented"
    _install_vision_write_helpers(bo, monkeypatch)

    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    bo.write_sensor_vision_frame(group=0, channel_index=0, frame_rgb=frame)
    bo.write_sensor_vision_frame(group=1, channel_index=0, frame_rgb=frame)

    assert ("write_simple_vision", 0, 0, ("frame", (2, 2, 3))) in bo._cache.calls
    assert ("write_segmented_vision", 1, 0, ("frame", (2, 2, 3))) in bo._cache.calls
    assert not any(
        call[0] == "write_segmented_vision" and call[1] == 0 for call in bo._cache.calls
    )


def test_write_sensor_vision_frame_rejects_unregistered_group():
    """Vision writes require an explicit simple or segmented registration for the group."""
    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._cache_available = True
    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    with pytest.raises(RuntimeError, match="has not been registered"):
        bo.write_sensor_vision_frame(group=7, channel_index=0, frame_rgb=frame)


def test_drop_cached_vision_units_removes_segmented_and_simple_from_export():
    """Switching cameras must drop stale Vision/SegmentedVision cache entries."""
    bo = BrainOutput()
    cache = _FakeCache()
    cache.input_units = {
        "Vision": [{"cortical_unit_index": 0}],
        "SegmentedVision": [{"cortical_unit_index": 1}],
        "Proximity": [{"cortical_unit_index": 2}],
    }
    bo._cache = cache
    bo._cache_available = True
    bo._vision_group_modes = {0: "simple", 1: "segmented"}

    bo.drop_cached_vision_units()

    assert "Vision" not in cache.input_units
    assert "SegmentedVision" not in cache.input_units
    assert "Proximity" in cache.input_units
    assert bo._vision_group_modes == {}
    assert ("import_capabilities", ["Proximity"]) in cache.calls


def test_readvertise_device_registrations_noops_when_disconnected():
    """Capability push is only valid after connect()."""
    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._connected = False
    bo._client = None
    bo.readvertise_device_registrations()


def test_collect_motor_cortical_ids_empty_when_no_motor_outputs_registered():
    """
    Sensory-only agents must not get synthetic PositionalServo IDs; those inflated
    ``output_count`` and forced the motor registration path (ROS SmartIMU-only).
    """
    bo = BrainOutput()
    assert bo._outputs == []
    assert bo._motor_total_channels == 0
    assert bo._collect_motor_cortical_ids() == []
