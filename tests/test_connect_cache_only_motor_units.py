"""connect() must register every motor unit held by the Rust ConnectorAgent cache."""

import base64

import pytest

from feagi.pns import client as client_module
from feagi.pns.brain_output import BrainOutput


class _RecordingClient:
    """Stands in for FeagiAgentClient: records configure kwargs, no network."""

    instances: list = []

    def __init__(self, agent_id, agent_type):
        self.configure_kwargs = None
        self.device_config = None
        _RecordingClient.instances.append(self)

    def configure(self, **kwargs):
        self.configure_kwargs = kwargs

    def connect(self):
        return True

    def set_motor_cortical_ids(self, ids):
        pass

    def send_device_configuration(self, payload, expected_cortical_ids=None):
        self.device_config = payload


def _configured_output() -> BrainOutput:
    out = BrainOutput()
    out._agent_id = base64.b64encode(bytes(range(48))).decode()
    out._init_cache()
    out.configure(
        out._agent_id,
        feagi_host="127.0.0.1",
        feagi_registration_port=30011,
        feagi_sensory_port=30012,
        feagi_motor_port=30010,
        transport="zmq",
        feagi_connection_timeout_ms=1000,
        feagi_registration_retries=1,
        feagi_heartbeat_interval_s=1.0,
        feagi_api_port=8000,
        feagi_http_timeout_s=1.0,
        auth_token_b64=base64.b64encode(bytes(32)).decode(),
    )
    return out


@pytest.fixture(autouse=True)
def _fake_client(monkeypatch):
    _RecordingClient.instances.clear()
    monkeypatch.setattr(client_module, "FeagiAgentClient", _RecordingClient)
    yield
    _RecordingClient.instances.clear()


def test_cache_only_motors_reach_registration_payload():
    out = _configured_output()
    out.register_motor_groups(
        {0: {"positional_servo": ["a", "b"]}}, z_neuron_resolution=10
    )
    out.register_motor_misc_data(group=1, width=4, height=1, depth=2)
    assert out._outputs == []

    out.connect()  # no RuntimeError

    modality, output_count, units = _RecordingClient.instances[0].configure_kwargs[
        "motor_units"
    ]
    assert modality == "motor"
    assert units == [("misc_data", 1), ("positional_servo", 0)]
    assert output_count == 2  # largest unit has 2 channels, not the area count
    assert _RecordingClient.instances[0].device_config is not None


def test_python_servo_objects_and_cache_do_not_duplicate_specs():
    from feagi.pns.outputs.motor import ServoMotor

    out = _configured_output()
    servo = ServoMotor(range=(0.0, 180.0), encoding="absolute")
    servo.group_id = 0
    servo.channel = 1
    out._outputs.append(servo)
    out.register_motor_groups(
        {0: {"positional_servo": ["a", "b"]}}, z_neuron_resolution=10
    )
    specs = out._collect_motor_unit_specs()
    assert specs == [("positional_servo", 0)]
    assert out._motor_feagi_output_count() == 2


def test_unit_names_outside_registration_contract_are_skipped():
    out = _configured_output()
    out.register_motor_groups({0: {"rotary_motor": ["r"]}}, z_neuron_resolution=10)
    out.register_motor_spatial_pointer(group=3, width=3, height=1, depth=4)
    assert out._collect_motor_unit_specs() == [("rotary_motor", 0)]
