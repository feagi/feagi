"""Tests for the typed-BrainOutput BluetoothRobot and shared FEAGI args."""

import argparse
import asyncio
import base64

import numpy as np
import pytest

from feagi.agent.args import add_feagi_connection_args
from feagi.agent.bluetooth import BluetoothRobot
from feagi.pns.brain_output import BrainOutput

BLE = {
    "device_name": "dev",
    "service_uuid": "svc",
    "rx_uuid": "rx",
    "tx_uuid": "tx",
}
FULL_ARGV = [
    "--ip", "10.0.0.2",
    "--port", "8000",
    "--feagi-zmq-motor-port", "30010",
    "--feagi-zmq-registration-port", "30011",
    "--feagi-zmq-sensory-port", "30012",
    "--feagi-zmq-connection-timeout-ms", "5000",
    "--feagi-zmq-registration-retries", "3",
    "--feagi-zmq-heartbeat-interval-s", "1.5",
    "--feagi-http-timeout-s", "2.5",
    "--feagi-burst-period-s", "0.05",
    "--agent_id", "AGENT",
]


class FakeTransport:
    """Async transport double: queued inbound chunks, recorded outbound."""

    def __init__(self, inbound=()):
        self.inbound = list(inbound)
        self.sent = []

    async def send(self, data):
        self.sent.append(data)

    async def receive(self):
        return self.inbound.pop(0) if self.inbound else None


class DemoRobot(BluetoothRobot):
    BLUETOOTH_CONFIG = BLE
    Z_NEURON_RESOLUTION = 10

    def __init__(self, *a, **kw):
        super().__init__(*a, platform="desktop", **kw)
        self.format_calls = []

    def sensor_units(self):
        return {"RawIMU": 1}

    def motor_groups(self):
        return {0: {"positional_servo": ["a", "b"]}}

    def misc_motor_groups(self):
        return {1: (4, 1, 2)}

    def parse_sensors(self, raw):
        return {"gyroscope": (0.1, -0.2, 0.3)} if raw == b"G" else {}

    def format_motors(self, servo_targets, misc_values):
        self.format_calls.append((servo_targets, misc_values))
        return b"cmd"


def _real_output() -> BrainOutput:
    """BrainOutput backed by the real Rust cache (no FEAGI connection)."""
    out = BrainOutput()
    out._agent_id = base64.b64encode(bytes(range(48))).decode()
    out._init_cache()
    return out


def _robot(transport, frame=True):
    out = _real_output()
    robot = DemoRobot("r", burst_period_s=0.05, brain_output=out)
    robot.transport = transport
    robot.register_devices()
    out.flushes = 0

    def fake_receive():
        # Stand-in for the ZMQ pull; the cache itself is real.
        out._last_motor_frame = b"x" if frame else None

    def fake_flush():
        out.flushes += 1
        return 0

    out.receive = fake_receive
    out.flush_sensory_bytes = fake_flush
    return robot, out


def test_misc_motor_register_and_read_real_cache():
    out = _real_output()
    out.register_motor_misc_data(group=1, width=4, height=1, depth=2)
    volume = out.read_misc_data(1, 0)
    assert volume.shape == (4, 1, 2)
    assert not volume.any()
    with pytest.raises(ValueError):
        out.read_misc_data(7, 0)


def test_servo_targets_reach_format_motors_real_cache():
    transport = FakeTransport()
    robot, out = _robot(transport)
    # Real cache write stands in for a decoded FEAGI frame.
    out._cache.motor_positional_servo_write_preprocessed_cache_value(
        0, 1, 0.8
    )
    asyncio.run(robot.tick())
    servo_targets, misc_values = robot.format_calls[0]
    assert set(servo_targets) == {0}
    assert servo_targets[0][1] == pytest.approx(0.8, abs=0.02)
    assert servo_targets[0][0] == pytest.approx(0.5, abs=0.02)
    assert misc_values == {1: []}
    assert transport.sent == [b"cmd"]


def test_no_format_motors_before_first_motor_frame():
    robot, _ = _robot(FakeTransport(), frame=False)
    asyncio.run(robot.tick())
    assert robot.format_calls == []
    assert robot.transport.sent == []


def test_misc_active_voxels_passed_to_format_motors():
    # PARTIAL FAKE: the real cache has no motor-side misc write and a real
    # frame needs a live FEAGI encoder, so the decoded volume is injected.
    robot, out = _robot(FakeTransport())
    volume = np.zeros((4, 1, 2), dtype=np.float32)
    volume[3, 0, 1] = 1.0
    out.read_misc_data = lambda group, channel=0: volume
    asyncio.run(robot.tick())
    assert robot.format_calls[0][1] == {1: [(3, 0, 1)]}


def test_gyro_notification_writes_gyroscope_once_real_cache():
    robot, out = _robot(FakeTransport([b"G"]), frame=False)
    writes = []
    real_write = out.write_sensor_raw_imu_gyroscope

    def spy(**kwargs):
        writes.append(kwargs)
        real_write(**kwargs)

    out.write_sensor_raw_imu_gyroscope = spy
    asyncio.run(robot.tick())
    assert len(writes) == 1
    assert writes[0]["gyroscope_xyz"] == (0.1, -0.2, 0.3)
    assert writes[0]["channel_index"] == 0
    assert out.flushes == 1


def test_args_helper_parses_full_list_and_env_default(monkeypatch):
    monkeypatch.setenv("FEAGI_AUTH_TOKEN_B64", "ENVTOKEN")
    parser = argparse.ArgumentParser()
    add_feagi_connection_args(parser)
    args = parser.parse_args(FULL_ARGV)
    assert args.ip == "10.0.0.2"
    assert args.port == 8000
    assert args.feagi_zmq_motor_port == 30010
    assert args.feagi_zmq_registration_port == 30011
    assert args.feagi_zmq_sensory_port == 30012
    assert args.feagi_zmq_connection_timeout_ms == 5000
    assert args.feagi_zmq_registration_retries == 3
    assert args.feagi_zmq_heartbeat_interval_s == 1.5
    assert args.feagi_http_timeout_s == 2.5
    assert args.feagi_burst_period_s == 0.05
    assert args.agent_id == "AGENT"
    assert args.auth_token_b64 == "ENVTOKEN"
    explicit = parser.parse_args(FULL_ARGV + ["--auth-token-b64", "ARG"])
    assert explicit.auth_token_b64 == "ARG"


def test_configure_from_args_forwards_and_requires_token(monkeypatch):
    monkeypatch.delenv("FEAGI_AUTH_TOKEN_B64", raising=False)
    parser = argparse.ArgumentParser()
    add_feagi_connection_args(parser)
    out = BrainOutput()
    seen = {}
    out.configure = lambda **kw: seen.update(kw)
    with pytest.raises(ValueError):
        out.configure_from_args(parser.parse_args(FULL_ARGV))
    out.configure_from_args(
        parser.parse_args(FULL_ARGV + ["--auth-token-b64", "T"])
    )
    assert seen["feagi_host"] == "10.0.0.2"
    assert seen["feagi_api_port"] == 8000
    assert seen["feagi_motor_port"] == 30010
    assert seen["auth_token_b64"] == "T"
    assert seen["transport"] == "zmq"


def test_bluetooth_args_override_class_fallback():
    parser = argparse.ArgumentParser()
    DemoRobot.add_arguments(parser)
    args = parser.parse_args(
        FULL_ARGV + ["--auth-token-b64", "T", "--bt-device-name", "other"]
    )
    robot = DemoRobot.from_args(args, brain_output=BrainOutput())
    assert robot.bluetooth_config["device_name"] == "other"
    assert robot.bluetooth_config["rx_uuid"] == "rx"
    assert robot.burst_period_s == 0.05
