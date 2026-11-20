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

    state = {"calls": 0}

    def fake_receive():
        # Stand-in for the ZMQ pull; the cache itself is real. One queued
        # frame per tick (tick drains until the queue is empty).
        state["calls"] += 1
        out._last_motor_frame = b"x" if frame and state["calls"] % 2 else None

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
    # Explicit opt-out: "held" keeps the cached-target contract.
    transport = FakeTransport()
    robot, out = _robot(transport)
    robot.SERVO_TARGETS_MODE = "held"
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


def _fire_misc(out, voxel=(3, 0, 1)):
    volume = np.zeros((4, 1, 2), dtype=np.float32)
    volume[voxel] = 1.0
    out._misc_frame_volumes = {(1, 0): volume}


def test_misc_fire_gives_voxel_then_silent_frame_gives_empty():
    # PARTIAL FAKE: a real frame needs a live FEAGI encoder, so the per-frame
    # fired volume is injected where receive() would capture it.
    robot, out = _robot(FakeTransport())
    _fire_misc(out)
    asyncio.run(robot.tick())
    out._misc_frame_volumes = {}  # next frame: misc area silent
    asyncio.run(robot.tick())
    assert robot.format_calls[0][1] == {1: [(3, 0, 1)]}
    assert robot.format_calls[1][1] == {1: []}


def test_misc_group_absent_without_a_frame():
    robot, out = _robot(FakeTransport())
    asyncio.run(robot.tick())  # frame: first motor frame seen
    out.receive = lambda: setattr(out, "_last_motor_frame", None)
    asyncio.run(robot.tick())  # no frame this tick
    assert robot.format_calls[1][1] == {}


def test_read_misc_data_this_frame_zero_when_not_fired_real_cache():
    out = _real_output()
    out.register_motor_misc_data(group=1, width=4, height=1, depth=2)
    volume = out.read_misc_data_this_frame(1, 0)
    assert volume.shape == (4, 1, 2)
    assert not volume.any()
    assert out._cache.motor_channel_updated_last_decode("misc_data", 1, 0) is False
    with pytest.raises(ValueError):
        out.read_misc_data_this_frame(7, 0)


def test_capture_misc_frame_follows_the_decode_flag():
    out = _real_output()
    out.register_motor_misc_data(group=1, width=4, height=1, depth=2)
    fired = np.zeros((4, 1, 2), dtype=np.float32)
    fired[2, 0, 0] = 1.0

    class FlagCache:
        flag = True

        def motor_channel_updated_last_decode(self, unit, group, channel):
            assert unit == "misc_data"
            return self.flag

    out._cache = FlagCache()
    out.read_misc_data = lambda group, channel=0: fired
    out._capture_misc_frame()
    assert out.read_misc_data_this_frame(1, 0)[2, 0, 0] == 1.0
    FlagCache.flag = False
    out._capture_misc_frame()
    assert not out.read_misc_data_this_frame(1, 0).any()


def test_drain_unions_misc_voxels_across_frames():
    out = _real_output()
    out.register_motor_misc_data(group=1, width=4, height=1, depth=2)
    frames = [(3, 0, 1), None, None]  # event only in a non-last frame

    def fake_receive():
        voxel = frames.pop(0)
        if voxel is None:
            out._last_motor_frame = None if not frames else b"x"
            out._misc_frame_volumes = {}
            return
        out._last_motor_frame = b"x"
        volume = np.zeros((4, 1, 2), dtype=np.float32)
        volume[voxel] = 1.0
        out._misc_frame_volumes = {(1, 0): volume}

    out.receive = fake_receive
    assert out.drain_motor_frames() == 2
    assert out.read_misc_data_this_frame(1, 0)[3, 0, 1] == 1.0


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


# Event-based servo targets (default "tick"), driven by REAL encoded frames
# decoded through the real Rust cache.

def _servo_frame(out, channels, z=5):
    """Encode a real motor frame firing the given positional-servo channels."""
    import feagi_rust_py_libs as frpl

    x = frpl.data_structures.neurons_voxels.xyzp
    cid_cls = frpl.data_structures.genomic.cortical_area.CorticalID
    # Servo-position area of group 0 (flag byte 0x01, group byte 0).
    cid = cid_cls.try_from_bytes(b"opse\x01\x00\x00\x00")
    arrays = x.PyNeuronVoxelXYZPArrays.new_from_numpy(
        np.array(channels, dtype=np.uint32),
        np.zeros(len(channels), dtype=np.uint32),
        np.full(len(channels), z, dtype=np.uint32),
        np.ones(len(channels), dtype=np.float32),
    )
    mapped = x.CorticalMappedXYZPNeuronVoxels.new_with_capacity(1)
    mapped.insert(cid, arrays)
    container = frpl.data_serialization.FeagiByteContainer()
    container.add_struct(mapped, True)
    return bytes(container.copy_out_as_byte_vector())


class _FrameClient:
    def __init__(self):
        self.frames = []

    def receive_motor_data_raw(self):
        return self.frames.pop(0) if self.frames else b""


def _real_frame_robot():
    out = _real_output()
    robot = DemoRobot("r", burst_period_s=0.05, brain_output=out)
    robot.transport = FakeTransport()
    robot.register_devices()
    out._connected = True
    out._sensory_only_mode = False
    out._client = _FrameClient()
    out.flush_sensory_bytes = lambda: 0
    return robot, out


def test_tick_mode_is_the_default():
    assert BluetoothRobot.SERVO_TARGETS_MODE == "tick"


def test_tick_servo_targets_follow_real_frames():
    robot, out = _real_frame_robot()
    # One frame per tick (queue holds one frame at each tick).
    for channels in ([1], [1], [0]):
        out._client.frames = [_servo_frame(out, channels)]
        asyncio.run(robot.tick())
    first, second, third = (c[0][0] for c in robot.format_calls)
    assert set(first) == {1}
    assert first[1] == pytest.approx(4 / 9, abs=0.02)
    assert set(second) == {1}
    assert set(third) == {0}  # channel 1 absent: it did not fire


def _misc_frame(out, voxel_x, z=1, servo_channels=None, servo_z=5):
    """Encode a real frame firing a misc voxel (group 1) and optional servos."""
    import feagi_rust_py_libs as frpl

    x = frpl.data_structures.neurons_voxels.xyzp
    cid_cls = frpl.data_structures.genomic.cortical_area.CorticalID
    mapped = x.CorticalMappedXYZPNeuronVoxels.new_with_capacity(2)

    def arr(xs, zs):
        n = len(xs)
        return x.PyNeuronVoxelXYZPArrays.new_from_numpy(
            np.array(xs, dtype=np.uint32),
            np.zeros(n, dtype=np.uint32),
            np.array(zs, dtype=np.uint32),
            np.ones(n, dtype=np.float32),
        )

    if voxel_x is not None:
        mapped.insert(
            cid_cls.try_from_bytes(b"omis\x0a\x00\x01\x00"),
            arr([voxel_x], [z]),
        )
    if servo_channels:
        mapped.insert(
            cid_cls.try_from_bytes(b"opse\x01\x00\x00\x00"),
            arr(servo_channels, [servo_z] * len(servo_channels)),
        )
    container = frpl.data_serialization.FeagiByteContainer()
    container.add_struct(mapped, True)
    return bytes(container.copy_out_as_byte_vector())


def test_tick_drains_backlog_newest_servo_and_misc_union(caplog):
    robot, out = _real_frame_robot()
    out._client.frames = [
        _servo_frame(out, [1], z=9),  # ch1 older (decodes to 0.0; z is inverted)
        _misc_frame(out, 3, servo_channels=[0], servo_z=9),
        _misc_frame(out, 0),
        _servo_frame(out, [1], z=0),  # ch1 newest (decodes to 1.0)
    ]
    with caplog.at_level("WARNING"):
        asyncio.run(robot.tick())
    assert out._client.frames == []
    assert len(robot.format_calls) == 1
    servo, misc = robot.format_calls[0]
    assert set(servo[0]) == {0, 1}
    assert servo[0][1] == pytest.approx(1.0, abs=0.02)  # newest ch1
    assert servo[0][0] == pytest.approx(0.0, abs=0.02)
    assert sorted(misc[1]) == [(0, 0, 1), (3, 0, 1)]
    assert sum("backlog" in r.message for r in caplog.records) == 1
    # Second heavy tick within 5 s: rate-limited, no new warning.
    out._client.frames = [_servo_frame(out, [1]) for _ in range(4)]
    with caplog.at_level("WARNING"):
        asyncio.run(robot.tick())
    assert sum("backlog" in r.message for r in caplog.records) == 1


def test_tick_misc_empty_when_frames_arrived_without_misc():
    robot, out = _real_frame_robot()
    out._client.frames = [_servo_frame(out, [1]), _servo_frame(out, [0])]
    asyncio.run(robot.tick())
    assert robot.format_calls[0][1] == {1: []}


def test_tick_drain_is_capped():
    robot, out = _real_frame_robot()
    robot.MAX_DRAIN_FRAMES = 3
    out._client.frames = [_servo_frame(out, [1]) for _ in range(10)]
    asyncio.run(robot.tick())
    assert len(out._client.frames) == 7


def test_tick_servo_targets_empty_without_a_frame():
    robot, out = _real_frame_robot()
    out._client.frames = [_servo_frame(out, [1])]
    asyncio.run(robot.tick())
    asyncio.run(robot.tick())  # no frame this tick
    assert robot.format_calls[1][0] == {0: {}}


def test_bad_servo_mode_raises():
    robot, out = _real_frame_robot()
    robot.SERVO_TARGETS_MODE = "bogus"
    out._client.frames = [_servo_frame(out, [1])]
    with pytest.raises(ValueError):
        asyncio.run(robot.tick())
