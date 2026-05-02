"""
ROS 2 ↔ FEAGI integration round-trip (numeric / distance-style I/O).

Level 1 — ROS only (no FEAGI):
    Verifies ``std_msgs/msg/Float64`` publish → subscribe in-process.

Level 2 — FEAGI + ROS (opt-in):
    Registers a minimal agent (dummy Camera + Infrared + RotaryMotor), runs a
    short loop: ``brain_output.receive()`` → publish motor speed to ROS →
    subscriber maps the value into ``Infrared.set_distance()`` →
    ``brain_input.send()``. This closes the loop FEAGI motor → ROS → FEAGI
    sensory (same pattern as Neurorobotics Studio ROS bridge for Float64).

Run Level 1 only::

    pytest feagi-python-sdk/tests/integration/test_ros_feagi_numeric_roundtrip.py -v

Run Level 2 (requires live FEAGI + same env as ``examples/simple_robot``)::

    export ROS_FEAGI_E2E=1
    # plus FEAGI_AGENT_DESCRIPTOR_B64, FEAGI_HOST, ports, FEAGI_AUTH_TOKEN_B64, ...
    pytest feagi-python-sdk/tests/integration/test_ros_feagi_numeric_roundtrip.py -v -k e2e

Add more I/O profiles later by parametrizing encode/decode helpers (Vision,
String, …) while reusing the same ROS wiring pattern.
"""

from __future__ import annotations

import os
import threading
import time
from typing import List

import pytest

_TOPIC_FLOAT = "/feagi_ros_e2e/test_float_ping"
_TOPIC_MOTOR_BRIDGE = "/feagi_ros_e2e/motor_to_ir_bridge"


@pytest.fixture(scope="module")
def rclpy_initialized() -> None:
    """One ROS graph init per module for DDS tests."""
    try:
        import rclpy
    except ImportError:
        pytest.skip("rclpy is not installed (install ROS 2 Python bindings)")
    rclpy.init()
    yield
    rclpy.shutdown()


def rotary_speed_to_infrared_cm(speed_signed: float) -> float:
    """Map RotaryMotor signed speed [-1, 1] to IR distance [0, 400] cm (test heuristic)."""
    s = float(max(-1.0, min(1.0, speed_signed)))
    return (s + 1.0) * 200.0


@pytest.mark.integration
def test_ros2_float64_pub_sub_loopback(rclpy_initialized: None) -> None:
    """ROS-only: DDS delivers Float64 publisher → subscriber."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64

    received: List[float] = []
    node = Node("feagi_ros_e2e_float_loopback")

    def on_msg(msg: Float64) -> None:
        received.append(float(msg.data))

    node.create_subscription(Float64, _TOPIC_FLOAT, on_msg, 10)
    pub = node.create_publisher(Float64, _TOPIC_FLOAT, 10)

    spins = 0
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and len(received) < 5:
        m = Float64()
        spins += 1
        m.data = float(spins) * 0.01 + 0.42
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    assert received, "No Float64 messages received on loopback topic"
    assert pytest.approx(received[0], rel=0, abs=1e-6) == (0.01 + 0.42)


def _feagi_e2e_env_ready() -> bool:
    if os.environ.get("ROS_FEAGI_E2E", "").strip() not in ("1", "true", "True", "yes"):
        return False
    required = (
        "FEAGI_AGENT_DESCRIPTOR_B64",
        "FEAGI_HOST",
        "FEAGI_REGISTRATION_PORT",
        "FEAGI_SENSORY_PORT",
        "FEAGI_MOTOR_PORT",
        "FEAGI_CONNECTION_TIMEOUT_MS",
        "FEAGI_REGISTRATION_RETRIES",
        "FEAGI_HEARTBEAT_INTERVAL_S",
        "FEAGI_API_PORT",
        "FEAGI_HTTP_TIMEOUT_S",
    )
    return all(os.environ.get(k) for k in required)


@pytest.mark.integration
@pytest.mark.skipif(
    not _feagi_e2e_env_ready(),
    reason="Set ROS_FEAGI_E2E=1 and all FEAGI_* env vars (see module docstring).",
)
def test_feagi_ros_rotary_motor_infrared_roundtrip(rclpy_initialized: None) -> None:
    """
    Live FEAGI: motor command → ROS Float64 → IR distance → brain_input.

    Proximity-style semantics use Infrared distance (cm) as the FEAGI sensory
    stand-in; ROS side uses ``std_msgs/msg/Float64`` like the desktop ROS bridge.
    """
    import numpy as np
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float64

    from feagi.pns import brain_input, brain_output
    from feagi.pns.inputs import Camera, Infrared
    from feagi.pns.outputs import RotaryMotor

    auth = os.environ.get("FEAGI_AUTH_TOKEN_B64")
    if not auth:
        pytest.skip("FEAGI_AUTH_TOKEN_B64 is required for brain_output.connect()")

    agent_id = os.environ["FEAGI_AGENT_DESCRIPTOR_B64"]
    host = os.environ["FEAGI_HOST"]
    reg = int(os.environ["FEAGI_REGISTRATION_PORT"])
    sensory = int(os.environ["FEAGI_SENSORY_PORT"])
    motor = int(os.environ["FEAGI_MOTOR_PORT"])
    conn_to = int(os.environ["FEAGI_CONNECTION_TIMEOUT_MS"])
    reg_retries = int(os.environ["FEAGI_REGISTRATION_RETRIES"])
    hb = float(os.environ["FEAGI_HEARTBEAT_INTERVAL_S"])
    api = int(os.environ["FEAGI_API_PORT"])
    http_to = float(os.environ["FEAGI_HTTP_TIMEOUT_S"])
    hb_join = float(os.environ.get("FEAGI_HEARTBEAT_JOIN_TIMEOUT_S", "2.0"))

    cam = Camera.register(
        resolution=(32, 32),
        encoding="absolute",
        position="center",
        group_id=251,
    )
    infra = Infrared.register(
        encoding="absolute",
        inverted=False,
        min_distance=0.0,
        max_distance=400.0,
    )

    rotary = RotaryMotor.register(
        encoding="absolute",
        bidirectional=True,
        unit_id=0,
        channel_index=int(os.environ.get("FEAGI_E2E_MOTOR_CHANNEL", "0")),
    )

    brain_input.configure(
        feagi_host=host,
        feagi_port=sensory,
        motor_port=motor,
        registration_port=reg,
        transport="zmq",
        api_port=api,
        feagi_http_timeout_s=http_to,
        heartbeat_interval_s=hb,
        heartbeat_join_timeout_s=hb_join,
        connection_timeout_ms=conn_to,
        registration_retries=reg_retries,
        auth_token_b64=auth,
    )

    vision_unit = (
        "camera",
        int(cam.resolution[0]),
        int(cam.resolution[1]),
        3,
        "vision",
        int(cam.group_id),
    )

    brain_output.configure(
        agent_id=agent_id,
        feagi_host=host,
        feagi_registration_port=reg,
        feagi_sensory_port=sensory,
        feagi_motor_port=motor,
        transport="zmq",
        feagi_connection_timeout_ms=conn_to,
        feagi_registration_retries=reg_retries,
        feagi_heartbeat_interval_s=hb,
        feagi_api_port=api,
        feagi_http_timeout_s=http_to,
        auth_token_b64=auth,
        vision_unit=vision_unit,
    )

    brain_input.register_agent(agent_id=agent_id, agent_type="sensory")

    ros_node = Node("feagi_ros_e2e_bridge")
    ir_updates: List[float] = []

    def on_ros_motor_feedback(msg: Float64) -> None:
        infra.set_distance(rotary_speed_to_infrared_cm(float(msg.data)))
        ir_updates.append(float(msg.data))

    ros_node.create_subscription(
        Float64, _TOPIC_MOTOR_BRIDGE, on_ros_motor_feedback, 10
    )
    pub_motor = ros_node.create_publisher(Float64, _TOPIC_MOTOR_BRIDGE, 10)

    stop_spin = threading.Event()

    def spin_ros() -> None:
        exe = SingleThreadedExecutor()
        exe.add_node(ros_node)
        while not stop_spin.is_set():
            exe.spin_once(timeout_sec=0.02)

    spin_thread = threading.Thread(target=spin_ros, daemon=True)
    spin_thread.start()

    frame = np.zeros((32, 32, 3), dtype=np.uint8)

    try:
        brain_input.connect()
        brain_output.connect()
    except Exception as exc:
        stop_spin.set()
        spin_thread.join(timeout=2.0)
        ros_node.destroy_node()
        pytest.skip(f"FEAGI not reachable (configure live stack): {exc}")

    loop_ticks = int(os.environ.get("FEAGI_E2E_ROS_LOOPS", "120"))
    try:
        for _ in range(loop_ticks):
            brain_output.receive()
            speed = float(rotary.get_speed())
            out = Float64()
            out.data = speed
            pub_motor.publish(out)
            time.sleep(0.002)
            cam.set_frame(frame)
            brain_input.send()
            time.sleep(0.005)
    finally:
        try:
            brain_input.disconnect()
        except Exception:
            pass
        try:
            brain_output.disconnect()
        except Exception:
            pass
        stop_spin.set()
        spin_thread.join(timeout=2.0)
        try:
            ros_node.destroy_node()
        except Exception:
            pass

    assert ir_updates, (
        "ROS bridge received no motor→Float64 messages; check executor / DDS."
    )


@pytest.mark.integration
def test_encode_profile_float64_roundtrip_shapes() -> None:
    """Pure unit: document mapping used by the ROS bridge + this E2E."""
    assert pytest.approx(rotary_speed_to_infrared_cm(-1.0)) == 0.0
    assert pytest.approx(rotary_speed_to_infrared_cm(1.0)) == 400.0
    assert pytest.approx(rotary_speed_to_infrared_cm(0.0)) == 200.0
