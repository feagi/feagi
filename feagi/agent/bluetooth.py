"""
Bluetooth Robot Agent

Base class for Bluetooth-enabled robots on typed ``BrainOutput``.
The base owns the loop, the FEAGI connection and the transport; a subclass
declares only its devices and two byte codecs. All motor decoding happens
once, in the Rust motor cache; nothing is decoded in Python.
"""

import argparse
import asyncio
import logging
import os
import socket
import time
from abc import abstractmethod
from typing import Any, Dict, List, Optional, Tuple

from feagi.agent.args import add_feagi_connection_args
from feagi.pns.brain_output import brain_output as _default_brain_output

from .base import BaseAgent

logger = logging.getLogger(__name__)

_BLE_KEYS = ("device_name", "service_uuid", "rx_uuid", "tx_uuid")
_ARG_TO_BLE_KEY = {
    "bt_device_name": "device_name",
    "bt_service_uuid": "service_uuid",
    "bt_rx_uuid": "rx_uuid",
    "bt_tx_uuid": "tx_uuid",
}
# Typed IMU readings returned by parse_sensors -> brain_output writer.
_IMU_WRITERS = {
    "accelerometer": (
        "write_sensor_raw_imu_accelerometer",
        "accelerometer_xyz",
    ),
    "gyroscope": ("write_sensor_raw_imu_gyroscope", "gyroscope_xyz"),
    "magnetometer": ("write_sensor_raw_imu_magnetometer", "magnetometer_xyz"),
}
_SCALAR_UNITS = ("Proximity", "Servo", "Shock", "MiscData")
_MAX_NOTIFICATIONS_PER_TICK = 64
_NOTIFICATION_WAIT_S = 0.005


def add_bluetooth_args(parser: argparse.ArgumentParser) -> None:
    """Add the optional BLE connection arguments (all default to unset)."""
    parser.add_argument("--bt-device-name", default=None)
    parser.add_argument("--bt-service-uuid", default=None)
    parser.add_argument("--bt-rx-uuid", default=None)
    parser.add_argument("--bt-tx-uuid", default=None)


class BluetoothRobot(BaseAgent):
    """
    Base class for Bluetooth robots.

    A subclass declares:

    - ``Z_NEURON_RESOLUTION``: cortical depth for its sensor and motor units.
    - ``BLUETOOTH_CONFIG``: optional fallback BLE settings. The launcher
      normally passes ``--bt-device-name --bt-service-uuid --bt-rx-uuid
      --bt-tx-uuid``; those win over the class value.
    - ``sensor_units()``: ``{unit_name: channel_count}`` (e.g. ``RawIMU``).
    - ``motor_groups()``: ``{group: {"positional_servo": [channel names]}}``.
    - ``misc_motor_groups()`` (optional): ``{group: (width, height, depth)}``.
    - ``parse_sensors(raw)``: robot bytes -> typed readings.
    - ``format_motors(servo_targets, misc_values)``: decoded FEAGI motor
      values -> robot bytes.

    Typed readings returned by ``parse_sensors`` are
    ``{"gyroscope": (x, y, z)}`` (also ``accelerometer``, ``magnetometer``;
    each component in ``[-1, 1]``) or ``{"Proximity": 0.4}`` (also ``Servo``,
    ``Shock``, ``MiscData``; value in ``[0, 1]``). A value may instead be a
    ``{channel_index: reading}`` dict; a bare reading means channel 0.

    ``format_motors`` receives ``servo_targets`` as
    ``{group: {channel: target_0_1}}`` (last-held targets) and
    ``misc_values`` as ``{group: [(x, y, z), ...]}`` (active voxels of the
    frame received this tick; empty on ticks without a frame). It is first
    called once FEAGI has sent a motor frame. Return ``b""`` to send nothing.

    Example::

        class MyRobot(BluetoothRobot):
            Z_NEURON_RESOLUTION = 10

            def sensor_units(self):
                return {"RawIMU": 1}

            def motor_groups(self):
                return {0: {"positional_servo": ["a", "b"]}}

            def parse_sensors(self, raw):
                return {"gyroscope": (0.0, 0.0, 0.0)}

            def format_motors(self, servo_targets, misc_values):
                return bytes(...)

        parser = argparse.ArgumentParser()
        MyRobot.add_arguments(parser)
        asyncio.run(MyRobot.from_args(parser.parse_args()).run())
    """

    BLUETOOTH_CONFIG: Dict[str, str] = {}
    Z_NEURON_RESOLUTION: Optional[int] = None

    def __init__(
        self,
        agent_id: str,
        feagi_host: str = "localhost",
        feagi_port: int = 3000,
        platform: Optional[str] = None,
        capabilities: Optional[Dict[str, Any]] = None,
        *,
        args: Optional[argparse.Namespace] = None,
        burst_period_s: Optional[float] = None,
        brain_output: Any = None,
    ):
        """
        Initialize the Bluetooth robot agent.

        Args:
            agent_id: Identifier for this robot.
            feagi_host: FEAGI host (the ``--ip`` argument overrides it).
            feagi_port: FEAGI HTTP port (``--port`` overrides it).
            platform: ``"desktop"`` (bleak) or ``"cloud"`` (relay); detected
                from the environment when omitted.
            capabilities: Agent capabilities.
            args: Parsed shared + Bluetooth arguments (see ``add_arguments``).
                Required to ``run()`` against FEAGI.
            burst_period_s: Loop period; default ``--feagi-burst-period-s``.
            brain_output: BrainOutput instance (module singleton by default).
        """
        super().__init__(agent_id, feagi_host, capabilities)
        self.feagi_port = feagi_port
        self.args = args
        self.platform = platform or self._detect_platform()
        self.transport = None
        self.logger = logger
        self.brain_output = brain_output or _default_brain_output
        if burst_period_s is None and args is not None:
            burst_period_s = getattr(args, "feagi_burst_period_s", None)
        self.burst_period_s = burst_period_s
        self.bluetooth_config = self._resolve_bluetooth_config(args)
        self._sensor_groups: Dict[str, int] = {}
        self._seen_motor_frame = False

        logger.info(
            "Initializing %s on %s platform", self.__class__.__name__,
            self.platform,
        )

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
        """Add the shared FEAGI connection args plus the BLE args."""
        add_feagi_connection_args(parser)
        add_bluetooth_args(parser)

    @classmethod
    def from_args(cls, args: argparse.Namespace, **kwargs: Any):
        """Build the robot from parsed arguments."""
        return cls(
            args.agent_id,
            feagi_host=args.ip,
            feagi_port=args.port,
            args=args,
            **kwargs,
        )

    def _resolve_bluetooth_config(
        self, args: Optional[argparse.Namespace]
    ) -> Dict[str, str]:
        """Merge BLE settings: arguments win, class value is the fallback."""
        config = dict(self.BLUETOOTH_CONFIG)
        for arg_name, key in _ARG_TO_BLE_KEY.items():
            value = getattr(args, arg_name, None) if args else None
            if value:
                config[key] = value
        missing = [key for key in _BLE_KEYS if not config.get(key)]
        if missing:
            raise ValueError(
                f"{self.__class__.__name__} has no Bluetooth settings for "
                f"{missing}: pass --bt-* arguments or define BLUETOOTH_CONFIG"
            )
        return config

    def _detect_platform(self) -> str:
        """
        Auto-detect if running on desktop or cloud.

        Detection strategy (in order):
        1. FEAGI_PLATFORM env var (explicit override)
        2. FEAGI_BLE_RELAY_ENABLED=true -> cloud (platform manages BLE)
        3. Check if BLE relay port is listening -> cloud
        4. Default -> desktop (Python manages BLE directly)

        Returns:
            "desktop" - Python owns BLE connection (direct bleak)
            "cloud" - Platform owns BLE (WebSocket relay)
        """
        platform_env = os.getenv("FEAGI_PLATFORM", "").lower()
        if platform_env in ("desktop", "cloud"):
            logger.info(
                "Platform explicitly set via FEAGI_PLATFORM: %s", platform_env
            )
            return platform_env

        if os.getenv("FEAGI_BLE_RELAY_ENABLED", "").lower() in (
            "true", "1", "yes"
        ):
            logger.info("BLE relay enabled by platform - using cloud mode")
            return "cloud"

        if os.getenv("FEAGI_BLE_RELAY_PORT"):
            try:
                relay_port = int(os.getenv("FEAGI_BLE_RELAY_PORT"))
                if self._is_port_listening(relay_port):
                    logger.info(
                        "BLE relay detected on port %s - using cloud mode",
                        relay_port,
                    )
                    return "cloud"
            except ValueError:
                logger.warning(
                    "Invalid FEAGI_BLE_RELAY_PORT: %s",
                    os.getenv("FEAGI_BLE_RELAY_PORT"),
                )

        logger.info("No BLE relay detected - using desktop mode (direct BLE)")
        return "desktop"

    def _is_port_listening(self, port: int) -> bool:
        """Check if a port is listening (relay server is running)."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(0.1)
                return sock.connect_ex(("127.0.0.1", port)) == 0
        except Exception as exc:
            logger.debug("Port check failed for %s: %s", port, exc)
            return False

    def initialize_hardware(self):
        """Create the Bluetooth transport for the detected platform."""
        logger.info("Initializing Bluetooth transport for %s", self.platform)
        if self.platform == "desktop":
            self._init_desktop_transport()
        else:
            self._init_cloud_transport()

    def _init_desktop_transport(self):
        """Initialize direct BLE connection for desktop."""
        from feagi.transports import BluetoothTransport

        self.transport = BluetoothTransport(self.bluetooth_config)
        logger.info("Desktop Bluetooth transport initialized")

    def _init_cloud_transport(self):
        """
        Initialize the WebSocket client to the platform BLE relay.

        Configuration comes only from environment variables set by the
        platform: ``FEAGI_BLE_RELAY_PORT`` (required) and
        ``FEAGI_BLE_RELAY_HOST`` (defaults to 127.0.0.1).
        """
        from feagi.transports import WebSocketClientTransport

        relay_host = os.getenv("FEAGI_BLE_RELAY_HOST", "127.0.0.1")
        relay_port_str = os.getenv("FEAGI_BLE_RELAY_PORT")
        if not relay_port_str:
            raise RuntimeError(
                "BLE relay mode selected but FEAGI_BLE_RELAY_PORT not set. "
                "The platform must set it when launching the controller."
            )
        try:
            relay_port = int(relay_port_str)
        except ValueError as exc:
            raise RuntimeError(
                "Invalid FEAGI_BLE_RELAY_PORT: must be integer, got "
                f"'{relay_port_str}'"
            ) from exc

        self.transport = WebSocketClientTransport(
            {
                "host": relay_host,
                "port": relay_port,
                "embodiment_id": self.agent_id,
            }
        )
        logger.info(
            "BLE relay client transport initialized: ws://%s:%s",
            relay_host,
            relay_port,
        )

    # Subclass hooks

    def sensor_units(self) -> Dict[str, int]:
        """``{unit_name: channel_count}`` (e.g. ``{"RawIMU": 1}``)."""
        return {}

    @abstractmethod
    def motor_groups(self) -> Dict[int, Dict[str, List[str]]]:
        """``{group: {"positional_servo": [channel names]}}``."""

    def misc_motor_groups(self) -> Dict[int, Tuple[int, int, int]]:
        """Optional ``{group: (width, height, depth)}`` MiscData outputs."""
        return {}

    @abstractmethod
    def parse_sensors(self, raw: bytes) -> Dict[str, Any]:
        """
        Parse robot bytes into typed readings.

        Returns e.g. ``{"gyroscope": (x, y, z)}`` or ``{"Proximity": 0.4}``;
        see the class docstring for the accepted shapes. Return ``{}`` when
        the bytes carry nothing to write.
        """

    @abstractmethod
    def format_motors(
        self,
        servo_targets: Dict[int, Dict[int, float]],
        misc_values: Dict[int, List[Tuple[int, int, int]]],
    ) -> bytes:
        """
        Format decoded FEAGI motor values into the robot's protocol.

        Args:
            servo_targets: ``{group: {channel: target_0_1}}``, last held.
            misc_values: ``{group: [(x, y, z), ...]}`` active voxels of the
                frame received this tick (empty without a new frame).

        Returns:
            Bytes to send over Bluetooth, or ``b""`` to send nothing.
        """

    # Setup

    def register_devices(self) -> None:
        """Register sensor units and motor outputs on the Rust cache."""
        if self.Z_NEURON_RESOLUTION is None:
            raise ValueError(
                f"{self.__class__.__name__} must define Z_NEURON_RESOLUTION"
            )
        z_res = int(self.Z_NEURON_RESOLUTION)
        sensors = self.sensor_units()
        if sensors:
            self._sensor_groups = self.brain_output.register_sensor_units(
                sensors, z_neuron_resolution=z_res
            )
        self.brain_output.register_motor_groups(
            self.motor_groups(), z_neuron_resolution=z_res
        )
        for group, (width, height, depth) in self.misc_motor_groups().items():
            self.brain_output.register_motor_misc_data(
                group=group, width=width, height=height, depth=depth
            )

    # Loop

    async def tick(self) -> None:
        """One cycle: motors out to the robot, sensors in to FEAGI."""
        bo = self.brain_output
        bo.receive()
        frame = bo.motor_frame_received
        self._seen_motor_frame = self._seen_motor_frame or frame
        if self._seen_motor_frame:
            servo_targets = {
                group: bo.read_positional_servo_targets(group)
                for group, kinds in self.motor_groups().items()
                if kinds.get("positional_servo")
            }
            misc_values = {}
            if frame:
                for group in self.misc_motor_groups():
                    misc_values[group] = self._active_voxels(
                        bo.read_misc_data(group, 0)
                    )
            payload = self.format_motors(servo_targets, misc_values)
            if payload:
                await self.transport.send(payload)
        for raw in await self._drain_notifications():
            self._write_readings(self.parse_sensors(raw))
        bo.flush_sensory_bytes()

    @staticmethod
    def _active_voxels(volume: Any) -> List[Tuple[int, int, int]]:
        """Return ``(x, y, z)`` of every non-zero voxel."""
        import numpy as np

        return [
            (int(x), int(y), int(z))
            for x, y, z in np.argwhere(np.asarray(volume) != 0)
        ]

    async def _drain_notifications(self) -> List[bytes]:
        """Collect the robot notifications that are waiting right now."""
        chunks: List[bytes] = []
        for _ in range(_MAX_NOTIFICATIONS_PER_TICK):
            try:
                data = await asyncio.wait_for(
                    self.transport.receive(), timeout=_NOTIFICATION_WAIT_S
                )
            except asyncio.TimeoutError:
                break
            if not data:
                break
            chunks.append(bytes(data))
        return chunks

    def _write_readings(self, readings: Dict[str, Any]) -> None:
        """Write typed readings through the typed ``write_sensor_*`` calls."""
        bo = self.brain_output
        for name, reading in (readings or {}).items():
            per_channel = (
                reading if isinstance(reading, dict) else {0: reading}
            )
            if name in _IMU_WRITERS:
                method, kwarg = _IMU_WRITERS[name]
                group = self._sensor_groups["RawIMU"]
                for channel, xyz in per_channel.items():
                    getattr(bo, method)(
                        group=group,
                        channel_index=int(channel),
                        **{kwarg: tuple(xyz)},
                    )
            elif name in _SCALAR_UNITS:
                for channel, value in per_channel.items():
                    bo.write_sensor_scalar(
                        unit_key=name,
                        group=self._sensor_groups[name],
                        channel_index=int(channel),
                        scalar_0_1=float(value),
                    )
            else:
                raise ValueError(f"Unsupported sensor reading '{name}'")

    async def run(self):
        """
        Configure, register, connect and run the loop until ``stop()``.

        Order: ``configure_from_args`` -> register devices -> open the
        Bluetooth transport -> connect to FEAGI -> ``tick()`` every burst
        period.
        """
        if self.args is None or not self.burst_period_s:
            raise RuntimeError(
                "BluetoothRobot.run() needs the shared arguments "
                "(see add_arguments) including --feagi-burst-period-s."
            )
        self.brain_output.configure_from_args(self.args)
        self.register_devices()
        if self.transport is None:
            self.initialize_hardware()
        await self.transport.connect()
        self.brain_output.connect()
        self.running = True
        try:
            while self.running:
                started = time.monotonic()
                await self.tick()
                remaining = self.burst_period_s - (time.monotonic() - started)
                if remaining > 0:
                    await asyncio.sleep(remaining)
        finally:
            await self.stop()

    async def stop(self):
        """Stop the loop and disconnect from Bluetooth and FEAGI."""
        logger.info("Stopping robot...")
        self.running = False
        if self.transport:
            await self.transport.disconnect()
        if self.brain_output.is_connected():
            self.brain_output.disconnect()
        logger.info("Robot stopped")

    # BaseAgent required methods (with sensible defaults)
    
    def map_sensors(self, hardware_data: Any) -> Dict[str, bytes]:
        """
        BaseAgent compatibility method.
        
        For BluetoothRobot, sensor mapping is done in parse_sensors().
        """
        return {}
    
    def map_motors(self, feagi_output: Dict[str, Any]) -> Any:
        """
        BaseAgent compatibility method.
        
        For BluetoothRobot, motor mapping is done in format_motors().
        """
        return None

