"""
Smart IMU (orientation quaternion) sensory input.

Maps to Rust ``sensor_SmartIMU_register`` / ``sensor_smart_i_m_u_write``.
Quaternion convention for writes: ``(w, x, y, z)`` unit quaternion with each
component in ``[-1, 1]``, consistent with ``BrainOutput.write_sensor_smart_imu``.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from feagi.pns.inputs.base import BaseInput


class SmartImu(BaseInput):
    """
    Smart IMU input: a single orientation quaternion per channel.

    Use :meth:`set_quaternion_ros_pose` with ``geometry_msgs/msg/Quaternion``
    fields ``x, y, z, w`` (ROS ordering); values are normalized and converted
    internally to ``(w, x, y, z)`` for the cache.
    """

    def __init__(self, channel_index: int = 0) -> None:
        super().__init__()
        self.channel_index = int(channel_index)
        self._quat_wxyz: Optional[Tuple[float, float, float, float]] = None

    @classmethod
    def register(cls, *, group_id: int, channel_index: int = 0) -> SmartImu:
        """
        Register a Smart IMU sensory group with the global ``brain_input``.

        Args:
            group_id: Cortical / cache group id (from genome / connector mapping).
            channel_index: Channel within the group (0-based).
        """
        from feagi.pns import brain_input

        sensor = cls(channel_index=channel_index)
        brain_input.register_input(sensor, group_id=int(group_id))
        return sensor

    def set_quaternion_ros_pose(self, x: float, y: float, z: float, w: float) -> None:
        """
        Set orientation from a ROS ``geometry_msgs/Quaternion`` (x,y,z,w).

        Non-finite or near-zero norm quaternions clear the pending sample so
        :meth:`_write_to_cache` skips until a valid pose arrives.
        """
        normed = _normalize_quaternion_xyzw(float(x), float(y), float(z), float(w))
        self._quat_wxyz = normed

    def _register_with_cache(self, cache: object, group_id: int) -> None:
        import feagi_rust_py_libs as frpl

        frame_enum = frpl.data_structures.genomic.cortical_area.FrameChangeHandling
        frame = frame_enum.Absolute()
        positioning_enum = (
            frpl.data_structures.genomic.cortical_area.PercentageNeuronPositioning
        )
        positioning = positioning_enum.Linear()
        n_ch = max(1, int(self.channel_index) + 1)
        register_method = None
        for name in ("sensor_SmartIMU_register", "sensor_smart_i_m_u_register"):
            if hasattr(cache, name):
                register_method = getattr(cache, name)
                break
        if register_method is None:
            raise AttributeError(
                "ConnectorAgent missing SmartIMU registration "
                "(expected sensor_SmartIMU_register or sensor_smart_i_m_u_register)",
            )
        register_method(
            group=group_id,
            number_channels=n_ch,
            frame_change_handling=frame,
            z_neuron_resolution=10,
            percentage_neuron_positioning=positioning,
        )

    def _write_to_cache(self, cache: object) -> None:
        if self._quat_wxyz is None:
            return
        w, x, y, z = self._quat_wxyz
        import feagi_rust_py_libs as frpl

        dt = frpl.connector_core.data_types
        sp = dt.SignedPercentage
        qudata = dt.SignedPercentage4D(
            sp.new_from_m1_1(float(w)),
            sp.new_from_m1_1(float(x)),
            sp.new_from_m1_1(float(y)),
            sp.new_from_m1_1(float(z)),
        )
        write_method = None
        for name in ("sensor_smart_i_m_u_write", "sensor_SmartIMU_write"):
            if hasattr(cache, name):
                write_method = getattr(cache, name)
                break
        if write_method is None:
            raise AttributeError(
                "ConnectorAgent missing SmartIMU write "
                "(expected sensor_smart_i_m_u_write)",
            )
        write_method(
            group=self.group_id,
            channel_index=int(self.channel_index),
            data=qudata,
        )


def _normalize_quaternion_xyzw(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Optional[Tuple[float, float, float, float]]:
    """Return ``(w, x, y, z)`` unit quaternion or ``None`` if invalid."""
    for v in (x, y, z, w):
        if not math.isfinite(v):
            return None
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    return (w, x, y, z)
