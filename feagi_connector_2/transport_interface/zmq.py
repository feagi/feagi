from __future__ import annotations

import asyncio
import json
from contextlib import suppress
from typing import Any, Dict, Optional

import zmq
import zmq.asyncio

from ..callback import AsyncSignalWithParam


class FeagiZmqClient:
    """Manage FEAGI ZeroMQ sockets for registration, sensory and motor data."""

    def __init__(
        self,
        registration_address: str,
        sensory_address: str,
        motor_address: str,
        *,
        context: Optional[zmq.asyncio.Context] = None,
    ) -> None:
        self._context = context or zmq.asyncio.Context.instance()

        self._registration_socket = self._context.socket(zmq.REQ)
        self._registration_socket.connect(registration_address)

        self._sensory_socket = self._context.socket(zmq.PUSH)
        self._sensory_socket.connect(sensory_address)

        self._motor_socket = self._context.socket(zmq.PULL)
        self._motor_socket.connect(motor_address)

        self.motor_signal: AsyncSignalWithParam[bytes] = AsyncSignalWithParam()
        self._motor_listener: Optional[asyncio.Task[None]] = None

    async def send_registration(self, camera_resolution_xyc: (int, int, int)) -> dict:
        """Send a registration payload as JSON and return the response bytes."""

        payload = {
            "method": "POST",
            "path": "/v1/agent/register",
            "body": {
                "agent_id": "autonomous_robot",
                "agent_type": "both",
                "capabilities": {
                    "vision": {
                        "modality": "camera",
                        "dimensions": [camera_resolution_xyc[0], camera_resolution_xyc[1]],
                        "channels": camera_resolution_xyc[2],
                        "target_cortical_area": "i_vision"
                    },
                    "motor": {
                        "modality": "wheel_motors",
                        "output_count": 2,
                        "source_cortical_areas": ["o_motor"]
                    }
                }
            }
        }

        message = json.dumps(payload).encode("utf-8")
        await self._registration_socket.send(message)
        response: bytes = await self._registration_socket.recv()
        response_str: str = response.decode("utf-8")
        data_dict = json.loads(response_str)
        print(data_dict)
        return data_dict

    async def push_sensory_data(self, data: bytes) -> None:
        """Push sensory byte data towards FEAGI."""

        await self._sensory_socket.send(data)

    def start_motor_listener(self) -> None:
        """Start the background task that relays motor bytes to registered callbacks."""

        if self._motor_listener is None or self._motor_listener.done():
            loop = asyncio.get_running_loop()
            self._motor_listener = loop.create_task(self._motor_loop())

    async def _motor_loop(self) -> None:
        while True:
            data = await self._motor_socket.recv()
            await self.motor_signal.emit(data)

    async def close(self) -> None:
        """Close sockets and stop the motor listener."""

        if self._motor_listener is not None:
            self._motor_listener.cancel()
            with suppress(asyncio.CancelledError):
                await self._motor_listener

        self._registration_socket.close(linger=0)
        self._sensory_socket.close(linger=0)
        self._motor_socket.close(linger=0)


