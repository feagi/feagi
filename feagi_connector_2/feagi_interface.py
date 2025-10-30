from typing import Callable, Awaitable, TypeVar, Generic
from feagi_connector_2.callback import AsyncSignal
from feagi_connector_2.transport_interface.zmq import FeagiZmqClient

class FeagiInterface:

    def __init__(self):
        self._signal_connected_and_registered: AsyncSignal = AsyncSignal()
        self._signal_connection_failed: AsyncSignal = AsyncSignal() # TODO reason?
        self._signal_connection_ended: AsyncSignal = AsyncSignal()

        self._transport = None

    def register_callback_for_connection_and_registration_success(self, callback: Callable[[], Awaitable[None]]) -> None:
        self._signal_connected_and_registered.register(callback)

    def register_callback_for_connection_failure(self, callback: Callable[[], Awaitable[None]]) -> None:
        self._signal_connection_failed.register(callback)

    def register_callback_for_connection_end_success(self, callback: Callable[[], Awaitable[None]]) -> None:
        self._signal_connection_ended.register(callback)

    async def connect_to_neurorobotics_studio(self) -> bool:
        raise NotImplementedError

    async def connect_via_zmq(self, feagi_host_address: str, camera_resolution_xyc: (int, int, int), registration_port: int = 30001, brain_input_port: int = 5555, brain_output_port: int = 30005, heartbeat_interval: float = 5.0) -> dict:
        # TODO block device registration
        # TODO check valid heartbeat_interval
        # TODO actually get device config


        registration_endpoint: str = feagi_host_address + ":" +  str(registration_port)
        brain_input_endpoint: str = feagi_host_address + ":" + str(brain_input_port)
        brain_output_endpoint: str = feagi_host_address + ":" + str(brain_output_port)

        capabilities: dict = {} # TODO

        self._transport = FeagiZmqClient(registration_endpoint, brain_input_endpoint, brain_output_endpoint)
        response = await self._transport.send_registration(camera_resolution_xyc)
        return response