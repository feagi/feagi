from typing import Callable, Awaitable, TypeVar, Generic
from feagi_connector_2.callback import AsyncSignal

class FeagiInterface:

    def __init__(self):
        self._signal_connected_and_registered: AsyncSignal = AsyncSignal()
        self._signal_connection_failed: AsyncSignal = AsyncSignal() # TODO reason?
        self._signal_connection_ended: AsyncSignal = AsyncSignal()

    def register_callback_for_connection_and_registration_success(self, callback: Callable[[], Awaitable[None]]) -> None:
        self._signal_connected_and_registered.register(callback)

    def register_callback_for_connection_failure(self, callback: Callable[[], Awaitable[None]]) -> None:
        self._signal_connection_failed.register(callback)

    def register_callback_for_connection_end_success(self, callback: Callable[[], Awaitable[None]]) -> None:
        self._signal_connection_ended.register(callback)

    async def connect_via_neurorobotics_studio(self) -> bool:
        raise NotImplementedError

    async def connect_via_zmq(self, feagi_host: str = "localhost", communication_port: int = 5558, brain_input_port: int = 5555, brain_output_port: int = 5556) -> bool:
        #TODO block device registration

        pass