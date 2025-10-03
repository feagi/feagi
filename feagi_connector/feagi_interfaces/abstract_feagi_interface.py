from typing import Callable, Awaitable
from .feagi_connection_status import FEAGIConnectionStatus
import abc

# NOTE: Callable[[obj1, obj2], Awaitable[obj3]] -> A callable function that takes 2 arguments (object 1, object 2) and has a awaited return of object 3

class AbstractFeagiInterface(abc.ABC):
    def __init__(self):
        self._receive_bytes_callbacks = []
        self._status_change_callbacks = []
        self._status: FEAGIConnectionStatus = FEAGIConnectionStatus.DISCONNECTED

    @abc.abstractmethod
    async def disconnect(self) -> None:
        ...

    @abc.abstractmethod
    async def _send_bytes(self, sending: bytes) -> None:
        if self._status != FEAGIConnectionStatus.FEAGI_READY:
            raise Exception(f"Cannot send bytes when agent connector status is {self._status.name}!")
        ...

    def get_current_connectivity_status(self) -> FEAGIConnectionStatus:
        return self._status

    def subscribe_receive_bytes(self, callback: Callable[[bytes], Awaitable[None]]) -> None:
        self._receive_bytes_callbacks.append(callback)

    def subscribe_status_change(self, callback: Callable[[FEAGIConnectionStatus, FEAGIConnectionStatus], Awaitable[None]]) -> None: # old status, new status
        self._status_change_callbacks.append(callback)

    async def _received_bytes_from_server(self, received: bytes) -> None:
        for cb in self._receive_bytes_callbacks:
            cb(received)

    async def _change_reported_status(self, new_status: FEAGIConnectionStatus) -> None:
        if new_status == self._status:
            return # don't action on no change
        self._status = new_status
        for cb in self._status_change_callbacks:
            cb(new_status)