import asyncio
from src.feagi_interfaces.abstract_feagi_interface import AbstractFeagiInterface
from src.feagi_interfaces import FEAGIConnectionStatus

class DummyFeagiInterface(AbstractFeagiInterface):
    def __init__(self):
        super().__init__()

    async def disconnect(self) -> None:
        ...

    async def _send_bytes(self, sending: bytes) -> None:
        print(f"Sending {len(sending)} bytes over dummy connector!")


    async def connect(self) -> None:
        await self._change_reported_status(FEAGIConnectionStatus.CONNECTING)
        print("Connecting...")
        await asyncio.sleep(0.20)
        await self._change_reported_status(FEAGIConnectionStatus.AUTHENTICATING)
        print("Authenticating...")
        await asyncio.sleep(0.05)
        await self._change_reported_status(FEAGIConnectionStatus.FEAGI_READY)
        print("Connected!")



    # NOTE: There is no server to be receiving bytes from
    async def _received_bytes_from_server(self, received: bytes) -> None:
        await super()._received_bytes_from_server(received)