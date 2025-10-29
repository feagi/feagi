from typing import Callable, Awaitable, TypeVar, Generic
import asyncio

T = TypeVar("T")

class AsyncSignal:
    def __init__(self):
        self._callbacks: list[tuple[Callable[[], Awaitable[None]], asyncio.Lock]] = []

    def register(self, callback: Callable[[], Awaitable[None]]) -> None:
        """Register a callback with its own lock to prevent overlapping calls."""
        self._callbacks.append((callback, asyncio.Lock()))

    async def emit(self) -> None:
        """Invoke all callbacks concurrently, skipping ones still running."""
        async def run(callback: Callable[[], Awaitable[None]], lock: asyncio.Lock):
            if lock.locked():
                # Skip overlapping
                return
            async with lock:
                await callback()

        await asyncio.gather(*(run(callback, lock) for callback, lock in self._callbacks))

class AsyncSignalWithParam(Generic[T]):
    def __init__(self):
        self._callbacks: list[tuple[Callable[[T], Awaitable[None]], asyncio.Lock]] = []

    def register(self, callback: Callable[[T], Awaitable[None]]) -> None:
        """Register a callback with its own lock to prevent overlapping calls."""
        self._callbacks.append((callback, asyncio.Lock()))

    async def emit(self, value: T) -> None:
        """Invoke all callbacks concurrently, skipping ones still running."""
        async def run(callback: Callable[[T], Awaitable[None]], lock: asyncio.Lock):
            if lock.locked():
                # Skip overlapping
                return
            async with lock:
                await callback(value)

        await asyncio.gather(*(run(callback, lock) for callback, lock in self._callbacks))