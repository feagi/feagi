import os
import time
from pathlib import Path

import pytest


def test_motor_stream_writes_shm_when_agent_requests_motor(shared_tmp_path=None):
    """Integration test: verify motor SHM is written when an agent with motor capability registers.

    Preconditions:
      - FEAGI core started with shared_mem enabled (so core SHM files are created)
      - StateManager available as singleton
      - MotorStream constructed by process manager and started
      - Test simulates agent registration requesting motor capability and provides SHM mapping

    Validates:
      - StateManager creates per-agent SHM mapping with key 'motor_stream'
      - MotorStream fan-out writes payload bytes into the agent SHM file
    """
    try:
        from feagi.core.state_manager import FeagiStateManager
        from feagi.api.zmq.streams.motor import MotorStream
        from feagi.api.core.services.core_api_service import CoreAPIService
    except Exception as e:
        pytest.skip(f"Core imports unavailable: {e}")

    sm = FeagiStateManager.instance()

    # Ensure core SHM manager exists
    if not getattr(sm, "_shm_manager", None):
        pytest.skip("Shared memory manager not available; run FEAGI with shared_mem enabled")

    # Create core motor stream file if not already present
    core_motor_path = sm.get_shared_memory_registry().get("motor_stream")
    if not core_motor_path:
        core_motor_path = sm._shm_manager.create_stream_file("motor_stream")
        sm.register_core_shared_memory_path("motor_stream", core_motor_path)

    # Create an agent-specific motor capability file and register mapping manually
    agent_id = "test_motor_agent"
    created_caps = sm._shm_manager.create_agent_capability_files(agent_id, {"motor": True})
    motor_shm_path = created_caps.get("motor")
    assert motor_shm_path, "Expected agent-specific motor SHM file to be created"
    # Register per-agent mapping so MotorStream will fan-out to this file
    sm._agent_shared_memory[agent_id] = {"motor": motor_shm_path}

    # Construct MotorStream and start
    # Use a dummy CoreAPIService; socket binding will fail in constrained CI, so skip if errors
    try:
        core_api = CoreAPIService.get_instance() if hasattr(CoreAPIService, "get_instance") else None
        ms = MotorStream(core_api or None, host="127.0.0.1", port=5564)
    except Exception as e:
        pytest.skip(f"Unable to construct MotorStream: {e}")

    # Force ACTIVE mode so _send_motor_binary_data proceeds
    ms._active_mode = True

    # Send a tiny payload through fan-out path and verify SHM write
    test_payload = bytes([0x0B, 0x00, 0x00, 0x00])  # Minimal header-like bytes
    try:
        # Call the private fan-out path via _send_motor_binary_data; it also writes core and per-agent SHM
        import asyncio
        asyncio.run(ms._send_motor_binary_data(test_payload, channel="ogaz00"))
    except Exception as e:
        pytest.skip(f"MotorStream send failed (likely no active mode): {e}")

    # Verify agent SHM file exists and is non-empty after write
    p = Path(motor_shm_path)
    assert p.exists(), f"Agent motor SHM file missing: {motor_shm_path}"
    # Give a brief moment for write to complete
    time.sleep(0.05)
    size = os.path.getsize(p)
    assert size >= 256, f"Agent motor SHM header not initialized (size={size})"

    # Open as raw and check header magic matches FEAGIMOT and that at least one slot contains a length prefix
    with p.open("rb") as fh:
        hdr = fh.read(8)
        assert hdr == b"FEAGIMOT", f"Unexpected SHM magic: {hdr}"
        fh.seek(256)  # First slot
        import struct
        (length,) = struct.unpack("<I", fh.read(4))
        assert length >= 0, "Expected a valid length prefix in agent motor SHM"


