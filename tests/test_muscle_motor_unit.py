"""Muscle unit: a channel without a spike decodes to 0.0, and the newest drained frame wins."""

import pytest

pytest.importorskip("feagi_rust_py_libs")
import feagi_rust_py_libs as frpl

from feagi.pns.brain_output import BrainOutput

_DESCRIPTOR_B64 = "A" * 64
_DEPTH = 10
# Percentage, Absolute, Linear flags; unit index 0.
_MUSCLE_ID = b"omus" + bytes([1, 0, 0, 0])


class _Client:
    def __init__(self, frames):
        self._frames = list(frames)

    def receive_motor_data_raw(self):
        return self._frames.pop(0) if self._frames else b""


def _frame(spikes):
    """Serialized motor frame; ``spikes`` is [(channel, z)], None = no Muscle area at all."""
    xyzp = frpl.data_structures.neurons_voxels.xyzp
    mapped = xyzp.CorticalMappedXYZPNeuronVoxels.new_with_capacity(1)
    if spikes is not None:
        arrays = xyzp.PyNeuronVoxelXYZPArrays()
        for x, z in spikes:
            arrays.push(xyzp.PyNeuronVoxelXYZP(x, 0, z, 1.0))
        cid = frpl.data_structures.genomic.cortical_area.CorticalID.try_from_bytes(
            _MUSCLE_ID
        )
        mapped.insert(cid, arrays)
    container = frpl.data_serialization.FeagiByteContainer()
    container.add_struct(mapped, 0)
    return bytes(container.copy_out_as_byte_vector())


@pytest.fixture
def bo(monkeypatch):
    monkeypatch.setenv("FEAGI_AGENT_DESCRIPTOR_B64", _DESCRIPTOR_B64)
    out = BrainOutput()
    out.register_motor_groups(
        {0: {"muscle": ["m0", "m1", "m2"]}}, z_neuron_resolution=_DEPTH
    )
    out._connected = True
    out._sensory_only_mode = False
    return out


def _muscle(bo):
    return bo.read_decoded_motor_values("muscle", 0)


def test_muscle_reads_zero_on_silent_frame(bo):
    bo._client = _Client([_frame([(1, 0)]), _frame(None)])
    bo.receive()
    fired = _muscle(bo)
    assert fired[(0, 1, "absolute")] == pytest.approx(1.0)
    bo.receive()
    silent = _muscle(bo)
    assert silent[(0, 1, "absolute")] == 0.0
    assert bo.read_muscle_excitations(0)[1] == 0.0


def test_drain_motor_frames_newest_silent_frame_wins(bo):
    bo._client = _Client([_frame([(1, 0)]), _frame(None)])
    assert bo.drain_motor_frames() == 2
    assert _muscle(bo)[(0, 1, "absolute")] == 0.0
    assert bo.read_muscle_excitations(0)[1] == 0.0
