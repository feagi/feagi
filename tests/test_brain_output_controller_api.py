"""Tests for controller-facing BrainOutput helper APIs."""

from types import SimpleNamespace

from feagi.pns.brain_output import BrainOutput


class _FakeCache:
    """Minimal cache stub for controller API tests."""

    def __init__(self) -> None:
        self.calls = []
        self._encoded = b"encoded"

    def sensor_Vision_register(self, group, count, frame_mode, image_props):
        self.calls.append(("Vision", group, count, frame_mode, image_props))

    def sensor_Proximity_register(self, group, count, frame_mode, z_res, positioning):
        self.calls.append(("Proximity", group, count, frame_mode, z_res, positioning))

    def sensor_Shock_register(self, group, count, frame_mode, z_res, positioning):
        self.calls.append(("Shock", group, count, frame_mode, z_res, positioning))

    def sensor_MiscData_register(self, group, count, frame_mode, dims):
        self.calls.append(("MiscData", group, count, frame_mode, dims))

    def sensor_proximity_write(self, *, group, channel_index, data):
        self.calls.append(("write_proximity", group, channel_index, data))

    def sensor_misc_data_write(self, *, group, channel_index, data):
        self.calls.append(("write_misc", group, channel_index, data))

    def sensors_encode_cached_sensor_data_to_bytes(self):
        self.calls.append(("encode",))

    def sensors_read_bytes(self):
        return self._encoded


class _FakeClient:
    """Minimal client stub for sensory send tests."""

    def __init__(self) -> None:
        self.sent = []

    def send_sensory_bytes(self, payload: bytes) -> None:
        self.sent.append(payload)


def _install_fake_frpl(monkeypatch):
    """Install a fake `feagi_rust_py_libs` module into sys.modules."""

    class _FrameChangeHandling:
        @staticmethod
        def Absolute():
            return "ABS"

    class _Positioning:
        @staticmethod
        def Linear():
            return "LIN"

    class _Descriptors:
        class ImageXYResolution:
            def __init__(self, x, y):
                self.x = x
                self.y = y

        class ColorSpace:
            Gamma = "Gamma"

        class ColorChannelLayout:
            RGB = "RGB"

        class ImageFrameProperties:
            def __init__(self, resolution, color_space, channel_layout):
                self.resolution = resolution
                self.color_space = color_space
                self.channel_layout = channel_layout

        class MiscDataDimensions:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z

    fake = SimpleNamespace(
        data_structures=SimpleNamespace(
            genomic=SimpleNamespace(
                cortical_area=SimpleNamespace(
                    FrameChangeHandling=_FrameChangeHandling,
                    PercentageNeuronPositioning=_Positioning,
                )
            )
        ),
        connector_core=SimpleNamespace(
            data_types=SimpleNamespace(
                descriptors=_Descriptors,
            )
        ),
    )
    monkeypatch.setitem(__import__("sys").modules, "feagi_rust_py_libs", fake)


def test_register_sensor_units_deterministic_groups(monkeypatch):
    """Sensor units register through SDK without controller Rust imports."""
    _install_fake_frpl(monkeypatch)

    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._cache_available = True

    groups = bo.register_sensor_units(
        {
            "Shock": 2,
            "Proximity": 3,
        },
        z_neuron_resolution=10,
    )

    assert groups == {"Proximity": 0, "Shock": 1}
    assert ("Proximity", 0, 3, "ABS", 10, "LIN") in bo._cache.calls
    assert ("Shock", 1, 2, "ABS", 10, "LIN") in bo._cache.calls


def test_write_and_flush_sensory_bytes_without_direct_rust_imports(monkeypatch):
    """Controller can write sensory scalars and flush using SDK wrappers."""
    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._cache_available = True
    bo._client = _FakeClient()

    class _PctFactory:
        @staticmethod
        def new_from_0_1(value):
            return ("pct", value)

    class _MiscFactory:
        @staticmethod
        def new_from_array(arr):
            return ("misc", float(arr[0, 0, 0]))

    def _fake_init_helpers():
        bo._sensory_percentage_factory = _PctFactory
        bo._sensory_misc_factory = _MiscFactory

    monkeypatch.setattr(bo, "_init_sensory_write_helpers", _fake_init_helpers)

    bo.write_sensor_scalar(unit_key="Proximity", group=0, channel_index=4, scalar_0_1=0.25)
    bo.write_sensor_scalar(unit_key="MiscData", group=2, channel_index=1, scalar_0_1=0.75)
    sent_len = bo.flush_sensory_bytes()

    assert ("write_proximity", 0, 4, ("pct", 0.25)) in bo._cache.calls
    assert ("write_misc", 2, 1, ("misc", 0.75)) in bo._cache.calls
    assert sent_len == len(b"encoded")
    assert bo._client.sent == [b"encoded"]
