"""Tests for RGBD helper APIs in BrainOutput."""

from types import SimpleNamespace

import numpy as np

from feagi.pns.brain_output import BrainOutput


class _FakeCache:
    """Minimal cache stub for RGBD registration/write tests."""

    def __init__(self) -> None:
        self.calls = []

    def sensor_Vision_register(self, group, count, frame_mode, image_props):
        self.calls.append(("Vision", group, count, frame_mode, image_props))

    def sensor_DepthMap_register(self, group, count, frame_mode, depth_dims):
        self.calls.append(("DepthMap", group, count, frame_mode, depth_dims))

    def sensor_depth_map_write(self, *, group, channel_index, data):
        self.calls.append(("write_depth", group, channel_index, data))


def _install_fake_frpl(monkeypatch):
    """Install a fake `feagi_rust_py_libs` module into sys.modules."""

    class _FrameChangeHandling:
        @staticmethod
        def Absolute():
            return "ABS"

        @staticmethod
        def Incremental():
            return "INC"

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

    class _Positioning:
        @staticmethod
        def Linear():
            return "LIN"

    fake = SimpleNamespace(
        data_structures=SimpleNamespace(
            genomic=SimpleNamespace(
                cortical_area=SimpleNamespace(
                    FrameChangeHandling=_FrameChangeHandling,
                    PercentageNeuronPositioning=_Positioning,
                ),
            ),
        ),
        connector_core=SimpleNamespace(
            data_types=SimpleNamespace(
                descriptors=_Descriptors,
            ),
        ),
    )
    monkeypatch.setitem(__import__("sys").modules, "feagi_rust_py_libs", fake)


def test_register_sensor_units_supports_depth_map(monkeypatch):
    """DepthMap can be registered through standard sensor helper API."""
    _install_fake_frpl(monkeypatch)

    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._cache_available = True

    groups = bo.register_sensor_units(
        {"DepthMap": 2},
        z_neuron_resolution=10,
        group_index_start=4,
        misc_dimensions_xyz=(64, 48, 32),
    )

    assert groups == {"DepthMap": 4}
    assert bo._cache.calls[0][0:4] == (
        "DepthMap",
        4,
        2,
        "ABS",
    )
    dims = bo._cache.calls[0][4]
    assert (dims.x, dims.y, dims.z) == (64, 48, 32)


def test_register_rgbd_sensor_pair_sets_bundle_metadata(monkeypatch):
    """RGBD helper pairs Vision and DepthMap with shared bundle metadata."""
    _install_fake_frpl(monkeypatch)

    bo = BrainOutput()
    bo._cache = _FakeCache()
    bo._cache_available = True

    groups = bo.register_rgbd_sensor_pair(
        rgb_group=3,
        depth_group=9,
        rgb_resolution_xy=(320, 240),
        depth_dimensions_xyz=(320, 240, 64),
        bundle_id="front_rgbd",
    )
    assert groups == {"Vision": 3, "DepthMap": 9}

    payload = {
        "input_units_and_encoder_properties": {
            "Vision": [[{
                "friendly_name": None,
                "cortical_unit_index": 3,
                "device_grouping": [
                    {"friendly_name": None, "device_properties": {}},
                ],
            }, {}]],
            "DepthMap": [[{
                "friendly_name": None,
                "cortical_unit_index": 9,
                "device_grouping": [
                    {"friendly_name": None, "device_properties": {}},
                ],
            }, {}]],
        },
    }
    enriched = bo._device_registration_enricher(payload)
    vision = enriched["input_units_and_encoder_properties"]["Vision"][0][0]
    depth = enriched["input_units_and_encoder_properties"]["DepthMap"][0][0]

    assert vision["friendly_name"] == "front_rgbd_rgb"
    assert depth["friendly_name"] == "front_rgbd_depth"
    assert (
        vision["device_grouping"][0]["device_properties"]["bundle_type"]["value"]
        == "rgbd_camera"
    )
    assert (
        depth["device_grouping"][0]["device_properties"]["sensor_role"]["value"]
        == "depth"
    )


def test_rgb_frame_to_depth_map_bins_returns_one_hot_volume():
    """RGB luminance conversion produces deterministic one-hot depth bins."""
    frame = np.array(
        [[[0, 0, 0], [255, 255, 255]]],
        dtype=np.uint8,
    )
    depth = BrainOutput.rgb_frame_to_depth_map_bins(frame, 8)
    assert depth.shape == (1, 2, 8)
    assert depth[0, 0, 0] == 1.0
    assert depth[0, 1, 7] == 1.0
    assert float(np.sum(depth)) == 2.0


def test_write_rgbd_tick_writes_both_streams(monkeypatch):
    """One RGBD tick writes paired Vision + DepthMap channels."""
    bo = BrainOutput()
    calls = []

    def fake_write_vision_frame(*, group, channel_index, frame_rgb):
        calls.append(("vision", group, channel_index, frame_rgb.shape))

    def fake_write_depth_map(*, group, channel_index, depth_map_xyz):
        calls.append(("depth", group, channel_index, depth_map_xyz.shape))

    monkeypatch.setattr(
        bo,
        "write_sensor_vision_frame",
        fake_write_vision_frame,
    )
    monkeypatch.setattr(bo, "write_sensor_depth_map", fake_write_depth_map)

    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    bo.write_rgbd_tick(
        rgb_group=1,
        depth_group=5,
        channel_index=0,
        frame_rgb=frame,
        depth_bins=4,
    )

    assert calls[0] == ("vision", 1, 0, (2, 2, 3))
    assert calls[1] == ("depth", 5, 0, (2, 2, 4))
