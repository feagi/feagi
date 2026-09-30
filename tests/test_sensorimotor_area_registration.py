"""SDK registration for audio, pose, and simple vision output areas."""

from types import SimpleNamespace

from feagi.pns.brain_output import BrainOutput


class _FrameChangeHandling:
    @staticmethod
    def Absolute():
        return "ABS"

    @staticmethod
    def Incremental():
        return "INC"


class _AudioProperties:
    def __init__(self, args):
        self.args = args

    @staticmethod
    def new_linear(*args):
        return _AudioProperties(args)


class _PoseSchema:
    @staticmethod
    def HumanBody():
        return "HumanBody"


class _PoseProperties:
    def __init__(self, width, height, depth):
        self.width = width
        self.height = height
        self.depth = depth


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


class _Cache:
    def __init__(self):
        self.calls = []

    def sensor_AudioInput_register(self, group, count, frame_mode, audio_properties):
        self.calls.append(("audio_in", group, count, frame_mode, audio_properties.args))

    def sensor_audio_input_write(self, *, group, channel_index, data):
        self.calls.append(("audio_write", group, channel_index, data))

    def motor_AudioOutput_register(self, group, count, frame_mode, audio_properties):
        self.calls.append(("audio_out", group, count, frame_mode, audio_properties.args))

    def motor_audio_output_read_postprocessed_cache_value(self, group, channel_index):
        self.calls.append(("audio_read", group, channel_index))
        return "spectrum"

    def motor_SimpleVisionOutput_register(self, group, count, frame_mode, image_properties):
        self.calls.append(
            (
                "vision_out",
                group,
                count,
                frame_mode,
                image_properties.resolution.x,
                image_properties.resolution.y,
            )
        )

    def motor_simple_vision_output_read_postprocessed_cache_value(self, group, channel_index):
        self.calls.append(("vision_read", group, channel_index))
        return "image"

    def motor_PoseEstimation_register(
        self, group, count, frame_mode, pose_schema, pose_properties
    ):
        self.calls.append(
            (
                "pose",
                group,
                count,
                frame_mode,
                pose_schema,
                pose_properties.width,
                pose_properties.depth,
            )
        )

    def motor_pose_estimation_read_postprocessed_cache_value(self, group, channel_index):
        self.calls.append(("pose_read", group, channel_index))
        return "joints"


def _install(monkeypatch):
    fake = SimpleNamespace(
        data_structures=SimpleNamespace(
            genomic=SimpleNamespace(
                cortical_area=SimpleNamespace(
                    FrameChangeHandling=_FrameChangeHandling,
                    PercentageNeuronPositioning=type(
                        "PercentageNeuronPositioning",
                        (),
                        {"Linear": staticmethod(lambda: "LIN")},
                    ),
                    PoseSchema=_PoseSchema,
                )
            )
        ),
        connector_core=SimpleNamespace(
            data_types=SimpleNamespace(
                AudioSpectrumProperties=_AudioProperties,
                PoseEstimationProperties=_PoseProperties,
                descriptors=_Descriptors,
            )
        ),
    )
    monkeypatch.setitem(__import__("sys").modules, "feagi_rust_py_libs", fake)


def _brain():
    brain = BrainOutput()
    brain._cache = _Cache()
    brain._cache_available = True
    return brain


def test_register_audio_input(monkeypatch):
    _install(monkeypatch)
    brain = _brain()
    groups = brain.register_sensor_units({"AudioInput": 2}, z_neuron_resolution=10)
    assert groups == {"AudioInput": 0}
    assert brain._cache.calls[0][0] == "audio_in"
    assert brain._cache.calls[0][4] == (16000, 1024, 512, 16, -80, 0)
    brain.write_sensor_audio_spectrum(group=0, channel_index=1, frame="frame")
    assert ("audio_write", 0, 1, "frame") in brain._cache.calls


def test_register_audio_pose_and_vision_outputs(monkeypatch):
    _install(monkeypatch)
    brain = _brain()
    brain.register_motor_audio_output(group=3, number_channels=1)
    brain.register_motor_simple_vision_output(group=4, width=64, height=48)
    brain.register_motor_pose_estimation(group=5, width=32, height=32, depth=17)
    assert brain.read_motor_audio_spectrum(group=3, channel_index=0) == "spectrum"
    assert brain.read_motor_simple_vision_output(group=4, channel_index=0) == "image"
    assert brain.read_motor_pose_estimation(group=5, channel_index=0) == "joints"
    kinds = [call[0] for call in brain._cache.calls]
    assert kinds == [
        "audio_out",
        "vision_out",
        "pose",
        "audio_read",
        "vision_read",
        "pose_read",
    ]
