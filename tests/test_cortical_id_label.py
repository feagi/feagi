"""Tests for decode_cortical_id_label (human-readable cortical ID labels)."""

import base64

import pytest

pytest.importorskip("feagi_rust_py_libs", reason="feagi_rust_py_libs required")

from feagi.pns.client import decode_cortical_id_label


def _make_id(category_char: str, subtype: bytes, extra: bytes = b"\x00\x00\x00\x00") -> str:
    """Build a base64 cortical ID from category char + 3-byte subtype."""
    raw = category_char.encode("ascii") + subtype + extra
    return base64.b64encode(raw).decode("ascii")


class TestDecodeCorticalIdLabel:
    """Verify human-readable labels for known and unknown cortical IDs."""

    def test_spatial_pointer_output(self):
        """The exact ID from the xARM error report."""
        label = decode_cortical_id_label("b3B0cgMAAAI=")
        assert "Spatial Pointer" in label
        assert "output" in label
        assert "b3B0cgMAAAI=" in label

    def test_positional_servo_output(self):
        cid = _make_id("o", b"pse")
        label = decode_cortical_id_label(cid)
        assert "Positional Servo" in label
        assert "output" in label

    def test_rotary_motor_output(self):
        cid = _make_id("o", b"mot")
        label = decode_cortical_id_label(cid)
        assert "Rotary Motor" in label
        assert "output" in label

    def test_segmented_vision_input(self):
        cid = _make_id("i", b"svi")
        label = decode_cortical_id_label(cid)
        assert "Segmented Vision" in label
        assert "input" in label

    def test_shared_subtype_misc_input(self):
        cid = _make_id("i", b"mis")
        label = decode_cortical_id_label(cid)
        assert "Miscellaneous Sensor" in label

    def test_shared_subtype_misc_output(self):
        cid = _make_id("o", b"mis")
        label = decode_cortical_id_label(cid)
        assert "Miscellaneous Motor" in label

    def test_unknown_subtype_still_readable(self):
        cid = _make_id("o", b"zzz")
        label = decode_cortical_id_label(cid)
        assert "zzz" in label
        assert "output" in label
        assert cid in label

    def test_invalid_base64_returns_input(self):
        label = decode_cortical_id_label("===")
        assert label == "==="

    def test_too_short_returns_input(self):
        short = base64.b64encode(b"ab").decode()
        label = decode_cortical_id_label(short)
        assert label == short

    @pytest.mark.parametrize(
        "subtype,cat,expected_name",
        [
            (b"inf", "i", "Infrared Sensor"),
            (b"bat", "i", "Battery Sensor"),
            (b"rim", "i", "Raw IMU"),
            (b"sim", "i", "Smart IMU"),
            (b"gaz", "o", "Gaze Control"),
            (b"ptr", "o", "Spatial Pointer"),
            (b"cnt", "i", "Count Input"),
            (b"cnt", "o", "Count Output"),
            (b"ten", "i", "Text Input (English)"),
            (b"ten", "o", "Text Output (English)"),
        ],
    )
    def test_known_subtypes(self, subtype: bytes, cat: str, expected_name: str):
        cid = _make_id(cat, subtype)
        label = decode_cortical_id_label(cid)
        assert expected_name in label
