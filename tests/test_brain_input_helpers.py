"""
Tests for brain_input cortical helpers: decode_cortical_id_to_subtype,
register_cortical_areas_with_cache.
"""

import base64
import pytest

from feagi.pns.brain_input import decode_cortical_id_to_subtype


def test_decode_cortical_id_to_subtype_svi():
    """vision_LL (SegmentedVision) base64 -> b'svi'."""
    # aXN2aQkAAAA= decodes to isvi\x09\x00\x00\x00 (8 bytes)
    cortical_id = "aXN2aQkAAAA="
    result = decode_cortical_id_to_subtype(cortical_id)
    assert result == b"svi"


def test_decode_cortical_id_to_subtype_img():
    """Vision (simple) base64 -> b'img'."""
    # Cortical ID: byte 0=category, bytes 1-3=unit id. For img: b"i" + b"img"
    raw = b"i" + b"img" + b"\x00\x00\x00\x00"
    cortical_id = base64.b64encode(raw).decode("ascii")
    result = decode_cortical_id_to_subtype(cortical_id)
    assert result == b"img"


def test_decode_cortical_id_to_subtype_mis():
    """MiscData base64 -> b'mis'."""
    raw = b"i" + b"mis" + b"\x00\x00\x00\x00"
    cortical_id = base64.b64encode(raw).decode("ascii")
    result = decode_cortical_id_to_subtype(cortical_id)
    assert result == b"mis"


def test_decode_cortical_id_to_subtype_dpt():
    """DepthMap base64 -> b'dpt'."""
    raw = b"i" + b"dpt" + b"\x00\x00\x00\x00"
    cortical_id = base64.b64encode(raw).decode("ascii")
    result = decode_cortical_id_to_subtype(cortical_id)
    assert result == b"dpt"


def test_decode_cortical_id_to_subtype_too_short():
    """Raises ValueError for ID shorter than 4 bytes."""
    raw = b"abc"  # 3 bytes
    cortical_id = base64.b64encode(raw).decode("ascii")
    with pytest.raises(ValueError, match="at least 4 bytes"):
        decode_cortical_id_to_subtype(cortical_id)


def test_decode_cortical_id_to_subtype_invalid_base64():
    """Raises for invalid base64."""
    with pytest.raises(Exception):  # base64.b64decode raises
        decode_cortical_id_to_subtype("not-valid-base64!!")
