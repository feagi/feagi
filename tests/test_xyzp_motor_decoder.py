"""Tests for motor XYZP decoding semantics."""

from feagi.pns.xyzp_decoders import decode_motor_xyzp


def _make_cortical_id(unit: bytes, data_type_flag: int, group: int) -> str:
    import base64

    b = bytes(
        [
            ord("o"),
            unit[0],
            unit[1],
            unit[2],
            data_type_flag & 0xFF,
            (data_type_flag >> 8) & 0xFF,
            0,
            group & 0xFF,
        ]
    )
    return base64.b64encode(b).decode("ascii")


def test_decode_motor_xyzp_signed_linear_endpoints():
    """Signed linear decoding should map endpoint z bins to full range."""
    # SignedPercentage + absolute + linear => variant=5, frame=0, pos=0
    cid = _make_cortical_id(b"pse", data_type_flag=5, group=0)

    # Channel 0 positive lane (x=0), z=0 -> +1.0
    xyzp = {
        cid: {
            "x": [0],
            "y": [0],
            "z": [0],
            "p": [1.0],
        }
    }
    out = decode_motor_xyzp(xyzp, [cid], include_groups=True)
    assert out["0:0:absolute"] == 1.0

    # Channel 0 negative lane (x=1), z=0 -> -1.0
    xyzp = {
        cid: {
            "x": [1],
            "y": [0],
            "z": [0],
            "p": [1.0],
        }
    }
    out = decode_motor_xyzp(xyzp, [cid], include_groups=True)
    assert out["0:0:absolute"] == -1.0


def test_decode_motor_xyzp_signed_incremental_keys():
    """Signed incremental IDs should produce incremental command keys."""
    cid = _make_cortical_id(b"pse", data_type_flag=(5 | (1 << 8)), group=3)
    xyzp = {
        cid: {
            "x": [0],
            "y": [0],
            "z": [0],
            "p": [1.0],
        }
    }
    out = decode_motor_xyzp(xyzp, [cid], include_groups=True)
    assert out["3:0:incremental"] == 1.0


def test_decode_motor_xyzp_legacy_p_scaling():
    """Fallback path supports legacy p-based scaling."""
    cid = _make_cortical_id(b"mis", data_type_flag=10, group=2)
    xyzp = {
        cid: {
            "x": [0, 1],
            "y": [0, 0],
            "z": [0, 0],
            "p": [50.0, -100.0],
        }
    }
    out = decode_motor_xyzp(xyzp, [cid], include_groups=True)
    assert out["2:0"] == 0.5
    assert out["2:1"] == -1.0
