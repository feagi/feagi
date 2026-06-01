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


def test_decode_motor_xyzp_rotary_mot_signed_absolute_along_z():
    """RotaryMotor (mot): single column per channel; z=0 -> +1, z=8 -> -1 (depth 9)."""
    cid = _make_cortical_id(b"mot", data_type_flag=5, group=0)
    out_hi = decode_motor_xyzp(
        {cid: {"x": [0], "y": [0], "z": [0], "p": [100.0]}},
        [cid],
        include_groups=True,
    )
    assert out_hi["0:0:absolute"] == 1.0
    out_lo = decode_motor_xyzp(
        {cid: {"x": [0], "y": [0], "z": [8], "p": [100.0]}},
        [cid],
        include_groups=True,
    )
    assert out_lo["0:0:absolute"] == -1.0


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


def test_decode_motor_xyzp_unsigned_incremental_two_lane_decode():
    """
    Unsigned incremental motor areas use two X lanes per channel.

    Even X is forward lane, odd X is backward lane.
    """
    # Percentage + incremental + linear => variant=1, frame=1, pos=0
    cid = _make_cortical_id(b"pse", data_type_flag=(1 | (1 << 8)), group=4)
    xyzp = {
        cid: {
            "x": [0, 1],
            "y": [0, 0],
            "z": [0, 0],
            "p": [1.0, 0.0],
        }
    }
    out = decode_motor_xyzp(xyzp, [cid], include_groups=True)
    assert out["4:0:incremental"] == 1.0


def test_decode_motor_xyzp_unsigned_absolute_single_lane_decode():
    """
    Unsigned absolute positional servo uses one lane per channel (X=channel).
    """
    # Percentage + absolute + linear => variant=1, frame=0, pos=0
    cid = _make_cortical_id(b"pse", data_type_flag=1, group=5)
    xyzp = {
        cid: {
            "x": [0],
            "y": [0],
            "z": [0],
            "p": [1.0],
        }
    }
    out = decode_motor_xyzp(xyzp, [cid], include_groups=True)
    assert out["5:0:absolute"] == 1.0


def test_decode_motor_xyzp_unsigned_absolute_linear_span_hits_full_range():
    """Linear absolute decode should cover full signed range across Z bins."""
    cid = _make_cortical_id(b"pse", data_type_flag=1, group=6)

    # Top bin -> +1.0
    xyzp_hi = {cid: {"x": [0], "y": [0], "z": [0], "p": [1.0]}}
    out_hi = decode_motor_xyzp(xyzp_hi, [cid], include_groups=True)
    assert out_hi["6:0:absolute"] == 1.0

    # Bottom bin in 10-depth map -> -1.0
    xyzp_lo = {cid: {"x": [0], "y": [0], "z": [9], "p": [1.0]}}
    out_lo = decode_motor_xyzp(xyzp_lo, [cid], include_groups=True)
    assert out_lo["6:0:absolute"] == -1.0


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
