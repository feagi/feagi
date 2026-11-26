"""Single-source guardrail for motor decoding.

FEAGI motor decoding has exactly one implementation: the Rust ``MotorDeviceCache``
in feagi-core, surfaced to Python through ``feagi_rust_py_libs``. The Python SDK
must remain a thin transport + wrapper layer and must never re-implement motor
decode math (that path was retired so behaviour cannot diverge across SDKs).

These tests fail if motor-decode logic ever creeps back into the Python SDK, or
if ``brain_output.receive`` stops delegating decode to the Rust cache.
"""

from __future__ import annotations

import ast
from pathlib import Path
from typing import List

# Package root: <repo>/feagi-python-sdk/feagi
FEAGI_PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "feagi"

# Function names that represent SDK-side decode logic. Their presence anywhere in
# the Python package means decoding has leaked out of the Rust core.
FORBIDDEN_DECODER_FUNCTIONS = frozenset(
    {
        "decode_motor_xyzp",
        "decode_sensor_xyzp_to_grid",
        "decode_1d_array_xyzp",
        "_decode_signed_percentage_linear",
        "_decode_signed_percentage_fractional",
        "_decode_unsigned_percentage_linear",
        "_decode_unsigned_percentage_fractional",
    }
)


def _python_sources() -> List[Path]:
    """Return every ``.py`` file shipped inside the feagi package."""
    return sorted(FEAGI_PACKAGE_ROOT.rglob("*.py"))


def test_retired_python_decoder_module_is_absent() -> None:
    """The retired Python motor decoder module must not reappear."""
    retired = FEAGI_PACKAGE_ROOT / "pns" / "xyzp_decoders.py"
    assert not retired.exists(), (
        f"{retired} was retired: motor/sensor decode must go through the Rust "
        "MotorDeviceCache, not a Python module."
    )


def test_no_motor_decode_functions_defined_in_sdk() -> None:
    """No Python SDK module may define motor/sensor XYZP decode functions."""
    offenders: List[str] = []
    for source in _python_sources():
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                if node.name in FORBIDDEN_DECODER_FUNCTIONS:
                    rel = source.relative_to(FEAGI_PACKAGE_ROOT)
                    offenders.append(f"{rel}:{node.lineno} def {node.name}")

    assert not offenders, (
        "Motor/sensor decode logic must live only in the Rust core. Found "
        "forbidden decoder definitions in the Python SDK:\n  "
        + "\n  ".join(offenders)
    )


def test_brain_output_receive_delegates_decode_to_rust_cache() -> None:
    """``brain_output.receive`` must decode via the Rust cache, not in Python."""
    source = (FEAGI_PACKAGE_ROOT / "pns" / "brain_output.py").read_text(
        encoding="utf-8"
    )
    required_rust_calls = (
        "receive_motor_data_raw",
        "motors_load_in_bytes_and_verify",
        "motors_decode_cached_byte_data_to_motor",
        "motors_read_decoded_snapshot",
    )
    missing = [call for call in required_rust_calls if call not in source]
    assert not missing, (
        "brain_output.py must delegate motor decode to the Rust cache. Missing "
        f"required Rust calls: {missing}"
    )
