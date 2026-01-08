from __future__ import annotations

from pathlib import Path

import pytest


def _find_gpt2_tokenizer_json() -> Path | None:
    # Prefer the vendored tokenizer in feagi-desktop (monorepo).
    repo_root = Path(__file__).resolve().parents[2]
    candidate = (
        repo_root
        / "feagi-desktop"
        / "src-tauri"
        / "resources"
        / "tokenizers"
        / "gpt2"
        / "tokenizer.json"
    )
    return candidate if candidate.exists() else None


def test_text_token_codec_roundtrip_misc_data() -> None:
    try:
        import feagi_rust_py_libs as frpl
    except ImportError:
        pytest.skip("feagi_rust_py_libs not installed")

    depth = 16
    token_id = 42
    data_types = frpl.connector_core.data_types
    misc = data_types.TextTokenCodec.encode_to_misc_data(token_id, depth)
    decoded = data_types.TextTokenCodec.decode_from_misc_data(misc)
    assert decoded == token_id


def test_gpt2_tokenizer_encode_decode_smoke() -> None:
    try:
        import feagi_rust_py_libs as frpl
    except ImportError:
        pytest.skip("feagi_rust_py_libs not installed")

    tok_path = _find_gpt2_tokenizer_json()
    if tok_path is None:
        pytest.skip("vendored GPT-2 tokenizer.json not found in monorepo")

    tok = frpl.connector_core.data_types.Gpt2Tokenizer.from_file(str(tok_path))
    ids = tok.encode("Hello world")
    assert isinstance(ids, list)
    assert all(isinstance(x, int) for x in ids)
    out = tok.decode(ids, True)
    assert isinstance(out, str)
    assert out != ""


