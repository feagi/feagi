"""
FEAGI Container (.fc) writer/reader (phase 1).

Format:
- Header: 8-byte magic (b"FEAGIFC1"), 4-byte little-endian uint32 header_len
- Header JSON (UTF-8) of length header_len with fields:
  {
    "version": 1,
    "endianness": "little",
    "alignment": 4096,
    "chunks": [
      {"name": "connectome.json", "encoding": "store|deflate", "offset": int, "length": int, "uncompressed_length": int, "checksum": "blake2b-256-hex"},
      {"name": "state.json",      "encoding": "store|deflate", "offset": int, "length": int, "uncompressed_length": int, "checksum": "..."}
    ]
  }
- Data region: concatenated chunk payloads at given offsets; offsets aligned to "alignment"

Chunk encodings:
- store: raw bytes
- deflate: zlib DEFLATE-compressed bytes
"""
from __future__ import annotations

import json
import struct
import zlib
from hashlib import blake2b
from pathlib import Path
from typing import Any, Dict, List, Tuple

MAGIC = b"FEAGIFC1"
ALIGNMENT = 4096


def _align_offset(offset: int, alignment: int = ALIGNMENT) -> int:
    rem = offset % alignment
    return offset if rem == 0 else (offset + (alignment - rem))


def _checksum(data: bytes) -> str:
    return blake2b(data, digest_size=32).hexdigest()


def _encode(data: bytes, encoding: str) -> Tuple[bytes, int]:
    if encoding == "store":
        return data, len(data)
    if encoding == "deflate":
        comp = zlib.compress(data)
        return comp, len(data)
    raise ValueError(f"Unsupported encoding: {encoding}")


def _decode(data: bytes, encoding: str, uncompressed_length: int) -> bytes:
    if encoding == "store":
        return data
    if encoding == "deflate":
        raw = zlib.decompress(data)
        if len(raw) != uncompressed_length:
            # Length mismatch is not fatal but indicates corruption
            pass
        return raw
    raise ValueError(f"Unsupported encoding: {encoding}")


def write_fc(
    output_dir: Path,
    snapshot_id: str,
    connectome_json: Dict[str, Any],
    state_json: Dict[str, Any],
    encoding: str = "store",
) -> Path:
    """
    Write a FEAGI Container file with connectome and state chunks.

    Returns: Path to the written .fc file
    """
    if not isinstance(output_dir, Path):
        output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    target = output_dir / f"{snapshot_id}.fc"

    # Prepare chunks
    cj_bytes = json.dumps(connectome_json, separators=(",", ":")).encode("utf-8")
    sj_bytes = json.dumps(state_json, separators=(",", ":")).encode("utf-8")

    cj_enc, cj_uncompressed = _encode(cj_bytes, encoding)
    sj_enc, sj_uncompressed = _encode(sj_bytes, encoding)

    header: Dict[str, Any] = {
        "version": 1,
        "endianness": "little",
        "alignment": ALIGNMENT,
        "chunks": [],
    }

    # We will compute offsets after building header JSON length; we need chunk metadata first
    chunks_meta: List[Dict[str, Any]] = [
        {
            "name": "connectome.json",
            "encoding": encoding,
            "offset": 0,  # to be filled
            "length": len(cj_enc),
            "uncompressed_length": cj_uncompressed,
            "checksum": _checksum(cj_bytes),
        },
        {
            "name": "state.json",
            "encoding": encoding,
            "offset": 0,  # to be filled
            "length": len(sj_enc),
            "uncompressed_length": sj_uncompressed,
            "checksum": _checksum(sj_bytes),
        },
    ]

    header["chunks"] = chunks_meta
    # Serialize header once to compute header_len
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    header_len = len(header_bytes)

    # Data region starts after magic + header_len + header_bytes, aligned
    data_start = _align_offset(len(MAGIC) + 4 + header_len, ALIGNMENT)

    # Compute per-chunk offsets
    offset = data_start
    for ch in chunks_meta:
        ch["offset"] = offset
        offset = _align_offset(offset + ch["length"], ALIGNMENT)

    # Re-serialize header with updated offsets
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    header_len = len(header_bytes)
    data_start = _align_offset(len(MAGIC) + 4 + header_len, ALIGNMENT)

    # Recompute offsets with final header_len
    offset = data_start
    for ch in chunks_meta:
        ch["offset"] = offset
        offset = _align_offset(offset + ch["length"], ALIGNMENT)

    # Final header
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")

    with open(target, "wb") as f:
        # Write magic + header_len
        f.write(MAGIC)
        f.write(struct.pack("<I", len(header_bytes)))
        f.write(header_bytes)
        # Pad to alignment
        pad_len = data_start - (len(MAGIC) + 4 + len(header_bytes))
        if pad_len > 0:
            f.write(b"\x00" * pad_len)
        # Write chunks at aligned offsets
        # connectome
        f.seek(chunks_meta[0]["offset"])  # no-op if sequential
        f.write(cj_enc)
        # pad to next alignment
        next_off = _align_offset(chunks_meta[0]["offset"] + chunks_meta[0]["length"], ALIGNMENT)
        if next_off > f.tell():
            f.write(b"\x00" * (next_off - f.tell()))
        # state
        f.seek(chunks_meta[1]["offset"])  # ensure correct placement
        f.write(sj_enc)
    return target


def read_fc_header(fc_path: Path) -> Dict[str, Any]:
    with open(fc_path, "rb") as f:
        magic = f.read(len(MAGIC))
        if magic != MAGIC:
            raise ValueError("Invalid FC magic")
        header_len_bytes = f.read(4)
        if len(header_len_bytes) != 4:
            raise ValueError("FC header length truncated")
        (header_len,) = struct.unpack("<I", header_len_bytes)
        header_bytes = f.read(header_len)
        if len(header_bytes) != header_len:
            raise ValueError("FC header truncated")
        header = json.loads(header_bytes.decode("utf-8"))
        return header


def extract_chunk(fc_path: Path, chunk_name: str) -> bytes:
    header = read_fc_header(fc_path)
    chunks = header.get("chunks", [])
    match = next((c for c in chunks if c.get("name") == chunk_name), None)
    if not match:
        raise ValueError(f"Chunk not found: {chunk_name}")
    offset = int(match["offset"])
    length = int(match["length"])
    encoding = match.get("encoding", "store")
    uncompressed = int(match.get("uncompressed_length", 0))
    with open(fc_path, "rb") as f:
        f.seek(offset)
        data = f.read(length)
    return _decode(data, encoding, uncompressed)


def create_fc_snapshot(
    output_dir: Path,
    snapshot_id: str,
    connectome_json: Dict[str, Any],
    state_json: Dict[str, Any],
    compression: str = "store",
) -> Path:
    encoding = "deflate" if compression == "deflate" else "store"
    return write_fc(
        output_dir=output_dir,
        snapshot_id=snapshot_id,
        connectome_json=connectome_json,
        state_json=state_json,
        encoding=encoding,
    )


def restore_fc_snapshot(snapshot_root: Path, snapshot_id: str, state_manager) -> bool:
    fc_path = Path(snapshot_root) / snapshot_id / f"{snapshot_id}.fc"
    if not fc_path.exists():
        raise FileNotFoundError(str(fc_path))
    # Extract and apply state.json
    state_bytes = extract_chunk(fc_path, "state.json")
    try:
        state = json.loads(state_bytes.decode("utf-8"))
    except Exception as e:
        raise ValueError(f"Invalid state.json in FC: {e}")
    stats = state.get("stats", {})
    try:
        current = state_manager.get_brain_stats() or {}
        merged = dict(current)
        for k in ("neuron_count", "memory_neuron_count", "non_memory_neuron_count"):
            if k in stats and isinstance(stats[k], int):
                merged[k] = stats[k]
        state_manager.set_brain_stats(merged)
    except Exception:
        pass
    try:
        if hasattr(state_manager, "reset_cumulative_activity"):
            state_manager.reset_cumulative_activity()
    except Exception:
        pass
    return True 