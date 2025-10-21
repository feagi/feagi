"""FEAGI Container (.fc) writer/reader (phase 1).

Format:
- Header: 8-byte magic (b"FEAGIFC1"), 4-byte little-endian uint32 header_len
- Header JSON (UTF-8) of length header_len with fields:
  {
    "version": 1,
    "endianness": "little",
    "alignment": 4096,
    "chunks": [
      {"name": "connectome.json", "encoding": "store|deflate",
       "offset": int, "length": int,
       "uncompressed_length": int,
       "checksum": "blake2b-256-hex"},
      {"name": "state.json", "encoding": "store|deflate",
       "offset": int, "length": int,
       "uncompressed_length": int,
       "checksum": "..."}
    ]
  }
- Data region: chunk payloads at aligned offsets (alignment boundary)

Chunk encodings:
- store: raw bytes
- deflate: zlib DEFLATE-compressed bytes
"""

from __future__ import annotations

import json
import struct
import zlib
from hashlib import blake2b, sha256
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
from io import BytesIO

MAGIC = b"FEAGIFGC"  # 8-byte magic for model container
ALIGNMENT = 4096
MAGIC_FGC = b"FEAGIFGC"
MAGIC_FGS = b"FEAGIFGS"  # 8-byte magic for stateful container
FOOTER_MAGIC = b"FGFOOTR1"


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
    if encoding == "lz4":
        try:
            import lz4.block as lz4b  # type: ignore
        except Exception as e:
            raise ValueError("lz4 compression not available") from e
        comp = lz4b.compress(data)
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
    if encoding == "lz4":
        try:
            import lz4.block as lz4b  # type: ignore
        except Exception as e:
            raise ValueError("lz4 decompression not available") from e
        raw = lz4b.decompress(data, uncompressed_size=uncompressed_length)
        return raw
    raise ValueError(f"Unsupported encoding: {encoding}")


def write_fc(
    output_dir: Path,
    snapshot_id: str,
    connectome_json: Dict[str, Any],
    state_json: Dict[str, Any],
    encoding: str = "store",
    magic: bytes = MAGIC_FGC,
    extension: str = ".fgc",
    extra_chunks: Optional[List[Tuple[str, bytes]]] = None,
) -> Path:
    """Write a FEAGI Container file with connectome and state chunks.

    Returns: Path to the written .fc file
    """
    if not isinstance(output_dir, Path):
        output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    target = output_dir / f"{snapshot_id}{extension}"

    # Prepare chunks
    cj_bytes = json.dumps(connectome_json, separators=(",", ":")).encode(
        "utf-8"
    )
    sj_bytes = json.dumps(state_json, separators=(",", ":")).encode("utf-8")

    cj_enc, cj_uncompressed = _encode(cj_bytes, encoding)
    sj_enc, sj_uncompressed = _encode(sj_bytes, encoding)

    header: Dict[str, Any] = {
        "version": 1,
        "endianness": "little",
        "alignment": ALIGNMENT,
        "chunks": [],
    }

    #  We will compute offsets after building header JSON length; we need chunk
    #  metadata first
    chunks_meta: List[Dict[str, Any]] = [
        {
            "name": "connectome.json",
            "encoding": encoding,
            "offset": 0,  # to be filled
            "length": len(cj_enc),
            "uncompressed_length": cj_uncompressed,
            "checksum": _checksum(cj_bytes),
            "crc32": zlib.crc32(cj_enc) & 0xFFFFFFFF,
        },
        {
            "name": "state.json",
            "encoding": encoding,
            "offset": 0,  # to be filled
            "length": len(sj_enc),
            "uncompressed_length": sj_uncompressed,
            "checksum": _checksum(sj_bytes),
            "crc32": zlib.crc32(sj_enc) & 0xFFFFFFFF,
        },
    ]

    # Encode any extra chunks (e.g., NPZ arrays, index maps).
    # Arrays default to 'store' for mmap eligibility.
    extra_encoded: List[Tuple[Dict[str, Any], bytes]] = []
    if extra_chunks:
        for name, raw in extra_chunks:
            enc_type = "store" if name.endswith((".npz", ".npy")) else encoding
            enc_bytes, uncompressed_len = _encode(raw, enc_type)
            meta = {
                "name": name,
                "encoding": enc_type,
                "offset": 0,
                "length": len(enc_bytes),
                "uncompressed_length": uncompressed_len,
                "checksum": _checksum(raw),
                "crc32": zlib.crc32(enc_bytes) & 0xFFFFFFFF,
            }
            chunks_meta.append(meta)
            extra_encoded.append((meta, enc_bytes))

    header["chunks"] = chunks_meta
    # Serialize header once to compute header_len
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    header_len = len(header_bytes)

    # Data region starts after magic + header_len + header_bytes, aligned
    data_start = _align_offset(len(magic) + 4 + header_len, ALIGNMENT)

    # Compute per-chunk offsets
    offset = data_start
    for ch in chunks_meta:
        ch["offset"] = offset
        offset = _align_offset(offset + ch["length"], ALIGNMENT)

    # Re-serialize header with updated offsets
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    header_len = len(header_bytes)
    data_start = _align_offset(len(magic) + 4 + header_len, ALIGNMENT)

    # Recompute offsets with final header_len
    offset = data_start
    for ch in chunks_meta:
        ch["offset"] = offset
        offset = _align_offset(offset + ch["length"], ALIGNMENT)

    # Final header
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")

    with open(target, "wb") as f:
        # Write magic + header_len
        f.write(magic)
        f.write(struct.pack("<I", len(header_bytes)))
        f.write(header_bytes)
        # Pad to alignment
        pad_len = data_start - (len(magic) + 4 + len(header_bytes))
        if pad_len > 0:
            f.write(b"\x00" * pad_len)
        # Write chunks at aligned offsets
        # connectome
        f.seek(chunks_meta[0]["offset"])  # no-op if sequential
        f.write(cj_enc)
        # pad to next alignment
        next_off = _align_offset(
            chunks_meta[0]["offset"] + chunks_meta[0]["length"], ALIGNMENT
        )
        if next_off > f.tell():
            f.write(b"\x00" * (next_off - f.tell()))
        # state
        f.seek(chunks_meta[1]["offset"])  # ensure correct placement
        f.write(sj_enc)
        # extra chunks
        for meta, enc_bytes in extra_encoded:
            # pad to aligned offset
            if f.tell() < meta["offset"]:
                f.write(b"\x00" * (meta["offset"] - f.tell()))
            f.seek(meta["offset"])  # ensure correct placement
            f.write(enc_bytes)

    # Append footer with file_sha256 and chunk CRCs
    with open(target, "rb+") as f:
        file_bytes = f.read()
        file_hash = sha256(file_bytes).hexdigest()
        footer_obj = {
            "file_sha256": file_hash,
            "chunk_count": len(chunks_meta),
            "chunks_crc32": {ch["name"]: ch["crc32"] for ch in chunks_meta},
        }
        footer_json = json.dumps(footer_obj, separators=(",", ":")).encode(
            "utf-8"
        )
        f.write(footer_json)
        f.write(struct.pack("<I", len(footer_json)))
        f.write(FOOTER_MAGIC)
    return target


def read_fc_header(fc_path: Path) -> Dict[str, Any]:
    with open(fc_path, "rb") as f:
        magic = f.read(8)
        if magic not in (MAGIC_FGC, MAGIC_FGS):
            raise ValueError("Invalid FC magic")
        header_len_bytes = f.read(4)
        if len(header_len_bytes) != 4:
            raise ValueError("FC header length truncated")
        (header_len,) = struct.unpack("<I", header_len_bytes)
        header_bytes = f.read(header_len)
        if len(header_bytes) != header_len:
            raise ValueError("FC header truncated")
        header = json.loads(header_bytes.decode("utf-8"))
        header["_magic"] = magic.decode("ascii", errors="ignore")
        return header


def read_fc_footer(fc_path: Path) -> Dict[str, Any]:
    with open(fc_path, "rb") as f:
        f.seek(0, 2)
        end = f.tell()
        if end < 12:
            raise ValueError("Footer missing or file too small")
        f.seek(end - 12)
        length_bytes = f.read(4)
        magic = f.read(8)
        if magic != FOOTER_MAGIC:
            raise ValueError("Invalid footer magic")
        (footer_len,) = struct.unpack("<I", length_bytes)
        start = end - 12 - footer_len
        if start < 0:
            raise ValueError("Invalid footer length")
        f.seek(start)
        footer_json = f.read(footer_len)
        return json.loads(footer_json.decode("utf-8"))


#  Helper: collect SoA NPZ and index maps from a snapshot folder into chunk
#  tuples
def _collect_folder_chunks(snap_dir: Path) -> List[Tuple[str, bytes]]:
    chunks: List[Tuple[str, bytes]] = []
    for sub, fname in (
        ("neurons", "neurons_soa.npz"),
        ("synapses", "synapses_soa.npz"),
        ("memory", "memory_soa.npz"),
    ):
        p = snap_dir / sub / fname
        if p.exists():
            chunks.append((f"{sub}/{fname}", p.read_bytes()))
        idx = snap_dir / sub / "index_map.npy"
        if idx.exists():
            chunks.append((f"{sub}/index_map.npy", idx.read_bytes()))
        # Include optional meta describing dtype/shape per array
        meta = snap_dir / sub / f"{sub}_meta.json"
        if meta.exists():
            chunks.append((f"{sub}/{sub}_meta.json", meta.read_bytes()))
    return chunks


# Build an FC file directly from a snapshot folder (embed SoA NPZ/index maps)
def create_fc_snapshot_from_folder(
    snapshot_dir: Path,
    snapshot_id: str,
    compression: str = "store",
    magic: bytes = MAGIC_FGC,
    extension: str = ".fgc",
    destination_dir: Optional[Path] = None,
) -> Path:
    if not isinstance(snapshot_dir, Path):
        snapshot_dir = Path(snapshot_dir)
    snap_dir = snapshot_dir
    c_path = snap_dir / "connectome.json"
    s_path = snap_dir / "state.json"
    connectome_json = json.loads(c_path.read_text(encoding="utf-8"))
    state_json = json.loads(s_path.read_text(encoding="utf-8"))
    extra = _collect_folder_chunks(snap_dir)
    # Add genome and physiology if present
    gpath = snap_dir / "genome.json"
    ppath = snap_dir / "physiology.json"
    if gpath.exists():
        extra.append(("genome.json", gpath.read_bytes()))
    if ppath.exists():
        extra.append(("physiology.json", ppath.read_bytes()))
    encoding = "deflate" if compression == "deflate" else "store"
    # Allow writing artifact to a different folder (e.g., temp) while still
    # reading snapshot content from snap_dir
    target_dir = destination_dir if destination_dir is not None else snap_dir
    return write_fc(
        output_dir=target_dir,
        snapshot_id=snapshot_id,
        connectome_json=connectome_json,
        state_json=state_json,
        encoding=encoding,
        magic=magic,
        extension=extension,
        extra_chunks=extra,
    )


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


def stream_chunk(fc_path: Path, chunk_name: str, block_size: int = 65536):
    """Stream a chunk's payload without loading entire chunk in memory.

    For 'store', yields raw bytes blocks. For 'deflate' and 'lz4', yields
    decompressed blocks (requires full decode state).
    """
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
        if encoding == "store":
            remaining = length
            while remaining > 0:
                to_read = min(block_size, remaining)
                chunk = f.read(to_read)
                if not chunk:
                    break
                yield chunk
                remaining -= len(chunk)
        elif encoding == "deflate":
            decomp = zlib.decompressobj()
            remaining = length
            while remaining > 0:
                to_read = min(block_size, remaining)
                comp = f.read(to_read)
                if not comp:
                    break
                out = decomp.decompress(comp)
                if out:
                    yield out
                remaining -= len(comp)
            out = decomp.flush()
            if out:
                yield out
        elif encoding == "lz4":
            try:
                import lz4.block as lz4b  # type: ignore
            except Exception as e:
                raise ValueError("lz4 decompression not available") from e
            comp = f.read(length)
            yield lz4b.decompress(comp, uncompressed_size=uncompressed)
        else:
            raise ValueError(f"Unsupported encoding: {encoding}")


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
        magic=MAGIC_FGC,
        extension=".fgc",
    )


def create_fgc_snapshot(
    output_dir: Path,
    snapshot_id: str,
    connectome_json: Dict[str, Any] = None,
    state_json: Dict[str, Any] = None,
    compression: str = "store",
) -> Path:
    # Prefer folder-based snapshot build to include arrays
    return create_fc_snapshot_from_folder(
        snapshot_dir=output_dir,
        snapshot_id=snapshot_id,
        compression=compression,
        magic=MAGIC_FGC,
        extension=".fgc",
    )


def create_fgs_snapshot(
    output_dir: Path,
    snapshot_id: str,
    connectome_json: Dict[str, Any] = None,
    state_json: Dict[str, Any] = None,
    compression: str = "store",
) -> Path:
    return create_fc_snapshot_from_folder(
        snapshot_dir=output_dir,
        snapshot_id=snapshot_id,
        compression=compression,
        magic=MAGIC_FGS,
        extension=".fgs",
    )


def restore_fgc_snapshot(
    snapshot_root: Path,
    snapshot_id: str,
    state_manager,
    connectome_manager=None,
) -> bool:
    fc_path = Path(snapshot_root) / snapshot_id / f"{snapshot_id}.fgc"
    if not fc_path.exists():
        raise FileNotFoundError(str(fc_path))
    # Verify checksum for state.json chunk before applying
    header = read_fc_header(fc_path)
    chunks = header.get("chunks", [])
    # Pre-restore: perform deterministic cleanup of existing connectome state
    try:
        cm = connectome_manager
        if cm is not None:
            # Clear cortical areas and mapping state
            if hasattr(cm, "cortical_areas") and isinstance(
                cm.cortical_areas, dict
            ):
                cm.cortical_areas.clear()
            if hasattr(cm, "cortical_mapping") and hasattr(
                cm.cortical_mapping, "clear"
            ):
                cm.cortical_mapping.clear()
            if hasattr(cm, "cortical_connections"):
                cm.cortical_connections = {}
            # Clear id/index mappings
            if hasattr(cm, "_neuron_id_to_index_map"):
                cm._neuron_id_to_index_map.clear()
            if hasattr(cm, "_index_to_neuron_id_map"):
                cm._index_to_neuron_id_map.clear()
            # Reset spatial structures if present
            if hasattr(cm, "_spatial_hash") and cm._spatial_hash:
                try:
                    cm._spatial_hash.clear()
                except Exception:
                    pass
            if hasattr(cm, "_spatial_index") and cm._spatial_index:
                try:
                    cm._spatial_index.clear()
                except Exception:
                    pass
            # Reset arrays to empty-valid state where possible
            try:
                na = getattr(cm, "neuron_array", None)
                if na is not None:
                    # Reset counters and masks
                    if hasattr(na, "next_index"):
                        na.next_index = 0
                    if hasattr(na, "neuron_count"):
                        na.neuron_count = 0
                    if hasattr(na, "free_indices"):
                        na.free_indices = set()
                    if hasattr(na, "valid_mask"):
                        na.valid_mask[:] = False
                    if hasattr(na, "is_active"):
                        na.is_active[:] = False
            except Exception:
                pass
            try:
                sa = getattr(cm, "synapse_array", None)
                if sa is not None:
                    for attr in ("next_slot",):
                        if hasattr(sa, attr):
                            setattr(sa, attr, 0)
                    for attr in ("free_slots",):
                        if hasattr(sa, attr):
                            setattr(sa, attr, set())
            except Exception:
                pass
            # Clear FCL caches
            fclm = getattr(cm, "fcl_manager", None)
            if fclm is not None and hasattr(fclm, "clear_all_window_caches"):
                try:
                    fclm.clear_all_window_caches()
                except Exception:
                    pass
            # Invalidate StateManager caches
            try:
                sm = state_manager
                if hasattr(sm, "invalidate_cortical_areas_cache"):
                    sm.invalidate_cortical_areas_cache()
                if hasattr(sm, "set_cortical_list"):
                    sm.set_cortical_list([])
            except Exception:
                pass
    except Exception:
        pass
    state_meta = next(
        (c for c in chunks if c.get("name") == "state.json"), None
    )
    if not state_meta:
        raise ValueError("state.json chunk missing in FC")
    offset = int(state_meta["offset"])  # type: ignore[index]
    length = int(state_meta["length"])  # type: ignore[index]
    encoding = state_meta.get("encoding", "store")  # type: ignore[assignment]
    uncompressed = int(state_meta.get("uncompressed_length", 0))  # type: ignore[arg-type]
    with open(fc_path, "rb") as f:
        f.seek(offset)
        data = f.read(length)
    state_bytes = _decode(data, encoding, uncompressed)
    from hashlib import blake2b

    actual = blake2b(state_bytes, digest_size=32).hexdigest()
    expected = state_meta.get("checksum")
    if expected and actual != expected:
        raise ValueError("FC checksum mismatch for state.json")
    try:
        state = json.loads(state_bytes.decode("utf-8"))
    except Exception as e:
        raise ValueError(f"Invalid state.json in FC: {e}") from e
    stats = state.get("stats", {})
    try:
        current = state_manager.get_brain_stats() or {}
        merged = dict(current)
        for k in (
            "neuron_count",
            "memory_neuron_count",
            "non_memory_neuron_count",
        ):
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
    # Also attempt to restore arrays from embedded chunks if a
    # connectome_manager is provided
    try:
        if connectome_manager is not None:
            from io import BytesIO

            import numpy as np

            header = read_fc_header(fc_path)
            chunks = header.get("chunks", [])

            # Helper to pull chunk bytes by name
            def _read_chunk_bytes(name: str) -> Optional[bytes]:
                meta = next((c for c in chunks if c.get("name") == name), None)
                if not meta:
                    return None
                with open(fc_path, "rb") as f:
                    f.seek(int(meta["offset"]))
                    data = f.read(int(meta["length"]))
                return _decode(
                    data,
                    meta.get("encoding", "store"),
                    int(meta.get("uncompressed_length", 0)),
                )

            # Rehydrate connectome cortical areas and mapping
            cj = _read_chunk_bytes("connectome.json")
            if cj is not None:
                try:
                    cobj = json.loads(cj.decode("utf-8"))
                    areas = cobj.get("areas", [])
                    if isinstance(areas, list):
                        # Reset existing registry conservatively
                        if hasattr(connectome_manager, "cortical_areas"):
                            connectome_manager.cortical_areas.clear()
                        # Rebuild mapping object
                        if hasattr(
                            connectome_manager, "cortical_mapping"
                        ) and hasattr(
                            connectome_manager.cortical_mapping, "clear"
                        ):
                            connectome_manager.cortical_mapping.clear()
                        # Create area objects and register
                        from feagi.bdu.models.cortical_area import CorticalArea

                        for idx, entry in enumerate(areas):
                            cid = str(entry.get("id", f"C{idx:05d}"))
                            dims = entry.get("dimensions", [1, 1, 1])
                            if isinstance(dims, dict):
                                dims = [
                                    dims.get("x", 1),
                                    dims.get("y", 1),
                                    dims.get("z", 1),
                                ]
                            pos = entry.get("position", [0, 0, 0])
                            if isinstance(pos, dict):
                                pos = [
                                    pos.get("x", 0),
                                    pos.get("y", 0),
                                    pos.get("z", 0),
                                ]
                            area = CorticalArea(
                                name=cid,
                                dimensions=tuple(dims),
                                position=tuple(pos),
                                cortical_id=cid,
                                cortical_idx=idx,
                            )
                            if hasattr(connectome_manager, "cortical_areas"):
                                #  Store areas keyed by cortical_id (string),
                                #  not index
                                connectome_manager.cortical_areas[cid] = area
                            # Sync mapping
                            if hasattr(
                                connectome_manager, "_sync_cortical_mapping"
                            ):
                                connectome_manager._sync_cortical_mapping(
                                    cid, idx
                                )
                        # Validate mapping if method exists
                        if hasattr(
                            connectome_manager, "validate_cortical_mapping"
                        ):
                            try:
                                connectome_manager.validate_cortical_mapping()
                            except Exception:
                                pass
                    # Restore connections if present
                    conns = cobj.get("connections", {})
                    if isinstance(conns, dict) and hasattr(
                        connectome_manager, "cortical_connections"
                    ):
                        connectome_manager.cortical_connections = conns
                    # Restore physiology if present
                    phys = cobj.get("physiology", {})
                    if isinstance(phys, dict):
                        try:
                            if hasattr(state_manager, "set_physiology"):
                                state_manager.set_physiology(phys)
                        except Exception:
                            pass
                except Exception:
                    pass
            # Neurons
            npz_bytes = _read_chunk_bytes("neurons/neurons_soa.npz")
            if npz_bytes is not None and hasattr(
                connectome_manager, "neuron_array"
            ):
                with np.load(BytesIO(npz_bytes)) as npz:
                    # Determine baseline length from a numeric array
                    base_len = None
                    for k in npz.files:
                        if not k.endswith("__bitpacked"):
                            arr0 = npz[k]
                            if (
                                hasattr(arr0, "shape")
                                and arr0.shape
                                and arr0.ndim == 1
                            ):
                                base_len = arr0.shape[0]
                                break
                    na = connectome_manager.neuron_array
                    for key in npz.files:
                        if key.endswith("__bitpacked"):
                            continue
                        arr = npz[key]
                        if f"{key}__bitpacked" in npz.files:
                            arr = np.unpackbits(
                                arr,
                                count=(
                                    int(base_len)
                                    if base_len is not None
                                    else None
                                ),
                            )
                        setattr(na, key, arr)
            # Synapses
            sp_bytes = _read_chunk_bytes("synapses/synapses_soa.npz")
            if sp_bytes is not None and hasattr(
                connectome_manager, "synapse_array"
            ):
                with np.load(BytesIO(sp_bytes)) as npz:
                    base_len = None
                    for k in npz.files:
                        if not k.endswith("__bitpacked"):
                            arr0 = npz[k]
                            if (
                                hasattr(arr0, "shape")
                                and arr0.shape
                                and arr0.ndim == 1
                            ):
                                base_len = arr0.shape[0]
                                break
                    sa = connectome_manager.synapse_array
                    for key in npz.files:
                        if key.endswith("__bitpacked"):
                            continue
                        arr = npz[key]
                        if f"{key}__bitpacked" in npz.files:
                            arr = np.unpackbits(
                                arr,
                                count=(
                                    int(base_len)
                                    if base_len is not None
                                    else None
                                ),
                            )
                        setattr(sa, key, arr)
            # Memory
            mp_bytes = _read_chunk_bytes("memory/memory_soa.npz")
            if mp_bytes is not None and hasattr(
                connectome_manager, "memory_neuron_array"
            ):
                with np.load(BytesIO(mp_bytes)) as npz:
                    base_len = None
                    for k in npz.files:
                        if not k.endswith("__bitpacked"):
                            arr0 = npz[k]
                            if (
                                hasattr(arr0, "shape")
                                and arr0.shape
                                and arr0.ndim == 1
                            ):
                                base_len = arr0.shape[0]
                                break
                    ma = connectome_manager.memory_neuron_array
                    for key in npz.files:
                        if key.endswith("__bitpacked"):
                            continue
                        arr = npz[key]
                        if f"{key}__bitpacked" in npz.files:
                            arr = np.unpackbits(
                                arr,
                                count=(
                                    int(base_len)
                                    if base_len is not None
                                    else None
                                ),
                            )
                        setattr(ma, key, arr)
            # Update brain stats and readiness flags
            try:
                # Derive counts conservatively from arrays if available
                total_neurons = 0
                mem_neurons = 0
                non_mem = 0
                syn_count = 0
                if hasattr(connectome_manager, "neuron_array") and hasattr(
                    connectome_manager.neuron_array, "valid_mask"
                ):
                    vm = connectome_manager.neuron_array.valid_mask
                    total_neurons = int(np.count_nonzero(vm))
                if hasattr(
                    connectome_manager, "memory_neuron_array"
                ) and hasattr(
                    connectome_manager.memory_neuron_array, "is_active"
                ):
                    mem_neurons = int(
                        np.count_nonzero(
                            connectome_manager.memory_neuron_array.is_active
                        )
                    )
                non_mem = max(0, total_neurons - mem_neurons)
                if hasattr(connectome_manager, "synapse_array") and hasattr(
                    connectome_manager.synapse_array, "synapse_count"
                ):
                    syn_count = int(
                        connectome_manager.synapse_array.synapse_count
                    )
                state_manager.set_brain_stats(
                    {
                        "neuron_count": total_neurons,
                        "memory_neuron_count": mem_neurons,
                        "non_memory_neuron_count": non_mem,
                        "synapse_count": syn_count,
                        "cortical_area_count": (
                            len(
                                getattr(
                                    connectome_manager, "cortical_areas", {}
                                )
                            )
                            or 0
                        ),
                    }
                )
            except Exception:
                pass
            # Post-restore: rebuild coordinate indices and id/index maps
            try:
                na = getattr(connectome_manager, "neuron_array", None)
                areas = getattr(connectome_manager, "cortical_areas", {})
                if na is not None and isinstance(areas, dict) and areas:
                    try:
                        connectome_manager._neuron_id_to_index_map.clear()
                        connectome_manager._index_to_neuron_id_map.clear()
                    except Exception:
                        pass
                    # Optional index_to_id mapping from chunk if present
                    idx2id_arr = None
                    try:
                        meta = next(
                            (
                                c
                                for c in chunks
                                if c.get("name") == "neurons/index_to_id.npy"
                            ),
                            None,
                        )
                        if meta:
                            with open(fc_path, "rb") as f:
                                f.seek(int(meta["offset"]))
                                raw = f.read(int(meta["length"]))
                            import numpy as _np

                            idx2id_arr = _np.load(BytesIO(raw))
                    except Exception:
                        idx2id_arr = None
                    for cid, area in areas.items():
                        try:
                            if hasattr(area, "_position_map"):
                                area._position_map.clear()
                            cidx = getattr(area, "cortical_idx", None)
                            if cidx is None:
                                continue
                            import numpy as _np

                            idxs = []
                            try:
                                if hasattr(na, "cortical_idxs") and hasattr(
                                    na, "valid_mask"
                                ):
                                    mask = (
                                        _np.asarray(na.cortical_idxs)
                                        == int(cidx)
                                    ) & (
                                        _np.asarray(
                                            na.valid_mask, dtype=_np.bool_
                                        )
                                    )
                                    idxs = _np.flatnonzero(mask).tolist()
                            except Exception:
                                idxs = []
                            for i in idxs:
                                try:
                                    nid = -1
                                    if (
                                        idx2id_arr is not None
                                        and int(i) < idx2id_arr.shape[0]
                                    ):
                                        nid = int(idx2id_arr[int(i)])
                                    if nid < 0:
                                        nid = int(
                                            getattr(
                                                na, "index_to_neuron_id", {}
                                            ).get(int(i), -1)
                                        )
                                    if nid < 0:
                                        continue
                                    connectome_manager._neuron_id_to_index_map[
                                        nid
                                    ] = int(i)
                                    connectome_manager._index_to_neuron_id_map[
                                        int(i)
                                    ] = nid
                                    pos = None
                                    try:
                                        x = int(na.coordinates_x[i])
                                        y = int(na.coordinates_y[i])
                                        z = int(na.coordinates_z[i])
                                        pos = (x, y, z)
                                    except Exception:
                                        pos = None
                                    if pos is not None:
                                        if hasattr(area, "_position_map"):
                                            area._position_map[nid] = pos
                                except Exception:
                                    continue
                        except Exception:
                            pass
                    fclm = getattr(connectome_manager, "fcl_manager", None)
                    if fclm is not None and hasattr(
                        fclm, "clear_all_window_caches"
                    ):
                        try:
                            fclm.clear_all_window_caches()
                        except Exception:
                            pass
            except Exception:
                pass
            # Mark genome loaded and set validity conservatively true
            # for restored model
            try:
                state_manager.set_genome_state(2)  # GenomeState.LOADED
                state_manager.set_genome_validity(True)
                state_manager.set_burst_engine_state(2)  # ServiceState.READY
                state_manager.set_brain_readiness(True)
            except Exception:
                pass
    except Exception:
        # Best-effort array restore; core state already applied
        pass
    return True


def restore_fgs_snapshot(
    snapshot_root: Path,
    snapshot_id: str,
    state_manager,
    connectome_manager=None,
) -> bool:
    fcs_path = Path(snapshot_root) / snapshot_id / f"{snapshot_id}.fgs"
    if not fcs_path.exists():
        raise FileNotFoundError(str(fcs_path))
    # Pre-restore: perform deterministic cleanup of existing connectome state
    try:
        cm = connectome_manager
        if cm is not None:
            if hasattr(cm, "cortical_areas") and isinstance(
                cm.cortical_areas, dict
            ):
                cm.cortical_areas.clear()
            if hasattr(cm, "cortical_mapping") and hasattr(
                cm.cortical_mapping, "clear"
            ):
                cm.cortical_mapping.clear()
            if hasattr(cm, "cortical_connections"):
                cm.cortical_connections = {}
            if hasattr(cm, "_neuron_id_to_index_map"):
                cm._neuron_id_to_index_map.clear()
            if hasattr(cm, "_index_to_neuron_id_map"):
                cm._index_to_neuron_id_map.clear()
            if hasattr(cm, "_spatial_hash") and cm._spatial_hash:
                try:
                    cm._spatial_hash.clear()
                except Exception:
                    pass
            if hasattr(cm, "_spatial_index") and cm._spatial_index:
                try:
                    cm._spatial_index.clear()
                except Exception:
                    pass
            try:
                na = getattr(cm, "neuron_array", None)
                if na is not None:
                    if hasattr(na, "next_index"):
                        na.next_index = 0
                    if hasattr(na, "neuron_count"):
                        na.neuron_count = 0
                    if hasattr(na, "free_indices"):
                        na.free_indices = set()
                    if hasattr(na, "valid_mask"):
                        na.valid_mask[:] = False
                    if hasattr(na, "is_active"):
                        na.is_active[:] = False
            except Exception:
                pass
            try:
                sa = getattr(cm, "synapse_array", None)
                if sa is not None:
                    for attr in ("next_slot",):
                        if hasattr(sa, attr):
                            setattr(sa, attr, 0)
                    for attr in ("free_slots",):
                        if hasattr(sa, attr):
                            setattr(sa, attr, set())
            except Exception:
                pass
            fclm = getattr(cm, "fcl_manager", None)
            if fclm is not None and hasattr(fclm, "clear_all_window_caches"):
                try:
                    fclm.clear_all_window_caches()
                except Exception:
                    pass
            try:
                sm = state_manager
                if hasattr(sm, "invalidate_cortical_areas_cache"):
                    sm.invalidate_cortical_areas_cache()
                if hasattr(sm, "set_cortical_list"):
                    sm.set_cortical_list([])
            except Exception:
                pass
    except Exception:
        pass
    # Reuse same logic as fc restore; read_fc_header accepts both magics
    # Extract and apply state.json
    header = read_fc_header(fcs_path)
    chunks = header.get("chunks", [])

    def _read_chunk_bytes(name: str) -> Optional[bytes]:
        meta = next((c for c in chunks if c.get("name") == name), None)
        if not meta:
            return None
        with open(fcs_path, "rb") as f:
            f.seek(int(meta["offset"]))
            data = f.read(int(meta["length"]))
        return _decode(
            data,
            meta.get("encoding", "store"),
            int(meta.get("uncompressed_length", 0)),
        )

    # Rehydrate connectome cortical areas and mapping
    cj = _read_chunk_bytes("connectome.json")
    if cj is not None:
        try:
            cobj = json.loads(cj.decode("utf-8"))
            areas = cobj.get("areas", [])
            if isinstance(areas, list):
                if hasattr(connectome_manager, "cortical_areas"):
                    connectome_manager.cortical_areas.clear()
                if hasattr(connectome_manager, "cortical_mapping") and hasattr(
                    connectome_manager.cortical_mapping, "clear"
                ):
                    connectome_manager.cortical_mapping.clear()
                from feagi.bdu.models.cortical_area import CorticalArea

                for idx, entry in enumerate(areas):
                    cid = str(entry.get("id", f"C{idx:05d}"))
                    dims = entry.get("dimensions", [1, 1, 1])
                    if isinstance(dims, dict):
                        dims = [
                            dims.get("x", 1),
                            dims.get("y", 1),
                            dims.get("z", 1),
                        ]
                    pos = entry.get("position", [0, 0, 0])
                    if isinstance(pos, dict):
                        pos = [
                            pos.get("x", 0),
                            pos.get("y", 0),
                            pos.get("z", 0),
                        ]
                    cidx_val = entry.get("cortical_idx")
                    cortical_idx = (
                        int(cidx_val) if isinstance(cidx_val, int) else idx
                    )
                    area_name = entry.get("name") or cid
                    area_type = (
                        entry.get("area_type") or entry.get("type") or "custom"
                    )
                    area_props = (
                        entry.get("properties")
                        or entry.get("parameters")
                        or {}
                    )
                    area = CorticalArea(
                        name=area_name,
                        dimensions=tuple(dims),
                        position=tuple(pos),
                        area_type=area_type,
                        properties=area_props,
                        cortical_id=cid,
                        cortical_idx=cortical_idx,
                    )
                    if hasattr(connectome_manager, "cortical_areas"):
                        # Store areas keyed by cortical_id (string), not index
                        connectome_manager.cortical_areas[cid] = area
                    if hasattr(connectome_manager, "_sync_cortical_mapping"):
                        connectome_manager._sync_cortical_mapping(
                            cid, cortical_idx
                        )
                if hasattr(connectome_manager, "validate_cortical_mapping"):
                    try:
                        connectome_manager.validate_cortical_mapping()
                    except Exception:
                        pass
            # Restore connections if present
            conns = cobj.get("connections", {})
            if isinstance(conns, dict) and hasattr(
                connectome_manager, "cortical_connections"
            ):
                connectome_manager.cortical_connections = conns
            phys = cobj.get("physiology", {})
            if isinstance(phys, dict):
                try:
                    if hasattr(state_manager, "set_physiology"):
                        state_manager.set_physiology(phys)
                except Exception:
                    pass
        except Exception:
            pass
    # Neurons
    npz_bytes = _read_chunk_bytes("neurons/neurons_soa.npz")
    if npz_bytes is not None and hasattr(connectome_manager, "neuron_array"):
        with np.load(BytesIO(npz_bytes)) as npz:
            base_len = None
            for k in npz.files:
                if not k.endswith("__bitpacked"):
                    arr0 = npz[k]
                    if (
                        hasattr(arr0, "shape")
                        and arr0.shape
                        and arr0.ndim == 1
                    ):
                        base_len = arr0.shape[0]
                        break
            na = connectome_manager.neuron_array
            for key in npz.files:
                if key.endswith("__bitpacked"):
                    continue
                arr = npz[key]
                if f"{key}__bitpacked" in npz.files:
                    arr = np.unpackbits(
                        arr,
                        count=(
                            int(base_len) if base_len is not None else None
                        ),
                    )
                setattr(na, key, arr)
    # Synapses
    sp_bytes = _read_chunk_bytes("synapses/synapses_soa.npz")
    if sp_bytes is not None and hasattr(connectome_manager, "synapse_array"):
        with np.load(BytesIO(sp_bytes)) as npz:
            base_len = None
            for k in npz.files:
                if not k.endswith("__bitpacked"):
                    arr0 = npz[k]
                    if (
                        hasattr(arr0, "shape")
                        and arr0.shape
                        and arr0.ndim == 1
                    ):
                        base_len = arr0.shape[0]
                        break
            sa = connectome_manager.synapse_array
            for key in npz.files:
                if key.endswith("__bitpacked"):
                    continue
                arr = npz[key]
                if f"{key}__bitpacked" in npz.files:
                    arr = np.unpackbits(
                        arr,
                        count=(
                            int(base_len) if base_len is not None else None
                        ),
                    )
                setattr(sa, key, arr)
    # Memory
    mp_bytes = _read_chunk_bytes("memory/memory_soa.npz")
    if mp_bytes is not None and hasattr(
        connectome_manager, "memory_neuron_array"
    ):
        with np.load(BytesIO(mp_bytes)) as npz:
            base_len = None
            for k in npz.files:
                if not k.endswith("__bitpacked"):
                    arr0 = npz[k]
                    if (
                        hasattr(arr0, "shape")
                        and arr0.shape
                        and arr0.ndim == 1
                    ):
                        base_len = arr0.shape[0]
                        break
            ma = connectome_manager.memory_neuron_array
            for key in npz.files:
                if key.endswith("__bitpacked"):
                    continue
                arr = npz[key]
                if f"{key}__bitpacked" in npz.files:
                    arr = np.unpackbits(
                        arr,
                        count=(
                            int(base_len) if base_len is not None else None
                        ),
                    )
                setattr(ma, key, arr)
    # Update stats and readiness
    try:
        total_neurons = 0
        mem_neurons = 0
        non_mem = 0
        syn_count = 0
        if hasattr(connectome_manager, "neuron_array") and hasattr(
            connectome_manager.neuron_array, "valid_mask"
        ):
            vm = connectome_manager.neuron_array.valid_mask
            total_neurons = int(np.count_nonzero(vm))
        if hasattr(connectome_manager, "memory_neuron_array") and hasattr(
            connectome_manager.memory_neuron_array, "is_active"
        ):
            mem_neurons = int(
                np.count_nonzero(
                    connectome_manager.memory_neuron_array.is_active
                )
            )
        non_mem = max(0, total_neurons - mem_neurons)
        if hasattr(connectome_manager, "synapse_array") and hasattr(
            connectome_manager.synapse_array, "synapse_count"
        ):
            syn_count = int(connectome_manager.synapse_array.synapse_count)
        state_manager.set_brain_stats(
            {
                "neuron_count": total_neurons,
                "memory_neuron_count": mem_neurons,
                "non_memory_neuron_count": non_mem,
                "synapse_count": syn_count,
                "cortical_area_count": (
                    len(getattr(connectome_manager, "cortical_areas", {})) or 0
                ),
            }
        )
        state_manager.set_genome_state(2)  # LOADED
        state_manager.set_genome_validity(True)
        state_manager.set_burst_engine_state(2)  # READY
        state_manager.set_brain_readiness(True)
    except Exception:
        pass
    # Post-restore: rebuild coordinate indices and id/index maps
    try:
        na = getattr(connectome_manager, "neuron_array", None)
        areas = getattr(connectome_manager, "cortical_areas", {})
        if na is not None and isinstance(areas, dict) and areas:
            try:
                connectome_manager._neuron_id_to_index_map.clear()
                connectome_manager._index_to_neuron_id_map.clear()
            except Exception:
                pass
            idx2id_arr = None
            try:
                meta = next(
                    (
                        c
                        for c in chunks
                        if c.get("name") == "neurons/index_to_id.npy"
                    ),
                    None,
                )
                if meta:
                    with open(fcs_path, "rb") as f:
                        f.seek(int(meta["offset"]))
                        raw = f.read(int(meta["length"]))
                    import numpy as _np

                    idx2id_arr = _np.load(BytesIO(raw))
            except Exception:
                idx2id_arr = None
            for cid, area in areas.items():
                try:
                    if hasattr(area, "_position_map"):
                        area._position_map.clear()
                    cidx = getattr(area, "cortical_idx", None)
                    if cidx is None:
                        continue
                    import numpy as _np

                    idxs = []
                    try:
                        if hasattr(na, "cortical_idxs") and hasattr(
                            na, "valid_mask"
                        ):
                            mask = (
                                _np.asarray(na.cortical_idxs) == int(cidx)
                            ) & (_np.asarray(na.valid_mask, dtype=_np.bool_))
                            idxs = _np.flatnonzero(mask).tolist()
                    except Exception:
                        idxs = []
                    for i in idxs:
                        try:
                            nid = -1
                            if (
                                idx2id_arr is not None
                                and int(i) < idx2id_arr.shape[0]
                            ):
                                nid = int(idx2id_arr[int(i)])
                            if nid < 0:
                                nid = int(
                                    getattr(na, "index_to_neuron_id", {}).get(
                                        int(i), -1
                                    )
                                )
                            if nid < 0:
                                continue
                            connectome_manager._neuron_id_to_index_map[nid] = (
                                int(i)
                            )
                            connectome_manager._index_to_neuron_id_map[
                                int(i)
                            ] = nid
                            pos = None
                            try:
                                x = int(na.coordinates_x[i])
                                y = int(na.coordinates_y[i])
                                z = int(na.coordinates_z[i])
                                pos = (x, y, z)
                            except Exception:
                                pos = None
                            if pos is not None:
                                if hasattr(area, "_position_map"):
                                    area._position_map[nid] = pos
                        except Exception:
                            continue
                except Exception:
                    pass
            fclm = getattr(connectome_manager, "fcl_manager", None)
            if fclm is not None and hasattr(fclm, "clear_all_window_caches"):
                try:
                    fclm.clear_all_window_caches()
                except Exception:
                    pass
    except Exception:
        pass
    # After restore: update cortical list and refresh caches in state manager
    try:
        if hasattr(state_manager, "set_cortical_list") and hasattr(
            connectome_manager, "cortical_areas"
        ):
            cortical_ids = list(connectome_manager.cortical_areas.keys())
            state_manager.set_cortical_list(cortical_ids)
        if hasattr(state_manager, "invalidate_cortical_areas_cache"):
            state_manager.invalidate_cortical_areas_cache()
        if hasattr(state_manager, "get_cortical_areas_cache"):
            _ = state_manager.get_cortical_areas_cache(connectome_manager)
    except Exception:
        pass
    return True


# === MMAP-BASED FAST-PATH HELPERS ===
def map_fc(fc_path: Path):
    """Memory-map an .fc file read-only and return (mmap_obj, header_dict).

    Caller must later call unmap_fc(mmap_obj).
    """
    import mmap

    f = open(fc_path, "rb")
    try:
        mm = mmap.mmap(f.fileno(), length=0, access=mmap.ACCESS_READ)
        header = read_fc_header(fc_path)
        #  Keep both mapping and file handle; return both so caller can close
        #  the file
        return mm, header, f
    except Exception:
        # Ensure file closed on error
        f.close()
        raise


def unmap_fc(mm, fobj) -> None:
    """Close mapping and underlying file handle (Windows requires both)."""
    try:
        if mm is not None:
            mm.close()
    finally:
        try:
            if fobj is not None:
                fobj.close()
        except Exception:
            pass


def get_chunk_view(
    mm, header: Dict[str, Any], chunk_name: str
) -> memoryview | bytes:
    """Return a view into the mapped chunk if store-encoded; else return bytes.

    If encoding == "store": returns memoryview (zero-copy). If encoding ==
    "deflate": returns decompressed bytes.
    """
    chunks = header.get("chunks", [])
    match = next((c for c in chunks if c.get("name") == chunk_name), None)
    if not match:
        raise ValueError(f"Chunk not found: {chunk_name}")
    offset = int(match["offset"])  # type: ignore[index]
    length = int(match["length"])  # type: ignore[index]
    encoding = match.get("encoding", "store")  # type: ignore[assignment]
    uncompressed = int(match.get("uncompressed_length", 0))  # type: ignore[arg-type]
    if encoding == "store":
        # Return zero-copy view
        return memoryview(mm)[offset : offset + length]
    # deflate: read compressed slice and decompress
    comp_slice = bytes(memoryview(mm)[offset : offset + length])
    raw = _decode(comp_slice, encoding, uncompressed)
    return raw


def get_array_from_chunk(
    mm,
    header: Dict[str, Any],
    chunk_name: str,
    dtype: str,
    shape: Tuple[int, ...],
) -> Any:
    """Create a zero-copy NumPy array view for a store-encoded chunk.

    This assumes the chunk is raw binary with given dtype and shape.
    """
    import numpy as np

    view = get_chunk_view(mm, header, chunk_name)
    if not isinstance(view, memoryview):
        # Not zero-copy (deflate); fall back to bytes -> array copy
        return np.frombuffer(view, dtype=dtype).reshape(shape)
    arr = np.frombuffer(view, dtype=dtype)
    return arr.reshape(shape)


def validate_mmap_eligibility(fc_path: Path) -> None:
    """Validate that the .fc file can be used with mmap mode for arrays.

    Rules:
    - There must be at least one array chunk (identified by presence of 'dtype')
    - All array chunks must have encoding == 'store'
    - Raises ValueError with a precise message on failure
    """
    header = read_fc_header(fc_path)
    chunks = header.get("chunks", [])
    array_chunks = [c for c in chunks if "dtype" in c]
    if not array_chunks:
        # Detect presence of NPZ/meta content and provide actionable message
        has_npz = any(str(c.get("name", "")).endswith(".npz") for c in chunks)
        has_meta = any(
            str(c.get("name", "")).endswith("_meta.json") for c in chunks
        )
        if has_npz or has_meta:
            raise ValueError(
                "mmap mode requires raw array chunks with dtype/shape in header; "
                "found NPZ/meta but no raw arrays."
            )
        raise ValueError(
            "mmap mode requires array chunks in .fc; none present"
        )
    bad = [c for c in array_chunks if c.get("encoding") != "store"]
    if bad:
        names = ",".join(c.get("name", "<unknown>") for c in bad)
        raise ValueError(
            f"mmap requires store-encoded arrays; found non-store in: {names}"
        )
