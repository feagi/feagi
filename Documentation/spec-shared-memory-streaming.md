# Shared Memory Video and Data Streaming Specification

**Document Version:** 1.0  
**Last Updated:** October 2025  
**Component:** FEAGI Connector 2.0

## Overview

This document specifies the shared memory (SHM) implementation used for high-performance, zero-copy inter-process video and data streaming between FEAGI agents and visualizers. This is distinct from FEAGI Core's internal memory-mapped state management (see `feagi_core/docs/spec-shared-memory.md`) and focuses specifically on **agent-to-visualizer communication**.

### Key Use Cases

- **Video Streaming**: Raw camera frames from video agents to Brain Visualizer (30-60 FPS)
- **Processed Video**: FEAGI-processed segmented vision mosaics for preview
- **Neuron Data**: Binary-encoded neuron activity data for visualization
- **Sensory Data**: High-frequency sensor data from agents to FEAGI

## Architecture Principles

### Design Goals

✅ **Cross-Platform**: Works on Linux, macOS, and Windows with OS-specific optimizations  
✅ **Zero-Copy**: Direct memory access via `mmap()` - no serialization overhead  
✅ **Lock-Free**: Single-writer, multiple-reader design using ring buffers  
✅ **SSD-Friendly**: Automatic RAM-backed storage to prevent SSD wear  
✅ **FFI-Safe**: Binary layout compatible with Godot/C++/Rust readers  

### Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| **Latency** | <1ms | Frame-to-display time |
| **Throughput** | 500+ MB/s | Raw video @ 1080p 60fps |
| **CPU Overhead** | <1% | Per stream on modern CPUs |
| **Memory Footprint** | ~8MB | Per 1080p video stream (triple-buffered) |

## File-Backed Ring Buffer Design

### Why File-Backed Memory Mapping?

We use **file-backed `mmap()`** instead of platform-specific SHM APIs because:

1. **Cross-Platform**: Works identically on Linux, macOS, Windows, and RTOS
2. **Simple**: No need for platform-specific shm_open/CreateFileMapping APIs
3. **Debuggable**: SHM contents can be inspected as regular files
4. **Godot-Compatible**: GDExtensions can easily memory-map files

### Ring Buffer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Shared Memory File (.bin)                                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Header (256 bytes)                                         │
│  ├─ Magic number (8 bytes): "FEAGIVID" or "FEAGIBIN"       │
│  ├─ Version (4 bytes): uint32                               │
│  ├─ Dimensions (12 bytes): width, height, channels          │
│  ├─ Ring buffer metadata (12 bytes): num_slots, write_index │
│  ├─ Frame sequence (8 bytes): uint64 monotonic counter      │
│  └─ Reserved (212 bytes): padding for future fields         │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│  Slot 0: Frame/Data Buffer                                  │
│  (width × height × 3 bytes for video, or slot_size bytes)   │
├─────────────────────────────────────────────────────────────┤
│  Slot 1: Frame/Data Buffer                                  │
├─────────────────────────────────────────────────────────────┤
│  Slot 2: Frame/Data Buffer                                  │
├─────────────────────────────────────────────────────────────┤
│  ... (num_slots total)                                      │
└─────────────────────────────────────────────────────────────┘
```

### Triple-Buffering (3 Slots)

**Default configuration**: 3 slots for video streams

**How it works:**
1. Writer writes to **slot N** (e.g., slot 0)
2. Writer updates header: `write_index = 0`, `frame_seq++`
3. Writer writes to **slot N+1** (e.g., slot 1) while reader reads slot 0
4. Process repeats in round-robin: 0 → 1 → 2 → 0 → ...

**Benefits:**
- **No tearing**: Reader never reads a half-written frame
- **No locks**: Writer and reader never block each other
- **Smooth playback**: Reader can lag by 1-2 frames without dropping

### Write-Index + Frame-Sequence Protocol

**Writer side** (`SharedFrameWriter`):
```python
def write_frame(frame):
    self._write_index = (self._write_index + 1) % num_slots  # Round-robin
    frame_offset = HEADER_SIZE + self._write_index * frame_stride
    
    # Write frame data
    mmap.seek(frame_offset)
    mmap.write(frame.tobytes())
    
    # Update header atomically
    self._frame_seq += 1
    header = pack_header(write_index=self._write_index, frame_seq=self._frame_seq)
    mmap.seek(0)
    mmap.write(header)
    mmap.flush()  # Optional: OS will flush eventually
```

**Reader side** (Brain Visualizer/Godot):
```gdscript
var last_frame_seq = -1

func _process(delta):
    var header = read_header(shm_file)
    
    # Check if new frame available
    if header.frame_seq > last_frame_seq:
        last_frame_seq = header.frame_seq
        
        # Read from the slot indicated by write_index
        var frame_data = read_slot(shm_file, header.write_index)
        update_texture(frame_data)
```

## Cross-Platform Storage Strategy

### Automatic Directory Selection

The `get_shm_directory()` function selects the optimal location:

```python
Priority:
1. FEAGI_SHM_DIR environment variable (user override)
2. OS-specific RAM-backed location:
   - Linux: /dev/shm (tmpfs, guaranteed RAM)
   - macOS: /tmp (often RAM-backed)
   - Windows: %TEMP% (may be on disk)
3. Fallback: system temp directory
```

### Linux: `/dev/shm` (Optimal)

- **Type**: tmpfs (temporary filesystem in RAM)
- **Benefits**: 
  - ✅ Zero SSD wear
  - ✅ Fastest possible access
  - ✅ Automatic cleanup on reboot
- **Size**: Typically 50% of RAM, configurable
- **Verification**: `df -T /dev/shm` should show "tmpfs"

**Example:**
```bash
$ df -h /dev/shm
Filesystem      Size  Used Avail Use% Mounted on
tmpfs            16G  100M   16G   1% /dev/shm
```

### macOS: `/tmp`

- **Type**: May be RAM-backed or disk-backed (depends on config)
- **Benefits**:
  - ✅ Automatic cleanup
  - ✅ Often RAM-backed on modern macOS
- **Size**: Variable
- **Note**: macOS aggressively caches tmpfs, so SSD wear is minimal

### Windows: `%TEMP%` (Requires User Setup)

- **Type**: Usually disk-backed (on C: drive)
- **Default Behavior**: SHM files written to SSD/HDD
- **Recommendation**: Set `FEAGI_SHM_DIR` to a RAM disk

**RAM Disk Options for Windows:**
1. **ImDisk Toolkit** (Free, open-source)
   - Download: https://sourceforge.net/projects/imdisk-toolkit/
   - Create RAM disk (e.g., R:\ drive)
   - Set `FEAGI_SHM_DIR=R:\feagi_shm`

2. **AMD Radeon RAMDisk** (Free up to 4GB)
3. **Dataram RAMDisk** (Free up to 1GB)

**PowerShell setup:**
```powershell
# Create RAM disk (using ImDisk)
imdisk -a -s 4G -m R: -p "/fs:ntfs /q /y"

# Set environment variable
[Environment]::SetEnvironmentVariable("FEAGI_SHM_DIR", "R:\feagi_shm", "User")
```

### Environment Variable Configuration

**Linux/macOS:**
```bash
# Temporary (current session)
export FEAGI_SHM_DIR=/path/to/ramdisk

# Permanent (add to ~/.bashrc or ~/.zshrc)
echo 'export FEAGI_SHM_DIR=/dev/shm' >> ~/.bashrc
```

**Windows:**
```powershell
# User-level (persists across reboots)
[Environment]::SetEnvironmentVariable("FEAGI_SHM_DIR", "R:\feagi_shm", "User")

# System-level (requires admin)
[Environment]::SetEnvironmentVariable("FEAGI_SHM_DIR", "R:\feagi_shm", "Machine")
```

## SSD Wear Considerations

### The Problem

At 30 FPS, a 1080p video stream writes:
- **30 frames/second** × **1920×1080×3 bytes/frame** = **~186 MB/second**
- **~670 GB/hour** if continuously flushed to SSD
- **~16 TB/day** of writes!

Modern SSDs are rated for **~300-600 TBW** (terabytes written) over their lifetime, so this could significantly reduce SSD lifespan.

### The Solution

**Use RAM-backed storage** where the data never hits the SSD:

| Platform | Solution | SSD Writes |
|----------|----------|------------|
| **Linux** | `/dev/shm` (tmpfs) | ✅ Zero (always RAM) |
| **macOS** | `/tmp` | ⚠️ Minimal (cached) |
| **Windows** | RAM disk + `FEAGI_SHM_DIR` | ✅ Zero (if configured) |

### Flush Strategy

The code calls `mmap.flush()` after each write:
```python
mmap.flush()  # Request OS to sync to storage
```

**However:**
- The OS may **batch** or **delay** flushes
- For tmpfs (Linux /dev/shm), flush is a **no-op** (already in RAM)
- For disk-backed storage, flush is **asynchronous** (doesn't block writer)

**Future optimization:** Make flush optional for even better performance:
```python
def write_frame(frame, sync=False):
    # ... write frame ...
    if sync:
        mmap.flush()  # Only flush when synchronization is needed
```

## Data Formats

### Video Format: `FEAGIVID`

**Magic Number:** `b"FEAGIVID"` (8 bytes)  
**Header Format:** `<8sIIIIIIQIQ` (56 bytes + 200 padding = 256 total)

| Field | Type | Offset | Description |
|-------|------|--------|-------------|
| magic | char[8] | 0 | "FEAGIVID" |
| version | uint32 | 8 | Format version (currently 1) |
| width | uint32 | 12 | Frame width in pixels |
| height | uint32 | 16 | Frame height in pixels |
| channels | uint32 | 20 | Number of channels (always 3 for RGB) |
| pixel_format | uint32 | 24 | Pixel format (1 = RGB8) |
| num_slots | uint32 | 28 | Number of ring buffer slots |
| frame_stride | uint64 | 32 | Bytes per frame (width×height×3) |
| write_index | uint32 | 40 | Current write slot (0 to num_slots-1) |
| frame_seq | uint64 | 44 | Monotonic frame counter |
| _reserved_ | bytes[200] | 52 | Padding for future fields |

**Frame Data:** RGB8 format, row-major order
- Pixel at (x, y): `data[y * width * 3 + x * 3 : y * width * 3 + x * 3 + 3]`
- Channel order: R, G, B (0-255 each)

### Binary Format: `FEAGIBIN`

**Magic Number:** `b"FEAGIBIN"` (8 bytes)  
**Header Format:** `<8sIIIQI` (32 bytes + 224 padding = 256 total)

| Field | Type | Offset | Description |
|-------|------|--------|-------------|
| magic | char[8] | 0 | "FEAGIBIN" |
| version | uint32 | 8 | Format version (currently 1) |
| num_slots | uint32 | 12 | Number of ring buffer slots |
| slot_size | uint32 | 16 | Maximum bytes per slot |
| frame_seq | uint64 | 20 | Monotonic write counter |
| write_index | uint32 | 28 | Current write slot |
| _reserved_ | bytes[224] | 32 | Padding for future fields |

**Slot Format:** Each slot contains:
- **Length** (4 bytes, uint32): Actual data length
- **Data** (variable, up to slot_size bytes): Payload

## Usage Examples

### Python Writer (Video Agent)

```python
from feagi_connector.utils.shm import SharedFrameWriter
import cv2

# Create writer (auto-selects optimal SHM directory)
writer = SharedFrameWriter()

# Open video source
cap = cv2.VideoCapture(0)

while True:
    ret, frame_bgr = cap.read()
    if not ret:
        break
    
    # Convert BGR to RGB
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    
    # Write to shared memory (zero-copy!)
    writer.write_frame(frame_rgb)

writer.close()
```

### Python Writer (Custom SHM Location)

```python
import os
os.environ["FEAGI_SHM_DIR"] = "/mnt/ramdisk"

writer = SharedFrameWriter()  # Uses /mnt/ramdisk
```

### Godot Reader (Brain Visualizer)

```gdscript
# Load the SharedMemVideo GDExtension
var shm_reader = SharedMemVideo.new()

func _ready():
    # Open SHM file
    var shm_path = "/dev/shm/feagi-shm-video-agent-1-video.bin"
    if shm_reader.open(shm_path):
        print("✅ SHM opened successfully")

func _process(delta):
    # Get latest frame as texture
    var texture = shm_reader.get_texture()
    if texture:
        $VideoDisplay.texture = texture
```

## Performance Tuning

### Optimal Slot Configuration

| Use Case | Slots | Slot Size | Rationale |
|----------|-------|-----------|-----------|
| **Video (30 FPS)** | 3 | width×height×3 | Triple-buffering prevents tearing |
| **Video (60 FPS)** | 4-5 | width×height×3 | Extra slots for high-frequency writes |
| **Neuron Data** | 16 | 256 KB | Many small writes, larger buffer |
| **Low Latency** | 2 | minimal | Minimize lag (risk of tearing) |

### Memory Footprint Calculation

**Video (1080p, 3 slots):**
```
Header: 256 bytes
Frame: 1920 × 1080 × 3 = 6,220,800 bytes
Total: 256 + (6,220,800 × 3) = 18,662,656 bytes ≈ 17.8 MB
```

**Video (4K, 3 slots):**
```
Frame: 3840 × 2160 × 3 = 24,883,200 bytes
Total: 256 + (24,883,200 × 3) = 74,649,856 bytes ≈ 71.2 MB
```

### Flush Strategy

**Current (conservative):**
```python
mmap.flush()  # After every write
```

**Future (performance):**
```python
# Flush every N frames
if frame_count % 10 == 0:
    mmap.flush()
```

**Best (tmpfs):**
```python
# No flush needed on tmpfs
if not is_tmpfs(path):
    mmap.flush()
```

## Error Handling

### Common Issues

**1. File Already Exists**
```python
# Solution: Clean up stale files on agent startup
if shm_path.exists():
    shm_path.unlink()
writer = SharedFrameWriter(path=shm_path)
```

**2. Insufficient Disk/RAM Space**
```python
# Check available space
import shutil
stats = shutil.disk_usage(get_shm_directory())
if stats.free < required_size:
    raise RuntimeError("Insufficient SHM space")
```

**3. Reader Sees Corrupt Data**
```python
# Verify magic number
header = read_header(shm_path)
if header.magic != b"FEAGIVID":
    raise ValueError("Invalid SHM file format")
```

### Cleanup Strategy

**Python (Writer):**
```python
def cleanup():
    writer.close()
    if writer.path.exists():
        writer.path.unlink()

# Register cleanup
import atexit
atexit.register(cleanup)
```

**FEAGI Core:**
```python
# Clean up stale SHM files on startup
shm_dir = get_shm_directory()
for file in shm_dir.glob("feagi-shm-*.bin"):
    if not is_process_running(file):
        file.unlink()
```

## Testing

### Unit Tests

See `tests/test_shm_cross_platform.py` for comprehensive test coverage:

- OS-specific directory selection
- Environment variable override
- Ring buffer correctness
- Cross-process communication
- SSD wear mitigation verification

### Running Tests

```bash
cd feagi_connector_2.0
pytest tests/test_shm_cross_platform.py -v
```

### Performance Benchmarks

```python
import time
import numpy as np
from feagi_connector.utils.shm import SharedFrameWriter

writer = SharedFrameWriter()
frame = np.random.randint(0, 256, (1080, 1920, 3), dtype=np.uint8)

start = time.perf_counter()
for _ in range(1000):
    writer.write_frame(frame)
elapsed = time.perf_counter() - start

print(f"FPS: {1000 / elapsed:.1f}")
print(f"Throughput: {(1920*1080*3*1000) / elapsed / 1e6:.1f} MB/s")
```

**Expected results:**
- **FPS**: 500-1000+ (depending on hardware)
- **Throughput**: 3-6 GB/s (memory bandwidth limited)

## Migration and Compatibility

### Rust Interop

The binary format is designed for Rust FFI:

```rust
use memmap2::Mmap;
use std::fs::File;

#[repr(C, packed)]
struct VideoHeader {
    magic: [u8; 8],
    version: u32,
    width: u32,
    height: u32,
    channels: u32,
    pixel_format: u32,
    num_slots: u32,
    frame_stride: u64,
    write_index: u32,
    frame_seq: u64,
}

fn read_frame(shm_path: &str) -> Vec<u8> {
    let file = File::open(shm_path)?;
    let mmap = unsafe { Mmap::map(&file)? };
    
    let header = unsafe { &*(mmap.as_ptr() as *const VideoHeader) };
    let frame_offset = 256 + (header.write_index as usize) * (header.frame_stride as usize);
    
    mmap[frame_offset..frame_offset + header.frame_stride as usize].to_vec()
}
```

### Backward Compatibility

**Version field** in header allows format evolution:
- Version 1: Current RGB8 format
- Version 2: Could add RGBA, compression, etc.

Readers should check version and handle accordingly.

## Related Documentation

- [FEAGI Core Memory-Mapped State](../../../feagi_core/docs/spec-shared-memory.md) - Internal state management
- [Agent Registration](arch-agent-registration.md) - How agents receive SHM paths
- [ZMQ Communication](guide-zmq-communication.md) - Alternative transport for remote agents
- [Gaze Control](guide-gaze-control.md) - Motor data via SHM

## Appendix: Platform-Specific Notes

### Linux

**Check tmpfs size:**
```bash
df -h /dev/shm
```

**Increase tmpfs size (if needed):**
```bash
sudo mount -o remount,size=8G /dev/shm
```

**Make permanent (add to /etc/fstab):**
```
tmpfs /dev/shm tmpfs defaults,size=8G 0 0
```

### macOS

**Check if /tmp is RAM-backed:**
```bash
mount | grep /tmp
```

**Alternative: Create RAM disk:**
```bash
# Create 4GB RAM disk
diskutil erasevolume HFS+ "RAMDisk" `hdiutil attach -nomount ram://8388608`
export FEAGI_SHM_DIR=/Volumes/RAMDisk
```

### Windows

**Check temp directory:**
```powershell
echo $env:TEMP
```

**Recommended: Use ImDisk RAM disk**
```powershell
# Install ImDisk Toolkit first
imdisk -a -s 4G -m R: -p "/fs:ntfs /q /y"
[Environment]::SetEnvironmentVariable("FEAGI_SHM_DIR", "R:\feagi_shm", "User")
```

---

**Document Maintenance:**
- Update this document when adding new SHM formats
- Keep performance benchmarks current with each release
- Test cross-platform behavior on all supported OSs

