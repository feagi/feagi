"""
Performance profiling test for visualization pipeline with high neuron counts.

This test simulates video streaming with 36k+ neurons to identify bottlenecks.
"""
import pytest
import time
import asyncio
from unittest.mock import Mock, MagicMock, patch
from typing import Dict, Any, List


@pytest.fixture
def mock_cortical_data_high_load():
    """Generate mock cortical data with 36,000 neurons (typical video frame)."""
    data = {}
    
    # Simulate 9 cortical areas (3x3 segmented vision) with ~4000 neurons each
    for area_idx in range(9):
        cortical_idx = 100 + area_idx
        num_neurons = 4000
        
        data[cortical_idx] = {
            "neuron_ids": list(range(num_neurons)),
            "coordinates_x": [i % 64 for i in range(num_neurons)],
            "coordinates_y": [(i // 64) % 64 for i in range(num_neurons)],
            "coordinates_z": [0 for i in range(num_neurons)],
            "membrane_potentials": [1.5 for i in range(num_neurons)]
        }
    
    return data


@pytest.fixture
def mock_rust_encoder():
    """Mock the Rust visualization encoder."""
    mock_encoder = Mock()
    # Simulate encoding time proportional to neuron count
    def mock_encode():
        time.sleep(0.001)  # 1ms simulated encoding
        return b'\x0b\x01' + b'mock_encoded_data' * 100  # Type 11 header + data
    
    mock_encoder.encode.side_effect = mock_encode
    mock_encoder.add_neurons = Mock()
    return mock_encoder


@pytest.fixture
def mock_fq_sampler():
    """Mock FQ sampler that returns high neuron count data."""
    sampler = Mock()
    sampler.sample_frequency = 30.0
    return sampler


class TestVisualizationPipelineProfile:
    """Test suite for profiling the visualization pipeline."""
    
    def test_profile_fq_sampling(self, mock_cortical_data_high_load, mock_fq_sampler):
        """Profile FQ sampler data retrieval time."""
        # Simulate FQ sampler returning data
        times = []
        
        for i in range(100):
            start = time.perf_counter()
            mock_fq_sampler.sample.return_value = mock_cortical_data_high_load if i % 2 == 0 else None
            data = mock_fq_sampler.sample()
            elapsed = (time.perf_counter() - start) * 1000
            if data:
                times.append(elapsed)
        
        avg_time = sum(times) / len(times) if times else 0
        max_time = max(times) if times else 0
        
        print(f"\n📊 FQ Sampling Profile:")
        print(f"  Average: {avg_time:.3f}ms")
        print(f"  Maximum: {max_time:.3f}ms")
        print(f"  Samples: {len(times)}")
        
        # Should be very fast (just mock overhead)
        assert avg_time < 1.0, f"FQ sampling too slow: {avg_time:.3f}ms"
    
    def test_profile_rust_encoding(self, mock_cortical_data_high_load, mock_rust_encoder):
        """Profile Rust binary encoding with 36k neurons."""
        with patch('feagi_rust.VisualizationEncoder', return_value=mock_rust_encoder):
            import feagi_rust
            
            times = []
            for i in range(50):
                encoder = feagi_rust.VisualizationEncoder()
                
                start = time.perf_counter()
                
                # Simulate adding neurons for all areas
                for cortical_idx, area_data in mock_cortical_data_high_load.items():
                    encoder.add_neurons(
                        cortical_id=f"iic{cortical_idx}",
                        x_coords=area_data["coordinates_x"],
                        y_coords=area_data["coordinates_y"],
                        z_coords=area_data["coordinates_z"],
                        potentials=area_data["membrane_potentials"]
                    )
                
                binary_data = encoder.encode()
                elapsed = (time.perf_counter() - start) * 1000
                times.append(elapsed)
            
            avg_time = sum(times) / len(times)
            max_time = max(times)
            
            print(f"\n📊 Rust Encoding Profile (36k neurons):")
            print(f"  Average: {avg_time:.3f}ms")
            print(f"  Maximum: {max_time:.3f}ms")
            print(f"  Samples: {len(times)}")
            
            # With mock, should be ~1ms (our simulated time)
            # Real Rust encoding should be 1-3ms
            assert avg_time < 10.0, f"Encoding too slow: {avg_time:.3f}ms (target: <5ms)"
    
    def test_profile_python_json_encoding(self, mock_cortical_data_high_load):
        """Profile Python JSON encoding (old slow path) for comparison."""
        import json
        
        times = []
        for i in range(50):
            start = time.perf_counter()
            
            # Simulate old JSON encoding path
            output = {"type": 11, "areas": {}}
            for cortical_idx, area_data in mock_cortical_data_high_load.items():
                output["areas"][f"iic{cortical_idx}"] = {
                    "neuron_ids": area_data["neuron_ids"],
                    "x": [int(x) for x in area_data["coordinates_x"]],  # List comprehension
                    "y": [int(y) for y in area_data["coordinates_y"]],
                    "z": [int(z) for z in area_data["coordinates_z"]],
                    "p": [float(p) for p in area_data["membrane_potentials"]]
                }
            
            json_bytes = json.dumps(output, separators=(',', ':')).encode('utf-8')
            payload = bytes([1, 1]) + json_bytes
            
            elapsed = (time.perf_counter() - start) * 1000
            times.append(elapsed)
        
        avg_time = sum(times) / len(times)
        max_time = max(times)
        
        print(f"\n📊 Python JSON Encoding Profile (36k neurons) - OLD SLOW PATH:")
        print(f"  Average: {avg_time:.3f}ms")
        print(f"  Maximum: {max_time:.3f}ms")
        print(f"  Samples: {len(times)}")
        print(f"  ⚠️  This is why we switched to Rust binary encoding!")
        
        # This should be slow (10-20ms) to demonstrate the problem we fixed
        assert avg_time > 5.0, f"JSON encoding faster than expected: {avg_time:.3f}ms"
    
    def test_profile_shm_write(self, mock_cortical_data_high_load, tmp_path):
        """Profile SHM ring buffer write performance."""
        import mmap
        import struct
        import os
        
        # Create temp SHM file
        shm_path = tmp_path / "viz_test.shm"
        num_slots = 64
        slot_size = 1024 * 1024  # 1MB
        header_size = 256
        
        # Initialize SHM
        total_size = header_size + num_slots * slot_size
        fd = os.open(str(shm_path), os.O_CREAT | os.O_RDWR)
        os.ftruncate(fd, total_size)
        mm = mmap.mmap(fd, total_size, access=mmap.ACCESS_WRITE)
        
        # Write header
        header = struct.pack("<8sIIIQI", b"FEAGIVIS", 1, num_slots, slot_size, 0, 0)
        mm.seek(0)
        mm.write(header)
        
        # Profile writes
        times = []
        test_payload = b'\x0b\x01' + b'x' * 50000  # ~50KB payload
        
        for i in range(100):
            start = time.perf_counter()
            
            # Write to slot
            write_idx = i % num_slots
            slot_off = header_size + write_idx * slot_size
            mm.seek(slot_off)
            mm.write(struct.pack("<I", len(test_payload)))
            mm.write(test_payload)
            
            # Update header
            mm.seek(0)
            header = struct.pack("<8sIIIQI", b"FEAGIVIS", 1, num_slots, slot_size, i+1, (write_idx+1) % num_slots)
            mm.write(header)
            
            elapsed = (time.perf_counter() - start) * 1000
            times.append(elapsed)
        
        mm.close()
        os.close(fd)
        
        avg_time = sum(times) / len(times)
        max_time = max(times)
        
        print(f"\n📊 SHM Write Profile (50KB payload):")
        print(f"  Average: {avg_time:.3f}ms")
        print(f"  Maximum: {max_time:.3f}ms")
        print(f"  Samples: {len(times)}")
        
        # SHM writes should be very fast (<1ms)
        assert avg_time < 2.0, f"SHM writes too slow: {avg_time:.3f}ms"
        assert max_time < 5.0, f"SHM max write too slow: {max_time:.3f}ms"
    
    def test_profile_end_to_end_pipeline(self, mock_cortical_data_high_load, mock_rust_encoder):
        """Profile the complete visualization pipeline end-to-end."""
        with patch('feagi_rust.VisualizationEncoder', return_value=mock_rust_encoder):
            import feagi_rust
            
            pipeline_times = {
                "fq_sample": [],
                "encoding": [],
                "total": []
            }
            
            # Simulate 100 frames
            for frame in range(100):
                frame_start = time.perf_counter()
                
                # Step 1: FQ Sample
                sample_start = time.perf_counter()
                cortical_data = mock_cortical_data_high_load if frame % 3 != 0 else None  # Simulate rate limiting
                sample_time = (time.perf_counter() - sample_start) * 1000
                
                if cortical_data is None:
                    time.sleep(0.001)  # Simulate sleep on rate limit
                    continue
                
                pipeline_times["fq_sample"].append(sample_time)
                
                # Step 2: Encoding
                encode_start = time.perf_counter()
                encoder = feagi_rust.VisualizationEncoder()
                for cortical_idx, area_data in cortical_data.items():
                    encoder.add_neurons(
                        cortical_id=f"iic{cortical_idx}",
                        x_coords=area_data["coordinates_x"],
                        y_coords=area_data["coordinates_y"],
                        z_coords=area_data["coordinates_z"],
                        potentials=area_data["membrane_potentials"]
                    )
                binary_data = encoder.encode()
                encode_time = (time.perf_counter() - encode_start) * 1000
                pipeline_times["encoding"].append(encode_time)
                
                # Total frame time
                total_time = (time.perf_counter() - frame_start) * 1000
                pipeline_times["total"].append(total_time)
            
            # Calculate statistics
            print(f"\n📊 END-TO-END PIPELINE PROFILE (36k neurons, 100 frames):")
            print(f"\n  FQ Sampling:")
            print(f"    Average: {sum(pipeline_times['fq_sample'])/len(pipeline_times['fq_sample']):.3f}ms")
            print(f"    Maximum: {max(pipeline_times['fq_sample']):.3f}ms")
            
            print(f"\n  Encoding:")
            print(f"    Average: {sum(pipeline_times['encoding'])/len(pipeline_times['encoding']):.3f}ms")
            print(f"    Maximum: {max(pipeline_times['encoding']):.3f}ms")
            
            print(f"\n  Total Frame Processing:")
            print(f"    Average: {sum(pipeline_times['total'])/len(pipeline_times['total']):.3f}ms")
            print(f"    Maximum: {max(pipeline_times['total']):.3f}ms")
            
            avg_total = sum(pipeline_times['total']) / len(pipeline_times['total'])
            print(f"\n  Theoretical Max FPS: {1000.0/avg_total:.1f} Hz")
            print(f"  Target: 30 Hz (33.3ms per frame)")
            
            # At 30 Hz, we have 33.3ms per frame budget
            assert avg_total < 33.0, f"Pipeline too slow for 30Hz: {avg_total:.3f}ms (need <33ms)"
            
            # Report what percentage of frame budget is used
            budget_used = (avg_total / 33.3) * 100
            print(f"\n  Frame Budget Used: {budget_used:.1f}%")
            
            if budget_used > 80:
                print(f"  ⚠️  WARNING: Using >80% of frame budget - potential for frame drops")
            else:
                print(f"  ✅ Good headroom for 30 Hz operation")


if __name__ == "__main__":
    # Allow running directly for quick profiling
    pytest.main([__file__, "-v", "-s"])

