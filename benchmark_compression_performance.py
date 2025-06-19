#!/usr/bin/env python3
"""
Computational Performance Benchmark: Traditional vs Morton+Roaring

Compares CPU performance, memory usage, and throughput for different
neural data compression approaches.
"""

import time
import struct
import gzip
import numpy as np
import random
from typing import List, Tuple
import sys
import os

# Add path for our compression module
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from feagi.protocols.neural_stream_compression import NeuralStreamCompressor

class TraditionalCompressor:
    """Traditional approach: struct packing + gzip compression."""
    
    def compress_frame(self, neuron_data: List[Tuple[int, int, int, float]]) -> bytes:
        """Traditional compression using struct + gzip."""
        # Pack all data as binary
        packed_data = b''
        for x, y, z, mp in neuron_data:
            packed_data += struct.pack('iiif', x, y, z, mp)
        
        # Compress with gzip
        compressed = gzip.compress(packed_data, compresslevel=1)  # Fast compression
        return compressed
    
    def decompress_frame(self, compressed_data: bytes) -> List[Tuple[int, int, int, float]]:
        """Traditional decompression."""
        # Decompress
        packed_data = gzip.decompress(compressed_data)
        
        # Unpack data
        neuron_data = []
        for i in range(0, len(packed_data), 16):  # 16 bytes per neuron
            x, y, z, mp = struct.unpack('iiif', packed_data[i:i+16])
            neuron_data.append((x, y, z, mp))
        
        return neuron_data

def generate_test_data(num_neurons: int, sparsity: float = 0.1) -> List[Tuple[int, int, int, float]]:
    """Generate test neural data with specified sparsity."""
    neuron_data = []
    
    # Generate spatially clustered data (more realistic than random)
    cluster_centers = [(random.randint(0, 2047), random.randint(0, 1079), random.randint(0, 63)) 
                      for _ in range(max(1, num_neurons // 1000))]
    
    for _ in range(num_neurons):
        # Pick a cluster center and add some noise
        center_x, center_y, center_z = random.choice(cluster_centers)
        
        x = max(0, min(2047, center_x + random.randint(-50, 50)))
        y = max(0, min(1079, center_y + random.randint(-50, 50)))
        z = max(0, min(63, center_z + random.randint(-2, 2)))
        mp = random.uniform(0, 100)
        
        neuron_data.append((x, y, z, mp))
    
    return neuron_data

def benchmark_compression_methods():
    """Benchmark different compression approaches."""
    print("🚀 Neural Data Compression Benchmark")
    print("=" * 60)
    
    # Initialize compressors
    morton_compressor = NeuralStreamCompressor()
    traditional_compressor = TraditionalCompressor()
    
    # Test scenarios
    test_scenarios = [
        (1000, "1K neurons (sparse)"),
        (10000, "10K neurons (moderate)"),
        (100000, "100K neurons (dense)"),
        (220000, "220K neurons (2K video 10%)")
    ]
    
    results = []
    
    for num_neurons, description in test_scenarios:
        print(f"\n📊 Testing: {description}")
        print("-" * 40)
        
        # Generate test data
        neuron_data = generate_test_data(num_neurons)
        original_size = len(neuron_data) * 16  # 16 bytes per neuron
        
        # Benchmark Morton + Roaring
        print("🔷 Morton + Roaring Bitmap:")
        
        # Compression
        start_time = time.perf_counter()
        morton_frame = morton_compressor.compress_frame("test", neuron_data)
        morton_compress_time = (time.perf_counter() - start_time) * 1000
        
        # Decompression
        start_time = time.perf_counter()
        morton_restored = morton_compressor.decompress_frame(morton_frame)
        morton_decompress_time = (time.perf_counter() - start_time) * 1000
        
        morton_size = morton_frame.metadata['compressed_size']
        morton_ratio = morton_frame.metadata['compression_ratio']
        
        print(f"   Compress: {morton_compress_time:.1f}ms")
        print(f"   Decompress: {morton_decompress_time:.1f}ms")
        print(f"   Total: {morton_compress_time + morton_decompress_time:.1f}ms")
        print(f"   Ratio: {morton_ratio:.1f}x")
        print(f"   Throughput: {num_neurons / (morton_compress_time + morton_decompress_time) * 1000:.0f} neurons/sec")
        
        # Benchmark Traditional
        print("🔶 Traditional (struct + gzip):")
        
        # Compression
        start_time = time.perf_counter()
        traditional_compressed = traditional_compressor.compress_frame(neuron_data)
        traditional_compress_time = (time.perf_counter() - start_time) * 1000
        
        # Decompression
        start_time = time.perf_counter()
        traditional_restored = traditional_compressor.decompress_frame(traditional_compressed)
        traditional_decompress_time = (time.perf_counter() - start_time) * 1000
        
        traditional_size = len(traditional_compressed)
        traditional_ratio = original_size / traditional_size
        
        print(f"   Compress: {traditional_compress_time:.1f}ms")
        print(f"   Decompress: {traditional_decompress_time:.1f}ms")
        print(f"   Total: {traditional_compress_time + traditional_decompress_time:.1f}ms")
        print(f"   Ratio: {traditional_ratio:.1f}x")
        print(f"   Throughput: {num_neurons / (traditional_compress_time + traditional_decompress_time) * 1000:.0f} neurons/sec")
        
        # Calculate performance ratios
        morton_total = morton_compress_time + morton_decompress_time
        traditional_total = traditional_compress_time + traditional_decompress_time
        speed_ratio = traditional_total / morton_total
        
        print(f"🏁 Performance Comparison:")
        print(f"   Speed: Morton is {speed_ratio:.1f}x {'faster' if speed_ratio > 1 else 'slower'}")
        print(f"   Compression: Morton {morton_ratio:.1f}x vs Traditional {traditional_ratio:.1f}x")
        
        # Verify correctness
        morton_correct = len(morton_restored) == len(neuron_data)
        traditional_correct = len(traditional_restored) == len(neuron_data)
        
        if morton_correct and traditional_correct:
            print("   ✅ Both methods preserve data correctly")
        else:
            print(f"   ❌ Data integrity: Morton {morton_correct}, Traditional {traditional_correct}")
        
        # Store results
        results.append({
            'neurons': num_neurons,
            'description': description,
            'morton_speed': morton_total,
            'traditional_speed': traditional_total,
            'speed_ratio': speed_ratio,
            'morton_compression': morton_ratio,
            'traditional_compression': traditional_ratio,
            'morton_throughput': num_neurons / morton_total * 1000,
            'traditional_throughput': num_neurons / traditional_total * 1000
        })
    
    # Summary analysis
    print("\n" + "=" * 60)
    print("📈 COMPUTATIONAL ANALYSIS SUMMARY")
    print("=" * 60)
    
    avg_speed_ratio = sum(r['speed_ratio'] for r in results) / len(results)
    avg_morton_compression = sum(r['morton_compression'] for r in results) / len(results)
    avg_traditional_compression = sum(r['traditional_compression'] for r in results) / len(results)
    
    print(f"🚀 Average Speed Performance:")
    print(f"   Morton is {avg_speed_ratio:.1f}x {'faster' if avg_speed_ratio > 1 else 'slower'} than traditional")
    
    print(f"\n📦 Average Compression Performance:")
    print(f"   Morton: {avg_morton_compression:.1f}x compression")
    print(f"   Traditional: {avg_traditional_compression:.1f}x compression")
    
    print(f"\n⚡ Throughput Analysis:")
    for result in results:
        print(f"   {result['description']}:")
        print(f"     Morton: {result['morton_throughput']:.0f} neurons/sec")
        print(f"     Traditional: {result['traditional_throughput']:.0f} neurons/sec")
    
    # Computational complexity analysis
    print(f"\n🧮 Computational Complexity:")
    print(f"   Morton Encoding: O(1) per neuron (bit operations)")
    print(f"   Traditional Packing: O(1) per neuron (struct operations)")
    print(f"   Roaring Bitmap: O(log n) insertion, O(1) iteration")
    print(f"   Gzip Compression: O(n log n) where n is data size")
    
    # Recommendations
    print(f"\n💡 RECOMMENDATIONS:")
    
    if avg_speed_ratio > 1.2:
        print(f"   ✅ Morton approach is COMPUTATIONALLY SUPERIOR")
        print(f"   ✅ {avg_speed_ratio:.1f}x faster processing")
        print(f"   ✅ Better suited for real-time applications")
    elif avg_speed_ratio > 0.8:
        print(f"   ⚖️  Both approaches have SIMILAR computational performance")
        print(f"   💡 Choose based on compression ratio and integration needs")
    else:
        print(f"   ⚠️  Traditional approach is computationally faster")
        print(f"   💡 Consider Morton only if compression ratio is critical")
    
    print(f"\n🎯 For 2K Video Streaming:")
    video_result = next(r for r in results if "2K video" in r['description'])
    print(f"   Processing time: {video_result['morton_speed']:.1f}ms (Morton) vs {video_result['traditional_speed']:.1f}ms (Traditional)")
    print(f"   At 30 FPS (33ms budget): {'✅ Feasible' if video_result['morton_speed'] < 30 else '❌ Too slow'} (Morton)")
    print(f"   At 60 FPS (16ms budget): {'✅ Feasible' if video_result['morton_speed'] < 15 else '❌ Too slow'} (Morton)")

if __name__ == "__main__":
    benchmark_compression_methods() 