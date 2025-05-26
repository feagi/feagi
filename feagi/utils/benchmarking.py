"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""Benchmarking utilities for FEAGI.

This module provides tools for benchmarking performance-critical sections of code,
which helps identify areas that would benefit from Rust implementation.
"""
import time
import functools
import statistics
from feagi.utils.logger import setup_logger
logger = setup_logger()
from typing import Dict, List, Optional, Any, Callable, Union, Tuple
import numpy as np

logger = logging.getLogger("feagi.benchmarking")

class BenchmarkResult:
    """Results of a benchmark run."""
    
    def __init__(self, name: str, iterations: int, times: List[float]):
        """
        Initialize benchmark results.
        
        Args:
            name: Name of the benchmark
            iterations: Number of iterations
            times: List of execution times
        """
        self.name = name
        self.iterations = iterations
        self.times = times
        
        # Calculate statistics
        self.mean = statistics.mean(times)
        self.median = statistics.median(times)
        self.min = min(times)
        self.max = max(times)
        
        if len(times) > 1:
            self.stdev = statistics.stdev(times)
        else:
            self.stdev = 0.0
    
    def __str__(self) -> str:
        """Get string representation of the benchmark result."""
        return (
            f"Benchmark: {self.name}\n"
            f"Iterations: {self.iterations}\n"
            f"Mean: {self.mean * 1000:.2f} ms\n"
            f"Median: {self.median * 1000:.2f} ms\n"
            f"Min: {self.min * 1000:.2f} ms\n"
            f"Max: {self.max * 1000:.2f} ms\n"
            f"StdDev: {self.stdev * 1000:.2f} ms"
        )


class Benchmark:
    """A benchmark runner for measuring performance."""
    
    def __init__(self):
        """Initialize the benchmark runner."""
        self.results: Dict[str, BenchmarkResult] = {}
    
    def run(
        self, 
        func: Callable, 
        args: tuple = (), 
        kwargs: Dict = None, 
        iterations: int = 100, 
        warmup: int = 10,
        name: Optional[str] = None
    ) -> BenchmarkResult:
        """
        Run a benchmark on a function.
        
        Args:
            func: Function to benchmark
            args: Arguments to pass to the function
            kwargs: Keyword arguments to pass to the function
            iterations: Number of iterations to run
            warmup: Number of warmup iterations
            name: Name of the benchmark (defaults to function name)
            
        Returns:
            BenchmarkResult
        """
        func_name = name or func.__name__
        kwargs = kwargs or {}
        
        logger.info(f"Running benchmark: {func_name}")
        logger.info(f"Warmup iterations: {warmup}")
        logger.info(f"Benchmark iterations: {iterations}")
        
        # Warmup
        for _ in range(warmup):
            func(*args, **kwargs)
        
        # Benchmark
        times = []
        for _ in range(iterations):
            start_time = time.perf_counter()
            func(*args, **kwargs)
            end_time = time.perf_counter()
            times.append(end_time - start_time)
        
        # Create result
        result = BenchmarkResult(func_name, iterations, times)
        self.results[func_name] = result
        
        logger.info(f"Benchmark complete: {func_name}")
        logger.info(f"Mean time: {result.mean * 1000:.2f} ms")
        
        return result
    
    def compare(self, name1: str, name2: str) -> Dict[str, Any]:
        """
        Compare two benchmark results.
        
        Args:
            name1: Name of the first benchmark
            name2: Name of the second benchmark
            
        Returns:
            Dictionary with comparison statistics
        """
        if name1 not in self.results:
            raise ValueError(f"Benchmark '{name1}' not found")
        if name2 not in self.results:
            raise ValueError(f"Benchmark '{name2}' not found")
        
        result1 = self.results[name1]
        result2 = self.results[name2]
        
        ratio = result1.mean / result2.mean
        diff = result1.mean - result2.mean
        percent = (diff / result1.mean) * 100
        
        return {
            "name1": name1,
            "name2": name2,
            "mean1": result1.mean,
            "mean2": result2.mean,
            "ratio": ratio,
            "diff": diff,
            "percent": percent,
        }
    
    def print_comparison(self, name1: str, name2: str) -> None:
        """
        Print a comparison between two benchmark results.
        
        Args:
            name1: Name of the first benchmark
            name2: Name of the second benchmark
        """
        comparison = self.compare(name1, name2)
        logger.info(f"Benchmark Comparison: {name1} vs {name2}")
        logger.info(f"{name1} mean: {comparison['mean1'] * 1000:.2f} ms")
        logger.info(f"{name2} mean: {comparison['mean2'] * 1000:.2f} ms")
        logger.info(f"Ratio: {comparison['ratio']:.2f}x")
        logger.info(f"Difference: {comparison['diff'] * 1000:.2f} ms")
        logger.info(f"Percent change: {comparison['percent']:.2f}%")


def benchmark(
    iterations: int = 100, 
    warmup: int = 10, 
    name: Optional[str] = None
) -> Callable:
    """
    Decorator to benchmark a function.
    
    Args:
        iterations: Number of iterations to run
        warmup: Number of warmup iterations
        name: Name of the benchmark (defaults to function name)
        
    Returns:
        Decorated function
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            # Create benchmark runner
            runner = Benchmark()
            
            # Generate benchmark name
            benchmark_name = name or func.__name__
            
            # Run the benchmark
            result = runner.run(
                func=func, 
                args=args, 
                kwargs=kwargs, 
                iterations=iterations, 
                warmup=warmup,
                name=benchmark_name
            )
            
            # Print the result
            logger.info(result)
            
            # Return the original function result
            return func(*args, **kwargs)
        return wrapper
    return decorator


# Create a global benchmark runner
benchmark_runner = Benchmark()


# Examples of critical sections to benchmark

def python_matrix_multiply(matrix_a: np.ndarray, matrix_b: np.ndarray) -> np.ndarray:
    """Pure Python implementation of matrix multiplication."""
    rows_a, cols_a = matrix_a.shape
    rows_b, cols_b = matrix_b.shape
    
    if cols_a != rows_b:
        raise ValueError(f"Matrix shapes incompatible: {matrix_a.shape} and {matrix_b.shape}")
    
    result = np.zeros((rows_a, cols_b))
    
    for i in range(rows_a):
        for j in range(cols_b):
            for k in range(cols_a):
                result[i, j] += matrix_a[i, k] * matrix_b[k, j]
    
    return result

def numpy_matrix_multiply(matrix_a: np.ndarray, matrix_b: np.ndarray) -> np.ndarray:
    """NumPy implementation of matrix multiplication."""
    return np.matmul(matrix_a, matrix_b)

# Example benchmark comparison between pure Python and NumPy implementation
def benchmark_comparison_example():
    """Example of benchmarking and comparing two implementations."""
    # Create test matrices
    a = np.random.random((100, 100))
    b = np.random.random((100, 100))
    
    # Run benchmarks
    result1 = benchmark_runner.run(
        python_matrix_multiply, 
        args=(a, b), 
        iterations=10,
        warmup=2,
        name="python_matmul"
    )
    
    result2 = benchmark_runner.run(
        numpy_matrix_multiply, 
        args=(a, b), 
        iterations=10,
        warmup=2,
        name="numpy_matmul"
    )
    
    # Print results
    logger.info(result1)
    logger.info(result2)
    
    # Compare results
    benchmark_runner.print_comparison("python_matmul", "numpy_matmul")


# Example of using the benchmark decorator
@benchmark(iterations=10, warmup=2, name="decorated_matmul")
def decorated_matrix_multiply(matrix_a: np.ndarray, matrix_b: np.ndarray) -> np.ndarray:
    """Matrix multiplication with benchmark decorator."""
    return np.matmul(matrix_a, matrix_b) 