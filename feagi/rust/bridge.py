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

"""Bridge module for Python to Rust functions."""
from typing import List, Union, Optional
import numpy as np

try:
    from feagi.rust.feagi_rust import (
        add_to_array,
        fast_matrix_vector_mul,
        relu,
        sigmoid,
    )
    
    HAS_RUST_EXTENSION = True
except ImportError:
    HAS_RUST_EXTENSION = False
    
    def add_to_array(array: List[float], value: float) -> List[float]:
        """
        Python fallback implementation for add_to_array.
        
        Args:
            array: Input array.
            value: Value to add to each element.
            
        Returns:
            Array with value added to each element.
        """
        return [x + value for x in array]
    
    def fast_matrix_vector_mul(matrix: List[List[float]], vector: List[float]) -> List[float]:
        """
        Python fallback implementation for fast_matrix_vector_mul.
        
        Args:
            matrix: Input matrix.
            vector: Input vector.
            
        Returns:
            Result of matrix-vector multiplication.
        """
        if not matrix or not vector:
            raise ValueError("Empty matrix or vector provided")
        
        n_cols = len(matrix[0])
        if len(vector) != n_cols:
            raise ValueError(f"Matrix columns ({n_cols}) and vector length ({len(vector)}) must match")
        
        result = []
        for row in matrix:
            if len(row) != n_cols:
                raise ValueError("All rows in matrix must have the same length")
            
            result.append(sum(a * b for a, b in zip(row, vector)))
        
        return result
    
    def relu(array: List[float]) -> List[float]:
        """
        Python fallback implementation for relu.
        
        Args:
            array: Input array.
            
        Returns:
            Array with ReLU applied to each element.
        """
        return [max(0.0, x) for x in array]
    
    def sigmoid(array: List[float]) -> List[float]:
        """
        Python fallback implementation for sigmoid.
        
        Args:
            array: Input array.
            
        Returns:
            Array with sigmoid applied to each element.
        """
        import math
        return [1.0 / (1.0 + math.exp(-x)) for x in array]

def matrix_multiply(
    matrix: Union[List[List[float]], np.ndarray], 
    vector: Union[List[float], np.ndarray]
) -> np.ndarray:
    """
    Perform matrix-vector multiplication.
    
    This function will use the Rust implementation if available,
    otherwise it will fall back to the Python implementation.
    
    Args:
        matrix: Input matrix.
        vector: Input vector.
        
    Returns:
        Result of matrix-vector multiplication.
    """
    # Convert inputs to lists if they are numpy arrays
    if isinstance(matrix, np.ndarray):
        matrix_list = matrix.tolist()
    else:
        matrix_list = matrix
        
    if isinstance(vector, np.ndarray):
        vector_list = vector.tolist()
    else:
        vector_list = vector
    
    # Call the appropriate implementation
    result = fast_matrix_vector_mul(matrix_list, vector_list)
    
    # Return the result as a numpy array
    return np.array(result)

def apply_activation(
    array: Union[List[float], np.ndarray], 
    activation: str = "relu"
) -> np.ndarray:
    """
    Apply an activation function to an array.
    
    Args:
        array: Input array.
        activation: Name of the activation function (relu or sigmoid).
        
    Returns:
        Array with activation function applied.
    """
    # Convert input to list if it is a numpy array
    if isinstance(array, np.ndarray):
        array_list = array.tolist()
    else:
        array_list = array
    
    # Apply the appropriate activation function
    if activation.lower() == "relu":
        result = relu(array_list)
    elif activation.lower() == "sigmoid":
        result = sigmoid(array_list)
    else:
        raise ValueError(f"Unknown activation function: {activation}")
    
    # Return the result as a numpy array
    return np.array(result) 