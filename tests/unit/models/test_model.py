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

"""Tests for the FEAGI Model class."""
import pytest
import numpy as np
from feagi.models.model import Model

def test_model_initialization():
    """Test that a Model can be initialized."""
    model = Model("test_model")
    assert model is not None
    assert model.name == "test_model"
    assert model.model_type == "default"

def test_model_training():
    """Test that a Model can be trained."""
    model = Model("test_model")
    
    # Create some dummy training data
    data = [{"x": 1, "y": 2}, {"x": 3, "y": 4}]
    
    # Train the model
    metrics = model.train(data, epochs=5)
    
    # Check that training returned metrics
    assert isinstance(metrics, dict)
    assert "loss" in metrics
    assert "accuracy" in metrics
    
    # Check that model metadata was updated
    assert "updated_at" in model.metadata
    assert "last_trained" in model.metadata

def test_model_prediction():
    """Test that a Model can make predictions."""
    model = Model("test_model")
    
    # Create some dummy prediction data
    data = [{"x": 1, "y": 2}, {"x": 3, "y": 4}]
    
    # Make predictions
    predictions = model.predict(data)
    
    # Check that predictions were returned
    assert predictions is not None
    assert isinstance(predictions, np.ndarray)
    assert len(predictions) == len(data)

def test_model_evaluation():
    """Test that a Model can be evaluated."""
    model = Model("test_model")
    
    # Create some dummy evaluation data
    data = [{"x": 1, "y": 2}, {"x": 3, "y": 4}]
    
    # Evaluate the model
    metrics = model.evaluate(data)
    
    # Check that evaluation returned metrics
    assert isinstance(metrics, dict)
    assert "loss" in metrics
    assert "accuracy" in metrics
    assert "f1" in metrics 