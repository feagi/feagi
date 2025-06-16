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

"""Tests for the FEAGI core functionality."""

from feagi.core.feagi import FEAGI


def test_feagi_initialization():
    """Test that FEAGI can be initialized."""
    feagi = FEAGI()
    assert feagi is not None
    assert isinstance(feagi, FEAGI)
    assert isinstance(feagi.models, dict)
    assert len(feagi.models) == 0


def test_model_creation():
    """Test that models can be created."""
    feagi = FEAGI()
    model = feagi.create_model("test_model")

    assert model is not None
    assert model.name == "test_model"
    assert model.model_type == "default"

    # Check that the model was added to the models dict
    assert "test_model" in feagi.models
    assert feagi.models["test_model"] == model

    # Check that we can retrieve the model
    retrieved_model = feagi.get_model("test_model")
    assert retrieved_model == model

    # Check that listing models works
    models = feagi.list_models()
    assert isinstance(models, list)
    assert "test_model" in models


def test_model_removal():
    """Test that models can be removed."""
    feagi = FEAGI()
    model = feagi.create_model("test_model")

    # Check that the model exists
    assert "test_model" in feagi.models

    # Remove the model
    result = feagi.remove_model("test_model")

    # Check that the removal was successful
    assert result is True
    assert "test_model" not in feagi.models

    # Try to remove a non-existent model
    result = feagi.remove_model("non_existent_model")
    assert result is False
