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

"""
Test module for ConnectivityRule class.
"""

import pytest
from feagi.bdu.connectivity.connectivity_rules import ConnectivityRule

@pytest.fixture
def connectivity_rule():
    """Create a basic connectivity rule for testing."""
    return ConnectivityRule(
        rule_id="test_rule",
        name="Test Rule",
        source_area_id="source_area",
        target_area_id="target_area",
        rule_type="one_to_one",
        parameters={"weight": 0.5},
        description="A test connectivity rule"
    )

def test_connectivity_rule_init():
    """Test connectivity rule initialization."""
    rule = ConnectivityRule(
        rule_id="test_rule",
        name="Test Rule",
        source_area_id="source_area",
        target_area_id="target_area", 
        rule_type="one_to_one",
        parameters={"weight": 0.5, "probability": 0.8},
        description="A test connectivity rule"
    )
    
    # Check properties
    assert rule.id == "test_rule"
    assert rule.name == "Test Rule"
    assert rule.source_area_id == "source_area"
    assert rule.target_area_id == "target_area"
    assert rule.rule_type == "one_to_one"
    assert rule.parameters["weight"] == 0.5
    assert rule.parameters["probability"] == 0.8
    assert rule.description == "A test connectivity rule"

def test_connectivity_rule_to_dict(connectivity_rule):
    """Test conversion to dictionary."""
    rule_dict = connectivity_rule.to_dict()
    
    # Check dictionary contents
    assert rule_dict["id"] == "test_rule"
    assert rule_dict["name"] == "Test Rule"
    assert rule_dict["source_area_id"] == "source_area"
    assert rule_dict["target_area_id"] == "target_area"
    assert rule_dict["rule_type"] == "one_to_one"
    assert rule_dict["parameters"]["weight"] == 0.5
    assert rule_dict["description"] == "A test connectivity rule"

def test_connectivity_rule_from_dict():
    """Test creating rule from dictionary."""
    rule_dict = {
        "id": "test_rule_2",
        "name": "Test Rule 2",
        "source_area_id": "area1",
        "target_area_id": "area2",
        "rule_type": "probabilistic",
        "parameters": {"probability": 0.3, "weight": 1.0},
        "description": "Another test rule"
    }
    
    rule = ConnectivityRule.from_dict(rule_dict)
    
    # Check properties
    assert rule.id == "test_rule_2"
    assert rule.name == "Test Rule 2"
    assert rule.source_area_id == "area1"
    assert rule.target_area_id == "area2"
    assert rule.rule_type == "probabilistic"
    assert rule.parameters["probability"] == 0.3
    assert rule.parameters["weight"] == 1.0
    assert rule.description == "Another test rule"

def test_connectivity_rule_update(connectivity_rule):
    """Test updating rule properties."""
    updates = {
        "name": "Updated Rule",
        "rule_type": "probabilistic",
        "parameters": {"probability": 0.3, "weight": 1.0},
        "description": "Updated description"
    }
    
    connectivity_rule.update(updates)
    
    # Check updated properties
    assert connectivity_rule.name == "Updated Rule"
    assert connectivity_rule.rule_type == "probabilistic"
    assert connectivity_rule.parameters["probability"] == 0.3
    assert connectivity_rule.parameters["weight"] == 1.0
    assert connectivity_rule.description == "Updated description"
    
    # Check that non-updated properties remain the same
    assert connectivity_rule.id == "test_rule"
    assert connectivity_rule.source_area_id == "source_area"
    assert connectivity_rule.target_area_id == "target_area"

def test_connectivity_rule_update_invalid_property(connectivity_rule):
    """Test updating with an invalid property."""
    with pytest.raises(KeyError):
        connectivity_rule.update({"invalid_property": "value"})

def test_connectivity_rule_validate():
    """Test rule validation."""
    # Valid rule
    valid_rule = ConnectivityRule(
        rule_id="test_rule",
        name="Test Rule",
        source_area_id="source_area",
        target_area_id="target_area",
        rule_type="one_to_one",
        parameters={"weight": 0.5},
        description="A test connectivity rule"
    )
    assert valid_rule.validate() == True
    
    # Invalid rule: missing name
    invalid_rule = ConnectivityRule(
        rule_id="test_rule",
        name="",  # Empty name
        source_area_id="source_area",
        target_area_id="target_area",
        rule_type="one_to_one",
        parameters={"weight": 0.5},
        description="A test connectivity rule"
    )
    assert invalid_rule.validate() == False
    
    # Invalid rule: missing source area
    invalid_rule = ConnectivityRule(
        rule_id="test_rule",
        name="Test Rule",
        source_area_id="",  # Empty source area
        target_area_id="target_area",
        rule_type="one_to_one",
        parameters={"weight": 0.5},
        description="A test connectivity rule"
    )
    assert invalid_rule.validate() == False
    
    # Invalid rule: missing target area
    invalid_rule = ConnectivityRule(
        rule_id="test_rule",
        name="Test Rule",
        source_area_id="source_area",
        target_area_id="",  # Empty target area
        rule_type="one_to_one",
        parameters={"weight": 0.5},
        description="A test connectivity rule"
    )
    assert invalid_rule.validate() == False
    
    # Invalid rule: missing rule type
    invalid_rule = ConnectivityRule(
        rule_id="test_rule",
        name="Test Rule",
        source_area_id="source_area",
        target_area_id="target_area",
        rule_type="",  # Empty rule type
        parameters={"weight": 0.5},
        description="A test connectivity rule"
    )
    assert invalid_rule.validate() == False 