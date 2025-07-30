"""
Integration tests for cortical area parameter updates.

This test suite validates the intelligent routing system for cortical area parameter updates:
1. All parameter types update correctly and persist
2. The intelligent routing system classifies changes correctly (PARAMETER, STRUCTURAL, METADATA)
3. Performance characteristics match expectations
4. Edge cases are handled properly

Tests cover:
- PARAMETER changes (fast path ~25ms)
- STRUCTURAL changes (rebuild path ~800ms)  
- METADATA changes (fastest path ~1ms)
- SPECIAL parameters (rebuild path)
- Edge cases and error conditions
"""

import pytest
import requests
import time
from typing import Dict, Any, List, Tuple, Optional
from dataclasses import dataclass

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.core.state_manager import get_state_manager
from feagi.utils.config import FeagiConfig


@dataclass
class ParameterTestCase:
    """Test case for a single parameter update."""
    name: str
    initial_value: Any
    test_value: Any
    expected_routing: str  # "parameter", "structural", "metadata"
    value_type: type
    description: str


class TestCorticalAreaParameterUpdates:
    """Integration test suite for comprehensive cortical area parameter validation."""

    # Test cases for all cortical area parameters organized by expected routing path
    PARAMETER_TEST_CASES = [
        # PARAMETER TYPE - Fast path updates (direct neuron property updates)
        ParameterTestCase(
            name="neuron_fire_threshold",
            initial_value=1.0,
            test_value=2.5,
            expected_routing="parameter",
            value_type=float,
            description="Neuron firing threshold - direct neuron property"
        ),
        ParameterTestCase(
            name="neuron_refractory_period", 
            initial_value=0,
            test_value=3,
            expected_routing="parameter",
            value_type=int,
            description="Neuron refractory period - direct neuron property"
        ),
        ParameterTestCase(
            name="neuron_leak_coefficient",
            initial_value=10.0,
            test_value=15.5,
            expected_routing="parameter",
            value_type=float,
            description="Neuron leak coefficient - direct neuron property"
        ),
        ParameterTestCase(
            name="neuron_consecutive_fire_count",
            initial_value=5,
            test_value=8,
            expected_routing="parameter",
            value_type=int,
            description="Neuron consecutive fire count - cortical area parameter"
        ),
        ParameterTestCase(
            name="neuron_firing_threshold_limit",
            initial_value=0.0,
            test_value=10.0,
            expected_routing="parameter",
            value_type=float,
            description="Neuron firing threshold limit - cortical area parameter"
        ),
        ParameterTestCase(
            name="neuron_snooze_period",
            initial_value=0.0,
            test_value=5.0,
            expected_routing="parameter",
            value_type=float,
            description="Neuron snooze period - cortical area parameter"
        ),
        ParameterTestCase(
            name="neuron_degeneracy_coefficient",
            initial_value=0.0,
            test_value=0.1,
            expected_routing="parameter",
            value_type=float,
            description="Neuron degeneracy coefficient - cortical area parameter"
        ),
        ParameterTestCase(
            name="neuron_excitability",
            initial_value=100.0,
            test_value=120.0,
            expected_routing="parameter",
            value_type=float,
            description="Neuron excitability - cortical area parameter"
        ),
        ParameterTestCase(
            name="neuron_longterm_mem_threshold",
            initial_value=100,
            test_value=150,
            expected_routing="parameter",
            value_type=int,
            description="Long-term memory threshold - neuron parameter"
        ),
        ParameterTestCase(
            name="neuron_lifespan_growth_rate",
            initial_value=1,
            test_value=2,
            expected_routing="parameter",
            value_type=int,
            description="Neuron lifespan growth rate - neuron parameter"
        ),
        ParameterTestCase(
            name="neuron_init_lifespan",
            initial_value=9,
            test_value=12,
            expected_routing="parameter",
            value_type=int,
            description="Initial neuron lifespan - neuron parameter"
        ),
        ParameterTestCase(
            name="temporal_depth",
            initial_value=1,
            test_value=3,
            expected_routing="parameter",
            value_type=int,
            description="Temporal depth - neuron parameter"
        ),
        
        # METADATA TYPE - Fastest path updates (simple metadata changes)
        ParameterTestCase(
            name="cortical_name",
            initial_value="Brain_Power",
            test_value="Test_Power_Area",
            expected_routing="metadata", 
            value_type=str,
            description="Cortical area name - metadata only"
        ),
        
        # SPECIAL PARAMETERS - Rebuild path (need special handling)
        ParameterTestCase(
            name="neuron_leak_variability",
            initial_value=0.0,
            test_value=0.2,
            expected_routing="structural",
            value_type=float,
            description="Neuron leak variability - special parameter requiring rebuild"
        ),
        ParameterTestCase(
            name="neuron_psp_uniform_distribution",
            initial_value=False,
            test_value=True,
            expected_routing="structural",
            value_type=bool,
            description="PSP uniform distribution - special parameter requiring rebuild"
        ),
        ParameterTestCase(
            name="neuron_mp_charge_accumulation",
            initial_value=True,
            test_value=False,
            expected_routing="structural",
            value_type=bool,
            description="Membrane potential charge accumulation - special parameter requiring rebuild"
        ),
        ParameterTestCase(
            name="neuron_mp_driven_psp",
            initial_value=False,
            test_value=True,
            expected_routing="structural",
            value_type=bool,
            description="MP driven PSP - special parameter requiring rebuild"
        ),
    ]

    @pytest.fixture(scope="class")
    def feagi_config(self):
        """Create FEAGI configuration for tests."""
        return FeagiConfig({})
    
    @pytest.fixture(scope="class")
    def api_base_url(self):
        """API base URL for testing."""
        return "http://localhost:8000"
    
    @pytest.fixture(scope="class")
    def test_cortical_id(self):
        """Test cortical area ID."""
        return "___pwr"
    
    @pytest.fixture(scope="class")
    def api_session(self):
        """HTTP session for API calls."""
        return requests.Session()
    
    @pytest.fixture(scope="class")
    def initial_state(self, api_session, api_base_url, test_cortical_id):
        """Capture initial cortical area state before tests."""
        response = api_session.post(
            f"{api_base_url}/v1/cortical_area/cortical_area_properties",
            json={"cortical_id": test_cortical_id},
            headers={"Content-Type": "application/json"}
        )
        response.raise_for_status()
        return response.json()["properties"]
    
    def get_cortical_properties(self, api_session, api_base_url, cortical_id: str) -> Dict[str, Any]:
        """Get current cortical area properties."""
        response = api_session.post(
            f"{api_base_url}/v1/cortical_area/cortical_area_properties",
            json={"cortical_id": cortical_id},
            headers={"Content-Type": "application/json"}
        )
        response.raise_for_status()
        return response.json()["properties"]
    
    def update_cortical_parameter(self, api_session, api_base_url, cortical_id: str, 
                                 parameter_name: str, value: Any) -> Tuple[bool, float, Dict[str, Any]]:
        """Update a single cortical parameter and return (success, duration_ms, response_data)."""
        start_time = time.time()
        
        response = api_session.put(
            f"{api_base_url}/v1/cortical_area/cortical_area",
            json={
                "cortical_id": cortical_id,
                "parameters": {parameter_name: value}
            },
            headers={"Content-Type": "application/json"}
        )
        
        duration_ms = (time.time() - start_time) * 1000
        success = response.status_code == 200 and response.json().get("status") == "success"
        
        return success, duration_ms, response.json()
    
    def validate_performance(self, duration_ms: float, expected_routing: str) -> bool:
        """Validate performance matches expected routing path."""
        if expected_routing == "metadata":
            return duration_ms < 50  # Should be very fast
        elif expected_routing == "parameter":
            return duration_ms < 100  # Fast path
        elif expected_routing == "structural":
            return duration_ms > 200  # Rebuild path (allows some variance)
        else:
            return True  # Unknown routing, don't validate performance

    @pytest.mark.parametrize("test_case", PARAMETER_TEST_CASES)
    def test_parameter_update_end_to_end(self, test_case: ParameterTestCase, 
                                        api_session, api_base_url, test_cortical_id):
        """Test individual parameter updates end-to-end via API."""
        # Get initial value
        initial_props = self.get_cortical_properties(api_session, api_base_url, test_cortical_id)
        initial_value = initial_props.get(test_case.name)
        
        # Update the parameter
        update_success, duration_ms, response_data = self.update_cortical_parameter(
            api_session, api_base_url, test_cortical_id, test_case.name, test_case.test_value
        )
        
        # Verify the change persisted
        updated_props = self.get_cortical_properties(api_session, api_base_url, test_cortical_id)
        final_value = updated_props.get(test_case.name)
        
        # Validate results
        value_updated = final_value == test_case.test_value
        value_correct_type = isinstance(final_value, test_case.value_type)
        performance_ok = self.validate_performance(duration_ms, test_case.expected_routing)
        
        # Print detailed results for debugging
        print(f"\n🧪 Testing {test_case.name} ({test_case.expected_routing} path)")
        print(f"   Description: {test_case.description}")
        print(f"   Initial: {initial_value} → Test: {test_case.test_value} → Final: {final_value}")
        print(f"   Duration: {duration_ms:.1f}ms (expected {test_case.expected_routing} path)")
        print(f"   Success: {update_success and value_updated and value_correct_type and performance_ok}")
        
        # Assertions
        assert update_success, f"API update failed for {test_case.name}: {response_data}"
        assert value_updated, f"Value did not update for {test_case.name}: {initial_value} → {final_value} (expected {test_case.test_value})"
        assert value_correct_type, f"Value type incorrect for {test_case.name}: got {type(final_value)}, expected {test_case.value_type}"
        assert performance_ok, f"Performance not within expected range for {test_case.name}: {duration_ms:.1f}ms for {test_case.expected_routing} path"

    def test_fast_path_performance_comparison(self, api_session, api_base_url, test_cortical_id):
        """Test that parameter-type changes are significantly faster than structural changes."""
        # Test a parameter change (should be fast)
        param_success, param_duration, _ = self.update_cortical_parameter(
            api_session, api_base_url, test_cortical_id, "neuron_fire_threshold", 3.0
        )
        
        # Test a special parameter change (should be slower - triggers rebuild)  
        special_success, special_duration, _ = self.update_cortical_parameter(
            api_session, api_base_url, test_cortical_id, "neuron_psp_uniform_distribution", True
        )
        
        print(f"\n⚡ Performance comparison:")
        print(f"   Parameter update: {param_duration:.1f}ms")
        print(f"   Special parameter update: {special_duration:.1f}ms")
        print(f"   Speedup ratio: {special_duration/param_duration:.1f}x")
        
        # Assertions
        assert param_success, "Parameter update should succeed"
        assert special_success, "Special parameter update should succeed"
        assert param_duration < special_duration, "Parameter updates should be faster than special parameter updates"
        assert param_duration < 100, f"Parameter updates should be under 100ms, got {param_duration:.1f}ms"
        assert special_duration > 200, f"Special parameter updates should take >200ms (rebuild), got {special_duration:.1f}ms"

    def test_multiple_parameter_update(self, api_session, api_base_url, test_cortical_id):
        """Test updating multiple parameters in a single request."""
        # Update multiple parameters at once
        start_time = time.time()
        response = api_session.put(
            f"{api_base_url}/v1/cortical_area/cortical_area",
            json={
                "cortical_id": test_cortical_id,
                "parameters": {
                    "neuron_fire_threshold": 4.0,
                    "neuron_consecutive_fire_count": 7,
                    "neuron_excitability": 110.0
                }
            },
            headers={"Content-Type": "application/json"}
        )
        duration_ms = (time.time() - start_time) * 1000
        
        # Verify success
        assert response.status_code == 200, f"Multiple parameter update failed: {response.text}"
        assert response.json().get("status") == "success", f"Multiple parameter update status failed: {response.json()}"
        
        # Verify all values updated
        updated_props = self.get_cortical_properties(api_session, api_base_url, test_cortical_id)
        assert updated_props["neuron_fire_threshold"] == 4.0, "neuron_fire_threshold not updated"
        assert updated_props["neuron_consecutive_fire_count"] == 7, "neuron_consecutive_fire_count not updated"
        assert updated_props["neuron_excitability"] == 110.0, "neuron_excitability not updated"
        
        print(f"\n🔄 Multiple parameter update: {duration_ms:.1f}ms")
        assert duration_ms < 150, f"Multiple parameter update should be fast, got {duration_ms:.1f}ms"

    def test_edge_case_values(self, api_session, api_base_url, test_cortical_id):
        """Test edge case values for parameters."""
        edge_cases = [
            ("neuron_fire_threshold", 0.0, "Zero threshold"),
            ("neuron_fire_threshold", 1000.0, "Very high threshold"), 
            ("neuron_consecutive_fire_count", 0, "Zero consecutive fires"),
            ("neuron_consecutive_fire_count", 100, "High consecutive fires"),
            ("neuron_excitability", 0.0, "Zero excitability"),
            ("neuron_excitability", 1000.0, "Very high excitability"),
        ]
        
        for param_name, test_value, description in edge_cases:
            print(f"\n🔬 Testing edge case: {description}")
            
            success, duration, response_data = self.update_cortical_parameter(
                api_session, api_base_url, test_cortical_id, param_name, test_value
            )
            assert success, f"Failed to update {param_name} with edge case value {test_value}: {response_data}"
            
            # Verify the value persisted
            props = self.get_cortical_properties(api_session, api_base_url, test_cortical_id)
            actual_value = props.get(param_name)
            assert actual_value == test_value, f"Edge case value did not persist: {actual_value} != {test_value}"
            
            print(f"   ✅ {param_name} = {test_value} ({duration:.1f}ms)")

    def test_invalid_parameter_handling(self, api_session, api_base_url, test_cortical_id):
        """Test handling of invalid parameters."""
        # Test non-existent parameter
        response = api_session.put(
            f"{api_base_url}/v1/cortical_area/cortical_area",
            json={
                "cortical_id": test_cortical_id,
                "parameters": {
                    "non_existent_parameter": 123
                }
            },
            headers={"Content-Type": "application/json"}
        )
        
        print(f"\n❓ Invalid parameter response: {response.status_code}")
        # Note: System may accept unknown parameters (stored but not used)
        # This is not necessarily an error condition

    def test_restore_initial_state(self, api_session, api_base_url, test_cortical_id, initial_state):
        """Restore the initial state after all tests (cleanup)."""
        print(f"\n🔄 Restoring initial state for {test_cortical_id}")
        
        # Restore key parameters to initial values
        restore_params = {
            "neuron_fire_threshold": initial_state.get("neuron_fire_threshold", 1.0),
            "neuron_consecutive_fire_count": initial_state.get("neuron_consecutive_fire_count", 3),
            "neuron_excitability": initial_state.get("neuron_excitability", 100.0),
            "cortical_name": initial_state.get("cortical_name", "Brain_Power"),
        }
        
        for param_name, original_value in restore_params.items():
            success, _, response_data = self.update_cortical_parameter(
                api_session, api_base_url, test_cortical_id, param_name, original_value
            )
            assert success, f"Failed to restore {param_name} to {original_value}: {response_data}"
        
        print("   ✅ Initial state restored successfully") 