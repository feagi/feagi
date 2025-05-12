import pytest
from unittest.mock import Mock, patch
from feagi.core.genome_transaction import GenomeTransaction
from feagi.core.state_manager import ServiceState, GenomeState

def test_transaction_creation():
    """Test that transaction can be created with proper description"""
    transaction = GenomeTransaction("Test transaction")
    assert transaction.description == "Test transaction"
    assert not transaction._has_committed

def test_record_change():
    """Test that changes can be recorded"""
    transaction = GenomeTransaction("Test transaction")
    transaction.record_change("add", "cortical_areas.test_area", None, {"name": "Test"})
    assert len(transaction._changes) == 1
    assert transaction._changes[0]["operation"] == "add"
    assert transaction._changes[0]["path"] == "cortical_areas.test_area"

def test_transaction_commit():
    """Test that committing a transaction applies changes"""
    mock_core_api = Mock()
    transaction = GenomeTransaction("Test transaction", core_api_service=mock_core_api)
    transaction.record_change("update_cortical_area", "test_id", {"old": "value"}, {"new": "value"})
    
    # Mock the _apply_to_genome method to avoid actual genome updates
    with patch.object(transaction, '_apply_to_genome'):
        result = transaction.commit()
        
    assert result is True
    assert transaction._has_committed

def test_transaction_rollback():
    """Test transaction rollback functionality"""
    mock_core_api = Mock()
    mock_genome = {"cortical_areas": {"test_area": {"prop": "old_value"}}}
    mock_core_api.get_genome.return_value = mock_genome
    
    transaction = GenomeTransaction("Test transaction", core_api_service=mock_core_api)
    transaction._has_committed = True  # Simulate a committed transaction
    
    # Mock needed methods
    with patch.object(transaction, '_set_at_path') as mock_set, \
         patch.object(transaction, '_synchronize_connectome'):
        
        transaction.record_change("update_cortical_area", "test_id", {"prop": "old_value"}, {"prop": "new_value"})
        transaction.rollback()
        
        # Should try to restore old values
        mock_set.assert_called() 