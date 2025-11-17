"""Tests for Rust connectome serialization integration.

Tests the full pipeline:
1. Python bindings (export/import via feagi_rust module)
2. Python service layer (RustConnectomeSerializer)
3. REST API endpoints (POST /v1/snapshots/connectome, etc.)

Verifies:
- Binary serialization/deserialization
- File I/O operations
- In-memory operations
- Error handling
- Data integrity after round-trip
"""

import pytest
import tempfile
from pathlib import Path
import json


@pytest.fixture
def temp_snapshot_dir():
    """Create a temporary directory for snapshot tests."""
    with tempfile.TemporaryDirectory() as tmpdir:
        yield Path(tmpdir)


@pytest.fixture
def simple_npu():
    """Create a simple Rust NPU for testing.
    
    Returns None if feagi_rust module is not available (tests will be skipped).
    """
    try:
        import feagi_rust
        
        # Create a small NPU with a few neurons and synapses
        npu = feagi_rust.RustNPU(
            neuron_capacity=100,
            synapse_capacity=500,
            fire_ledger_window=20
        )
        
        # Add some test neurons
        for i in range(10):
            npu.add_neuron(
                threshold=1.0,
                leak_coefficient=0.1,
                resting_potential=0.0,
                neuron_type=0,
                refractory_period=5,
                excitability=1.0,
                consecutive_fire_limit=0,
                snooze_period=0,
                mp_charge_accumulation=False,
                cortical_area=1,
                x=i, y=0, z=0
            )
        
        # Register cortical area
        npu.register_cortical_area(1, "test_area")
        
        # Add some synapses
        for i in range(5):
            npu.add_synapse(
                source=i,
                target=i+1,
                weight=128,
                conductance=255,
                synapse_type=0  # excitatory
            )
        
        # Rebuild indexes
        npu.rebuild_indexes()
        
        return npu
    except ImportError:
        return None


@pytest.fixture
def rust_connectome_serializer():
    """Get RustConnectomeSerializer class."""
    try:
        from feagi.core.snapshot.rust_connectome import RustConnectomeSerializer
        return RustConnectomeSerializer
    except ImportError:
        return None


class TestRustNPUBindings:
    """Test the Rust NPU Python bindings for connectome serialization."""
    
    def test_export_import_bytes(self, simple_npu):
        """Test exporting and importing connectome as bytes."""
        if simple_npu is None:
            pytest.skip("feagi_rust module not available")
        
        # Check if connectome serialization is supported
        if not hasattr(simple_npu, 'export_connectome_bytes'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        # Export to bytes
        binary_data = simple_npu.export_connectome_bytes()
        
        assert isinstance(binary_data, bytes)
        assert len(binary_data) > 0
        
        # Create a new NPU and import the data
        import feagi_rust
        new_npu = feagi_rust.RustNPU(
            neuron_capacity=100,
            synapse_capacity=500,
            fire_ledger_window=20
        )
        
        success = new_npu.import_connectome_bytes(binary_data)
        assert success is True
        
        # Verify basic counts match
        assert simple_npu.get_neuron_count() == new_npu.get_neuron_count()
        assert simple_npu.get_synapse_count() == new_npu.get_synapse_count()
        assert simple_npu.get_burst_count() == new_npu.get_burst_count()
    
    def test_save_load_file(self, simple_npu, temp_snapshot_dir):
        """Test saving and loading connectome to/from file."""
        if simple_npu is None:
            pytest.skip("feagi_rust module not available")
        
        if not hasattr(simple_npu, 'save_connectome_to_file'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        # Save to file
        connectome_file = temp_snapshot_dir / "test.connectome"
        success = simple_npu.save_connectome_to_file(str(connectome_file))
        
        assert success is True
        assert connectome_file.exists()
        assert connectome_file.stat().st_size > 0
        
        # Load from file
        import feagi_rust
        new_npu = feagi_rust.RustNPU(
            neuron_capacity=100,
            synapse_capacity=500,
            fire_ledger_window=20
        )
        
        success = new_npu.load_connectome_from_file(str(connectome_file))
        assert success is True
        
        # Verify stats
        assert simple_npu.get_neuron_count() == new_npu.get_neuron_count()
        assert simple_npu.get_synapse_count() == new_npu.get_synapse_count()
        assert simple_npu.get_burst_count() == new_npu.get_burst_count()
    
    def test_export_bytes_empty_npu(self):
        """Test exporting an empty NPU."""
        try:
            import feagi_rust
        except ImportError:
            pytest.skip("feagi_rust module not available")
        
        npu = feagi_rust.RustNPU(
            neuron_capacity=100,
            synapse_capacity=500,
            fire_ledger_window=20
        )
        
        if not hasattr(npu, 'export_connectome_bytes'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        binary_data = npu.export_connectome_bytes()
        
        assert isinstance(binary_data, bytes)
        assert len(binary_data) > 0  # Should still have header/metadata


class TestRustConnectomeSerializer:
    """Test the Python service layer for Rust connectome serialization."""
    
    def test_export_to_file(self, simple_npu, temp_snapshot_dir, rust_connectome_serializer):
        """Test export_to_file method."""
        if simple_npu is None or rust_connectome_serializer is None:
            pytest.skip("Required modules not available")
        
        if not hasattr(simple_npu, 'save_connectome_to_file'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        output_path = temp_snapshot_dir / "test_export"
        metadata = {"description": "Test export", "version": "1.0"}
        
        result_path = rust_connectome_serializer.export_to_file(
            simple_npu,
            output_path,
            metadata=metadata
        )
        
        assert result_path.exists()
        assert str(result_path).endswith('.connectome')
        assert result_path.stat().st_size > 0
        
        # Check metadata file was created
        meta_file = result_path.with_suffix('.connectome.meta.json')
        assert meta_file.exists()
        
        with open(meta_file, 'r') as f:
            saved_meta = json.load(f)
            assert saved_meta['description'] == "Test export"
            assert saved_meta['version'] == "1.0"
    
    def test_import_from_file(self, simple_npu, temp_snapshot_dir, rust_connectome_serializer):
        """Test import_from_file method."""
        if simple_npu is None or rust_connectome_serializer is None:
            pytest.skip("Required modules not available")
        
        if not hasattr(simple_npu, 'save_connectome_to_file'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        # Export first
        output_path = temp_snapshot_dir / "test_roundtrip.connectome"
        rust_connectome_serializer.export_to_file(simple_npu, output_path)
        
        # Import into new NPU
        import feagi_rust
        new_npu = feagi_rust.RustNPU(
            neuron_capacity=100,
            synapse_capacity=500,
            fire_ledger_window=20
        )
        
        success = rust_connectome_serializer.import_from_file(new_npu, output_path)
        assert success is True
        
        # Verify data integrity
        assert simple_npu.get_neuron_count() == new_npu.get_neuron_count()
        assert simple_npu.get_synapse_count() == new_npu.get_synapse_count()
        assert simple_npu.get_burst_count() == new_npu.get_burst_count()
    
    def test_export_to_bytes(self, simple_npu, rust_connectome_serializer):
        """Test export_to_bytes method."""
        if simple_npu is None or rust_connectome_serializer is None:
            pytest.skip("Required modules not available")
        
        if not hasattr(simple_npu, 'export_connectome_bytes'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        binary_data = rust_connectome_serializer.export_to_bytes(simple_npu)
        
        assert isinstance(binary_data, bytes)
        assert len(binary_data) > 0
    
    def test_import_from_bytes(self, simple_npu, rust_connectome_serializer):
        """Test import_from_bytes method."""
        if simple_npu is None or rust_connectome_serializer is None:
            pytest.skip("Required modules not available")
        
        if not hasattr(simple_npu, 'export_connectome_bytes'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        # Export to bytes
        binary_data = rust_connectome_serializer.export_to_bytes(simple_npu)
        
        # Import into new NPU
        import feagi_rust
        new_npu = feagi_rust.RustNPU(
            neuron_capacity=100,
            synapse_capacity=500,
            fire_ledger_window=20
        )
        
        success = rust_connectome_serializer.import_from_bytes(new_npu, binary_data)
        assert success is True
    
    def test_create_snapshot(self, simple_npu, temp_snapshot_dir, rust_connectome_serializer):
        """Test create_snapshot method."""
        if simple_npu is None or rust_connectome_serializer is None:
            pytest.skip("Required modules not available")
        
        if not hasattr(simple_npu, 'save_connectome_to_file'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        snapshot_id = "test_snapshot_123"
        description = "Test snapshot creation"
        
        result = rust_connectome_serializer.create_snapshot(
            npu=simple_npu,
            snapshot_dir=temp_snapshot_dir,
            snapshot_id=snapshot_id,
            description=description
        )
        
        assert result['snapshot_id'] == snapshot_id
        assert 'path' in result
        assert 'connectome_file' in result
        assert 'size_bytes' in result
        assert 'timestamp' in result
        
        # Verify files were created
        snapshot_path = Path(result['path'])
        assert snapshot_path.exists()
        assert (snapshot_path / f"{snapshot_id}.connectome").exists()
        assert (snapshot_path / "manifest.json").exists()
        
        # Check manifest content
        with open(snapshot_path / "manifest.json", 'r') as f:
            manifest = json.load(f)
            assert manifest['snapshot_id'] == snapshot_id
            assert manifest['type'] == "rust_connectome"
            assert manifest['description'] == description
    
    def test_restore_snapshot(self, simple_npu, temp_snapshot_dir, rust_connectome_serializer):
        """Test restore_snapshot method."""
        if simple_npu is None or rust_connectome_serializer is None:
            pytest.skip("Required modules not available")
        
        if not hasattr(simple_npu, 'save_connectome_to_file'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        # Create a snapshot first
        snapshot_id = "test_restore_456"
        rust_connectome_serializer.create_snapshot(
            npu=simple_npu,
            snapshot_dir=temp_snapshot_dir,
            snapshot_id=snapshot_id,
            description="Test restore"
        )
        
        # Restore into new NPU
        import feagi_rust
        new_npu = feagi_rust.RustNPU(
            neuron_capacity=100,
            synapse_capacity=500,
            fire_ledger_window=20
        )
        
        success = rust_connectome_serializer.restore_snapshot(
            npu=new_npu,
            snapshot_dir=temp_snapshot_dir,
            snapshot_id=snapshot_id
        )
        
        assert success is True
        
        # Verify data
        assert simple_npu.get_neuron_count() == new_npu.get_neuron_count()
        assert simple_npu.get_synapse_count() == new_npu.get_synapse_count()
        assert simple_npu.get_burst_count() == new_npu.get_burst_count()
    
    def test_error_handling_missing_file(self, temp_snapshot_dir, rust_connectome_serializer):
        """Test error handling when importing from non-existent file."""
        if rust_connectome_serializer is None:
            pytest.skip("Required modules not available")
        
        try:
            import feagi_rust
        except ImportError:
            pytest.skip("feagi_rust module not available")
        
        npu = feagi_rust.RustNPU(100, 500, 20)
        
        if not hasattr(npu, 'load_connectome_from_file'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        missing_file = temp_snapshot_dir / "nonexistent.connectome"
        
        with pytest.raises(FileNotFoundError):
            rust_connectome_serializer.import_from_file(npu, missing_file)
    
    def test_error_handling_empty_bytes(self, rust_connectome_serializer):
        """Test error handling when importing empty bytes."""
        if rust_connectome_serializer is None:
            pytest.skip("Required modules not available")
        
        try:
            import feagi_rust
        except ImportError:
            pytest.skip("feagi_rust module not available")
        
        npu = feagi_rust.RustNPU(100, 500, 20)
        
        if not hasattr(npu, 'import_connectome_bytes'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        with pytest.raises(ValueError):
            rust_connectome_serializer.import_from_bytes(npu, b"")


class TestDataIntegrity:
    """Test that data remains intact after serialization round-trips."""
    
    def test_neuron_data_integrity(self, simple_npu, temp_snapshot_dir, rust_connectome_serializer):
        """Verify neuron properties are preserved after save/load."""
        if simple_npu is None or rust_connectome_serializer is None:
            pytest.skip("Required modules not available")
        
        if not hasattr(simple_npu, 'save_connectome_to_file'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        # Get original neuron data
        original_neuron_count = simple_npu.get_neuron_count()
        original_synapse_count = simple_npu.get_synapse_count()
        original_burst_count = simple_npu.get_burst_count()
        
        # Save and load
        file_path = temp_snapshot_dir / "integrity_test.connectome"
        rust_connectome_serializer.export_to_file(simple_npu, file_path)
        
        import feagi_rust
        new_npu = feagi_rust.RustNPU(100, 500, 20)
        rust_connectome_serializer.import_from_file(new_npu, file_path)
        
        # Verify stats match exactly
        assert original_neuron_count == new_npu.get_neuron_count()
        assert original_synapse_count == new_npu.get_synapse_count()
        assert original_burst_count == new_npu.get_burst_count()
    
    def test_multiple_round_trips(self, simple_npu, temp_snapshot_dir, rust_connectome_serializer):
        """Test that multiple save/load cycles preserve data."""
        if simple_npu is None or rust_connectome_serializer is None:
            pytest.skip("Required modules not available")
        
        if not hasattr(simple_npu, 'export_connectome_bytes'):
            pytest.skip("NPU does not support connectome serialization feature")
        
        import feagi_rust
        
        current_npu = simple_npu
        original_neuron_count = current_npu.get_neuron_count()
        original_synapse_count = current_npu.get_synapse_count()
        original_burst_count = current_npu.get_burst_count()
        
        # Perform 3 round-trips
        for i in range(3):
            # Export
            binary_data = rust_connectome_serializer.export_to_bytes(current_npu)
            
            # Import into new NPU
            new_npu = feagi_rust.RustNPU(100, 500, 20)
            rust_connectome_serializer.import_from_bytes(new_npu, binary_data)
            
            # Verify stats still match
            assert original_neuron_count == new_npu.get_neuron_count()
            assert original_synapse_count == new_npu.get_synapse_count()
            assert original_burst_count == new_npu.get_burst_count()
            
            current_npu = new_npu


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

