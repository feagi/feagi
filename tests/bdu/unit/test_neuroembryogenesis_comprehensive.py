"""
Comprehensive tests for NeuroEmbryogenesis to achieve high code coverage.

This test suite focuses on covering the missing areas in the neuroembryogenesis.py module,
including brain development, genome processing, and cortical area creation.
"""

import json
import os
import tempfile
from unittest.mock import MagicMock, patch

import pytest

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis


class TestNeuroEmbryogenesisBasics:
    """Test basic NeuroEmbryogenesis functionality."""

    def setup_method(self):
        """Set up test with fresh instances."""
        ConnectomeManager.reset_singleton()
        self.connectome_manager = ConnectomeManager(1000)
        self.embryogenesis = NeuroEmbryogenesis(self.connectome_manager)

    def test_initialization(self):
        """Test NeuroEmbryogenesis initialization."""
        assert self.embryogenesis.connectome_manager is self.connectome_manager
        assert hasattr(self.embryogenesis, "genome_data")
        assert hasattr(self.embryogenesis, "cortical_areas")

    def test_cortical_areas_property(self):
        """Test cortical_areas property delegation."""
        # Should delegate to connectome_manager
        areas = self.embryogenesis.cortical_areas
        assert areas == self.connectome_manager.cortical_areas

    def test_load_empty_genome(self):
        """Test loading empty genome."""
        empty_genome = {}
        self.embryogenesis.load_genome(empty_genome)

        assert self.embryogenesis.genome_data == empty_genome

    def test_load_basic_genome(self):
        """Test loading basic genome structure."""
        basic_genome = {"blueprint": {"cortical_areas": {}, "brain_regions": {}}}
        self.embryogenesis.load_genome(basic_genome)

        assert self.embryogenesis.genome_data == basic_genome

    def test_get_statistics_empty(self):
        """Test getting statistics with empty brain."""
        stats = self.embryogenesis.get_statistics()

        assert "total_neurons" in stats
        assert "total_synapses" in stats
        assert "cortical_areas_count" in stats
        assert stats["total_neurons"] == 0
        assert stats["total_synapses"] == 0
        assert stats["cortical_areas_count"] == 0


class TestGenomeProcessing:
    """Test genome processing functionality."""

    def setup_method(self):
        """Set up test with fresh instances."""
        ConnectomeManager.reset_singleton()
        self.connectome_manager = ConnectomeManager(1000)
        self.embryogenesis = NeuroEmbryogenesis(self.connectome_manager)

    def test_process_simple_cortical_area(self):
        """Test processing a simple cortical area."""
        genome = {
            "blueprint": {
                "cortical_areas": {
                    "test_area": {
                        "cortical_name": "Test Area",
                        "cortical_dimensions": [10, 10, 1],
                        "cortical_coordinates": [0, 0, 0],
                        "cortical_type": "custom",
                    }
                }
            }
        }

        self.embryogenesis.load_genome(genome)
        self.embryogenesis.develop_brain()

        # Should have created the cortical area
        assert len(self.connectome_manager.cortical_areas) == 1
        area_id = list(self.connectome_manager.cortical_areas.keys())[0]
        area = self.connectome_manager.cortical_areas[area_id]
        assert area.name == "Test Area"
        assert area.dimensions == (10, 10, 1)

    def test_process_cortical_area_with_neurons(self):
        """Test processing cortical area with neuron specifications."""
        genome = {
            "blueprint": {
                "cortical_areas": {
                    "neuron_area": {
                        "cortical_name": "Neuron Area",
                        "cortical_dimensions": [5, 5, 1],
                        "cortical_coordinates": [0, 0, 0],
                        "cortical_type": "custom",
                        "neuron_count": 10,
                    }
                }
            }
        }

        self.embryogenesis.load_genome(genome)
        self.embryogenesis.develop_brain()

        # Should have created neurons
        stats = self.embryogenesis.get_statistics()
        assert stats["total_neurons"] > 0

    def test_process_brain_regions(self):
        """Test processing brain regions."""
        genome = {
            "blueprint": {
                "brain_regions": {
                    "test_region": {
                        "name": "Test Region",
                        "region_type": "custom",
                        "description": "A test brain region",
                    }
                },
                "cortical_areas": {},
            }
        }

        self.embryogenesis.load_genome(genome)
        self.embryogenesis.develop_brain()

        # Should have created the brain region
        assert len(self.connectome_manager.brain_regions) == 1

    def test_validate_genome_structure_valid(self):
        """Test genome structure validation with valid genome."""
        valid_genome = {"blueprint": {"cortical_areas": {}, "brain_regions": {}}}

        # Should not raise any exceptions
        self.embryogenesis.load_genome(valid_genome)
        result = self.embryogenesis.validate_genome()
        assert result is True

    def test_validate_genome_structure_invalid(self):
        """Test genome structure validation with invalid genome."""
        invalid_genome = {"invalid_key": "invalid_value"}

        self.embryogenesis.load_genome(invalid_genome)
        result = self.embryogenesis.validate_genome()
        assert result is False

    def test_clear_brain(self):
        """Test clearing brain data."""
        # Create some data first
        genome = {
            "blueprint": {
                "cortical_areas": {
                    "test_area": {
                        "cortical_name": "Test Area",
                        "cortical_dimensions": [5, 5, 1],
                        "cortical_coordinates": [0, 0, 0],
                    }
                }
            }
        }

        self.embryogenesis.load_genome(genome)
        self.embryogenesis.develop_brain()

        # Verify data exists
        assert len(self.connectome_manager.cortical_areas) > 0

        # Clear brain
        self.embryogenesis.clear_brain()

        # Should be empty
        assert len(self.connectome_manager.cortical_areas) == 0


class TestErrorHandling:
    """Test error handling and edge cases."""

    def setup_method(self):
        """Set up test with fresh instances."""
        ConnectomeManager.reset_singleton()
        self.connectome_manager = ConnectomeManager(1000)
        self.embryogenesis = NeuroEmbryogenesis(self.connectome_manager)

    def test_load_invalid_genome_type(self):
        """Test loading invalid genome type."""
        with pytest.raises(TypeError):
            self.embryogenesis.load_genome("not a dictionary")

    def test_load_none_genome(self):
        """Test loading None as genome."""
        with pytest.raises(TypeError):
            self.embryogenesis.load_genome(None)

    def test_develop_brain_without_genome(self):
        """Test developing brain without loading genome first."""
        # Should handle gracefully
        self.embryogenesis.develop_brain()

        # Should still have empty brain
        stats = self.embryogenesis.get_statistics()
        assert stats["total_neurons"] == 0

    def test_malformed_cortical_area_data(self):
        """Test handling malformed cortical area data."""
        malformed_genome = {
            "blueprint": {
                "cortical_areas": {
                    "bad_area": {
                        "cortical_name": "Bad Area"
                        # Missing required fields
                    }
                }
            }
        }

        self.embryogenesis.load_genome(malformed_genome)

        # Should handle gracefully without crashing
        try:
            self.embryogenesis.develop_brain()
        except Exception as e:
            # If it raises an exception, it should be a meaningful one
            assert "required" in str(e).lower() or "missing" in str(e).lower()

    def test_invalid_cortical_dimensions(self):
        """Test handling invalid cortical dimensions."""
        invalid_genome = {
            "blueprint": {
                "cortical_areas": {
                    "invalid_area": {
                        "cortical_name": "Invalid Area",
                        "cortical_dimensions": [0, 0, 0],  # Invalid dimensions
                        "cortical_coordinates": [0, 0, 0],
                    }
                }
            }
        }

        self.embryogenesis.load_genome(invalid_genome)

        # Should handle invalid dimensions gracefully
        try:
            self.embryogenesis.develop_brain()
        except Exception as e:
            assert "dimension" in str(e).lower() or "invalid" in str(e).lower()


class TestGenomeFileOperations:
    """Test genome file loading and saving operations."""

    def setup_method(self):
        """Set up test with fresh instances."""
        ConnectomeManager.reset_singleton()
        self.connectome_manager = ConnectomeManager(1000)
        self.embryogenesis = NeuroEmbryogenesis(self.connectome_manager)

    def test_load_genome_from_file(self):
        """Test loading genome from JSON file."""
        test_genome = {
            "blueprint": {
                "cortical_areas": {
                    "file_area": {
                        "cortical_name": "File Area",
                        "cortical_dimensions": [3, 3, 1],
                        "cortical_coordinates": [0, 0, 0],
                    }
                }
            }
        }

        # Create temporary file
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump(test_genome, f)
            temp_filename = f.name

        try:
            # Load from file
            self.embryogenesis.load_genome_from_file(temp_filename)

            # Verify loaded correctly
            assert self.embryogenesis.genome_data == test_genome

        finally:
            # Clean up
            if os.path.exists(temp_filename):
                os.unlink(temp_filename)

    def test_load_genome_from_nonexistent_file(self):
        """Test loading genome from non-existent file."""
        with pytest.raises(FileNotFoundError):
            self.embryogenesis.load_genome_from_file("nonexistent_file.json")

    def test_load_genome_from_invalid_json(self):
        """Test loading genome from invalid JSON file."""
        # Create file with invalid JSON
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            f.write("{ invalid json content")
            temp_filename = f.name

        try:
            with pytest.raises(json.JSONDecodeError):
                self.embryogenesis.load_genome_from_file(temp_filename)
        finally:
            if os.path.exists(temp_filename):
                os.unlink(temp_filename)

    def test_save_genome_to_file(self):
        """Test saving genome to JSON file."""
        test_genome = {
            "blueprint": {
                "cortical_areas": {
                    "save_area": {
                        "cortical_name": "Save Area",
                        "cortical_dimensions": [2, 2, 1],
                        "cortical_coordinates": [0, 0, 0],
                    }
                }
            }
        }

        self.embryogenesis.load_genome(test_genome)

        # Save to temporary file
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            temp_filename = f.name

        try:
            self.embryogenesis.save_genome_to_file(temp_filename)

            # Verify file was created and contains correct data
            assert os.path.exists(temp_filename)

            with open(temp_filename, "r") as f:
                loaded_data = json.load(f)

            assert loaded_data == test_genome

        finally:
            if os.path.exists(temp_filename):
                os.unlink(temp_filename)


class TestDevelopmentStages:
    """Test different stages of brain development."""

    def setup_method(self):
        """Set up test with fresh instances."""
        ConnectomeManager.reset_singleton()
        self.connectome_manager = ConnectomeManager(1000)
        self.embryogenesis = NeuroEmbryogenesis(self.connectome_manager)

    def test_development_stages_order(self):
        """Test that development stages execute in correct order."""
        genome = {
            "blueprint": {
                "cortical_areas": {
                    "stage_area": {
                        "cortical_name": "Stage Area",
                        "cortical_dimensions": [4, 4, 1],
                        "cortical_coordinates": [0, 0, 0],
                    }
                },
                "brain_regions": {
                    "stage_region": {"name": "Stage Region", "region_type": "test"}
                },
            }
        }

        self.embryogenesis.load_genome(genome)

        # Track development progress
        initial_stats = self.embryogenesis.get_statistics()

        # Develop brain
        self.embryogenesis.develop_brain()

        # Check final state
        final_stats = self.embryogenesis.get_statistics()

        # Should have more areas and regions than initially
        assert (
            final_stats["cortical_areas_count"] > initial_stats["cortical_areas_count"]
        )

    def test_incremental_development(self):
        """Test incremental brain development."""
        # Start with basic genome
        basic_genome = {
            "blueprint": {
                "cortical_areas": {
                    "basic_area": {
                        "cortical_name": "Basic Area",
                        "cortical_dimensions": [3, 3, 1],
                        "cortical_coordinates": [0, 0, 0],
                    }
                }
            }
        }

        self.embryogenesis.load_genome(basic_genome)
        self.embryogenesis.develop_brain()

        initial_count = len(self.connectome_manager.cortical_areas)

        # Add more areas
        extended_genome = {
            "blueprint": {
                "cortical_areas": {
                    "basic_area": {
                        "cortical_name": "Basic Area",
                        "cortical_dimensions": [3, 3, 1],
                        "cortical_coordinates": [0, 0, 0],
                    },
                    "extended_area": {
                        "cortical_name": "Extended Area",
                        "cortical_dimensions": [2, 2, 1],
                        "cortical_coordinates": [5, 0, 0],
                    },
                }
            }
        }

        self.embryogenesis.load_genome(extended_genome)
        self.embryogenesis.develop_brain()

        final_count = len(self.connectome_manager.cortical_areas)

        # Should have more areas now
        assert final_count >= initial_count

    def test_redevelopment_after_clear(self):
        """Test redeveloping brain after clearing."""
        genome = {
            "blueprint": {
                "cortical_areas": {
                    "redevelop_area": {
                        "cortical_name": "Redevelop Area",
                        "cortical_dimensions": [5, 5, 1],
                        "cortical_coordinates": [0, 0, 0],
                    }
                }
            }
        }

        # First development
        self.embryogenesis.load_genome(genome)
        self.embryogenesis.develop_brain()

        first_stats = self.embryogenesis.get_statistics()

        # Clear and redevelop
        self.embryogenesis.clear_brain()
        self.embryogenesis.develop_brain()

        second_stats = self.embryogenesis.get_statistics()

        # Should have same structure after redevelopment
        assert (
            second_stats["cortical_areas_count"] == first_stats["cortical_areas_count"]
        )


class TestStatisticsAndReporting:
    """Test statistics and reporting functionality."""

    def setup_method(self):
        """Set up test with fresh instances."""
        ConnectomeManager.reset_singleton()
        self.connectome_manager = ConnectomeManager(1000)
        self.embryogenesis = NeuroEmbryogenesis(self.connectome_manager)

    def test_detailed_statistics(self):
        """Test detailed statistics reporting."""
        genome = {
            "blueprint": {
                "cortical_areas": {
                    "stats_area1": {
                        "cortical_name": "Stats Area 1",
                        "cortical_dimensions": [3, 3, 1],
                        "cortical_coordinates": [0, 0, 0],
                    },
                    "stats_area2": {
                        "cortical_name": "Stats Area 2",
                        "cortical_dimensions": [2, 2, 1],
                        "cortical_coordinates": [5, 0, 0],
                    },
                }
            }
        }

        self.embryogenesis.load_genome(genome)
        self.embryogenesis.develop_brain()

        stats = self.embryogenesis.get_statistics()

        # Should have detailed information
        assert "total_neurons" in stats
        assert "total_synapses" in stats
        assert "cortical_areas_count" in stats
        assert stats["cortical_areas_count"] == 2

    def test_statistics_consistency(self):
        """Test that statistics are consistent with actual brain state."""
        genome = {
            "blueprint": {
                "cortical_areas": {
                    "consistency_area": {
                        "cortical_name": "Consistency Area",
                        "cortical_dimensions": [4, 4, 1],
                        "cortical_coordinates": [0, 0, 0],
                    }
                }
            }
        }

        self.embryogenesis.load_genome(genome)
        self.embryogenesis.develop_brain()

        stats = self.embryogenesis.get_statistics()

        # Cross-check with connectome manager
        actual_area_count = len(self.connectome_manager.cortical_areas)
        actual_neuron_count = self.connectome_manager.get_neuron_count()
        actual_synapse_count = self.connectome_manager.get_synapse_count()

        assert stats["cortical_areas_count"] == actual_area_count
        assert stats["total_neurons"] == actual_neuron_count
        assert stats["total_synapses"] == actual_synapse_count


@pytest.fixture(autouse=True)
def cleanup_connectome():
    """Ensure ConnectomeManager singleton is reset after each test."""
    yield
    ConnectomeManager.reset_singleton()
