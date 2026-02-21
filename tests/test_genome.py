"""
Tests for feagi.genome module: GenomeLoader, validate_genome, auto_fix_genome, GenomeAPI.
"""

from unittest.mock import patch

import pytest

from feagi.genome import (
    GenomeAPI,
    GenomeAPIError,
    GenomeLoader,
    auto_fix_genome,
    validate_genome,
)


# -------------------------------------------------------------------------
# GenomeLoader tests
# -------------------------------------------------------------------------


def test_genome_loader_load_from_absolute_path(tmp_path):
    """GenomeLoader loads genome from absolute path."""
    genome_file = tmp_path / "test_genome.json"
    genome_data = {"version": "2.0", "blueprint": {}, "neuron_morphologies": {}, "physiology": {}}
    genome_file.write_text('{"version": "2.0", "blueprint": {}, "neuron_morphologies": {}, "physiology": {}}')

    loader = GenomeLoader()
    result = loader.load(str(genome_file))
    assert result == genome_data


def test_genome_loader_load_file_not_found():
    """GenomeLoader raises FileNotFoundError when file does not exist."""
    loader = GenomeLoader()
    with pytest.raises(FileNotFoundError, match="Genome file not found"):
        loader.load("/nonexistent/path/genome.json")


# -------------------------------------------------------------------------
# validate_genome and auto_fix_genome tests (require feagi_rust_py_libs)
# -------------------------------------------------------------------------


@pytest.mark.skipif(
    __import__("importlib.util").find_spec("feagi_rust_py_libs") is None
    or not hasattr(__import__("feagi_rust_py_libs", fromlist=["genome"]), "genome"),
    reason="feagi_rust_py_libs.genome not available",
)
def test_validate_genome_valid():
    """validate_genome returns (True, []) for valid minimal genome."""
    genome = {
        "version": "2.0",
        "genome_id": "g-test",
        "blueprint": {},
        "neuron_morphologies": {},
        "physiology": {"simulation_timestep": 0.01, "max_age": 100},
        "brain_regions": {},
        "signatures": {},
        "stats": {},
    }
    valid, errors = validate_genome(genome)
    assert valid is True
    assert isinstance(errors, list)


@pytest.mark.skipif(
    __import__("importlib.util").find_spec("feagi_rust_py_libs") is None
    or not hasattr(__import__("feagi_rust_py_libs", fromlist=["genome"]), "genome"),
    reason="feagi_rust_py_libs.genome not available",
)
def test_auto_fix_genome_returns_tuple():
    """auto_fix_genome returns (dict, int)."""
    genome = {"version": "2.0", "blueprint": {}, "neuron_morphologies": {}, "physiology": {}}
    fixed, num_fixes = auto_fix_genome(genome)
    assert isinstance(fixed, dict)
    assert isinstance(num_fixes, int)


# -------------------------------------------------------------------------
# GenomeAPI tests (mock requests - I/O layer outside domain under test)
# -------------------------------------------------------------------------


@pytest.fixture
def genome_api():
    """GenomeAPI instance for tests."""
    return GenomeAPI("http://test:8000", timeout=5.0)


def test_genome_api_upload(genome_api):
    """GenomeAPI.upload sends POST to /v1/genome/upload."""
    genome = {"version": "2.0", "blueprint": {}, "neuron_morphologies": {}, "physiology": {}}
    mock_response = {
        "success": True,
        "message": "Genome uploaded successfully",
        "cortical_area_count": 10,
        "brain_region_count": 2,
    }

    with patch("feagi.genome.api.requests.request") as mock_req:
        mock_req.return_value.status_code = 200
        mock_req.return_value.content = b'{"success": true, "message": "Genome uploaded successfully", "cortical_area_count": 10, "brain_region_count": 2}'
        mock_req.return_value.json.return_value = mock_response
        mock_req.return_value.text = ""

        result = genome_api.upload(genome)

    assert result["success"] is True
    assert result["cortical_area_count"] == 10
    mock_req.assert_called_once()
    assert mock_req.call_args[0][0] == "POST"
    assert "/genome/upload" in mock_req.call_args[0][1]


def test_genome_api_download(genome_api):
    """GenomeAPI.download sends GET to /v1/genome/download."""
    mock_genome = {"version": "2.0", "blueprint": {}, "neuron_morphologies": {}, "physiology": {}}

    with patch("feagi.genome.api.requests.request") as mock_req:
        mock_req.return_value.status_code = 200
        mock_req.return_value.json.return_value = mock_genome
        mock_req.return_value.content = b"{}"

        result = genome_api.download()

    assert result == mock_genome


def test_genome_api_add_custom_cortical_area(genome_api):
    """GenomeAPI.add_custom_cortical_area sends correct payload."""
    mock_response = {"message": "Cortical area created", "cortical_id": "c_abc123"}

    with patch("feagi.genome.api.requests.request") as mock_req:
        mock_req.return_value.status_code = 200
        mock_req.return_value.json.return_value = mock_response
        mock_req.return_value.content = b"{}"

        result = genome_api.add_custom_cortical_area(
            cortical_name="my_area",
            cortical_dimensions=(10, 10, 10),
            coordinates_3d=(0, 0, 0),
        )

    assert result["cortical_id"] == "c_abc123"
    call_kwargs = mock_req.call_args[1]
    payload = call_kwargs.get("json") or call_kwargs.get("json_data")
    assert payload["cortical_name"] == "my_area"
    assert payload["cortical_dimensions"] == [10, 10, 10]
    assert payload["coordinates_3d"] == [0, 0, 0]


def test_genome_api_update_cortical_area(genome_api):
    """GenomeAPI.update_cortical_area sends PUT with changes."""
    mock_response = {"message": "Cortical area updated", "cortical_id": "c_xyz", "previous_cortical_id": "c_old"}

    with patch("feagi.genome.api.requests.request") as mock_req:
        mock_req.return_value.status_code = 200
        mock_req.return_value.json.return_value = mock_response
        mock_req.return_value.content = b"{}"

        result = genome_api.update_cortical_area("c_old", {"name": "NewName"})

    assert result["cortical_id"] == "c_xyz"
    call_kwargs = mock_req.call_args[1]
    payload = call_kwargs.get("json") or call_kwargs.get("json_data")
    assert payload["cortical_id"] == "c_old"
    assert payload["name"] == "NewName"


def test_genome_api_remove_cortical_area(genome_api):
    """GenomeAPI.remove_cortical_area sends DELETE with cortical_id."""
    with patch("feagi.genome.api.requests.request") as mock_req:
        mock_req.return_value.status_code = 200
        mock_req.return_value.json.return_value = {"message": "Cortical area deleted"}
        mock_req.return_value.content = b"{}"

        genome_api.remove_cortical_area("c_to_remove")

    call_kwargs = mock_req.call_args[1]
    payload = call_kwargs.get("json") or call_kwargs.get("json_data")
    assert payload["cortical_id"] == "c_to_remove"
    assert mock_req.call_args[0][0] == "DELETE"


def test_genome_api_list_cortical_area_ids(genome_api):
    """GenomeAPI.list_cortical_area_ids returns list from response."""
    mock_response = {"cortical_ids": ["c_1", "c_2", "c_3"]}

    with patch("feagi.genome.api.requests.request") as mock_req:
        mock_req.return_value.status_code = 200
        mock_req.return_value.json.return_value = mock_response
        mock_req.return_value.content = b"{}"

        result = genome_api.list_cortical_area_ids()

    assert result == ["c_1", "c_2", "c_3"]


def test_genome_api_error_on_http_4xx(genome_api):
    """GenomeAPI raises GenomeAPIError on HTTP 4xx/5xx."""
    with patch("feagi.genome.api.requests.request") as mock_req:
        mock_req.return_value.status_code = 404
        mock_req.return_value.text = "Not Found"
        mock_req.return_value.content = b""

        with pytest.raises(GenomeAPIError) as exc_info:
            genome_api.download()

    assert exc_info.value.status_code == 404
    assert "404" in str(exc_info.value)
