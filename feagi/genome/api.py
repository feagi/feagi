"""
GenomeAPI - REST client for genome and cortical area operations on a running FEAGI instance.

Add/remove cortical areas, modify parameters, upload/download genomes via FEAGI REST API.
"""

from __future__ import annotations

import json
from typing import Any

import requests


class GenomeAPIError(Exception):
    """Raised when a GenomeAPI operation fails."""

    def __init__(self, message: str, status_code: int | None = None, response_text: str = ""):
        super().__init__(message)
        self.status_code = status_code
        self.response_text = response_text


class GenomeAPI:
    """
    REST client for genome and cortical area operations on a running FEAGI instance.

    All methods require the FEAGI API to be running and reachable at base_url.
    """

    def __init__(self, base_url: str, timeout: float = 30.0):
        """
        Initialize GenomeAPI client.

        Args:
            base_url: FEAGI API base URL (e.g., "http://localhost:8000").
            timeout: Request timeout in seconds.
        """
        self._base_url = base_url.rstrip("/")
        self._timeout = timeout

    def _url(self, path: str) -> str:
        """Build full URL for an API path."""
        return f"{self._base_url}/v1{path}"

    def _request(
        self,
        method: str,
        path: str,
        json_data: dict[str, Any] | None = None,
    ) -> dict[str, Any] | list[Any]:
        """Execute HTTP request and return JSON response."""
        url = self._url(path)
        try:
            resp = requests.request(
                method,
                url,
                json=json_data,
                timeout=self._timeout,
            )
        except requests.RequestException as e:
            raise GenomeAPIError(f"Request failed: {e}") from e

        if resp.status_code >= 400:
            raise GenomeAPIError(
                f"API error {resp.status_code}: {resp.text[:500]}",
                status_code=resp.status_code,
                response_text=resp.text,
            )

        if not resp.content:
            return {}

        try:
            return resp.json()
        except json.JSONDecodeError as e:
            raise GenomeAPIError(
                f"Invalid JSON response: {e}",
                status_code=resp.status_code,
                response_text=resp.text[:500],
            ) from e

    # -------------------------------------------------------------------------
    # Genome operations
    # -------------------------------------------------------------------------

    def upload(self, genome: dict[str, Any]) -> dict[str, Any]:
        """
        Upload and load a genome on the running FEAGI instance.

        Args:
            genome: Genome dict (version, blueprint, neuron_morphologies, physiology, etc.).

        Returns:
            Response with success, cortical_area_count, brain_region_count.
        """
        result = self._request("POST", "/genome/upload", json_data=genome)
        return result if isinstance(result, dict) else {}

    def download(self) -> dict[str, Any]:
        """
        Download the current genome from the running FEAGI instance.

        Returns:
            Genome dict (version, blueprint, neuron_morphologies, physiology, etc.).
        """
        result = self._request("GET", "/genome/download")
        return result if isinstance(result, dict) else {}

    def add_custom_cortical_area(
        self,
        cortical_name: str,
        cortical_dimensions: tuple[int, int, int],
        coordinates_3d: tuple[int, int, int],
        brain_region_id: str | None = None,
        cortical_sub_group: str | None = None,
        sub_group_id: str | None = None,
    ) -> dict[str, Any]:
        """
        Add a custom cortical area (or memory area if sub_group_id="MEMORY").

        Args:
            cortical_name: Human-readable name for the area.
            cortical_dimensions: (x, y, z) dimensions.
            coordinates_3d: (x, y, z) position.
            brain_region_id: Optional parent brain region ID.
            cortical_sub_group: Optional cortical sub-group.
            sub_group_id: "MEMORY" for memory area, else custom.

        Returns:
            Response with message and cortical_id.
        """
        payload: dict[str, Any] = {
            "cortical_name": cortical_name,
            "cortical_dimensions": list(cortical_dimensions),
            "coordinates_3d": list(coordinates_3d),
        }
        if brain_region_id is not None:
            payload["brain_region_id"] = brain_region_id
        if cortical_sub_group is not None:
            payload["cortical_sub_group"] = cortical_sub_group
        if sub_group_id is not None:
            payload["sub_group_id"] = sub_group_id

        result = self._request(
            "POST",
            "/cortical_area/custom_cortical_area",
            json_data=payload,
        )
        return result if isinstance(result, dict) else {}

    def update_cortical_area(
        self,
        cortical_id: str,
        changes: dict[str, Any],
    ) -> dict[str, Any]:
        """
        Update properties of an existing cortical area.

        Args:
            cortical_id: Cortical area ID to update.
            changes: Map of property_name -> new_value (e.g., {"name": "NewName"}).

        Returns:
            Response with message, cortical_id, previous_cortical_id.
        """
        payload = {"cortical_id": cortical_id, **changes}
        result = self._request(
            "PUT",
            "/cortical_area/cortical_area",
            json_data=payload,
        )
        return result if isinstance(result, dict) else {}

    def remove_cortical_area(self, cortical_id: str) -> dict[str, Any]:
        """
        Remove a cortical area and all associated neurons and synapses.

        Args:
            cortical_id: Cortical area ID to remove.

        Returns:
            Response with message.
        """
        result = self._request(
            "DELETE",
            "/cortical_area/cortical_area",
            json_data={"cortical_id": cortical_id},
        )
        return result if isinstance(result, dict) else {}

    def list_cortical_area_ids(self) -> list[str]:
        """
        Get list of all cortical area IDs.

        Returns:
            List of cortical area ID strings.
        """
        result = self._request("GET", "/cortical_area/cortical_area_id_list")
        if isinstance(result, dict) and "cortical_ids" in result:
            return list(result["cortical_ids"])
        return []

    def list_cortical_area_names(self) -> list[str]:
        """
        Get list of all cortical area names (human-readable labels).

        Returns:
            List of cortical area name strings.
        """
        result = self._request("GET", "/cortical_area/cortical_area_name_list")
        if isinstance(result, dict) and "cortical_area_name_list" in result:
            return list(result["cortical_area_name_list"])
        return []

    def get_cortical_area_properties(self, cortical_id: str) -> dict[str, Any]:
        """
        Get properties of a cortical area.

        Args:
            cortical_id: Cortical area ID.

        Returns:
            Cortical area properties dict.
        """
        result = self._request(
            "POST",
            "/cortical_area/cortical_area_properties",
            json_data={"cortical_id": cortical_id},
        )
        return result if isinstance(result, dict) else {}
