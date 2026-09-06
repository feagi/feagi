"""REST client for binary connectome operations on a running FEAGI instance."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

import requests

from feagi.connectome.artifact import (
    CONNECTOME_ARTIFACT_MEDIA_TYPE,
    is_connectome_artifact_file_name,
    read_connectome_artifact,
    write_connectome_artifact,
)
from feagi.connectome.model import ConnectomeMigrationReport

ConnectomeMode = Literal["full", "lite"]


class ConnectomeAPIError(Exception):
    """Raised when a connectome REST operation fails."""

    def __init__(
        self,
        message: str,
        status_code: int | None = None,
        response_text: str = "",
    ) -> None:
        """Initialize an error with optional HTTP response details."""
        super().__init__(message)
        self.status_code = status_code
        self.response_text = response_text


class ConnectomeAPI:
    """Upload, download, validate, and migrate connectome artifacts via REST."""

    def __init__(self, base_url: str, timeout: float) -> None:
        """Initialize the client with an API URL and explicit request timeout."""
        self._base_url = base_url.rstrip("/")
        self._timeout = timeout

    def _url(self, path: str) -> str:
        """Build a versioned FEAGI API URL."""
        return f"{self._base_url}/v1{path}"

    def _request(
        self,
        method: str,
        path: str,
        *,
        files: dict[str, tuple[str, bytes, str]] | None = None,
        params: dict[str, str] | None = None,
    ) -> requests.Response:
        """Execute a connectome request and enforce successful HTTP status."""
        try:
            response = requests.request(
                method,
                self._url(path),
                files=files,
                params=params,
                timeout=self._timeout,
            )
        except requests.RequestException as exc:
            raise ConnectomeAPIError(f"Request failed: {exc}") from exc

        if response.status_code >= 400:
            raise ConnectomeAPIError(
                f"API error {response.status_code}: {response.text[:500]}",
                status_code=response.status_code,
                response_text=response.text,
            )
        return response

    @staticmethod
    def _upload_files(
        artifact: bytes,
        file_name: str,
    ) -> dict[str, tuple[str, bytes, str]]:
        """Build the multipart payload for a validated artifact filename."""
        if not is_connectome_artifact_file_name(file_name):
            raise ValueError(
                "Connectome files must use the .connectome extension"
            )
        return {
            "file": (
                file_name,
                artifact,
                CONNECTOME_ARTIFACT_MEDIA_TYPE,
            )
        }

    @staticmethod
    def _json_object(response: requests.Response) -> dict[str, Any]:
        """Decode a JSON object response or raise a typed API error."""
        try:
            payload = response.json()
        except requests.JSONDecodeError as exc:
            raise ConnectomeAPIError(
                f"Invalid JSON response: {exc}",
                status_code=response.status_code,
                response_text=response.text[:500],
            ) from exc
        if not isinstance(payload, dict):
            raise ConnectomeAPIError(
                "FEAGI API response must contain a JSON object",
                status_code=response.status_code,
                response_text=response.text[:500],
            )
        return payload

    def upload(self, artifact: bytes, file_name: str) -> dict[str, Any]:
        """Upload and restore a complete binary connectome artifact."""
        response = self._request(
            "POST",
            "/connectome/upload",
            files=self._upload_files(artifact, file_name),
        )
        return self._json_object(response)

    def upload_file(self, path: str | Path) -> dict[str, Any]:
        """Read and upload a `.connectome` artifact from disk."""
        artifact_path = Path(path)
        return self.upload(
            read_connectome_artifact(artifact_path),
            artifact_path.name,
        )

    def download(self, mode: ConnectomeMode = "full") -> bytes:
        """Download the running FEAGI connectome in full or lite mode."""
        if mode not in ("full", "lite"):
            raise ValueError("Connectome mode must be 'full' or 'lite'")
        response = self._request(
            "GET",
            "/connectome/download-bytes",
            params={"mode": mode},
        )
        return response.content

    def download_to_file(
        self,
        path: str | Path,
        mode: ConnectomeMode = "full",
    ) -> None:
        """Download the running connectome to a `.connectome` file."""
        write_connectome_artifact(path, self.download(mode))

    def validate(
        self,
        artifact: bytes,
        file_name: str,
    ) -> ConnectomeMigrationReport:
        """Validate an artifact without changing the running connectome."""
        response = self._request(
            "POST",
            "/connectome/validate",
            files=self._upload_files(artifact, file_name),
        )
        return ConnectomeMigrationReport(**self._json_object(response))

    def validate_file(self, path: str | Path) -> ConnectomeMigrationReport:
        """Read and validate a `.connectome` artifact from disk."""
        artifact_path = Path(path)
        return self.validate(
            read_connectome_artifact(artifact_path),
            artifact_path.name,
        )

    def migrate(self, artifact: bytes, file_name: str) -> bytes:
        """Return migrated artifact bytes without modifying the source."""
        response = self._request(
            "POST",
            "/connectome/migrate",
            files=self._upload_files(artifact, file_name),
        )
        return response.content

    def migrate_file(
        self,
        source: str | Path,
        destination: str | Path,
    ) -> None:
        """Migrate a source artifact and write a distinct destination file."""
        source_path = Path(source)
        migrated = self.migrate(
            read_connectome_artifact(source_path),
            source_path.name,
        )
        write_connectome_artifact(destination, migrated)


__all__ = [
    "ConnectomeAPI",
    "ConnectomeAPIError",
    "ConnectomeMode",
]
