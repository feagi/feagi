"""Rust connectome serialization integration.

This module provides Python wrappers for the Rust-based connectome serialization
system. It bridges the Python FEAGI API with the high-performance Rust NPU
for saving and loading brain states.

Design:
- Export connectome: RustNPU (Python) → Rust NPU → Binary .connectome file
- Import connectome: .connectome file → Rust NPU → RustNPU (Python)
- Supports full brain state: neurons, synapses, cortical areas, runtime state

Benefits:
- High performance: ~100x faster than Python JSON serialization
- Compact: Binary format with optional compression
- Portable: Can be loaded by standalone Rust inference engine
- Complete: Captures entire brain state for exact restore
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional
import datetime

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class RustConnectomeSerializer:
    """Rust-based connectome serialization for high-performance brain state management."""

    @staticmethod
    def export_to_file(
        npu,
        output_path: Path,
        metadata: Optional[dict] = None
    ) -> Path:
        """Export a Rust NPU connectome to a .connectome file.

        Args:
            npu: RustNPU instance from feagi_rust module
            output_path: Path where the .connectome file will be saved
            metadata: Optional metadata dict to include (for documentation purposes)

        Returns:
            Path to the saved .connectome file

        Raises:
            RuntimeError: If the NPU doesn't support connectome serialization
            IOError: If file writing fails
        """
        try:
            # Check if the NPU has the required methods
            if not hasattr(npu, 'save_connectome_to_file'):
                raise RuntimeError(
                    "NPU instance does not support connectome serialization. "
                    "Ensure feagi_rust is built with 'connectome-serialization' feature."
                )

            # Ensure output path has .connectome extension
            if not str(output_path).endswith('.connectome'):
                output_path = Path(str(output_path) + '.connectome')

            # Create parent directory if it doesn't exist
            output_path.parent.mkdir(parents=True, exist_ok=True)

            # Call Rust method to save connectome
            logger.info(f"Exporting Rust connectome to: {output_path}")
            success = npu.save_connectome_to_file(str(output_path))

            if not success:
                raise IOError(f"Failed to save connectome to {output_path}")

            # Optionally save metadata alongside (for documentation)
            if metadata:
                metadata_path = output_path.with_suffix('.connectome.meta.json')
                import json
                with open(metadata_path, 'w', encoding='utf-8') as f:
                    json.dump(metadata, f, indent=2)
                logger.info(f"Saved connectome metadata to: {metadata_path}")

            logger.info(f"✓ Connectome exported successfully: {output_path.stat().st_size} bytes")
            return output_path

        except Exception as e:
            logger.error(f"Failed to export Rust connectome: {e}")
            raise

    @staticmethod
    def import_from_file(
        npu,
        input_path: Path
    ) -> bool:
        """Import a connectome from a .connectome file into a Rust NPU.

        This completely replaces the NPU's state with the loaded connectome.

        Args:
            npu: RustNPU instance from feagi_rust module
            input_path: Path to the .connectome file to load

        Returns:
            True if import was successful

        Raises:
            FileNotFoundError: If the .connectome file doesn't exist
            RuntimeError: If the NPU doesn't support connectome serialization
            IOError: If file reading or deserialization fails
        """
        try:
            # Validate input path
            if not input_path.exists():
                raise FileNotFoundError(f"Connectome file not found: {input_path}")

            # Check if the NPU has the required methods
            if not hasattr(npu, 'load_connectome_from_file'):
                raise RuntimeError(
                    "NPU instance does not support connectome serialization. "
                    "Ensure feagi_rust is built with 'connectome-serialization' feature."
                )

            # Call Rust method to load connectome
            logger.info(f"Importing Rust connectome from: {input_path}")
            success = npu.load_connectome_from_file(str(input_path))

            if not success:
                raise IOError(f"Failed to load connectome from {input_path}")

            logger.info(f"✓ Connectome imported successfully from: {input_path}")
            return True

        except Exception as e:
            logger.error(f"Failed to import Rust connectome: {e}")
            raise

    @staticmethod
    def export_to_bytes(npu) -> bytes:
        """Export a Rust NPU connectome to bytes (in-memory).

        Useful for network transmission or in-memory checkpointing.

        Args:
            npu: RustNPU instance from feagi_rust module

        Returns:
            Binary data containing the serialized connectome

        Raises:
            RuntimeError: If the NPU doesn't support connectome serialization
        """
        try:
            if not hasattr(npu, 'export_connectome_bytes'):
                raise RuntimeError(
                    "NPU instance does not support connectome serialization. "
                    "Ensure feagi_rust is built with 'connectome-serialization' feature."
                )

            logger.debug("Exporting Rust connectome to bytes")
            binary_data = npu.export_connectome_bytes()
            logger.debug(f"✓ Connectome exported to bytes: {len(binary_data)} bytes")
            return binary_data

        except Exception as e:
            logger.error(f"Failed to export Rust connectome to bytes: {e}")
            raise

    @staticmethod
    def import_from_bytes(npu, binary_data: bytes) -> bool:
        """Import a connectome from bytes into a Rust NPU.

        This completely replaces the NPU's state with the loaded connectome.

        Args:
            npu: RustNPU instance from feagi_rust module
            binary_data: Binary data from export_to_bytes()

        Returns:
            True if import was successful

        Raises:
            RuntimeError: If the NPU doesn't support connectome serialization
            ValueError: If binary data is invalid
        """
        try:
            if not hasattr(npu, 'import_connectome_bytes'):
                raise RuntimeError(
                    "NPU instance does not support connectome serialization. "
                    "Ensure feagi_rust is built with 'connectome-serialization' feature."
                )

            if not binary_data:
                raise ValueError("Binary data is empty")

            logger.debug(f"Importing Rust connectome from bytes: {len(binary_data)} bytes")
            success = npu.import_connectome_bytes(binary_data)

            if not success:
                raise ValueError("Failed to import connectome from bytes")

            logger.debug("✓ Connectome imported successfully from bytes")
            return True

        except Exception as e:
            logger.error(f"Failed to import Rust connectome from bytes: {e}")
            raise

    @staticmethod
    def create_snapshot(
        npu,
        snapshot_dir: Path,
        snapshot_id: str,
        description: str = ""
    ) -> dict:
        """Create a complete snapshot with Rust connectome + metadata.

        Creates a snapshot directory containing:
        - <snapshot_id>.connectome: Binary brain state
        - manifest.json: Snapshot metadata
        - <snapshot_id>.connectome.meta.json: Connectome-specific metadata

        Args:
            npu: RustNPU instance
            snapshot_dir: Directory to create snapshot in
            snapshot_id: Unique snapshot identifier
            description: Human-readable description

        Returns:
            Dictionary with snapshot info:
                - snapshot_id: str
                - path: str (path to snapshot directory)
                - connectome_file: str (relative path to .connectome file)
                - size_bytes: int
                - timestamp: str (ISO format)
        """
        try:
            # Create snapshot directory
            snapshot_path = snapshot_dir / snapshot_id
            snapshot_path.mkdir(parents=True, exist_ok=True)

            # Export connectome
            connectome_file = snapshot_path / f"{snapshot_id}.connectome"
            timestamp = datetime.datetime.now(datetime.timezone.utc).isoformat()

            metadata = {
                "description": description,
                "timestamp": timestamp,
                "source": "rust_npu",
            }

            RustConnectomeSerializer.export_to_file(
                npu,
                connectome_file,
                metadata=metadata
            )

            # Create manifest
            manifest = {
                "snapshot_id": snapshot_id,
                "format_version": "2.0",
                "type": "rust_connectome",
                "created_at": timestamp,
                "description": description,
                "files": {
                    "connectome": f"{snapshot_id}.connectome",
                    "metadata": f"{snapshot_id}.connectome.meta.json",
                },
            }

            manifest_path = snapshot_path / "manifest.json"
            import json
            with open(manifest_path, 'w', encoding='utf-8') as f:
                json.dump(manifest, f, indent=2)

            return {
                "snapshot_id": snapshot_id,
                "path": str(snapshot_path),
                "connectome_file": f"{snapshot_id}.connectome",
                "size_bytes": connectome_file.stat().st_size,
                "timestamp": timestamp,
            }

        except Exception as e:
            logger.error(f"Failed to create Rust connectome snapshot: {e}")
            raise

    @staticmethod
    def restore_snapshot(
        npu,
        snapshot_dir: Path,
        snapshot_id: str
    ) -> bool:
        """Restore a Rust connectome snapshot.

        Args:
            npu: RustNPU instance to restore into
            snapshot_dir: Directory containing the snapshot
            snapshot_id: Snapshot identifier

        Returns:
            True if restore was successful

        Raises:
            FileNotFoundError: If snapshot files don't exist
        """
        try:
            snapshot_path = snapshot_dir / snapshot_id
            if not snapshot_path.exists():
                raise FileNotFoundError(f"Snapshot not found: {snapshot_path}")

            connectome_file = snapshot_path / f"{snapshot_id}.connectome"
            if not connectome_file.exists():
                raise FileNotFoundError(f"Connectome file not found: {connectome_file}")

            logger.info(f"Restoring Rust connectome snapshot: {snapshot_id}")
            success = RustConnectomeSerializer.import_from_file(npu, connectome_file)

            if success:
                logger.info(f"✓ Successfully restored snapshot: {snapshot_id}")

            return success

        except Exception as e:
            logger.error(f"Failed to restore Rust connectome snapshot: {e}")
            raise

