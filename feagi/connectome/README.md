# Connectome artifacts

The connectome module provides binary `.connectome` filesystem I/O, FEAGI
core's authoritative validation and migration, and REST operations against a
running FEAGI instance.

`validate_and_migrate_connectome()` accepts immutable artifact bytes and returns
new bytes with a structured compatibility report. Connectome-lite migration
upgrades its embedded genome first, rewrites semantic cortical references, then
validates the complete artifact. Invalid artifacts raise `ValueError`; the
module does not guess formats or provide permissive fallbacks.

An artifact written by a newer FEAGI build raises
`UnsupportedArtifactVersionError`, a `ValueError` subclass (so existing
`except ValueError` handlers still catch it). It carries `component`
(`"contract"`, `"container"`, `"connectome_schema"` or `"genome_schema"`),
`artifact_version` and `supported_version`; its message starts with
`Connectome validation failed: `. The remedy is upgrading the reader, not
repairing the file. `supported_artifact_versions()` returns the highest version
this build reads as a `dict[str, int]` keyed by those same component names.

```python
from feagi.connectome import (
    UnsupportedArtifactVersionError,
    supported_artifact_versions,
    validate_and_migrate_connectome,
)

try:
    migrated, report = validate_and_migrate_connectome(artifact_bytes)
except UnsupportedArtifactVersionError as error:
    print(error.component, error.artifact_version, error.supported_version)
    print(supported_artifact_versions())
```

`UnsupportedArtifactVersionError` is resolved lazily from
`feagi_rust_py_libs.connectome` on first access, so importing `feagi.connectome`
does not require the Rust library; accessing the class does, and requires a
`feagi-rust-py-libs` release that provides it.

The binary parser and migration rules remain in `feagi-services`. This Python
module uses `feagi_rust_py_libs.connectome` as its local typed adapter.

Use `read_connectome_artifact()` and `write_connectome_artifact()` for
filesystem operations. Both enforce the `.connectome` extension.

Use `ConnectomeAPI` to upload, download, validate, or migrate artifacts through
FEAGI's REST API:

```python
from feagi.connectome import ConnectomeAPI

api = ConnectomeAPI(api_url, timeout=request_timeout)
api.upload_file("trained.connectome")
api.download_to_file("checkpoint.connectome", mode="full")
report = api.validate_file("checkpoint.connectome")
api.migrate_file("old.connectome", "migrated.connectome")
```

The API URL and timeout must come from the caller's active FEAGI configuration.
