# Connectome artifacts

The connectome module provides binary `.connectome` filesystem I/O, FEAGI
core's authoritative validation and migration, and REST operations against a
running FEAGI instance.

`validate_and_migrate_connectome()` accepts immutable artifact bytes and returns
new bytes with a structured compatibility report. Connectome-lite migration
upgrades its embedded genome first, rewrites semantic cortical references, then
validates the complete artifact. Invalid artifacts raise `ValueError`; the
module does not guess formats or provide permissive fallbacks.

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
