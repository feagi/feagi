# Genome module

The genome module provides external artifact I/O, schema validation and repair,
and live FEAGI API operations.

## Artifact boundary

`artifact.py` owns the `.genome` filename contract and converts external bytes
to and from genome dictionaries. The current `JsonGenomeArtifactCodec` uses
UTF-8 JSON and `application/vnd.feagi.genome+json`.

Artifact encoding does not version or migrate genomes. The decoded dictionary's
`genome_schema_version` is handled by the Rust-backed validation and migration
pipeline exported from `feagi.genome`.

File-oriented callers should use `read_genome_artifact()` and
`write_genome_artifact()`. JSON request bodies used by FEAGI REST endpoints are
API documents rather than `.genome` artifacts and should continue using normal
JSON request handling.

## Future encodings

The `.genome` extension is representation-neutral. A future codec may change
the byte encoding while preserving the artifact filename and existing genome
schema-version chain. New encodings must be selected explicitly; this module
does not guess or fall back between formats.
