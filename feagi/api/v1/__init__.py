"""
FEAGI v1 API - Transport Agnostic Business Logic

This module contains the pure business logic for FEAGI's v1 API endpoints.
These functions are transport-agnostic and can be called by any adapter
(FastAPI, ZMQ, gRPC, etc.) with identical results.

The goal is to have a single source of truth for v1 API behavior that
can be accessed through multiple transport protocols.
"""
