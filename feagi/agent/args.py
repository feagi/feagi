"""
Shared FEAGI connection arguments for controllers.

Every controller that talks to FEAGI needs the same block of connection
arguments (host, ports, timeouts, retries, heartbeat, auth token, agent id,
burst period). This module declares that block once so controllers do not
re-declare it by hand with diverging names.

Example::

    parser = argparse.ArgumentParser()
    add_feagi_connection_args(parser)
    args = parser.parse_args()
    brain_output.configure_from_args(args)
"""

from __future__ import annotations

import argparse
import os

AUTH_TOKEN_ENV_VAR = "FEAGI_AUTH_TOKEN_B64"


def add_feagi_connection_args(parser: argparse.ArgumentParser) -> None:
    """
    Add the shared FEAGI connection arguments to ``parser``.

    All values are explicit (supplied by the launcher); only the auth token
    has a fallback, the ``FEAGI_AUTH_TOKEN_B64`` environment variable read at
    call time.
    """
    parser.add_argument("--ip", required=True, help="FEAGI API host/IP")
    parser.add_argument(
        "--port", type=int, required=True, help="FEAGI HTTP API port"
    )
    parser.add_argument("--feagi-zmq-motor-port", type=int, required=True)
    parser.add_argument(
        "--feagi-zmq-registration-port", type=int, required=True
    )
    parser.add_argument("--feagi-zmq-sensory-port", type=int, required=True)
    parser.add_argument(
        "--feagi-zmq-connection-timeout-ms", type=int, required=True
    )
    parser.add_argument(
        "--feagi-zmq-registration-retries", type=int, required=True
    )
    parser.add_argument(
        "--feagi-zmq-heartbeat-interval-s", type=float, required=True
    )
    parser.add_argument("--feagi-http-timeout-s", type=float, required=True)
    parser.add_argument(
        "--feagi-burst-period-s",
        type=float,
        required=True,
        help="Control loop period in seconds (from the FEAGI burst config)",
    )
    parser.add_argument(
        "--auth-token-b64",
        default=os.environ.get(AUTH_TOKEN_ENV_VAR),
        help=(
            "Base64 auth token (decodes to 32 bytes). "
            f"Defaults to {AUTH_TOKEN_ENV_VAR}."
        ),
    )
    parser.add_argument(
        "--agent_id",
        required=True,
        help="Base64 AgentDescriptor (48-byte payload) for FEAGI registration",
    )
