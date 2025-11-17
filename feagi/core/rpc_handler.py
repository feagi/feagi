"""RPC Handler for Main FEAGI Process

This module handles RPC calls from the API subprocess, forwarding them to the
real CoreAPIService and returning the results.

Architecture:
    API Subprocess → ZMQ → Rust ApiControlStream → This Handler → Real Service → Result
"""

import json
import logging
import asyncio
import inspect
from typing import Any, Dict

logger = logging.getLogger(__name__)


class CoreAPIServiceRpcHandler:
    """RPC handler that forwards method calls to the real CoreAPIService."""
    
    def __init__(self, core_api_service):
        """Initialize RPC handler.
        
        Args:
            core_api_service: Real CoreAPIService instance in main process
        """
        self.core_api_service = core_api_service
        logger.info("🦀 [RPC-HANDLER] Initialized for CoreAPIService")
    
    def handle_rpc_call(self, method_name: str, payload_json: str) -> str:
        """Handle an RPC call from API subprocess.
        
        Args:
            method_name: Name of method to call on CoreAPIService
            payload_json: JSON string containing {"method": str, "args": list, "kwargs": dict}
            
        Returns:
            JSON string containing {"result": Any} or raises exception
        """
        try:
            print(f"🦀 [RPC-HANDLER] Received call: method={method_name}", flush=True)
            
            # Parse payload
            payload = json.loads(payload_json)
            args = payload.get("args", [])
            kwargs = payload.get("kwargs", {})
            
            print(f"🦀 [RPC-HANDLER] Parsed payload: args={args}, kwargs={kwargs}", flush=True)
            logger.debug(f"🦀 [RPC-HANDLER] {method_name}(*{args}, **{kwargs})")
            
            # Get method from service
            if not hasattr(self.core_api_service, method_name):
                raise AttributeError(f"CoreAPIService has no method '{method_name}'")
            
            method = getattr(self.core_api_service, method_name)
            print(f"🦀 [RPC-HANDLER] Calling method: {method}", flush=True)
            
            # Call method
            result = method(*args, **kwargs)
            
            print(f"🦀 [RPC-HANDLER] Method returned: {type(result)}", flush=True)
            
            # If result is a coroutine, await it
            if inspect.iscoroutine(result):
                print(f"🦀 [RPC-HANDLER] Method is async, awaiting coroutine...", flush=True)
                result = asyncio.run(result)
                print(f"🦀 [RPC-HANDLER] Coroutine completed, result: {type(result)}", flush=True)
            
            # Return serialized result
            response = {"result": result}
            result_json = json.dumps(response, default=str)  # default=str handles non-JSON types
            
            print(f"🦀 [RPC-HANDLER] {method_name}() → success (response len={len(result_json)})", flush=True)
            logger.debug(f"🦀 [RPC-HANDLER] {method_name}() → success")
            return result_json
            
        except Exception as e:
            print(f"🦀 [RPC-HANDLER] {method_name}() → ERROR: {e}", flush=True)
            logger.error(f"🦀 [RPC-HANDLER] {method_name}() → error: {e}")
            import traceback
            traceback.print_exc()
            # Re-raise to let Rust handle error response
            raise

