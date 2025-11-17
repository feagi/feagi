"""Generic RPC Proxy for API Subprocess

This module provides a generic proxy that forwards method calls from the API
subprocess to the main FEAGI process via ZMQ, eliminating the need to manually
implement every API endpoint in the subprocess.

Architecture:
    API Subprocess → ZmqRpcProxy.__getattr__() → ZMQ → Main Process → Real Service
    
Usage:
    # In API subprocess:
    real_service = CoreAPIService()  # This becomes a proxy
    result = real_service.load_genome(genome_data)  # Forwarded via ZMQ
"""

import json
import os
from typing import Any
import feagi_rust_py_libs.feagi_rust_py_libs as frl

# File-based logging for debugging (survives uvicorn takeover)
_RPC_LOG_FILE = "/Users/nadji/code/FEAGI-2.0/feagi-py/tmp/rpc_proxy.log"
_rpc_log = None

def _log_rpc(msg: str):
    """Write to RPC log file (persists after uvicorn starts)."""
    global _rpc_log
    if _rpc_log is None:
        _rpc_log = open(_RPC_LOG_FILE, "a", buffering=1)
    print(msg, file=_rpc_log, flush=True)
    print(msg, flush=True)  # Also to console


class ZmqRpcProxy:
    """Generic RPC proxy that forwards all method calls via ZMQ.
    
    This proxy intercepts attribute access and method calls, serializes them,
    sends them over ZMQ to the main process, and deserializes the response.
    
    This is a proper class (not a function) so it can be subclassed.
    """
    
    def __init__(self, zmq_address: str = None, service_name: str = None, *args, **kwargs):
        """Initialize RPC proxy.
        
        Args:
            zmq_address: ZMQ address of main process (e.g., "tcp://127.0.0.1:5565")
            service_name: Name of service being proxied (for debugging)
            *args, **kwargs: Ignored (for compatibility with real service constructors)
        """
        # Handle case where called without args (for inheritance)
        if zmq_address is None:
            zmq_address = "tcp://0.0.0.0:5565"  # Default
        if service_name is None:
            service_name = self.__class__.__name__
            
        _log_rpc(f"🦀 [RPC-PROXY-INIT] Initializing proxy for {service_name} @ {zmq_address}")
        ZmqApiClient = frl.feagi_python.ZmqApiClient
        self._client = ZmqApiClient(zmq_address)
        self._client.connect()
        self._service_name = service_name
        self._zmq_address = zmq_address
        _log_rpc(f"🦀 [RPC-PROXY-INIT] ✅ Connected to main process")
    
    def __getattr__(self, method_name: str):
        """Intercept method calls and forward via ZMQ.
        
        Args:
            method_name: Name of method being called
            
        Returns:
            Callable that forwards the call to main process
        """
        _log_rpc(f"🦀 [RPC-PROXY-GETATTR] Attribute access: {self._service_name}.{method_name}")
        
        # Return an async function so API endpoints can await it
        async def rpc_call(*args, **kwargs):
            _log_rpc(f"🦀 [RPC-PROXY-CALL] {self._service_name}.{method_name}() INVOKED with args={args[:50] if args else []}, kwargs={list(kwargs.keys())}")
            
            try:
                # Serialize call
                rpc_payload = {
                    "method": method_name,
                    "args": args,
                    "kwargs": kwargs
                }
                
                _log_rpc(f"🦀 [RPC-PROXY-CALL] Payload serialized, sending ZMQ request...")
                
                # Send via ZMQ (POST to /rpc/core_api)
                response = self._client.request("POST", "/rpc/core_api", json.dumps(rpc_payload))
                
                _log_rpc(f"🦀 [RPC-PROXY-CALL] ZMQ response received: status={response.get('status')}")
                
                # Check response status
                status = response.get("status", 500)
                body = response.get("body", {})
                
                if status != 200:
                    error_msg = body.get("error", "Unknown RPC error")
                    _log_rpc(f"🦀 [RPC-PROXY-CALL] ❌ {method_name}() failed: {error_msg}")
                    raise RuntimeError(f"RPC call failed: {error_msg}")
                
                _log_rpc(f"🦀 [RPC-PROXY-CALL] ✅ {method_name}() success, returning result")
                return body.get("result")
                
            except Exception as e:
                _log_rpc(f"🦀 [RPC-PROXY-CALL] 💥 EXCEPTION in RPC call: {e}")
                import traceback
                traceback.print_exc()
                _log_rpc(traceback.format_exc())
                raise
        
        _log_rpc(f"🦀 [RPC-PROXY-GETATTR] Returning async callable for {method_name}")
        return rpc_call
    
    def __repr__(self):
        return f"<ZmqRpcProxy({self._service_name})>"


def create_core_api_service_proxy(zmq_address: str):
    """Create a CoreAPIService proxy class that can be inherited from.
    
    This is a factory that creates a class (not an instance) which inherits from
    ZmqRpcProxy. This allows other code to inherit from it: class X(CoreAPIService).
    
    Args:
        zmq_address: ZMQ address of main process
        
    Returns:
        A class that inherits from ZmqRpcProxy
    """
    class CoreAPIServiceProxy(ZmqRpcProxy):
        """Proxy for CoreAPIService that forwards all calls via ZMQ."""
        
        def __init__(self, *args, **kwargs):
            # Ignore all args (connectome_manager, state_manager, etc.)
            # and just initialize the RPC proxy
            super().__init__(zmq_address=zmq_address, service_name="CoreAPIService")
    
    return CoreAPIServiceProxy

