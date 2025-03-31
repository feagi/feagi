"""Server script for running the FEAGI API."""
import os
import argparse
import uvicorn
from feagi.api.app import create_app

def main():
    """Run the FEAGI API server."""
    parser = argparse.ArgumentParser(description="FEAGI API Server")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Host to run the server on")
    parser.add_argument("--port", type=int, default=8000, help="Port to run the server on")
    parser.add_argument("--reload", action="store_true", help="Enable auto-reload")
    args = parser.parse_args()
    
    print(f"Starting FEAGI API server on {args.host}:{args.port}")
    uvicorn.run(
        "feagi.api.app:create_app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        factory=True,
    )

if __name__ == "__main__":
    main() 