#!/usr/bin/env python3
"""
Test script for the ZMQ REST API client.

This script demonstrates how to use the standardized ZMQ REST API protocol
to send control commands to FEAGI. It provides the same API interface as
HTTP-based REST API clients but uses ZMQ as the transport.
"""

import sys
import time
import argparse
import json
from typing import Dict, Any, List, Optional

# Add FEAGI to Python path
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import the ZMQ REST client
from feagi.api.zmq.rest_client import ZMQRestClient

def print_json(data: Any):
    """Print data as formatted JSON."""
    print(json.dumps(data, indent=2))

def test_system_endpoints(client: ZMQRestClient):
    """Test system-related endpoints."""
    print("\n----- Testing System Endpoints -----\n")
    
    # Get system health
    print("Getting system health...")
    try:
        health = client.get_health()
        print_json(health)
    except Exception as e:
        print(f"Error getting health: {e}")
    
    # Get system configuration
    print("\nGetting system configuration...")
    try:
        config = client.get_configuration()
        print_json(config)
    except Exception as e:
        print(f"Error getting configuration: {e}")
    
    # Get system versions
    print("\nGetting system versions...")
    try:
        versions = client.get_versions()
        print_json(versions)
    except Exception as e:
        print(f"Error getting versions: {e}")
    
    # Get cortical area types
    print("\nGetting cortical area types...")
    try:
        types = client.get_cortical_area_types()
        print_json(types)
    except Exception as e:
        print(f"Error getting cortical area types: {e}")

def test_genome_endpoints(client: ZMQRestClient):
    """Test genome-related endpoints."""
    print("\n----- Testing Genome Endpoints -----\n")
    
    # Get system status first to check if genome is loaded
    print("Checking genome status...")
    try:
        status = client.get_status()
        print_json(status)
        
        if not status.get('genome_availability', False):
            print("\nNo genome is loaded. Some tests will be skipped.")
            return
    except Exception as e:
        print(f"Error getting status: {e}")
        return
    
    # Get genome blueprint
    print("\nGetting genome blueprint...")
    try:
        blueprint = client.get_genome_blueprint()
        # Just print a summary to avoid overwhelming output
        print(f"Blueprint contains {len(blueprint)} cortical areas")
        # Print first cortical area as example
        if blueprint:
            first_area = next(iter(blueprint.items()))
            print(f"Example area '{first_area[0]}':")
            print_json(first_area[1])
    except Exception as e:
        print(f"Error getting genome blueprint: {e}")
    
    # Get full genome
    print("\nGetting full genome...")
    try:
        genome = client.get_genome()
        # Just print keys to avoid overwhelming output
        print(f"Genome contains these sections: {', '.join(genome.keys())}")
    except Exception as e:
        print(f"Error getting genome: {e}")

def test_connectome_endpoints(client: ZMQRestClient):
    """Test connectome-related endpoints."""
    print("\n----- Testing Connectome Endpoints -----\n")
    
    # Get all cortical areas
    print("Getting all cortical areas...")
    try:
        areas = client.get_cortical_areas()
        print(f"Found {len(areas)} cortical areas")
        
        # If areas exist, get details for the first one
        if areas:
            area_id = areas[0]['id']
            print(f"\nGetting details for cortical area {area_id}...")
            area = client.get_cortical_area(area_id)
            print_json(area)
    except Exception as e:
        print(f"Error accessing cortical areas: {e}")

def test_custom_request(client: ZMQRestClient, method: str, route: str, body: Optional[Dict] = None):
    """Test a custom request to any endpoint."""
    print(f"\n----- Testing Custom Request: {method} {route} -----\n")
    
    try:
        if method.upper() == 'GET':
            response = client.get(route)
        elif method.upper() == 'POST':
            response = client.post(route, body or {})
        elif method.upper() == 'PUT':
            response = client.put(route, body or {})
        elif method.upper() == 'DELETE':
            response = client.delete(route)
        else:
            print(f"Unsupported method: {method}")
            return
            
        print_json(response)
    except Exception as e:
        print(f"Error with custom request: {e}")

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Test FEAGI ZMQ REST API')
    parser.add_argument('--host', default='localhost', help='FEAGI host')
    parser.add_argument('--port', type=int, default=5555, help='FEAGI ZMQ control port')
    parser.add_argument('--test', choices=['system', 'genome', 'connectome', 'all'], 
                        default='all', help='Test category to run')
    parser.add_argument('--custom', action='store_true', help='Run a custom request')
    parser.add_argument('--method', choices=['GET', 'POST', 'PUT', 'DELETE'], 
                        help='HTTP method for custom request')
    parser.add_argument('--route', help='API route for custom request')
    parser.add_argument('--body', help='JSON body for custom request')
    
    args = parser.parse_args()
    
    # Create client
    client = ZMQRestClient(host=args.host, port=args.port)
    
    try:
        # Connect to FEAGI
        print(f"Connecting to FEAGI at {args.host}:{args.port}...")
        client.connect()
        print("Connected successfully")
        
        # Run requested tests
        if args.custom and args.method and args.route:
            # Parse body if provided
            body = None
            if args.body:
                try:
                    body = json.loads(args.body)
                except json.JSONDecodeError:
                    print(f"Error: Invalid JSON body: {args.body}")
                    return 1
            
            # Run custom test
            test_custom_request(client, args.method, args.route, body)
        else:
            # Run standard tests
            if args.test in ['system', 'all']:
                test_system_endpoints(client)
            
            if args.test in ['genome', 'all']:
                test_genome_endpoints(client)
            
            if args.test in ['connectome', 'all']:
                test_connectome_endpoints(client)
        
    except ConnectionError as e:
        print(f"Connection error: {e}")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        # Disconnect from FEAGI
        client.disconnect()
        print("\nDisconnected from FEAGI")
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 