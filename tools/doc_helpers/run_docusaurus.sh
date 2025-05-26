#!/bin/bash
# Script to run Docusaurus development server with proper error handling

# Go to the Docusaurus directory
cd "$(dirname "$0")/../../website/docs" || { echo "Could not navigate to Docusaurus directory"; exit 1; }

# Make sure no other Docusaurus server is running
echo "Ensuring no other Docusaurus servers are running..."
pkill -f "npm start" || echo "No existing Docusaurus servers found"

# Wait a moment to ensure ports are freed
sleep 1

# Try different ports if needed
port=3000
max_attempts=5
attempt=1

while [ $attempt -le $max_attempts ]; do
    echo "Attempting to start Docusaurus on port $port (attempt $attempt/$max_attempts)..."
    
    # Start Docusaurus in the background
    npm start -- --port $port &
    pid=$!
    
    # Wait for server to start or fail
    for i in {1..20}; do
        if grep -q "Docusaurus website is running at" <(tail -n 20 /tmp/docusaurus_log.txt 2>/dev/null); then
            echo "Docusaurus started successfully on port $port"
            echo "View the documentation at http://localhost:$port"
            echo "Press Ctrl+C to stop the server"
            wait $pid
            exit 0
        fi
        
        if ! ps -p $pid > /dev/null; then
            echo "Docusaurus process exited unexpectedly"
            break
        fi
        
        sleep 1
    done
    
    # Kill the process if it's still running
    if ps -p $pid > /dev/null; then
        kill $pid 2>/dev/null
    fi
    
    port=$((port + 1))
    attempt=$((attempt + 1))
    sleep 2
done

echo "Failed to start Docusaurus after $max_attempts attempts"
echo "Check for error messages or try manually with: cd website/docs && npm start"
exit 1 