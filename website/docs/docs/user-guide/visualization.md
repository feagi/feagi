---
sidebar_position: 2
---

# Visualization Guide

FEAGI provides powerful visualization tools to help you understand and monitor the activity in your neural networks. This guide covers the visualization features and how to use them effectively.

## Recent Updates

The FEAGI visualization system has been enhanced with a new **threading-based implementation** that provides:

- **Enhanced Reliability**: Threading-based architecture eliminates previous connection issues
- **Automatic Client Management**: Real-time heartbeat monitoring and cleanup
- **Improved Performance**: Responsive shutdown and better resource management
- **Production-Ready**: Clean logging and error handling suitable for production environments

## Web Interface Visualizations

### 3D Brain Viewer

The 3D Brain Viewer provides an interactive visualization of your brain's structure and activity.

#### Key Features:

- **Brain Structure**: View the spatial arrangement of cortical areas
- **Neural Activity**: See real-time firing patterns with color-coded intensity
- **Interactive Navigation**: Rotate, zoom, and pan to explore the brain

#### Controls:

- Left-click + drag: Rotate the brain
- Right-click + drag: Pan the view
- Scroll wheel: Zoom in/out
- Double-click on a cortical area: Focus and display details

### Activity Monitors

Monitor detailed neural activity in specific brain regions:

- **Spike Raster Plot**: Visualize individual neuron firing over time
- **Heat Map**: View aggregate activity across cortical areas
- **Time Series**: Track activity patterns over time

## Connecting to FEAGI Visualization

### Enhanced Client Connection

The new visualization system requires proper client registration for optimal performance:

```python
import zmq
import requests
import threading
import time

class FeagiVisualizationClient:
    def __init__(self, feagi_host="localhost", client_id="my_viz_client"):
        self.feagi_host = feagi_host
        self.client_id = client_id
        self.running = False

        # Set up ZMQ connection for data
        self.context = zmq.Context()
        self.data_socket = self.context.socket(zmq.SUB)
        self.data_socket.connect(f"tcp://{feagi_host}:5562")
        self.data_socket.setsockopt(zmq.SUBSCRIBE, b"activity")

    def start(self):
        """Start the visualization client with heartbeat."""
        self.running = True

        # Start heartbeat thread
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_worker, daemon=True)
        self.heartbeat_thread.start()

        # Start data processing
        self._process_data()

    def stop(self):
        """Stop the client and cleanup."""
        self.running = False
        if hasattr(self, 'heartbeat_thread'):
            self.heartbeat_thread.join(timeout=1.0)
        self.context.term()

    def _heartbeat_worker(self):
        """Send periodic heartbeats to maintain connection."""
        while self.running:
            try:
                response = requests.post(
                    f"http://{self.feagi_host}:8000/v1/visualization/heartbeat",
                    json={"client_id": self.client_id},
                    timeout=5.0
                )
                if response.status_code == 200:
                    print(f"Heartbeat sent: {self.client_id}")
                else:
                    print(f"Heartbeat failed: {response.status_code}")
            except Exception as e:
                print(f"Heartbeat error: {e}")

            time.sleep(5)  # Send every 5 seconds

    def _process_data(self):
        """Process incoming visualization data."""
        while self.running:
            try:
                # Set timeout for responsive shutdown
                if self.data_socket.poll(timeout=1000):  # 1 second timeout
                    topic, data = self.data_socket.recv_multipart(zmq.NOBLOCK)
                    if topic == b"activity":
                        self._handle_neural_activity(data)
            except zmq.Again:
                continue  # Timeout, check running flag
            except Exception as e:
                print(f"Data processing error: {e}")

    def _handle_neural_activity(self, data):
        """Handle incoming neural activity data."""
        # Decode using feagi_bytes library
        try:
            from feagi_bytes import ByteStructureDecoder
            decoder = ByteStructureDecoder()
            decoded = decoder.decode_neuron_flat(data)

            print(f"Received activity from {len(set(decoded['cortical_ids']))} cortical areas")
            print(f"Total neurons: {len(decoded['x_coords'])}")

        except ImportError:
            print(f"Received {len(data)} bytes of neural data (feagi_bytes not available)")

# Usage example
if __name__ == "__main__":
    client = FeagiVisualizationClient(client_id="tutorial_client")
    try:
        client.start()
    except KeyboardInterrupt:
        client.stop()
```

### Client Lifecycle Management

The enhanced visualization system automatically manages client connections:

1. **Registration**: Send initial heartbeat via REST API
2. **Automatic FQ Enablement**: FEAGI enables neural data sampling when first client connects
3. **Heartbeat Maintenance**: Send heartbeat every 5-15 seconds (30-second timeout)
4. **Automatic Cleanup**: Inactive clients are removed after timeout
5. **Resource Conservation**: FEAGI disables sampling when no clients are connected

## Setting Up Visualizations

### Enabling Visualization

Ensure visualization is enabled in your configuration:

```toml
[feagi.api.zmq.streams]
enabled = ["visualization", "motor", "sensory", "control", "rest"]

[feagi.api.zmq.streams.visualization]
port = 5562
auto_enable_on_subscribers = true
subscriber_check_interval = 1.0
client_heartbeat_timeout = 30
```

### Custom Visualization Settings

You can customize visualization appearance and behavior:

```toml
[visualization]
colors.active_neuron = "#FF5733"
colors.inactive_neuron = "#3366FF"
colors.background = "#1A1A1A"
refresh_rate_ms = 100
show_connections = true

# Cortical area specific sampling rates
[cortical_areas.visual_cortex.properties]
fq_sample_rate = 30.0  # High rate for visual analysis

[cortical_areas.memory_area.properties]
fq_sample_rate = 5.0   # Lower rate for memory areas
```

## Advanced Features

### Real-Time Performance Monitoring

Monitor your visualization client performance:

```python
import time

class MonitoredVisualizationClient(FeagiVisualizationClient):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stats = {
            'messages_received': 0,
            'bytes_received': 0,
            'start_time': time.time()
        }

    def _handle_neural_activity(self, data):
        # Update statistics
        self.stats['messages_received'] += 1
        self.stats['bytes_received'] += len(data)

        # Print stats every 100 messages
        if self.stats['messages_received'] % 100 == 0:
            runtime = time.time() - self.stats['start_time']
            rate = self.stats['messages_received'] / runtime
            print(f"Received {self.stats['messages_received']} messages "
                  f"({rate:.1f} msg/s, {self.stats['bytes_received']} bytes)")

        # Process the data
        super()._handle_neural_activity(data)
```

### Standby Mode Detection

The visualization system automatically detects when FEAGI's genome is not loaded:

```python
def check_feagi_status():
    """Check if FEAGI is ready for visualization."""
    try:
        response = requests.get("http://localhost:8000/v1/system/health_check")
        health = response.json()

        if health.get('genome_availability') and health.get('brain_readiness'):
            print("✅ FEAGI ready for visualization")
            return True
        else:
            print("⏸️ FEAGI in standby mode (genome not loaded)")
            return False
    except Exception as e:
        print(f"❌ Cannot connect to FEAGI: {e}")
        return False
```

## Recording and Playback

### Recording Activity

To record neural activity for later analysis:

1. Click the "Record" button in the visualization panel
2. Specify a file name and duration
3. Run your simulation
4. Click "Stop Recording" when finished

### Playback

To review recorded activity:

1. Go to the "Recordings" tab
2. Select the recording file
3. Use the playback controls to review the activity
4. Adjust playback speed as needed

## Data Export

### Exporting Visualization Data

You can export visualization data for external analysis:

1. Click the "Export" button in the visualization panel
2. Choose the data format (CSV, JSON, or HDF5)
3. Select the time range and data resolution
4. Click "Export" to save the file

### Supported Export Formats

- **CSV**: Simple tabular data for spreadsheet analysis
- **JSON**: Hierarchical data for web applications
- **HDF5**: High-performance scientific data format

### Programmatic Data Export

Access visualization data programmatically:

```python
# Save neural activity data to file
class DataLogger:
    def __init__(self, filename):
        self.filename = filename
        self.data_log = []

    def log_activity(self, decoded_data):
        """Log neural activity with timestamp."""
        timestamp = time.time()
        self.data_log.append({
            'timestamp': timestamp,
            'cortical_areas': list(set(decoded_data['cortical_ids'])),
            'neuron_count': len(decoded_data['x_coords']),
            'activity_level': sum(decoded_data['membrane_potentials']) / len(decoded_data['membrane_potentials'])
        })

    def save_to_json(self):
        """Save logged data to JSON file."""
        import json
        with open(self.filename, 'w') as f:
            json.dump(self.data_log, f, indent=2)
        print(f"Saved {len(self.data_log)} activity records to {self.filename}")
```

## Troubleshooting

### Common Issues

**No Data Received:**
- Verify FEAGI is running and genome is loaded
- Check that heartbeats are being sent successfully
- Ensure firewall allows connections to port 5562

**Connection Timeouts:**
- Ensure heartbeat interval is less than 30 seconds
- Check network connectivity between client and FEAGI
- Verify REST API is accessible on port 8000

**Performance Issues:**
- Monitor data rate and adjust `fq_sample_rate` for cortical areas
- Use appropriate buffer sizes in your client
- Consider filtering data for specific cortical areas only

### Debug Mode

Enable debug logging for detailed troubleshooting:

```python
import logging

# Enable debug logging for FEAGI components
logging.getLogger('feagi.api.zmq.streams.visualization').setLevel(logging.DEBUG)
logging.getLogger('feagi.api.zmq.rest_adapter').setLevel(logging.DEBUG)

# Your visualization client code here...
```

## Advanced Visualization

### Custom Views

Create custom visualization dashboards:

1. Click "Custom View" in the visualization panel
2. Select the visualization components to include
3. Arrange the components in the desired layout
4. Save your custom view for future use

### Multi-Client Coordination

Multiple visualization clients can connect simultaneously:

```python
# Example: Multiple specialized clients
brain_monitor = FeagiVisualizationClient(client_id="brain_monitor")
activity_logger = FeagiVisualizationClient(client_id="activity_logger")
performance_analyzer = FeagiVisualizationClient(client_id="performance_analyzer")

# Each client gets the same data but can process it differently
```

### Integration with Analysis Tools

Integrate with popular analysis frameworks:

```python
# Example: Integration with matplotlib for real-time plotting
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

class RealTimePlotter(FeagiVisualizationClient):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.activity_history = deque(maxlen=100)

        # Set up real-time plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])

    def _handle_neural_activity(self, data):
        # Calculate average activity level
        try:
            from feagi_bytes import ByteStructureDecoder
            decoder = ByteStructureDecoder()
            decoded = decoder.decode_neuron_flat(data)

            avg_activity = np.mean(decoded['membrane_potentials'])
            self.activity_history.append(avg_activity)

            # Update plot
            self.line.set_data(range(len(self.activity_history)),
                             list(self.activity_history))
            self.ax.relim()
            self.ax.autoscale_view()
            plt.pause(0.01)

        except ImportError:
            pass  # Skip plotting if feagi_bytes not available
```

## Performance Optimization

### Client-Side Optimization

1. **Use Efficient Data Structures**: Choose appropriate data structures for your analysis
2. **Implement Backpressure Handling**: Drop data if processing can't keep up
3. **Monitor Memory Usage**: Avoid memory leaks in long-running clients
4. **Profile Your Code**: Use Python profiling tools to identify bottlenecks

### Server-Side Configuration

1. **Adjust Sampling Rates**: Configure per-area sampling rates based on needs
2. **Monitor Client Count**: Use `GET /v1/system/zmq_status` to monitor active clients
3. **Resource Monitoring**: Monitor FEAGI system resources during visualization

## Next Steps

- Learn about [Agent Connections](/user-guide/agents) to integrate external systems
- Explore [Tutorial Projects](/user-guide/tutorials) for practical examples
- Review [ZMQ Streams Documentation](../../feagi/api/zmq/streams/README.md) for technical details
