---
sidebar_position: 2
---

# Visualization Guide

FEAGI provides powerful visualization tools to help you understand and monitor the activity in your neural networks. This guide covers the visualization features and how to use them effectively.

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

## Setting Up Visualizations

### Enabling Visualization

Ensure visualization is enabled in your configuration:

```yaml
feagi:
  runtime:
    visualization: true
    visualization_level: "detailed"  # Options: basic, detailed, performance
```

### Custom Visualization Settings

You can customize visualization appearance:

```yaml
visualization:
  colors:
    active_neuron: "#FF5733"
    inactive_neuron: "#3366FF"
    background: "#1A1A1A"
  refresh_rate_ms: 100
  show_connections: true
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

## Advanced Visualization

### Custom Views

Create custom visualization dashboards:

1. Click "Custom View" in the visualization panel
2. Select the visualization components to include
3. Arrange the components in the desired layout
4. Save your custom view for future use

### Programmatic Visualization Access

Access visualization data programmatically through the API:

```python
import requests

# Get neural activity data
response = requests.get('http://localhost:8000/api/v1/visualization/activity')
activity_data = response.json()

# Process and visualize with your own tools
import matplotlib.pyplot as plt
plt.imshow(activity_data['heatmap'])
plt.colorbar()
plt.show()
```

## Next Steps

- Learn about [Agent Connections](/user-guide/agents) to integrate external systems
- Explore [Tutorial Projects](/user-guide/tutorials) for practical examples 