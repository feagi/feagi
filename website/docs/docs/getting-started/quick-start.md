---
sidebar_position: 2
---

# Quick Start Guide

This guide will help you get up and running with FEAGI quickly. We'll create a simple neural network and run a basic simulation.

## Starting FEAGI

After [installing FEAGI](installation), you can start it using:

```bash
python run_feagi.py
```

This will start the FEAGI server with default settings.

## Creating Your First Brain

### 1. Access the Web Interface

Open your browser and navigate to [http://localhost:8000](http://localhost:8000).

### 2. Create a New Genome

1. Click on "Create New Genome" in the dashboard
2. Enter a name for your genome (e.g., "MyFirstBrain")
3. Select "Empty Template" as the starting point
4. Click "Create"

### 3. Add Cortical Areas

To add cortical areas to your brain:

1. Navigate to the "Cortical Areas" tab
2. Click "Add Cortical Area"
3. Configure the following properties:
   - Name: "Visual_Input"
   - Type: "Sensory"
   - Dimensions: 10x10x1
   - Position: X=0, Y=0, Z=0
4. Click "Save"
5. Repeat to create a second area:
   - Name: "Processing"
   - Type: "Association"
   - Dimensions: 10x10x1
   - Position: X=20, Y=0, Z=0

### 4. Connect Cortical Areas

To create connections between cortical areas:

1. Select "Visual_Input" in the cortical areas list
2. Click "Add Connection"
3. Select "Processing" as the target
4. Configure connection properties:
   - Type: "One to One"
   - Efficiency: 1.0
5. Click "Save"

## Running a Simulation

### 1. Start the Simulation

1. Click on the "Simulation" tab
2. Click "Start Simulation"

### 2. Send Input

1. Click on the "Visual_Input" cortical area
2. Use the pattern tool to draw a pattern
3. Click "Send" to stimulate the input area

### 3. Observe Results

Watch the activity propagate from the input area to the processing area.

## Next Steps

Now that you've created your first brain in FEAGI:

- Explore the [User Guide](/user-guide/configuration) to learn about advanced configuration options
- Check out the [Visualization Guide](/user-guide/visualization) to understand how to visualize neural activity
- Learn about [Agents](/user-guide/agents) to connect external systems to your FEAGI brain

For more detailed information, refer to our [System Documentation](/system/arch-system-overview).
