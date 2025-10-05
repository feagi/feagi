"""Shared memory capability types recognized by FEAGI.

Defines the canonical set of capability identifiers that FEAGI and agents
must use when negotiating shared-memory based data streams.

Capabilities (all values are lower-case strings):

- video: Raw RGB video stream intended for Brain Visualizer video preview.
         This path is provided to the agent so it can WRITE frames. The file
         is agent-owned. The stream may optionally contain multiple embedded
         feeds (e.g., original and segmented preview).

- sensory: Sensory stream from agent to FEAGI. This path is provided to the
           agent so it can WRITE neuron bytes destined for FEAGI.

- motor: Motor stream from FEAGI to agent. This path is created and written by
         FEAGI so agents can READ motor bytes.

- visualization: FEAGI neuron activity stream intended for visualization
                 clients like the Brain Visualizer (BV). FEAGI owns and writes
                 this core stream for BV to READ.
"""

from __future__ import annotations

from enum import Enum


class SharedMemoryCapability(str, Enum):
    """Canonical shared memory capability identifiers.

    Use these constants as the single source of truth across FEAGI and agents.
    """

    VIDEO = "video"
    SENSORY = "sensory"
    MOTOR = "motor"
    VISUALIZATION = "visualization"


CAPABILITY_DESCRIPTIONS = {
    SharedMemoryCapability.VIDEO.value: (
        "Raw RGB video stream for Brain Visualizer preview (agent writes)."
    ),
    SharedMemoryCapability.SENSORY.value: (
        "Sensory neuron bytes from agent to FEAGI (agent writes)."
    ),
    SharedMemoryCapability.MOTOR.value: (
        "Motor neuron bytes from FEAGI to agent (FEAGI writes)."
    ),
    SharedMemoryCapability.VISUALIZATION.value: (
        "Neuron activity data for visualization clients (FEAGI writes)."
    ),
}


