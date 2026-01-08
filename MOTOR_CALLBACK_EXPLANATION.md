# Motor Callback System - How It Works

## Overview

The FEAGI motor callback system uses an **event-driven architecture** where Python callbacks are automatically invoked when FEAGI sends motor commands. This eliminates the need for polling and provides real-time, low-latency motor control.

## Data Flow

```
FEAGI Brain → ZMQ Transport → Rust Decoder → Python Callback → Motor Control
```

### Step-by-Step Flow

1. **FEAGI sends neuron data** via ZMQ (port 5564) as binary bytes
2. **`brain_output.receive()`** receives the bytes from ZMQ
3. **Rust `MotorDeviceCache.process_neurons()`** deserializes and decodes the neuron data
4. **Rust decoder** maps neuron X-coordinates to motor channels and calculates motor values
5. **Rust callback system** invokes the registered Python callback for each updated channel
6. **Python callback** (`_on_motor_command`) receives the decoded value and updates motor state
7. **Controller** reads the updated motor state via `motor.get_angle()` or `motor.get_speed()`

## Registration Process

### Phase 1: Motor Registration (Before Connect)

When you call `ServoMotor.register()`, the motor is added to the registry but the callback is **deferred**:

```python
# Motor registers itself with brain_output
servo = ServoMotor.register(range=(0, 180))

# At this point:
# - Motor is added to _outputs list
# - Channel index is assigned (0, 1, 2, ...)
# - Callback registration is DEFERRED (decoder_registered=False)
```

### Phase 2: Decoder Registration (During Connect)

When `brain_output.connect()` is called, it:

1. **Registers the decoder** with the total number of channels:
   ```python
   cache.register(
       motor_unit=PositionalServo,
       group=0,
       channels=21,  # Total number of motors
       z_resolution=100,
       frame_change_handling=0,  # Absolute
       percentage_positioning=0  # Linear
   )
   ```

2. **Registers all motor callbacks**:
   ```python
   for motor in motors:
       cache.register_callback(
           motor_unit=PositionalServo,
           group=0,
           channel=motor.channel,  # 0, 1, 2, ...
           callback=motor._on_motor_command
       )
   ```

## Callback Signature

The callback function receives a **single float value** representing the decoded motor command:

```python
def _on_motor_command(self, value: float):
    """
    Callback invoked when FEAGI sends a motor command.
    
    Args:
        value: Decoded motor value
            - For ServoMotor: SignedPercentage (-1.0 to 1.0)
            - For RotaryMotor: SignedPercentage (-1.0 to 1.0)
    """
```

### Value Interpretation

- **ServoMotor**: `value` is a **SignedPercentage** (-1.0 to 1.0)
  - `-1.0` → maps to `min_angle`
  - `0.0` → maps to center angle
  - `1.0` → maps to `max_angle`

- **RotaryMotor**: `value` is a **SignedPercentage** (-1.0 to 1.0)
  - `-1.0` → full reverse
  - `0.0` → stopped
  - `1.0` → full forward

## Simple Example

```python
from feagi.pns.outputs import ServoMotor
from feagi.pns import brain_output

# Step 1: Register motors (callbacks deferred)
shoulder = ServoMotor.register(range=(-90, 90))  # Channel 0
elbow = ServoMotor.register(range=(0, 180))      # Channel 1
wrist = ServoMotor.register(range=(-45, 45))     # Channel 2

# Step 2: Configure and connect (triggers decoder + callback registration)
brain_output.configure(
    agent_id="my_robot",
    feagi_host="127.0.0.1",
    feagi_port=5564,
    feagi_api_port=8000,
    transport="zmq"
)
brain_output.connect()  # ← Decoder and callbacks registered here!

# Step 3: Main loop - just call receive() and read motor values
while True:
    # This receives ZMQ data, decodes neurons, and triggers callbacks
    brain_output.receive()
    
    # Callbacks have already updated motor state, just read it:
    shoulder_angle = shoulder.get_angle()  # Already updated by callback!
    elbow_angle = elbow.get_angle()        # Already updated by callback!
    wrist_angle = wrist.get_angle()        # Already updated by callback!
    
    # Apply to your robot
    robot.set_shoulder(shoulder_angle)
    robot.set_elbow(elbow_angle)
    robot.set_wrist(wrist_angle)
```

## What Happens Inside `receive()`

```python
def receive(self):
    # 1. Receive ZMQ message (non-blocking)
    parts = self._transport.recv_multipart(zmq.NOBLOCK)
    motor_bytes = parts[1]  # Extract motor data
    
    # 2. Process through Rust decoder (this triggers callbacks!)
    self._cache.process_neurons(list(motor_bytes))
    # ↑ Inside this call:
    #   - Neurons are decoded to motor values
    #   - For each updated channel, the Python callback is invoked
    #   - motor._on_motor_command(value) is called
    #   - motor._current_angle is updated
    
    # 3. Done! Motor values are already updated via callbacks
```

## Channel Mapping

Each motor is assigned a **channel index** (0, 1, 2, ...) within the same cortical area:

- **Cortical Area**: `b3BzZQQAAAA=` (PositionalServo, SignedPercentage)
- **Channel 0**: X-coordinates 0,1 → Motor 0
- **Channel 1**: X-coordinates 2,3 → Motor 1
- **Channel 2**: X-coordinates 4,5 → Motor 2
- etc.

FEAGI sends neuron data with X-coordinates, and the Rust decoder maps them to channels:

```rust
// X=0,1 → channel 0 (positive/negative pair)
// X=2,3 → channel 1 (positive/negative pair)
// X=4,5 → channel 2 (positive/negative pair)
let channel_index = neuron.x / 2;
```

## Key Points

1. **No Polling**: Callbacks are event-driven - they fire automatically when data arrives
2. **Automatic Updates**: Motor state (`_current_angle`, `_current_speed`) is updated inside the callback
3. **Thread-Safe**: Rust handles the callback invocation safely across threads
4. **Low Latency**: Direct callback invocation without Python polling overhead
5. **Channel-Based**: Multiple motors share the same cortical area, differentiated by channel index

## Advanced: Custom Callbacks

You can also register custom callbacks directly (though the built-in motor classes handle this automatically):

```python
import feagi_rust_py_libs as frpl

def my_custom_callback(value: float):
    print(f"Motor received: {value}")
    # Do something custom with the value

cache.register_callback(
    motor_unit=frpl.data_structures.genomic.MotorCorticalType.PositionalServo,
    group=0,
    channel=5,
    callback=my_custom_callback
)
```

## Troubleshooting

- **No callbacks firing?** Make sure `brain_output.connect()` was called after registering motors
- **Wrong values?** Check that the cortical area in FEAGI matches the motor type (PositionalServo vs RotaryMotor)
- **Channels not updating?** Verify neuron X-coordinates in FEAGI match the channel mapping (X=0,1 → channel 0, etc.)

