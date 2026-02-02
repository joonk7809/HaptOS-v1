# HAPTOS API Reference

Complete API documentation for HAPTOS Platform v0.3.0

---

## Module: `haptos`

Top-level package providing the public API.

### Quick Import

```python
import haptos

# Main classes
sim = haptos.Simulation("model.xml")
renderer = haptos.Renderer()
driver = haptos.Driver()
homunculus = haptos.Homunculus()

# Convenience functions
haptos.demo()
haptos.calibrate_user()
```

---

## Class: `haptos.Simulation`

Physics simulation wrapper (Layer 1).

### Constructor

```python
Simulation(
    model_path: str,
    max_contacts: int = 20,
    homunculus: Optional[Homunculus] = None,
    timestep: float = 0.001
)
```

**Parameters**:
- `model_path` (str): Path to MuJoCo XML model file
- `max_contacts` (int): Maximum simultaneous contacts to track (default: 20)
- `homunculus` (Homunculus, optional): Custom perceptual model (default: standard)
- `timestep` (float): Physics timestep in seconds (default: 0.001 = 1ms)

**Raises**:
- `FileNotFoundError`: If model_path does not exist

**Example**:
```python
sim = haptos.Simulation(
    "assets/hand_models/simple_hand.xml",
    max_contacts=20
)
```

---

### Methods

#### `step() → List[ContactPatch]`

Advance simulation by one timestep and return raw contacts.

**Returns**:
- `List[ContactPatch]`: Raw contacts from physics engine

**Example**:
```python
contacts = sim.step()
print(f"Detected {len(contacts)} contacts")
```

---

#### `step_filtered() → List[FilteredContact]`

Advance simulation and return perceptually-filtered contacts.

Applies biological filtering:
- Sensitivity thresholds (weak forces filtered out)
- Rendering tier assignment (VCA/LRA/ERM)
- Cue mask assignment (which cues to render)

**Returns**:
- `List[FilteredContact]`: Routed contacts through Homunculus

**Example**:
```python
filtered = sim.step_filtered()
for contact in filtered:
    print(f"Body part {contact.patch.body_part_id}: tier {contact.rendering_tier}")
```

---

#### `reset()`

Reset simulation to initial state.

Clears all contact history and resets time to 0.

**Example**:
```python
sim.reset()
```

---

#### `get_state() → dict`

Get current simulation state.

**Returns**:
- `dict`: Dictionary with keys:
  - `time` (float): Current simulation time (seconds)
  - `step_count` (int): Number of steps taken
  - `active_contacts` (int): Number of current contacts
  - `router_stats` (dict): Statistics from somatotopic router

**Example**:
```python
state = sim.get_state()
print(f"Time: {state['time']:.3f}s")
print(f"Contacts: {state['active_contacts']}")
```

---

#### `set_qpos(qpos: ndarray)`

Set joint positions directly.

**Parameters**:
- `qpos` (ndarray): Array of joint positions (size depends on model)

**Example**:
```python
import numpy as np
qpos = np.zeros(sim.engine.model.nq)
sim.set_qpos(qpos)
```

---

#### `set_qvel(qvel: ndarray)`

Set joint velocities directly.

**Parameters**:
- `qvel` (ndarray): Array of joint velocities

---

#### `get_qpos() → ndarray`

Get current joint positions.

**Returns**:
- `ndarray`: Copy of joint positions

---

#### `get_qvel() → ndarray`

Get current joint velocities.

**Returns**:
- `ndarray`: Copy of joint velocities

---

## Class: `haptos.Renderer`

Neural renderer for haptic cue generation (Layer 2).

### Constructor

```python
Renderer(
    model_v0: Optional[str] = None,
    model_v1: Optional[str] = None,
    device: str = 'cpu',
    batch_size: int = 1
)
```

**Parameters**:
- `model_v0` (str, optional): Path to baseline model checkpoint (default: auto-detect)
- `model_v1` (str, optional): Path to delta model checkpoint (default: auto-detect)
- `device` (str): Device for inference ('cpu' or 'cuda', default: 'cpu')
- `batch_size` (int): Batch size for inference (default: 1)

**Raises**:
- `FileNotFoundError`: If model files not found

**Example**:
```python
# Auto-load default models
renderer = haptos.Renderer()

# Use GPU
renderer = haptos.Renderer(device='cuda')

# Custom models
renderer = haptos.Renderer(
    model_v0="models/custom/nn_v0.pt",
    model_v1="models/custom/nn_v1.pt"
)
```

---

### Methods

#### `render(filtered_contacts: List[FilteredContact], timestamp_us: Optional[int] = None) → Dict[int, CueParams]`

Render haptic cue parameters from filtered contacts.

**Parameters**:
- `filtered_contacts` (List[FilteredContact]): Contacts from router
- `timestamp_us` (int, optional): Current time in microseconds

**Returns**:
- `Dict[int, CueParams]`: Mapping body_part_id → CueParams

**Example**:
```python
filtered = sim.step_filtered()
cues = renderer.render(filtered)

for body_part_id, cue_params in cues.items():
    print(f"Body part {body_part_id}:")
    print(f"  Texture: {cue_params.texture_grain_hz:.1f}Hz")
    print(f"  Weight: {cue_params.weight_offset:.2f}")
```

---

#### `reset()`

Reset renderer state.

Clears all contact buffers and FSM state.

**Example**:
```python
renderer.reset()
```

---

#### `get_stats() → dict`

Get rendering statistics.

**Returns**:
- `dict`: Dictionary with keys:
  - `total_inferences` (int): Number of inference calls
  - `avg_inference_time_ms` (float): Average inference time
  - `contacts_rendered` (int): Total contacts processed

**Example**:
```python
stats = renderer.get_stats()
print(f"Inferences: {stats['total_inferences']}")
print(f"Avg time: {stats['avg_inference_time_ms']:.2f}ms")
```

---

#### `load_model(model_path: str, version: str = "v0")`

Load a different model checkpoint.

**Parameters**:
- `model_path` (str): Path to .pt checkpoint file
- `version` (str): Model version ("v0" for baseline, "v1" for delta)

**Raises**:
- `FileNotFoundError`: If model file not found
- `ValueError`: If version is not "v0" or "v1"

**Example**:
```python
renderer.load_model("models/zoo/nn_v0_fast.pt", version="v0")
```

---

## Class: `haptos.Driver`

Hardware driver wrapper (Layer 3).

### Constructor

```python
Driver(
    driver_type: str = "mock",
    enable_sync: bool = False
)
```

**Parameters**:
- `driver_type` (str): Type of driver ("mock" or "serial")
- `enable_sync` (bool): Enable channel synchronization (default: False)

**Raises**:
- `ValueError`: If driver_type is not "mock" or "serial"

**Example**:
```python
# Mock hardware (simulation)
driver = haptos.Driver(driver_type="mock")

# Real hardware with synchronization
driver = haptos.Driver(
    driver_type="serial",
    enable_sync=True
)
```

---

### Methods

#### `register(body_part_id: int, port: str = "MOCK", config: Optional[dict] = None)`

Register a hardware channel for a body part.

**Parameters**:
- `body_part_id` (int): Body part identifier (e.g., 10 for index fingertip)
- `port` (str): Port identifier:
  - `"MOCK"` for simulated hardware
  - `"/dev/ttyACM0"` for real serial port (Linux/Mac)
  - `"COM3"` for real serial port (Windows)
- `config` (dict, optional): Driver configuration:
  - `baudrate` (int): Serial baud rate (default: 115200)
  - `packet_loss_rate` (float): Simulated packet loss (mock only, 0-1)
  - `simulate_latency` (bool): Enable latency simulation (mock only)

**Example**:
```python
# Mock hardware
driver.register(10, "MOCK")

# Real hardware
driver.register(
    body_part_id=10,
    port="/dev/ttyACM0",
    config={'baudrate': 115200}
)
```

---

#### `send(cue_params_dict: Dict[int, CueParams]) → Dict[int, bool]`

Send cue parameters to all registered channels.

**Parameters**:
- `cue_params_dict` (Dict[int, CueParams]): Mapping body_part_id → CueParams

**Returns**:
- `Dict[int, bool]`: Mapping body_part_id → success (True/False)

**Example**:
```python
cues = renderer.render(contacts)
results = driver.send(cues)

for body_part_id, success in results.items():
    if not success:
        print(f"Warning: Failed to send to body part {body_part_id}")
```

---

#### `disconnect_all()`

Disconnect all hardware channels.

Should be called before exiting to ensure clean shutdown.

**Example**:
```python
driver.disconnect_all()
```

---

#### `reset_stats()`

Reset statistics for all channels.

**Example**:
```python
driver.reset_stats()
```

---

#### `get_stats() → dict`

Get aggregated statistics from all channels.

**Returns**:
- `dict`: Dictionary with keys:
  - `driver_count` (int): Number of registered channels
  - `total_sent` (int): Total packets sent
  - `total_dropped` (int): Total packets dropped
  - `success_rate` (float): Transmission success rate (0-1)
  - `avg_latency_ms` (float): Average transmission latency
  - `bandwidth_hz` (float): Effective bandwidth (packets/second)
  - `bandwidth_kbps` (float): Effective bandwidth (kilobits/second)
  - `per_channel_stats` (dict): Statistics per body part

**Example**:
```python
stats = driver.get_stats()
print(f"Channels: {stats['driver_count']}")
print(f"Success rate: {stats['success_rate']:.1%}")
print(f"Bandwidth: {stats['bandwidth_hz']:.1f} packets/s")
```

---

#### `get_bandwidth_stats() → dict`

Get bandwidth utilization statistics.

**Returns**:
- `dict`: Dictionary with keys:
  - `total_packets` (int): Total packets sent
  - `elapsed_time_s` (float): Time since initialization
  - `bandwidth_hz` (float): Packets per second
  - `bandwidth_kbps` (float): Kilobits per second (52 bytes/packet)
  - `channels_active` (int): Number of active channels

**Example**:
```python
bandwidth = driver.get_bandwidth_stats()
print(f"Bandwidth: {bandwidth['bandwidth_kbps']:.1f} kbps")
```

---

### Context Manager Support

Driver supports context manager protocol for automatic cleanup:

```python
with haptos.Driver() as driver:
    driver.register(10, "MOCK")
    # ... simulation loop
# driver.disconnect_all() called automatically
```

---

## Class: `haptos.Homunculus`

Perceptual body model representing biological touch sensitivity.

### Constructor

```python
Homunculus(config_path: Optional[str] = None)
```

**Parameters**:
- `config_path` (str, optional): Path to JSON config file (default: use standard human model)

**Raises**:
- `FileNotFoundError`: If config_path does not exist

**Example**:
```python
# Default human model
homunculus = haptos.Homunculus()

# Custom configuration
homunculus = haptos.Homunculus("configs/user_profile.json")
```

---

### Methods

#### `lookup(body_part_id: int) → BodyPartProperties`

Get perceptual properties for a body part.

**Parameters**:
- `body_part_id` (int): Body part identifier

**Returns**:
- `BodyPartProperties`: Object with attributes:
  - `spatial_res_mm` (float): Spatial resolution (mm)
  - `freq_range_hz` (tuple): Frequency response range (Hz)
  - `sensitivity` (float): Relative sensitivity (0-2, nominal=1.0)
  - `rendering_tier` (int): Hardware tier (1=VCA, 2=LRA, 3=ERM)
  - `cue_mask` (int): Bitmask of perceptible cues

**Example**:
```python
props = homunculus.lookup(10)  # Index fingertip
print(f"Resolution: {props.spatial_res_mm}mm")
print(f"Sensitivity: {props.sensitivity:.2f}")
print(f"Tier: {props.rendering_tier}")
```

---

#### `save(filepath: str)`

Save Homunculus configuration to JSON file.

**Parameters**:
- `filepath` (str): Path to save config file

**Example**:
```python
homunculus.save("my_profile.json")
```

---

#### `load(filepath: str) → Homunculus` (static method)

Load Homunculus configuration from JSON file.

**Parameters**:
- `filepath` (str): Path to JSON config file

**Returns**:
- `Homunculus`: Instance with loaded configuration

**Raises**:
- `FileNotFoundError`: If filepath does not exist

**Example**:
```python
custom = haptos.Homunculus.load("my_profile.json")
sim = haptos.Simulation("model.xml", homunculus=custom)
```

---

#### `get_table() → dict`

Get complete Homunculus table.

**Returns**:
- `dict`: Dictionary mapping body part name → BodyPartProperties

**Example**:
```python
table = homunculus.get_table()
for name, props in table.items():
    print(f"{name}: {props.spatial_res_mm}mm resolution")
```

---

## Convenience Functions

### `haptos.demo(duration: float = 5.0, render_output: bool = True)`

Run a quick demo of the HAPTOS platform.

**Parameters**:
- `duration` (float): Duration to run demo in seconds (default: 5.0)
- `render_output` (bool): Whether to print output to console (default: True)

**Example**:
```python
import haptos
haptos.demo(duration=10.0)  # 10-second demo
```

---

### `haptos.calibrate_user(interactive: bool = True, save_path: Optional[str] = None) → Homunculus`

Calibrate a custom Homunculus for a user.

**Parameters**:
- `interactive` (bool): Run interactive calibration (default: True)
- `save_path` (str, optional): Path to save calibrated config

**Returns**:
- `Homunculus`: Calibrated Homunculus instance

**Example**:
```python
homunculus = haptos.calibrate_user(save_path="my_profile.json")
sim = haptos.Simulation("model.xml", homunculus=homunculus)
```

**Note**: Full interactive calibration coming in Phase 4. Currently returns default Homunculus.

---

## Data Structures

### `ContactPatch`

Raw physics contact from Layer 1.

**Attributes**:
- `body_part_id` (int): Body part identifier
- `force_normal` (float): Normal force (Newtons)
- `force_shear` (Tuple[float, float]): Shear forces (Fx, Fy)
- `velocity` (Tuple[float, float, float]): Contact velocity (vx, vy, vz)
- `contact_area` (float): Contact area (m²)
- `material_hint` (int): Material identifier (0 = unknown)
- `timestamp_us` (int): Timestamp (microseconds)

**Example**:
```python
for contact in sim.step():
    print(f"Force: {contact.force_normal:.3f}N")
    print(f"Body part: {contact.body_part_id}")
```

---

### `FilteredContact`

Routed contact from Somatotopic Router.

**Attributes**:
- `patch` (ContactPatch): Original contact patch
- `rendering_tier` (int): Hardware tier (1=VCA, 2=LRA, 3=ERM)
- `cue_mask` (int): Bitmask of enabled cues

**Cue Mask Constants**:
- `FilteredContact.CUE_IMPACT = 0b00001`
- `FilteredContact.CUE_RING = 0b00010`
- `FilteredContact.CUE_TEXTURE = 0b00100`
- `FilteredContact.CUE_SHEAR = 0b01000`
- `FilteredContact.CUE_WEIGHT = 0b10000`
- `FilteredContact.CUE_ALL = 0b11111`

**Example**:
```python
for contact in sim.step_filtered():
    print(f"Tier: {contact.rendering_tier}")
    if contact.cue_mask & FilteredContact.CUE_TEXTURE:
        print("  Texture enabled")
```

---

### `CueParams`

Haptic cue parameters from Layer 2.

**Attributes (Continuous Cues)**:
- `texture_grain_hz` (float): Texture frequency (50-500 Hz)
- `texture_amplitude` (float): Texture strength (0-1)
- `shear_direction` (Tuple[float, float]): Shear direction (x, y)
- `shear_magnitude` (float): Shear strength (0-1)
- `weight_offset` (float): Normal force (0-1)

**Attributes (Transient Cues)**:
- `impact_amplitude` (float): Impact strength (0-1)
- `impact_decay_ms` (float): Impact decay time (10-500 ms)
- `impact_frequency_hz` (float): Impact frequency (80-400 Hz)
- `ring_amplitude` (float): Ring strength (0-1)
- `ring_decay_ms` (float): Ring decay time (50-1000 ms)
- `trigger_impulse` (bool): Trigger flag for transients
- `timestamp_us` (int): Timestamp (microseconds)

**Example**:
```python
cues = renderer.render(contacts)
for body_part_id, cue in cues.items():
    print(f"Body part {body_part_id}:")
    print(f"  Texture: {cue.texture_grain_hz:.1f}Hz @ {cue.texture_amplitude:.2f}")
    print(f"  Weight: {cue.weight_offset:.2f}")
    if cue.trigger_impulse:
        print(f"  Impact: {cue.impact_amplitude:.2f}")
```

---

## Error Handling

### Common Exceptions

**FileNotFoundError**:
```python
try:
    sim = haptos.Simulation("nonexistent.xml")
except FileNotFoundError as e:
    print(f"Model not found: {e}")
```

**ValueError**:
```python
try:
    driver = haptos.Driver(driver_type="invalid")
except ValueError as e:
    print(f"Invalid driver type: {e}")
```

**RuntimeError** (from MuJoCo):
```python
try:
    sim.step()
except RuntimeError as e:
    print(f"Physics error: {e}")
```

---

## Version Information

```python
import haptos

print(haptos.__version__)  # "0.3.0"
print(haptos.__author__)   # "HAPTOS Team"
print(haptos.__license__)  # "MIT"
```

---

## See Also

- **[Quickstart Guide](quickstart.md)** - Get started in 5 minutes
- **[Tutorials](tutorials/)** - In-depth guides
- **[Examples](../examples/)** - Working code examples
- **[Architecture](ARCHITECTURE.md)** - System design details

---

**Version**: 0.3.0
**Last Updated**: February 1, 2026
