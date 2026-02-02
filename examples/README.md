# HAPTOS Examples

This directory contains 6 example applications demonstrating the HAPTOS SDK, from simple one-liners to a full-featured interactive GUI simulator.

## Getting Started

### 1. Hello HAPTOS (Simplest)

The most minimal example possible - just runs the built-in demo.

```bash
python examples/hello_haptos.py
```

**What it demonstrates:**
- Quickstart with zero configuration
- Built-in demo functionality

---

### 2. Basic Simulation

Complete minimal example showing all three layers.

```bash
python examples/basic_simulation.py
```

**What it demonstrates:**
- Creating a `Simulation` (Layer 1)
- Creating a `Renderer` (Layer 2)
- Creating a `Driver` (Layer 3)
- Running a simulation loop
- Reading statistics

**Output:**
```
Basic HAPTOS Simulation
=======================

✓ Simulation initialized: simple_hand.xml
✓ Renderer initialized
✓ Driver initialized

Running simulation (1000 steps = 1 second)...
  Step 0: 0 contacts, 0 cues, 0 sent
  Step 100: 2 contacts, 2 cues, 2 sent
  ...
```

---

### 3. Grasp Demo (Multi-Contact)

Demonstrates full hand grasping with 6 simultaneous channels.

```bash
python examples/grasp_demo.py
```

**What it demonstrates:**
- Multi-contact simulation (6 channels)
- Channel synchronization (`enable_sync=True`)
- Bandwidth monitoring
- Per-channel statistics

**Use cases:**
- Haptic gloves
- Multi-actuator systems
- Complex manipulation tasks

---

### 4. Custom Homunculus

Shows how to create and use custom perceptual models.

```bash
python examples/custom_homunculus.py
```

**What it demonstrates:**
- Inspecting default Homunculus properties
- Saving/loading custom configurations
- Using custom Homunculus in simulation

**Use cases:**
- User-specific calibration
- Non-human bodies (robotic manipulators)
- Accessibility features (enhanced sensitivity)

**Output files:**
- `custom_homunculus.json` - Saved configuration

---

### 5. Hardware Integration

Demonstrates real hardware communication (Teensy 4.1).

```bash
# Mock hardware (no physical device needed)
python examples/hardware_integration.py

# Real hardware (requires Teensy + firmware)
python examples/hardware_integration.py --real
```

**What it demonstrates:**
- Switching between mock and real hardware
- Serial port configuration
- Packet transmission statistics
- Latency monitoring

**Requirements for `--real` mode:**
- Teensy 4.1 with `firmware/HaptosReceiver.ino` flashed
- USB connection
- Voice coil actuator (optional, for testing output)

---

### 6. Interactive Simulator ✨

Full-featured PyQt5 GUI for interactive testing and debugging.

```bash
# Default model (simple hand)
python examples/interactive_simulator.py

# Custom model
python examples/interactive_simulator.py path/to/model.xml
```

**What it demonstrates:**
- Real-time physics and haptics visualization
- Contact force vectors and body part IDs
- Live plotting of all 5 haptic cue types
- Interactive simulation control (play/pause/step/reset)
- Performance monitoring (latency, bandwidth, contacts)
- 6-channel hand simulation with visual feedback

**Key Features:**
- **MuJoCo Viewer**: Real-time physics rendering
- **Contact Visualization**: Force vectors overlaid on model
- **Cue Display**: Real-time CueParams table (all 5 cue types)
- **Live Plotting**: Matplotlib graphs with 100-frame history
- **Control Panel**: Play/Pause/Step/Reset, speed control (0.1x-5.0x)
- **Statistics Dashboard**: Latency, contacts, bandwidth tracking
- **Visualization Toggles**: Show/hide contacts, IDs, cues

**GUI Layout:**
```
┌─────────────────────────────────────────────────────┐
│ Control Panel: [Play] [Pause] [Step] [Reset]       │
└─────────────────────────────────────────────────────┘
┌──────────────────────┬──────────────────────────────┐
│ MuJoCo Viewer        │ Cue Parameters + Statistics  │
│ (with force overlays)│                              │
└──────────────────────┴──────────────────────────────┘
┌─────────────────────────────────────────────────────┐
│ Live Cue Plots (5 matplotlib subplots)              │
└─────────────────────────────────────────────────────┘
```

**Use Cases:**
- Physics debugging (visualize contact forces)
- Haptic cue validation (inspect renderer output)
- Performance profiling (track latency/bandwidth)
- Demo and presentation (visual showcase)
- Development workflow (test new models/algorithms)

**Requirements:**
```bash
pip install PyQt5 matplotlib
```

**Performance:**
- 60 FPS GUI rendering
- 1kHz physics simulation
- 100Hz haptic rendering
- Handles 6+ simultaneous contacts

---

## Example Structure

All examples follow this pattern:

```python
import haptos

# 1. Create simulation
sim = haptos.Simulation("model.xml")

# 2. Create renderer
renderer = haptos.Renderer()

# 3. Create driver
driver = haptos.Driver(driver_type="mock")

# 4. Register channels
driver.register(body_part_id=10, port="MOCK")

# 5. Simulation loop
for step in range(1000):
    contacts = sim.step_filtered()

    if step % 10 == 0:  # 100Hz rendering
        cues = renderer.render(contacts)
        driver.send(cues)

# 6. Cleanup
driver.disconnect_all()
```

---

## Example Comparison

| Example | Lines | Complexity | GUI | Multi-Contact | Hardware |
|---------|-------|------------|-----|---------------|----------|
| hello_haptos.py | 1 | ⭐ | - | - | - |
| basic_simulation.py | 30 | ⭐⭐ | - | - | - |
| grasp_demo.py | 60 | ⭐⭐⭐ | - | ✓ | - |
| custom_homunculus.py | 80 | ⭐⭐⭐ | - | - | - |
| hardware_integration.py | 100 | ⭐⭐⭐⭐ | - | - | ✓ |
| interactive_simulator.py | 560 | ⭐⭐⭐⭐⭐ | ✓ | ✓ | - |

---

## Tips

### Running Without Hand Models

If you don't have MuJoCo hand models, the examples will fall back to mock data or skip physics simulation.

To get models:
```bash
# Download from HAPTOS repository
git clone https://github.com/anthropics/haptos-models assets/
```

### Modifying Examples

All examples are designed to be modified. Try:

- Changing `duration` to run longer/shorter
- Adding more channels in grasp_demo.py
- Adjusting `max_contacts` to test prioritization
- Enabling/disabling `enable_sync` to compare performance

### Debugging

Enable verbose output by setting environment variable:
```bash
export HAPTOS_VERBOSE=1
python examples/basic_simulation.py
```

---

## Architecture Reminder

```
┌─────────────────────────────────────────┐
│ Layer 1: Simulation @ 1kHz              │
│  Physics → ContactPatch → FilteredContact│
└─────────────────────────────────────────┘
                 ↓
┌─────────────────────────────────────────┐
│ Layer 2: Renderer @ 100Hz               │
│  FilteredContact → CueParams            │
└─────────────────────────────────────────┘
                 ↓
┌─────────────────────────────────────────┐
│ Layer 3: Driver @ 2kHz+                 │
│  CueParams → Actuator                   │
└─────────────────────────────────────────┘
```

Each example demonstrates different aspects of this three-layer architecture.

---

## Next Steps

After running the examples:

1. Read the [Quickstart Guide](../docs/quickstart.md)
2. Explore the [API Reference](../docs/api_reference.md)
3. Try [Tutorials](../docs/tutorials/)
4. Build your own application!

---

**Questions?** See [GitHub Issues](https://github.com/anthropics/haptos/issues)
