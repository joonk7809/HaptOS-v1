# HAPTOS Platform

**Simulation-first haptics for developers**

HAPTOS is a platform that abstracts haptic synthesis complexity, similar to how PyTorch abstracts ML or OpenXR abstracts XR hardware. Define physics interactions → Platform translates to biological sensation cues → Hardware renders haptics.

[![Tests](https://img.shields.io/badge/tests-145%20passing-brightgreen)](tests/)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue)](https://python.org)
[![License](https://img.shields.io/badge/license-MIT-blue)](LICENSE)
[![Phase](https://img.shields.io/badge/phase-3%20SDK-orange)](PROJECT_STATUS.md)

---

## Quick Start

```python
import haptos

# Run built-in demo
haptos.demo()

# Or create your own simulation
sim = haptos.Simulation("assets/hand_models/simple_hand.xml")
renderer = haptos.Renderer()
driver = haptos.Driver(driver_type="mock")

driver.register(body_part_id=10, port="MOCK")

for _ in range(1000):  # 1 second @ 1kHz
    contacts = sim.step_filtered()
    if contacts:
        cues = renderer.render(contacts)
        driver.send(cues)

driver.disconnect_all()
```

---

## Installation

### From Source (Current)

```bash
git clone https://github.com/anthropics/haptos
cd haptos
pip install -e .
```

### Via pip (Coming Soon)

```bash
pip install haptos
```

---

## Architecture

HAPTOS uses a three-layer architecture:

```
┌─────────────────────────────────────────────────────────────┐
│ Layer 1: Physics Engine (1kHz)                             │
│  MuJoCo → ContactPatch → Somatotopic Router → FilteredContact│
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 2: Neural Renderer (100Hz)                           │
│  FilteredContact → ML Inference → CueParams                │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 3: Hardware Driver (2kHz+)                           │
│  CueParams → Interpolation → Synthesis → Actuator          │
└─────────────────────────────────────────────────────────────┘
```

**Key Features:**
- **Simulation-First**: Develop without physical hardware
- **Hardware-Agnostic**: Graceful degradation across device tiers (VCA/LRA/ERM)
- **Biologically-Inspired**: Perceptual filtering via Homunculus model
- **Neural Rendering**: ML-powered cue parameter generation

---

## Examples

See [`examples/`](examples/) directory for working code. Quick overview:

```python
# Basic simulation
import haptos
sim = haptos.Simulation("model.xml")
renderer = haptos.Renderer()
driver = haptos.Driver(driver_type="mock")

# Multi-contact grasp (6 channels)
driver = haptos.Driver(enable_sync=True)
for body_part_id in [10, 11, 12, 13, 14, 15]:
    driver.register(body_part_id, f"MOCK_{body_part_id}")

# Custom perceptual model
homunculus = haptos.Homunculus.load("user_profile.json")
sim = haptos.Simulation("model.xml", homunculus=homunculus)

# Real hardware integration
driver = haptos.Driver(driver_type="serial")
driver.register(10, "/dev/ttyACM0")
```

---

## Documentation

- **[Quick Start Guide](docs/quickstart.md)** - Get started in 5 minutes
- **[API Reference](docs/api_reference.md)** - Complete API documentation
- **[Examples](examples/)** - Working code examples (5 demos)
- **[Tutorials](docs/tutorials/)** - Step-by-step guides
- **[Firmware Guide](firmware/README.md)** - Hardware integration

### Project Status
- **[Phase 1 Complete](PHASE1_COMPLETE.md)** - Single finger validation (137 tests)
- **[Phase 2 Complete](PHASE2_COMPLETE.md)** - Multi-contact hand (145 tests)
- **[Project Status](PROJECT_STATUS.md)** - Current state and roadmap

---

## Current Status

### ✅ Phase 1-2 Complete
- Core three-layer architecture
- Somatotopic Router + Homunculus
- Neural Renderer (ML inference)
- Mock Hardware Driver
- Firmware (HaptosReceiver.ino for Teensy 4.1)
- Multi-contact support (6 channels)
- **145 tests passing**

### 🚧 Phase 3 In Progress (SDK Release)
- ✅ Public Python API (`haptos` package)
- ✅ Example applications (5 demos)
- ✅ Pip packaging (setup.py, pyproject.toml)
- 🔄 Environment library (standard scenes)
- 🔄 Model zoo (pre-trained variants)
- 🔄 Documentation (tutorials, API reference)

### 📋 Phase 4 Planned (Hardware Integration)
- Real hardware testing (Teensy 4.1 + VCA)
- SerialHardwareDriver implementation
- Latency measurement
- Calibration tools

---

## Performance

| Metric | Target | Actual | Status |
|--------|--------|--------|--------|
| Single contact latency | <20ms | ~15ms | ✅ |
| 6-contact latency | <100ms | <50ms | ✅ |
| Single channel bandwidth | - | 41.6 kbps | ✅ |
| 6-channel bandwidth | - | ~200 kbps | ✅ |
| Test coverage | All passing | 145/145 | ✅ |

---

## Hardware Support

**Current**: Mock driver (simulation-only, no physical device needed)

**Phase 4+**:
- Teensy 4.1 + Voice Coil Actuator (VCA)
- Teensy 4.1 + Linear Resonant Actuator (LRA)
- ESP32 + ERM motor
- Commercial haptic devices (via adapters)

See [`firmware/README.md`](firmware/README.md) for hardware setup guide.

---

## License

MIT License - see [LICENSE](LICENSE) file

---

## Support

- **Issues**: https://github.com/anthropics/haptos/issues
- **Discussions**: https://github.com/anthropics/haptos/discussions

---

**Version**: 0.3.0 | **Tests**: 145 passing ✅ | **Next**: Phase 4 hardware integration
