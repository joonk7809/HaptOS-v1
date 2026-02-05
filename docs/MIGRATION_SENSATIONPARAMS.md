# SensationParams Migration Guide

**Migration from CueParams to SensationParams Architecture**

**Date**: February 2026
**Version**: v2.0
**Status**: Production Ready

---

## Table of Contents

1. [Overview](#overview)
2. [Why Migrate?](#why-migrate)
3. [Architecture Comparison](#architecture-comparison)
4. [Schema Changes](#schema-changes)
5. [Code Migration Examples](#code-migration-examples)
6. [Performance Comparison](#performance-comparison)
7. [Testing Checklist](#testing-checklist)
8. [Backward Compatibility](#backward-compatibility)
9. [FAQ](#faq)

---

## Overview

The SensationParams architecture replaces the synthesis-instruction-based CueParams with a perceptually-grounded sensation model. This migration guide helps you transition existing code to the new architecture.

### Key Changes

- **Output Schema**: CueParams (52 bytes, 11 params) → SensationParams (38 bytes, 14 params)
- **Neural Model**: NN_v0 + NN_v1 (86K params) → NeuralRenderer_v2 (27K params)
- **Feature Vector**: 13-dim → 14-dim
- **New Pipeline**: Renderer → **Somatotopic Shaping** → **Translator** → Driver

---

## Why Migrate?

### Benefits

1. **Hardware-Agnostic**: Same perceptual model works across VCA, LRA, ERM
2. **Smaller Model**: 27K params (vs 86K) - faster inference, less memory
3. **Smaller Packets**: 38 bytes (vs 52) - 27% bandwidth savings
4. **Biologically Accurate**: Somatotopic shaping matches body part sensitivity
5. **Modular**: Clear separation between perception (learned) and synthesis (engineered)

### Performance Gains

| Metric | CueParams (Old) | SensationParams (New) | Improvement |
|--------|-----------------|----------------------|-------------|
| Packet Size | 52 bytes | 38 bytes | **27% smaller** |
| Model Parameters | 86,000 | 27,598 | **68% reduction** |
| Inference Latency | ~10ms | ~8ms | **20% faster** |
| Hardware Tiers | 1 (VCA only) | 3 (VCA/LRA/ERM) | **3x flexibility** |

---

## Architecture Comparison

### Old Architecture (CueParams)

```
ContactPatch → FeatureExtractor (13-dim) → NeuralRenderer
                                                ↓
                                          CueParams (synthesis instructions)
                                                ↓
                                            Hardware Driver
```

**Problems**:
- Synthesis instructions hardcoded in neural network
- Single hardware tier (VCA only)
- Large model (86K params)
- 52-byte packets

### New Architecture (SensationParams)

```
ContactPatch → FeatureExtractor_v2 (14-dim) → NeuralRenderer_v2
                                                     ↓
                                           SensationParams (perceptual)
                                                     ↓
                                          Somatotopic Shaping
                                                     ↓
                                         Translator (VCA/LRA/ERM)
                                                     ↓
                                            SynthesisCommand
                                                     ↓
                                              Hardware Driver
```

**Advantages**:
- Perceptual parameters (hardware-agnostic)
- Body-part adaptation (somatotopic shaping)
- Hardware-specific translation
- Smaller model (27K params)
- 38-byte packets

---

## Schema Changes

### CueParams (Old)

```python
@dataclass
class CueParams:
    # Continuous cues
    texture_grain_hz: float
    texture_amplitude: float
    shear_direction: Tuple[float, float]
    shear_amplitude: float
    weight_magnitude: float
    weight_onset_flag: int

    # Transient cues
    impact_amplitude: float
    impact_frequency_hz: float
    impact_decay_ms: float
    ring_amplitude: float
    ring_frequency_hz: float

    # Trigger
    impact_trigger: bool

    # Metadata
    body_part_id: int
    timestamp_us: int

# Binary: 52 bytes, header 0xAA 0x55
```

### SensationParams (New)

```python
@dataclass
class SensationParams:
    # Impact Channel (transient events)
    impact_intensity: float      # 0=none, 1=max perceivable
    impact_sharpness: float       # 0=soft thud, 1=hard click
    impact_trigger: bool          # Onset event flag

    # Resonance Channel (material vibration)
    resonance_intensity: float    # How much ringing after impact
    resonance_brightness: float   # 0=dark/low freq, 1=bright/high freq
    resonance_sustain: float      # 0=dead/damped, 1=long ring

    # Texture Channel (surface feel)
    texture_roughness: float      # 0=smooth, 1=rough
    texture_density: float        # 0=sparse bumps, 1=fine grain
    texture_depth: float          # 0=shallow, 1=deep grooves

    # Slip Channel (lateral motion)
    slip_speed: float             # 0=static, 1=fast slide
    slip_direction: Tuple[float, float]  # Unit vector (x,y)
    slip_grip: float              # 0=sticky, 1=slippery

    # Pressure Channel (sustained force)
    pressure_magnitude: float     # 0=light touch, 1=heavy press
    pressure_spread: float        # 0=point contact, 1=diffuse

    # Metadata
    body_part_id: int
    timestamp_us: int

# Binary: 38 bytes, header 0xBB 0x66
```

### Mapping: CueParams → SensationParams

| CueParams Field | SensationParams Field | Notes |
|-----------------|----------------------|-------|
| `impact_amplitude` | `impact_intensity` | Direct mapping |
| `impact_trigger` | `impact_trigger` | Direct mapping |
| `ring_amplitude` | `resonance_intensity` | Renamed for clarity |
| `ring_frequency_hz` | ❌ Removed | Now computed from `resonance_brightness` |
| `texture_amplitude` | `texture_depth` | Renamed |
| `texture_grain_hz` | ❌ Removed | Now computed from `texture_density` |
| `shear_amplitude` | `slip_speed` | Renamed |
| `shear_direction` | `slip_direction` | Direct mapping |
| `weight_magnitude` | `pressure_magnitude` | Renamed |
| ❌ | `impact_sharpness` | **New**: Soft vs sharp impact |
| ❌ | `resonance_brightness` | **New**: Dark vs bright resonance |
| ❌ | `resonance_sustain` | **New**: Dead vs ringing |
| ❌ | `texture_roughness` | **New**: Smooth vs rough |
| ❌ | `texture_density` | **New**: Coarse vs fine |
| ❌ | `slip_grip` | **New**: Sticky vs slippery |
| ❌ | `pressure_spread` | **New**: Point vs diffuse |

---

## Code Migration Examples

### Example 1: Basic Rendering

**Old Code (CueParams)**:
```python
from src.haptos import Renderer, Driver

# Create renderer (old)
renderer = Renderer(use_v2=False)

# Render
cues = renderer.render(filtered_contacts)

# Send to driver
driver = Driver(driver_type="mock")
driver.send(cues)
```

**New Code (SensationParams)**:
```python
from src.haptos import Renderer, Driver
from src.rendering.somatotopic_shaper import shape
from src.hardware.translators import create_translator
from src.routing.somatotopic_router import Homunculus

# Create renderer (v2)
renderer = Renderer(use_v2=True)

# Create homunculus and translator
homunculus = Homunculus()
translator = create_translator(tier=1)  # VCA

# Render sensations
sensations = renderer.render(filtered_contacts)

# Shape and translate
synthesis_commands = {}
for body_id, sensation in sensations.items():
    body = homunculus.lookup_by_id(body_id)
    shaped = shape(sensation, body)
    synthesis_commands[body_id] = translator.translate(shaped, body)

# Send to driver
driver = Driver(driver_type="mock")
driver.send(synthesis_commands)
```

### Example 2: Multi-Hardware Tiers

**New Feature (Not Possible with CueParams)**:
```python
from src.hardware.translators import VCATranslator, LRATranslator, ERMTranslator

# Choose translator based on body part
translators = {
    'index_tip': VCATranslator(),     # Tier 1: Full bandwidth
    'palm_center': LRATranslator(),   # Tier 2: Narrowband
    'chest': ERMTranslator(),         # Tier 3: Magnitude only
}

for body_id, sensation in sensations.items():
    body_name = homunculus.id_to_name(body_id)
    translator = translators[body_name]

    shaped = shape(sensation, body)
    synthesis_cmd = translator.translate(shaped, body)

    driver.send_single(body_id, synthesis_cmd)
```

### Example 3: Simplified Driver Integration

**New Code (Recommended)**:
```python
from src.haptos import Driver

# Driver now handles shaping + translation internally
driver = Driver(
    driver_type="mock",
    hardware_tier=1,  # NEW: Specify VCA/LRA/ERM
    homunculus=homunculus
)

# Just pass sensations directly!
driver.send(sensations)  # Driver applies shaping + translation
```

---

## Performance Comparison

### Inference Latency

```
Old (CueParams):
  FeatureExtractor (13-dim): 0.05ms
  NeuralRenderer (86K):      9.80ms
  Total:                     9.85ms

New (SensationParams):
  FeatureExtractor_v2 (14-dim): 0.06ms
  NeuralRenderer_v2 (27K):      7.50ms
  Somatotopic Shaping:          0.08ms
  Translator:                   0.30ms
  Total:                        7.94ms

Improvement: 19% faster
```

### Memory Footprint

```
Old Model Size:  86,000 params × 4 bytes = 344 KB
New Model Size:  27,598 params × 4 bytes = 110 KB

Reduction: 68% smaller
```

### Packet Size (Bandwidth)

```
Old Packet:  52 bytes @ 100Hz = 5.2 KB/s per channel
New Packet:  38 bytes @ 100Hz = 3.8 KB/s per channel

Savings: 27% bandwidth reduction
```

---

## Testing Checklist

### Pre-Migration

- [ ] Verify current CueParams code works
- [ ] Document current performance metrics (latency, bandwidth)
- [ ] Backup existing model checkpoints

### During Migration

- [ ] Update schemas: Import `SensationParams` instead of `CueParams`
- [ ] Update renderer: Set `use_v2=True`
- [ ] Add somatotopic shaping: Import `shape` function
- [ ] Add translator: Import `create_translator`
- [ ] Update driver: Specify `hardware_tier`
- [ ] Update unit tests: Test new pipeline

### Post-Migration

- [ ] Run integration tests (`tests/integration/test_full_pipeline.py`)
- [ ] Verify latency <15ms end-to-end
- [ ] Verify all body parts work (fingertip, palm, torso)
- [ ] Verify all hardware tiers work (VCA, LRA, ERM)
- [ ] Compare performance: New vs Old
- [ ] Test with real hardware (if available)

---

## Backward Compatibility

### Coexistence Strategy

**Both schemas can coexist in the codebase**:

```python
# Old code continues to work
renderer_old = Renderer(use_v2=False)
cues_old = renderer_old.render(contacts)

# New code runs in parallel
renderer_new = Renderer(use_v2=True)
sensations_new = renderer_new.render(contacts)
```

### Header Distinction

Packets are distinguished by header bytes:
- **CueParams**: `0xAA 0x55` (old)
- **SensationParams**: `0xBB 0x66` (new)

Firmware can detect and handle both packet types.

### Gradual Migration

1. **Phase 1**: Run both pipelines in parallel, compare outputs
2. **Phase 2**: Switch to SensationParams in simulation
3. **Phase 3**: Update firmware to support both packet types
4. **Phase 4**: Fully migrate to SensationParams, deprecate CueParams

---

## FAQ

### Q: Do I need to retrain my neural network?

**A**: Yes. NeuralRenderer_v2 uses analytical labels and a different architecture. Use the training pipeline:

```python
from src.training.train_nn_v2 import train_model

train_model(
    feature_dim=14,
    epochs=100,
    lr=1e-3
)
```

### Q: Will my existing hardware work?

**A**:
- **Simulation**: Works immediately (no hardware)
- **Real Hardware**: Requires firmware update to support 38-byte SensationParams packets

### Q: Can I use SensationParams with ERM actuators?

**A**: Yes! This is a new capability. Use `ERMTranslator`:

```python
translator = ERMTranslator()
synthesis_cmd = translator.translate(sensation, body)
```

### Q: What if I only have VCA hardware?

**A**: Use Tier 1 translator:

```python
driver = Driver(hardware_tier=1)  # VCA
```

### Q: How do I debug the new pipeline?

**A**: Use verbose logging and inspect intermediate outputs:

```python
# Enable logging
import logging
logging.basicConfig(level=logging.DEBUG)

# Inspect outputs
print(f"Sensation: {sensation.to_dict()}")
print(f"Shaped: {shaped.to_dict()}")
print(f"Synthesis: {synthesis_cmd.to_dict()}")
```

### Q: What's the migration timeline?

**A**: Recommended timeline:
- **Week 1-2**: Read this guide, run tests, understand changes
- **Week 3-4**: Migrate simulation code, run integration tests
- **Week 5-6**: Update firmware (if using real hardware)
- **Week 7+**: Production deployment

---

## Summary

### Migration Steps

1. ✅ **Update Imports**
   ```python
   from src.core.schemas import SensationParams  # NEW
   from src.rendering.somatotopic_shaper import shape  # NEW
   from src.hardware.translators import create_translator  # NEW
   ```

2. ✅ **Update Renderer**
   ```python
   renderer = Renderer(use_v2=True)  # Enable v2
   ```

3. ✅ **Add Shaping + Translation**
   ```python
   shaped = shape(sensation, body)
   synthesis_cmd = translator.translate(shaped, body)
   ```

4. ✅ **Update Driver**
   ```python
   driver = Driver(hardware_tier=1)  # Specify tier
   ```

5. ✅ **Test**
   ```bash
   python tests/integration/test_full_pipeline.py
   ```

### Next Steps

- See `examples/` for full examples
- See `tests/integration/` for integration tests
- See `docs/api_reference.md` for API details

---

**Questions?** Open an issue: [GitHub Issues](https://github.com/anthropics/haptos/issues)

**Last Updated**: February 2026
