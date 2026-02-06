# HAPTOS Architecture

## System Overview

HAPTOS is a perceptually-grounded haptic rendering system that converts physics simulation data into device-specific haptic commands through a three-layer pipeline.

```
ContactPatch (physics) → SensationParams (perception) → SynthesisCommand (hardware)
```

### Design Philosophy

**Separation of Concerns:**
- Neural network learns perception (physics → sensation)
- Deterministic functions handle body adaptation and hardware translation
- No learned synthesis parameters - keeps hardware abstraction clean

**Hardware Agnostic:**
- Same 38-byte SensationParams work across VCA, LRA, ERM actuators
- Translation happens after perception, not during

**Biologically Grounded:**
- Somatotopic shaping based on mechanoreceptor density
- Frequency ranges match body part sensitivity
- Spatial resolution follows two-point discrimination thresholds

---

## Pipeline Architecture

### Layer 1: Feature Extraction
**Input:** ContactPatch (physics simulation)
**Output:** 14-dim feature vector
**Logic:** Deterministic physics analysis

```python
ContactPatch {
    force_normal: float          # Normal force [N]
    force_shear: ndarray[2]      # Tangential forces [N]
    velocity: ndarray[3]         # Contact velocity [m/s]
    contact_area: float          # Contact area [m²]
    material_hint: int           # Material ID (0-5)
    body_part_id: int            # Body location
    timestamp_us: int            # Microsecond timestamp
}
↓
FeatureExtractorV2.extract()
↓
features[14] = [
    normal_force,                # [0] Current force [N]
    shear_magnitude,             # [1] |force_shear| [N]
    force_delta,                 # [2] F_t - F_{t-10} [N]
    slip_speed,                  # [3] Tangential velocity [mm/s]
    acceleration,                # [4] d(slip)/dt [mm/s²]
    hardness,                    # [5] Material hardness [0-1]
    friction,                    # [6] Friction coefficient [0-1]
    roughness,                   # [7] Surface roughness [0-1]
    phase_impact,                # [8] Binary phase flags
    phase_hold,                  # [9]
    phase_slip,                  # [10]
    phase_release,               # [11]
    contact_duration,            # [12] Time in contact [ms]
    contact_area                 # [13] Contact area [m²]
]
```

**Key Components:**
- **10-frame force history buffer** (per body part) for force_delta calculation
- **FSM phase detection** (NO_CONTACT → IMPACT → HOLD → SLIP → RELEASE)
- **Hertzian contact model** for contact_area estimation
- **Material database** lookup for hardness/friction/roughness

---

### Layer 2: Neural Rendering
**Input:** 14-dim features + cue_mask
**Output:** SensationParams (perceptual)
**Logic:** Learned perception model

```python
NeuralRenderer_v2 Architecture:

Input[14]
    ↓
Trunk: Linear(14→128) → ReLU → Linear(128→128) → ReLU → Linear(128→64) → ReLU
    ↓
    ├─→ Impact Head[2]:     intensity, sharpness
    ├─→ Resonance Head[3]:  intensity, brightness, sustain
    ├─→ Texture Head[3]:    roughness, density, depth
    ├─→ Slip Head[4]:       speed, dir_x, dir_y, grip
    └─→ Pressure Head[2]:   magnitude, spread

Total: 27,598 parameters (68% reduction from v1)
```

**SensationParams Schema (38 bytes):**
```python
SensationParams {
    # Impact Channel (transient)
    impact_intensity: float      # 0=none, 1=max perceivable
    impact_sharpness: float       # 0=soft thud, 1=hard click
    impact_trigger: bool          # Onset event flag

    # Resonance Channel (material vibration)
    resonance_intensity: float    # Ringing magnitude
    resonance_brightness: float   # 0=dark/low freq, 1=bright/high freq
    resonance_sustain: float      # 0=damped, 1=long ring

    # Texture Channel (surface feel)
    texture_roughness: float      # 0=smooth, 1=rough
    texture_density: float        # 0=sparse bumps, 1=fine grain
    texture_depth: float          # 0=shallow, 1=deep grooves

    # Slip Channel (lateral motion)
    slip_speed: float             # 0=static, 1=fast slide
    slip_direction: (float, float) # Unit vector (x, y)
    slip_grip: float              # 0=sticky, 1=slippery

    # Pressure Channel (sustained force)
    pressure_magnitude: float     # 0=light touch, 1=heavy press
    pressure_spread: float        # 0=point, 1=diffuse

    # Metadata
    body_part_id: int
    timestamp_us: int
}
```

**Cue Mask Gating:**
```python
CUE_IMPACT    = 0b00001
CUE_RESONANCE = 0b00010
CUE_TEXTURE   = 0b00100
CUE_SLIP      = 0b01000
CUE_PRESSURE  = 0b10000
CUE_ALL       = 0b11111
```

Disabled channels output zero. Palm has reduced texture (0b11011), torso has minimal detail (0b10001).

**Onset Detection:**
- FSM-based rising edge detector
- Trigger on phase_impact transition (0 → 1)
- 30ms debounce to prevent double-firing
- NOT learned by network - deterministic logic

**Training:**
- Analytical labels generated from physics (no human data required)
- Calibration constants k1-k12 map physics → perception
- AdamW optimizer (lr=1e-3, weight_decay=1e-4)
- Weighted MSE loss (impact=2.0x, resonance=1.5x, texture=1.0x, slip=1.0x, pressure=0.5x)

---

### Layer 3A: Somatotopic Shaping
**Input:** SensationParams (body-agnostic)
**Output:** SensationParams (body-adapted)
**Logic:** Deterministic biological scaling

```python
shape(sensation, body) → shaped_sensation

Three transformations:

1. Amplitude Gain (sensitivity compensation)
   gain = min(1.0 / body.sensitivity, 5.0)

   - Fingertip (sensitivity=1.0) → gain=1.0x (no change)
   - Palm (sensitivity=0.5)       → gain=2.0x (amplify)
   - Torso (sensitivity=0.2)      → gain=5.0x (max amplify)

   Applied to: impact_intensity, resonance_intensity, texture_depth,
               slip_speed, pressure_magnitude

2. Frequency Compression (perceptible range)
   freq_ratio = (body.freq_max - body.freq_min) / (500 - 20)

   - Fingertip (20-500Hz)   → ratio=1.00 (full range)
   - Palm (50-300Hz)        → ratio=0.56 (narrowed)
   - Torso (20-100Hz)       → ratio=0.17 (minimal)

   Applied to: resonance_brightness, texture_density

3. Spatial Filtering (two-point discrimination)
   spatial_factor = min(1.0, 5.0 / body.spatial_res_mm)

   - Fingertip (2mm)  → factor=1.00 (full detail)
   - Palm (10mm)      → factor=0.50 (reduced)
   - Torso (45mm)     → factor=0.11 (nearly zero)

   Applied to: texture_roughness, texture_density

Final clamp: all values to [0, 1]
```

**Body Part Properties (Homunculus Table):**
```python
BodyPartProperties {
    sensitivity: float          # 0.2-1.0 (torso-fingertip)
    freq_range_hz: (float, float)  # Perceptible frequency range
    spatial_res_mm: float       # Two-point discrimination threshold
    rendering_tier: int         # 1=VCA, 2=LRA, 3=ERM
    cue_mask: int               # Enabled sensation channels
}
```

---

### Layer 3B: Hardware Translation
**Input:** SensationParams (shaped)
**Output:** SynthesisCommand (device-specific)
**Logic:** Deterministic perceptual-to-synthesis mapping

#### Tier 1: VCA (Voice Coil Actuator)
**Capabilities:** Full bandwidth (20-500Hz), all channels
**Strategy:** Direct perceptual-to-synthesis mapping

```python
VCATranslator.translate(sensation, body):
    # Impact → decaying sine burst
    impact_amplitude = sensation.impact_intensity
    impact_frequency_hz = lerp(freq_min, freq_max, sensation.impact_sharpness)
    impact_decay_ms = lerp(50, 2, sensation.impact_sharpness)  # Sharp = fast decay

    # Resonance → damped oscillation
    resonance_amplitude = sensation.resonance_intensity
    resonance_frequency_hz = lerp(freq_min, freq_max, sensation.resonance_brightness)
    resonance_decay_ms = lerp(5, 200, sensation.resonance_sustain)

    # Texture → bandpass filtered noise
    texture_amplitude = sensation.texture_depth
    texture_center_hz = lerp(freq_min, freq_max, sensation.texture_density)
    texture_bandwidth_hz = lerp(10, 200, sensation.texture_roughness)

    # Slip → amplitude-modulated noise
    slip_amplitude = sensation.slip_speed * (1 - sensation.slip_grip)
    slip_modulation_hz = lerp(5, 50, sensation.slip_speed)

    # Pressure → sustained low-frequency tone
    pressure_amplitude = sensation.pressure_magnitude
    pressure_frequency_hz = freq_min
```

#### Tier 2: LRA (Linear Resonant Actuator)
**Capabilities:** Fixed resonant frequency (~175Hz), amplitude modulation only
**Strategy:** Map all channels to resonant carrier

```python
LRATranslator.translate(sensation, body):
    f_res = 175.0  # Fixed resonant frequency

    # All frequencies clamped to resonance
    impact_frequency_hz = f_res
    resonance_frequency_hz = f_res
    texture_center_hz = f_res

    # Brightness/density lost, only amplitude/duration preserved
    impact_decay_ms = lerp(30, 5, sensation.impact_sharpness)
    resonance_decay_ms = lerp(5, 200, sensation.resonance_sustain)

    # Texture becomes amplitude modulation
    texture_amplitude = sensation.texture_roughness * sensation.texture_depth
    texture_bandwidth_hz = 20.0  # Narrow around resonance
```

#### Tier 3: ERM (Eccentric Rotating Mass)
**Capabilities:** Magnitude only (duty cycle), ~50ms spin-up
**Strategy:** Collapse all channels to single intensity

```python
ERMTranslator.translate(sensation, body):
    # Weighted sum of all channels
    intensity = 0.0
    intensity += sensation.impact_intensity * 1.0
    intensity += sensation.texture_roughness * sensation.texture_depth * 0.5
    intensity += sensation.pressure_magnitude * 0.3
    intensity += sensation.slip_speed * 0.4

    erm_duty_cycle = min(intensity, 1.0)

    # All frequency/direction information lost
```

**SynthesisCommand Schema:**
```python
SynthesisCommand {
    # Impact (one-shot envelope)
    impact_amplitude: float
    impact_frequency_hz: float
    impact_decay_ms: float
    impact_fire: bool

    # Resonance (damped oscillation)
    resonance_amplitude: float
    resonance_frequency_hz: float
    resonance_decay_ms: float

    # Texture (filtered noise)
    texture_amplitude: float
    texture_center_hz: float
    texture_bandwidth_hz: float

    # Slip (AM noise)
    slip_amplitude: float
    slip_direction: (float, float)
    slip_modulation_hz: float

    # Pressure (sustained tone)
    pressure_amplitude: float
    pressure_frequency_hz: float

    # ERM (magnitude only)
    erm_duty_cycle: float
}
```

---

## Data Flow Example

### Scenario: Finger taps metal bar

```
1. Physics Simulation
   ContactPatch {
       force_normal: 2.5 N
       force_shear: [0.1, 0.0] N
       velocity: [0.0, 0.0, 0.0]
       contact_area: 0.0002 m²
       material_hint: 1 (metal)
       body_part_id: 10 (index_tip)
       timestamp_us: 1000000
   }

2. Feature Extraction (14-dim)
   [2.5, 0.1, 2.5, 0.0, 0.0, 0.9, 0.15, 0.1, 1, 0, 0, 0, 0.0, 0.0002]
   Phase: IMPACT (rising edge)

3. Neural Rendering
   SensationParams {
       impact_intensity: 0.82       # High force
       impact_sharpness: 0.91        # Metal = sharp
       impact_trigger: True          # Phase transition
       resonance_intensity: 0.45     # Metal rings
       resonance_brightness: 0.88    # High stiffness = bright
       resonance_sustain: 0.35       # Metal = some ring
       texture_roughness: 0.12       # Metal = smooth
       texture_density: 0.08
       texture_depth: 0.05
       slip_speed: 0.0               # No motion
       slip_direction: (0.0, 0.0)
       slip_grip: 0.5
       pressure_magnitude: 0.65
       pressure_spread: 0.15         # Small contact
   }

4. Somatotopic Shaping (fingertip)
   Unchanged (gain=1.0, freq_ratio=1.0, spatial_factor=1.0)

5. VCA Translation
   SynthesisCommand {
       impact_fire: True
       impact_amplitude: 0.82
       impact_frequency_hz: 420 Hz  # lerp(20, 500, 0.91)
       impact_decay_ms: 4.2 ms      # lerp(50, 2, 0.91)

       resonance_amplitude: 0.45
       resonance_frequency_hz: 442 Hz  # lerp(20, 500, 0.88)
       resonance_decay_ms: 68 ms       # lerp(5, 200, 0.35)

       texture_amplitude: 0.05
       texture_center_hz: 58 Hz
       texture_bandwidth_hz: 32 Hz

       slip_amplitude: 0.0
       pressure_amplitude: 0.65
       pressure_frequency_hz: 20 Hz
   }

6. Hardware Driver
   Synthesizes waveform and sends to actuator
```

---

## Performance Characteristics

### Latency Breakdown (p95)
```
Feature Extraction:    0.06 ms
Neural Inference:      7.50 ms
Somatotopic Shaping:   0.08 ms
Translation:           0.30 ms
------------------------
Total Pipeline:        7.94 ms
```

Target: <15ms for real-time haptics (100Hz update rate)

### Memory Footprint
```
Model Parameters:   27,598 params × 4 bytes = 110 KB
Per-Contact State:  14-dim features × 4 bytes = 56 bytes
History Buffers:    10 frames × 4 bytes × N_bodies
```

### Packet Size
```
SensationParams:    38 bytes (float16 precision)
SynthesisCommand:   ~50 bytes (float32, device-dependent)

Binary Format (SensationParams):
[0xBB 0x66] [version] [body_id] [timestamp:4] [14×float16:28] [checksum]
```

---

## Design Decisions

### Why Separate Perception and Synthesis?

**Problem:** v1 (CueParams) had synthesis instructions hardcoded in neural network output
**Issue:** Different hardware needs different synthesis → requires retraining
**Solution:** Network outputs perceptual parameters (hardware-agnostic) → translators handle synthesis

**Benefits:**
- Same model works on VCA, LRA, ERM
- Can add new hardware without retraining
- Translators are debuggable (no black box)
- Cleaner separation of learned vs engineered

### Why Somatotopic Shaping After Rendering?

**Alternative:** Body-specific networks (12 models for 12 body parts)
**Issue:** 12× model size, 12× training data, no transfer learning
**Solution:** Single fingertip-referenced model + deterministic shaping

**Benefits:**
- 27K params total (vs 331K for 12 models)
- Biological correctness enforced (not learned)
- Easy to tune gain/frequency/spatial factors
- Matches human haptic homunculus

### Why Analytical Labels Instead of Human Data?

**Phase 1:** Physics-based labels (k1-k12 calibration constants)
**Phase 2 (future):** Fine-tune with human perceptual studies

**Rationale:**
- Bootstraps system without expensive human experiments
- Physics provides ground truth structure
- Calibration constants are interpretable
- Can validate network outputs against physics

### Why 14-Dim Features?

**Selected dimensions provide:**
- Force magnitude and dynamics (normal, shear, delta, duration)
- Motion information (slip speed, acceleration)
- Material properties (hardness, friction, roughness)
- Phase context (impact/hold/slip/release)
- Contact geometry (area)

**Excluded:**
- Raw velocity (derived → slip_speed)
- Absolute position (irrelevant to sensation)
- History beyond 10 frames (diminishing returns)

---

## Module Structure

```
src/
├── core/
│   └── schemas.py              # ContactPatch, SensationParams, SynthesisCommand
├── converter/
│   └── feature_extractor_v2.py # 14-dim feature extraction
├── models/
│   └── nn_v2.py                # NeuralRenderer_v2 architecture
├── training/
│   ├── label_generator.py      # Analytical label generation
│   └── train_nn_v2.py          # Training pipeline
├── inference/
│   └── neural_renderer_v2.py   # NeuralRendererService_v2 (inference wrapper)
├── rendering/
│   └── somatotopic_shaper.py   # shape() function
├── hardware/
│   └── translators.py          # VCA/LRA/ERM translators
└── routing/
    └── somatotopic_router.py   # Homunculus (body part properties)
```

---

## Key Invariants

1. **All sensation parameters in [0, 1]** - Normalized perceptual space
2. **Shaping never removes information** - Only attenuates based on body capability
3. **Translators are stateless** - Pure functions (sensation → synthesis)
4. **Onset detection is deterministic** - Not learned, always FSM-based
5. **Body-agnostic rendering** - Network assumes fingertip reference
6. **Cue mask respected everywhere** - Disabled channels always output zero
7. **Phase transitions are clean** - No hysteresis loops in FSM
8. **Force history per body part** - No cross-contamination between contacts

---

## Testing Strategy

### Unit Tests (80 tests)
- Schema serialization/deserialization
- Feature extraction (14-dim correctness)
- Neural network output shapes/ranges
- Somatotopic shaping factors
- Translator logic (VCA/LRA/ERM)

### Integration Tests (8 tests)
- Full pipeline (ContactPatch → SynthesisCommand)
- Multi-body-part scenarios
- Edge cases (zero force, high force)
- Performance benchmarks (<15ms)

### Validation Tests
- Canonical scenarios (impact, hold, texture, slip, release)
- Cross-hardware consistency (same sensation → appropriate synthesis)
- Biological plausibility (shaping matches mechanoreceptor data)
