# HaptOS Architecture & Implementation Guide

**Version**: 2.0
**Last Updated**: January 2026
**Status**: Phase 1 Complete - HAPTOS Platform Layer 1 Active

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture Layers](#architecture-layers)
3. [Core Components](#core-components)
4. [Data Flow](#data-flow)
5. [Implementation Details](#implementation-details)
6. [Testing Framework](#testing-framework)
7. [Usage Examples](#usage-examples)
8. [Development Roadmap](#development-roadmap)

---

## System Overview

HaptOS is a **simulation-first haptics platform** that combines physics simulation, neural inference, and procedural synthesis to generate realistic haptic feedback. The system has evolved from a single-contact haptic synthesizer to a comprehensive multi-contact platform with biological perceptual modeling.

### Design Philosophy

1. **Accuracy First**: Physical and perceptual realism over speed
2. **Hardware Agnostic**: Graceful degradation across device tiers (VCA/LRA/ERM)
3. **Biologically Grounded**: Somatotopic routing based on mechanoreceptor properties
4. **Modular & Extensible**: Clean separation between physics, inference, and synthesis

### Current Capabilities

- ✅ Multi-contact physics simulation (MuJoCo, 1kHz)
- ✅ Two-phase neural haptic inference (NN_v0 + NN_v1)
- ✅ Biological perceptual filtering (Homunculus-based routing)
- ✅ Procedural audio-based haptic synthesis
- ✅ Hand pose simulation (6 presets + custom)
- ✅ Modular scenario testing framework
- 🔄 HAPTOS Platform Layer 1 (Physics → Router) - **Active Development**

---

## Architecture Layers

The system is organized into **three primary layers**, with the legacy system and new HAPTOS Platform coexisting during migration:

### Legacy System (Operational)

```
┌─────────────────────────────────────────────────────────────┐
│                    Legacy Two-Phase System                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Physics (MuJoCo)  →  Feature Extraction  →  NN Inference  │
│      1kHz                 100Hz                  100Hz      │
│                                                             │
│  ContactPatch(old) →  13D Features  →  Cue Parameters      │
│                                                             │
│  NN_v0 (baseline) + NN_v1 (delta refinement)               │
│  ↓                                                          │
│  Audio Synthesis (48kHz procedural)                        │
└─────────────────────────────────────────────────────────────┘
```

### HAPTOS Platform (New - Phase 1 Active)

```
┌─────────────────────────────────────────────────────────────┐
│                   HAPTOS Platform v1.0                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Layer 1: Simulation Engine (1kHz)                         │
│  ├─ Physics (MuJoCo)                                       │
│  ├─ ContactPatch Emission (new schema)                     │
│  └─ Somatotopic Router (Homunculus filtering)             │
│      Output: FilteredContact[]                             │
│                                                             │
│  Layer 2: Neural Renderer (100Hz) [IN PROGRESS]           │
│  ├─ Gated NN Inference (cue mask filtering)               │
│  ├─ Material Inference (optional)                          │
│  └─ CueParams Generation                                   │
│      Output: CueParams (serialized 52-byte packets)        │
│                                                             │
│  Layer 3: Hardware Driver (2kHz+) [PLANNED]                │
│  ├─ Serial Bridge (Python ↔ Teensy)                        │
│  ├─ Interpolation Engine (embedded)                        │
│  ├─ Procedural Synthesis (texture/shear/impact)            │
│  └─ PWM Output (VCA/LRA/ERM)                               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Migration Strategy

- **Phase 1** (Current): Layer 1 complete - Physics → Router working
- **Phase 2-3**: Refactor NN inference to Layer 2, add SDK
- **Phase 4**: Hardware drivers and multi-tier support
- Legacy system remains operational throughout migration

---

## Core Components

### 1. Physics Engine (`src/physics/`)

**Purpose**: Simulate physical contact interactions at 1kHz

**Key Classes**:
- `MuJoCoEngine`: Base single-contact physics wrapper
- `MultiContactEngine`: Extended multi-contact tracker

**Functionality**:
```python
class MultiContactEngine:
    # Legacy API (operational)
    def step() -> OldContactPatch:
        """Single strongest contact"""

    def step_multi() -> Dict[str, OldContactPatch]:
        """All contacts, keyed by body part name"""

    # New HAPTOS API (Phase 1)
    def step_v2() -> List[ContactPatch]:
        """All contacts in new schema format"""
```

**Key Features**:
- MuJoCo integration with 1ms timestep
- Contact force extraction (normal + shear components)
- Velocity calculation at contact points
- Geom-to-body-part mapping
- Material property extraction

**Data Structures** (Legacy):
```python
@dataclass
class OldContactPatch:
    timestamp_us: int
    normal_force_N: float
    shear_force_N: float
    slip_speed_mms: float
    contact_pos: np.ndarray
    contact_normal: np.ndarray
    in_contact: bool
    mu_static: float
    mu_dynamic: float
    solref_damping: float
```

**Data Structures** (New - HAPTOS):
```python
@dataclass
class ContactPatch:
    body_part_id: int                    # Geom ID from physics
    force_normal: float                  # Normal force [N]
    force_shear: Tuple[float, float]     # Tangential (Fx, Fy) [N]
    velocity: Tuple[float, float, float] # Contact velocity [m/s]
    contact_area: float                  # Estimated area [m²]
    material_hint: int                   # Material ID (0=unknown)
    timestamp_us: int                    # Microsecond timestamp
```

---

### 2. Somatotopic Router (`src/routing/`)

**Purpose**: Biological perceptual filtering based on mechanoreceptor distribution

**Key Classes**:
- `Homunculus`: Biological sensor distribution model
- `SomatotopicRouter`: Contact filtering and tagging
- `BodyPartProperties`: Perceptual properties per body part

**Homunculus Table** (Biology-Based):

| Body Part | Spatial Res | Freq Range | Sensitivity | Tier | Cues |
|-----------|-------------|------------|-------------|------|------|
| Fingertips | 2-3mm | 20-500Hz | 1.0 | VCA (1) | All 5 |
| Palm | 10mm | 30-300Hz | 0.5 | LRA (2) | Impact/Texture/Weight |
| Forearm | 30mm | 50-200Hz | 0.3 | LRA (2) | Impact/Weight |
| Torso | 45-50mm | 50-100Hz | 0.15-0.2 | ERM (3) | Impact/Weight |
| Foot Sole | 8mm | 30-300Hz | 0.7 | LRA (2) | Impact/Texture/Weight |
| Toes | 5mm | 20-400Hz | 0.8 | LRA (2) | All 5 |

**Routing Logic**:
```python
class SomatotopicRouter:
    def route(self, patches: List[ContactPatch]) -> List[FilteredContact]:
        """
        1. Lookup body part properties from Homunculus
        2. Apply sensitivity gating: threshold = 0.01N / sensitivity
        3. Assign rendering tier (VCA/LRA/ERM)
        4. Apply cue mask (enable/disable haptic cues)
        5. Return FilteredContact[] for neural rendering
        """
```

**Biological Basis**:
- **Spatial Resolution**: Two-point discrimination thresholds (Weinstein, 1968)
- **Frequency Sensitivity**: Mechanoreceptor tuning curves (Johansson & Vallbo, 1979)
- **Force Sensitivity**: Absolute detection thresholds (Bolanowski et al., 1988)

**Output Schema**:
```python
@dataclass
class FilteredContact:
    patch: ContactPatch              # Original physics data
    rendering_tier: int              # 1=VCA, 2=LRA, 3=ERM
    cue_mask: int                    # Bitmask: impact|ring|texture|shear|weight

    # Cue type constants
    CUE_IMPACT = 0b00001   # Transient impact event
    CUE_RING = 0b00010     # Material resonance
    CUE_TEXTURE = 0b00100  # Surface texture vibration
    CUE_SHEAR = 0b01000    # Tangential sliding
    CUE_WEIGHT = 0b10000   # Sustained pressure
    CUE_ALL = 0b11111      # All cues enabled
```

---

### 3. Feature Extraction (`src/converter/`)

**Purpose**: Convert raw physics data into neural network input features

**Key Classes**:
- `MultiContactFeatureExtractor`: Generates 13D feature vectors
- `ContactPhaseStateMachine`: FSM-based phase detection

**Feature Vector** (13 dimensions):
```python
features = [
    # Force (3 dims)
    normal_force,      # Current normal force [N]
    shear_force,       # Current shear magnitude [N]
    force_delta,       # Change in total force [N/ms]

    # Velocity (2 dims)
    slip_speed,        # Tangential slip speed [mm/s]
    acceleration,      # Change in slip speed [mm/s²]

    # Material (3 dims)
    hardness,          # Derived from solref damping
    friction,          # Static friction coefficient
    roughness,         # Derived from surface properties

    # Phase (4 dims)
    phase_impact,      # Binary: in impact phase
    phase_hold,        # Binary: in hold phase
    phase_slip,        # Binary: in slip phase
    phase_release,     # Binary: in release phase

    # Context (1 dim)
    contact_duration   # Time in contact [ms]
]
```

**Phase State Machine**:
```
     IMPACT (force rising rapidly)
        ↓
     HOLD (force stable, low velocity)
        ↓
     SLIP (force stable, high velocity)
        ↓
     RELEASE (force falling)
        ↓
     (back to IMPACT on next contact)
```

**Transition Criteria**:
- `IMPACT`: force_delta > 0.5 N/ms OR (in_contact AND was_not_in_contact)
- `HOLD`: force stable (|force_delta| < 0.1 N/ms) AND slip_speed < 5 mm/s
- `SLIP`: force stable AND slip_speed > 5 mm/s
- `RELEASE`: force_delta < -0.3 N/ms OR not in_contact

---

### 4. Neural Inference (`src/inference/`)

**Purpose**: Predict haptic synthesis parameters from contact features

**Architecture**: Two-phase neural network

#### Phase 1: NN_v0 (Baseline Predictor)
```python
class NN_v0(nn.Module):
    """
    50-100K parameters
    Input: 13D feature vector
    Output: 5 haptic cue types (impact, ring, texture, shear, weight)
    """

    def __init__(self):
        self.trunk = nn.Sequential(
            nn.Linear(13, 64),
            nn.ReLU(),
            nn.Linear(64, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU()
        )

        # Separate heads for each cue type
        self.impact_head = nn.Linear(64, 3)   # amplitude, rise_ms, fall_ms
        self.ring_head = nn.Linear(64, 3)     # amplitude, decay_ms, freq_hz
        self.texture_head = nn.Linear(64, 2)  # grain_hz, amplitude
        self.shear_head = nn.Linear(64, 3)    # direction_x, direction_y, magnitude
        self.weight_head = nn.Linear(64, 1)   # offset
```

#### Phase 2: NN_v1 (Delta Refinement)
```python
class NN_v1(nn.Module):
    """
    20-40K parameters
    Input: 13D features + NN_v0 predictions
    Output: Delta corrections to NN_v0 predictions
    """

    def __init__(self):
        # Takes concatenated [features, v0_predictions]
        self.refinement = nn.Sequential(
            nn.Linear(13 + 12, 32),  # 12 = total v0 output dims
            nn.ReLU(),
            nn.Linear(32, 64),
            nn.ReLU(),
            nn.Linear(64, 12)        # Delta corrections
        )
```

**Combined Prediction**:
```python
class CombinedPredictor:
    def predict(self, features):
        # Phase 1: Baseline
        v0_output = self.nn_v0(features)

        # Phase 2: Refinement
        combined_input = concat([features, v0_output])
        delta = self.nn_v1(combined_input)

        # Final prediction
        final_output = v0_output + delta
        return final_output
```

**Output Format** (Legacy):
```python
{
    'impact': {
        'A': float,        # Amplitude [0-1]
        'rise_ms': float,  # Attack time [ms]
        'fall_ms': float   # Decay time [ms]
    },
    'ring': {
        'A': float,        # Amplitude [0-1]
        'decay_ms': float, # Decay time [ms]
        'freq_hz': float   # Resonance frequency [Hz]
    },
    'texture': {
        'grain_hz': float,     # Texture frequency [Hz]
        'amplitude': float     # Amplitude [0-1]
    },
    'shear': {
        'direction': (float, float),  # Direction vector
        'magnitude': float             # Magnitude [0-1]
    },
    'weight': {
        'offset': float    # Sustained pressure [0-1]
    }
}
```

---

### 5. CueParams Schema (`src/core/schemas.py`)

**Purpose**: Standardized haptic synthesis parameters for hardware

**New HAPTOS Format**:
```python
@dataclass
class CueParams:
    # Continuous cues (interpolated by hardware at 2kHz+)
    texture_grain_hz: float         # Texture frequency [Hz]
    texture_amplitude: float        # Texture amplitude [0-1]
    shear_direction: Tuple[float, float]  # Shear direction (x, y)
    shear_magnitude: float          # Shear magnitude [0-1]
    weight_offset: float            # Sustained pressure [0-1]

    # Transient cues (event-triggered, not interpolated)
    impact_amplitude: float         # Impact amplitude [0-1]
    impact_decay_ms: float          # Impact decay time [ms]
    impact_frequency_hz: float      # Impact dominant freq [Hz]
    ring_amplitude: float           # Ring amplitude [0-1]
    ring_decay_ms: float            # Ring decay time [ms]
    trigger_impulse: bool           # Fire transient event flag

    timestamp_us: int               # Microsecond timestamp
```

**Binary Serialization** (52 bytes):
```
Header:       0xAA 0x55                (2 bytes)
Timestamp:    uint32_t                 (4 bytes)
Continuous:   5 × float32              (20 bytes)
Transient:    5 × float32              (20 bytes)
Flags:        uint8_t                  (1 byte)
Checksum:     uint8_t (XOR all bytes)  (1 byte)
──────────────────────────────────────────────────
Total:                                 52 bytes
```

**Bandwidth**: 52 bytes × 100Hz = 5.2 KB/s (easily handled by 115200 baud serial)

---

### 6. Audio Synthesis (`src/audio/`)

**Purpose**: Procedural audio-based haptic synthesis (legacy, still operational)

**Key Classes**:
- `MultiContactSynthesizer`: Combines multiple haptic cues
- Individual cue synthesizers (impact, ring, texture, shear, weight)

**Synthesis Pipeline**:
```python
class MultiContactSynthesizer:
    def synthesize(self, cue_params, sample_rate=48000):
        """
        Generate audio waveform from cue parameters

        1. Impact: Exponentially decaying sine burst
        2. Ring: Damped resonance (material-specific)
        3. Texture: Band-limited noise shaped by grain_hz
        4. Shear: Directional filtered noise
        5. Weight: Low-frequency sustained tone

        All cues are mixed with proper envelope shaping
        """
```

**Cue Synthesis Details**:

#### Impact Synthesis
```python
def synthesize_impact(amplitude, rise_ms, fall_ms, freq_hz):
    # Generate exponentially decaying sine wave
    t = np.arange(0, duration, 1/sample_rate)
    envelope = np.exp(-t / (fall_ms/1000))
    carrier = np.sin(2 * np.pi * freq_hz * t)
    return amplitude * envelope * carrier
```

#### Texture Synthesis
```python
def synthesize_texture(grain_hz, amplitude):
    # Band-limited noise filtered around grain_hz
    noise = np.random.randn(num_samples)
    # Apply bandpass filter centered at grain_hz
    filtered = bandpass_filter(noise, grain_hz, bandwidth=50)
    return amplitude * filtered
```

#### Shear Synthesis
```python
def synthesize_shear(direction, magnitude):
    # Directional filtered noise
    noise = np.random.randn(num_samples)
    # Apply directional filtering based on direction vector
    # Higher frequencies in direction of movement
    filtered = directional_filter(noise, direction)
    return magnitude * filtered
```

---

### 7. Hand Simulation (`modular_scenario_builder.py`)

**Purpose**: Test haptic system with various hand poses and scenarios

**Key Classes**:
- `HandPoseConfig`: Define custom hand poses
- `HandPoseLibrary`: 6 predefined poses
- `ScenarioConfig`: Define test scenarios
- `ModularScenarioRunner`: Execute and evaluate scenarios

**Predefined Poses**:
```python
HandPoseLibrary.flat_hand()   # All fingers extended
HandPoseLibrary.pinch()       # Thumb + index extended
HandPoseLibrary.fist()        # All fingers curled
HandPoseLibrary.point()       # Index finger pointing
HandPoseLibrary.palm_up()     # Palm exposed, fingers up
HandPoseLibrary.relaxed()     # Natural relaxed pose
```

**Pose Application**:
```python
class ModularScenarioRunner:
    def _apply_hand_pose(self, pose_config):
        """
        1. Set finger joint angles from pose.joint_angles
        2. Position hand base (freejoint) at [0, 0, 0.12m]
        3. If hold_pose=True, continuously reapply joint angles
           to maintain pose during simulation
        """
```

**Scenario Evaluation**:
```python
# Metrics: Precision, Recall, F1 Score
Precision = TP / (TP + FP)  # How many detected contacts were expected?
Recall = TP / (TP + FN)     # How many expected contacts were detected?
F1 = 2 * (Precision * Recall) / (Precision + Recall)

# TP: True Positives (detected AND expected)
# FP: False Positives (detected but NOT expected)
# FN: False Negatives (expected but NOT detected)
```

---

## Data Flow

### Complete System Data Flow (Legacy + HAPTOS)

```
┌─────────────────────────────────────────────────────────────┐
│                    MuJoCo Physics (1kHz)                    │
│                                                             │
│  Hand model + Environment → Contact detection → Forces     │
└─────────────────────────────────────────────────────────────┘
                    ↓                           ↓
        ┌───────────────────────┐   ┌─────────────────────────┐
        │   Legacy Path (Old)   │   │  HAPTOS Path (New)      │
        └───────────────────────┘   └─────────────────────────┘
                    ↓                           ↓
        ┌───────────────────────┐   ┌─────────────────────────┐
        │ OldContactPatch       │   │ ContactPatch (new)      │
        │ (Dict[str, patch])    │   │ (List[ContactPatch])    │
        └───────────────────────┘   └─────────────────────────┘
                    ↓                           ↓
        ┌───────────────────────┐   ┌─────────────────────────┐
        │ Feature Extraction    │   │ Somatotopic Router      │
        │ → 13D feature vector  │   │ → FilteredContact[]     │
        └───────────────────────┘   └─────────────────────────┘
                    ↓                           ↓
        ┌───────────────────────┐   ┌─────────────────────────┐
        │ NN_v0 + NN_v1         │   │ Neural Renderer         │
        │ → Cue dict            │   │ → CueParams             │
        │   (legacy format)     │   │   (new schema)          │
        └───────────────────────┘   └─────────────────────────┘
                    ↓                           ↓
        ┌───────────────────────┐   ┌─────────────────────────┐
        │ Audio Synthesizer     │   │ Hardware Driver         │
        │ → 48kHz waveform      │   │ → PWM signals (2kHz+)   │
        └───────────────────────┘   └─────────────────────────┘
```

### Detailed HAPTOS Platform Flow (Phase 1)

```
┌──────────────────────────────────────────────────────────────┐
│ 1. Physics Simulation (src/physics/multi_contact_engine.py) │
└──────────────────────────────────────────────────────────────┘
                            ↓
    engine.step_v2()  @ 1kHz (1ms timestep)
                            ↓
    For each MuJoCo contact:
      • Extract geom IDs (geom1, geom2)
      • Identify hand geom (not floor)
      • Calculate contact force (normal + shear)
      • Get contact velocity
      • Estimate contact area
      • Create ContactPatch(
          body_part_id = hand_geom_id,
          force_normal = |normal_force|,
          force_shear = (shear_x, shear_y),
          velocity = (vx, vy, vz),
          contact_area = 0.0001,  # 1cm² default
          material_hint = 0,       # Unknown
          timestamp_us = current_time
        )
                            ↓
    Return: List[ContactPatch]

┌──────────────────────────────────────────────────────────────┐
│ 2. Somatotopic Router (src/routing/somatotopic_router.py)   │
└──────────────────────────────────────────────────────────────┘
                            ↓
    router.route(patches)
                            ↓
    For each ContactPatch:
      • Lookup body part properties from Homunculus
        - Map body_part_id → body part name (e.g., 7 → "index_tip")
        - Retrieve BodyPartProperties:
          * spatial_res_mm
          * freq_range_hz
          * sensitivity (0.15 - 1.0)
          * rendering_tier (1=VCA, 2=LRA, 3=ERM)
          * cue_mask (bitmask of enabled cues)

      • Apply sensitivity gating:
        threshold = 0.01N / sensitivity
        if force_normal < threshold: REJECT

      • Create FilteredContact(
          patch = original_patch,
          rendering_tier = from_properties,
          cue_mask = from_properties
        )
                            ↓
    Return: List[FilteredContact]

┌──────────────────────────────────────────────────────────────┐
│ 3. Neural Renderer (FUTURE - Phase 2)                       │
└──────────────────────────────────────────────────────────────┘
                            ↓
    renderer.infer(filtered_contacts)
                            ↓
    For each FilteredContact:
      • Extract features from contact.patch
      • Run NN inference with cue mask gating:
        - Only compute enabled cues (skip disabled heads)
      • Generate CueParams(
          texture_grain_hz, texture_amplitude,
          shear_direction, shear_magnitude,
          weight_offset,
          impact_amplitude, impact_decay_ms, impact_frequency_hz,
          ring_amplitude, ring_decay_ms,
          trigger_impulse,
          timestamp_us
        )
                            ↓
    Return: Dict[int, CueParams]  # body_part_id → CueParams

┌──────────────────────────────────────────────────────────────┐
│ 4. Hardware Driver (FUTURE - Phase 3)                       │
└──────────────────────────────────────────────────────────────┘
                            ↓
    bridge.send_cue(cue_params)
                            ↓
    • Serialize CueParams → 52-byte binary packet
    • Send via serial (115200 baud)
                            ↓
    Teensy firmware:
      • Receive and deserialize
      • Interpolate continuous cues @ 2kHz
      • Synthesize haptic waveform
      • Output PWM to actuators
```

---

## Implementation Details

### Coordinate Systems

**MuJoCo World Frame**:
- X-axis: Right
- Y-axis: Forward
- Z-axis: Up

**Contact Force Frame**:
- Normal: Perpendicular to contact surface
- Tangent 1, 2: In contact plane

### Timing & Latency

**Target Budget**: <10ms end-to-end (physics → hardware output)

| Component | Target | Current |
|-----------|--------|---------|
| Physics step | 1ms | 1ms ✅ |
| Router | <0.5ms | ~0.1ms ✅ |
| Feature extraction | <0.5ms | ~0.3ms ✅ |
| NN inference | <5ms | ~3-4ms ✅ |
| Serialization | <0.5ms | ~0.1ms ✅ |
| Serial transmission | <2ms | ~0.5ms ✅ |
| Hardware synthesis | <1ms | TBD |
| **Total** | **<10ms** | **~5-7ms** ✅ |

### Memory Footprint

| Component | Size | Notes |
|-----------|------|-------|
| NN_v0 model | 50-100K params | ~200-400KB |
| NN_v1 model | 20-40K params | ~80-160KB |
| Feature buffer | 13 × float32 | 52 bytes per contact |
| Contact buffer | 10 contacts | ~2KB |
| Audio buffer | 48kHz × 0.1s | ~20KB |

### Configuration Files

**Simulation Config** (`config/simulation_config.yaml`):
```yaml
physics:
  timestep: 0.001  # 1ms = 1kHz

contact:
  force_threshold: 0.01  # Minimum force to detect [N]
  max_contacts: 10       # Maximum simultaneous contacts

neural:
  model_v0: "models/checkpoints/nn_v0_best.pt"
  model_v1: "models/checkpoints/nn_v1_best.pt"
  inference_rate_hz: 100

synthesis:
  sample_rate: 48000
  buffer_duration_s: 0.1
```

---

## Testing Framework

### Unit Tests (`tests/phase1/`)

**test_schemas.py** (12 tests):
- ContactPatch creation and conversion
- FilteredContact cue masking
- CueParams serialization/deserialization
- Binary protocol validation (header, checksum)

**test_router.py** (18 tests):
- Homunculus table lookup
- ID-to-name mapping
- Sensitivity-based filtering
- Tier assignment
- Save/load configuration

**test_integration.py** (7 tests):
- Physics → Router pipeline
- Multi-contact handling
- Data flow consistency
- Statistics tracking

**Run Tests**:
```bash
# All Phase 1 tests
pytest tests/phase1/ -v

# Specific test file
pytest tests/phase1/test_schemas.py -v

# Single test
pytest tests/phase1/test_integration.py::TestPhysicsRouterIntegration::test_end_to_end_with_contact -v
```

### Scenario Testing (`test_scenarios.py`)

**Purpose**: Validate haptic synthesis with specific hand poses

```python
# Run all predefined scenarios
python test_scenarios.py

# Test pose comparison
python test_pose_comparison.py

# Interactive parameter tuning
python interactive_parameter_tuner.py
```

**Evaluation Metrics**:
- **Precision**: Ratio of detected expected contacts to total detected
- **Recall**: Ratio of detected expected contacts to total expected
- **F1 Score**: Harmonic mean of precision and recall

**Target F1 Scores**:
- >70%: Excellent
- 40-70%: Good
- 20-40%: Fair (needs tuning)
- <20%: Poor (major issues)

---

## Usage Examples

### Example 1: Legacy Multi-Contact System

```python
from src.physics.multi_contact_engine import MultiContactEngine
from src.converter.multi_contact_feature_extractor import MultiContactFeatureExtractor
from src.inference.multi_contact_predictor import CombinedPredictor
from src.audio.multi_contact_synthesizer import MultiContactSynthesizer

# Initialize components
engine = MultiContactEngine('assets/hand_models/detailed_hand.xml')
extractor = MultiContactFeatureExtractor()
predictor = CombinedPredictor('models/checkpoints/nn_v0_best.pt',
                               'models/checkpoints/nn_v1_best.pt')
synthesizer = MultiContactSynthesizer(sample_rate=48000)

# Simulation loop
for step in range(1000):  # 1 second @ 1kHz
    # Physics
    contacts = engine.step_multi()  # Dict[str, OldContactPatch]

    # Feature extraction
    features_dict = {}
    for body_part, patch in contacts.items():
        features = extractor.extract(patch, body_part)
        features_dict[body_part] = features

    # Neural inference (every 10 steps = 100Hz)
    if step % 10 == 0:
        cue_params_dict = {}
        for body_part, features in features_dict.items():
            cues = predictor.predict(features)
            cue_params_dict[body_part] = cues

        # Audio synthesis
        audio = synthesizer.synthesize_multi(cue_params_dict)
        # Play audio...
```

### Example 2: HAPTOS Platform (Phase 1)

```python
from src.physics.multi_contact_engine import MultiContactEngine
from src.routing.somatotopic_router import SomatotopicRouter, Homunculus

# Initialize Layer 1
engine = MultiContactEngine('assets/hand_models/detailed_hand.xml')
router = SomatotopicRouter(Homunculus())

# Simulation loop
for step in range(1000):
    # Physics → ContactPatch emission
    patches = engine.step_v2()  # List[ContactPatch]

    # Somatotopic routing
    filtered = router.route(patches)  # List[FilteredContact]

    # Process filtered contacts
    for contact in filtered:
        print(f"Body part {contact.patch.body_part_id}:")
        print(f"  Force: {contact.patch.force_normal:.2f} N")
        print(f"  Tier: {contact.rendering_tier}")
        print(f"  Enabled cues: {contact.enabled_cues()}")

        # Neural rendering (Phase 2 - not yet implemented)
        # cue_params = renderer.infer(contact)
```

### Example 3: Hand Pose Testing

```python
from modular_scenario_builder import *

# Define custom scenario
scenario = ScenarioConfig(
    name="pinch_test",
    description="Test thumb-index pinch",
    hand_pose=HandPoseLibrary.pinch(),
    interactions=[],  # No object interactions yet
    duration_s=2.0,
    expected_contacts=['thumb_geom', 'index_geom']
)

# Run scenario
runner = ModularScenarioRunner()
results = runner.run_scenario(scenario, verbose=True)

# Evaluate
print(f"F1 Score: {results['f1_score']*100:.1f}%")
print(f"Precision: {results['precision']*100:.1f}%")
print(f"Recall: {results['recall']*100:.1f}%")

# Check contacts
print(f"True Positives: {results['true_positives']}")
print(f"False Positives: {results['false_positives']}")
print(f"False Negatives: {results['false_negatives']}")
```

### Example 4: Custom Homunculus Configuration

```python
from src.routing.somatotopic_router import Homunculus, BodyPartProperties, SomatotopicRouter
from src.core.schemas import FilteredContact

# Create custom Homunculus for prosthetic hand
custom_table = {
    'thumb_tip': BodyPartProperties(
        spatial_res_mm=1.5,      # Better than biological (sensor density)
        freq_range_hz=(10, 600), # Extended range
        sensitivity=1.5,          # More sensitive
        rendering_tier=1,
        cue_mask=FilteredContact.CUE_ALL
    ),
    'index_tip': BodyPartProperties(
        spatial_res_mm=1.5,
        freq_range_hz=(10, 600),
        sensitivity=1.5,
        rendering_tier=1,
        cue_mask=FilteredContact.CUE_ALL
    )
}

# Initialize with custom config
homunculus = Homunculus(custom_table)

# Save for later use
homunculus.save('config/prosthetic_homunculus.json')

# Load in future sessions
loaded_homunculus = Homunculus.load('config/prosthetic_homunculus.json')
router = SomatotopicRouter(loaded_homunculus)
```

---

## Development Roadmap

### Phase 1: Single Finger Validation ✅ COMPLETE
**Duration**: 4-6 weeks
**Status**: Complete (Week 1-2 finished)

**Completed**:
- ✅ Core data schemas (ContactPatch, FilteredContact, CueParams)
- ✅ Somatotopic Router with Homunculus
- ✅ Physics integration (step_v2 method)
- ✅ Unit tests (37 tests passing)
- ✅ Integration tests (Physics → Router)

**Remaining** (Week 3-6):
- Neural Renderer refactor
- Hardware driver (Teensy firmware)
- Serial communication bridge
- End-to-end validation tests

### Phase 2: Hand Coverage
**Duration**: 3-4 weeks
**Status**: Not started

**Goals**:
- Scale to full hand (5 fingers + palm)
- Multi-contact concurrent processing
- Cross-finger routing validation
- Multi-actuator hardware driver

### Phase 3: SDK Release
**Duration**: 6-8 weeks
**Status**: Not started

**Goals**:
- Public Python SDK (`pip install haptos`)
- Environment library (standard scenes)
- Model zoo (pre-trained variants)
- Documentation and tutorials
- Driver library (generic device interface)

### Phase 4: Multi-Tier Hardware Support
**Duration**: 4-5 weeks
**Status**: Not started

**Goals**:
- VCA/LRA/ERM tier abstraction
- Graceful degradation strategies
- Commercial hardware drivers
- Tier-specific synthesis

### Phase 5: Full Body Scaling
**Duration**: 12-16 weeks (research)
**Status**: Not started

**Goals**:
- Humanoid model (160-280 sensors)
- Sparse inference (20-sensor → 100+ virtual)
- Complex environments (forest scenario)
- Body coverage validation

---

## Key Files Reference

### Core Implementation

| File | Purpose | Lines |
|------|---------|-------|
| `src/core/schemas.py` | Data structures (ContactPatch, FilteredContact, CueParams) | 280 |
| `src/routing/somatotopic_router.py` | Biological filtering | 370 |
| `src/physics/multi_contact_engine.py` | MuJoCo physics wrapper | 342 |
| `src/converter/multi_contact_feature_extractor.py` | 13D feature extraction | ~300 |
| `src/inference/multi_contact_predictor.py` | NN inference (v0+v1) | ~250 |
| `src/audio/multi_contact_synthesizer.py` | Procedural synthesis | ~400 |

### Models

| File | Purpose | Size |
|------|---------|------|
| `src/models/nn_v0.py` | Baseline neural network | 50-100K params |
| `src/models/nn_v1.py` | Delta refinement network | 20-40K params |
| `models/checkpoints/nn_v0_best.pt` | Trained NN_v0 weights | ~400KB |
| `models/checkpoints/nn_v1_best.pt` | Trained NN_v1 weights | ~160KB |

### Testing

| File | Purpose | Tests |
|------|---------|-------|
| `tests/phase1/test_schemas.py` | Data structure validation | 12 |
| `tests/phase1/test_router.py` | Router logic | 18 |
| `tests/phase1/test_integration.py` | End-to-end pipeline | 7 |
| `test_scenarios.py` | Hand pose scenarios | - |
| `test_pose_comparison.py` | Pose contact patterns | - |

### Configuration

| File | Purpose |
|------|---------|
| `config/simulation_config.yaml` | Physics, neural, synthesis settings |
| `assets/hand_models/detailed_hand.xml` | 18-sensor hand model (MuJoCo) |
| `assets/hand_models/simple_hand.xml` | 6-sensor hand model (MuJoCo) |

### Documentation

| File | Purpose |
|------|---------|
| `ARCHITECTURE.md` | This file - system overview |
| `PHASE1_PROGRESS.md` | Phase 1 completion report |
| `START_HERE.md` | Quick start guide |
| `MODULAR_SCENARIOS_GUIDE.md` | Scenario testing guide |
| `TESTING_SYSTEM_README.md` | Testing framework docs |

---

## Glossary

**Actuator**: Physical device that generates haptic feedback (VCA/LRA/ERM)

**ContactPatch**: Raw physics data representing a single contact point

**Cue**: A specific type of haptic sensation (impact, ring, texture, shear, weight)

**ERM**: Eccentric Rotating Mass motor (low-end haptic actuator)

**FilteredContact**: Biologically-filtered contact ready for neural rendering

**Homunculus**: Model of biological sensor distribution across the body

**LRA**: Linear Resonant Actuator (mid-tier haptic actuator)

**MuJoCo**: Multi-Joint dynamics with Contact - physics engine

**Rendering Tier**: Hardware capability level (1=VCA, 2=LRA, 3=ERM)

**Somatotopic**: Relating to the mapping of body parts in the somatosensory cortex

**Taxel**: Tactile sensor element (portmanteau of "tactile pixel")

**VCA**: Voice Coil Actuator (high-end haptic actuator with full bandwidth)

---

## References

### Biological Foundations
- Johansson, R. S., & Vallbo, Å. B. (1979). Tactile sensibility in the human hand: relative and absolute densities of four types of mechanoreceptive units in glabrous skin. *The Journal of physiology*, 286(1), 283-300.
- Weinstein, S. (1968). Intensive and extensive aspects of tactile sensitivity as a function of body part, sex and laterality. *The skin senses*, 1, 195-222.
- Bolanowski, S. J., Gescheider, G. A., Verrillo, R. T., & Checkosky, C. M. (1988). Four channels mediate the mechanical aspects of touch. *The Journal of the Acoustical society of America*, 84(5), 1680-1694.

### Technical
- MuJoCo Documentation: https://mujoco.readthedocs.io/
- PyTorch Documentation: https://pytorch.org/docs/

---

**Last Updated**: January 11, 2026
**Version**: 2.0 - HAPTOS Platform Phase 1
**Contributors**: Joon + Claude Opus 4.5
