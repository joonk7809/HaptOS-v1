# HAPTOS Environment Library

Standard MuJoCo environments for haptic development and testing.

## Overview

The environment library provides pre-built scenes for common haptic scenarios:
- **Baseline Testing**: Flat surfaces for controlled experiments
- **Material Testing**: Different textures (wood, metal, rubber, etc.)
- **Manipulation**: Graspable objects for multi-contact scenarios
- **Complex Scenes**: Outdoor environments with natural elements

All environments are validated against the HAPTOS platform and include:
- Proper contact geometry
- Material properties tuned for haptic rendering
- Body part ID mapping for somatotopic routing
- Performance benchmarks

---

## Available Environments

### 1. Flat Surface (Baseline)
**File**: `flat_surface.xml`

Simple flat ground plane for baseline testing.

**Use Cases**:
- Initial calibration
- Single-contact testing
- Impact/hold/release validation

**Properties**:
- Ground material: Rigid (metal-like)
- Contact model: Pyramidal friction cone
- Size: 2m × 2m
- Body parts: Hand model with 6 contact points

**Example**:
```python
import haptos
sim = haptos.Simulation("environments/flat_surface.xml")
```

**Status**: 📋 Planned (Phase 3)

---

### 2. Textured Floor
**File**: `textured_floor.xml`

Floor with multiple texture patches (wood, metal, rubber).

**Use Cases**:
- Texture discrimination testing
- Material inference validation
- Sliding contact scenarios

**Properties**:
- 3 material zones (wood, metal, rubber)
- Different friction coefficients (0.3, 0.7, 0.9)
- Color-coded visual feedback
- Texture frequency ranges validated

**Material Properties**:
| Material | Friction | Hardness | Texture Hz |
|----------|----------|----------|------------|
| Wood     | 0.7      | 0.6      | 80-120     |
| Metal    | 0.3      | 0.9      | 150-250    |
| Rubber   | 0.9      | 0.4      | 50-100     |

**Example**:
```python
import haptos
sim = haptos.Simulation("environments/textured_floor.xml")

# Slide finger across different textures
for _ in range(5000):  # 5 seconds
    contacts = sim.step_filtered()
    # ... render texture cues
```

**Status**: 📋 Planned (Phase 3)

---

### 3. Table with Objects
**File**: `table_with_objects.xml`

Table scene with graspable objects (sphere, cube, cylinder).

**Use Cases**:
- Multi-contact grasping
- Object manipulation
- 6-channel validation

**Properties**:
- 3 graspable objects
- Table surface (wood)
- Realistic object weights (100g-500g)
- Stable grasping poses

**Objects**:
- **Sphere**: 5cm diameter, 100g, smooth
- **Cube**: 6cm side, 250g, textured
- **Cylinder**: 4cm × 8cm, 180g, ridged

**Example**:
```python
import haptos
sim = haptos.Simulation("environments/table_with_objects.xml")

# Multi-finger grasp
driver = haptos.Driver(enable_sync=True)
for body_part_id in [10, 11, 12, 13, 14, 15]:  # 6 channels
    driver.register(body_part_id, f"MOCK_{body_part_id}")

# Grasp sequence: approach → contact → lift
```

**Status**: 📋 Planned (Phase 3)

---

### 4. Forest Medium
**File**: `forest_medium.xml`

Outdoor scene with natural elements (trees, rocks, grass).

**Use Cases**:
- Complex multi-contact scenarios
- Natural material diversity
- Stress testing (20+ simultaneous contacts)

**Properties**:
- 5 tree trunks (rough bark texture)
- 3 large rocks (hard, irregular)
- Grass ground (soft, compliant)
- Ambient debris (twigs, leaves)

**Materials**:
- Bark: High friction, rough texture (200-400 Hz)
- Rock: Low friction, hard (150-200 Hz)
- Grass: Medium compliance, low frequency (30-80 Hz)

**Complexity**:
- 50+ collidable objects
- 100+ potential contact points
- Requires contact prioritization (max_contacts=20)

**Example**:
```python
import haptos
sim = haptos.Simulation(
    "environments/forest_medium.xml",
    max_contacts=20  # Enable prioritization
)

# Navigate complex environment
renderer = haptos.Renderer()
driver = haptos.Driver()

for _ in range(10000):  # 10 seconds
    contacts = sim.step_filtered()

    # May have 20+ contacts (trees, rocks, ground)
    # Router prioritizes by sensitivity × force
```

**Status**: 📋 Planned (Phase 5 - Full Body Scaling)

---

## Environment Specifications

### Contact Geometry

All environments use MuJoCo's collision detection:
- **Contact Model**: Pyramidal friction cone
- **Contact Pairs**: Hand geoms ↔ Environment geoms
- **Friction**: Anisotropic (separate sliding/torsional)

### Material Properties

Mapped to HAPTOS CueParams:
```
MuJoCo solimp/solref → Impact parameters
MuJoCo friction      → Shear cues
Surface texture      → Texture grain_hz
Compliance          → Weight offset scaling
```

### Body Part IDs

Standard hand model mapping (6 contact points):
- `10`: Index fingertip
- `11`: Thumb tip
- `12`: Middle fingertip
- `13`: Ring fingertip
- `14`: Pinky fingertip
- `15`: Palm center

Extended models (16+ points) add:
- `8-9`: Index proximal/middle
- `5-6`: Thumb proximal/middle
- `16-19`: Forearm segments

### Performance Targets

| Environment | Contact Points | Target FPS | Latency |
|-------------|---------------|-----------|---------|
| Flat Surface | 1-2 | 1000 Hz | <10ms |
| Textured Floor | 1-3 | 1000 Hz | <10ms |
| Table Objects | 6-10 | 1000 Hz | <50ms |
| Forest Medium | 20+ | 500 Hz | <100ms |

---

## Creating Custom Environments

### Minimal Template

```xml
<mujoco model="custom_environment">
  <option timestep="0.001" gravity="0 0 -9.81"/>

  <asset>
    <texture name="ground" type="2d" builtin="checker" width="512" height="512"/>
    <material name="ground_mat" texture="ground" rgba="0.8 0.8 0.8 1"/>
  </asset>

  <worldbody>
    <!-- Ground plane -->
    <geom name="floor" type="plane" size="2 2 0.1"
          material="ground_mat" friction="0.7 0.005 0.0001"/>

    <!-- Include hand model -->
    <body name="hand" pos="0 0 0.5">
      <!-- Your hand model here -->
    </body>
  </worldbody>
</mujoco>
```

### Material Configuration

For haptic-friendly materials, set:

```xml
<geom name="object" type="sphere" size="0.05"
      friction="0.7 0.005 0.0001"
      solimp="0.9 0.95 0.001"
      solref="0.02 1.0"
      condim="4"/>
```

**Parameters**:
- `friction`: [sliding, torsional, rolling]
  - Sliding: Primary friction (0.3-0.9)
  - Torsional: Twist resistance (0.001-0.01)
  - Rolling: Roll resistance (0.0001-0.001)

- `solimp`: [dmin, dmax, width]
  - Controls contact compliance
  - Softer: [0.5, 0.9, 0.01]
  - Harder: [0.9, 0.99, 0.001]

- `solref`: [timeconst, dampratio]
  - Spring-damper contact model
  - Stiff: [0.01, 1.0]
  - Compliant: [0.1, 0.5]

### Testing Your Environment

```python
import haptos

# Load custom environment
sim = haptos.Simulation("my_environment.xml")

# Test contact detection
for _ in range(1000):
    contacts = sim.step()
    if contacts:
        print(f"Contact detected: {len(contacts)} points")
        for c in contacts:
            print(f"  Body part {c.body_part_id}: {c.force_normal:.3f}N")
```

---

## Validation Criteria

All standard environments must pass:

1. **Contact Detection**
   - ✅ Contacts detected within 10ms of impact
   - ✅ Force magnitudes realistic (0.1-10N typical)
   - ✅ Body part IDs correctly mapped

2. **Material Properties**
   - ✅ Friction coefficients match specification
   - ✅ Texture frequencies in expected ranges
   - ✅ Compliance matches material type

3. **Performance**
   - ✅ Maintains 1kHz physics rate
   - ✅ Renderer <10ms inference time
   - ✅ No NaN/inf values in contact data

4. **Rendering Quality**
   - ✅ All 5 cue types synthesized correctly
   - ✅ Material discrimination possible
   - ✅ Perceptually realistic feedback

---

## Environment Contributions

Want to add an environment? Submit a PR with:

1. **MuJoCo XML file**
   - Properly formatted
   - Commented material properties
   - Body part ID mapping documented

2. **Validation Script**
   - Tests all validation criteria
   - Reports performance metrics
   - Example usage code

3. **Documentation**
   - Use cases
   - Material properties table
   - Expected contact patterns
   - Performance benchmarks

See [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines.

---

## References

- MuJoCo Documentation: https://mujoco.readthedocs.io
- Contact Modeling: https://mujoco.readthedocs.io/en/stable/modeling.html#contact
- Friction Models: https://mujoco.readthedocs.io/en/stable/modeling.html#friction

---

**Status**: Environment library planned for Phase 3 completion

**Priority Environments**:
1. Flat Surface (baseline testing) - High priority
2. Textured Floor (material testing) - High priority
3. Table Objects (multi-contact) - Medium priority
4. Forest Medium (complex scenes) - Low priority (Phase 5)
