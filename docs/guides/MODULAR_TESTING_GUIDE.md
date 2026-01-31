# Modular Testing System - User Guide

## Overview

The modular testing system provides flexible, parameter-adjustable testing of the multi-contact haptic system with:

- ✅ **Interactive parameter tuning** - Adjust physics, contact detection, and phase parameters
- ✅ **Detailed hand model** - 18 contact sensors across fingers and palm
- ✅ **Predefined scenarios** - Test specific hand poses and contact patterns
- ✅ **Contact visualization** - 2D hand diagrams showing force distribution
- ✅ **Configuration management** - Save/load parameter sets
- ✅ **Systematic evaluation** - Automated testing with metrics

## New Tools

### 1. Interactive Parameter Tuner
**File:** `interactive_parameter_tuner.py`

Adjust simulation parameters in real-time via CLI menu.

```bash
python interactive_parameter_tuner.py
```

**Features:**
- Adjust physics parameters (timestep, gravity)
- Tune contact detection thresholds
- Configure phase detection FSM
- Run simulations with current parameters
- Save/load configurations
- Reset to defaults

**Example Session:**
```
HAPTIC PARAMETER TUNER
Options:
  1) View current parameters
  2) Adjust physics parameters
  3) Adjust contact detection parameters
  4) Adjust phase detection parameters
  5) Run simulation with current parameters
  ...

Enter choice: 3

ADJUST CONTACT DETECTION PARAMETERS
Current force threshold: 0.010 N
(Minimum force to register contact)
New threshold (N) [press Enter to keep]: 0.05
✓ Set to 0.050 N
```

### 2. Scenario Testing Framework
**File:** `test_scenarios.py`

Test predefined hand poses with automatic evaluation.

```bash
python test_scenarios.py
```

**Available Scenarios:**

1. **Flat Hand Impact** - All fingers extended, palm down
   - Expected contacts: palm + all 5 fingertips
   - Tests: Full hand contact detection

2. **Pinch Pose** - Thumb + index finger
   - Expected contacts: thumb_tip, index_tip
   - Tests: Selective contact, two-finger interaction

3. **Grasp Pose** - All fingers curled
   - Expected contacts: palm + finger middle segments
   - Tests: Multi-segment contact, grasping

4. **Finger Tap** - Index finger pointing
   - Expected contacts: index_tip only
   - Tests: Isolated contact, single-finger interaction

5. **Palm Pressure** - Fingers up, palm down
   - Expected contacts: palm zones only
   - Tests: Palm-only contact, no finger interference

**Output Metrics:**
- Precision: Detected contacts that were expected
- Recall: Expected contacts that were detected
- F1 Score: Harmonic mean of precision and recall
- Force peaks: Maximum force per contact point

**Example Output:**
```
SCENARIO: pinch_pose
Description: Thumb and index finger extended, others closed
Duration: 3.0s
Expected contacts: thumb_tip, index_tip

RESULTS
Detected contacts: 2
  ✓ thumb_tip: peak force = 2.34N
  ✓ index_tip: peak force = 3.12N

Metrics:
  Precision: 100.00%
  Recall: 100.00%
  F1 Score: 100.00%
```

### 3. Contact Visualizer
**File:** `visualize_contacts.py`

Create 2D hand diagrams showing contact force distribution.

```bash
python visualize_contacts.py
```

**Features:**
- Hand diagram with all 18 contact sensors
- Contact markers scaled by force magnitude
- Color-coded contact points
- Force statistics (total, average, max)
- Export to PNG for documentation

**Visual Elements:**
- Red circles: Contact points (size = force magnitude)
- Palm zones: Base, center, thumb base
- Finger segments: Proximal, middle, tip
- Force labels: N values on each contact

### 4. Improved Hand Model
**File:** `assets/hand_models/detailed_hand.xml`

Enhanced hand model with:
- **18 contact sensors** (vs 6 in simple model)
- **5 fingers** with 3 segments each (proximal, middle, distal)
- **3 palm zones** (center, thumb base, base)
- **Capsule geometry** (better visual than spheres)
- **Realistic proportions** and joint ranges
- **Individual joint control** (15 DOF)

**Contact Sensors:**
```
Palm (3 sensors):
  - palm_center
  - palm_thumb_base
  - palm_base

Thumb (3 sensors):
  - thumb_proximal, thumb_middle, thumb_tip

Index (3 sensors):
  - index_proximal, index_middle, index_tip

Middle (3 sensors):
  - middle_proximal, middle_middle, middle_tip

Ring (3 sensors):
  - ring_proximal, ring_middle, ring_tip

Pinky (3 sensors):
  - pinky_proximal, pinky_middle, pinky_tip

Total: 18 contact sensors
```

### 5. Configuration System
**File:** `config/simulation_config.yaml`

YAML-based parameter configuration.

**Editable Parameters:**

```yaml
# Physics
physics:
  timestep: 0.001  # 1 kHz
  gravity: [0, 0, -9.81]

# Contact Detection
contact:
  force_threshold: 0.01  # Minimum force (N)
  slip_threshold: 5.0    # Minimum slip (mm/s)
  max_contacts: 20       # Max simultaneous contacts

# Phase Detection (FSM)
phase_detection:
  impact_threshold: 0.5      # IMPACT trigger (N)
  hold_time_ms: 20           # Time before HOLD (ms)
  slip_speed_threshold: 5.0  # SLIP trigger (mm/s)
  release_threshold: 0.25    # RELEASE trigger (N)

# Material Properties
materials:
  default:
    friction: [0.8, 0.005, 0.0001]
    solref: [0.02, 1.0]  # stiffness, damping
  soft:
    friction: [0.6, 0.004, 0.0001]
    solref: [0.05, 2.0]  # Softer
  hard:
    friction: [0.9, 0.006, 0.0001]
    solref: [0.01, 0.5]  # Harder
```

**Usage:**
1. Edit `config/simulation_config.yaml` directly
2. Or use interactive tuner to adjust and save
3. Load in any script: `yaml.safe_load(open('config/simulation_config.yaml'))`

## Quick Start Guides

### Guide 1: Test New Hand Model

```bash
# 1. Load and visualize new hand model
python test_hand_simulation.py

# Expected: More detailed contact information
# You should see: thumb_tip, index_middle, etc. (18 possible sensors)
```

### Guide 2: Tune Contact Detection

```bash
# 1. Run interactive tuner
python interactive_parameter_tuner.py

# 2. Choose option 3 (Adjust contact detection)
# 3. Set force threshold (e.g., 0.05N instead of 0.01N)
# 4. Choose option 5 (Run simulation)
# 5. Observe: Fewer weak contacts detected
# 6. Choose option 7 (Save configuration)
```

### Guide 3: Run Scenario Tests

```bash
# 1. Run scenario framework
python test_scenarios.py

# 2. Choose option 0 (Run all scenarios)
# 3. Review results for each scenario
# 4. Check F1 scores and contact accuracy
# 5. Save results to JSON
```

### Guide 4: Visualize Contacts

```bash
# 1. Run visualization demo
python visualize_contacts.py

# 2. View 2D hand diagram with contacts
# 3. Save to PNG file
# 4. Use for documentation or analysis
```

## Parameter Tuning Guidelines

### Contact Detection Threshold

**Force Threshold** (`contact.force_threshold`)
- **Too low (< 0.01N)**: Noise detected as contacts
- **Too high (> 0.1N)**: Light touches missed
- **Recommended**: 0.01-0.05N depending on application

**Slip Threshold** (`contact.slip_threshold`)
- **Too low (< 1.0 mm/s)**: Everything classified as slip
- **Too high (> 10.0 mm/s)**: Slip phase never triggered
- **Recommended**: 3.0-7.0 mm/s

### Phase Detection Tuning

**Impact Threshold** (`phase_detection.impact_threshold`)
- **Effect**: Force required to trigger IMPACT phase
- **Too low**: Every contact immediately goes to IMPACT
- **Too high**: IMPACT phase never reached
- **Recommended**: 0.3-0.7N

**Hold Time** (`phase_detection.hold_time_ms`)
- **Effect**: Duration before HOLD phase activates
- **Too short**: Immediate transition to HOLD
- **Too long**: HOLD phase rarely reached
- **Recommended**: 15-30ms

**Release Threshold** (`phase_detection.release_threshold`)
- **Effect**: Force below which RELEASE phase triggers
- **Should be**: 0.3-0.5x impact_threshold
- **Recommended**: 0.2-0.3N

### Material Properties

**Friction** (`materials.*.friction`)
- Format: `[static, dynamic, rolling]`
- **Static**: Resistance to initial movement (0.5-1.0)
- **Dynamic**: Resistance during motion (0.003-0.007)
- **Rolling**: Rolling resistance (0.0001-0.001)

**Solref** (`materials.*.solref`)
- Format: `[timeconst, dampratio]`
- **Timeconst**: Lower = stiffer (0.01-0.05)
- **Dampratio**: Higher = more damped (0.5-2.0)
- **Examples:**
  - Hard surface: `[0.01, 0.5]`
  - Medium: `[0.02, 1.0]`
  - Soft surface: `[0.05, 2.0]`

## Comparison: Simple vs Detailed Hand

| Feature | Simple Hand | Detailed Hand |
|---------|-------------|---------------|
| Contact sensors | 6 | 18 |
| Finger segments | 1 (tip only) | 3 (proximal, middle, tip) |
| Palm zones | 1 | 3 |
| Geometry | Spheres | Capsules |
| Visual realism | Low | Medium-High |
| Computation | Faster | Slightly slower |
| Contact detail | Basic | Comprehensive |

**When to use Simple Hand:**
- Quick prototyping
- Performance-critical applications
- Testing basic concepts
- Minimal contact detail needed

**When to use Detailed Hand:**
- Realistic simulations
- Research and evaluation
- Detailed contact analysis
- Publication-quality visualizations

## Advanced Usage

### Custom Scenario Creation

```python
from test_scenarios import TestScenario, ScenarioRunner

# Define custom scenario
custom_scenario = TestScenario(
    name="my_custom_test",
    description="Custom finger configuration",
    hand_model="assets/hand_models/detailed_hand.xml",
    duration_s=5.0,
    hand_pose={
        'thumb_cmc': 0.5,
        'index_mcp': 0.8,
        # ... other joints
    },
    expected_contacts=['thumb_tip', 'index_tip', 'middle_tip']
)

# Run scenario
runner = ScenarioRunner()
results = runner.run_scenario(custom_scenario)
```

### Programmatic Parameter Tuning

```python
import yaml

# Load config
with open('config/simulation_config.yaml', 'r') as f:
    config = yaml.safe_load(f)

# Modify parameters
config['contact']['force_threshold'] = 0.05
config['phase_detection']['impact_threshold'] = 0.6

# Save config
with open('config/simulation_config.yaml', 'w') as f:
    yaml.dump(config, f)

# Use in simulation
from sync_multi_runner import MultiContactSyncRunner
runner = MultiContactSyncRunner(
    model_path=config['hand']['model_path']
)
```

### Batch Testing

```python
# Test multiple parameter combinations
force_thresholds = [0.01, 0.02, 0.05, 0.1]
results_summary = {}

for threshold in force_thresholds:
    # Modify config
    config['contact']['force_threshold'] = threshold

    # Run scenarios
    runner = ScenarioRunner()
    for scenario in scenarios:
        result = runner.run_scenario(scenario, verbose=False)
        results_summary[f"threshold_{threshold}"] = result
```

## Performance Tips

1. **Reduce contact threshold** if simulation is slow with many contacts
2. **Use simple hand model** for initial testing
3. **Limit max_contacts** in config (e.g., 10 instead of 20)
4. **Disable inference** if only testing physics (`inference.enabled: false`)
5. **Reduce simulation duration** for quick iterations

## Troubleshooting

### "Unknown body part" in visualization
**Cause:** Contact sensor name doesn't match visualization mapping
**Fix:** Check geom names in hand model match expected pattern

### No contacts detected
**Cause:** Force threshold too high or hand spawn height too high
**Fix:** Lower `contact.force_threshold` or check hand position

### Too many false positives
**Cause:** Force threshold too low, detecting noise
**Fix:** Increase `contact.force_threshold` to 0.03-0.05N

### IMPACT phase never reached
**Cause:** `impact_threshold` too high
**Fix:** Lower to 0.3-0.5N

### Contacts detected but no specific body part
**Cause:** Geom naming doesn't match extraction pattern
**Fix:** Check `_extract_body_part()` in `multi_contact_engine.py`

## File Reference

**New Files:**
- `interactive_parameter_tuner.py` - Parameter adjustment tool
- `test_scenarios.py` - Scenario testing framework
- `visualize_contacts.py` - Contact visualization
- `config/simulation_config.yaml` - Configuration file
- `assets/hand_models/detailed_hand.xml` - Enhanced hand model

**Modified Files:**
- None (backward compatible)

**Data Outputs:**
- `data/test_results/*.json` - Scenario test results
- `data/visualizations/*.png` - Contact diagrams
- `logs/simulation.log` - Simulation logs

## Next Steps

1. **Test detailed hand model:**
   ```bash
   python test_hand_simulation.py
   ```

2. **Tune parameters:**
   ```bash
   python interactive_parameter_tuner.py
   ```

3. **Run scenarios:**
   ```bash
   python test_scenarios.py
   ```

4. **Visualize results:**
   ```bash
   python visualize_contacts.py
   ```

5. **Iterate:** Adjust parameters based on results, repeat

## Tips for Best Results

- Start with default parameters
- Tune one parameter at a time
- Run multiple scenarios to validate changes
- Save configurations that work well
- Document your parameter choices
- Use visualization to understand contact patterns
- Compare F1 scores across parameter sets

Enjoy exploring the modular testing system! 🚀
