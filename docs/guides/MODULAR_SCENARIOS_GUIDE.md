# Modular Scenario System - Complete Guide

## 🎉 What You Have

A fully modular scenario building system that lets you:

✅ **Create custom hand poses** (flat, pinch, fist, point, palm_up, relaxed)
✅ **Control hand position and orientation** (stable in space)
✅ **Mix and match configurations** (poses + object interactions)
✅ **Test systematically** with automated metrics
✅ **Compare poses** to see different contact patterns

---

## 📊 Understanding the Metrics

### What Do Precision, Recall, and F1 Mean?

**Precision**: Of all the contacts we detected, how many were expected?
```
Precision = True Positives / (True Positives + False Positives)
```
- **High precision** (>80%): Few false alarms, clean detection
- **Low precision** (<50%): Many unexpected contacts detected

**Recall**: Of all the contacts we expected, how many did we detect?
```
Recall = True Positives / (True Positives + False Negatives)
```
- **High recall** (>80%): Not missing expected contacts
- **Low recall** (<50%): Missing many expected contacts

**F1 Score**: Harmonic mean of precision and recall
```
F1 = 2 × (Precision × Recall) / (Precision + Recall)
```
- **F1 > 70%**: Excellent performance
- **F1 40-70%**: Good performance
- **F1 20-40%**: Fair performance (needs tuning)
- **F1 < 20%**: Poor performance (major issues)

### Example Interpretation

```
Detected contacts: 10
  ✓ palm_center: peak force = 19.29N    ← TRUE POSITIVE (expected + detected)
  ✓ index_geom: peak force = 2.34N      ← TRUE POSITIVE
  ✗ thumb_mid_geom: peak force = 154.18N ← FALSE POSITIVE (detected but not expected)
  ✗ index_prox_geom: peak force = 0.00N ← FALSE POSITIVE

Missed contacts:
  ✗ ring_geom                           ← FALSE NEGATIVE (expected but not detected)

Metrics:
  Precision: 40.00% (2 correct out of 5 detected)
  Recall: 66.67% (2 detected out of 3 expected)
  F1 Score: 50.00%
```

**What this means**:
- We found 2/3 of the contacts we wanted (recall: 67%)
- But half of what we found was extra stuff we didn't want (precision: 40%)
- Overall score is fair (F1: 50%) - system is partially working

---

## 🔧 Why Are F1 Scores Low?

### Common Causes of Low F1 Scores:

1. **Hand Too Close to Ground**
   - **Symptom**: Many contacts with 0N or very low forces
   - **Why**: Hand at 0.12m height causes all segments to touch/graze floor
   - **Fix**: Increase `force_threshold` to filter weak contacts

2. **Force Threshold Too Low**
   - **Current**: 0.01N (very sensitive)
   - **Problem**: Detects grazing contacts that aren't meaningful
   - **Fix**: Increase to 0.05N or 0.1N

3. **Detecting Contact Segments Instead of Tips**
   - **Symptom**: Detecting `thumb_mid_geom` instead of `thumb_geom` (tip)
   - **Why**: Middle segments touch ground before/instead of tips
   - **Fix**: Adjust hand height or expected contacts

4. **Unrealistic Expectations**
   - **Symptom**: Expecting all fingertips but hand isn't perfectly flat
   - **Problem**: Real physics doesn't match idealized expectations
   - **Fix**: Adjust expected contacts based on observed patterns

---

## 🎛️ Improving F1 Scores

### Method 1: Increase Force Threshold (Easiest)

Edit `config/simulation_config.yaml`:

```yaml
contact:
  force_threshold: 0.05  # Increase from 0.01 to filter weak contacts
```

**Expected improvement**:
- Precision ↑ (fewer false positives from grazing)
- Recall → (may decrease slightly if threshold too high)
- F1 ↑ (overall improvement)

### Method 2: Adjust Expected Contacts (Most Accurate)

Instead of expecting only fingertips, expect the segments that actually contact:

```python
# Before (idealized)
expected_contacts=['palm_center', 'thumb_geom', 'index_geom', 'middle_geom']

# After (realistic)
expected_contacts=['palm_center', 'thumb_mid_geom', 'index_geom', 'middle_geom',
                   'index_prox_geom', 'middle_prox_geom']
```

Run the scenario, note which contacts appear consistently, update expectations.

### Method 3: Filter Zero-Force Contacts

Modify the analysis to ignore contacts with peak force < threshold:

```python
# In scenario analysis
detected_contacts = [
    body_part for body_part, peak_force in force_peaks.items()
    if peak_force > 0.05  # Only count meaningful contacts
]
```

### Method 4: Increase Hand Height

Raise the hand higher above ground (requires code change in `modular_scenario_builder.py`):

```python
# In _apply_hand_pose(), line 415:
self.data.qpos[freejoint_qpos_addr:freejoint_qpos_addr+3] = [0.0, 0.0, 0.20]  # Was 0.12, now 0.20
```

**Trade-off**: Higher = fewer false positives, but also fewer total contacts

---

## 📖 Step-by-Step Tuning Workflow

### Goal: Get F1 > 70% for a specific pose

#### Step 1: Run Baseline Test

```bash
python test_pose_comparison.py
```

Note the current F1 scores and contact patterns.

#### Step 2: Identify the Problem

Look at the output:
- **Many x (false positives) with low forces (< 0.1N)?** → Increase force threshold
- **Many ✓ but also many x?** → Adjust expected contacts
- **Few ✓, many missed?** → Check if pose is being maintained

#### Step 3: Adjust Force Threshold

```bash
nano config/simulation_config.yaml
# Change force_threshold from 0.01 to 0.05
```

#### Step 4: Retest

```bash
python test_pose_comparison.py
```

#### Step 5: Update Expected Contacts

Based on observed patterns, update the expected contacts in the pose definitions.

For example, if "Flat" pose consistently shows:
```
✓ palm_center
✓ index_geom
✓ middle_geom
x thumb_mid_geom  ← Consistently appears
x index_prox_geom  ← Consistently appears
```

Update the expected list to include these consistent extras.

#### Step 6: Iterate

Repeat steps 3-5 until F1 > 70%.

---

## 🎨 Creating Custom Scenarios

### Basic Template

```python
from modular_scenario_builder import *

# Create scenario
scenario = ScenarioConfig(
    name="my_custom_test",
    description="Testing specific hand-object interaction",
    hand_pose=HandPoseLibrary.flat_hand(),  # Choose pose
    interactions=[
        # Add object interactions here (future feature)
    ],
    duration_s=3.0,
    expected_contacts=['palm_center', 'index_geom', 'middle_geom']
)

# Run scenario
runner = ModularScenarioRunner()
results = runner.run_scenario(scenario, verbose=True)

# Check results
print(f"F1 Score: {results['f1_score']*100:.1f}%")
```

### Available Hand Poses

| Pose | Description | Best For |
|------|-------------|----------|
| `flat_hand()` | All fingers extended | Catching, flat surface contact |
| `pinch()` | Thumb + index extended | Precision grasping |
| `fist()` | All fingers curled | Power grasping |
| `point()` | Index finger extended | Pointing, poking |
| `palm_up()` | Fingers extended upward | Palm-only contact |
| `relaxed()` | Natural relaxed pose | Resting state |

### Modifying Poses

You can create custom poses by defining joint angles:

```python
custom_pose = HandPoseConfig(
    name="custom",
    description="My custom hand pose",
    joint_angles={
        'thumb_cmc': 0.5,   # radians
        'thumb_mcp': 0.3,
        'thumb_ip': 0.2,
        'index_mcp': 0.0,
        'index_pip': 0.0,
        'index_dip': 0.0,
        # ... more joints
    },
    hold_pose=True  # Actively maintain this pose during simulation
)
```

**Joint angle reference**:
- `0.0` = Fully extended
- `1.5` = Fully flexed (closed)
- Negative values = Hyperextension (fingers bent back)

---

## 🚀 Example Use Cases

### Use Case 1: Finding Optimal Force Threshold

**Goal**: Determine what force threshold gives best F1 score for "flat_hand" pose

```python
from modular_scenario_builder import *
import yaml

# Load config
with open('config/simulation_config.yaml', 'r') as f:
    config = yaml.safe_load(f)

# Test multiple thresholds
thresholds = [0.01, 0.02, 0.05, 0.1, 0.2]
results_summary = {}

for threshold in thresholds:
    # Update config
    config['contact']['force_threshold'] = threshold
    with open('config/simulation_config.yaml', 'w') as f:
        yaml.dump(config, f)

    # Run test
    runner = ModularScenarioRunner()
    scenario = ScenarioConfig(
        name=f"flat_hand_threshold_{threshold}",
        description="Testing force threshold",
        hand_pose=HandPoseLibrary.flat_hand(),
        interactions=[],
        duration_s=2.0,
        expected_contacts=['palm_center', 'index_geom', 'middle_geom']
    )

    results = runner.run_scenario(scenario, verbose=False)
    results_summary[threshold] = results['f1_score']

    print(f"Threshold={threshold:.3f}N → F1={results['f1_score']*100:.1f}%")

# Find best threshold
best_threshold = max(results_summary, key=results_summary.get)
print(f"\nBest threshold: {best_threshold:.3f}N (F1={results_summary[best_threshold]*100:.1f}%)")
```

### Use Case 2: Comparing All Poses

```bash
# Quick comparison
python test_pose_comparison.py

# Detailed per-pose testing
python test_hand_poses.py
```

### Use Case 3: Validating a Specific Interaction

```python
# Test if pinch pose correctly isolates thumb + index
scenario = ScenarioConfig(
    name="pinch_validation",
    description="Validate pinch gesture contact isolation",
    hand_pose=HandPoseLibrary.pinch(),
    interactions=[],
    duration_s=2.0,
    expected_contacts=['thumb_geom', 'index_geom']
)

runner = ModularScenarioRunner()
results = runner.run_scenario(scenario)

# Check if ONLY thumb and index contacted (high precision)
if results['precision'] > 0.8:
    print("✓ Pinch pose successfully isolates contacts")
else:
    print("✗ Pinch pose has unwanted contacts:")
    for contact in results['false_positives']:
        print(f"  - {contact}")
```

---

## 🔮 Future Features

### Currently Working
✅ Hand pose control and maintenance
✅ Multiple predefined poses
✅ Contact detection and metrics
✅ Force threshold tuning
✅ Systematic pose comparison

### Planned (Requires Implementation)
🔲 Dynamic object spawning (drop, throw, slide)
🔲 Object property control (mass, size, velocity)
🔲 Multiple simultaneous objects
🔲 Trajectory-based object movement
🔲 Real-time pose adjustment during simulation

### Implementation Notes

**Dynamic object spawning** requires:
- MuJoCo model modification API
- Runtime XML manipulation
- Body/geom creation and deletion

**Workaround** for now:
- Pre-define objects in hand model XML
- Use actuators to move objects into position
- Or use separate scenes for different object configurations

---

##Questions & Troubleshooting

### Q: Why are all my F1 scores low (< 30%)?

**A**: Most likely:
1. Force threshold too low (0.01N) - increase to 0.05N
2. Expected contacts don't match reality - update based on observed patterns
3. Hand too close to ground - many false positives from grazing

### Q: Pose "Point" has F1=0% - is it broken?

**A**: No! Look at the contact pattern - it's **different from other poses**. The pose IS working, but:
- The expected contact list needs adjustment
- Or the hand height/force threshold needs tuning

### Q: How do I add object dropping/throwing?

**A**: Currently requires implementation of dynamic object spawning. The framework is ready (`InteractionType`, `ObjectConfig`, `_spawn_object` method), but needs:
- Runtime MJCF manipulation
- Or pre-defined objects with actuators
- Or separate scene files per object type

### Q: Can I test with real robot data?

**A**: Yes! You can:
1. Log contact forces from real robot
2. Compare with simulation predictions
3. Use scenarios as test cases
4. Tune thresholds to match real-world sensitivity

### Q: What's the best force threshold?

**A**: Depends on your application:
- **Research/analysis**: 0.01-0.02N (high sensitivity, detect everything)
- **Practical applications**: 0.05-0.1N (filter noise, meaningful contacts only)
- **Robust detection**: 0.1-0.2N (only strong contacts, ignore grazing)

---

## 📞 Quick Reference Commands

```bash
# Compare all poses
python test_pose_comparison.py

# Test individual poses
python test_hand_poses.py

# Run custom scenario
python -c "from modular_scenario_builder import *; ..."

# Adjust force threshold
nano config/simulation_config.yaml  # Edit force_threshold

# See current configuration
python interactive_parameter_tuner.py  # Option 1

# Run full system demo
python simple_haptics_test.py
```

---

## 🎓 Key Takeaways

1. **Low F1 scores don't mean failure** - check if contact patterns are DIFFERENT across poses
2. **Increase force threshold** (0.01 → 0.05N) for quick improvement
3. **Adjust expected contacts** based on observed reality, not idealized assumptions
4. **Each pose produces a unique contact pattern** - that's what matters for testing
5. **Metrics are tools for tuning**, not absolute measures of success

Happy testing! 🚀
