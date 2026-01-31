# HaptOS - Quick Start Guide

## What You Now Have

A complete multi-contact haptic testing system with:
- ✅ **Detailed 16-sensor hand model** (3 segments per finger + palm)
- ✅ **Real-time multi-contact detection** and tracking
- ✅ **Parameter adjustment system** (CLI + YAML)
- ✅ **Automated scenario testing** with metrics
- ✅ **2D visualization** tools

---

## 🚀 Quick Commands

### 1. Test Basic System (Simple 6-sensor hand)
```bash
python test_hand_simulation.py
```

### 2. Test Detailed Hand (16 sensors with segment information)
```bash
python test_detailed_hand.py
```
**Output**: Shows which segment of each finger makes contact (proximal/middle/tip)

### 3. Run Real-Time Multi-Contact System
```bash
python simple_haptics_test.py
```
**Output**: Real-time contact detection, force tracking, console logging

### 4. Adjust Parameters Interactively
```bash
python interactive_parameter_tuner.py
```
**Menu options**:
- View current parameters
- Adjust contact detection (force/slip thresholds)
- Adjust phase detection (FSM timing)
- Run test simulations
- Save/load configurations

### 5. Edit Parameters Directly
```bash
nano config/simulation_config.yaml
```
**Key parameters to adjust**:
```yaml
contact:
  force_threshold: 0.01  # Lower = more sensitive
  slip_threshold: 5.0    # Lower = detect slip easier

phase_detection:
  impact_threshold: 0.5  # Force needed for IMPACT
  hold_time_ms: 20       # Time before HOLD phase
```

### 6. Run Predefined Scenarios
```bash
python test_scenarios.py
```
**Available tests**:
- Flat hand impact (all fingers + palm)
- Pinch pose (thumb + index)
- Grasp pose (all fingers curled)
- Finger tap (index only)
- Palm pressure (palm only)

**Outputs**: Precision, Recall, F1 Score, force peaks

### 7. Visualize Contacts
```bash
python visualize_contacts.py
```
**Creates**: 2D hand diagram with force-scaled contact points

### 8. See All Features
```bash
python demo_modular_system.py
```

---

## 📊 Understanding Contact Information

### Simple Hand (6 sensors)
```
Contact points:
- palm (1 sensor)
- thumb_tip, index_tip, middle_tip, ring_tip, pinky_tip (5 sensors)
```

### Detailed Hand (16 sensors)
```
Contact points per finger:
- thumb: thumb_prox_geom, thumb_mid_geom, thumb_geom (3 segments)
- index: index_prox_geom, index_mid_geom, index_geom (3 segments)
- middle: middle_prox_geom, middle_mid_geom, middle_geom (3 segments)
- ring: ring_prox_geom, ring_mid_geom, ring_geom (3 segments)
- pinky: pinky_prox_geom, pinky_mid_geom, pinky_geom (3 segments)
- palm: palm_center (1 zone)

Total: 16 sensors (167% more detail than simple hand)
```

---

## 🎛️ Parameter Tuning Tips

### Contact Detection

**Force Threshold** (`contact.force_threshold`):
- `0.001` - Ultra-sensitive (catches everything, including noise)
- `0.01` - Normal (default, good balance)
- `0.05` - Less sensitive (only stronger contacts)
- `0.1` - Very insensitive (only hard impacts)

**When to adjust**:
- Too many false contacts? → Increase threshold
- Missing light touches? → Decrease threshold

### Phase Detection

**Impact Threshold** (`phase_detection.impact_threshold`):
- Controls when IMPACT phase triggers
- Default: 0.5N
- Lower = more sensitive to impacts

**Hold Time** (`phase_detection.hold_time_ms`):
- Time before transitioning to HOLD phase
- Default: 20ms
- Increase for stricter "sustained contact" detection

---

## 🔧 Common Workflows

### Workflow 1: Testing Contact Detection

```bash
# 1. Run baseline test
python test_detailed_hand.py

# 2. Note how many contacts detected
# Example output: "13 unique contacts detected"

# 3. Adjust force threshold
nano config/simulation_config.yaml
# Change: force_threshold: 0.05 (from 0.01)

# 4. Run test again
python test_detailed_hand.py

# 5. Compare: fewer weak contacts, cleaner data
```

### Workflow 2: Interactive Parameter Exploration

```bash
# 1. Launch parameter tuner
python interactive_parameter_tuner.py

# 2. Choose option 3 (Adjust contact detection)
# 3. Set force threshold: 0.05
# 4. Choose option 5 (Run simulation)
# 5. Observe results
# 6. Choose option 7 (Save configuration)
```

### Workflow 3: Systematic Testing

```bash
# 1. Run predefined scenarios
python test_scenarios.py

# 2. Choose option 0 (Run all)
# 3. Note F1 scores for each scenario
# 4. Adjust parameters in config
# 5. Run scenarios again
# 6. Compare F1 scores to validate improvements
```

---

## 📈 Example Session

```bash
$ python test_detailed_hand.py
======================================================================
               DETAILED HAND MODEL TEST
          (18 Contact Sensors vs 6 Simple)
======================================================================

Loading detailed hand model...
✓ Hand model loaded with 16 contact sensors:

  Palm zones (1): palm_center
  Thumb segments (3): thumb_geom, thumb_mid_geom, thumb_prox_geom
  Index segments (3): index_geom, index_mid_geom, index_prox_geom
  ...

Running 3.0-second simulation...

[0.000s] ✓ thumb_mid_geom contact! Force: 154.181N
[0.000s] ✓ thumb_geom contact! Force: 10.082N
[0.014s] ✓ palm_center contact! Force: 28.341N
[0.014s] ✓ index_prox_geom contact! Force: 0.730N
[0.015s] ✓ index_mid_geom contact! Force: 0.294N
...

Contact Summary (13 unique contacts detected):
  thumb_mid_geom           : First contact at 0.000s
  thumb_geom               : First contact at 0.000s
  palm_center              : First contact at 0.014s
  index_prox_geom          : First contact at 0.014s
  ...

💡 Comparison:
  Simple hand:   6 contact sensors
  Detailed hand: 16 contact sensors
  Improvement:   10 additional sensors (167% more detail)
```

---

## 📁 File Structure

```
haptOS/
├── config/
│   └── simulation_config.yaml          # ← Edit parameters here
│
├── assets/hand_models/
│   ├── simple_hand.xml                 # 6 sensors (fast)
│   └── detailed_hand.xml               # 16 sensors (detailed)
│
├── test_hand_simulation.py             # Test simple hand
├── test_detailed_hand.py               # Test detailed hand
├── simple_haptics_test.py              # Full multi-contact system
├── interactive_parameter_tuner.py      # ← Adjust parameters
├── test_scenarios.py                   # Scenario testing
├── visualize_contacts.py               # 2D visualization
├── demo_modular_system.py              # Feature demo
│
├── QUICK_START.md                      # This file
├── TESTING_SYSTEM_README.md            # Quick reference
├── MODULAR_TESTING_GUIDE.md            # Detailed guide
└── SUMMARY.md                          # Feature summary
```

---

## 🎯 Next Steps

### Immediate (Try Now!)
```bash
# See the improved 16-sensor hand
python test_detailed_hand.py

# Explore parameter adjustment
python interactive_parameter_tuner.py

# Run full multi-contact system
python simple_haptics_test.py
```

### Advanced
- Create custom test scenarios in `test_scenarios.py`
- Batch test multiple parameter configurations
- Export contact visualizations for documentation
- Integrate with your own analysis pipelines

---

## 💡 Key Improvements Over Original

| Feature | Before | After |
|---------|--------|-------|
| Contact sensors | 6 | **16** |
| Segment detail | Tips only | **Proximal + Middle + Tip** |
| Parameter adjustment | Edit code | **YAML + Interactive CLI** |
| Testing | Manual | **Automated scenarios + metrics** |
| Visualization | Console only | **2D diagrams + PNG export** |

---

## ❓ Troubleshooting

**Issue**: "Too many contacts detected"
→ Increase `force_threshold` in config to 0.05 or 0.1

**Issue**: "Missing expected contacts"
→ Decrease `force_threshold` to 0.001 or 0.005

**Issue**: "yaml module not found"
→ Run: `pip install pyyaml`

**Issue**: "MuJoCo viewer error on macOS"
→ Use `mjpython` instead of `python`, or run tests without viewer

---

## 📚 Full Documentation

- **TESTING_SYSTEM_README.md** - Quick reference
- **MODULAR_TESTING_GUIDE.md** - Complete guide
- **MACOS_SETUP.md** - macOS troubleshooting
- **SUMMARY.md** - System overview

---

**Happy Testing! 🚀**
