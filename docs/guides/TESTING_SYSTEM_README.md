# Modular Testing System - Quick Reference

## 🎉 What's New

You now have a **complete modular testing framework** with parameter adjustment capabilities and an improved hand model!

### New Tools (5 files)

1. **`interactive_parameter_tuner.py`** - CLI parameter adjustment
2. **`test_scenarios.py`** - Automated scenario testing
3. **`visualize_contacts.py`** - 2D hand contact diagrams
4. **`config/simulation_config.yaml`** - Parameter configuration
5. **`assets/hand_models/detailed_hand.xml`** - Enhanced hand (18 sensors)

### Improvements

#### Better Hand Model
- **18 contact sensors** (vs 6 simple)
- **3 segments per finger** (proximal, middle, tip)
- **3 palm zones** (center, thumb base, base)
- **Capsule geometry** (better visual than spheres)
- **Realistic proportions**

#### Parameter Control
- Adjust physics (timestep, gravity)
- Tune contact detection (force/slip thresholds)
- Configure phase FSM (impact/hold/slip/release)
- Save/load configurations
- Test multiple parameter sets

## 🚀 Quick Start

### 1. Test New Hand Model

```bash
# Simple test (no viewer)
python test_hand_simulation.py
```

**What you'll see:**
- More detailed contact names (e.g., `thumb_middle`, `index_proximal`)
- 18 possible contact sensors instead of 6
- Better contact localization

### 2. Interactive Parameter Tuning

```bash
python interactive_parameter_tuner.py
```

**What you can do:**
- View all current parameters
- Adjust physics settings
- Tune contact detection thresholds
- Modify phase detection FSM
- Run simulations with new parameters
- Save configurations

**Example workflow:**
```
1. Choose option 3 (Adjust contact detection)
2. Set force threshold to 0.05N (higher = fewer weak contacts)
3. Choose option 5 (Run simulation)
4. Observe results
5. Choose option 7 (Save configuration)
```

### 3. Run Predefined Scenarios

```bash
python test_scenarios.py
```

**Available tests:**
- **Flat hand** - All fingers extended
- **Pinch** - Thumb + index
- **Grasp** - All fingers curled
- **Finger tap** - Index only
- **Palm pressure** - Palm only

**Each test provides:**
- Precision/Recall metrics
- F1 Score
- True/False positives
- Force peaks per contact

### 4. Visualize Contacts

```bash
python visualize_contacts.py
```

**Creates:**
- 2D hand diagram
- Contact points (size = force)
- Force labels
- Statistics (total, average)
- Exportable PNG images

## 📊 Hand Model Comparison

| Feature | Simple Hand | Detailed Hand |
|---------|-------------|---------------|
| **Contact Sensors** | 6 | 18 |
| **Palm zones** | 1 | 3 |
| **Finger segments** | Tips only | Proximal + Middle + Tip |
| **Visual quality** | Basic spheres | Realistic capsules |
| **Use case** | Quick tests | Detailed analysis |

### Contact Sensor Map

**Detailed Hand (18 sensors):**
```
Palm (3):
  ├─ palm_center
  ├─ palm_thumb_base
  └─ palm_base

Each Finger (3 sensors × 5 fingers = 15):
  ├─ {finger}_proximal  (base segment)
  ├─ {finger}_middle    (middle segment)
  └─ {finger}_tip       (fingertip)

Total: 18 sensors
```

## 🎛️ Parameter Tuning Tips

### Contact Detection

**Force Threshold** (N):
- `0.01` - Very sensitive (default)
- `0.05` - Moderate (recommended for noisy environments)
- `0.10` - Only strong contacts

**Slip Threshold** (mm/s):
- `3.0` - Sensitive to motion
- `5.0` - Moderate (default)
- `10.0` - Only fast motion

### Phase Detection

**Impact Threshold** (N):
- `0.3` - Light taps trigger IMPACT
- `0.5` - Moderate (default)
- `0.8` - Only hard impacts

**Hold Time** (ms):
- `10` - Quick transition to HOLD
- `20` - Moderate (default)
- `50` - Sustained contact before HOLD

## 📈 Scenario Testing Workflow

### 1. Baseline Test
```bash
python test_scenarios.py
# Choose option 0 (Run all)
# Note F1 scores for each scenario
```

### 2. Adjust Parameters
```bash
python interactive_parameter_tuner.py
# Tune contact detection
# Save configuration
```

### 3. Retest
```bash
python test_scenarios.py
# Run same scenarios
# Compare F1 scores
```

### 4. Visualize
```bash
python visualize_contacts.py
# Create diagrams
# Document results
```

## 🔬 Advanced Usage

### Custom Scenarios

Edit `test_scenarios.py` to add your own:

```python
@staticmethod
def my_custom_test():
    return TestScenario(
        name="custom_test",
        description="My test description",
        hand_model="assets/hand_models/detailed_hand.xml",
        duration_s=5.0,
        hand_pose={
            'thumb_cmc': 0.5,
            'index_mcp': 0.8,
            # ... more joints
        },
        expected_contacts=['thumb_tip', 'index_tip']
    )
```

### Batch Parameter Testing

```python
# Test multiple thresholds
for threshold in [0.01, 0.03, 0.05, 0.1]:
    # Update config
    config['contact']['force_threshold'] = threshold

    # Run scenarios
    runner = ScenarioRunner()
    results = runner.run_all_scenarios()

    # Compare F1 scores
    print(f"Threshold {threshold}: F1 = {results['avg_f1']}")
```

### Configuration Files

**Edit directly:**
```bash
nano config/simulation_config.yaml
```

**Or use tuner:**
```bash
python interactive_parameter_tuner.py
# Adjust parameters via menu
# Save when done
```

## 📁 File Organization

```
haptOS/
├── config/
│   └── simulation_config.yaml     (NEW - Parameters)
├── assets/hand_models/
│   ├── simple_hand.xml            (Original - 6 sensors)
│   └── detailed_hand.xml          (NEW - 18 sensors)
├── interactive_parameter_tuner.py (NEW - Tuning tool)
├── test_scenarios.py              (NEW - Scenario tests)
├── visualize_contacts.py          (NEW - Visualization)
├── data/
│   ├── test_results/              (Scenario results)
│   └── visualizations/            (Contact diagrams)
└── MODULAR_TESTING_GUIDE.md       (Complete documentation)
```

## 🎯 Use Cases

### Research & Development
- Compare parameter configurations systematically
- Document contact patterns with visualizations
- Generate metrics (precision/recall) for publications
- Test edge cases with custom scenarios

### Tuning & Optimization
- Find optimal thresholds for your application
- Balance sensitivity vs. noise
- Validate FSM state transitions
- Verify contact detection accuracy

### Debugging
- Visualize which contacts are detected
- Check if expected contacts are missing
- Identify false positives
- Understand force distribution

### Documentation
- Export contact diagrams for reports
- Generate test results (JSON)
- Track parameter changes
- Compare configurations

## 🐛 Troubleshooting

### Issue: "yaml module not found"
```bash
pip install pyyaml
```

### Issue: No contacts detected with detailed hand
- Check force threshold (may need lower for detailed hand)
- Verify hand is falling (not stuck in air)
- Check spawn height in model

### Issue: Too many contacts
- Increase force threshold to 0.05N or higher
- Check for collision artifacts
- Verify geom sizes are reasonable

### Issue: Visualization shows "Unknown body part"
- Check geom names in hand model match pattern
- Update `_extract_body_part()` in multi_contact_engine.py
- Ensure contact sensor names follow convention

## 📚 Documentation

- **MODULAR_TESTING_GUIDE.md** - Complete guide (detailed)
- **TESTING_SYSTEM_README.md** - This file (quick reference)
- **MACOS_SETUP.md** - macOS-specific instructions
- **README_INTERACTIVE.md** - Interactive system overview

## ✅ Testing Checklist

- [ ] Test detailed hand model: `python test_hand_simulation.py`
- [ ] Run parameter tuner: `python interactive_parameter_tuner.py`
- [ ] Test all scenarios: `python test_scenarios.py` (option 0)
- [ ] Create visualization: `python visualize_contacts.py`
- [ ] Adjust parameters and retest
- [ ] Save optimal configuration
- [ ] Document results

## 🎓 Learning Path

1. **Start simple**: Run `test_hand_simulation.py` with detailed hand
2. **Explore parameters**: Use interactive tuner to adjust one parameter
3. **Run scenarios**: Test predefined scenarios, note F1 scores
4. **Visualize**: Create contact diagrams to understand patterns
5. **Iterate**: Adjust parameters, retest, compare results
6. **Advanced**: Create custom scenarios for your specific needs

## 💡 Pro Tips

1. **Save configurations** - When you find good parameters, save them!
2. **Document changes** - Note what parameters you changed and why
3. **Use scenarios** - Systematic testing is better than random trials
4. **Visualize early** - Diagrams help understand what's happening
5. **Start with defaults** - Don't change too many things at once
6. **Compare metrics** - Use F1 scores to objectively evaluate changes

## 🚀 Next Steps

**Immediate:**
```bash
# 1. Test new hand model
python test_hand_simulation.py

# 2. Try interactive tuner
python interactive_parameter_tuner.py

# 3. Run a scenario test
python test_scenarios.py
```

**Advanced:**
```bash
# 1. Create custom scenario
# Edit test_scenarios.py

# 2. Batch test parameters
# Write script to test multiple configs

# 3. Integrate with your research
# Use APIs in your own code
```

Happy testing! 🎉
