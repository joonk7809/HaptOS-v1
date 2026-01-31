# macOS Setup Guide for HaptOS Interactive System

## Known macOS Issues

There are **two known compatibility issues** on macOS:

1. **Tkinter GUI Crash** - Python 3.13 + macOS Sequoia compatibility
2. **MuJoCo Viewer Requirement** - Requires `mjpython` instead of regular Python

## ✅ Working Solutions

### Option 1: Test Without Any Viewer (Simplest)

```bash
# Test the physics and multi-contact system
# No viewer, just console output and logging
python test_hand_simulation.py
```

**What this does:**
- ✅ Loads the hand model
- ✅ Runs 3-second physics simulation at 1kHz
- ✅ Detects contacts (palm, fingers)
- ✅ Shows contact forces in console
- ✅ No viewer needed, works with regular Python

**Expected Output:**
```
[0.0s] Active contacts: 0, Total force: 0.00N
[0.5s] Active contacts: 0, Total force: 0.00N
[1.2s] ✓ palm contact! Force: 2.345N
[1.5s] Active contacts: 1, Total force: 8.12N
[2.0s] Active contacts: 2, Total force: 9.45N
...
```

### Option 2: Use mjpython for MuJoCo Viewer

```bash
# Install MuJoCo (if not already)
pip install mujoco

# Use mjpython instead of python
mjpython test_hand_visual.py
```

**What is mjpython?**
- Special Python wrapper for MuJoCo on macOS
- Enables the interactive 3D viewer
- Automatically installed with `pip install mujoco`
- Located at: `~/.local/bin/mjpython` or in your Python's bin directory

**Finding mjpython:**
```bash
# Check if it's in PATH
which mjpython

# If not found, try:
python -m mujoco.viewer --help

# Or find it manually
find ~/. -name mjpython 2>/dev/null
```

### Option 3: Use the Helper Script

```bash
# Interactive menu to choose test
./test_with_mjpython.sh
```

This will ask you which test to run and use the correct Python wrapper.

## Testing Workflow for macOS

### Step 1: Test Physics (No Viewer)

```bash
# This always works, no special requirements
python test_hand_simulation.py
```

**Expected:** Console output showing contacts as hand falls

### Step 2: Test Multi-Contact System (No Viewer)

```bash
# Test if you have trained models
python simple_haptics_test.py
```

**Expected:** Contact events with force values and haptic predictions

### Step 3: Test with MuJoCo Viewer

```bash
# Use mjpython for 3D visualization
mjpython test_hand_visual.py
```

**Expected:** MuJoCo window opens showing hand falling onto floor

### Step 4: Full GUI (After Fixing Tkinter)

```bash
# First fix Tkinter (see below)
brew install python-tk@3.13

# Then run with mjpython for full system
mjpython interactive_haptics_demo.py
```

## Installation Steps for macOS

### 1. Install Dependencies

```bash
# Install MuJoCo
pip install mujoco

# Install other dependencies
pip install torch matplotlib scipy numpy
```

### 2. Fix Tkinter (Optional, for GUI)

```bash
# Option A: Install via Homebrew
brew install python-tk@3.13

# Option B: Use Python 3.11 instead
brew install python@3.11
python3.11 -m pip install mujoco torch matplotlib scipy numpy
```

### 3. Verify MuJoCo Installation

```bash
# Check MuJoCo is installed
python -c "import mujoco; print('MuJoCo version:', mujoco.__version__)"

# Check if mjpython is available
which mjpython || echo "mjpython not in PATH"
```

### 4. Test Hand Model

```bash
# Load hand model (no viewer)
python -c "import mujoco; m = mujoco.MjModel.from_xml_path('assets/hand_models/simple_hand.xml'); print('✓ Hand loaded')"
```

## Quick Start (3 Commands)

```bash
# 1. Install MuJoCo
pip install mujoco

# 2. Test simulation (no viewer)
python test_hand_simulation.py

# 3. View in 3D (with mjpython)
mjpython test_hand_visual.py
```

## Alternative: Programmatic Use

You don't need the GUI or viewer to use the multi-contact system:

```python
import sys
sys.path.append('src')

from physics.multi_contact_engine import MultiContactEngine

# Initialize
engine = MultiContactEngine("assets/hand_models/simple_hand.xml")

# Run simulation
for i in range(1000):  # 1 second @ 1kHz
    contacts = engine.step_multi()

    # Process contacts
    for body_part, contact in contacts.items():
        print(f"{body_part}: {contact.normal_force_N:.3f}N")
```

## What Works on macOS

### ✅ Without Any Special Setup
- Physics simulation (1kHz)
- Multi-contact detection
- Force extraction
- Contact logging
- Console output

### ✅ With mjpython
- MuJoCo 3D viewer
- Interactive camera controls
- Real-time visualization

### ✅ With Tkinter Fix
- Full GUI with sliders
- Control panel interface
- Real-time matplotlib plots
- Complete interactive system

### ❌ Known Not to Work
- Regular Python with MuJoCo viewer (requires mjpython)
- Tkinter GUI on Python 3.13 + macOS Sequoia (needs fix)

## Troubleshooting

### Issue: "mjpython not found"

**Solution 1:** Check if it's installed
```bash
pip show mujoco | grep Location
# Then look for mjpython in that location's bin folder
```

**Solution 2:** Use Python module syntax
```bash
python -m mujoco.viewer test_hand_visual.py
```

**Solution 3:** Find and add to PATH
```bash
find ~/Library -name mjpython 2>/dev/null
# Add the directory to your PATH
```

### Issue: "NSInvalidArgumentException" (Tkinter crash)

**Solution:** Use Python 3.11 or install python-tk via Homebrew
```bash
brew install python-tk@3.13
# Or use Python 3.11
brew install python@3.11
```

### Issue: "Model file not found"

**Solution:** Ensure you're in the haptOS directory
```bash
cd /Users/joon/haptOS
ls assets/hand_models/simple_hand.xml  # Should exist
```

### Issue: "Module not found"

**Solution:** Install dependencies
```bash
pip install mujoco torch matplotlib scipy numpy
```

## Recommended Testing Order

1. **First:** `python test_hand_simulation.py` (simplest, always works)
2. **Second:** `mjpython test_hand_visual.py` (if you want 3D view)
3. **Third:** `python simple_haptics_test.py` (if models are trained)
4. **Fourth:** Fix Tkinter and run `mjpython interactive_haptics_demo.py` (full system)

## Summary

**Right now, without any fixes:**
- ✅ Run `python test_hand_simulation.py` to see multi-contact detection
- ✅ Physics, inference, and logging all work

**With mjpython:**
- ✅ Run `mjpython test_hand_visual.py` for 3D visualization
- ✅ Interactive MuJoCo viewer works

**With Tkinter fix + mjpython:**
- ✅ Run `mjpython interactive_haptics_demo.py` for complete system
- ✅ Full GUI with sliders and real-time plots

Start with `python test_hand_simulation.py` - it's the simplest test and requires no special setup!
