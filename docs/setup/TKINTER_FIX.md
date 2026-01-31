# Tkinter Compatibility Fix for macOS

## The Problem

You're encountering this error:
```
NSInvalidArgumentException: -[NSApplication macOSVersion]: unrecognized selector
```

This is a known compatibility issue between:
- Python 3.13
- The bundled Tk/Tcl version
- macOS Sequoia (15.x)

## Quick Solutions

### Option 1: Test Without GUI (Recommended First)

I've created two simple test scripts that don't require Tkinter:

#### 1. Visual Hand Test (Just MuJoCo Viewer)
```bash
python test_hand_visual.py
```
This will:
- Load the hand model
- Open MuJoCo's 3D viewer
- Let you watch the hand fall and interact with the floor
- No ML models needed, just visualizes physics

#### 2. Simple Multi-Contact Test (Command-Line)
```bash
python simple_haptics_test.py
```
This will:
- Run full multi-contact inference
- Log contact events to console
- Show forces and predictions as text
- Save detailed log file

### Option 2: Fix Tkinter (For Full GUI)

#### Solution A: Install via Homebrew
```bash
# Install Python with proper Tk support
brew install python-tk@3.13

# Or reinstall Python
brew reinstall python@3.13

# Verify
python3 -c "import tkinter; print('Tkinter version:', tkinter.TkVersion)"
```

#### Solution B: Use Different Python Version
```bash
# Python 3.11 has better Tk compatibility
brew install python@3.11

# Use it for the project
python3.11 interactive_haptics_demo.py
```

#### Solution C: Install from python.org
1. Download Python 3.11 from https://www.python.org/downloads/
2. This version includes compatible Tk
3. Use that Python for the project

### Option 3: Alternative GUI Framework (Future)

If Tkinter continues to cause issues, we can replace it with:
- **PyQt5/PyQt6** - More robust, better macOS support
- **Kivy** - Cross-platform, touch-friendly
- **Web UI** - Flask + HTML/JavaScript
- **Dear ImGui** - High-performance, immediate mode

## Testing the System Now

### Step 1: Install MuJoCo (If Not Already)
```bash
pip install mujoco
```

### Step 2: Test Hand Model Visualization
```bash
python test_hand_visual.py
```

**Expected**: MuJoCo viewer opens, showing a hand falling onto a floor.

### Step 3: Test Multi-Contact System
```bash
# First check if models exist
ls models/checkpoints/

# If models exist, run test
python simple_haptics_test.py
```

**Expected**: Console output showing contact events and predictions.

### Step 4: (Optional) Fix Tkinter and Run Full GUI
```bash
# Fix Tkinter first (see Option 2 above)

# Then run full demo
python interactive_haptics_demo.py
```

## What Works Without Fixing Tkinter

✅ **Physics simulation** - MuJoCo works fine
✅ **MuJoCo 3D viewer** - Native viewer works
✅ **Multi-contact detection** - All contact tracking works
✅ **Neural inference** - NN_v0 + NN_v1 predictions work
✅ **Matplotlib plots** - Can display in separate windows
✅ **Logging** - Full event logging works

❌ **Tkinter GUI** - Control panel doesn't work without fix

## Recommended Testing Flow

```bash
# 1. Test hand visualization (no ML needed)
python test_hand_visual.py

# 2. Test if models are trained
python tests/test_multi_contact_engine.py  # If MuJoCo installed

# 3. Test multi-contact with logging
python simple_haptics_test.py  # If models exist

# 4. (Later) Fix Tkinter and run full GUI
python interactive_haptics_demo.py
```

## Alternative: Run Without Any GUI

You can also use the multi-contact system programmatically:

```python
import sys
sys.path.append('src')

from sync_multi_runner import MultiContactSyncRunner

# Initialize
runner = MultiContactSyncRunner(
    model_path="assets/hand_models/simple_hand.xml",
    nn_v0_path="models/checkpoints/nn_v0_best.pt",
    nn_v1_path="models/checkpoints/nn_v1_best.pt"
)

# Run simulation
for i in range(100):
    result = runner.step()
    print(f"Step {i}: {len(result['contacts'])} contacts")
```

## Getting Help

If you continue having issues:
1. Check Python version: `python --version`
2. Check Tk version: `python -c "import tkinter; print(tkinter.TkVersion)"`
3. Check macOS version: `sw_vers`
4. Try the no-GUI tests first

## Summary

**Right now, without fixing Tkinter:**
- ✅ You can visualize the hand in MuJoCo viewer
- ✅ You can test multi-contact detection
- ✅ You can run inference and see predictions
- ✅ You can log all contact events

**After fixing Tkinter:**
- ✅ Full interactive GUI with sliders
- ✅ Real-time matplotlib plots
- ✅ Complete control panel interface

Start with the simple tests, then fix Tkinter for the full GUI experience!
