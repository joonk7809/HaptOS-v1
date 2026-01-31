# HaptOS File Structure

**Last Updated**: January 2026

This document describes the organization of the HaptOS codebase.

---

## Directory Structure

```
haptOS/
├── README.md                    # Project overview and quick start
├── ARCHITECTURE.md              # Complete system architecture documentation
│
├── docs/                        # Documentation
│   ├── PHASE1_PROGRESS.md      # Phase 1 completion report
│   ├── SUMMARY.md              # Project summary
│   ├── WHATS_NEW.md            # Recent changes and updates
│   ├── project_overview.md     # Project overview
│   ├── technical_details.md    # Technical implementation details
│   │
│   ├── guides/                 # User guides
│   │   ├── START_HERE.md       # Quick start guide
│   │   ├── QUICK_START.md      # Alternative quick start
│   │   ├── MODULAR_SCENARIOS_GUIDE.md  # Scenario testing guide
│   │   ├── MODULAR_TESTING_GUIDE.md    # Testing framework guide
│   │   ├── TESTING_SYSTEM_README.md    # Testing system documentation
│   │   └── README_INTERACTIVE.md       # Interactive demo guide
│   │
│   ├── setup/                  # Setup instructions
│   │   ├── MACOS_SETUP.md      # macOS-specific setup
│   │   ├── INTERACTIVE_SETUP.md # Interactive demo setup
│   │   └── TKINTER_FIX.md      # Tkinter troubleshooting
│   │
│   ├── diagrams/               # Architecture diagrams
│   │   ├── system_architecture.png
│   │   ├── network_architecture.png
│   │   ├── data_flow.png
│   │   ├── results_comparison.png
│   │   └── generate_diagrams.py
│   │
│   └── demo/                   # Demo assets
│       └── interactive_demo.html
│
├── src/                        # Source code
│   ├── __init__.py
│   │
│   ├── core/                   # Core HAPTOS Platform schemas
│   │   ├── __init__.py
│   │   └── schemas.py          # ContactPatch, FilteredContact, CueParams
│   │
│   ├── routing/                # Somatotopic routing (Layer 1)
│   │   ├── __init__.py
│   │   └── somatotopic_router.py  # Homunculus-based filtering
│   │
│   ├── physics/                # Physics simulation (Layer 1)
│   │   ├── __init__.py
│   │   ├── mujoco_engine.py    # Base MuJoCo wrapper
│   │   └── multi_contact_engine.py  # Multi-contact tracker
│   │
│   ├── converter/              # Feature extraction
│   │   ├── __init__.py
│   │   └── multi_contact_feature_extractor.py  # 13D features + FSM
│   │
│   ├── inference/              # Neural inference (Layer 2)
│   │   ├── __init__.py
│   │   └── multi_contact_predictor.py  # NN_v0 + NN_v1
│   │
│   ├── audio/                  # Audio synthesis
│   │   ├── __init__.py
│   │   └── multi_contact_synthesizer.py  # Procedural haptic synthesis
│   │
│   ├── scenarios/              # Scenario testing framework
│   │   ├── __init__.py
│   │   └── modular_scenario_builder.py  # Hand poses + scenarios
│   │
│   ├── gui/                    # GUI components
│   │   ├── __init__.py
│   │   └── (interactive demo components)
│   │
│   ├── visualization/          # Visualization tools
│   │   ├── __init__.py
│   │   └── (contact visualization)
│   │
│   ├── models/                 # Neural network architectures
│   │   ├── __init__.py
│   │   ├── nn_v0.py           # Baseline network
│   │   └── nn_v1.py           # Refinement network
│   │
│   └── sync_multi_runner.py   # Multi-contact synchronous runner
│
├── scripts/                    # Executable scripts
│   ├── demos/                  # Demo scripts
│   │   ├── demo_metal_bar.py          # Metal bar haptic demo
│   │   ├── demo_modular_system.py     # Modular system demo
│   │   ├── interactive_haptics_demo.py # Interactive haptic demo
│   │   └── simple_haptics_test.py     # Simple haptic test
│   │
│   └── tools/                  # Utility tools
│       ├── interactive_parameter_tuner.py  # Parameter tuning UI
│       ├── visualize_contacts.py           # Contact visualization
│       └── test_with_mjpython.sh           # MuJoCo viewer wrapper
│
├── tests/                      # Test suite
│   ├── phase1/                 # Phase 1 tests (HAPTOS Platform)
│   │   ├── test_schemas.py     # Data structure tests (12 tests)
│   │   ├── test_router.py      # Router logic tests (18 tests)
│   │   └── test_integration.py # Integration tests (7 tests)
│   │
│   └── integration/            # Integration and system tests
│       ├── test_scenarios.py          # Scenario testing
│       ├── test_pose_comparison.py    # Pose comparison tests
│       ├── test_hand_poses.py         # Hand pose validation
│       ├── test_hand_simulation.py    # Hand simulation tests
│       ├── test_hand_visual.py        # Visual tests (MuJoCo viewer)
│       ├── test_detailed_hand.py      # Detailed hand model tests
│       ├── test_multi_contact_engine.py  # Physics engine tests
│       ├── test_sync.py               # Sync runner tests
│       ├── test_audio.py              # Audio synthesis tests
│       ├── test_validation.py         # Validation tests
│       ├── test_run_diagnostic.py     # Diagnostic tests
│       ├── debug_nan.py               # NaN debugging
│       └── verify_noise.py            # Noise verification
│
├── assets/                     # Assets and resources
│   └── hand_models/            # MuJoCo hand models
│       ├── detailed_hand.xml   # 18-sensor hand model
│       └── simple_hand.xml     # 6-sensor hand model
│
├── config/                     # Configuration files
│   └── simulation_config.yaml  # Physics, neural, synthesis settings
│
├── models/                     # Trained models
│   └── checkpoints/
│       ├── nn_v0_best.pt       # Baseline network weights
│       └── nn_v1_best.pt       # Refinement network weights
│
├── firmware/                   # Embedded firmware (Layer 3)
│   └── HaptosRuntime.ino       # Arduino/Teensy firmware (placeholder)
│
├── data/                       # Data files
│   └── test_results/           # Test output data
│
└── logs/                       # Log files
    └── contact_events_*.log    # Contact event logs

```

---

## Key File Locations

### Documentation
- **Main**: `README.md`, `ARCHITECTURE.md` (root level for GitHub visibility)
- **Guides**: `docs/guides/`
- **Setup**: `docs/setup/`

### Source Code
- **HAPTOS Platform Core**: `src/core/`, `src/routing/`
- **Legacy System**: `src/physics/`, `src/converter/`, `src/inference/`, `src/audio/`
- **Scenarios**: `src/scenarios/`

### Executable Scripts
- **Demos**: `scripts/demos/` - Try these to see the system in action
- **Tools**: `scripts/tools/` - Utilities for tuning and visualization

### Tests
- **Phase 1 Tests**: `tests/phase1/` - HAPTOS Platform Layer 1 tests
- **Integration Tests**: `tests/integration/` - Full system tests

---

## Running the System

### Quick Start
```bash
# See the demo
python scripts/demos/simple_haptics_test.py

# Run Phase 1 tests
pytest tests/phase1/ -v

# Interactive parameter tuning
python scripts/tools/interactive_parameter_tuner.py

# Visualize contacts
python scripts/tools/visualize_contacts.py
```

### Development Workflow
```bash
# 1. Make changes in src/
# 2. Run tests
pytest tests/phase1/ -v
pytest tests/integration/ -v

# 3. Try demos
python scripts/demos/demo_modular_system.py

# 4. Commit
git add .
git commit -m "Your changes"
```

---

## Import Conventions

### Importing from src/
```python
# Always import from src root
from src.core.schemas import ContactPatch, FilteredContact, CueParams
from src.routing.somatotopic_router import SomatotopicRouter, Homunculus
from src.physics.multi_contact_engine import MultiContactEngine
from src.scenarios.modular_scenario_builder import ScenarioConfig, HandPoseLibrary
```

### Running Scripts
```bash
# Scripts should be run from project root
cd /Users/joon/haptOS
python scripts/demos/simple_haptics_test.py

# Tests use pytest
pytest tests/phase1/test_schemas.py -v
```

---

## Adding New Files

### New Source Code
- Core schemas → `src/core/`
- Routing logic → `src/routing/`
- Physics → `src/physics/`
- Neural inference → `src/inference/`
- Scenarios → `src/scenarios/`

### New Tests
- HAPTOS Platform → `tests/phase1/`
- Integration/System → `tests/integration/`

### New Scripts
- Demos → `scripts/demos/`
- Tools → `scripts/tools/`

### New Documentation
- User guides → `docs/guides/`
- Setup instructions → `docs/setup/`
- Technical → `docs/`

---

## Git Ignore Patterns

The `.gitignore` includes:
- `__pycache__/`, `*.pyc` - Python bytecode
- `.DS_Store` - macOS system files
- `logs/*.log` - Log files
- `data/test_results/` - Test output
- `.claude/` - Claude Code session data

---

**Last Updated**: January 11, 2026
