# Quick Navigation Guide

## 📚 Want to Learn?
- **Start Here**: `docs/guides/START_HERE.md` or `docs/guides/QUICK_START.md`
- **Full Architecture**: `ARCHITECTURE.md` (at root)
- **Testing Guide**: `docs/guides/TESTING_SYSTEM_README.md`
- **Platform Setup**: `docs/setup/MACOS_SETUP.md`

## 🎮 Want to Try Demos?
```bash
# Simple haptic test
python scripts/demos/simple_haptics_test.py

# Interactive demo
python scripts/demos/interactive_haptics_demo.py

# Modular system demo
python scripts/demos/demo_modular_system.py
```

## 🔧 Want to Tune Parameters?
```bash
# Interactive parameter tuner
python scripts/tools/interactive_parameter_tuner.py

# Visualize contacts
python scripts/tools/visualize_contacts.py
```

## 🧪 Want to Run Tests?
```bash
# Phase 1 (HAPTOS Platform Layer 1)
pytest tests/phase1/ -v

# Integration tests
pytest tests/integration/ -v

# Specific test
python tests/integration/test_scenarios.py
```

## 💻 Want to Develop?
- **Source Code**: `src/` directory
  - Core schemas: `src/core/`
  - Router: `src/routing/`
  - Physics: `src/physics/`
  - Neural inference: `src/inference/`

- **Adding Features**: See `docs/FILE_STRUCTURE.md`

## 📖 Full Documentation
- `docs/guides/` - All user guides
- `docs/setup/` - Platform-specific setup
- `docs/PHASE1_PROGRESS.md` - Current development status
- `ARCHITECTURE.md` - Complete technical documentation

---

**Directory Structure**: See `docs/FILE_STRUCTURE.md` for complete file organization.
