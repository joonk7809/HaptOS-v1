
# Two-Phase Haptic Synthesis via Neural Learning

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.0+-ee4c2c.svg)](https://pytorch.org/)

A machine learning framework that generates realistic haptic feedback for virtual and extended reality applications by combining physics-based simulation with expert perceptual tuning.

**Key Innovation:** Two-phase training architecture achieving **89.1% improvement** over physics-based baseline by learning psychophysical principles like Weber's law through delta refinement.

![System Architecture](docs/diagrams/system_architecture.png)

---

## Problem Statement

Creating realistic haptic feedback requires mapping physical contact events to perceptually-accurate tactile sensations. Traditional approaches face limitations:

- **Pure physics-based**: Accurate but perceptually unrealistic (humans don't perceive forces linearly)
- **Pure data-driven**: Requires massive labeled datasets from human experts
- **Hybrid approaches**: No systematic way to combine physics accuracy with perceptual tuning

This project solves these challenges with a novel two-phase learning architecture.

---

## Quick Start

### Prerequisites

```bash
# Python 3.8 or higher
python --version

# Required libraries
pip install mujoco torch numpy scipy matplotlib h5py
```

### Installation

```bash
# Clone repository
git clone https://github.com/joonk7809/haptic-synthesis.git
cd haptic-synthesis

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### Run Demo

```bash
# Generate baseline dataset (Phase 1)
python tests/test_batch_generation.py

# Train baseline model (NN_v0)
python src/training/train_v0.py

# Generate expert tunings (Phase 2)
python tests/test_synthetic_expert.py

# Train refinement model (NN_v1)
python src/training/train_v1.py

# Validate combined system
python tests/test_combined_system.py
```

**Expected output:** `89.1% average improvement` over baseline

---

## Results

### Performance Metrics

| Parameter | NN_v0 Error | NN_v0+NN_v1 Error | Improvement |
|-----------|-------------|-------------------|-------------|
| **Impact Amplitude** | 0.0657 | 0.0155 | **76.4%** ✓ |
| **Weight Amplitude** | 0.1831 | 0.0125 | **93.1%** ✓ |
| **Fall Time** | 4.34 ms | 0.10 ms | **97.7%** ✓ |
| **Average** | — | — | **89.1%** ✓ |

![Results Comparison](docs/diagrams/results_comparison.png)

### Key Achievements

- **Weber's Law**: Successfully learned logarithmic force perception
- **Temporal Tuning**: 97.7% improvement in impact timing
- **Phase Detection**: Automatic contact phase classification
- **Scalable**: Trained on procedurally-generated dataset
- **Real-time**: 100Hz inference on CPU

---

## Architecture

### Two-Phase Learning Pipeline

```
Phase 1: Physics-Driven Baseline
─────────────────────────────────
MuJoCo (1kHz) ──┐
                ├──> Converter (100Hz) ──> NN_v0 ──> Baseline Cues
Audio (48kHz) ──┘

Phase 2: Expert Refinement
─────────────────────────────────
Features ──> NN_v1 ──> Delta Corrections

Runtime Inference
─────────────────────────────────
Features ──> [NN_v0 + NN_v1] ──> Final Cues (89.1% better!)
```

![Data Flow](docs/diagrams/data_flow.png)

### Phase 1: Procedural Baseline (NN_v0)

**Goal:** Learn physics-to-haptics mappings using procedural audio as training signal

**Pipeline:**
1. **Physics Simulation** (MuJoCo @ 1kHz): Extract contact forces, slip, material properties
2. **Audio Synthesis** (48kHz): Modal resonance, texture noise, contact-gated synthesis
3. **Converter**: Generate training pairs `(FeatureVec, CueParams)` from audio analysis
4. **NN_v0 Training**: Multi-head MLP predicts 5 haptic cue types

**Why audio?** Audio provides perceptually-meaningful training signal at scale without expensive human labeling.

### Phase 2: Expert Refinement (NN_v1)

**Goal:** Learn perceptual corrections through delta training

**Pipeline:**
1. **Expert Tuning**: Human (or synthetic) expert adjusts NN_v0 predictions
2. **Delta Dataset**: Compute `Δ = Expert_Gold - NN_v0_Baseline`
3. **NN_v1 Training**: Learn to predict deltas (can be negative!)
4. **Combined Inference**: `Final = clamp(NN_v0(X) + NN_v1(X))`

**Why deltas?**
- Preserves physics grounding
- Stable training (small corrections easier to learn)
- Efficient (trains on small expert dataset)
- Interpretable (can inspect what changed)

---

## Neural Network Architecture

![Network Architecture](docs/diagrams/network_architecture.png)

### Model Specifications

**Input:** 9-dimensional feature vector
- Phase (IMPACT/HOLD/SLIP/RELEASE)
- Phase confidence
- Normal force, shear force, slip speed
- Material: hardness, friction coefficient, roughness
- Uncertainty estimate

**Architecture:**
```
Shared Trunk:
  Linear(9 → 128) → ReLU → Dropout(0.2)
  Linear(128 → 128) → ReLU → Dropout(0.2)
  Linear(128 → 64) → ReLU

Specialized Heads:
  Impact  → [A, rise_ms, fall_ms, hf_weight]
  Weight  → [A, rate_ms]
  Ring    → [f_Hz×3, tau_ms×3, a×3]
  Shear   → [A, band_Hz]
  Texture → [A, color_class]
```

**Output:** 5 haptic cue types (21 total parameters)

**Key Design Choice:**
- `NN_v0`: Sigmoid/ReLU outputs (0-1 range, positive only)
- `NN_v1`: **Linear outputs** (can be negative for delta corrections)

**Parameters:** 
- NN_v0: ~27K
- NN_v1: ~27K  
- Combined: ~54K

---

## Project Structure

```
haptic_training/
├── assets/
│   └── contact_scene.xml           # MuJoCo simulation scene
│
├── src/
│   ├── physics/
│   │   └── mujoco_engine.py        # 1kHz contact extraction
│   │
│   ├── audio/
│   │   └── synthesizer.py          # 48kHz audio generation
│   │
│   ├── converter/
│   │   ├── feature_extractor.py    # Physics → FeatureVec (100Hz)
│   │   └── audio_analyzer.py       # Audio → CueParams labels
│   │
│   ├── models/
│   │   └── nn_v0.py                # NN_v0 & NN_v1_Delta architectures
│   │
│   ├── training/
│   │   ├── train_v0.py             # Phase 1 training script
│   │   └── train_v1.py             # Phase 2 training script
│   │
│   ├── inference/
│   │   ├── predictor.py            # Single model inference
│   │   ├── validator.py            # Performance evaluation
│   │   └── combined_predictor.py   # NN_v0 + NN_v1 runtime
│   │
│   ├── data_generator/
│   │   ├── scenario_generator.py   # Multi-scenario creation
│   │   ├── batch_generator.py      # Data pipeline orchestration
│   │   └── phase2_dataset.py       # Delta dataset management
│   │
│   └── tuning/
│       ├── synthetic_expert.py     # Rule-based expert (psychophysics)
│       └── cli_tuner.py            # Manual tuning interface
│
├── tests/                          # Validation & testing
│   ├── test_batch_generation.py
│   ├── test_synthetic_expert.py
│   └── test_combined_system.py
│
├── data/                           # Generated datasets (HDF5)
├── models/checkpoints/             # Trained weights (.pt)
├── logs/                           # Training curves, plots
└── docs/                           # Documentation & diagrams
```

---

## Technical Details

### Phase Detection FSM

Automatic contact phase classification:
```
NO_CONTACT → IMPACT → HOLD ⇄ SLIP → RELEASE → NO_CONTACT
```

**Transitions:**
- `NO_CONTACT → IMPACT`: Normal force > threshold & increasing
- `IMPACT → HOLD`: Stable force plateau
- `HOLD ⇄ SLIP`: Slip speed threshold crossing
- `HOLD/SLIP → RELEASE`: Force dropping

### Haptic Cue Types

1. **Impact**: Sharp transient at contact onset
   - Amplitude, rise time, fall time, high-frequency weight

2. **Weight**: Sustained pressure sensation
   - Amplitude (Weber's law!), rate of change

3. **Ring**: Modal resonance (material properties)
   - 3 harmonic modes: frequency, decay, amplitude

4. **Shear**: Lateral friction rumble
   - Amplitude, frequency band

5. **Texture**: Surface roughness noise
   - Amplitude, perceptual color (crispy/muffled/rough)

### Synthetic Expert Rules

Encodes known psychophysical principles:

**Universal:**
- **Weber's Law**: `perceived_weight = log10(force + 1) / log10(max_force)`
- Texture amplification based on roughness
- Material hardness → color mapping

**Phase-Specific:**
- **IMPACT**: Sharper rise for hard materials, longer decay for soft
- **HOLD**: Reduce lingering impact cues, boost texture
- **SLIP**: Amplify shear, dynamic friction texture
- **RELEASE**: Quick fade on all cues

---

## Training Details

### Phase 1 (NN_v0)

```python
Dataset:      1000+ samples (procedurally generated)
Loss:         MAE (L1)
Optimizer:    Adam (lr=1e-3)
Batch Size:   32
Epochs:       50
Time:         ~3 minutes (M1 Mac CPU)
Convergence:  1.2 → 0.4 MAE
```

### Phase 2 (NN_v1)

```python
Dataset:      500 expert tunings (synthetic)
Loss:         Weighted MAE (3x on Weber's law)
Optimizer:    Adam (lr=1e-3)
Batch Size:   16
Epochs:       30
Time:         ~1 minute
Convergence:  0.93 → 0.48 MAE (no overfitting)
```

### Hardware Requirements

- **Minimum**: CPU only (M-series Mac, modern Intel/AMD)
- **Recommended**: GPU optional (training time already fast)
- **Memory**: 4GB RAM sufficient
- **Storage**: ~500MB for datasets + models

---

## 🎓 Research Contributions

### Novel Techniques

1. **Procedural Audio as Perceptual Proxy**
   - First use of synchronized audio synthesis as training signal for haptics
   - Enables large-scale dataset generation without human labeling
   - Bridges physics and perception automatically

2. **Two-Phase Delta Learning Architecture**
   - Separates physics grounding (Phase 1) from perceptual tuning (Phase 2)
   - Delta training on small expert dataset (500 samples vs. typical 10K+)
   - 89.1% improvement demonstrates effectiveness

3. **Synthetic Expert Validation**
   - Rule-based expert encoding psychophysical laws (Weber's law, temporal tuning)
   - Proves pipeline end-to-end without requiring real hardware
   - Reproducible, consistent tunings for research

### Practical Impact

- **VR/AR Applications**: Real-time haptic rendering for virtual environments
- **Teleoperation**: Enhanced force feedback for remote manipulation
- **Medical Training**: Realistic surgical simulation
- **Gaming**: Advanced tactile effects

### Limitations & Future Work

**Current Limitations:**
- Synthetic expert not validated on human subjects
- Limited scenario diversity (mostly impacts, few sliding events)
- Single-device assumption (not tested for transfer)
- No embedded deployment optimization

**Planned Improvements:**
- [ ] Human subject validation study
- [ ] Expand scenario coverage (sliding, textures, multi-object)
- [ ] Multi-device transfer learning
- [ ] Active learning for efficient expert querying
- [ ] Real-time hardware integration (1kHz loop)
- [ ] Model quantization for embedded deployment

---

## Usage Examples

### Basic Inference

```python
from inference.combined_predictor import CombinedPredictor

# Load trained models
predictor = CombinedPredictor(
    nn_v0_path="models/checkpoints/nn_v0_best.pt",
    nn_v1_path="models/checkpoints/nn_v1_best.pt"
)

# Input features
feature_vec = {
    'phase': 'PHASE_HOLD',
    'phase_confidence_01': 0.85,
    'normal_force_N': 5.0,
    'shear_force_N': 0.0,
    'slip_speed_mms': 0.0,
    'material_features': {
        'hardness_01': 0.7,
        'mu_01': 0.5,
        'roughness_rms_um': 5.0
    },
    'uncertainty_pct': 0.0
}

# Predict haptic cues
cues = predictor.predict(feature_vec)

print(f"Impact amplitude: {cues['impact']['A']:.3f}")
print(f"Weight amplitude: {cues['weight']['A']:.3f}")
print(f"Fall time: {cues['impact']['fall_ms']:.1f} ms")
```

### Component Analysis

```python
# Get baseline, delta, and final predictions separately
baseline, delta, final = predictor.predict(
    feature_vec, 
    return_components=True
)

print(f"Baseline weight: {baseline['weight']['A']:.3f}")
print(f"Delta correction: {delta['weight']['A']:.3f}")
print(f"Final weight: {final['weight']['A']:.3f}")

# This shows how NN_v1 corrects NN_v0's predictions
```

### Batch Processing

```python
import numpy as np
from data_generator.phase1_dataset import DatasetLoader

# Load validation set
dataset = DatasetLoader("data/phase1_processed/validation.h5")

# Predict all samples
all_predictions = []
for i in range(len(dataset.X)):
    feature_vec = dataset.get_feature_dict(i)
    cues = predictor.predict(feature_vec)
    all_predictions.append(cues)

# Compute metrics
errors = compute_mae(all_predictions, dataset.Y)
print(f"Validation MAE: {errors['total']:.4f}")
```

---

## Reproducibility

### Running Experiments

```bash
# Full pipeline (from scratch)
./scripts/run_full_pipeline.sh

# Or step-by-step:

# 1. Generate Phase 1 data
python tests/test_batch_generation.py

# 2. Train baseline
python src/training/train_v0.py

# 3. Validate baseline
python tests/test_inference.py

# 4. Generate expert tunings
python tests/test_synthetic_expert.py

# 5. Train refinement
python src/training/train_v1.py

# 6. Final validation
python tests/test_combined_system.py
```

### Expected Results

Training should produce:
- Phase 1 convergence: ~0.4 MAE
- Phase 2 convergence: ~0.48 MAE (delta training)
- Combined improvement: 85-92% (may vary with random seed)
- Training time: <10 minutes total on modern CPU

### Checkpoints

Pre-trained models available in `models/checkpoints/`:
- `nn_v0_best.pt` - Baseline model (Phase 1)
- `nn_v1_best.pt` - Refinement model (Phase 2)

To use pre-trained models, skip training steps 2 and 5 above.

---

## Contributing

Contributions welcome! Areas of interest:

- [ ] Real human expert data collection
- [ ] Additional scenario types (scratching, tapping, rolling)
- [ ] Hardware integration (haptic gloves, controllers)
- [ ] Alternative audio synthesis methods
- [ ] Transfer learning across devices
- [ ] Mobile/embedded optimization

### Development Setup

```bash
# Install development dependencies
pip install -r requirements-dev.txt

# Run tests
pytest tests/

# Check code style
flake8 src/

# Format code
black src/ tests/
```

---

## Citation

If you use this work in your research, please cite:

```bibtex
@software{haptic_synthesis_2025,
  title={Two-Phase Haptic Synthesis via Neural Learning},
  author={Joon K},
  year={2025},
  url={https://github.com/joonk7809/haptic-synthesis}
}
```

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- **MuJoCo**: Physics simulation engine
- **PyTorch**: Deep learning framework
- **SciPy**: Signal processing utilities
- Psychophysics research: Weber's law, JND principles
- Modal synthesis: Van den Doel & Pai (1998)

---

## Contact

- **Author**: [Joon K]
- **Email**: joonkim.cyw@gmail.com
- **Website**: ycarus.dev
- **LinkedIn**: 

---

## Related Projects

- [Haptic Rendering Survey](link)
- [Modal Sound Synthesis](link)
- [Perceptual ML for Robotics](link)

---

## Star History

If you find this project useful, please consider giving it a star! ⭐

---

**Built with a mission for realistic haptic experiences**

*Last updated: December 2025*
```

---

Now create `requirements.txt`:

```txt
# Core dependencies
mujoco>=3.0.0
torch>=2.0.0
numpy>=1.24.0
scipy>=1.10.0
matplotlib>=3.7.0
h5py>=3.8.0

# Optional: for development
pytest>=7.3.0
flake8>=6.0.0
black>=23.0.0
```

And `requirements-dev.txt`:

```txt
-r requirements.txt

# Development tools
pytest>=7.3.0
pytest-cov>=4.0.0
flake8>=6.0.0
black>=23.0.0
mypy>=1.0.0
ipython>=8.10.0
jupyter>=1.0.0
```

---

## Quick Setup Script

Create `scripts/setup.sh`:

```bash
#!/bin/bash

echo "================================"
echo "Haptic Synthesis Setup"
echo "================================"

# Check Python version
python_version=$(python --version 2>&1 | awk '{print $2}')
echo "Python version: $python_version"

# Create virtual environment
echo "Creating virtual environment..."
python -m venv venv

# Activate virtual environment
echo "Activating virtual environment..."
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
    source venv/Scripts/activate
else
    source venv/bin/activate
fi

# Install dependencies
echo "Installing dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

# Create necessary directories
echo "Creating directories..."
mkdir -p data/phase1_processed
mkdir -p data/phase2_processed
mkdir -p data/phase2_tunings
mkdir -p models/checkpoints
mkdir -p logs/training
mkdir -p logs/validation
mkdir -p logs/phase2_validation
mkdir -p docs/diagrams

echo ""
echo "================================"
echo "Setup complete!"
echo "================================"
echo ""
echo "To activate the environment:"
echo "  source venv/bin/activate  (Linux/Mac)"
echo "  venv\\Scripts\\activate    (Windows)"
echo ""
echo "To run the full pipeline:"
echo "  ./scripts/run_full_pipeline.sh"
echo ""
```

Make it executable:
```bash
chmod +x scripts/setup.sh
```

---
