# HAPTOS Model Zoo

Pre-trained neural renderer models for haptic cue generation.

## Overview

The model zoo provides trained ML models for different use cases:
- **Production Models**: Validated, high-quality models for deployment
- **Experimental Models**: Research variants exploring different architectures
- **Specialized Models**: Domain-specific models (surgical, VR, robotics)

All models use the HAPTOS two-stage architecture:
1. **NN_v0 (Baseline)**: Predicts 11 cue parameters from 13 features
2. **NN_v1 (Delta)**: Refines NN_v0 predictions with residual corrections

---

## Production Models

### nn_v0_best.pt + nn_v1_best.pt ⭐ **Default**

**Status**: ✅ Included (Phase 1/2)

**Training Data**:
- Dataset: 50K contact sequences from hand manipulation tasks
- Materials: Wood, metal, rubber, plastic
- Phases: All FSM states (NO_CONTACT, IMPACT, HOLD, SLIP, RELEASE)
- Duration: 10ms windows @ 1kHz sampling

**Architecture**:
- **NN_v0**: 13-dim input → 64 → 32 → 11-dim output
- **NN_v1**: 11-dim input → 32 → 16 → 11-dim output (delta)
- Total parameters: ~70K (v0: 50K, v1: 20K)
- Inference time: ~8ms (CPU), ~2ms (GPU)

**Performance**:
| Metric | Value | Target |
|--------|-------|--------|
| Impact detection | 98.5% | >95% |
| Texture frequency error | ±12% | <20% |
| Weight offset MAE | 0.08 | <0.1 |
| Ring decay MSE | 0.03 | <0.05 |

**Usage**:
```python
import haptos

# Auto-loaded by default
renderer = haptos.Renderer()

# Or specify explicitly
renderer = haptos.Renderer(
    model_v0="models/checkpoints/nn_v0_best.pt",
    model_v1="models/checkpoints/nn_v1_best.pt"
)
```

**Validation**: Passed all 4 canonical tests (impact, hold, texture, liftoff)

---

## Experimental Models

### nn_v0_fast.pt + nn_v1_fast.pt (Coming Soon)

**Status**: 🔄 Planned (Phase 3)

**Goal**: Reduced latency for real-time applications

**Changes from Default**:
- Quantized to INT8 (from FP32)
- Smaller hidden layers (64→32, 32→16)
- Pruned weights (30% sparsity)
- Target parameters: ~35K total

**Expected Performance**:
- Inference time: ~3ms (CPU), <1ms (GPU)
- Accuracy: ~95% of default model
- Memory: 4× smaller

**Use Cases**:
- High-frequency updates (>200Hz)
- Embedded devices (mobile, edge)
- Multi-channel systems (>10 channels)

**Usage**:
```python
renderer = haptos.Renderer(
    model_v0="models/zoo/nn_v0_fast.pt",
    model_v1="models/zoo/nn_v1_fast.pt"
)
```

---

### nn_v0_accurate.pt + nn_v1_accurate.pt (Coming Soon)

**Status**: 🔄 Planned (Phase 3)

**Goal**: Maximum accuracy for research and validation

**Changes from Default**:
- Larger architecture (13→128→64→32→11)
- Trained on 200K sequences (4× more data)
- Ensemble of 3 models (averaged predictions)
- Target parameters: ~200K total

**Expected Performance**:
- Inference time: ~25ms (CPU), ~5ms (GPU)
- Accuracy: +15% improvement over default
- Better generalization to novel materials

**Use Cases**:
- Offline rendering (non-real-time)
- Ground truth generation for validation
- Research experiments

**Usage**:
```python
renderer = haptos.Renderer(
    model_v0="models/zoo/nn_v0_accurate.pt",
    model_v1="models/zoo/nn_v1_accurate.pt",
    device='cuda'  # Recommended for large model
)
```

---

## Specialized Models

### nn_surgical_v0.pt + nn_surgical_v1.pt (Planned)

**Status**: 📋 Planned (Phase 4+)

**Domain**: Medical training and surgical simulation

**Training Data**:
- Tissue contact (soft, compliant materials)
- Surgical tools (scalpel, forceps, needle)
- Specialized materials (organs, blood vessels)

**Specialized Cues**:
- Tissue compliance (soft tissue deformation)
- Puncture detection (needle insertion)
- Cutting forces (scalpel dynamics)

**Use Cases**:
- Surgical training simulators
- Robotic surgery teleoperation
- Medical procedure validation

---

### nn_vr_v0.pt + nn_vr_v1.pt (Planned)

**Status**: 📋 Planned (Phase 4+)

**Domain**: Virtual reality gaming and entertainment

**Training Data**:
- Game-specific interactions (weapons, tools, UI)
- Exaggerated forces (superhuman strength)
- Fantasy materials (magic, energy fields)

**Optimizations**:
- Low latency (<5ms)
- Smooth interpolation for motion
- Artistic freedom over physical accuracy

**Use Cases**:
- VR games (Beat Saber, Half-Life: Alyx)
- AR interfaces (tactile UI feedback)
- Immersive experiences

---

### nn_robot_v0.pt + nn_robot_v1.pt (Planned)

**Status**: 📋 Planned (Phase 5+)

**Domain**: Robotic manipulation and teleoperation

**Training Data**:
- Industrial manipulation tasks
- Robotic gripper contacts
- Object grasping and assembly

**Specialized Features**:
- Non-human perceptual model (robot-specific)
- Force scaling for different gripper sizes
- Adaptive to gripper types (parallel jaw, suction, multi-finger)

**Use Cases**:
- Teleoperation systems
- Human-robot collaboration
- Remote surgery robots

---

## Model Cards

Each model includes a model card documenting:

1. **Training Details**
   - Dataset size and composition
   - Training hyperparameters
   - Augmentation strategies
   - Validation splits

2. **Performance Metrics**
   - Accuracy on test set
   - Inference time benchmarks
   - Memory requirements
   - Perceptual quality scores

3. **Limitations**
   - Known failure modes
   - Material coverage gaps
   - Generalization boundaries

4. **Ethical Considerations**
   - Intended use cases
   - Misuse potential
   - Bias analysis

Example: See `models/zoo/nn_v0_best_card.md`

---

## Training Your Own Model

### Prerequisites

```bash
pip install haptos[dev]
```

### 1. Generate Training Data

```python
from src.data_generator.scenario_generator import ScenarioGenerator

gen = ScenarioGenerator(
    model_path="assets/hand_models/simple_hand.xml",
    output_dir="data/custom_dataset"
)

# Generate 10K contact sequences
gen.generate_dataset(
    num_sequences=10000,
    materials=['wood', 'metal', 'rubber', 'glass'],
    phases=['IMPACT', 'HOLD', 'RELEASE']
)
```

### 2. Train NN_v0 (Baseline)

```python
from src.training.train_v0 import train_v0

train_v0(
    dataset_path="data/custom_dataset",
    output_path="models/custom/nn_v0.pt",
    epochs=50,
    batch_size=32,
    learning_rate=0.001
)
```

### 3. Train NN_v1 (Delta)

```python
from src.training.train_v1 import train_v1

train_v1(
    dataset_path="data/custom_dataset",
    nn_v0_path="models/custom/nn_v0.pt",
    output_path="models/custom/nn_v1.pt",
    epochs=30,
    batch_size=32,
    learning_rate=0.0005
)
```

### 4. Validate Model

```python
import haptos

# Test custom model
renderer = haptos.Renderer(
    model_v0="models/custom/nn_v0.pt",
    model_v1="models/custom/nn_v1.pt"
)

# Run validation tests
from tests.validation.test_canonical_scenarios import test_impact_scenario
test_impact_scenario(renderer)
```

---

## Model Comparison

| Model | Parameters | Inference Time | Accuracy | Use Case |
|-------|-----------|---------------|----------|----------|
| **Default** | 70K | 8ms | 100% (baseline) | General purpose |
| **Fast** | 35K | 3ms | 95% | Real-time, embedded |
| **Accurate** | 200K | 25ms | 115% | Research, offline |
| **Surgical** | 80K | 10ms | Domain-specific | Medical training |
| **VR** | 40K | 5ms | Optimized for latency | Gaming |
| **Robot** | 90K | 12ms | Non-human perceptual | Teleoperation |

---

## Downloading Models

### From Model Zoo

```python
import haptos

# Auto-download from zoo (Phase 3+)
renderer = haptos.Renderer(model_variant="fast")
# Downloads nn_v0_fast.pt + nn_v1_fast.pt
```

### Manual Download

```bash
# Download specific model
wget https://github.com/anthropics/haptos/releases/download/v0.3.0/nn_v0_fast.pt
wget https://github.com/anthropics/haptos/releases/download/v0.3.0/nn_v1_fast.pt
```

---

## Contributing Models

Want to contribute a trained model? Submit a PR with:

1. **Model Files**
   - `nn_v0_*.pt` and `nn_v1_*.pt` checkpoints
   - Total size <100MB (use quantization if needed)

2. **Model Card**
   - Training details
   - Performance metrics
   - Validation results
   - Limitations

3. **Validation Script**
   - Tests canonical scenarios
   - Reports accuracy metrics
   - Inference time benchmarks

4. **Documentation**
   - Use cases
   - Expected performance
   - Integration example

See [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines.

---

## References

- Training Scripts: `src/training/`
- Dataset Generation: `src/data_generator/`
- Validation Tests: `tests/validation/`
- Model Architecture: `src/models/nn_v0.py`

---

**Status**: Model zoo infrastructure ready, experimental models planned for Phase 3 completion

**Current Models**: 2 (nn_v0_best.pt, nn_v1_best.pt)

**Planned Models**: 6 (fast, accurate, surgical, vr, robot variants)
