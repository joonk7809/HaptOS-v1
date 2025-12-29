# Technical Deep Dive: Two-Phase Haptic Synthesis

## System Architecture

### Data Flow
```
Physics Sim (1kHz)  ──┐
                      ├──> Converter (100Hz) ──> NN_v0 ──┐
Audio Synth (48kHz) ──┘                                  ├──> Final Cues
                                                          │
                      Feature Vector ────────> NN_v1 ────┘
```

---

## Phase 1: Procedural Baseline

### Data Generation Pipeline

**Step 1: Physics Simulation**
```python
# MuJoCo scene with configurable parameters
ContactPatch = {
    normal_force_N: float,      # Perpendicular contact force
    shear_force_N: float,       # Tangential friction force
    slip_speed_mms: float,      # Relative motion speed
    material_props: dict        # Hardness, friction, roughness
}
```

**Step 2: Audio Synthesis**
- **Impact**: Noise burst through resonant filters
- **Ring**: Modal synthesis (3 harmonic modes)
- **Texture**: Band-passed noise (4-12kHz)
- **Shear**: Low-frequency rumble (30-200Hz)

**Step 3: Converter (Procedural Teacher)**

Rules:
1. **Physics Backbone**: `weight.A = f(normal_force)`
2. **Audio Prior**: `impact.* = g(audio_envelope)`
3. **Spectral Analysis**: `ring.* = h(FFT_peaks)`

Output: `(FeatureVec, CueParams)` training pairs

### NN_v0 Architecture
```
Input (9D): [phase, confidence, force_n, force_s, slip, 
             hardness, mu, roughness, uncertainty]
             
Trunk:
  Linear(9 → 128) → ReLU → Dropout(0.2)
  Linear(128 → 128) → ReLU → Dropout(0.2)
  Linear(128 → 64) → ReLU

Heads (specialized per cue type):
  Impact Head  → [A, rise_ms, fall_ms, hf_weight]
  Weight Head  → [A, rate_ms]
  Ring Head    → [f_Hz×3, tau_ms×3, a×3]
  Shear Head   → [A, band_Hz]
  Texture Head → [A, color_class]
```

**Activation Functions**:
- Amplitudes: `sigmoid` (0-1 range)
- Timings: `relu + offset` (positive only)
- Frequencies: `relu + 100` (> 100Hz)

### Training Details

- **Loss**: MAE (L1) - robust to outliers
- **Optimizer**: Adam (lr=1e-3)
- **Batch Size**: 32
- **Epochs**: 50
- **Hardware**: CPU (M-series Mac)
- **Training Time**: ~5 minutes

---

## Phase 2: Expert Refinement

### Synthetic Expert Design

Since real haptic hardware was unavailable for validation, we implemented a rule-based "synthetic expert" encoding known psychophysical principles:

**Universal Rules** (all contact phases):
```python
# Weber's Law for force perception
perceived_weight = log10(force + 1) / log10(max_force)

# Texture amplification based on roughness
texture_factor = 1.0 + (roughness / 10) * 0.5

# Color from hardness (white=crispy, brown=muffled)
```

**Phase-Specific Rules**:
- **IMPACT**: Sharper rise for hard impacts, longer decay for soft materials
- **HOLD**: Reduce impact cues (avoid lingering), boost texture
- **SLIP**: Amplify shear, increase dynamic friction texture
- **RELEASE**: Quick fade on all cues

### Delta Dataset Generation
```python
# For each sample:
baseline = NN_v0(features)          # Get v0 prediction
gold = synthetic_expert.tune(baseline, features)  # Apply rules
delta = gold - baseline              # Compute difference

# Delta statistics (500 samples):
# - Impact A: mean=-0.062, range=[-0.159, 0.000]
# - Weight A: mean=+0.176, range=[0.000, 0.337]  ← Weber's law!
# - Fall time: mean=-4.26ms, range=[-6.44, 0.000]
```

### NN_v1 Architecture

**Critical Design Decision**: Linear outputs (no activation functions)
```python
# NN_v1_Delta differs from NN_v0:
class NN_v1_Delta(nn.Module):
    def forward(self, x):
        features = self.trunk(x)  # Same trunk
        
        # KEY: No sigmoid/relu on outputs!
        impact = self.impact_head(features)
        return {
            'impact': {
                'A': impact[:, 0],        # Can be NEGATIVE
                'fall_ms': impact[:, 2]   # Can be NEGATIVE
            },
            # ... other heads
        }
```

**Why?** Deltas can be negative (reduce amplitude, shorten timing). Sigmoid/ReLU force positive values → training instability → NaN loss.

### Training Details

- **Loss**: MAE on deltas (same as Phase 1)
- **Weighted**: 3x weight on Weber's law correction
- **Batch Size**: 16 (smaller dataset)
- **Epochs**: 30 (deltas train faster)
- **Convergence**: Loss 0.93 → 0.48 (stable, no overfitting)

---

## Combined Inference

### Runtime Logic
```python
def predict(features):
    # Get baseline
    baseline = NN_v0(features)
    
    # Get refinement
    delta = NN_v1(features)
    
    # Combine with clamping
    final = {
        'impact': {
            'A': clamp(baseline.A + delta.A, 0.0, 1.0),
            'fall_ms': clamp(baseline.fall_ms + delta.fall_ms, 2.0, 50.0)
        },
        # ... other cues
    }
    return final
```

### Performance Characteristics

- **Latency**: < 1ms per prediction (CPU)
- **Throughput**: > 10,000 predictions/sec
- **Memory**: ~350KB model weights
- **Accuracy**: 89.1% improvement over baseline

---

## Validation Methodology

### Metrics

**Mean Absolute Error (MAE)**: Primary metric
- Interpretable: same units as predictions
- Robust: less sensitive to outliers than MSE

**Improvement Calculation**:
```
improvement = (baseline_error - final_error) / baseline_error × 100%
```

### Test Protocol

1. Hold out 20% of synthetic expert tunings
2. Compare three systems:
   - NN_v0 alone (baseline)
   - NN_v0 + NN_v1 (combined)
   - Synthetic expert (gold standard)
3. Measure MAE on key parameters
4. Visualize scatter plots (predicted vs. gold)

### Statistical Significance

- **Sample size**: 50 test scenarios
- **Consistent improvement**: All parameters show gains
- **Large effect size**: 76-98% reduction in error

---

## Key Design Decisions

### 1. Why Audio as Training Signal?

**Problem**: Need large-scale labels for Phase 1, but human tuning is expensive.

**Solution**: Audio synthesis provides:
- Synchronized with physics (causal relationship)
- Perceptually meaningful (humans use audio to infer touch)
- Automatically generated at scale
- Contains timing/frequency info for cues

**Trade-off**: Not perfect (audio ≠ haptics), but good enough for baseline.

### 2. Why Delta Learning?

**Alternatives Considered**:
- Retrain NN_v0 with expert labels → Loses physics grounding
- Ensemble of models → Computationally expensive
- Fine-tune NN_v0 → Catastrophic forgetting

**Delta Advantages**:
- Preserves physics baseline
- Stable training (small deltas easier to learn)
- Interpretable (can inspect what expert changed)
- Efficient (trains on small dataset)

### 3. Why Synthetic Expert?

**Limitation**: No access to haptic hardware for validation.

**Justification**:
- Encodes known psychophysics (Weber's law)
- Generates consistent, reproducible tunings
- Proves pipeline works end-to-end
- Easy to replace with real expert later

**Future Work**: Validate with human subjects + real device.

---

## Ablation Studies

| Configuration | Impact MAE | Weight MAE | Fall MAE |
|---------------|------------|------------|----------|
| NN_v0 alone | 0.0657 | 0.1831 | 4.34 ms |
| + NN_v1 (unweighted loss) | 0.0234 | 0.0892 | 1.23 ms |
| + NN_v1 (weighted loss) | **0.0155** | **0.0125** | **0.10 ms** |

**Conclusion**: Weighted loss (3x on Weber's law) is critical for best performance.

---

## Limitations & Future Work

### Current Limitations

1. **Synthetic expert**: Not validated on real human preferences
2. **Limited scenarios**: Mostly impacts, few sliding/texture events
3. **Single device**: Not tested for transfer across haptic hardware
4. **CPU only**: Could be optimized for embedded deployment

### Planned Improvements

1. **Real expert data**: Conduct user studies with haptic devices
2. **Scenario expansion**: Add sliding, textures, varied materials
3. **Active learning**: Intelligently select scenarios needing tuning
4. **Hardware deployment**: Real-time integration at 1kHz
5. **Multi-device transfer**: Domain adaptation techniques

---

## Reproducibility

### Environment Setup
```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate

# Install dependencies
pip install mujoco torch numpy scipy matplotlib h5py

# Verify installation
python -c "import mujoco; print('MuJoCo version:', mujoco.__version__)"
```

### Running the Pipeline
```bash
# Phase 1: Generate baseline dataset
python tests/test_batch_generation.py

# Phase 1: Train NN_v0
python src/training/train_v0.py

# Phase 2: Generate expert tunings
python tests/test_synthetic_expert.py

# Phase 2: Train NN_v1
python src/training/train_v1.py

# Validation: Test combined system
python tests/test_combined_system.py
```

### Expected Results

Training times (M1 Mac):
- Phase 1 data generation: ~5 min
- NN_v0 training: ~3 min
- Phase 2 data generation: ~30 sec
- NN_v1 training: ~1 min

Final performance: 85-92% improvement (may vary with random seed)

---

## Code Structure
```
haptic_training/
├── assets/
│   └── contact_scene.xml       # MuJoCo simulation scene
├── src/
│   ├── physics/
│   │   └── mujoco_engine.py    # 1kHz contact extraction
│   ├── audio/
│   │   └── synthesizer.py      # 48kHz audio generation
│   ├── converter/
│   │   ├── feature_extractor.py    # Physics → FeatureVec
│   │   └── audio_analyzer.py       # Audio → CueParams
│   ├── models/
│   │   └── nn_v0.py            # NN_v0 & NN_v1_Delta architectures
│   ├── training/
│   │   ├── train_v0.py         # Phase 1 training
│   │   └── train_v1.py         # Phase 2 training
│   ├── inference/
│   │   ├── predictor.py        # Single model inference
│   │   ├── validator.py        # Performance metrics
│   │   └── combined_predictor.py   # NN_v0 + NN_v1
│   ├── data_generator/
│   │   ├── scenario_generator.py   # Multi-scenario creation
│   │   ├── batch_generator.py      # Data pipeline
│   │   └── phase2_dataset.py       # Delta dataset management
│   └── tuning/
│       ├── synthetic_expert.py     # Rule-based expert
│       └── cli_tuner.py            # Manual tuning interface
├── tests/                      # Validation & testing scripts
├── data/                       # Generated datasets (HDF5)
├── models/checkpoints/         # Trained model weights
└── logs/                       # Training curves, validation plots
```

---

## Citations & References

### Psychophysics
- Weber's Law: Weber, E. H. (1834). De pulsu, resorptione, auditu et tactu
- Just Noticeable Difference: Gescheider, G. A. (1997). Psychophysics

### Haptic Rendering
- Contact modeling: Salisbury & Srinivasan (1997)
- Modal synthesis: Van den Doel & Pai (1998)

### Machine Learning
- Delta learning: Howard & Ruder (2018). Universal Language Model Fine-tuning
- Expert-in-the-loop: Settles (2009). Active Learning Literature Survey

---

*Last updated: November 2025*