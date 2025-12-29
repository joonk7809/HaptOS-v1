# Perceptual Haptic Synthesis via Two-Phase Neural Learning

## Overview

A machine learning framework that generates realistic haptic feedback for virtual and extended reality applications by combining physics-based simulation with expert perceptual tuning.

**Key Innovation**: Two-phase training architecture where a baseline model learns physics-to-haptics mappings from procedural audio analysis, then a refinement model learns expert perceptual corrections through delta training.

**Result**: 89.1% improvement in prediction accuracy, successfully encoding psychophysical principles like Weber's law.

---

## Problem Statement

Creating realistic haptic feedback for virtual interactions requires mapping physical contact events to perceptually-accurate tactile sensations. Traditional approaches suffer from:

- **Pure physics**: Accurate but perceptually unrealistic (humans don't perceive forces linearly)
- **Pure data-driven**: Requires massive labeled datasets from human experts
- **Hybrid gap**: No systematic way to combine physics accuracy with perceptual tuning

---

## Solution Architecture

### Phase 1: Physics-Driven Baseline (NN_v0)

**Input**: Physical features (force, slip, material properties)  
**Training**: Procedural audio analysis as "perceptual proxy"  
**Output**: Baseline haptic cue parameters

- Physically grounded
- Scalable (automatic data generation)
- Perceptually imperfect

### Phase 2: Expert Refinement (NN_v1)

**Input**: Same physical features  
**Training**: Delta between NN_v0 and expert gold standards  
**Output**: Perceptual correction deltas

- Learns psychophysical laws (Weber's law, temporal tuning)
- Efficient (trains on small expert dataset)
- Stable (baseline + small delta)

### Combined System

Final_Cues = clamp(NN_v0(features) + NN_v1(features))
└─── baseline ───┘   └── refinement ──┘
## Results

### Quantitative Performance

| Parameter | NN_v0 Error | NN_v0+NN_v1 Error | Improvement |
|-----------|-------------|-------------------|-------------|
| **Impact Amplitude** | 0.0657 | 0.0155 | **76.4%** ✓ |
| **Weight Amplitude** | 0.1831 | 0.0125 | **93.1%** ✓ |
| **Fall Time** | 4.34 ms | 0.10 ms | **97.7%** ✓ |
| **Average** | - | - | **89.1%** ✓ |

### Key Achievements

- ✅ **Weber's Law**: Successfully learned logarithmic force perception
- ✅ **Temporal Tuning**: 97.7% improvement in impact timing
- ✅ **Phase Detection**: Automatic contact phase classification (impact/hold/slip/release)
- ✅ **Scalable**: Trained on procedurally-generated dataset (1000+ samples)
- ✅ **Efficient**: Real-time inference at 100Hz on CPU

---

## Technical Implementation

### Pipeline Components

**1. Physics Simulation** (MuJoCo @ 1kHz)
- Contact force extraction
- Material property tracking
- Multi-scenario generation

**2. Audio Synthesis** (48kHz, synchronized)
- Modal resonance modeling
- Texture noise generation
- Contact-gated synthesis

**3. Feature Extraction** (100Hz)
- 10ms sliding window
- Phase classification FSM
- Material property encoding

**4. Neural Architecture**
- Input: 9-dimensional feature vector
- Model: Multi-head MLP (128→128→64 trunk)
- Output: 5 haptic cue types (impact, ring, shear, weight, texture)
- Parameters: 84,509 total (NN_v0 + NN_v1)

**5. Training Strategy**
- Phase 1: 50 epochs, MAE loss, ~1000 samples
- Phase 2: 30 epochs, delta targets, ~500 expert tunings
- Validation: 20% holdout split

---

## Technologies Used

- **Physics**: MuJoCo (contact dynamics)
- **ML Framework**: PyTorch
- **Audio**: SciPy (signal processing)
- **Data**: HDF5 (efficient storage)
- **Visualization**: Matplotlib

---

## Impact & Applications

### Immediate Use Cases
- VR/AR haptic rendering
- Robot teleoperation feedback
- Medical simulation training
- Gaming tactile effects

### Research Contributions
- Novel delta learning architecture for perception
- Procedural audio as perceptual training signal
- Synthetic expert generation methodology
- Validation of psychophysics integration in ML

---

## Future Directions

- [ ] Real-time hardware deployment
- [ ] Sliding/friction scenario expansion
- [ ] Multi-expert consensus learning
- [ ] Active learning for efficient tuning
- [ ] Transfer to different haptic devices

---

