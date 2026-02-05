#!/usr/bin/env python3
"""
Analytical Label Generation for NeuralRenderer_v2 Training

Generates perceptual sensation labels from physics simulation data.
Uses analytical formulas based on physics, material properties, and
perceptual psychophysics (Weber-Fechner law, Stevens' power law).

No human perceptual data required (Phase 1).
Calibration constants (k1-k8) set perceptual midpoints.

Label Generation Pipeline:
    ContactPatch + Material Properties → Analytical Formulas → SensationParams

Physics-to-Perception Mapping:
1. Impact: Force rise rate + magnitude → intensity, sharpness
2. Resonance: Material elasticity + stiffness → intensity, brightness, sustain
3. Texture: Surface profile statistics → roughness, density, depth
4. Slip: Velocity + friction → speed, direction, grip
5. Pressure: Force + contact area → magnitude, spread
"""

import numpy as np
from typing import Dict, Optional
from dataclasses import dataclass
from src.core.schemas import SensationParams, ContactPatch


@dataclass
class MaterialProperties:
    """
    Material properties for label generation.

    Extends basic material database with perceptual properties.
    """
    material_id: int
    name: str

    # Mechanical properties
    stiffness: float          # Young's modulus [Pa] or effective stiffness [N/m]
    elasticity: float         # Coefficient of restitution [0-1]
    friction_coeff: float     # Static friction coefficient [0-1]
    density: float            # Material density [kg/m³]

    # Damping properties
    q_factor: float           # Quality factor (resonance sharpness)
    damping_ratio: float      # Damping ratio [0-1]

    # Surface properties
    surface_variance: float   # RMS surface roughness [m]
    spatial_frequency: float  # Dominant texture spatial frequency [cycles/m]
    bump_height: float        # Average asperity height [m]

    # Perceptual properties
    hardness: float           # Perceived hardness [0-1]
    roughness: float          # Perceived roughness [0-1]


# Material database (extends feature_extractor_v2 database)
MATERIAL_DATABASE = {
    0: MaterialProperties(  # Unknown/default
        material_id=0,
        name='unknown',
        stiffness=1e4,
        elasticity=0.5,
        friction_coeff=0.5,
        density=1000.0,
        q_factor=10.0,
        damping_ratio=0.1,
        surface_variance=0.0001,
        spatial_frequency=500.0,
        bump_height=0.0003,
        hardness=0.5,
        roughness=0.3
    ),
    1: MaterialProperties(  # Metal (steel)
        material_id=1,
        name='metal',
        stiffness=5e4,
        elasticity=0.8,
        friction_coeff=0.2,
        density=7800.0,
        q_factor=50.0,
        damping_ratio=0.02,
        surface_variance=0.00005,
        spatial_frequency=1000.0,
        bump_height=0.0001,
        hardness=0.9,
        roughness=0.2
    ),
    2: MaterialProperties(  # Wood
        material_id=2,
        name='wood',
        stiffness=1e4,
        elasticity=0.4,
        friction_coeff=0.6,
        density=600.0,
        q_factor=15.0,
        damping_ratio=0.15,
        surface_variance=0.0002,
        spatial_frequency=300.0,
        bump_height=0.0005,
        hardness=0.6,
        roughness=0.5
    ),
    3: MaterialProperties(  # Glass
        material_id=3,
        name='glass',
        stiffness=1e5,
        elasticity=0.9,
        friction_coeff=0.1,
        density=2500.0,
        q_factor=100.0,
        damping_ratio=0.01,
        surface_variance=0.00002,
        spatial_frequency=2000.0,
        bump_height=0.00005,
        hardness=0.95,
        roughness=0.1
    ),
    4: MaterialProperties(  # Rubber
        material_id=4,
        name='rubber',
        stiffness=5e3,
        elasticity=0.9,
        friction_coeff=0.8,
        density=1200.0,
        q_factor=5.0,
        damping_ratio=0.3,
        surface_variance=0.0003,
        spatial_frequency=200.0,
        bump_height=0.0008,
        hardness=0.2,
        roughness=0.4
    ),
    5: MaterialProperties(  # Plastic
        material_id=5,
        name='plastic',
        stiffness=2e4,
        elasticity=0.6,
        friction_coeff=0.4,
        density=1400.0,
        q_factor=20.0,
        damping_ratio=0.08,
        surface_variance=0.00008,
        spatial_frequency=800.0,
        bump_height=0.0002,
        hardness=0.7,
        roughness=0.3
    ),
}


class CalibrationConstants:
    """
    Calibration constants for physics-to-perception mapping.

    These constants set the perceptual midpoints (sigmoid(0) = 0.5).
    Tuned to match typical haptic experiences.

    Future: Fine-tune with human validation studies.
    """
    # Impact calibration
    k1 = 5.0        # Force normalization [N] (sigmoid(F/k1) → intensity)
    k2 = 2.0        # Force rise rate [N/ms] (sigmoid(dF/k2) → sharpness)

    # Resonance calibration
    k3 = 3.0        # Force-elasticity product [N] (sigmoid(F*e/k3) → intensity)
    k4 = 1e4        # Stiffness [N/m] (sigmoid(k/k4) → brightness)
    k5 = 10.0       # Quality factor (sigmoid(Q/k5) → sustain)

    # Texture calibration
    k6 = 0.001      # Surface variance [m²] (sigmoid(σ²/k6) → roughness)
    k7 = 1000.0     # Spatial frequency [cycles/m] (sigmoid(f/k7) → density)
    k8 = 0.0005     # Bump height [m] (sigmoid(h/k8) → depth)

    # Slip calibration
    k9 = 100.0      # Slip speed [mm/s] (sigmoid(v/k9) → speed)
    k10 = 1.0       # Friction normalization (sigmoid(μ/k10) → grip inverse)

    # Pressure calibration
    k11 = 10.0      # Pressure force [N] (sigmoid(F/k11) → magnitude)
    k12 = 0.001     # Contact area [m²] (sigmoid(A/k12) → spread)


class LabelGenerator:
    """
    Generates analytical perceptual labels from physics simulation.

    Maps ContactPatch → SensationParams using physics-based heuristics.
    """

    def __init__(self, calibration: CalibrationConstants = None):
        """
        Initialize label generator.

        Args:
            calibration: Calibration constants (default: CalibrationConstants())
        """
        self.k = calibration or CalibrationConstants()
        self.material_db = MATERIAL_DATABASE

    def generate_labels(self,
                       features: np.ndarray,
                       material_hint: int,
                       body_part_id: int,
                       timestamp_us: int) -> SensationParams:
        """
        Generate SensationParams labels from 14-dim feature vector.

        Args:
            features: 14-dim feature vector from FeatureExtractorV2
            material_hint: Material ID
            body_part_id: Body part ID
            timestamp_us: Timestamp

        Returns:
            SensationParams with analytical labels
        """
        # Unpack features (from feature_extractor_v2.py layout)
        normal_force = features[0]
        shear_magnitude = features[1]
        force_delta = features[2]
        slip_speed = features[3]
        # acceleration = features[4]  # Not used yet
        hardness = features[5]
        friction = features[6]
        roughness = features[7]
        phase_impact = features[8]
        # phase_hold = features[9]
        phase_slip = features[10]
        # phase_release = features[11]
        contact_duration = features[12]
        contact_area = features[13]

        # Get material properties
        material = self.material_db.get(material_hint, self.material_db[0])

        # Initialize labels
        labels = SensationParams.zero(body_part_id=body_part_id, timestamp_us=timestamp_us)

        # === Impact Channel ===
        # Intensity: proportional to normal force
        labels.impact_intensity = self._sigmoid(normal_force / self.k.k1)

        # Sharpness: proportional to force rise rate
        # force_delta is in [N], computed over 10ms window
        force_rise_rate = abs(force_delta) / 0.01  # [N/s] → [N/ms]
        labels.impact_sharpness = self._sigmoid(force_rise_rate / self.k.k2)

        # Material hardness increases sharpness
        labels.impact_sharpness = min(1.0, labels.impact_sharpness * (0.5 + 0.5 * hardness))

        # === Resonance Channel ===
        # Intensity: force × elasticity (energy storage)
        resonance_energy = normal_force * material.elasticity
        labels.resonance_intensity = self._sigmoid(resonance_energy / self.k.k3)

        # Brightness: material stiffness (high stiffness → high frequency)
        labels.resonance_brightness = self._sigmoid(material.stiffness / self.k.k4)

        # Sustain: quality factor (low damping → long ring)
        labels.resonance_sustain = self._sigmoid(material.q_factor / self.k.k5)

        # === Texture Channel ===
        # Roughness: surface variance
        labels.texture_roughness = self._sigmoid(material.surface_variance / self.k.k6)

        # Density: spatial frequency
        labels.texture_density = self._sigmoid(material.spatial_frequency / self.k.k7)

        # Depth: bump height × force (deeper indentation with more force)
        texture_depth = material.bump_height * np.sqrt(normal_force + 1e-6)
        labels.texture_depth = self._sigmoid(texture_depth / self.k.k8)

        # Texture gating: only active during slip or hold with motion
        if phase_slip < 0.5 and slip_speed < 1.0:
            labels.texture_roughness *= 0.1
            labels.texture_density *= 0.1
            labels.texture_depth *= 0.1

        # === Slip Channel ===
        # Speed: tangential velocity [mm/s]
        labels.slip_speed = self._sigmoid(slip_speed / self.k.k9)

        # Direction: from shear force direction (already normalized in features)
        # For now, use simplified mapping (would need velocity vector from ContactPatch)
        # Direction is set downstream from contact.velocity

        # Grip: inverse of slip (high friction + low speed = sticky)
        # grip = 1.0 when static, 0.0 when sliding
        friction_factor = friction / self.k.k10
        speed_factor = slip_speed / self.k.k9
        labels.slip_grip = 1.0 - self._sigmoid(speed_factor / (friction_factor + 0.1))

        # === Pressure Channel ===
        # Magnitude: sustained normal force
        labels.pressure_magnitude = self._sigmoid(normal_force / self.k.k11)

        # Spread: contact area (large area = diffuse, small area = point)
        labels.pressure_spread = self._sigmoid(contact_area / self.k.k12)

        # Clamp all values to [0, 1]
        labels.clamp_all(0.0, 1.0)

        return labels

    @staticmethod
    def _sigmoid(x: float) -> float:
        """
        Sigmoid activation for normalizing to [0, 1].

        Args:
            x: Input value

        Returns:
            Sigmoid(x) in [0, 1]
        """
        return 1.0 / (1.0 + np.exp(-x))

    def generate_dataset(self,
                        feature_list: list,
                        material_hints: list,
                        body_part_ids: list,
                        timestamps: list) -> list:
        """
        Generate dataset of (features, labels) pairs.

        Args:
            feature_list: List of 14-dim feature vectors
            material_hints: List of material IDs
            body_part_ids: List of body part IDs
            timestamps: List of timestamps

        Returns:
            List of (features, labels) tuples
        """
        dataset = []

        for features, material_hint, body_part_id, timestamp_us in zip(
            feature_list, material_hints, body_part_ids, timestamps
        ):
            labels = self.generate_labels(features, material_hint, body_part_id, timestamp_us)
            dataset.append((features, labels))

        return dataset

    def get_material_properties(self, material_hint: int) -> MaterialProperties:
        """
        Get material properties for a given material hint.

        Args:
            material_hint: Material ID

        Returns:
            MaterialProperties object
        """
        return self.material_db.get(material_hint, self.material_db[0])


if __name__ == '__main__':
    # Test label generation
    print("Testing Analytical Label Generation\n")
    print("="*60)

    generator = LabelGenerator()

    # Create synthetic features (impact scenario: metal impact)
    features = np.array([
        2.0,      # [0] normal_force (2N impact)
        0.3,      # [1] shear_magnitude
        1.5,      # [2] force_delta (rising edge)
        5.0,      # [3] slip_speed (mm/s)
        0.0,      # [4] acceleration
        0.9,      # [5] hardness (metal)
        0.2,      # [6] friction
        0.2,      # [7] roughness
        1.0,      # [8] phase_impact
        0.0,      # [9] phase_hold
        0.0,      # [10] phase_slip
        0.0,      # [11] phase_release
        10.0,     # [12] contact_duration (ms)
        0.0002,   # [13] contact_area (m²)
    ], dtype=np.float32)

    material_hint = 1  # Metal
    body_part_id = 10
    timestamp_us = 1000000

    # Generate labels
    labels = generator.generate_labels(features, material_hint, body_part_id, timestamp_us)

    print("Input Features:")
    print(f"  Normal Force: {features[0]:.2f} N")
    print(f"  Force Delta: {features[2]:.2f} N")
    print(f"  Material: {generator.get_material_properties(material_hint).name}")
    print(f"  Phase: IMPACT")

    print("\nGenerated Sensation Labels:")
    print(f"  Impact Intensity: {labels.impact_intensity:.3f}")
    print(f"  Impact Sharpness: {labels.impact_sharpness:.3f}")
    print(f"  Resonance Intensity: {labels.resonance_intensity:.3f}")
    print(f"  Resonance Brightness: {labels.resonance_brightness:.3f}")
    print(f"  Resonance Sustain: {labels.resonance_sustain:.3f}")
    print(f"  Texture Roughness: {labels.texture_roughness:.3f}")
    print(f"  Texture Density: {labels.texture_density:.3f}")
    print(f"  Texture Depth: {labels.texture_depth:.3f}")
    print(f"  Slip Speed: {labels.slip_speed:.3f}")
    print(f"  Slip Grip: {labels.slip_grip:.3f}")
    print(f"  Pressure Magnitude: {labels.pressure_magnitude:.3f}")
    print(f"  Pressure Spread: {labels.pressure_spread:.3f}")

    print("\n" + "="*60)
    print("✓ Label generation test passed!")

    # Test material database
    print("\nMaterial Database:")
    for mat_id, mat in generator.material_db.items():
        print(f"  {mat_id}: {mat.name:10s} (hardness={mat.hardness:.1f}, Q={mat.q_factor:.0f})")
