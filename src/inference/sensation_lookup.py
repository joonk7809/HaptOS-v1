#!/usr/bin/env python3
"""
Sensation Lookup Table - Analytical Sensation Generation for Tier 3 Contacts.

Provides fast, non-neural sensation generation for low-sensitivity body parts
(torso, thighs, etc.). Saves ~7.5ms per contact compared to neural inference.

Tier 3 contacts only render IMPACT and PRESSURE channels, so we can use
simple analytical functions based on physics.
"""

import logging
from typing import Optional

from src.core.schemas import FilteredContact, SensationParams

logger = logging.getLogger(__name__)


class SensationLookupTable:
    """
    Precomputed sensation parameters for Tier 3 (ERM) body parts.

    Uses analytical functions instead of neural network inference.
    Suitable for body parts with:
        - Low spatial resolution (>30mm two-point discrimination)
        - rendering_tier = 3 (ERM actuators)
        - cue_mask limited to IMPACT | PRESSURE

    Performance: ~0.01ms per contact (vs ~7.5ms for neural inference)
    """

    def __init__(self):
        """Initialize lookup table with material property mappings."""
        # Material hint → impact sharpness mapping
        # Based on material elasticity and surface hardness
        self.material_sharpness = {
            0: 0.5,   # Unknown: medium
            1: 0.9,   # Metal: very sharp
            2: 0.7,   # Wood: medium-sharp
            3: 0.6,   # Plastic: medium
            4: 0.3,   # Rubber: soft
            5: 0.2,   # Fabric: very soft
        }

        # Force normalization constants
        self.impact_force_scale = 5.0      # [N] Force mapping to impact intensity = 1.0
        self.pressure_force_scale = 10.0   # [N] Force mapping to pressure = 1.0

        logger.info("SensationLookupTable initialized")

    def lookup(self, contact: FilteredContact) -> SensationParams:
        """
        Generate SensationParams from analytical lookup.

        No neural inference. Pure physics-based computation.
        Suitable for Tier 3 contacts (ERM body parts, low priority).

        Args:
            contact: Filtered contact from router

        Returns:
            SensationParams with IMPACT and PRESSURE channels populated

        Performance: ~0.01ms per contact
        """
        # Extract physics data
        force = contact.patch.force_normal
        material = contact.patch.material_hint

        # Create sensation (all channels initially zero)
        sensation = SensationParams(
            # Impact channel (transient)
            impact_intensity=self._compute_impact_intensity(force),
            impact_sharpness=self._estimate_sharpness(material),
            impact_trigger=False,  # Set by onset detector, not here

            # Resonance channel (disabled for Tier 3)
            resonance_intensity=0.0,
            resonance_brightness=0.0,
            resonance_sustain=0.0,

            # Texture channel (disabled for Tier 3)
            texture_roughness=0.0,
            texture_density=0.0,
            texture_depth=0.0,

            # Slip channel (disabled for Tier 3)
            slip_speed=0.0,
            slip_direction=(0.0, 0.0),
            slip_grip=0.5,  # Neutral grip

            # Pressure channel (sustained force)
            pressure_magnitude=self._compute_pressure_magnitude(force),
            pressure_spread=0.5,  # Default for large body parts

            # Metadata
            body_part_id=contact.patch.body_part_id,
            timestamp_us=contact.patch.timestamp_us
        )

        return sensation

    def _compute_impact_intensity(self, force: float) -> float:
        """
        Compute impact intensity from force magnitude.

        Linear scaling with saturation at impact_force_scale.

        Args:
            force: Normal contact force [N]

        Returns:
            Impact intensity [0, 1]
        """
        intensity = force / self.impact_force_scale
        return min(intensity, 1.0)

    def _compute_pressure_magnitude(self, force: float) -> float:
        """
        Compute pressure magnitude from force.

        Linear scaling with saturation at pressure_force_scale.

        Args:
            force: Normal contact force [N]

        Returns:
            Pressure magnitude [0, 1]
        """
        magnitude = force / self.pressure_force_scale
        return min(magnitude, 1.0)

    def _estimate_sharpness(self, material_hint: int) -> float:
        """
        Estimate impact sharpness from material type.

        Based on material hardness and elasticity.
        Hard materials (metal, wood) → sharp impacts
        Soft materials (rubber, fabric) → soft impacts

        Args:
            material_hint: Material ID (0=unknown, 1=metal, 2=wood, etc.)

        Returns:
            Impact sharpness [0, 1]
        """
        return self.material_sharpness.get(material_hint, 0.5)

    def set_force_scales(self, impact_scale: float, pressure_scale: float):
        """
        Configure force normalization scales.

        Allows calibration for different actuator sensitivities.

        Args:
            impact_scale: Force [N] mapping to impact_intensity = 1.0
            pressure_scale: Force [N] mapping to pressure_magnitude = 1.0
        """
        if impact_scale <= 0 or pressure_scale <= 0:
            raise ValueError("Force scales must be positive")

        self.impact_force_scale = impact_scale
        self.pressure_force_scale = pressure_scale

        logger.info(f"Updated force scales: "
                   f"impact={impact_scale}N, pressure={pressure_scale}N")
