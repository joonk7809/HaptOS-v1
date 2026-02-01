"""
Schema Adapters for Neural Renderer

Converts between NEW HAPTOS Platform schemas and legacy ML pipeline schemas.
Preserves existing trained models (NN_v0 + NN_v1) without retraining.

Key conversions:
- NEW ContactPatch → OLD ContactPatch (for feature extraction)
- Legacy CueParams dict → NEW CueParams dataclass (for output)
- Cue masking (selective rendering based on bitmask)
"""

from typing import Tuple, Dict, Optional
import math
import numpy as np
from dataclasses import dataclass

from src.core.schemas import ContactPatch, CueParams, FilteredContact


# ============================================================================
# Material Property Lookup Tables
# ============================================================================

MATERIAL_PROPERTIES = {
    0: {"mu": 0.5, "damping": 1.0, "name": "unknown"},      # Generic rigid
    1: {"mu": 0.3, "damping": 0.8, "name": "metal"},        # Metal
    2: {"mu": 0.6, "damping": 1.5, "name": "wood"},         # Wood
    3: {"mu": 0.8, "damping": 2.0, "name": "rubber"},       # Rubber/soft
    4: {"mu": 0.4, "damping": 0.6, "name": "plastic"},      # Plastic
    5: {"mu": 0.2, "damping": 0.5, "name": "glass"},        # Glass/smooth
}

MATERIAL_IMPACT_FREQ = {
    0: 150.0,  # Unknown/generic → mid frequency
    1: 200.0,  # Metal → high frequency (sharp impact)
    2: 100.0,  # Wood → medium-low (damped impact)
    3: 80.0,   # Rubber → low frequency (soft impact)
    4: 150.0,  # Plastic → medium
    5: 250.0,  # Glass → very high (sharp, brittle)
}

COLOR_TO_GRAIN_HZ = {
    "COLOR_WHITE": 80.0,    # Fine grain (smooth, high frequency)
    "COLOR_PINK": 150.0,    # Medium grain (moderate texture)
    "COLOR_BROWN": 250.0,   # Coarse grain (rough, lower frequency)
}


# ============================================================================
# OLD ContactPatch Schema (legacy ML pipeline)
# ============================================================================

@dataclass
class OldContactPatch:
    """Legacy ContactPatch format used by existing FeatureExtractor."""
    timestamp_us: int
    normal_force_N: float
    shear_force_N: float        # Scalar magnitude (not vector)
    slip_speed_mms: float
    contact_pos: np.ndarray     # [x, y, z]
    contact_normal: np.ndarray  # [nx, ny, nz]
    in_contact: bool
    mu_static: float
    mu_dynamic: float
    solref_damping: float


# ============================================================================
# Schema Conversion: NEW → OLD ContactPatch
# ============================================================================

def new_contact_to_old(contact: ContactPatch) -> OldContactPatch:
    """
    Convert NEW ContactPatch to OLD ContactPatch for feature extraction.

    Field Mappings:
    - force_normal: Direct copy
    - force_shear: Vector norm sqrt(fx² + fy²)
    - slip_speed_mms: Velocity vector magnitude * 1000
    - in_contact: Always True (guaranteed by router filtering)
    - contact_pos: Default [0, 0, 0] (not available in new schema)
    - contact_normal: Default [0, 0, 1] (not available)
    - mu_static: Derive from material_hint lookup table
    - mu_dynamic: Same as mu_static (Phase 1 simplification)
    - solref_damping: Derive from material_hint lookup table

    Args:
        contact: NEW ContactPatch from physics engine

    Returns:
        OldContactPatch compatible with legacy FeatureExtractor
    """
    # Shear force magnitude (vector norm)
    fx, fy = contact.force_shear
    shear_magnitude = math.sqrt(fx**2 + fy**2)

    # Slip speed from velocity (convert m/s to mm/s)
    vx, vy, vz = contact.velocity
    velocity_magnitude = math.sqrt(vx**2 + vy**2 + vz**2)
    slip_speed_mms = velocity_magnitude * 1000.0

    # Material properties from lookup table
    mat_props = MATERIAL_PROPERTIES.get(contact.material_hint, MATERIAL_PROPERTIES[0])

    return OldContactPatch(
        timestamp_us=contact.timestamp_us,
        normal_force_N=contact.force_normal,
        shear_force_N=shear_magnitude,
        slip_speed_mms=slip_speed_mms,
        contact_pos=np.array([0.0, 0.0, 0.0]),      # Default (not available)
        contact_normal=np.array([0.0, 0.0, 1.0]),   # Default (upward normal)
        in_contact=True,                             # Always true (filtered by router)
        mu_static=mat_props["mu"],
        mu_dynamic=mat_props["mu"],                  # Simplified (same as static)
        solref_damping=mat_props["damping"]
    )


# ============================================================================
# Schema Conversion: Legacy CueParams dict → NEW CueParams
# ============================================================================

def legacy_cues_to_new(
    legacy_dict: dict,
    contact: ContactPatch,
    timestamp_us: int,
    trigger_impulse: bool
) -> CueParams:
    """
    Convert legacy CueParams dict to NEW CueParams dataclass.

    Field Resolution Strategy:

    IMPACT:
    - impact_amplitude: legacy['impact']['A']
    - impact_decay_ms: legacy['impact']['fall_ms']
    - impact_frequency_hz: Derive from material_hint (not in legacy)

    RING:
    - ring_amplitude: legacy['ring']['a'][0] (mode 0 - strongest)
    - ring_decay_ms: legacy['ring']['tau_ms'][0]
    - Note: 3 ring modes → 1 dominant mode (select max amplitude)

    SHEAR:
    - shear_magnitude: legacy['shear']['A']
    - shear_direction: Extract from contact.force_shear (normalized)

    WEIGHT:
    - weight_offset: legacy['weight']['A']

    TEXTURE:
    - texture_amplitude: legacy['texture']['A']
    - texture_grain_hz: Derive from legacy['texture']['color']

    TRANSIENT:
    - trigger_impulse: Passed as argument (from FSM state tracker)

    Args:
        legacy_dict: Output from CombinedPredictor (nested dict format)
        contact: Original ContactPatch (for shear_direction extraction)
        timestamp_us: Current simulation timestamp
        trigger_impulse: Event flag from FSM state change detection

    Returns:
        NEW CueParams dataclass ready for hardware driver
    """
    # IMPACT parameters
    impact = legacy_dict.get('impact', {})
    impact_amplitude = float(impact.get('A', 0.0))
    impact_decay_ms = float(impact.get('fall_ms', 50.0))
    impact_frequency_hz = MATERIAL_IMPACT_FREQ.get(contact.material_hint, 150.0)

    # RING parameters (select dominant mode)
    ring = legacy_dict.get('ring', {})
    ring_amplitude, ring_decay_ms = select_dominant_ring_mode(ring)

    # SHEAR parameters
    shear = legacy_dict.get('shear', {})
    shear_magnitude = float(shear.get('A', 0.0))
    shear_direction = derive_shear_direction(contact)

    # WEIGHT parameters
    weight = legacy_dict.get('weight', {})
    weight_offset = float(weight.get('A', 0.0))

    # TEXTURE parameters
    texture = legacy_dict.get('texture', {})
    texture_amplitude = float(texture.get('A', 0.0))
    texture_color = texture.get('color', 'COLOR_PINK')
    texture_grain_hz = derive_texture_grain(texture_color)

    return CueParams(
        # Continuous cues
        texture_grain_hz=texture_grain_hz,
        texture_amplitude=texture_amplitude,
        shear_direction=shear_direction,
        shear_magnitude=shear_magnitude,
        weight_offset=weight_offset,

        # Transient cues
        impact_amplitude=impact_amplitude,
        impact_decay_ms=impact_decay_ms,
        impact_frequency_hz=impact_frequency_hz,
        ring_amplitude=ring_amplitude,
        ring_decay_ms=ring_decay_ms,
        trigger_impulse=trigger_impulse,

        timestamp_us=timestamp_us
    )


def select_dominant_ring_mode(ring_dict: dict) -> Tuple[float, float]:
    """
    Select strongest ring mode from 3 modes.

    Legacy output has 3 ring modes (f_Hz, tau_ms, a arrays).
    New schema has only 1 mode. Select dominant (max amplitude).

    Args:
        ring_dict: {'f_Hz': [f1, f2, f3], 'tau_ms': [t1, t2, t3], 'a': [a1, a2, a3]}

    Returns:
        (ring_amplitude, ring_decay_ms) - Dominant mode parameters
    """
    amplitudes = ring_dict.get('a', [0.0, 0.0, 0.0])
    tau_ms_array = ring_dict.get('tau_ms', [100.0, 100.0, 100.0])

    # Handle list or numpy array
    if isinstance(amplitudes, (list, np.ndarray)):
        if len(amplitudes) == 0:
            return 0.0, 100.0
        dominant_idx = int(np.argmax(amplitudes))
        return float(amplitudes[dominant_idx]), float(tau_ms_array[dominant_idx])
    else:
        # Scalar (shouldn't happen, but handle gracefully)
        return float(amplitudes), float(tau_ms_array[0] if isinstance(tau_ms_array, list) else tau_ms_array)


def derive_shear_direction(contact: ContactPatch) -> Tuple[float, float]:
    """
    Extract normalized shear direction from force vector.

    Args:
        contact: ContactPatch with force_shear tuple

    Returns:
        (dir_x, dir_y) - Normalized direction vector
    """
    fx, fy = contact.force_shear
    magnitude = math.sqrt(fx**2 + fy**2)

    if magnitude < 1e-6:
        # No shear force, return default direction
        return (1.0, 0.0)

    # Normalize
    return (fx / magnitude, fy / magnitude)


def derive_texture_grain(color: str) -> float:
    """
    Map legacy color classification to texture grain frequency.

    Args:
        color: Color classification string (COLOR_WHITE, COLOR_PINK, COLOR_BROWN)

    Returns:
        Texture grain frequency in Hz
    """
    return COLOR_TO_GRAIN_HZ.get(color, 150.0)  # Default: medium grain


# ============================================================================
# Cue Masking
# ============================================================================

def apply_cue_mask(cue_params: CueParams, cue_mask: int) -> CueParams:
    """
    Zero out disabled cue parameters based on cue_mask bitmask.

    Cue Mask Mapping:
    - CUE_IMPACT (0b00001): impact_amplitude, impact_decay_ms, impact_frequency_hz
    - CUE_RING (0b00010): ring_amplitude, ring_decay_ms
    - CUE_TEXTURE (0b00100): texture_grain_hz, texture_amplitude
    - CUE_SHEAR (0b01000): shear_direction, shear_magnitude
    - CUE_WEIGHT (0b10000): weight_offset

    Args:
        cue_params: Original CueParams from renderer
        cue_mask: Bitmask from FilteredContact (which cues are enabled)

    Returns:
        CueParams with disabled cues zeroed
    """
    # Copy all fields
    masked = CueParams(
        texture_grain_hz=cue_params.texture_grain_hz,
        texture_amplitude=cue_params.texture_amplitude,
        shear_direction=cue_params.shear_direction,
        shear_magnitude=cue_params.shear_magnitude,
        weight_offset=cue_params.weight_offset,
        impact_amplitude=cue_params.impact_amplitude,
        impact_decay_ms=cue_params.impact_decay_ms,
        impact_frequency_hz=cue_params.impact_frequency_hz,
        ring_amplitude=cue_params.ring_amplitude,
        ring_decay_ms=cue_params.ring_decay_ms,
        trigger_impulse=cue_params.trigger_impulse,
        timestamp_us=cue_params.timestamp_us
    )

    # Zero disabled cues
    if not (cue_mask & FilteredContact.CUE_IMPACT):
        masked.impact_amplitude = 0.0
        masked.impact_decay_ms = 0.0
        masked.impact_frequency_hz = 0.0
        masked.trigger_impulse = False  # Disable impulse if impact disabled

    if not (cue_mask & FilteredContact.CUE_RING):
        masked.ring_amplitude = 0.0
        masked.ring_decay_ms = 0.0

    if not (cue_mask & FilteredContact.CUE_TEXTURE):
        masked.texture_grain_hz = 0.0
        masked.texture_amplitude = 0.0

    if not (cue_mask & FilteredContact.CUE_SHEAR):
        masked.shear_direction = (0.0, 0.0)
        masked.shear_magnitude = 0.0

    if not (cue_mask & FilteredContact.CUE_WEIGHT):
        masked.weight_offset = 0.0

    return masked
