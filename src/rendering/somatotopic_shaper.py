#!/usr/bin/env python3
"""
Somatotopic Shaping: Body-Part Adaptation for Haptic Sensations

Adapts body-agnostic SensationParams to specific body parts based on
biological perceptual properties (Homunculus table).

Transformations:
1. Amplitude Gain: Less sensitive body parts need stronger signals
2. Frequency Clamping: Brightness/density clamped to perceptible range
3. Spatial Resolution Filtering: Coarse body parts lose fine texture detail

Deterministic. No learned parameters. ~15 operations per sensation.

Example:
    # Same metal impact on different body parts
    fingertip: full detail (gain=1.0x, spatial_factor=1.0)
    palm: moderate detail (gain=2.0x, spatial_factor=0.5)
    torso: pure impact (gain=5.0x, spatial_factor=0.11, texture→0)

Usage:
    from src.rendering.somatotopic_shaper import shape
    from src.routing.somatotopic_router import Homunculus

    homunculus = Homunculus()
    body = homunculus.lookup_by_id(10)  # index_tip
    shaped = shape(sensation, body)
"""

from src.core.schemas import SensationParams
from src.routing.somatotopic_router import BodyPartProperties


# Reference perceptual ranges (fingertip baseline)
REFERENCE_FREQ_RANGE_HZ = (20.0, 500.0)  # Full bandwidth
REFERENCE_SPATIAL_RES_MM = 2.0           # Fingertip resolution


def shape(sensation: SensationParams, body: BodyPartProperties) -> SensationParams:
    """
    Adapt body-agnostic SensationParams to specific body part.

    Applies three transformations based on body part properties:
    1. Amplitude gain (inversely proportional to sensitivity)
    2. Frequency ratio compression (limited freq range)
    3. Spatial resolution filtering (coarse body parts lose texture detail)

    Args:
        sensation: Body-agnostic sensation (fingertip-referenced)
        body: BodyPartProperties from Homunculus

    Returns:
        Shaped sensation adapted to body part

    Example:
        >>> sensation = SensationParams(impact_intensity=0.5, ...)
        >>> body = BodyPartProperties(sensitivity=0.5, ...)  # Palm
        >>> shaped = shape(sensation, body)
        >>> shaped.impact_intensity  # 1.0 (2x gain for half sensitivity)
    """
    # Deep copy to avoid modifying original
    s = sensation.copy()

    # === 1. Amplitude Gain ===
    # Less sensitive body parts need stronger signals to perceive the same intensity
    # sensitivity: fingertip=1.0, palm=0.5, torso=0.2
    # gain: fingertip=1.0x, palm=2.0x, torso=5.0x (capped at 5.0)
    gain = min(1.0 / body.sensitivity, 5.0)

    # Apply gain to amplitude-like parameters
    s.impact_intensity *= gain
    s.resonance_intensity *= gain
    s.texture_depth *= gain        # Depth is amplitude of texture vibration
    s.slip_speed *= gain           # Slip speed modulates synthesis amplitude
    s.pressure_magnitude *= gain

    # === 2. Frequency Ratio Compression ===
    # Brightness/density map to frequency downstream
    # Clamp to body part's perceptible range
    freq_min, freq_max = body.freq_range_hz
    ref_freq_min, ref_freq_max = REFERENCE_FREQ_RANGE_HZ

    # Calculate frequency range ratio
    body_range = freq_max - freq_min
    total_range = ref_freq_max - ref_freq_min
    freq_ratio = body_range / total_range  # <1.0 for limited body parts

    # Compress brightness/density to fit perceptible frequency range
    # Example: Palm (30-300Hz, ratio=0.56) compresses full brightness range
    s.resonance_brightness *= freq_ratio
    s.texture_density *= freq_ratio

    # === 3. Spatial Resolution Filtering ===
    # Coarse body parts (high spatial_res_mm) cannot perceive fine texture
    # Reduce texture detail proportionally
    spatial_factor = min(1.0, REFERENCE_SPATIAL_RES_MM / body.spatial_res_mm)
    # Examples:
    #   fingertip (2mm): factor = min(1.0, 2/2) = 1.0 (full detail)
    #   palm (10mm): factor = min(1.0, 2/10) = 0.2
    #   torso (45mm): factor = min(1.0, 2/45) = 0.044

    s.texture_roughness *= spatial_factor
    s.texture_density *= spatial_factor  # Further reduction (combined with freq_ratio)

    # === 4. Clamp All Outputs to [0, 1] ===
    # Gain can push values above 1.0
    s.clamp_all(0.0, 1.0)

    return s


def shape_dict(sensation_dict: dict, body: BodyPartProperties) -> dict:
    """
    Convenience function for shaping dictionary-formatted sensations.

    Args:
        sensation_dict: Dict with sensation parameters
        body: BodyPartProperties from Homunculus

    Returns:
        Dict with shaped sensation parameters
    """
    # Convert dict to SensationParams
    sensation = SensationParams(**sensation_dict)

    # Shape
    shaped = shape(sensation, body)

    # Convert back to dict
    return shaped.to_dict()


def get_shaping_factors(body: BodyPartProperties) -> dict:
    """
    Calculate shaping factors for a body part (for debugging/analysis).

    Args:
        body: BodyPartProperties from Homunculus

    Returns:
        Dict with shaping factors:
        {
            'amplitude_gain': float,
            'frequency_ratio': float,
            'spatial_factor': float
        }
    """
    # Amplitude gain
    gain = min(1.0 / body.sensitivity, 5.0)

    # Frequency ratio
    freq_min, freq_max = body.freq_range_hz
    ref_freq_min, ref_freq_max = REFERENCE_FREQ_RANGE_HZ
    body_range = freq_max - freq_min
    total_range = ref_freq_max - ref_freq_min
    freq_ratio = body_range / total_range

    # Spatial factor
    spatial_factor = min(1.0, REFERENCE_SPATIAL_RES_MM / body.spatial_res_mm)

    return {
        'amplitude_gain': gain,
        'frequency_ratio': freq_ratio,
        'spatial_factor': spatial_factor
    }


if __name__ == '__main__':
    # Test somatotopic shaping
    print("Testing Somatotopic Shaping\n")
    print("="*70)

    from src.routing.somatotopic_router import Homunculus

    # Create homunculus
    homunculus = Homunculus()

    # Create test sensation (fingertip-referenced)
    sensation = SensationParams(
        impact_intensity=0.5,
        impact_sharpness=0.7,
        impact_trigger=True,
        resonance_intensity=0.4,
        resonance_brightness=0.8,
        resonance_sustain=0.6,
        texture_roughness=0.5,
        texture_density=0.6,
        texture_depth=0.4,
        slip_speed=0.3,
        slip_direction=(0.7, 0.7),
        slip_grip=0.5,
        pressure_magnitude=0.6,
        pressure_spread=0.3,
        body_part_id=10,
        timestamp_us=1000000
    )

    print("Original Sensation (fingertip-referenced):")
    print(f"  Impact Intensity: {sensation.impact_intensity:.3f}")
    print(f"  Resonance Brightness: {sensation.resonance_brightness:.3f}")
    print(f"  Texture Roughness: {sensation.texture_roughness:.3f}")
    print(f"  Texture Density: {sensation.texture_density:.3f}")

    print("\n" + "="*70)

    # Test 1: Fingertip (should be passthrough)
    print("\n1. Fingertip (index_tip):")
    body_fingertip = homunculus.table['index_tip']
    factors_fingertip = get_shaping_factors(body_fingertip)
    shaped_fingertip = shape(sensation, body_fingertip)

    print(f"  Shaping Factors:")
    print(f"    Amplitude Gain: {factors_fingertip['amplitude_gain']:.2f}x")
    print(f"    Frequency Ratio: {factors_fingertip['frequency_ratio']:.2f}")
    print(f"    Spatial Factor: {factors_fingertip['spatial_factor']:.2f}")
    print(f"  Shaped Sensation:")
    print(f"    Impact Intensity: {shaped_fingertip.impact_intensity:.3f} (unchanged)")
    print(f"    Texture Roughness: {shaped_fingertip.texture_roughness:.3f} (full detail)")

    # Test 2: Palm (moderate shaping)
    print("\n2. Palm (palm_center):")
    body_palm = homunculus.table['palm_center']
    factors_palm = get_shaping_factors(body_palm)
    shaped_palm = shape(sensation, body_palm)

    print(f"  Shaping Factors:")
    print(f"    Amplitude Gain: {factors_palm['amplitude_gain']:.2f}x")
    print(f"    Frequency Ratio: {factors_palm['frequency_ratio']:.2f}")
    print(f"    Spatial Factor: {factors_palm['spatial_factor']:.2f}")
    print(f"  Shaped Sensation:")
    print(f"    Impact Intensity: {shaped_palm.impact_intensity:.3f} (2x gain)")
    print(f"    Resonance Brightness: {shaped_palm.resonance_brightness:.3f} (compressed)")
    print(f"    Texture Roughness: {shaped_palm.texture_roughness:.3f} (reduced)")
    print(f"    Texture Density: {shaped_palm.texture_density:.3f} (reduced)")

    # Test 3: Torso (heavy attenuation)
    print("\n3. Torso (chest):")
    body_chest = homunculus.table['chest']
    factors_chest = get_shaping_factors(body_chest)
    shaped_chest = shape(sensation, body_chest)

    print(f"  Shaping Factors:")
    print(f"    Amplitude Gain: {factors_chest['amplitude_gain']:.2f}x")
    print(f"    Frequency Ratio: {factors_chest['frequency_ratio']:.2f}")
    print(f"    Spatial Factor: {factors_chest['spatial_factor']:.2f}")
    print(f"  Shaped Sensation:")
    print(f"    Impact Intensity: {shaped_chest.impact_intensity:.3f} (5x gain, clamped)")
    print(f"    Resonance Brightness: {shaped_chest.resonance_brightness:.3f} (compressed)")
    print(f"    Texture Roughness: {shaped_chest.texture_roughness:.3f} (nearly zero)")
    print(f"    Texture Density: {shaped_chest.texture_density:.3f} (nearly zero)")

    print("\n" + "="*70)

    # Verify clamping
    print("\nClamping Verification:")
    assert shaped_fingertip.impact_intensity <= 1.0, "Fingertip not clamped!"
    assert shaped_palm.impact_intensity <= 1.0, "Palm not clamped!"
    assert shaped_chest.impact_intensity <= 1.0, "Chest not clamped!"
    print("  ✓ All values clamped to [0, 1]")

    # Verify spatial filtering
    print("\nSpatial Filtering Verification:")
    assert shaped_fingertip.texture_roughness == sensation.texture_roughness, "Fingertip texture changed!"
    assert shaped_palm.texture_roughness < sensation.texture_roughness, "Palm texture not reduced!"
    assert shaped_chest.texture_roughness < shaped_palm.texture_roughness, "Chest texture not reduced!"
    print("  ✓ Texture detail decreases with coarser body parts")

    print("\n" + "="*70)
    print("✓ Somatotopic shaping test passed!")
