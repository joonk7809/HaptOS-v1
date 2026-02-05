#!/usr/bin/env python3
"""
Unit tests for somatotopic shaping function.

Tests cover:
- Fingertip passthrough (no changes)
- Palm moderate shaping
- Torso heavy attenuation
- Gain clamping (values stay in [0, 1])
- Frequency ratio compression
- Spatial factor calculation
- Copy semantics (original unchanged)
"""

import pytest
import numpy as np
from src.rendering.somatotopic_shaper import shape, get_shaping_factors
from src.routing.somatotopic_router import Homunculus, BodyPartProperties
from src.core.schemas import SensationParams, FilteredContact


class TestFingertipPassthrough:
    """Test that fingertip shaping is identity (no changes)."""

    def test_fingertip_amplitude_unchanged(self):
        """Test that fingertip amplitudes pass through unchanged."""
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

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

        shaped = shape(sensation, body)

        # All amplitudes should be unchanged (gain=1.0)
        assert shaped.impact_intensity == sensation.impact_intensity
        assert shaped.resonance_intensity == sensation.resonance_intensity
        assert shaped.texture_depth == sensation.texture_depth
        assert shaped.pressure_magnitude == sensation.pressure_magnitude

    def test_fingertip_texture_unchanged(self):
        """Test that fingertip texture detail is preserved."""
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.texture_roughness = 0.5
        sensation.texture_density = 0.6

        shaped = shape(sensation, body)

        # Texture should be unchanged (spatial_factor=1.0)
        assert shaped.texture_roughness == sensation.texture_roughness
        assert shaped.texture_density == sensation.texture_density

    def test_fingertip_frequency_unchanged(self):
        """Test that fingertip frequency range is not compressed."""
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.resonance_brightness = 0.8
        sensation.texture_density = 0.7

        shaped = shape(sensation, body)

        # Frequency parameters unchanged (freq_ratio=1.0)
        assert shaped.resonance_brightness == sensation.resonance_brightness
        # Note: texture_density affected by both freq_ratio and spatial_factor
        # For fingertip: both are 1.0, so unchanged
        assert shaped.texture_density == sensation.texture_density

    def test_fingertip_shaping_factors(self):
        """Test that fingertip shaping factors are all 1.0."""
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        factors = get_shaping_factors(body)

        assert factors['amplitude_gain'] == 1.0
        assert factors['frequency_ratio'] == 1.0
        assert factors['spatial_factor'] == 1.0


class TestPalmShaping:
    """Test moderate shaping for palm."""

    def test_palm_amplitude_gain(self):
        """Test that palm has 2x amplitude gain (sensitivity=0.5)."""
        homunculus = Homunculus()
        body = homunculus.table['palm_center']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.impact_intensity = 0.4
        sensation.pressure_magnitude = 0.3

        shaped = shape(sensation, body)

        # Palm sensitivity=0.5 → gain=2.0x
        assert abs(shaped.impact_intensity - 0.8) < 1e-6
        assert abs(shaped.pressure_magnitude - 0.6) < 1e-6

    def test_palm_texture_reduction(self):
        """Test that palm texture is reduced (spatial_factor=0.2)."""
        homunculus = Homunculus()
        body = homunculus.table['palm_center']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.texture_roughness = 0.5

        shaped = shape(sensation, body)

        # Palm spatial_res=10mm, reference=2mm → factor=0.2
        expected_roughness = 0.5 * 0.2
        assert abs(shaped.texture_roughness - expected_roughness) < 1e-6

    def test_palm_frequency_compression(self):
        """Test that palm frequency range is compressed."""
        homunculus = Homunculus()
        body = homunculus.table['palm_center']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.resonance_brightness = 0.8

        shaped = shape(sensation, body)

        # Palm freq_range=(30-300Hz), reference=(20-500Hz)
        # freq_ratio = 270/480 = 0.5625
        expected_brightness = 0.8 * 0.5625
        assert abs(shaped.resonance_brightness - expected_brightness) < 0.01

    def test_palm_shaping_factors(self):
        """Test palm shaping factors."""
        homunculus = Homunculus()
        body = homunculus.table['palm_center']

        factors = get_shaping_factors(body)

        assert factors['amplitude_gain'] == 2.0
        assert abs(factors['frequency_ratio'] - 0.5625) < 0.01
        assert factors['spatial_factor'] == 0.2


class TestTorsoShaping:
    """Test heavy attenuation for torso."""

    def test_torso_max_gain(self):
        """Test that torso has maximum gain (5x, clamped)."""
        homunculus = Homunculus()
        body = homunculus.table['chest']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.impact_intensity = 0.3

        shaped = shape(sensation, body)

        # Chest sensitivity=0.2 → gain=5.0x
        # 0.3 * 5.0 = 1.5, clamped to 1.0
        assert shaped.impact_intensity == 1.0

    def test_torso_texture_near_zero(self):
        """Test that torso texture is nearly eliminated."""
        homunculus = Homunculus()
        body = homunculus.table['chest']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.texture_roughness = 0.5
        sensation.texture_density = 0.6

        shaped = shape(sensation, body)

        # Chest spatial_res=45mm, reference=2mm → factor=0.044
        assert shaped.texture_roughness < 0.05
        assert shaped.texture_density < 0.05

    def test_torso_frequency_heavy_compression(self):
        """Test that torso frequency range is heavily compressed."""
        homunculus = Homunculus()
        body = homunculus.table['chest']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.resonance_brightness = 0.8

        shaped = shape(sensation, body)

        # Chest freq_range=(50-100Hz), reference=(20-500Hz)
        # freq_ratio = 50/480 = 0.104
        expected_brightness = 0.8 * 0.104
        assert abs(shaped.resonance_brightness - expected_brightness) < 0.01

    def test_torso_shaping_factors(self):
        """Test torso shaping factors."""
        homunculus = Homunculus()
        body = homunculus.table['chest']

        factors = get_shaping_factors(body)

        assert factors['amplitude_gain'] == 5.0
        assert abs(factors['frequency_ratio'] - 0.104) < 0.01
        assert abs(factors['spatial_factor'] - 0.044) < 0.01


class TestGainClamping:
    """Test that gain doesn't push values above 1.0."""

    def test_clamping_impact(self):
        """Test that impact intensity is clamped to 1.0."""
        # Create high-sensitivity body part with gain > 1.0
        body = BodyPartProperties(
            spatial_res_mm=2.0,
            freq_range_hz=(20, 500),
            sensitivity=0.1,  # Very insensitive → 10x gain (clamped to 5x)
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.impact_intensity = 0.5

        shaped = shape(sensation, body)

        # 0.5 * 5.0 = 2.5, should be clamped to 1.0
        assert shaped.impact_intensity == 1.0

    def test_clamping_all_parameters(self):
        """Test that all parameters are clamped to [0, 1]."""
        body = BodyPartProperties(
            spatial_res_mm=2.0,
            freq_range_hz=(20, 500),
            sensitivity=0.2,  # 5x gain
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        # Create sensation with high values
        sensation = SensationParams(
            impact_intensity=0.8,
            impact_sharpness=0.9,
            impact_trigger=True,
            resonance_intensity=0.7,
            resonance_brightness=0.8,
            resonance_sustain=0.6,
            texture_roughness=0.5,
            texture_density=0.6,
            texture_depth=0.9,
            slip_speed=0.8,
            slip_direction=(0.7, 0.7),
            slip_grip=0.5,
            pressure_magnitude=0.9,
            pressure_spread=0.3,
            body_part_id=10,
            timestamp_us=1000000
        )

        shaped = shape(sensation, body)

        # All values should be in [0, 1]
        assert 0.0 <= shaped.impact_intensity <= 1.0
        assert 0.0 <= shaped.impact_sharpness <= 1.0
        assert 0.0 <= shaped.resonance_intensity <= 1.0
        assert 0.0 <= shaped.resonance_brightness <= 1.0
        assert 0.0 <= shaped.resonance_sustain <= 1.0
        assert 0.0 <= shaped.texture_roughness <= 1.0
        assert 0.0 <= shaped.texture_density <= 1.0
        assert 0.0 <= shaped.texture_depth <= 1.0
        assert 0.0 <= shaped.slip_speed <= 1.0
        assert 0.0 <= shaped.slip_grip <= 1.0
        assert 0.0 <= shaped.pressure_magnitude <= 1.0
        assert 0.0 <= shaped.pressure_spread <= 1.0


class TestCopySemantics:
    """Test that shaping doesn't modify original sensation."""

    def test_original_unchanged(self):
        """Test that original sensation is not modified."""
        homunculus = Homunculus()
        body = homunculus.table['palm_center']

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

        # Store original values
        original_impact = sensation.impact_intensity
        original_texture = sensation.texture_roughness

        # Shape (should create copy)
        shaped = shape(sensation, body)

        # Original should be unchanged
        assert sensation.impact_intensity == original_impact
        assert sensation.texture_roughness == original_texture

        # Shaped should be different
        assert shaped.impact_intensity != sensation.impact_intensity
        assert shaped.texture_roughness != sensation.texture_roughness


class TestSpatialFactorCalculation:
    """Test spatial factor calculation for various body parts."""

    def test_spatial_factor_fingertip(self):
        """Test spatial factor for fingertip (2mm)."""
        body = BodyPartProperties(
            spatial_res_mm=2.0,
            freq_range_hz=(20, 500),
            sensitivity=1.0,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        factors = get_shaping_factors(body)
        # 2mm / 2mm = 1.0
        assert factors['spatial_factor'] == 1.0

    def test_spatial_factor_palm(self):
        """Test spatial factor for palm (10mm)."""
        body = BodyPartProperties(
            spatial_res_mm=10.0,
            freq_range_hz=(20, 500),
            sensitivity=1.0,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        factors = get_shaping_factors(body)
        # 2mm / 10mm = 0.2
        assert factors['spatial_factor'] == 0.2

    def test_spatial_factor_clamped_at_one(self):
        """Test that spatial factor is clamped at 1.0 (no enhancement)."""
        # Body part better than fingertip (hypothetical)
        body = BodyPartProperties(
            spatial_res_mm=1.0,  # Better than reference
            freq_range_hz=(20, 500),
            sensitivity=1.0,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        factors = get_shaping_factors(body)
        # min(1.0, 2mm/1mm) = min(1.0, 2.0) = 1.0
        assert factors['spatial_factor'] == 1.0


class TestFrequencyRatioCalculation:
    """Test frequency ratio calculation for various ranges."""

    def test_frequency_ratio_full_range(self):
        """Test frequency ratio for full range (20-500Hz)."""
        body = BodyPartProperties(
            spatial_res_mm=2.0,
            freq_range_hz=(20, 500),
            sensitivity=1.0,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        factors = get_shaping_factors(body)
        # (500-20) / (500-20) = 1.0
        assert factors['frequency_ratio'] == 1.0

    def test_frequency_ratio_narrowband(self):
        """Test frequency ratio for narrowband (50-100Hz)."""
        body = BodyPartProperties(
            spatial_res_mm=2.0,
            freq_range_hz=(50, 100),
            sensitivity=1.0,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        factors = get_shaping_factors(body)
        # (100-50) / (500-20) = 50/480 = 0.104
        assert abs(factors['frequency_ratio'] - 0.104) < 0.01

    def test_frequency_ratio_palm(self):
        """Test frequency ratio for palm (30-300Hz)."""
        body = BodyPartProperties(
            spatial_res_mm=2.0,
            freq_range_hz=(30, 300),
            sensitivity=1.0,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        factors = get_shaping_factors(body)
        # (300-30) / (500-20) = 270/480 = 0.5625
        assert abs(factors['frequency_ratio'] - 0.5625) < 0.01


class TestMultiBodyPartComparison:
    """Test shaping behavior across multiple body parts."""

    def test_texture_decreases_with_coarser_body_parts(self):
        """Test that texture detail decreases monotonically."""
        homunculus = Homunculus()

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.texture_roughness = 0.5

        # Shape for fingertip, palm, chest
        shaped_fingertip = shape(sensation, homunculus.table['index_tip'])
        shaped_palm = shape(sensation, homunculus.table['palm_center'])
        shaped_chest = shape(sensation, homunculus.table['chest'])

        # Texture should decrease: fingertip > palm > chest
        assert shaped_fingertip.texture_roughness > shaped_palm.texture_roughness
        assert shaped_palm.texture_roughness > shaped_chest.texture_roughness

    def test_amplitude_increases_with_less_sensitive_parts(self):
        """Test that amplitude increases for less sensitive body parts."""
        homunculus = Homunculus()

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.impact_intensity = 0.3

        # Shape for fingertip, palm, chest
        shaped_fingertip = shape(sensation, homunculus.table['index_tip'])
        shaped_palm = shape(sensation, homunculus.table['palm_center'])
        shaped_chest = shape(sensation, homunculus.table['chest'])

        # Amplitude should increase: fingertip < palm < chest
        assert shaped_fingertip.impact_intensity < shaped_palm.impact_intensity
        assert shaped_palm.impact_intensity < shaped_chest.impact_intensity


class TestEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_zero_sensation(self):
        """Test that zero sensation stays zero after shaping."""
        homunculus = Homunculus()
        body = homunculus.table['palm_center']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        shaped = shape(sensation, body)

        # All zeros should stay zero (0 * gain = 0)
        assert shaped.impact_intensity == 0.0
        assert shaped.texture_roughness == 0.0
        assert shaped.pressure_magnitude == 0.0

    def test_max_sensitivity_body_part(self):
        """Test body part with sensitivity > 1.0 (hypothetical)."""
        # Super-sensitive body part (gain < 1.0)
        body = BodyPartProperties(
            spatial_res_mm=2.0,
            freq_range_hz=(20, 500),
            sensitivity=2.0,  # Twice as sensitive as baseline
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.impact_intensity = 0.8

        shaped = shape(sensation, body)

        # Gain = 1/2.0 = 0.5 (attenuation)
        assert abs(shaped.impact_intensity - 0.4) < 1e-6


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
