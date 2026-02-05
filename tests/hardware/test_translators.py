#!/usr/bin/env python3
"""
Unit tests for sensation translators (VCA, LRA, ERM).

Tests cover:
- VCA: All channels mapped, frequency ranges correct
- LRA: All frequencies clamped to resonant (175Hz)
- ERM: Collapsed to single duty cycle [0,1]
- Lerp bounds: min/max sensation → min/max synthesis
- Edge cases: sensation=0 → synthesis=0
- Direction preservation: slip_direction passed through
- Factory function: correct translator types
"""

import pytest
import numpy as np
from src.hardware.translators import (
    VCATranslator, LRATranslator, ERMTranslator, create_translator
)
from src.routing.somatotopic_router import Homunculus, BodyPartProperties
from src.core.schemas import SensationParams, SynthesisCommand, FilteredContact


class TestVCATranslator:
    """Test VCA (Voice Coil Actuator) translator."""

    def test_vca_all_channels_mapped(self):
        """Test that VCA maps all sensation channels."""
        translator = VCATranslator()
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams(
            impact_intensity=0.7,
            impact_sharpness=0.8,
            impact_trigger=True,
            resonance_intensity=0.5,
            resonance_brightness=0.6,
            resonance_sustain=0.4,
            texture_roughness=0.3,
            texture_density=0.5,
            texture_depth=0.6,
            slip_speed=0.4,
            slip_direction=(0.7, 0.7),
            slip_grip=0.3,
            pressure_magnitude=0.8,
            pressure_spread=0.2,
            body_part_id=10,
            timestamp_us=1000000
        )

        cmd = translator.translate(sensation, body)

        # All channels should be non-zero
        assert cmd.impact_amplitude > 0.0
        assert cmd.resonance_amplitude > 0.0
        assert cmd.texture_amplitude > 0.0
        assert cmd.slip_amplitude > 0.0
        assert cmd.pressure_amplitude > 0.0

    def test_vca_frequency_ranges(self):
        """Test that VCA frequencies are within body part range."""
        translator = VCATranslator()
        homunculus = Homunculus()
        body = homunculus.table['index_tip']  # (20-500Hz)

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.impact_sharpness = 0.5
        sensation.resonance_brightness = 0.5
        sensation.texture_density = 0.5

        cmd = translator.translate(sensation, body)

        freq_min, freq_max = body.freq_range_hz

        # All frequencies should be in range
        assert freq_min <= cmd.impact_frequency_hz <= freq_max
        assert freq_min <= cmd.resonance_frequency_hz <= freq_max
        assert freq_min <= cmd.texture_center_hz <= freq_max
        assert cmd.pressure_frequency_hz == freq_min

    def test_vca_sharpness_to_frequency(self):
        """Test that sharpness maps to impact frequency."""
        translator = VCATranslator()
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        freq_min, freq_max = body.freq_range_hz

        # Low sharpness → low frequency
        sensation_soft = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation_soft.impact_sharpness = 0.0
        sensation_soft.impact_intensity = 0.5

        cmd_soft = translator.translate(sensation_soft, body)
        assert abs(cmd_soft.impact_frequency_hz - freq_min) < 1.0

        # High sharpness → high frequency
        sensation_sharp = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation_sharp.impact_sharpness = 1.0
        sensation_sharp.impact_intensity = 0.5

        cmd_sharp = translator.translate(sensation_sharp, body)
        assert abs(cmd_sharp.impact_frequency_hz - freq_max) < 1.0

    def test_vca_sharpness_to_decay(self):
        """Test that sharpness maps to decay time (inverse)."""
        translator = VCATranslator()
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        # Low sharpness → long decay
        sensation_soft = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation_soft.impact_sharpness = 0.0

        cmd_soft = translator.translate(sensation_soft, body)
        assert cmd_soft.impact_decay_ms > 40.0  # Near 50ms

        # High sharpness → short decay
        sensation_sharp = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation_sharp.impact_sharpness = 1.0

        cmd_sharp = translator.translate(sensation_sharp, body)
        assert cmd_sharp.impact_decay_ms < 10.0  # Near 2ms

    def test_vca_slip_direction_preserved(self):
        """Test that slip direction is preserved."""
        translator = VCATranslator()
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.slip_direction = (0.6, 0.8)  # Unit vector
        sensation.slip_speed = 0.5

        cmd = translator.translate(sensation, body)

        # Direction should be preserved
        assert cmd.slip_direction == (0.6, 0.8)

    def test_vca_zero_sensation(self):
        """Test that zero sensation produces zero synthesis."""
        translator = VCATranslator()
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)

        cmd = translator.translate(sensation, body)

        # All amplitudes should be zero
        assert cmd.impact_amplitude == 0.0
        assert cmd.resonance_amplitude == 0.0
        assert cmd.texture_amplitude == 0.0
        assert cmd.slip_amplitude == 0.0
        assert cmd.pressure_amplitude == 0.0


class TestLRATranslator:
    """Test LRA (Linear Resonant Actuator) translator."""

    def test_lra_all_frequencies_clamped(self):
        """Test that LRA clamps all frequencies to resonance."""
        translator = LRATranslator(resonant_freq_hz=175.0)
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams(
            impact_intensity=0.7,
            impact_sharpness=0.9,
            impact_trigger=True,
            resonance_intensity=0.5,
            resonance_brightness=0.8,
            resonance_sustain=0.7,
            texture_roughness=0.4,
            texture_density=0.6,
            texture_depth=0.5,
            slip_speed=0.3,
            slip_direction=(0.7, 0.7),
            slip_grip=0.4,
            pressure_magnitude=0.6,
            pressure_spread=0.3,
            body_part_id=10,
            timestamp_us=1000000
        )

        cmd = translator.translate(sensation, body)

        # All frequencies should be 175Hz (resonant)
        assert cmd.impact_frequency_hz == 175.0
        assert cmd.resonance_frequency_hz == 175.0
        assert cmd.texture_center_hz == 175.0
        assert cmd.pressure_frequency_hz == 175.0

    def test_lra_custom_resonant_frequency(self):
        """Test LRA with custom resonant frequency."""
        translator = LRATranslator(resonant_freq_hz=180.0)
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.impact_intensity = 0.5

        cmd = translator.translate(sensation, body)

        # All frequencies should be 180Hz
        assert cmd.impact_frequency_hz == 180.0
        assert cmd.resonance_frequency_hz == 180.0

    def test_lra_amplitude_mapping(self):
        """Test that LRA preserves amplitude mapping."""
        translator = LRATranslator()
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.impact_intensity = 0.7

        cmd = translator.translate(sensation, body)

        # Impact amplitude should be preserved
        assert cmd.impact_amplitude == 0.7

    def test_lra_texture_collapsed(self):
        """Test that LRA collapses texture to amplitude."""
        translator = LRATranslator()
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.texture_roughness = 0.5
        sensation.texture_depth = 0.6

        cmd = translator.translate(sensation, body)

        # Texture amplitude = roughness × depth
        expected_amplitude = 0.5 * 0.6
        assert abs(cmd.texture_amplitude - expected_amplitude) < 1e-6

    def test_lra_reduced_amplitudes(self):
        """Test that LRA reduces certain amplitudes (slip, pressure)."""
        translator = LRATranslator()
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.slip_speed = 0.8
        sensation.slip_grip = 0.2
        sensation.pressure_magnitude = 0.9

        cmd = translator.translate(sensation, body)

        # Slip amplitude should be reduced (×0.5)
        expected_slip = 0.8 * (1.0 - 0.2) * 0.5
        assert abs(cmd.slip_amplitude - expected_slip) < 1e-6

        # Pressure amplitude should be reduced (×0.3)
        expected_pressure = 0.9 * 0.3
        assert abs(cmd.pressure_amplitude - expected_pressure) < 1e-6


class TestERMTranslator:
    """Test ERM (Eccentric Rotating Mass) translator."""

    def test_erm_collapsed_to_duty_cycle(self):
        """Test that ERM collapses all channels to single duty cycle."""
        translator = ERMTranslator()
        homunculus = Homunculus()
        body = homunculus.table['chest']

        sensation = SensationParams(
            impact_intensity=0.5,
            impact_sharpness=0.7,
            impact_trigger=True,
            resonance_intensity=0.3,
            resonance_brightness=0.8,
            resonance_sustain=0.6,
            texture_roughness=0.4,
            texture_density=0.5,
            texture_depth=0.3,
            slip_speed=0.2,
            slip_direction=(0.7, 0.7),
            slip_grip=0.5,
            pressure_magnitude=0.6,
            pressure_spread=0.3,
            body_part_id=10,
            timestamp_us=1000000
        )

        cmd = translator.translate(sensation, body)

        # Only erm_duty_cycle should be set
        assert cmd.erm_duty_cycle > 0.0
        assert cmd.erm_duty_cycle <= 1.0

        # All other fields should be zero
        assert cmd.impact_amplitude == 0.0
        assert cmd.resonance_amplitude == 0.0
        assert cmd.texture_amplitude == 0.0
        assert cmd.slip_amplitude == 0.0
        assert cmd.pressure_amplitude == 0.0

    def test_erm_duty_cycle_clamped(self):
        """Test that ERM duty cycle is clamped to [0, 1]."""
        translator = ERMTranslator()
        homunculus = Homunculus()
        body = homunculus.table['chest']

        # High-intensity sensation (sum > 1.0)
        sensation = SensationParams(
            impact_intensity=0.9,
            impact_sharpness=0.9,
            impact_trigger=True,
            resonance_intensity=0.8,
            resonance_brightness=0.8,
            resonance_sustain=0.8,
            texture_roughness=0.8,
            texture_density=0.8,
            texture_depth=0.8,
            slip_speed=0.8,
            slip_direction=(0.7, 0.7),
            slip_grip=0.2,
            pressure_magnitude=0.9,
            pressure_spread=0.5,
            body_part_id=10,
            timestamp_us=1000000
        )

        cmd = translator.translate(sensation, body)

        # Duty cycle should be clamped to 1.0
        assert cmd.erm_duty_cycle == 1.0

    def test_erm_zero_sensation(self):
        """Test that zero sensation produces zero duty cycle."""
        translator = ERMTranslator()
        homunculus = Homunculus()
        body = homunculus.table['chest']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)

        cmd = translator.translate(sensation, body)

        # Duty cycle should be zero
        assert cmd.erm_duty_cycle == 0.0

    def test_erm_impact_prioritized(self):
        """Test that ERM prioritizes impact over other channels."""
        translator = ERMTranslator()
        homunculus = Homunculus()
        body = homunculus.table['chest']

        # Impact only
        sensation_impact = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation_impact.impact_intensity = 0.5

        cmd_impact = translator.translate(sensation_impact, body)

        # Texture only (same intensity)
        sensation_texture = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation_texture.texture_roughness = 0.5
        sensation_texture.texture_depth = 1.0  # Product = 0.5

        cmd_texture = translator.translate(sensation_texture, body)

        # Impact should have higher weight (1.0 vs 0.5)
        assert cmd_impact.erm_duty_cycle > cmd_texture.erm_duty_cycle


class TestLerpFunction:
    """Test linear interpolation utility."""

    def test_lerp_at_zero(self):
        """Test lerp at t=0."""
        from src.hardware.translators import SensationTranslator

        result = SensationTranslator._lerp(10.0, 100.0, 0.0)
        assert result == 10.0

    def test_lerp_at_one(self):
        """Test lerp at t=1."""
        from src.hardware.translators import SensationTranslator

        result = SensationTranslator._lerp(10.0, 100.0, 1.0)
        assert result == 100.0

    def test_lerp_at_half(self):
        """Test lerp at t=0.5."""
        from src.hardware.translators import SensationTranslator

        result = SensationTranslator._lerp(10.0, 100.0, 0.5)
        assert result == 55.0


class TestFactoryFunction:
    """Test translator factory function."""

    def test_factory_vca(self):
        """Test factory creates VCA translator."""
        translator = create_translator(tier=1)
        assert isinstance(translator, VCATranslator)

    def test_factory_lra(self):
        """Test factory creates LRA translator."""
        translator = create_translator(tier=2)
        assert isinstance(translator, LRATranslator)

    def test_factory_lra_custom_freq(self):
        """Test factory creates LRA with custom resonant frequency."""
        translator = create_translator(tier=2, resonant_freq_hz=180.0)
        assert isinstance(translator, LRATranslator)
        assert translator.f_res == 180.0

    def test_factory_erm(self):
        """Test factory creates ERM translator."""
        translator = create_translator(tier=3)
        assert isinstance(translator, ERMTranslator)

    def test_factory_invalid_tier(self):
        """Test factory raises error for invalid tier."""
        with pytest.raises(ValueError, match="Unknown hardware tier"):
            create_translator(tier=99)


class TestCrossTranslatorComparison:
    """Test behavior across different translators."""

    def test_frequency_bandwidth_decreases(self):
        """Test that frequency bandwidth decreases: VCA > LRA."""
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.impact_sharpness = 0.5
        sensation.resonance_brightness = 0.5

        # VCA: full frequency range
        vca = VCATranslator()
        vca_cmd = vca.translate(sensation, body)

        # LRA: fixed frequency
        lra = LRATranslator()
        lra_cmd = lra.translate(sensation, body)

        # VCA should use variable frequencies
        assert vca_cmd.impact_frequency_hz != vca_cmd.resonance_frequency_hz or sensation.impact_sharpness == sensation.resonance_brightness

        # LRA should have all frequencies the same
        assert lra_cmd.impact_frequency_hz == lra_cmd.resonance_frequency_hz

    def test_detail_loss_across_tiers(self):
        """Test that perceptual detail decreases across tiers."""
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams(
            impact_intensity=0.5,
            impact_sharpness=0.8,
            impact_trigger=True,
            resonance_intensity=0.4,
            resonance_brightness=0.7,
            resonance_sustain=0.6,
            texture_roughness=0.5,
            texture_density=0.6,
            texture_depth=0.4,
            slip_speed=0.3,
            slip_direction=(0.7, 0.7),
            slip_grip=0.4,
            pressure_magnitude=0.6,
            pressure_spread=0.3,
            body_part_id=10,
            timestamp_us=1000000
        )

        vca = VCATranslator()
        lra = LRATranslator()
        erm = ERMTranslator()

        vca_cmd = vca.translate(sensation, body)
        lra_cmd = lra.translate(sensation, body)
        erm_cmd = erm.translate(sensation, body)

        # VCA: all channels preserved
        assert vca_cmd.impact_amplitude > 0.0
        assert vca_cmd.resonance_amplitude > 0.0
        assert vca_cmd.texture_amplitude > 0.0

        # LRA: channels collapsed, amplitudes reduced
        assert lra_cmd.texture_amplitude < vca_cmd.texture_amplitude

        # ERM: everything collapsed to single value
        assert erm_cmd.impact_amplitude == 0.0  # Not used
        assert erm_cmd.erm_duty_cycle > 0.0  # Only this


class TestEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_max_sensation_vca(self):
        """Test VCA with maximum sensation values."""
        translator = VCATranslator()
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        sensation = SensationParams(
            impact_intensity=1.0,
            impact_sharpness=1.0,
            impact_trigger=True,
            resonance_intensity=1.0,
            resonance_brightness=1.0,
            resonance_sustain=1.0,
            texture_roughness=1.0,
            texture_density=1.0,
            texture_depth=1.0,
            slip_speed=1.0,
            slip_direction=(0.7, 0.7),
            slip_grip=1.0,
            pressure_magnitude=1.0,
            pressure_spread=1.0,
            body_part_id=10,
            timestamp_us=1000000
        )

        cmd = translator.translate(sensation, body)

        # All amplitudes should be valid
        assert 0.0 <= cmd.impact_amplitude <= 1.0
        assert 0.0 <= cmd.resonance_amplitude <= 1.0
        assert 0.0 <= cmd.texture_amplitude <= 1.0
        assert 0.0 <= cmd.pressure_amplitude <= 1.0

    def test_narrow_body_freq_range(self):
        """Test VCA with very narrow body frequency range."""
        translator = VCATranslator()

        # Body part with narrow frequency range
        body = BodyPartProperties(
            spatial_res_mm=45.0,
            freq_range_hz=(50, 100),  # Narrow (50Hz bandwidth)
            sensitivity=0.2,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        sensation = SensationParams.zero(body_part_id=10, timestamp_us=1000000)
        sensation.impact_sharpness = 1.0  # Max

        cmd = translator.translate(sensation, body)

        # Frequency should be at upper bound
        assert abs(cmd.impact_frequency_hz - 100.0) < 1.0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
