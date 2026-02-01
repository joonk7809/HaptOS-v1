"""
Unit tests for Neural Renderer and Schema Adapters

Tests:
- Schema conversion functions (NEW ↔ OLD)
- Legacy dict → NEW CueParams conversion
- Cue masking logic
- Neural Renderer core functionality
- Window buffering
- Phase transition detection
"""

import unittest
import numpy as np
import time
from typing import Tuple

from src.core.schemas import ContactPatch, FilteredContact, CueParams
from src.inference.adapters import (
    new_contact_to_old,
    legacy_cues_to_new,
    apply_cue_mask,
    select_dominant_ring_mode,
    derive_shear_direction,
    derive_texture_grain,
    MATERIAL_PROPERTIES,
    MATERIAL_IMPACT_FREQ,
    COLOR_TO_GRAIN_HZ,
    OldContactPatch
)
from src.inference.neural_renderer import NeuralRenderer, ContactBuffer


class TestAdapters(unittest.TestCase):
    """Test schema conversion functions."""

    def test_new_contact_to_old_conversion(self):
        """Verify NEW → OLD ContactPatch field mapping."""
        # Create NEW ContactPatch
        new_contact = ContactPatch(
            body_part_id=10,
            force_normal=1.5,
            force_shear=(0.3, 0.4),  # Magnitude = 0.5
            velocity=(0.05, 0.03, 0.0),  # Magnitude = 0.058 m/s = 58 mm/s
            contact_area=0.01,
            material_hint=1,  # Metal
            timestamp_us=1000000
        )

        # Convert to OLD
        old_contact = new_contact_to_old(new_contact)

        # Verify field mappings
        self.assertIsInstance(old_contact, OldContactPatch)
        self.assertEqual(old_contact.timestamp_us, 1000000)
        self.assertEqual(old_contact.normal_force_N, 1.5)

        # Shear force should be vector magnitude
        expected_shear = np.sqrt(0.3**2 + 0.4**2)
        self.assertAlmostEqual(old_contact.shear_force_N, expected_shear, places=5)

        # Slip speed should be velocity magnitude in mm/s
        expected_slip_speed = np.sqrt(0.05**2 + 0.03**2) * 1000
        self.assertAlmostEqual(old_contact.slip_speed_mms, expected_slip_speed, places=2)

        # Material properties from lookup table (metal)
        self.assertEqual(old_contact.mu_static, 0.3)
        self.assertEqual(old_contact.mu_dynamic, 0.3)
        self.assertEqual(old_contact.solref_damping, 0.8)

        # Defaults for missing fields
        self.assertTrue(old_contact.in_contact)
        self.assertEqual(old_contact.contact_pos.tolist(), [0.0, 0.0, 0.0])
        self.assertEqual(old_contact.contact_normal.tolist(), [0.0, 0.0, 1.0])

    def test_legacy_cues_to_new_conversion(self):
        """Verify legacy dict → NEW CueParams conversion."""
        # Create legacy CueParams dict (from CombinedPredictor output)
        legacy_dict = {
            'impact': {'A': 0.8, 'rise_ms': 5.0, 'fall_ms': 50.0, 'hf_weight': 0.3},
            'ring': {
                'f_Hz': [150.0, 200.0, 250.0],
                'tau_ms': [100.0, 80.0, 120.0],
                'a': [0.6, 0.4, 0.2]  # Mode 0 dominant
            },
            'shear': {'A': 0.5, 'band_Hz': [100.0]},
            'weight': {'A': 0.7, 'rate_ms': 200.0},
            'texture': {'A': 0.6, 'color': 'COLOR_PINK'}
        }

        # Create ContactPatch for shear direction extraction
        contact = ContactPatch(
            body_part_id=10,
            force_normal=1.0,
            force_shear=(0.4, 0.3),  # Direction: (0.8, 0.6) normalized
            velocity=(0.0, 0.0, 0.0),
            contact_area=0.01,
            material_hint=1,  # Metal
            timestamp_us=2000000
        )

        # Convert to NEW CueParams
        cue_params = legacy_cues_to_new(
            legacy_dict=legacy_dict,
            contact=contact,
            timestamp_us=2000000,
            trigger_impulse=True
        )

        # Verify conversions
        self.assertEqual(cue_params.impact_amplitude, 0.8)
        self.assertEqual(cue_params.impact_decay_ms, 50.0)
        self.assertEqual(cue_params.impact_frequency_hz, 200.0)  # From material_hint=1 (metal)

        # Ring: dominant mode (mode 0, amplitude 0.6)
        self.assertEqual(cue_params.ring_amplitude, 0.6)
        self.assertEqual(cue_params.ring_decay_ms, 100.0)

        # Shear
        self.assertEqual(cue_params.shear_magnitude, 0.5)
        # Direction normalized from (0.4, 0.3)
        expected_dir = (0.8, 0.6)
        self.assertAlmostEqual(cue_params.shear_direction[0], expected_dir[0], places=2)
        self.assertAlmostEqual(cue_params.shear_direction[1], expected_dir[1], places=2)

        # Weight
        self.assertEqual(cue_params.weight_offset, 0.7)

        # Texture
        self.assertEqual(cue_params.texture_amplitude, 0.6)
        self.assertEqual(cue_params.texture_grain_hz, 150.0)  # COLOR_PINK → 150 Hz

        # Transient
        self.assertTrue(cue_params.trigger_impulse)
        self.assertEqual(cue_params.timestamp_us, 2000000)

    def test_ring_mode_selection(self):
        """Test 3 ring modes → 1 dominant mode selection."""
        # Mode 1 has highest amplitude
        ring_dict = {
            'f_Hz': [150.0, 200.0, 250.0],
            'tau_ms': [100.0, 80.0, 120.0],
            'a': [0.3, 0.8, 0.5]  # Mode 1 dominant (amplitude 0.8)
        }

        amplitude, decay = select_dominant_ring_mode(ring_dict)

        self.assertEqual(amplitude, 0.8)
        self.assertEqual(decay, 80.0)

    def test_texture_grain_derivation(self):
        """Test color classification → texture_grain_hz mapping."""
        self.assertEqual(derive_texture_grain('COLOR_WHITE'), 80.0)
        self.assertEqual(derive_texture_grain('COLOR_PINK'), 150.0)
        self.assertEqual(derive_texture_grain('COLOR_BROWN'), 250.0)
        self.assertEqual(derive_texture_grain('UNKNOWN'), 150.0)  # Default

    def test_cue_masking(self):
        """Verify cue_mask correctly zeros disabled parameters."""
        # Create full CueParams
        cue_params = CueParams(
            texture_grain_hz=150.0,
            texture_amplitude=0.6,
            shear_direction=(1.0, 0.0),
            shear_magnitude=0.5,
            weight_offset=0.7,
            impact_amplitude=0.8,
            impact_decay_ms=50.0,
            impact_frequency_hz=200.0,
            ring_amplitude=0.6,
            ring_decay_ms=100.0,
            trigger_impulse=True,
            timestamp_us=1000000
        )

        # Test mask: Only IMPACT and WEIGHT enabled (0b10001 = 17)
        mask = FilteredContact.CUE_IMPACT | FilteredContact.CUE_WEIGHT
        masked = apply_cue_mask(cue_params, mask)

        # IMPACT: Enabled
        self.assertEqual(masked.impact_amplitude, 0.8)
        self.assertEqual(masked.impact_decay_ms, 50.0)
        self.assertTrue(masked.trigger_impulse)

        # WEIGHT: Enabled
        self.assertEqual(masked.weight_offset, 0.7)

        # RING: Disabled
        self.assertEqual(masked.ring_amplitude, 0.0)
        self.assertEqual(masked.ring_decay_ms, 0.0)

        # TEXTURE: Disabled
        self.assertEqual(masked.texture_amplitude, 0.0)
        self.assertEqual(masked.texture_grain_hz, 0.0)

        # SHEAR: Disabled
        self.assertEqual(masked.shear_magnitude, 0.0)
        self.assertEqual(masked.shear_direction, (0.0, 0.0))

    def test_material_property_lookup(self):
        """Test material_hint → mu/damping mapping."""
        # Metal (hint 1)
        self.assertEqual(MATERIAL_PROPERTIES[1]['mu'], 0.3)
        self.assertEqual(MATERIAL_PROPERTIES[1]['damping'], 0.8)

        # Wood (hint 2)
        self.assertEqual(MATERIAL_PROPERTIES[2]['mu'], 0.6)
        self.assertEqual(MATERIAL_PROPERTIES[2]['damping'], 1.5)

        # Impact frequency mapping
        self.assertEqual(MATERIAL_IMPACT_FREQ[1], 200.0)  # Metal: high freq
        self.assertEqual(MATERIAL_IMPACT_FREQ[3], 80.0)   # Rubber: low freq

    def test_shear_direction_zero_force(self):
        """Test shear direction when force is near zero."""
        contact = ContactPatch(
            body_part_id=10,
            force_normal=1.0,
            force_shear=(0.0, 0.0),  # No shear
            velocity=(0.0, 0.0, 0.0),
            contact_area=0.01,
            material_hint=0,
            timestamp_us=1000000
        )

        direction = derive_shear_direction(contact)
        # Should return default direction
        self.assertEqual(direction, (1.0, 0.0))


class TestContactBuffer(unittest.TestCase):
    """Test window buffering logic."""

    def test_buffer_initialization(self):
        """Test buffer starts empty."""
        buffer = ContactBuffer(window_size=10)
        self.assertIsNone(buffer.get_window(body_part_id=10))

    def test_buffer_accumulation(self):
        """Test buffer accumulates contacts."""
        buffer = ContactBuffer(window_size=3)

        # Add 2 contacts (not full yet)
        for i in range(2):
            patch = OldContactPatch(
                timestamp_us=i*1000,
                normal_force_N=1.0,
                shear_force_N=0.5,
                slip_speed_mms=10.0,
                contact_pos=np.array([0, 0, 0]),
                contact_normal=np.array([0, 0, 1]),
                in_contact=True,
                mu_static=0.5,
                mu_dynamic=0.5,
                solref_damping=1.0
            )
            buffer.add(10, patch)

        # Not full yet
        self.assertIsNone(buffer.get_window(10))

        # Add 3rd contact (now full)
        patch = OldContactPatch(
            timestamp_us=2000,
            normal_force_N=1.0,
            shear_force_N=0.5,
            slip_speed_mms=10.0,
            contact_pos=np.array([0, 0, 0]),
            contact_normal=np.array([0, 0, 1]),
            in_contact=True,
            mu_static=0.5,
            mu_dynamic=0.5,
            solref_damping=1.0
        )
        buffer.add(10, patch)

        # Now should return full window
        window = buffer.get_window(10)
        self.assertIsNotNone(window)
        self.assertEqual(len(window), 3)

    def test_buffer_rolling_window(self):
        """Test buffer maintains rolling window (FIFO)."""
        buffer = ContactBuffer(window_size=3)

        # Add 5 contacts (should keep only last 3)
        for i in range(5):
            patch = OldContactPatch(
                timestamp_us=i*1000,
                normal_force_N=float(i),
                shear_force_N=0.5,
                slip_speed_mms=10.0,
                contact_pos=np.array([0, 0, 0]),
                contact_normal=np.array([0, 0, 1]),
                in_contact=True,
                mu_static=0.5,
                mu_dynamic=0.5,
                solref_damping=1.0
            )
            buffer.add(10, patch)

        window = buffer.get_window(10)
        self.assertEqual(len(window), 3)
        # Should have forces 2.0, 3.0, 4.0 (last 3)
        self.assertEqual(window[0].normal_force_N, 2.0)
        self.assertEqual(window[1].normal_force_N, 3.0)
        self.assertEqual(window[2].normal_force_N, 4.0)


class TestNeuralRenderer(unittest.TestCase):
    """Test Neural Renderer core functionality."""

    def setUp(self):
        """Create renderer for tests."""
        # Note: This requires actual model files to exist
        # For now, we'll test the structure without full inference
        try:
            self.renderer = NeuralRenderer()
        except Exception as e:
            self.skipTest(f"Could not initialize renderer (missing models?): {e}")

    def test_renderer_initialization(self):
        """Verify renderer loads correctly."""
        self.assertIsNotNone(self.renderer.predictor)
        self.assertIsInstance(self.renderer.buffer, ContactBuffer)
        self.assertEqual(len(self.renderer.feature_extractors), 0)
        self.assertEqual(self.renderer.render_count, 0)

    def test_impulse_detection_impact(self):
        """Test impulse detection: NO_CONTACT → IMPACT."""
        renderer = NeuralRenderer()

        # NO_CONTACT (0) → IMPACT (1)
        trigger = renderer._detect_impulse(body_part_id=10, current_phase_idx=1)
        self.assertTrue(trigger)  # Should fire impulse

        # IMPACT → IMPACT (no transition)
        trigger = renderer._detect_impulse(body_part_id=10, current_phase_idx=1)
        self.assertFalse(trigger)  # No impulse

    def test_impulse_detection_slip(self):
        """Test impulse detection: HOLD → SLIP."""
        renderer = NeuralRenderer()

        # Simulate: NO_CONTACT → IMPACT → HOLD → SLIP
        renderer._detect_impulse(10, 1)  # NO_CONTACT → IMPACT (fires)
        renderer._detect_impulse(10, 2)  # IMPACT → HOLD (no fire)
        trigger = renderer._detect_impulse(10, 3)  # HOLD → SLIP
        self.assertTrue(trigger)  # Should fire impulse

    def test_impulse_detection_release(self):
        """Test impulse detection: Any → RELEASE."""
        renderer = NeuralRenderer()

        # Simulate: HOLD → RELEASE
        renderer._detect_impulse(10, 2)  # Set to HOLD
        trigger = renderer._detect_impulse(10, 4)  # HOLD → RELEASE
        self.assertTrue(trigger)  # Should fire impulse

    def test_reset(self):
        """Test renderer reset clears all state."""
        renderer = NeuralRenderer()

        # Add some state
        renderer.render_count = 100
        renderer.total_inference_time_ms = 500.0
        renderer.prev_phases[10] = 2

        # Reset
        renderer.reset()

        # Verify cleared
        self.assertEqual(renderer.render_count, 0)
        self.assertEqual(renderer.total_inference_time_ms, 0.0)
        self.assertEqual(len(renderer.prev_phases), 0)

    def test_get_stats(self):
        """Test statistics tracking."""
        renderer = NeuralRenderer()

        # Simulate some renders
        renderer.render_count = 10
        renderer.total_inference_time_ms = 80.0
        renderer.active_contacts = {10, 11, 12}

        stats = renderer.get_stats()

        self.assertEqual(stats['total_renders'], 10)
        self.assertEqual(stats['avg_inference_time_ms'], 8.0)
        self.assertEqual(stats['active_contacts'], 3)


class TestIntegrationScenarios(unittest.TestCase):
    """Integration tests with realistic contact scenarios."""

    def test_single_contact_flow(self):
        """Test complete flow: FilteredContact → CueParams."""
        # Create FilteredContact (from Router)
        contact_patch = ContactPatch(
            body_part_id=10,
            force_normal=1.5,
            force_shear=(0.3, 0.4),
            velocity=(0.05, 0.0, 0.0),
            contact_area=0.01,
            material_hint=1,  # Metal
            timestamp_us=1000000
        )

        filtered = FilteredContact(
            patch=contact_patch,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        # Verify schema conversion
        old_patch = new_contact_to_old(contact_patch)
        self.assertIsInstance(old_patch, OldContactPatch)
        self.assertEqual(old_patch.normal_force_N, 1.5)

        # Verify we can create a buffer
        buffer = ContactBuffer(window_size=10)
        buffer.add(10, old_patch)

        # Window not full yet (only 1 sample)
        window = buffer.get_window(10)
        self.assertIsNone(window)


if __name__ == '__main__':
    unittest.main()
