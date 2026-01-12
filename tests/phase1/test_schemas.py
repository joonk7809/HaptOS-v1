#!/usr/bin/env python3
"""
Unit tests for core HAPTOS schemas.

Tests ContactPatch, FilteredContact, and CueParams data structures.
"""

import sys
sys.path.append('.')

import unittest
from src.core.schemas import ContactPatch, FilteredContact, CueParams, make_cue_mask


class TestContactPatch(unittest.TestCase):
    """Test ContactPatch data structure."""

    def test_creation(self):
        """Test basic ContactPatch creation."""
        patch = ContactPatch(
            body_part_id=10,
            force_normal=1.5,
            force_shear=(0.2, 0.3),
            velocity=(0.1, 0.0, -0.05),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=1000000
        )

        self.assertEqual(patch.body_part_id, 10)
        self.assertEqual(patch.force_normal, 1.5)
        self.assertEqual(patch.force_shear, (0.2, 0.3))
        self.assertEqual(patch.material_hint, 1)

    def test_to_dict(self):
        """Test ContactPatch dictionary conversion."""
        patch = ContactPatch(
            body_part_id=10,
            force_normal=1.5,
            force_shear=(0.2, 0.3),
            velocity=(0.1, 0.0, -0.05),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=1000000
        )

        d = patch.to_dict()
        self.assertEqual(d['body_part_id'], 10)
        self.assertEqual(d['force_normal'], 1.5)
        self.assertEqual(d['force_shear_x'], 0.2)
        self.assertEqual(d['force_shear_y'], 0.3)
        self.assertEqual(d['velocity_x'], 0.1)
        self.assertEqual(d['velocity_z'], -0.05)


class TestFilteredContact(unittest.TestCase):
    """Test FilteredContact data structure and cue masking."""

    def setUp(self):
        """Create test ContactPatch."""
        self.patch = ContactPatch(
            body_part_id=10,
            force_normal=1.5,
            force_shear=(0.2, 0.3),
            velocity=(0.1, 0.0, -0.05),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=1000000
        )

    def test_creation(self):
        """Test FilteredContact creation."""
        filtered = FilteredContact(
            patch=self.patch,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        self.assertEqual(filtered.rendering_tier, 1)
        self.assertEqual(filtered.cue_mask, 0b11111)

    def test_cue_masking(self):
        """Test cue mask bitmask operations."""
        # All cues enabled
        filtered = FilteredContact(
            patch=self.patch,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )
        self.assertTrue(filtered.has_cue(FilteredContact.CUE_IMPACT))
        self.assertTrue(filtered.has_cue(FilteredContact.CUE_TEXTURE))
        self.assertTrue(filtered.has_cue(FilteredContact.CUE_SHEAR))

        # Only impact and weight
        filtered = FilteredContact(
            patch=self.patch,
            rendering_tier=3,
            cue_mask=FilteredContact.CUE_IMPACT | FilteredContact.CUE_WEIGHT
        )
        self.assertTrue(filtered.has_cue(FilteredContact.CUE_IMPACT))
        self.assertTrue(filtered.has_cue(FilteredContact.CUE_WEIGHT))
        self.assertFalse(filtered.has_cue(FilteredContact.CUE_TEXTURE))
        self.assertFalse(filtered.has_cue(FilteredContact.CUE_SHEAR))
        self.assertFalse(filtered.has_cue(FilteredContact.CUE_RING))

    def test_enabled_cues(self):
        """Test enabled cue list generation."""
        filtered = FilteredContact(
            patch=self.patch,
            rendering_tier=2,
            cue_mask=FilteredContact.CUE_IMPACT | FilteredContact.CUE_TEXTURE | FilteredContact.CUE_WEIGHT
        )

        enabled = filtered.enabled_cues()
        self.assertEqual(len(enabled), 3)
        self.assertIn('impact', enabled)
        self.assertIn('texture', enabled)
        self.assertIn('weight', enabled)
        self.assertNotIn('shear', enabled)
        self.assertNotIn('ring', enabled)


class TestCueParams(unittest.TestCase):
    """Test CueParams data structure and serialization."""

    def test_creation(self):
        """Test CueParams creation."""
        cue = CueParams(
            texture_grain_hz=150.0,
            texture_amplitude=0.5,
            shear_direction=(0.7, 0.3),
            shear_magnitude=0.4,
            weight_offset=0.2,
            impact_amplitude=0.8,
            impact_decay_ms=50.0,
            impact_frequency_hz=200.0,
            ring_amplitude=0.3,
            ring_decay_ms=100.0,
            trigger_impulse=True,
            timestamp_us=2000000
        )

        self.assertEqual(cue.texture_grain_hz, 150.0)
        self.assertEqual(cue.impact_amplitude, 0.8)
        self.assertTrue(cue.trigger_impulse)

    def test_serialization_roundtrip(self):
        """Test CueParams binary serialization and deserialization."""
        original = CueParams(
            texture_grain_hz=150.0,
            texture_amplitude=0.5,
            shear_direction=(0.7, 0.3),
            shear_magnitude=0.4,
            weight_offset=0.2,
            impact_amplitude=0.8,
            impact_decay_ms=50.0,
            impact_frequency_hz=200.0,
            ring_amplitude=0.3,
            ring_decay_ms=100.0,
            trigger_impulse=True,
            timestamp_us=2000000
        )

        # Serialize
        data = original.to_bytes()
        self.assertEqual(len(data), 52)  # Expected packet size

        # Verify header
        self.assertEqual(data[0], 0xAA)
        self.assertEqual(data[1], 0x55)

        # Deserialize
        restored = CueParams.from_bytes(data)

        # Check all fields match
        self.assertAlmostEqual(restored.texture_grain_hz, original.texture_grain_hz, places=5)
        self.assertAlmostEqual(restored.texture_amplitude, original.texture_amplitude, places=5)
        self.assertAlmostEqual(restored.shear_direction[0], original.shear_direction[0], places=5)
        self.assertAlmostEqual(restored.shear_direction[1], original.shear_direction[1], places=5)
        self.assertAlmostEqual(restored.shear_magnitude, original.shear_magnitude, places=5)
        self.assertAlmostEqual(restored.weight_offset, original.weight_offset, places=5)
        self.assertAlmostEqual(restored.impact_amplitude, original.impact_amplitude, places=5)
        self.assertAlmostEqual(restored.impact_decay_ms, original.impact_decay_ms, places=5)
        self.assertAlmostEqual(restored.impact_frequency_hz, original.impact_frequency_hz, places=5)
        self.assertAlmostEqual(restored.ring_amplitude, original.ring_amplitude, places=5)
        self.assertAlmostEqual(restored.ring_decay_ms, original.ring_decay_ms, places=5)
        self.assertEqual(restored.trigger_impulse, original.trigger_impulse)

    def test_serialization_checksum(self):
        """Test that corrupted data fails checksum validation."""
        cue = CueParams.zero(timestamp_us=1000)
        data = bytearray(cue.to_bytes())

        # Corrupt a byte
        data[10] ^= 0xFF

        # Should raise ValueError due to checksum mismatch
        with self.assertRaises(ValueError) as context:
            CueParams.from_bytes(bytes(data))

        self.assertIn("Checksum mismatch", str(context.exception))

    def test_serialization_invalid_header(self):
        """Test that invalid header is detected."""
        cue = CueParams.zero(timestamp_us=1000)
        data = bytearray(cue.to_bytes())

        # Corrupt header
        data[0] = 0xFF

        # Recompute checksum for corrupted data
        checksum = 0
        for byte in data[:-1]:
            checksum ^= byte
        data[-1] = checksum

        # Should raise ValueError due to invalid header
        with self.assertRaises(ValueError) as context:
            CueParams.from_bytes(bytes(data))

        self.assertIn("Invalid header", str(context.exception))

    def test_zero_cue(self):
        """Test CueParams.zero() factory method."""
        zero_cue = CueParams.zero(timestamp_us=5000)

        self.assertEqual(zero_cue.texture_grain_hz, 0.0)
        self.assertEqual(zero_cue.texture_amplitude, 0.0)
        self.assertEqual(zero_cue.shear_magnitude, 0.0)
        self.assertEqual(zero_cue.impact_amplitude, 0.0)
        self.assertEqual(zero_cue.ring_amplitude, 0.0)
        self.assertFalse(zero_cue.trigger_impulse)
        self.assertEqual(zero_cue.timestamp_us, 5000)

    def test_to_dict(self):
        """Test CueParams dictionary conversion."""
        cue = CueParams(
            texture_grain_hz=150.0,
            texture_amplitude=0.5,
            shear_direction=(0.7, 0.3),
            shear_magnitude=0.4,
            weight_offset=0.2,
            impact_amplitude=0.8,
            impact_decay_ms=50.0,
            impact_frequency_hz=200.0,
            ring_amplitude=0.3,
            ring_decay_ms=100.0,
            trigger_impulse=True,
            timestamp_us=2000000
        )

        d = cue.to_dict()
        self.assertIn('continuous', d)
        self.assertIn('transient', d)
        self.assertEqual(d['continuous']['texture_grain_hz'], 150.0)
        self.assertEqual(d['transient']['impact_amplitude'], 0.8)
        self.assertTrue(d['transient']['trigger_impulse'])


class TestCueMaskHelper(unittest.TestCase):
    """Test cue mask helper function."""

    def test_make_cue_mask(self):
        """Test make_cue_mask convenience function."""
        # Single cue
        mask = make_cue_mask(FilteredContact.CUE_IMPACT)
        self.assertEqual(mask, 0b00001)

        # Multiple cues
        mask = make_cue_mask(FilteredContact.CUE_IMPACT, FilteredContact.CUE_TEXTURE)
        self.assertEqual(mask, 0b00101)

        # All cues
        mask = make_cue_mask(
            FilteredContact.CUE_IMPACT,
            FilteredContact.CUE_RING,
            FilteredContact.CUE_TEXTURE,
            FilteredContact.CUE_SHEAR,
            FilteredContact.CUE_WEIGHT
        )
        self.assertEqual(mask, FilteredContact.CUE_ALL)

        # No cues
        mask = make_cue_mask()
        self.assertEqual(mask, 0)


if __name__ == '__main__':
    unittest.main()
