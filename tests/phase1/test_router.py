#!/usr/bin/env python3
"""
Unit tests for Somatotopic Router.

Tests Homunculus table lookup and ContactPatch routing logic.
"""

import sys
sys.path.append('.')

import unittest
from src.core.schemas import ContactPatch, FilteredContact
from src.routing.somatotopic_router import (
    SomatotopicRouter,
    Homunculus,
    BodyPartProperties,
    create_default_router
)


class TestHomunculus(unittest.TestCase):
    """Test Homunculus perceptual properties table."""

    def setUp(self):
        """Create test Homunculus."""
        self.homunculus = Homunculus()

    def test_default_table_exists(self):
        """Test that default table is populated."""
        self.assertGreater(len(self.homunculus.table), 0)
        self.assertIn('index_tip', self.homunculus.table)
        self.assertIn('palm_center', self.homunculus.table)

    def test_fingertip_properties(self):
        """Test fingertip has high resolution properties."""
        props = self.homunculus.table['index_tip']

        # Fingertips should have:
        self.assertLessEqual(props.spatial_res_mm, 3.0)  # High spatial resolution
        self.assertEqual(props.rendering_tier, 1)         # Full VCA rendering
        self.assertEqual(props.cue_mask, FilteredContact.CUE_ALL)  # All cues
        self.assertGreaterEqual(props.sensitivity, 0.8)   # High sensitivity

    def test_palm_properties(self):
        """Test palm has lower resolution than fingertips."""
        fingertip = self.homunculus.table['index_tip']
        palm = self.homunculus.table['palm_center']

        # Palm should be less sensitive than fingertips
        self.assertGreater(palm.spatial_res_mm, fingertip.spatial_res_mm)
        self.assertLess(palm.sensitivity, fingertip.sensitivity)

    def test_id_to_name_mapping(self):
        """Test body part ID to name mapping."""
        # Known IDs from hand model
        self.assertEqual(self.homunculus._id_to_name(10), 'index_tip')  # index_geom
        self.assertEqual(self.homunculus._id_to_name(7), 'thumb_tip')   # thumb_geom
        self.assertEqual(self.homunculus._id_to_name(3), 'palm_center') # palm_center_geom

    def test_id_to_name_segments(self):
        """Test that finger segments map to tip properties."""
        # Middle segments should map to tip
        self.assertEqual(self.homunculus._id_to_name(9), 'index_tip')  # index_mid_geom -> index_tip
        self.assertEqual(self.homunculus._id_to_name(8), 'index_tip')  # index_prox_geom -> index_tip

    def test_lookup_known_id(self):
        """Test lookup with known body part ID."""
        props = self.homunculus.lookup(10)  # index_tip

        self.assertIsInstance(props, BodyPartProperties)
        self.assertEqual(props.spatial_res_mm, 2.0)
        self.assertEqual(props.rendering_tier, 1)

    def test_lookup_unknown_id(self):
        """Test lookup with unknown ID returns fallback."""
        props = self.homunculus.lookup(9999)  # Unknown ID

        self.assertIsInstance(props, BodyPartProperties)
        # Should have conservative fallback properties
        self.assertEqual(props.rendering_tier, 3)  # ERM tier
        self.assertGreaterEqual(props.spatial_res_mm, 30.0)  # Low resolution
        self.assertLessEqual(props.sensitivity, 0.3)  # Low sensitivity

    def test_fallback_properties(self):
        """Test fallback properties are conservative."""
        fallback = self.homunculus._fallback_properties()

        self.assertEqual(fallback.rendering_tier, 3)  # Simplest hardware
        self.assertEqual(fallback.cue_mask, FilteredContact.CUE_IMPACT | FilteredContact.CUE_WEIGHT)
        self.assertLess(fallback.sensitivity, 0.5)  # Low sensitivity

    def test_save_and_load(self):
        """Test Homunculus save/load roundtrip."""
        import tempfile
        import os

        # Create temporary file
        fd, filepath = tempfile.mkstemp(suffix='.json')
        os.close(fd)

        try:
            # Save
            self.homunculus.save(filepath)
            self.assertTrue(os.path.exists(filepath))

            # Load
            loaded = Homunculus.load(filepath)
            self.assertEqual(len(loaded.table), len(self.homunculus.table))

            # Check properties match
            for name in self.homunculus.table.keys():
                original = self.homunculus.table[name]
                restored = loaded.table[name]

                self.assertEqual(original.spatial_res_mm, restored.spatial_res_mm)
                self.assertEqual(original.rendering_tier, restored.rendering_tier)
                self.assertEqual(original.cue_mask, restored.cue_mask)
                self.assertEqual(original.sensitivity, restored.sensitivity)

        finally:
            # Cleanup
            if os.path.exists(filepath):
                os.remove(filepath)


class TestSomatotopicRouter(unittest.TestCase):
    """Test SomatotopicRouter contact filtering logic."""

    def setUp(self):
        """Create test router."""
        self.router = create_default_router()

    def test_creation(self):
        """Test router creation."""
        self.assertIsInstance(self.router, SomatotopicRouter)
        self.assertIsInstance(self.router.homunculus, Homunculus)

    def test_route_empty(self):
        """Test routing with no contacts."""
        filtered = self.router.route([])
        self.assertEqual(len(filtered), 0)

    def test_route_single_contact(self):
        """Test routing single contact."""
        patch = ContactPatch(
            body_part_id=10,  # index_tip
            force_normal=1.0,
            force_shear=(0.1, 0.0),
            velocity=(0.0, 0.0, 0.0),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=1000000
        )

        filtered = self.router.route([patch])

        self.assertEqual(len(filtered), 1)
        self.assertIsInstance(filtered[0], FilteredContact)
        self.assertEqual(filtered[0].patch, patch)
        self.assertEqual(filtered[0].rendering_tier, 1)  # Fingertip = tier 1
        self.assertEqual(filtered[0].cue_mask, FilteredContact.CUE_ALL)

    def test_route_below_threshold(self):
        """Test that contacts below perceptual threshold are filtered out."""
        # Very weak contact (0.001N)
        weak_patch = ContactPatch(
            body_part_id=10,  # index_tip (sensitivity=1.0, threshold=0.01N)
            force_normal=0.005,  # Below 0.01N threshold
            force_shear=(0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=1000000
        )

        filtered = self.router.route([weak_patch])

        # Should be filtered out
        self.assertEqual(len(filtered), 0)

    def test_route_above_threshold(self):
        """Test that contacts above threshold pass through."""
        strong_patch = ContactPatch(
            body_part_id=10,  # index_tip (sensitivity=1.0, threshold=0.01N)
            force_normal=0.5,  # Well above threshold
            force_shear=(0.1, 0.0),
            velocity=(0.0, 0.0, 0.0),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=1000000
        )

        filtered = self.router.route([strong_patch])

        # Should pass through
        self.assertEqual(len(filtered), 1)

    def test_route_multiple_contacts(self):
        """Test routing multiple contacts simultaneously."""
        patches = [
            ContactPatch(
                body_part_id=10,  # index_tip
                force_normal=1.0,
                force_shear=(0.1, 0.0),
                velocity=(0.0, 0.0, 0.0),
                contact_area=0.0001,
                material_hint=1,
                timestamp_us=1000000
            ),
            ContactPatch(
                body_part_id=7,   # thumb_tip
                force_normal=0.8,
                force_shear=(0.0, 0.1),
                velocity=(0.0, 0.0, 0.0),
                contact_area=0.0001,
                material_hint=1,
                timestamp_us=1000000
            ),
            ContactPatch(
                body_part_id=3,   # palm_center
                force_normal=2.0,
                force_shear=(0.0, 0.0),
                velocity=(0.0, 0.0, 0.0),
                contact_area=0.0005,
                material_hint=1,
                timestamp_us=1000000
            )
        ]

        filtered = self.router.route(patches)

        # All should pass
        self.assertEqual(len(filtered), 3)

        # Check each has correct properties
        index_contact = next(c for c in filtered if c.patch.body_part_id == 10)
        self.assertEqual(index_contact.rendering_tier, 1)
        self.assertEqual(index_contact.cue_mask, FilteredContact.CUE_ALL)

        palm_contact = next(c for c in filtered if c.patch.body_part_id == 3)
        self.assertEqual(palm_contact.rendering_tier, 2)  # Palm = tier 2
        # Palm should have limited cues (not all)
        self.assertNotEqual(palm_contact.cue_mask, FilteredContact.CUE_ALL)

    def test_route_mixed_threshold(self):
        """Test routing with mix of above/below threshold contacts."""
        patches = [
            ContactPatch(
                body_part_id=10,  # index_tip
                force_normal=1.0,  # Above threshold
                force_shear=(0.1, 0.0),
                velocity=(0.0, 0.0, 0.0),
                contact_area=0.0001,
                material_hint=1,
                timestamp_us=1000000
            ),
            ContactPatch(
                body_part_id=7,   # thumb_tip
                force_normal=0.005,  # Below threshold
                force_shear=(0.0, 0.0),
                velocity=(0.0, 0.0, 0.0),
                contact_area=0.0001,
                material_hint=1,
                timestamp_us=1000000
            )
        ]

        filtered = self.router.route(patches)

        # Only index should pass
        self.assertEqual(len(filtered), 1)
        self.assertEqual(filtered[0].patch.body_part_id, 10)

    def test_sensitivity_scaling(self):
        """Test that sensitivity affects force threshold."""
        # Higher sensitivity body part should accept weaker forces

        # Create custom Homunculus with different sensitivities
        custom_table = {
            'sensitive': BodyPartProperties(
                spatial_res_mm=2.0,
                freq_range_hz=(20, 500),
                sensitivity=2.0,  # 2x more sensitive (threshold = 0.005N)
                rendering_tier=1,
                cue_mask=FilteredContact.CUE_ALL
            ),
            'insensitive': BodyPartProperties(
                spatial_res_mm=2.0,
                freq_range_hz=(20, 500),
                sensitivity=0.5,  # 2x less sensitive (threshold = 0.02N)
                rendering_tier=1,
                cue_mask=FilteredContact.CUE_ALL
            )
        }

        # Hack: temporarily modify ID mapping
        homunculus = Homunculus()
        homunculus.table['sensitive'] = custom_table['sensitive']
        homunculus.table['insensitive'] = custom_table['insensitive']

        def custom_id_to_name(body_part_id):
            return 'sensitive' if body_part_id == 100 else 'insensitive'

        homunculus._id_to_name = custom_id_to_name

        router = SomatotopicRouter(homunculus)

        # 0.01N force - should pass sensitive, fail insensitive
        patches = [
            ContactPatch(
                body_part_id=100,  # sensitive
                force_normal=0.01,
                force_shear=(0.0, 0.0),
                velocity=(0.0, 0.0, 0.0),
                contact_area=0.0001,
                material_hint=0,
                timestamp_us=1000000
            ),
            ContactPatch(
                body_part_id=200,  # insensitive
                force_normal=0.01,
                force_shear=(0.0, 0.0),
                velocity=(0.0, 0.0, 0.0),
                contact_area=0.0001,
                material_hint=0,
                timestamp_us=1000000
            )
        ]

        filtered = router.route(patches)

        # Only sensitive should pass
        self.assertEqual(len(filtered), 1)
        self.assertEqual(filtered[0].patch.body_part_id, 100)

    def test_stats(self):
        """Test router statistics tracking."""
        patches = [
            ContactPatch(
                body_part_id=10,
                force_normal=1.0,
                force_shear=(0.0, 0.0),
                velocity=(0.0, 0.0, 0.0),
                contact_area=0.0001,
                material_hint=0,
                timestamp_us=i * 1000
            )
            for i in range(10)
        ]

        self.router.reset_stats()
        self.router.route(patches)

        stats = self.router.get_stats()
        self.assertEqual(stats['total_contacts'], 10)

        # Route more
        self.router.route(patches)
        stats = self.router.get_stats()
        self.assertEqual(stats['total_contacts'], 20)

        # Reset
        self.router.reset_stats()
        stats = self.router.get_stats()
        self.assertEqual(stats['total_contacts'], 0)


if __name__ == '__main__':
    unittest.main()
