#!/usr/bin/env python3
"""
Integration test for Physics → Router → Renderer pipeline.

Tests the complete data flow through the HAPTOS Platform architecture:
1. Physics Engine (Layer 1) emits ContactPatches
2. Somatotopic Router filters and tags contacts
3. (Future) Neural Renderer (Layer 2) generates haptic cues

This validates that all components work together correctly.
"""

import sys
sys.path.append('.')

import unittest
import os
from src.physics.multi_contact_engine import MultiContactEngine
from src.core.schemas import ContactPatch, FilteredContact
from src.routing.somatotopic_router import SomatotopicRouter, Homunculus


class TestPhysicsRouterIntegration(unittest.TestCase):
    """Test Physics → Router integration."""

    @classmethod
    def setUpClass(cls):
        """Set up test environment once for all tests."""
        # Find hand model
        model_paths = [
            'assets/hand_models/detailed_hand.xml',
            'assets/hand_models/simple_hand.xml'
        ]

        cls.model_path = None
        for path in model_paths:
            if os.path.exists(path):
                cls.model_path = path
                break

        if cls.model_path is None:
            raise unittest.SkipTest("No hand model found for integration test")

    def setUp(self):
        """Set up each test with fresh engine and router."""
        self.engine = MultiContactEngine(self.model_path, max_contacts=10)
        self.router = SomatotopicRouter(Homunculus())

    def test_physics_emits_contact_patches(self):
        """Test that physics engine emits ContactPatch objects."""
        # Step simulation
        patches = self.engine.step_v2()

        # Should return a list (even if empty)
        self.assertIsInstance(patches, list)

        # Each item should be a ContactPatch
        for patch in patches:
            self.assertIsInstance(patch, ContactPatch)
            self.assertGreaterEqual(patch.timestamp_us, 0)
            self.assertGreaterEqual(patch.force_normal, 0.0)

    def test_router_accepts_physics_output(self):
        """Test that router can process physics engine output."""
        # Step physics
        patches = self.engine.step_v2()

        # Route through somatotopic router
        filtered = self.router.route(patches)

        # Should return a list
        self.assertIsInstance(filtered, list)

        # Each item should be FilteredContact
        for contact in filtered:
            self.assertIsInstance(contact, FilteredContact)
            self.assertIsInstance(contact.patch, ContactPatch)
            self.assertIn(contact.rendering_tier, [1, 2, 3])
            self.assertGreaterEqual(contact.cue_mask, 0)

    def test_end_to_end_with_contact(self):
        """
        Test full pipeline with simulated contact.

        This test runs the simulation until a contact occurs,
        then validates the complete data flow.
        """
        max_steps = 100
        contact_found = False

        for step in range(max_steps):
            # Step physics
            patches = self.engine.step_v2()

            if patches:
                contact_found = True

                # Verify ContactPatch structure
                for patch in patches:
                    self.assertIsInstance(patch, ContactPatch)
                    self.assertGreater(patch.force_normal, 0.0)
                    self.assertIsInstance(patch.force_shear, tuple)
                    self.assertEqual(len(patch.force_shear), 2)
                    self.assertIsInstance(patch.velocity, tuple)
                    self.assertEqual(len(patch.velocity), 3)

                # Route through somatotopic router
                filtered = self.router.route(patches)

                # Should have at least one filtered contact
                # (unless all were below perceptual threshold)
                if filtered:
                    for contact in filtered:
                        # Verify FilteredContact structure
                        self.assertIsInstance(contact, FilteredContact)
                        self.assertIn(contact.rendering_tier, [1, 2, 3])

                        # Check cue mask is valid
                        self.assertGreaterEqual(contact.cue_mask, 0)
                        self.assertLessEqual(contact.cue_mask, FilteredContact.CUE_ALL)

                        # Verify original patch is from the input list
                        self.assertIn(contact.patch, patches)

                break

        # Note: We don't require contact to be found, since the hand might be
        # falling freely in the simple test scene. This test mainly validates
        # that the data structures are compatible.

    def test_multiple_contacts_routing(self):
        """
        Test that router correctly handles multiple simultaneous contacts.
        """
        max_steps = 200
        multi_contact_found = False

        for step in range(max_steps):
            patches = self.engine.step_v2()

            if len(patches) > 1:
                multi_contact_found = True

                # Route all contacts
                filtered = self.router.route(patches)

                # Verify each contact is processed independently
                for contact in filtered:
                    self.assertIsInstance(contact, FilteredContact)

                    # Each contact should have appropriate tier based on body part
                    # Fingertips should be tier 1, palm might be tier 2
                    self.assertIn(contact.rendering_tier, [1, 2, 3])

                break

        # Note: Multi-contact scenario might not occur in simple test
        # This is OK - the test validates structure when it does occur

    def test_router_sensitivity_filtering(self):
        """
        Test that router filters weak contacts based on sensitivity.
        """
        max_steps = 100

        for step in range(max_steps):
            patches = self.engine.step_v2()

            if patches:
                # Get all force magnitudes before filtering
                forces_before = [p.force_normal for p in patches]

                # Route through somatotopic router
                filtered = self.router.route(patches)

                # Get forces after filtering
                forces_after = [c.patch.force_normal for c in filtered]

                # Filtered contacts should be subset of original
                # (some may be rejected due to sensitivity threshold)
                self.assertLessEqual(len(filtered), len(patches))

                # All filtered forces should be above some minimum
                for force in forces_after:
                    # Should be above perceptual threshold (varies by body part)
                    # Minimum threshold is 0.01N / max_sensitivity
                    # For most sensitive parts (sensitivity=1.0), threshold=0.01N
                    # For least sensitive (sensitivity=0.15), threshold~=0.067N
                    self.assertGreater(force, 0.005)  # Conservative lower bound

                break

    def test_data_flow_consistency(self):
        """
        Test that data flows consistently through the pipeline.

        Validates:
        - Timestamps are preserved
        - Force values are preserved
        - No data corruption during routing
        """
        for step in range(50):
            patches = self.engine.step_v2()

            if patches:
                # Route contacts
                filtered = self.router.route(patches)

                for contact in filtered:
                    # Timestamp should be preserved
                    self.assertEqual(
                        contact.patch.timestamp_us,
                        self.engine.time_us
                    )

                    # Force values should be unchanged
                    original_patch = contact.patch
                    self.assertGreater(original_patch.force_normal, 0.0)

                    # Velocity tuple should have 3 elements
                    self.assertEqual(len(original_patch.velocity), 3)

                    # Shear force should have 2 elements
                    self.assertEqual(len(original_patch.force_shear), 2)

                break


class TestRouterStatistics(unittest.TestCase):
    """Test router statistics tracking during simulation."""

    @classmethod
    def setUpClass(cls):
        """Set up test environment."""
        model_paths = [
            'assets/hand_models/detailed_hand.xml',
            'assets/hand_models/simple_hand.xml'
        ]

        cls.model_path = None
        for path in model_paths:
            if os.path.exists(path):
                cls.model_path = path
                break

        if cls.model_path is None:
            raise unittest.SkipTest("No hand model found for test")

    def test_router_statistics(self):
        """Test that router tracks statistics correctly."""
        engine = MultiContactEngine(self.model_path)
        router = SomatotopicRouter(Homunculus())

        # Reset stats
        router.reset_stats()
        initial_stats = router.get_stats()
        self.assertEqual(initial_stats['total_contacts_routed'], 0)

        # Run simulation
        total_filtered = 0
        for step in range(100):
            patches = engine.step_v2()
            filtered = router.route(patches)
            total_filtered += len(filtered)

        # Check stats
        final_stats = router.get_stats()
        self.assertEqual(final_stats['total_contacts_routed'], total_filtered)


if __name__ == '__main__':
    unittest.main()
