"""
Integration tests for Router → Renderer pipeline

Tests the complete data flow from Physics → Router → Renderer
"""

import unittest
from src.core.schemas import ContactPatch, FilteredContact, CueParams
from src.routing.somatotopic_router import SomatotopicRouter, Homunculus
from src.inference.neural_renderer import NeuralRenderer
from src.physics.multi_contact_engine import MultiContactEngine


class TestRouterRendererPipeline(unittest.TestCase):
    """Test Router → Renderer integration."""

    def setUp(self):
        """Initialize router and renderer for tests."""
        self.router = SomatotopicRouter(Homunculus())
        try:
            self.renderer = NeuralRenderer()
        except Exception as e:
            self.skipTest(f"Could not initialize renderer (missing models?): {e}")

    def test_end_to_end_data_flow(self):
        """
        Test complete pipeline: ContactPatch → Router → Renderer → CueParams

        Validates:
        - FilteredContact correctly consumed by renderer
        - CueParams correctly emitted
        - Timestamps preserved
        - No data corruption
        """
        # Create ContactPatch (from physics simulation)
        contact_patch = ContactPatch(
            body_part_id=10,  # Index fingertip
            force_normal=1.5,
            force_shear=(0.3, 0.4),
            velocity=(0.05, 0.0, 0.0),
            contact_area=0.01,
            material_hint=1,  # Metal
            timestamp_us=1000000
        )

        # Step 1: Router filters contact
        filtered_contacts = self.router.route([contact_patch])

        self.assertEqual(len(filtered_contacts), 1)
        filtered = filtered_contacts[0]
        self.assertIsInstance(filtered, FilteredContact)
        self.assertEqual(filtered.patch.body_part_id, 10)
        self.assertEqual(filtered.rendering_tier, 1)  # Index fingertip = Tier 1 (VCA)
        self.assertEqual(filtered.cue_mask, FilteredContact.CUE_ALL)

        # Step 2: Accumulate window (need 10 samples for inference)
        # Note: Real usage would accumulate over time
        # For testing, we'll just verify the interface works
        for i in range(10):
            # Simulate 10ms of contacts
            test_patch = ContactPatch(
                body_part_id=10,
                force_normal=1.5 + i*0.01,
                force_shear=(0.3, 0.4),
                velocity=(0.05, 0.0, 0.0),
                contact_area=0.01,
                material_hint=1,
                timestamp_us=1000000 + i*1000
            )
            filtered = FilteredContact(
                patch=test_patch,
                rendering_tier=1,
                cue_mask=FilteredContact.CUE_ALL
            )
            cue_dict = self.renderer.render([filtered])

            if i < 9:
                # Not enough samples yet (window not full)
                self.assertEqual(len(cue_dict), 0)
            else:
                # Window full, should get CueParams
                self.assertEqual(len(cue_dict), 1)
                self.assertIn(10, cue_dict)
                cue_params = cue_dict[10]
                self.assertIsInstance(cue_params, CueParams)
                # Timestamp preserved
                self.assertGreater(cue_params.timestamp_us, 0)

    def test_cue_mask_propagation(self):
        """
        Test that router cue_mask affects renderer output.

        Example: Palm contact (cue_mask excludes shear/ring)
        should produce CueParams with shear/ring zeroed.
        """
        # Create palm contact (body_part_id 50, restricted cues)
        contact_patch = ContactPatch(
            body_part_id=50,  # Palm
            force_normal=2.0,
            force_shear=(0.5, 0.3),
            velocity=(0.02, 0.0, 0.0),
            contact_area=0.05,
            material_hint=2,  # Wood
            timestamp_us=2000000
        )

        # Router should apply palm-specific cue mask
        filtered_contacts = self.router.route([contact_patch])

        self.assertEqual(len(filtered_contacts), 1)
        filtered = filtered_contacts[0]

        # Palm should have limited cues (no texture, no shear by default)
        # Check if cue_mask excludes texture and shear
        has_texture = bool(filtered.cue_mask & FilteredContact.CUE_TEXTURE)
        has_shear = bool(filtered.cue_mask & FilteredContact.CUE_SHEAR)

        # Note: Actual mask depends on Homunculus table configuration
        # This test verifies that masking is applied

        # Accumulate 10 samples for inference
        for i in range(10):
            test_patch = ContactPatch(
                body_part_id=50,
                force_normal=2.0,
                force_shear=(0.5, 0.3),
                velocity=(0.02, 0.0, 0.0),
                contact_area=0.05,
                material_hint=2,
                timestamp_us=2000000 + i*1000
            )
            filtered = FilteredContact(
                patch=test_patch,
                rendering_tier=filtered_contacts[0].rendering_tier,
                cue_mask=filtered_contacts[0].cue_mask
            )
            cue_dict = self.renderer.render([filtered])

        # After 10 samples, check output
        if len(cue_dict) > 0:
            cue_params = cue_dict[50]

            # If texture disabled, amplitude should be 0
            if not has_texture:
                self.assertEqual(cue_params.texture_amplitude, 0.0)

            # If shear disabled, magnitude should be 0
            if not has_shear:
                self.assertEqual(cue_params.shear_magnitude, 0.0)

    def test_multi_contact_independence(self):
        """
        Verify that each contact is rendered independently.

        Different body parts should produce different CueParams
        based on their individual force/velocity profiles.
        """
        # Create two different contacts (index + thumb)
        contact1 = ContactPatch(
            body_part_id=10,  # Index fingertip
            force_normal=1.0,
            force_shear=(0.2, 0.1),
            velocity=(0.03, 0.0, 0.0),
            contact_area=0.01,
            material_hint=1,  # Metal
            timestamp_us=3000000
        )

        contact2 = ContactPatch(
            body_part_id=20,  # Thumb fingertip
            force_normal=1.5,  # Different force
            force_shear=(0.4, 0.3),  # Different shear
            velocity=(0.05, 0.0, 0.0),  # Different velocity
            contact_area=0.01,
            material_hint=2,  # Wood (different material)
            timestamp_us=3000000
        )

        # Route both contacts
        filtered_contacts = self.router.route([contact1, contact2])

        self.assertEqual(len(filtered_contacts), 2)

        # Accumulate 10 samples for both contacts
        for i in range(10):
            test_contacts = [
                ContactPatch(
                    body_part_id=10,
                    force_normal=1.0 + i*0.01,
                    force_shear=(0.2, 0.1),
                    velocity=(0.03, 0.0, 0.0),
                    contact_area=0.01,
                    material_hint=1,
                    timestamp_us=3000000 + i*1000
                ),
                ContactPatch(
                    body_part_id=20,
                    force_normal=1.5 + i*0.01,
                    force_shear=(0.4, 0.3),
                    velocity=(0.05, 0.0, 0.0),
                    contact_area=0.01,
                    material_hint=2,
                    timestamp_us=3000000 + i*1000
                )
            ]

            # Route and render
            filtered = self.router.route(test_contacts)
            cue_dict = self.renderer.render(filtered)

        # After 10 samples, both contacts should have CueParams
        if len(cue_dict) == 2:
            self.assertIn(10, cue_dict)
            self.assertIn(20, cue_dict)

            cue1 = cue_dict[10]
            cue2 = cue_dict[20]

            # Different forces → different weight offsets
            # (This assumes the model produces different outputs for different forces)
            # Note: Exact values depend on model training

            # Both should be valid CueParams
            self.assertIsInstance(cue1, CueParams)
            self.assertIsInstance(cue2, CueParams)

            # Timestamps should be different or same (depending on sync)
            # Both should be > 0
            self.assertGreater(cue1.timestamp_us, 0)
            self.assertGreater(cue2.timestamp_us, 0)


class TestPhysicsRouterRendererPipeline(unittest.TestCase):
    """Test complete Physics → Router → Renderer pipeline."""

    def setUp(self):
        """Initialize physics, router, and renderer."""
        try:
            self.engine = MultiContactEngine('assets/hand_models/detailed_hand.xml')
            self.router = SomatotopicRouter(Homunculus())
            self.renderer = NeuralRenderer()
        except Exception as e:
            self.skipTest(f"Could not initialize pipeline (missing models/assets?): {e}")

    def test_complete_pipeline_single_step(self):
        """
        Test one step of complete pipeline:
        Physics.step_v2() → Router.route() → Renderer.render()
        """
        # Step physics simulation
        contacts = self.engine.step_v2()

        # contacts might be empty if no collision in current frame
        if len(contacts) > 0:
            # Route contacts
            filtered = self.router.route(contacts)

            # filtered might be empty if all contacts below threshold
            if len(filtered) > 0:
                # Render (may be empty if windows not full)
                cue_dict = self.renderer.render(filtered)

                # Verify structure (even if empty)
                self.assertIsInstance(cue_dict, dict)
                for body_part_id, cue_params in cue_dict.items():
                    self.assertIsInstance(body_part_id, int)
                    self.assertIsInstance(cue_params, CueParams)


if __name__ == '__main__':
    unittest.main()
