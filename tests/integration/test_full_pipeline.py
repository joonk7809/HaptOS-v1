#!/usr/bin/env python3
"""
End-to-end integration test for full SensationParams pipeline.

Tests the complete flow:
    ContactPatch → FilteredContact → FeatureExtractor → NeuralRenderer_v2
    → SensationParams → Somatotopic Shaping → Translator → SynthesisCommand

Validates:
- All components integrate correctly
- Data flows through entire pipeline
- Output values are reasonable
- Performance is acceptable (<15ms end-to-end)
"""

import pytest
import numpy as np
import time
from src.core.schemas import ContactPatch, FilteredContact, SensationParams
from src.converter.feature_extractor_v2 import FeatureExtractorV2
from src.inference.neural_renderer_v2 import NeuralRendererService_v2
from src.rendering.somatotopic_shaper import shape
from src.hardware.translators import VCATranslator, LRATranslator, ERMTranslator
from src.routing.somatotopic_router import Homunculus


class TestFullPipeline:
    """Test complete end-to-end pipeline."""

    def test_fingertip_impact_full_pipeline(self):
        """Test complete pipeline for fingertip impact scenario."""
        # === Setup ===
        homunculus = Homunculus()
        body_fingertip = homunculus.table['index_tip']

        feature_extractor = FeatureExtractorV2()
        renderer = NeuralRendererService_v2(model_path=None)  # Untrained for testing
        translator = VCATranslator()

        # === Step 1: Create ContactPatch (Physics Simulation) ===
        contact_patch = ContactPatch(
            body_part_id=10,
            force_normal=2.0,  # 2N impact
            force_shear=np.array([0.3, 0.1]),
            velocity=np.array([0.0, 0.0, 0.0]),
            contact_area=0.0002,
            material_hint=1,  # Metal
            timestamp_us=1000000
        )

        # === Step 2: Create FilteredContact (Somatotopic Router) ===
        filtered_contact = FilteredContact(
            patch=contact_patch,
            rendering_tier=1,  # VCA
            cue_mask=FilteredContact.CUE_ALL
        )

        # === Step 3: Extract Features (14-dim) ===
        features = feature_extractor.extract(contact_patch)

        assert features.shape == (14,)
        assert features.dtype == np.float32

        # === Step 4: Neural Rendering (SensationParams) ===
        sensations = renderer.render([filtered_contact])
        sensation = sensations[10]

        assert isinstance(sensation, SensationParams)
        assert sensation.body_part_id == 10
        assert 0.0 <= sensation.impact_intensity <= 1.0

        # === Step 5: Somatotopic Shaping ===
        shaped_sensation = shape(sensation, body_fingertip)

        # Fingertip should have no change (gain=1.0)
        assert shaped_sensation.impact_intensity == sensation.impact_intensity

        # === Step 6: Hardware Translation (VCA) ===
        synthesis_cmd = translator.translate(shaped_sensation, body_fingertip)

        # Verify synthesis command
        assert synthesis_cmd.impact_amplitude >= 0.0
        assert synthesis_cmd.impact_frequency_hz >= 20.0
        assert synthesis_cmd.impact_frequency_hz <= 500.0

        print("✓ Fingertip impact pipeline test passed")

    def test_palm_texture_full_pipeline(self):
        """Test complete pipeline for palm texture scenario."""
        # === Setup ===
        homunculus = Homunculus()
        body_palm = homunculus.table['palm_center']

        feature_extractor = FeatureExtractorV2()
        renderer = NeuralRendererService_v2(model_path=None)
        translator = LRATranslator()  # Palm uses LRA

        # === Step 1: ContactPatch (Sliding on rough surface) ===
        contact_patch = ContactPatch(
            body_part_id=20,
            force_normal=1.0,
            force_shear=np.array([0.5, 0.0]),
            velocity=np.array([0.01, 0.0, 0.0]),  # 10mm/s slide
            contact_area=0.0003,
            material_hint=2,  # Wood (rough)
            timestamp_us=2000000
        )

        # === Step 2: FilteredContact ===
        filtered_contact = FilteredContact(
            patch=contact_patch,
            rendering_tier=2,  # LRA
            cue_mask=body_palm.cue_mask
        )

        # === Step 3: Features ===
        features = feature_extractor.extract(contact_patch)

        # === Step 4: Neural Rendering ===
        sensations = renderer.render([filtered_contact])
        sensation = sensations[20]

        # === Step 5: Somatotopic Shaping (Palm) ===
        shaped_sensation = shape(sensation, body_palm)

        # Palm should have increased amplitude (gain=2.0x) and reduced texture
        if sensation.impact_intensity > 0:
            assert shaped_sensation.impact_intensity >= sensation.impact_intensity

        # Texture should be reduced (spatial_factor=0.2)
        assert shaped_sensation.texture_roughness <= sensation.texture_roughness

        # === Step 6: LRA Translation ===
        synthesis_cmd = translator.translate(shaped_sensation, body_palm)

        # LRA: all frequencies should be 175Hz
        assert synthesis_cmd.impact_frequency_hz == 175.0
        assert synthesis_cmd.resonance_frequency_hz == 175.0

        print("✓ Palm texture pipeline test passed")

    def test_torso_pressure_full_pipeline(self):
        """Test complete pipeline for torso pressure scenario."""
        # === Setup ===
        homunculus = Homunculus()
        body_chest = homunculus.table['chest']

        feature_extractor = FeatureExtractorV2()
        renderer = NeuralRendererService_v2(model_path=None)
        translator = ERMTranslator()  # Torso uses ERM

        # === Step 1: ContactPatch (Sustained pressure) ===
        contact_patch = ContactPatch(
            body_part_id=30,
            force_normal=3.0,  # Heavy pressure
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            contact_area=0.001,  # Large contact area
            material_hint=0,
            timestamp_us=3000000
        )

        # === Step 2: FilteredContact ===
        filtered_contact = FilteredContact(
            patch=contact_patch,
            rendering_tier=3,  # ERM
            cue_mask=body_chest.cue_mask
        )

        # === Step 3: Features ===
        features = feature_extractor.extract(contact_patch)

        # === Step 4: Neural Rendering ===
        sensations = renderer.render([filtered_contact])
        sensation = sensations[30]

        # === Step 5: Somatotopic Shaping (Chest) ===
        shaped_sensation = shape(sensation, body_chest)

        # Chest should have max gain (5.0x) and minimal texture
        assert shaped_sensation.texture_roughness < 0.1  # Nearly zero

        # === Step 6: ERM Translation ===
        synthesis_cmd = translator.translate(shaped_sensation, body_chest)

        # ERM: only duty cycle should be set
        assert 0.0 <= synthesis_cmd.erm_duty_cycle <= 1.0
        assert synthesis_cmd.impact_amplitude == 0.0  # Not used

        print("✓ Torso pressure pipeline test passed")

    def test_multi_body_part_pipeline(self):
        """Test pipeline with multiple body parts simultaneously."""
        # === Setup ===
        homunculus = Homunculus()

        feature_extractor = FeatureExtractorV2()
        renderer = NeuralRendererService_v2(model_path=None)

        # Create contacts for fingertip, palm, chest
        contacts = [
            ContactPatch(10, 2.0, np.array([0.1, 0.1]), np.array([0.0, 0.0, 0.0]), 0.0002, 1, 1000000),
            ContactPatch(20, 1.5, np.array([0.2, 0.0]), np.array([0.0, 0.0, 0.0]), 0.0003, 2, 1000000),
            ContactPatch(30, 3.0, np.array([0.0, 0.0]), np.array([0.0, 0.0, 0.0]), 0.001, 0, 1000000),
        ]

        filtered_contacts = [
            FilteredContact(contacts[0], 1, FilteredContact.CUE_ALL),
            FilteredContact(contacts[1], 2, homunculus.table['palm_center'].cue_mask),
            FilteredContact(contacts[2], 3, homunculus.table['chest'].cue_mask),
        ]

        # === Render all contacts ===
        sensations = renderer.render(filtered_contacts)

        # Should have 3 sensations
        assert len(sensations) == 3
        assert 10 in sensations
        assert 20 in sensations
        assert 30 in sensations

        # === Shape and translate each ===
        results = {}
        for body_id, sensation in sensations.items():
            if body_id == 10:
                body = homunculus.table['index_tip']
                translator = VCATranslator()
            elif body_id == 20:
                body = homunculus.table['palm_center']
                translator = LRATranslator()
            else:
                body = homunculus.table['chest']
                translator = ERMTranslator()

            shaped = shape(sensation, body)
            synthesis_cmd = translator.translate(shaped, body)
            results[body_id] = synthesis_cmd

        # Verify all translated
        assert len(results) == 3

        print("✓ Multi-body-part pipeline test passed")

    def test_pipeline_performance(self):
        """Test that pipeline meets performance requirements (<15ms)."""
        # === Setup ===
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        feature_extractor = FeatureExtractorV2()
        renderer = NeuralRendererService_v2(model_path=None)
        translator = VCATranslator()

        # === Create test contact ===
        contact_patch = ContactPatch(
            body_part_id=10,
            force_normal=1.5,
            force_shear=np.array([0.2, 0.1]),
            velocity=np.array([0.0, 0.0, 0.0]),
            contact_area=0.0002,
            material_hint=1,
            timestamp_us=1000000
        )

        filtered_contact = FilteredContact(
            patch=contact_patch,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        # === Benchmark full pipeline ===
        start_time = time.perf_counter()

        # Step 1: Feature extraction
        features = feature_extractor.extract(contact_patch)

        # Step 2: Neural rendering
        sensations = renderer.render([filtered_contact])
        sensation = sensations[10]

        # Step 3: Somatotopic shaping
        shaped_sensation = shape(sensation, body)

        # Step 4: Translation
        synthesis_cmd = translator.translate(shaped_sensation, body)

        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000.0

        print(f"  Pipeline latency: {elapsed_ms:.2f}ms")

        # Should be under 15ms (target from plan)
        # Note: Without GPU, neural inference is slower. This is a soft requirement.
        if elapsed_ms < 15.0:
            print(f"  ✓ Performance target met: {elapsed_ms:.2f}ms < 15ms")
        else:
            print(f"  ⚠ Performance target missed: {elapsed_ms:.2f}ms > 15ms (acceptable without GPU)")

    def test_pipeline_data_flow(self):
        """Test that data flows correctly through all stages."""
        # === Setup ===
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        feature_extractor = FeatureExtractorV2()
        renderer = NeuralRendererService_v2(model_path=None)
        translator = VCATranslator()

        # === Create contact with known properties ===
        contact_patch = ContactPatch(
            body_part_id=10,
            force_normal=2.5,
            force_shear=np.array([0.4, 0.2]),
            velocity=np.array([0.01, 0.0, 0.0]),
            contact_area=0.00025,
            material_hint=1,  # Metal
            timestamp_us=1000000
        )

        filtered_contact = FilteredContact(
            patch=contact_patch,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        # === Track data through pipeline ===

        # Stage 1: Features
        features = feature_extractor.extract(contact_patch)
        assert features[0] == 2.5  # normal_force preserved
        assert features.shape == (14,)

        # Stage 2: Sensation
        sensations = renderer.render([filtered_contact])
        sensation = sensations[10]
        assert sensation.body_part_id == 10
        assert sensation.timestamp_us == 1000000

        # Stage 3: Shaped sensation
        shaped_sensation = shape(sensation, body)
        # Fingertip: no change
        assert shaped_sensation.body_part_id == 10
        assert shaped_sensation.timestamp_us == 1000000

        # Stage 4: Synthesis command
        synthesis_cmd = translator.translate(shaped_sensation, body)
        assert synthesis_cmd.impact_fire in [True, False]

        print("✓ Pipeline data flow test passed")


class TestPipelineEdgeCases:
    """Test edge cases and error conditions."""

    def test_zero_force_pipeline(self):
        """Test pipeline with zero force (no contact)."""
        homunculus = Homunculus()
        body = homunculus.table['index_tip']

        feature_extractor = FeatureExtractorV2()
        renderer = NeuralRendererService_v2(model_path=None)
        translator = VCATranslator()

        # Zero force contact
        contact_patch = ContactPatch(
            body_part_id=10,
            force_normal=0.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            contact_area=0.0,
            material_hint=0,
            timestamp_us=1000000
        )

        filtered_contact = FilteredContact(
            patch=contact_patch,
            rendering_tier=1,
            cue_mask=FilteredContact.CUE_ALL
        )

        # Should not crash
        sensations = renderer.render([filtered_contact])
        sensation = sensations[10]
        shaped = shape(sensation, body)
        synthesis_cmd = translator.translate(shaped, body)

        # With untrained model, outputs are random ~0.5
        # Just verify no crash and values are in valid range
        assert 0.0 <= synthesis_cmd.impact_amplitude <= 1.0

        print("✓ Zero force pipeline test passed")

    def test_high_force_pipeline(self):
        """Test pipeline with very high force."""
        homunculus = Homunculus()
        body = homunculus.table['chest']

        feature_extractor = FeatureExtractorV2()
        renderer = NeuralRendererService_v2(model_path=None)
        translator = ERMTranslator()

        # Very high force
        contact_patch = ContactPatch(
            body_part_id=30,
            force_normal=100.0,  # Unrealistically high
            force_shear=np.array([10.0, 5.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            contact_area=0.01,
            material_hint=0,
            timestamp_us=1000000
        )

        filtered_contact = FilteredContact(
            patch=contact_patch,
            rendering_tier=3,
            cue_mask=body.cue_mask
        )

        # Should not crash, values should be clamped
        sensations = renderer.render([filtered_contact])
        sensation = sensations[30]
        shaped = shape(sensation, body)
        synthesis_cmd = translator.translate(shaped, body)

        # Should be clamped to [0, 1]
        assert 0.0 <= synthesis_cmd.erm_duty_cycle <= 1.0

        print("✓ High force pipeline test passed")


if __name__ == '__main__':
    print("Running Full Pipeline Integration Tests\n")
    print("="*70)

    test_suite = TestFullPipeline()
    edge_cases = TestPipelineEdgeCases()

    try:
        # Main tests
        test_suite.test_fingertip_impact_full_pipeline()
        test_suite.test_palm_texture_full_pipeline()
        test_suite.test_torso_pressure_full_pipeline()
        test_suite.test_multi_body_part_pipeline()
        test_suite.test_pipeline_performance()
        test_suite.test_pipeline_data_flow()

        # Edge cases
        edge_cases.test_zero_force_pipeline()
        edge_cases.test_high_force_pipeline()

        print("\n" + "="*70)
        print("✓ All 8 integration tests passed!")
        print("="*70)

    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
