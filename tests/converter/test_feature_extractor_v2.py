#!/usr/bin/env python3
"""
Unit tests for FeatureExtractorV2 (14-dimensional feature extraction).

Tests cover:
- 14-dim output shape
- force_delta calculation correctness
- contact_area estimation (order of magnitude check)
- contact_duration tracking
- History buffer management (10 frames)
- Phase transition handling
- Material property lookup
- Reset functionality
"""

import pytest
import numpy as np
from src.converter.feature_extractor_v2 import FeatureExtractorV2, Phase
from src.core.schemas import ContactPatch


class TestFeatureExtractorV2Shape:
    """Test output shape and dimensionality."""

    def test_output_is_14_dimensional(self):
        """Test that extract() returns a 14-dimensional vector."""
        extractor = FeatureExtractorV2()

        contact = ContactPatch(
            body_part_id=10,
            force_normal=1.0,
            force_shear=np.array([0.1, 0.2]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        assert features.shape == (14,), f"Expected shape (14,), got {features.shape}"
        assert features.dtype == np.float32

    def test_feature_names_match_dimensions(self):
        """Test that get_feature_names() returns 14 names."""
        extractor = FeatureExtractorV2()
        names = extractor.get_feature_names()

        assert len(names) == 14
        assert names[0] == 'normal_force'
        assert names[2] == 'force_delta'
        assert names[12] == 'contact_duration'
        assert names[13] == 'contact_area'


class TestForceDeltaCalculation:
    """Test force_delta (F_t - F_{t-10}) calculation."""

    def test_force_delta_at_initialization(self):
        """Test that force_delta is 0 at first contact (no history)."""
        extractor = FeatureExtractorV2()

        contact = ContactPatch(
            body_part_id=10,
            force_normal=1.5,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        # force_delta = current_force - force_10ms_ago
        # At initialization, force_10ms_ago = 0.0
        # So force_delta = 1.5 - 0.0 = 1.5
        assert abs(features[2] - 1.5) < 1e-6

    def test_force_delta_after_10_frames(self):
        """Test force_delta after 10 frames (full buffer)."""
        extractor = FeatureExtractorV2()
        body_part_id = 10

        # Fill buffer with 10 frames (force = 1.0)
        for i in range(10):
            contact = ContactPatch(
                body_part_id=body_part_id,
                force_normal=1.0,
                force_shear=np.array([0.0, 0.0]),
                velocity=np.array([0.0, 0.0, 0.0]),
                material_hint=0,
                timestamp_us=1000000 + i * 1000
            )
            extractor.extract(contact)

        # Now apply a new force (2.5N)
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=2.5,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1010000
        )

        features = extractor.extract(contact)

        # force_delta = 2.5 - 1.0 = 1.5
        assert abs(features[2] - 1.5) < 1e-6

    def test_force_delta_rising_edge(self):
        """Test force_delta detects rising force edge."""
        extractor = FeatureExtractorV2()
        body_part_id = 10

        # Initial no contact (10 frames at 0.0N)
        for i in range(10):
            contact = ContactPatch(
                body_part_id=body_part_id,
                force_normal=0.0,
                force_shear=np.array([0.0, 0.0]),
                velocity=np.array([0.0, 0.0, 0.0]),
                material_hint=0,
                timestamp_us=1000000 + i * 1000
            )
            extractor.extract(contact)

        # Impact: force jumps to 3.0N
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=3.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1010000
        )

        features = extractor.extract(contact)

        # force_delta = 3.0 - 0.0 = 3.0 (large positive = impact)
        assert abs(features[2] - 3.0) < 1e-6
        assert features[2] > 0.5  # Strong impact signature

    def test_force_delta_per_body_part(self):
        """Test that force_delta is tracked independently per body part."""
        extractor = FeatureExtractorV2()

        # Body part 10: force = 1.0N
        for i in range(10):
            contact = ContactPatch(
                body_part_id=10,
                force_normal=1.0,
                force_shear=np.array([0.0, 0.0]),
                velocity=np.array([0.0, 0.0, 0.0]),
                material_hint=0,
                timestamp_us=1000000 + i * 1000
            )
            extractor.extract(contact)

        # Body part 20: force = 2.0N
        for i in range(10):
            contact = ContactPatch(
                body_part_id=20,
                force_normal=2.0,
                force_shear=np.array([0.0, 0.0]),
                velocity=np.array([0.0, 0.0, 0.0]),
                material_hint=0,
                timestamp_us=1000000 + i * 1000
            )
            extractor.extract(contact)

        # New contact for body part 10: force = 4.0N
        contact_10 = ContactPatch(
            body_part_id=10,
            force_normal=4.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1010000
        )
        features_10 = extractor.extract(contact_10)

        # force_delta for body 10 = 4.0 - 1.0 = 3.0
        assert abs(features_10[2] - 3.0) < 1e-6

        # New contact for body part 20: force = 5.0N
        contact_20 = ContactPatch(
            body_part_id=20,
            force_normal=5.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1010000
        )
        features_20 = extractor.extract(contact_20)

        # force_delta for body 20 = 5.0 - 2.0 = 3.0
        assert abs(features_20[2] - 3.0) < 1e-6


class TestContactAreaEstimation:
    """Test contact_area estimation (Hertzian contact)."""

    def test_contact_area_zero_force(self):
        """Test that contact_area is 0 when force is 0."""
        extractor = FeatureExtractorV2()

        contact = ContactPatch(
            body_part_id=10,
            force_normal=0.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        # contact_area should be 0 for zero force
        assert features[13] == 0.0

    def test_contact_area_order_of_magnitude(self):
        """Test that contact_area is in reasonable range [1e-6, 1e-3] m²."""
        extractor = FeatureExtractorV2()

        # Typical finger contact: 1.0N on rubber (k=5e3 N/m)
        contact = ContactPatch(
            body_part_id=10,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=4,  # Rubber
            timestamp_us=1000000
        )

        features = extractor.extract(contact)
        contact_area = features[13]

        # Expected: A ≈ π × (sqrt(F/k))² = π × (sqrt(1.0/5e3))²
        # A ≈ π × (0.0141)² ≈ 6.28e-4 m²
        assert 1e-6 < contact_area < 1e-3, f"Contact area {contact_area} out of range"
        assert abs(contact_area - 6.28e-4) < 1e-4

    def test_contact_area_increases_with_force(self):
        """Test that contact_area increases monotonically with force."""
        extractor = FeatureExtractorV2()

        forces = [0.5, 1.0, 2.0, 5.0]
        areas = []

        for force in forces:
            contact = ContactPatch(
                body_part_id=10,
                force_normal=force,
                force_shear=np.array([0.0, 0.0]),
                velocity=np.array([0.0, 0.0, 0.0]),
                material_hint=0,
                timestamp_us=1000000
            )
            features = extractor.extract(contact)
            areas.append(features[13])

        # Areas should be monotonically increasing
        for i in range(len(areas) - 1):
            assert areas[i+1] > areas[i], f"Area not increasing: {areas}"

    def test_contact_area_material_dependence(self):
        """Test that contact_area varies with material stiffness."""
        extractor = FeatureExtractorV2()

        # Same force, different materials
        materials = [
            (4, 5e3),   # Rubber (soft)
            (2, 1e4),   # Wood (medium)
            (1, 5e4),   # Metal (stiff)
        ]

        areas = []

        for material_hint, stiffness in materials:
            contact = ContactPatch(
                body_part_id=10,
                force_normal=1.0,
                force_shear=np.array([0.0, 0.0]),
                velocity=np.array([0.0, 0.0, 0.0]),
                material_hint=material_hint,
                timestamp_us=1000000
            )
            features = extractor.extract(contact)
            areas.append(features[13])

        # Softer materials have larger contact area
        # Rubber > Wood > Metal
        assert areas[0] > areas[1] > areas[2], f"Areas: {areas}"


class TestContactDurationTracking:
    """Test contact_duration tracking."""

    def test_contact_duration_starts_at_zero(self):
        """Test that contact_duration is 0 at first contact."""
        extractor = FeatureExtractorV2()

        contact = ContactPatch(
            body_part_id=10,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        # First contact: duration = 0
        assert features[12] == 0.0

    def test_contact_duration_accumulates(self):
        """Test that contact_duration accumulates during sustained contact."""
        extractor = FeatureExtractorV2()
        body_part_id = 10

        # First contact at t=0
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )
        features = extractor.extract(contact)
        assert features[12] == 0.0

        # Contact at t=50ms
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1050000  # +50ms
        )
        features = extractor.extract(contact)
        assert abs(features[12] - 50.0) < 1e-3  # 50ms

        # Contact at t=200ms
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1200000  # +200ms
        )
        features = extractor.extract(contact)
        assert abs(features[12] - 200.0) < 1e-3  # 200ms

    def test_contact_duration_resets_on_no_contact(self):
        """Test that contact_duration resets when contact breaks."""
        extractor = FeatureExtractorV2()
        body_part_id = 10

        # First contact
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )
        extractor.extract(contact)

        # Contact at t=100ms
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1100000
        )
        features = extractor.extract(contact)
        assert abs(features[12] - 100.0) < 1e-3

        # Break contact
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=0.0,  # NO_CONTACT
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1200000
        )
        features = extractor.extract(contact)
        assert features[12] == 0.0  # Duration reset

        # New contact
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1250000
        )
        features = extractor.extract(contact)
        assert features[12] == 0.0  # New contact starts at 0


class TestPhaseDetection:
    """Test FSM phase detection."""

    def test_no_contact_phase(self):
        """Test NO_CONTACT phase (force < 0.01N)."""
        extractor = FeatureExtractorV2()

        contact = ContactPatch(
            body_part_id=10,
            force_normal=0.005,  # Below threshold
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        # Phase one-hot: [impact, hold, slip, release]
        # NO_CONTACT → all zeros
        assert features[8] == 0.0  # phase_impact
        assert features[9] == 0.0  # phase_hold
        assert features[10] == 0.0  # phase_slip
        assert features[11] == 0.0  # phase_release

    def test_impact_phase(self):
        """Test IMPACT phase (rising force edge)."""
        extractor = FeatureExtractorV2()
        body_part_id = 10

        # Start with NO_CONTACT
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=0.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )
        extractor.extract(contact)

        # Impact: force jumps above threshold
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,  # Above IMPACT_FORCE_THRESHOLD (0.5N)
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1010000
        )

        features = extractor.extract(contact)

        # IMPACT → one-hot [1, 0, 0, 0]
        assert features[8] == 1.0  # phase_impact
        assert features[9] == 0.0
        assert features[10] == 0.0
        assert features[11] == 0.0

    def test_hold_phase(self):
        """Test HOLD phase (steady force after HOLD_TIME_MS)."""
        extractor = FeatureExtractorV2()
        body_part_id = 10

        # Impact
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )
        extractor.extract(contact)

        # Wait > HOLD_TIME_MS (20ms) with low slip speed
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.001, 0.001, 0.0]),  # <10mm/s
            material_hint=0,
            timestamp_us=1030000  # +30ms
        )

        features = extractor.extract(contact)

        # HOLD → one-hot [0, 1, 0, 0]
        assert features[8] == 0.0
        assert features[9] == 1.0  # phase_hold
        assert features[10] == 0.0
        assert features[11] == 0.0

    def test_slip_phase(self):
        """Test SLIP phase (high slip speed)."""
        extractor = FeatureExtractorV2()
        body_part_id = 10

        # Start with contact
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.2, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )
        extractor.extract(contact)

        # High slip speed (>10mm/s)
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.5, 0.0]),
            velocity=np.array([0.015, 0.0, 0.0]),  # 15mm/s
            material_hint=0,
            timestamp_us=1010000
        )

        features = extractor.extract(contact)

        # SLIP → one-hot [0, 0, 1, 0]
        assert features[8] == 0.0
        assert features[9] == 0.0
        assert features[10] == 1.0  # phase_slip
        assert features[11] == 0.0

    def test_release_phase(self):
        """Test RELEASE phase (falling force edge)."""
        extractor = FeatureExtractorV2()
        body_part_id = 10

        # HOLD phase
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )
        extractor.extract(contact)

        # Wait to enter HOLD
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1030000
        )
        extractor.extract(contact)

        # Release: force drops below threshold
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=0.2,  # Below IMPACT_FORCE_THRESHOLD * 0.5 = 0.25N
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1050000
        )

        features = extractor.extract(contact)

        # RELEASE → one-hot [0, 0, 0, 1]
        assert features[8] == 0.0
        assert features[9] == 0.0
        assert features[10] == 0.0
        assert features[11] == 1.0  # phase_release


class TestMaterialProperties:
    """Test material property lookup."""

    def test_default_material(self):
        """Test default material properties (material_hint=0)."""
        extractor = FeatureExtractorV2()

        contact = ContactPatch(
            body_part_id=10,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,  # Unknown/default
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        # Default: (hardness=0.5, friction=0.5, roughness=0.3)
        assert features[5] == 0.5  # hardness
        assert features[6] == 0.5  # friction
        assert features[7] == 0.3  # roughness

    def test_metal_properties(self):
        """Test metal material properties (material_hint=1)."""
        extractor = FeatureExtractorV2()

        contact = ContactPatch(
            body_part_id=10,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=1,  # Metal
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        # Metal: (hardness=0.9, friction=0.2, roughness=0.2)
        assert features[5] == 0.9  # hardness
        assert features[6] == 0.2  # friction
        assert features[7] == 0.2  # roughness

    def test_rubber_properties(self):
        """Test rubber material properties (material_hint=4)."""
        extractor = FeatureExtractorV2()

        contact = ContactPatch(
            body_part_id=10,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=4,  # Rubber
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        # Rubber: (hardness=0.2, friction=0.8, roughness=0.4)
        assert features[5] == 0.2  # hardness
        assert features[6] == 0.8  # friction
        assert features[7] == 0.4  # roughness


class TestResetFunctionality:
    """Test reset() method."""

    def test_reset_all(self):
        """Test reset() clears all state."""
        extractor = FeatureExtractorV2()

        # Create some history
        for i in range(5):
            contact = ContactPatch(
                body_part_id=10,
                force_normal=1.0,
                force_shear=np.array([0.1, 0.2]),
                velocity=np.array([0.01, 0.0, 0.0]),
                material_hint=0,
                timestamp_us=1000000 + i * 10000
            )
            extractor.extract(contact)

        # Verify state exists
        assert 10 in extractor.force_history
        assert 10 in extractor.contact_start_time

        # Reset all
        extractor.reset()

        # All state cleared
        assert len(extractor.force_history) == 0
        assert len(extractor.contact_start_time) == 0
        assert len(extractor.prev_phase) == 0
        assert len(extractor.phase_start_time) == 0
        assert extractor.last_features is None

    def test_reset_specific_body_part(self):
        """Test reset(body_part_id) clears only that body part."""
        extractor = FeatureExtractorV2()

        # Create history for two body parts
        for body_part_id in [10, 20]:
            for i in range(5):
                contact = ContactPatch(
                    body_part_id=body_part_id,
                    force_normal=1.0,
                    force_shear=np.array([0.0, 0.0]),
                    velocity=np.array([0.0, 0.0, 0.0]),
                    material_hint=0,
                    timestamp_us=1000000 + i * 10000
                )
                extractor.extract(contact)

        # Verify both exist
        assert 10 in extractor.force_history
        assert 20 in extractor.force_history

        # Reset only body part 10
        extractor.reset(body_part_id=10)

        # Body part 10 cleared, 20 remains
        assert 10 in extractor.force_history  # Re-initialized to zeros
        assert 20 in extractor.force_history
        assert all(f == 0.0 for f in extractor.force_history[10])

    def test_reset_body_part_after_extraction(self):
        """Test that reset properly re-initializes history buffer."""
        extractor = FeatureExtractorV2()
        body_part_id = 10

        # Create history
        for i in range(10):
            contact = ContactPatch(
                body_part_id=body_part_id,
                force_normal=2.0,
                force_shear=np.array([0.0, 0.0]),
                velocity=np.array([0.0, 0.0, 0.0]),
                material_hint=0,
                timestamp_us=1000000 + i * 1000
            )
            extractor.extract(contact)

        # Reset
        extractor.reset(body_part_id=body_part_id)

        # New extraction should have force_delta = force (no history)
        contact = ContactPatch(
            body_part_id=body_part_id,
            force_normal=3.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=2000000
        )
        features = extractor.extract(contact)

        # force_delta = 3.0 - 0.0 = 3.0 (buffer was reset)
        assert abs(features[2] - 3.0) < 1e-6


class TestEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_negative_force_clamped(self):
        """Test that negative forces are handled gracefully."""
        extractor = FeatureExtractorV2()

        contact = ContactPatch(
            body_part_id=10,
            force_normal=-0.5,  # Negative force (simulation artifact)
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        # Should be treated as NO_CONTACT
        assert features[8] == 0.0  # No phase active
        assert features[12] == 0.0  # Duration = 0

    def test_very_high_force(self):
        """Test that very high forces don't break calculations."""
        extractor = FeatureExtractorV2()

        contact = ContactPatch(
            body_part_id=10,
            force_normal=100.0,  # Very high force
            force_shear=np.array([5.0, 5.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=0,
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        # Should compute without errors
        assert features[0] == 100.0  # normal_force
        assert features[1] > 0.0  # shear_magnitude
        assert features[13] > 0.0  # contact_area
        assert not np.isnan(features).any()
        assert not np.isinf(features).any()

    def test_zero_stiffness_material(self):
        """Test that missing material stiffness uses default."""
        extractor = FeatureExtractorV2()

        # Use unknown material hint
        contact = ContactPatch(
            body_part_id=10,
            force_normal=1.0,
            force_shear=np.array([0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            material_hint=99,  # Unknown material
            timestamp_us=1000000
        )

        features = extractor.extract(contact)

        # Should use default material properties
        assert features[5] == 0.5  # default hardness
        assert features[13] > 0.0  # contact_area computed with default stiffness


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
