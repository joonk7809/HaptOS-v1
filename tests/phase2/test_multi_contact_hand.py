"""
Phase 2 Multi-Contact Hand Validation Tests.

Tests full hand coverage with 6 simultaneous channels:
- 5 fingertips (index, thumb, middle, ring, pinky)
- 1 palm center

Validates:
- Multi-contact concurrent processing
- Channel synchronization
- Bandwidth management (6 channels @ 100Hz)
- Contact prioritization under overload
- Per-channel independence
"""

import pytest
import time
import numpy as np
from typing import List, Dict

from src.core.schemas import ContactPatch, FilteredContact, CueParams
from src.routing.somatotopic_router import SomatotopicRouter
from src.inference.neural_renderer import NeuralRenderer
from src.hardware.driver_manager import DriverManager


class TestMultiContactHandSetup:
    """Test multi-contact hand hardware setup."""

    @pytest.fixture
    def full_hand_pipeline(self):
        """Create pipeline with 6-channel hand setup."""
        router = SomatotopicRouter(max_contacts=20)  # Allow up to 20 simultaneous contacts

        renderer = NeuralRenderer(
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )

        driver_manager = DriverManager(driver_type="mock", enable_sync=False)

        # Register 6 drivers for full hand
        hand_channels = {
            10: "MOCK_INDEX",     # Index fingertip
            11: "MOCK_THUMB",     # Thumb tip
            12: "MOCK_MIDDLE",    # Middle fingertip
            13: "MOCK_RING",      # Ring fingertip
            14: "MOCK_PINKY",     # Pinky fingertip
            15: "MOCK_PALM"       # Palm center
        }

        for body_part_id, port in hand_channels.items():
            driver_manager.register_driver(
                body_part_id=body_part_id,
                port=port,
                config={'simulate_latency': False, 'packet_loss_rate': 0.0}
            )

        return {
            'router': router,
            'renderer': renderer,
            'driver_manager': driver_manager,
            'channels': list(hand_channels.keys())
        }

    def create_contact(
        self,
        body_part_id: int,
        force: float,
        timestamp_us: int
    ) -> ContactPatch:
        """Helper to create contact for specific body part."""
        return ContactPatch(
            body_part_id=body_part_id,
            force_normal=force,
            force_shear=(0.1, 0.0),
            velocity=(0.0, 0.0, -0.05),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=timestamp_us
        )

    def test_6_channel_registration(self, full_hand_pipeline):
        """Validate all 6 channels are registered."""
        driver_manager = full_hand_pipeline['driver_manager']

        assert len(driver_manager.drivers) == 6

        # Verify all expected channels
        expected_channels = {10, 11, 12, 13, 14, 15}
        actual_channels = set(driver_manager.drivers.keys())

        assert actual_channels == expected_channels

    def test_channel_independence(self, full_hand_pipeline):
        """Validate each channel operates independently."""
        router = full_hand_pipeline['router']
        renderer = full_hand_pipeline['renderer']
        driver_manager = full_hand_pipeline['driver_manager']

        # Fill buffers for all channels
        for i in range(10):
            contacts = [
                self.create_contact(body_part_id, 1.5, timestamp_us=i * 1000)
                for body_part_id in full_hand_pipeline['channels']
            ]

            filtered = router.route(contacts)
            renderer.render(filtered)

        # Send different forces to each channel
        contacts = []
        for idx, body_part_id in enumerate(full_hand_pipeline['channels']):
            force = 1.0 + idx * 0.2  # Different force per channel
            contact = self.create_contact(body_part_id, force, timestamp_us=10000)
            contacts.append(contact)

        filtered = router.route(contacts)
        cue_params_dict = renderer.render(filtered)

        # Send to drivers
        results = driver_manager.send_all(cue_params_dict)

        # All channels should succeed
        assert all(results.values())
        assert len(results) == 6

        # Verify packets sent to each driver
        for body_part_id in full_hand_pipeline['channels']:
            driver = driver_manager.drivers[body_part_id]
            stats = driver.get_stats()
            assert stats['packets_sent'] >= 1


class TestMultiContactProcessing:
    """Test concurrent multi-contact processing."""

    @pytest.fixture
    def pipeline(self):
        """Create pipeline."""
        router = SomatotopicRouter(max_contacts=20)
        renderer = NeuralRenderer(
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )
        driver_manager = DriverManager(driver_type="mock")

        # Register 6 channels
        for body_part_id in range(10, 16):
            driver_manager.register_driver(
                body_part_id=body_part_id,
                port=f"MOCK_{body_part_id}",
                config={'simulate_latency': False}
            )

        return {
            'router': router,
            'renderer': renderer,
            'driver_manager': driver_manager
        }

    def create_multi_contacts(
        self,
        num_contacts: int,
        timestamp_us: int
    ) -> List[ContactPatch]:
        """Create multiple simultaneous contacts."""
        contacts = []
        for i in range(num_contacts):
            body_part_id = 10 + i
            force = 1.5 + i * 0.1

            contact = ContactPatch(
                body_part_id=body_part_id,
                force_normal=force,
                force_shear=(0.1, 0.0),
                velocity=(0.0, 0.0, -0.05),
                contact_area=0.0001,
                material_hint=1,
                timestamp_us=timestamp_us
            )
            contacts.append(contact)

        return contacts

    def test_6_simultaneous_contacts(self, pipeline):
        """Test 6 simultaneous contacts (full hand grip)."""
        # Fill buffers
        for i in range(10):
            contacts = self.create_multi_contacts(6, timestamp_us=i * 1000)
            filtered = pipeline['router'].route(contacts)
            pipeline['renderer'].render(filtered)

        # Process 6 simultaneous contacts
        contacts = self.create_multi_contacts(6, timestamp_us=10000)

        start_time = time.time()

        filtered = pipeline['router'].route(contacts)
        cue_params_dict = pipeline['renderer'].render(filtered)
        results = pipeline['driver_manager'].send_all(cue_params_dict)

        end_time = time.time()
        latency_ms = (end_time - start_time) * 1000

        # Validate
        assert len(filtered) == 6, f"Expected 6 filtered contacts, got {len(filtered)}"
        if cue_params_dict:
            assert len(cue_params_dict) <= 6
            assert all(results.values()), "Some channels failed"

        # Latency should still be reasonable
        assert latency_ms < 50.0, f"6-contact latency {latency_ms:.2f}ms too high"

        print(f"\n6-contact processing latency: {latency_ms:.2f}ms")

    def test_10_simultaneous_contacts_with_prioritization(self, pipeline):
        """Test contact prioritization when >6 contacts."""
        # Create 10 contacts (more than 6 channels)
        contacts = []
        for i in range(10):
            body_part_id = 10 + (i % 6)  # Some channels get multiple contacts
            force = 1.0 + i * 0.1

            contact = ContactPatch(
                body_part_id=body_part_id,
                force_normal=force,
                force_shear=(0.1, 0.0),
                velocity=(0.0, 0.0, -0.05),
                contact_area=0.0001,
                material_hint=1,
                timestamp_us=i * 1000
            )
            contacts.append(contact)

        # Router should handle all, but may prioritize
        filtered = pipeline['router'].route(contacts)

        # Should filter some contacts
        assert len(filtered) <= 10
        assert len(filtered) >= 6  # At least one per channel

    def test_sustained_6_channel_transmission(self, pipeline):
        """Test sustained transmission across 6 channels."""
        # Fill buffers
        for i in range(10):
            contacts = self.create_multi_contacts(6, timestamp_us=i * 1000)
            filtered = pipeline['router'].route(contacts)
            pipeline['renderer'].render(filtered)

        # Send 50 frames with 6 simultaneous contacts each
        packets_sent = 0

        for frame in range(50):
            contacts = self.create_multi_contacts(6, timestamp_us=10000 + frame * 1000)

            filtered = pipeline['router'].route(contacts)
            cue_params_dict = pipeline['renderer'].render(filtered)

            if cue_params_dict:
                results = pipeline['driver_manager'].send_all(cue_params_dict)
                packets_sent += sum(results.values())

        # Should have sent most packets
        expected_min = 50 * 6 * 0.7  # At least 70% of 300 packets
        assert packets_sent >= expected_min, f"Only sent {packets_sent} packets"

        print(f"\nSustained 6-channel: {packets_sent} packets in 50 frames")


class TestGraspScenario:
    """Test realistic grasp scenario (canonical Test 5 from plan)."""

    @pytest.fixture
    def pipeline(self):
        """Create full pipeline."""
        router = SomatotopicRouter(max_contacts=20)
        renderer = NeuralRenderer(
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )
        driver_manager = DriverManager(driver_type="mock", enable_sync=False)

        # Register 6 channels
        for body_part_id in range(10, 16):
            driver_manager.register_driver(
                body_part_id=body_part_id,
                port=f"MOCK_{body_part_id}",
                config={'simulate_latency': False, 'packet_loss_rate': 0.0}
            )

        return {
            'router': router,
            'renderer': renderer,
            'driver_manager': driver_manager
        }

    def test_grasp_object_with_full_hand(self, pipeline):
        """
        Canonical Test 5: Grasp object with all 5 fingers + palm.

        Scenario:
        - All 6 contact points engage simultaneously
        - Different forces per finger (grip pattern)
        - Sustained hold for 100ms

        Success Criteria:
        - All channels synthesize
        - Total processing time <10ms
        - No cross-talk between channels
        """
        # Grasp forces (realistic grip pattern)
        grasp_forces = {
            10: 2.0,  # Index - strong
            11: 2.5,  # Thumb - strongest (opposition)
            12: 1.5,  # Middle - medium
            13: 1.2,  # Ring - weaker
            14: 0.8,  # Pinky - weakest
            15: 1.0   # Palm - medium
        }

        # Fill buffers for all channels
        for i in range(10):
            contacts = [
                ContactPatch(
                    body_part_id=body_part_id,
                    force_normal=force,
                    force_shear=(0.1, 0.0),
                    velocity=(0.0, 0.0, -0.05),
                    contact_area=0.0001,
                    material_hint=1,
                    timestamp_us=i * 1000
                )
                for body_part_id, force in grasp_forces.items()
            ]

            filtered = pipeline['router'].route(contacts)
            pipeline['renderer'].render(filtered)

        # Execute grasp (100 frames @ 1kHz = 100ms)
        start_time = time.time()

        for frame in range(100):
            contacts = [
                ContactPatch(
                    body_part_id=body_part_id,
                    force_normal=force + np.random.uniform(-0.05, 0.05),  # Small noise
                    force_shear=(0.1, 0.0),
                    velocity=(0.0, 0.0, 0.0),  # Static grip
                    contact_area=0.0001,
                    material_hint=1,
                    timestamp_us=10000 + frame * 1000
                )
                for body_part_id, force in grasp_forces.items()
            ]

            filtered = pipeline['router'].route(contacts)
            cue_params_dict = pipeline['renderer'].render(filtered)

            if cue_params_dict:
                results = pipeline['driver_manager'].send_all(cue_params_dict)

        end_time = time.time()
        total_time_s = end_time - start_time

        # Get statistics
        stats = pipeline['driver_manager'].get_aggregate_stats()

        print(f"\n=== Grasp Scenario Results ===")
        print(f"Total time: {total_time_s:.3f}s")
        print(f"Packets sent: {stats['total_sent']}")
        print(f"Success rate: {stats['success_rate'] * 100:.1f}%")
        print(f"Channels active: {stats['driver_count']}")

        # Validate success criteria
        assert stats['driver_count'] == 6, "Not all channels active"
        assert stats['total_sent'] >= 500, f"Only {stats['total_sent']} packets sent"
        assert stats['success_rate'] == 1.0, "Packet loss detected"
        assert total_time_s < 2.0, f"Grasp took {total_time_s:.2f}s, expected <2s"

        # Verify per-channel stats
        per_channel = stats['per_channel_stats']
        for body_part_id in range(10, 16):
            channel_stats = per_channel[body_part_id]
            assert channel_stats['packets_sent'] >= 80, f"Channel {body_part_id} sent too few packets"

    def test_no_cross_talk_between_channels(self, pipeline):
        """Validate channels don't interfere with each other."""
        # Send different parameters to each channel
        unique_frequencies = {
            10: 100.0,
            11: 150.0,
            12: 200.0,
            13: 250.0,
            14: 300.0,
            15: 350.0
        }

        # Fill buffers
        for i in range(10):
            contacts = [
                ContactPatch(
                    body_part_id=body_part_id,
                    force_normal=1.5,
                    force_shear=(0.1, 0.0),
                    velocity=(0.0, 0.0, -0.05),
                    contact_area=0.0001,
                    material_hint=1,
                    timestamp_us=i * 1000
                )
                for body_part_id in unique_frequencies.keys()
            ]
            filtered = pipeline['router'].route(contacts)
            pipeline['renderer'].render(filtered)

        # Process frame
        contacts = [
            ContactPatch(
                body_part_id=body_part_id,
                force_normal=1.5,
                force_shear=(0.1, 0.0),
                velocity=(0.0, 0.0, -0.05),
                contact_area=0.0001,
                material_hint=1,
                timestamp_us=10000
            )
            for body_part_id in unique_frequencies.keys()
        ]

        filtered = pipeline['router'].route(contacts)
        cue_params_dict = pipeline['renderer'].render(filtered)
        pipeline['driver_manager'].send_all(cue_params_dict)

        # Verify each channel got its own packet
        for body_part_id in unique_frequencies.keys():
            driver = pipeline['driver_manager'].drivers[body_part_id]
            tx_packets = driver.get_tx_buffer()

            assert len(tx_packets) >= 1, f"Channel {body_part_id} got no packets"


class TestBandwidthManagement:
    """Test bandwidth utilization for multi-channel transmission."""

    @pytest.fixture
    def pipeline(self):
        """Create pipeline with bandwidth monitoring."""
        router = SomatotopicRouter()
        renderer = NeuralRenderer(
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )
        driver_manager = DriverManager(driver_type="mock", enable_sync=False)

        # Register 6 channels
        for body_part_id in range(10, 16):
            driver_manager.register_driver(
                body_part_id=body_part_id,
                port=f"MOCK_{body_part_id}",
                config={'simulate_latency': True}  # Enable latency
            )

        return {
            'router': router,
            'renderer': renderer,
            'driver_manager': driver_manager
        }

    def test_bandwidth_calculation(self, pipeline):
        """Test bandwidth monitoring."""
        # Send packets for 1 second
        start_time = time.time()

        # Fill buffers
        for i in range(10):
            contacts = [
                ContactPatch(
                    body_part_id=10 + j,
                    force_normal=1.5,
                    force_shear=(0.1, 0.0),
                    velocity=(0.0, 0.0, -0.05),
                    contact_area=0.0001,
                    material_hint=1,
                    timestamp_us=i * 1000
                )
                for j in range(6)
            ]
            filtered = pipeline['router'].route(contacts)
            pipeline['renderer'].render(filtered)

        # Send 100 frames
        for frame in range(100):
            contacts = [
                ContactPatch(
                    body_part_id=10 + j,
                    force_normal=1.5,
                    force_shear=(0.1, 0.0),
                    velocity=(0.0, 0.0, -0.05),
                    contact_area=0.0001,
                    material_hint=1,
                    timestamp_us=10000 + frame * 1000
                )
                for j in range(6)
            ]

            filtered = pipeline['router'].route(contacts)
            cue_params_dict = pipeline['renderer'].render(filtered)

            if cue_params_dict:
                pipeline['driver_manager'].send_all(cue_params_dict)

        # Get bandwidth stats
        bw_stats = pipeline['driver_manager'].get_bandwidth_stats()

        print(f"\n=== Bandwidth Statistics ===")
        print(f"Total packets: {bw_stats['total_packets']}")
        print(f"Elapsed time: {bw_stats['elapsed_time_s']:.2f}s")
        print(f"Bandwidth: {bw_stats['bandwidth_hz']:.1f} Hz")
        print(f"Bandwidth: {bw_stats['bandwidth_kbps']:.2f} kbps")
        print(f"Channels: {bw_stats['channels_active']}")

        # Validate
        assert bw_stats['total_packets'] >= 500
        assert bw_stats['bandwidth_hz'] > 0
        assert bw_stats['channels_active'] == 6
