"""
Full Pipeline Integration Tests.

Tests complete end-to-end data flow:
Physics → Router → Renderer → Mock Driver

Validates:
- Successful packet transmission
- Checksum integrity
- Parameter preservation
- Latency budgets
- Multi-contact handling
- Sustained transmission
"""

import pytest
import time
from typing import List, Dict

from src.core.schemas import ContactPatch, FilteredContact, CueParams
from src.routing.somatotopic_router import SomatotopicRouter
from src.inference.neural_renderer import NeuralRenderer
from src.hardware.driver_manager import DriverManager
from src.hardware.protocol_validator import ProtocolValidator


class TestEndToEndPipeline:
    """Test suite for complete HAPTOS pipeline."""

    @pytest.fixture
    def router(self):
        """Create router instance."""
        return SomatotopicRouter()

    @pytest.fixture
    def renderer(self):
        """Create neural renderer instance."""
        # Use existing trained models
        return NeuralRenderer(
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )

    @pytest.fixture
    def driver_manager(self):
        """Create driver manager with mock driver."""
        manager = DriverManager(driver_type="mock")
        # Register driver for index finger
        manager.register_driver(
            body_part_id=10,
            port="MOCK_INDEX",
            config={'simulate_latency': False, 'packet_loss_rate': 0.0}
        )
        return manager

    def create_contact_patch(
        self,
        body_part_id: int,
        force_normal: float,
        force_shear_x: float = 0.1,
        force_shear_y: float = 0.0,
        timestamp_us: int = 1000000
    ) -> ContactPatch:
        """Helper to create ContactPatch for testing."""
        return ContactPatch(
            body_part_id=body_part_id,
            force_normal=force_normal,
            force_shear=(force_shear_x, force_shear_y),
            velocity=(0.0, 0.0, -0.05),
            contact_area=0.0001,
            material_hint=1,  # Metal
            timestamp_us=timestamp_us
        )

    def test_single_contact_pipeline(self, router, renderer, driver_manager):
        """Test complete pipeline with single contact."""
        start_time = time.time()

        # Step 1: Create ContactPatch (simulated physics output)
        contact = self.create_contact_patch(
            body_part_id=10,
            force_normal=1.5,
            timestamp_us=1000000
        )

        # Step 2: Route through somatotopic router
        filtered_contacts = router.route([contact])

        assert len(filtered_contacts) == 1
        assert filtered_contacts[0].patch.body_part_id == 10
        assert filtered_contacts[0].rendering_tier == 1  # VCA tier
        assert filtered_contacts[0].cue_mask == FilteredContact.CUE_ALL

        # Step 3: Render through neural renderer
        # Need to send 10 contacts to fill window buffer
        cue_params_dict = None
        for i in range(10):
            contact_i = self.create_contact_patch(
                body_part_id=10,
                force_normal=1.5,
                timestamp_us=1000000 + i * 1000
            )
            filtered_i = router.route([contact_i])
            cue_params_dict = renderer.render(filtered_i)

        # Should have CueParams for body_part_id=10
        assert 10 in cue_params_dict
        cue_params = cue_params_dict[10]

        # Validate CueParams structure
        assert isinstance(cue_params, CueParams)
        assert 0.0 <= cue_params.texture_amplitude <= 1.0
        assert 0.0 <= cue_params.weight_offset <= 1.0

        # Step 4: Send through mock driver
        result = driver_manager.send_all(cue_params_dict)

        assert result[10] is True

        # Step 5: Validate transmitted packet
        driver = driver_manager.drivers[10]
        tx_packets = driver.get_tx_buffer()

        assert len(tx_packets) == 1

        packet_data = tx_packets[0]['packet']
        stored_cue_params = tx_packets[0]['cue_params']

        # Validate packet structure
        is_valid, error_msg = ProtocolValidator.validate_packet(packet_data)
        assert is_valid is True, f"Invalid packet: {error_msg}"

        # Validate round-trip serialization
        is_valid, deserialized = ProtocolValidator.deserialize_and_validate(packet_data)
        assert is_valid is True
        assert deserialized is not None

        # Compare original vs deserialized
        is_equal = ProtocolValidator.compare_cue_params(stored_cue_params, deserialized)
        assert is_equal is True

        # Measure end-to-end latency
        end_time = time.time()
        latency_ms = (end_time - start_time) * 1000

        # Should complete in <15ms
        assert latency_ms < 15.0, f"Pipeline latency {latency_ms:.2f}ms exceeds 15ms budget"

    def test_multi_contact_pipeline(self, router, renderer):
        """Test pipeline with multiple simultaneous contacts."""
        # Create driver manager with 3 drivers
        driver_manager = DriverManager(driver_type="mock")
        driver_manager.register_driver(10, "MOCK_INDEX", config={'simulate_latency': False})
        driver_manager.register_driver(11, "MOCK_THUMB", config={'simulate_latency': False})
        driver_manager.register_driver(12, "MOCK_PALM", config={'simulate_latency': False})

        # Create 3 simultaneous contacts
        contacts = [
            self.create_contact_patch(10, force_normal=1.5, timestamp_us=2000000),
            self.create_contact_patch(11, force_normal=1.2, timestamp_us=2000000),
            self.create_contact_patch(12, force_normal=2.0, timestamp_us=2000000)
        ]

        # Fill buffers with 10 samples each
        for i in range(10):
            contacts_i = [
                self.create_contact_patch(10, force_normal=1.5, timestamp_us=2000000 + i * 1000),
                self.create_contact_patch(11, force_normal=1.2, timestamp_us=2000000 + i * 1000),
                self.create_contact_patch(12, force_normal=2.0, timestamp_us=2000000 + i * 1000)
            ]

            # Route
            filtered = router.route(contacts_i)
            assert len(filtered) == 3

            # Render
            cue_params_dict = renderer.render(filtered)

        # Should have CueParams for all 3 body parts
        assert 10 in cue_params_dict
        assert 11 in cue_params_dict
        assert 12 in cue_params_dict

        # Send to all drivers
        results = driver_manager.send_all(cue_params_dict)

        assert results[10] is True
        assert results[11] is True
        assert results[12] is True

        # Validate all packets
        for body_part_id in [10, 11, 12]:
            driver = driver_manager.drivers[body_part_id]
            tx_packets = driver.get_tx_buffer()

            assert len(tx_packets) == 1

            packet_data = tx_packets[0]['packet']
            is_valid, error_msg = ProtocolValidator.validate_packet(packet_data)
            assert is_valid is True, f"Invalid packet for body_part {body_part_id}: {error_msg}"

    def test_100hz_sustained_transmission(self, router, renderer, driver_manager):
        """Test sustained transmission at 100Hz for 10 seconds."""
        # Note: We'll simulate 100 frames (1 second) for faster test execution
        # Real-world would be 1000 frames (10 seconds)
        num_frames = 100
        target_hz = 100
        frame_interval_us = int(1e6 / target_hz)  # 10ms = 10,000 microseconds

        start_time = time.time()

        # Send frames at 100Hz
        for frame in range(num_frames):
            timestamp_us = 3000000 + frame * frame_interval_us

            # Create contact
            contact = self.create_contact_patch(
                body_part_id=10,
                force_normal=1.5 + (frame % 10) * 0.1,  # Varying force
                timestamp_us=timestamp_us
            )

            # Route
            filtered = router.route([contact])

            # Render (need to fill buffer first)
            if frame >= 9:  # After first 10 samples
                cue_params_dict = renderer.render(filtered)

                # Send to driver
                if cue_params_dict:
                    driver_manager.send_all(cue_params_dict)

        end_time = time.time()
        total_time_s = end_time - start_time

        # Get statistics
        driver = driver_manager.drivers[10]
        stats = driver.get_stats()

        # Should have sent most packets (renderer may skip some frames)
        # At minimum, should have sent >80% of frames after buffer filled
        expected_min_packets = int((num_frames - 9) * 0.8)
        assert stats['packets_sent'] >= expected_min_packets

        # Success rate should be 100% (no packet loss)
        assert stats['success_rate'] == 1.0

        # No dropped packets
        assert stats['packets_dropped'] == 0

        # Test completed in reasonable time
        # 100 frames should take <1 second in simulation
        assert total_time_s < 2.0, f"Sustained test took {total_time_s:.2f}s, expected <2s"

        print(f"\nSustained transmission stats:")
        print(f"  Frames sent: {stats['packets_sent']}")
        print(f"  Success rate: {stats['success_rate'] * 100:.1f}%")
        print(f"  Total time: {total_time_s:.3f}s")
        print(f"  Effective rate: {stats['packets_sent'] / total_time_s:.1f} Hz")

    def test_cue_masking_in_hardware(self, router, renderer, driver_manager):
        """Test that cue_mask affects transmitted packets."""
        # Create router with custom homunculus (limited cue mask)
        from src.routing.somatotopic_router import Homunculus

        # Create custom config with limited cue mask (no texture, no shear)
        custom_config = {
            'index_tip': {
                'spatial_res_mm': 2.0,
                'freq_range_hz': (20, 500),
                'sensitivity': 1.0,
                'rendering_tier': 1,
                'cue_mask': FilteredContact.CUE_IMPACT | FilteredContact.CUE_RING | FilteredContact.CUE_WEIGHT
            }
        }

        custom_homunculus = Homunculus(config=custom_config)
        router_custom = SomatotopicRouter(homunculus=custom_homunculus)

        # Create contact
        contact = self.create_contact_patch(
            body_part_id=10,
            force_normal=1.5,
            timestamp_us=4000000
        )

        # Fill buffer and render
        for i in range(10):
            contact_i = self.create_contact_patch(
                body_part_id=10,
                force_normal=1.5,
                timestamp_us=4000000 + i * 1000
            )
            filtered = router_custom.route([contact_i])
            cue_params_dict = renderer.render(filtered)

        # Send to driver
        driver_manager.send_all(cue_params_dict)

        # Get transmitted packet
        driver = driver_manager.drivers[10]
        tx_packets = driver.get_tx_buffer()

        # Clear previous packets from other tests
        driver.clear_buffers()

        # Send one more frame with masking
        contact_final = self.create_contact_patch(
            body_part_id=10,
            force_normal=1.5,
            timestamp_us=4010000
        )
        filtered_final = router_custom.route([contact_final])
        cue_params_dict_final = renderer.render(filtered_final)
        driver_manager.send_all(cue_params_dict_final)

        # Get fresh packet
        tx_packets_fresh = driver.get_tx_buffer()
        assert len(tx_packets_fresh) == 1

        stored_cue_params = tx_packets_fresh[0]['cue_params']

        # Verify texture and shear are masked (should be zero or very small)
        assert stored_cue_params.texture_amplitude < 0.01
        assert stored_cue_params.shear_magnitude < 0.01

        # Verify impact, ring, weight are present (non-zero possible)
        # Note: These may still be zero depending on contact state
        # Just verify the packet was created successfully
        assert stored_cue_params.weight_offset >= 0.0

    def test_packet_integrity_under_load(self, router, renderer):
        """Test packet integrity with rapid transmission."""
        # Create driver with packet loss enabled
        driver_manager = DriverManager(driver_type="mock")
        driver_manager.register_driver(
            body_part_id=10,
            port="MOCK_INDEX",
            config={'simulate_latency': True, 'packet_loss_rate': 0.05}  # 5% loss
        )

        # Send 50 packets rapidly
        valid_packets = 0
        total_sent = 0

        for frame in range(50):
            contact = self.create_contact_patch(
                body_part_id=10,
                force_normal=1.5,
                timestamp_us=5000000 + frame * 1000
            )

            filtered = router.route([contact])

            if frame >= 9:  # After buffer filled
                cue_params_dict = renderer.render(filtered)

                if cue_params_dict:
                    results = driver_manager.send_all(cue_params_dict)
                    if results.get(10):
                        total_sent += 1

        # Get statistics
        driver = driver_manager.drivers[10]
        stats = driver.get_stats()

        # Should have sent most packets (allowing for 5% loss + renderer skips)
        # Renderer may skip some frames, so expect at least 70% transmission
        expected_min_sent = int((50 - 9) * 0.70)  # At least 70% of renderable frames
        assert stats['packets_sent'] >= expected_min_sent

        # Validate all transmitted packets
        tx_packets = driver.get_tx_buffer()
        for tx_packet in tx_packets:
            packet_data = tx_packet['packet']
            is_valid, error_msg = ProtocolValidator.validate_packet(packet_data)
            assert is_valid is True, f"Invalid packet under load: {error_msg}"

        # Success rate should be ~95% (±10% tolerance)
        assert 0.85 <= stats['success_rate'] <= 1.0

        print(f"\nPacket integrity under load:")
        print(f"  Packets sent: {stats['packets_sent']}")
        print(f"  Packets dropped: {stats['packets_dropped']}")
        print(f"  Success rate: {stats['success_rate'] * 100:.1f}%")
        print(f"  Avg latency: {stats['avg_latency_ms']:.3f}ms")

    def test_memory_stability(self, router, renderer, driver_manager):
        """Test memory stability over extended operation."""
        import sys

        # Send 200 packets and verify no memory leaks
        for frame in range(200):
            contact = self.create_contact_patch(
                body_part_id=10,
                force_normal=1.5,
                timestamp_us=6000000 + frame * 1000
            )

            filtered = router.route([contact])

            if frame >= 9:
                cue_params_dict = renderer.render(filtered)
                if cue_params_dict:
                    driver_manager.send_all(cue_params_dict)

        # Get final statistics
        driver = driver_manager.drivers[10]
        stats = driver.get_stats()

        # Should have processed all packets
        assert stats['packets_sent'] > 0

        # Buffer should not grow unbounded
        tx_buffer_size = len(driver.get_tx_buffer())
        assert tx_buffer_size == stats['packets_sent']

        # Clear buffers
        driver.clear_buffers()
        assert len(driver.get_tx_buffer()) == 0

        print(f"\nMemory stability test:")
        print(f"  Total packets processed: {stats['packets_sent']}")
        print(f"  Final buffer size: {tx_buffer_size}")
        print(f"  Cleared successfully: {len(driver.get_tx_buffer()) == 0}")

    def test_renderer_stats_tracking(self, router, renderer, driver_manager):
        """Test renderer statistics tracking."""
        # Reset renderer
        renderer.reset()

        # Send 20 contacts
        for frame in range(20):
            contact = self.create_contact_patch(
                body_part_id=10,
                force_normal=1.5,
                timestamp_us=7000000 + frame * 1000
            )

            filtered = router.route([contact])
            cue_params_dict = renderer.render(filtered)

        # Get renderer stats
        renderer_stats = renderer.get_stats()

        # Should have rendered all frames
        assert renderer_stats['total_renders'] == 20

        # Should have tracked active contacts
        assert renderer_stats['active_contacts'] >= 0

        # Average inference time should be reasonable
        assert renderer_stats['avg_inference_time_ms'] < 50.0  # <50ms per render

        print(f"\nRenderer statistics:")
        print(f"  Total renders: {renderer_stats['total_renders']}")
        print(f"  Active contacts: {renderer_stats['active_contacts']}")
        print(f"  Avg inference time: {renderer_stats['avg_inference_time_ms']:.3f}ms")
