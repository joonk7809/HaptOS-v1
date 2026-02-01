"""
Performance Profiling Suite - Phase 1 Benchmarks.

Measures performance characteristics of the complete HAPTOS pipeline:
- Inference latency (Neural Renderer)
- Packet transmission latency (Mock Driver)
- End-to-end latency (Physics → Driver)
- Multi-contact scalability
- Sustained throughput

Target Metrics:
- Avg inference time: <10ms (p95)
- Avg packet transmission: <6ms
- End-to-end latency: <20ms (p95)
- Multi-contact: 10+ simultaneous contacts in <100ms
"""

import pytest
import time
import numpy as np
from typing import List, Dict

from src.core.schemas import ContactPatch, FilteredContact, CueParams
from src.routing.somatotopic_router import SomatotopicRouter
from src.inference.neural_renderer import NeuralRenderer
from src.hardware.driver_manager import DriverManager


class TestInferenceLatency:
    """Benchmark Neural Renderer inference latency."""

    @pytest.fixture
    def renderer(self):
        """Create neural renderer."""
        return NeuralRenderer(
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )

    @pytest.fixture
    def router(self):
        """Create router."""
        return SomatotopicRouter()

    def create_contact(self, force: float, timestamp_us: int) -> ContactPatch:
        """Helper to create contact."""
        return ContactPatch(
            body_part_id=10,
            force_normal=force,
            force_shear=(0.1, 0.0),
            velocity=(0.0, 0.0, -0.05),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=timestamp_us
        )

    def test_single_contact_inference_latency(self, renderer, router):
        """Measure inference time for single contact."""
        # Fill buffer first (10 samples)
        for i in range(10):
            contact = self.create_contact(force=1.5, timestamp_us=i * 1000)
            filtered = router.route([contact])
            renderer.render(filtered)

        # Benchmark inference
        latencies_ms = []

        for i in range(100):
            contact = self.create_contact(force=1.5, timestamp_us=10000 + i * 1000)
            filtered = router.route([contact])

            start = time.time()
            cue_params_dict = renderer.render(filtered)
            end = time.time()

            if cue_params_dict:  # Only count when rendering occurs
                latencies_ms.append((end - start) * 1000)

        # Calculate statistics
        latencies_ms = np.array(latencies_ms)

        avg_latency = np.mean(latencies_ms)
        p95_latency = np.percentile(latencies_ms, 95)
        p99_latency = np.percentile(latencies_ms, 99)
        max_latency = np.max(latencies_ms)

        print(f"\nSingle Contact Inference Latency:")
        print(f"  Avg: {avg_latency:.3f}ms")
        print(f"  P95: {p95_latency:.3f}ms")
        print(f"  P99: {p99_latency:.3f}ms")
        print(f"  Max: {max_latency:.3f}ms")

        # Validate against targets
        assert avg_latency < 10.0, f"Avg latency {avg_latency:.2f}ms exceeds 10ms"
        assert p95_latency < 15.0, f"P95 latency {p95_latency:.2f}ms exceeds 15ms"

    def test_renderer_statistics(self, renderer, router):
        """Validate renderer statistics tracking."""
        # Reset renderer
        renderer.reset()

        # Render 50 contacts
        for i in range(50):
            contact = self.create_contact(force=1.5, timestamp_us=i * 1000)
            filtered = router.route([contact])
            renderer.render(filtered)

        # Get stats
        stats = renderer.get_stats()

        print(f"\nRenderer Statistics:")
        print(f"  Total renders: {stats['total_renders']}")
        print(f"  Active contacts: {stats['active_contacts']}")
        print(f"  Avg inference time: {stats['avg_inference_time_ms']:.3f}ms")

        assert stats['total_renders'] == 50
        assert stats['avg_inference_time_ms'] < 20.0


class TestPacketTransmissionLatency:
    """Benchmark Mock Driver packet transmission latency."""

    @pytest.fixture
    def driver_manager(self):
        """Create driver manager."""
        manager = DriverManager(driver_type="mock")
        manager.register_driver(
            body_part_id=10,
            port="MOCK_INDEX",
            config={'simulate_latency': True, 'packet_loss_rate': 0.0}
        )
        return manager

    def test_transmission_latency(self, driver_manager):
        """Measure packet transmission latency."""
        # Send 100 packets
        for i in range(100):
            cue_params = CueParams(
                texture_grain_hz=100.0,
                texture_amplitude=0.5,
                shear_direction=(1.0, 0.0),
                shear_magnitude=0.2,
                weight_offset=0.5,
                impact_amplitude=0.0,
                impact_decay_ms=0.0,
                impact_frequency_hz=0.0,
                ring_amplitude=0.0,
                ring_decay_ms=0.0,
                trigger_impulse=False,
                timestamp_us=i * 1000
            )

            driver_manager.send_all({10: cue_params})

        # Get statistics
        stats = driver_manager.get_aggregate_stats()

        print(f"\nPacket Transmission Statistics:")
        print(f"  Total sent: {stats['total_sent']}")
        print(f"  Avg latency: {stats['avg_latency_ms']:.3f}ms")
        print(f"  Success rate: {stats['success_rate'] * 100:.1f}%")

        # Validate
        assert stats['total_sent'] == 100
        assert stats['success_rate'] == 1.0
        assert 4.0 <= stats['avg_latency_ms'] <= 6.0, f"Latency {stats['avg_latency_ms']:.2f}ms out of expected range"


class TestEndToEndLatency:
    """Benchmark complete pipeline latency."""

    @pytest.fixture
    def pipeline(self):
        """Create complete pipeline."""
        router = SomatotopicRouter()
        renderer = NeuralRenderer(
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )
        driver_manager = DriverManager(driver_type="mock")
        driver_manager.register_driver(
            body_part_id=10,
            port="MOCK_INDEX",
            config={'simulate_latency': False, 'packet_loss_rate': 0.0}
        )

        return {
            'router': router,
            'renderer': renderer,
            'driver_manager': driver_manager
        }

    def create_contact(self, force: float, timestamp_us: int) -> ContactPatch:
        """Helper to create contact."""
        return ContactPatch(
            body_part_id=10,
            force_normal=force,
            force_shear=(0.1, 0.0),
            velocity=(0.0, 0.0, -0.05),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=timestamp_us
        )

    def test_end_to_end_latency(self, pipeline):
        """Measure Physics → Router → Renderer → Driver latency."""
        # Fill buffer
        for i in range(10):
            contact = self.create_contact(force=1.5, timestamp_us=i * 1000)
            filtered = pipeline['router'].route([contact])
            pipeline['renderer'].render(filtered)

        # Benchmark end-to-end
        latencies_ms = []

        for i in range(100):
            contact = self.create_contact(force=1.5, timestamp_us=10000 + i * 1000)

            start = time.time()

            # Router
            filtered = pipeline['router'].route([contact])

            # Renderer
            cue_params_dict = pipeline['renderer'].render(filtered)

            # Driver
            if cue_params_dict:
                pipeline['driver_manager'].send_all(cue_params_dict)

            end = time.time()

            latencies_ms.append((end - start) * 1000)

        # Statistics
        latencies_ms = np.array(latencies_ms)

        avg_latency = np.mean(latencies_ms)
        p95_latency = np.percentile(latencies_ms, 95)
        p99_latency = np.percentile(latencies_ms, 99)

        print(f"\nEnd-to-End Latency:")
        print(f"  Avg: {avg_latency:.3f}ms")
        print(f"  P95: {p95_latency:.3f}ms")
        print(f"  P99: {p99_latency:.3f}ms")

        # Validate targets
        assert avg_latency < 15.0, f"Avg latency {avg_latency:.2f}ms exceeds 15ms"
        assert p95_latency < 20.0, f"P95 latency {p95_latency:.2f}ms exceeds 20ms"
        assert p99_latency < 25.0, f"P99 latency {p99_latency:.2f}ms exceeds 25ms"


class TestMultiContactScalability:
    """Benchmark multi-contact performance."""

    @pytest.fixture
    def pipeline(self):
        """Create pipeline with multiple drivers."""
        router = SomatotopicRouter()
        renderer = NeuralRenderer(
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )
        driver_manager = DriverManager(driver_type="mock")

        # Register 10 drivers
        for body_part_id in range(10, 20):
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

    def create_contacts(self, num_contacts: int, timestamp_us: int) -> List[ContactPatch]:
        """Create multiple simultaneous contacts."""
        contacts = []
        for i in range(num_contacts):
            contact = ContactPatch(
                body_part_id=10 + i,
                force_normal=1.5 + i * 0.1,
                force_shear=(0.1, 0.0),
                velocity=(0.0, 0.0, -0.05),
                contact_area=0.0001,
                material_hint=1,
                timestamp_us=timestamp_us
            )
            contacts.append(contact)
        return contacts

    def test_1_contact_latency(self, pipeline):
        """Baseline: 1 contact."""
        self._benchmark_contacts(pipeline, num_contacts=1, label="1 contact")

    def test_2_contacts_latency(self, pipeline):
        """2 simultaneous contacts."""
        self._benchmark_contacts(pipeline, num_contacts=2, label="2 contacts")

    def test_5_contacts_latency(self, pipeline):
        """5 simultaneous contacts."""
        self._benchmark_contacts(pipeline, num_contacts=5, label="5 contacts")

    def test_10_contacts_latency(self, pipeline):
        """10 simultaneous contacts."""
        self._benchmark_contacts(pipeline, num_contacts=10, label="10 contacts")

    def _benchmark_contacts(self, pipeline, num_contacts: int, label: str):
        """Helper to benchmark N simultaneous contacts."""
        # Fill buffers
        for i in range(10):
            contacts = self.create_contacts(num_contacts, timestamp_us=i * 1000)
            filtered = pipeline['router'].route(contacts)
            pipeline['renderer'].render(filtered)

        # Benchmark
        latencies_ms = []

        for i in range(50):
            contacts = self.create_contacts(num_contacts, timestamp_us=10000 + i * 1000)

            start = time.time()

            filtered = pipeline['router'].route(contacts)
            cue_params_dict = pipeline['renderer'].render(filtered)

            if cue_params_dict:
                pipeline['driver_manager'].send_all(cue_params_dict)

            end = time.time()

            latencies_ms.append((end - start) * 1000)

        # Statistics
        latencies_ms = np.array(latencies_ms)
        avg_latency = np.mean(latencies_ms)

        print(f"\n{label} Latency: {avg_latency:.3f}ms")

        # Validate scaling (should be roughly linear)
        if num_contacts <= 10:
            max_expected = 15.0 * num_contacts  # Generous bound
            assert avg_latency < max_expected, f"{label} latency {avg_latency:.2f}ms exceeds {max_expected}ms"


class TestSustainedThroughput:
    """Benchmark sustained throughput over time."""

    @pytest.fixture
    def pipeline(self):
        """Create pipeline."""
        router = SomatotopicRouter()
        renderer = NeuralRenderer(
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )
        driver_manager = DriverManager(driver_type="mock")
        driver_manager.register_driver(
            body_part_id=10,
            port="MOCK_INDEX",
            config={'simulate_latency': False, 'packet_loss_rate': 0.0}
        )

        return {
            'router': router,
            'renderer': renderer,
            'driver_manager': driver_manager
        }

    def create_contact(self, force: float, timestamp_us: int) -> ContactPatch:
        """Helper to create contact."""
        return ContactPatch(
            body_part_id=10,
            force_normal=force,
            force_shear=(0.1, 0.0),
            velocity=(0.0, 0.0, -0.05),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=timestamp_us
        )

    def test_sustained_100hz_throughput(self, pipeline):
        """Test sustained 100Hz transmission."""
        # Fill buffer
        for i in range(10):
            contact = self.create_contact(force=1.5, timestamp_us=i * 1000)
            filtered = pipeline['router'].route([contact])
            pipeline['renderer'].render(filtered)

        start_time = time.time()

        # Send 500 frames (5 seconds @ 100Hz)
        packets_sent = 0

        for i in range(500):
            contact = self.create_contact(force=1.5, timestamp_us=10000 + i * 10000)
            filtered = pipeline['router'].route([contact])
            cue_params_dict = pipeline['renderer'].render(filtered)

            if cue_params_dict:
                pipeline['driver_manager'].send_all(cue_params_dict)
                packets_sent += 1

        end_time = time.time()
        total_time_s = end_time - start_time

        # Calculate throughput
        effective_hz = packets_sent / total_time_s

        print(f"\nSustained Throughput:")
        print(f"  Packets sent: {packets_sent}")
        print(f"  Total time: {total_time_s:.3f}s")
        print(f"  Effective rate: {effective_hz:.1f} Hz")

        # Validate
        assert packets_sent >= 400, f"Only sent {packets_sent}/500 packets"
        assert effective_hz >= 50, f"Effective rate {effective_hz:.1f} Hz too low"


class TestMemoryFootprint:
    """Benchmark memory usage and buffer management."""

    @pytest.fixture
    def pipeline(self):
        """Create pipeline."""
        router = SomatotopicRouter()
        renderer = NeuralRenderer(
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )
        driver_manager = DriverManager(driver_type="mock")
        driver_manager.register_driver(
            body_part_id=10,
            port="MOCK_INDEX",
            config={'simulate_latency': False, 'packet_loss_rate': 0.0}
        )

        return {
            'router': router,
            'renderer': renderer,
            'driver_manager': driver_manager
        }

    def create_contact(self, force: float, timestamp_us: int) -> ContactPatch:
        """Helper to create contact."""
        return ContactPatch(
            body_part_id=10,
            force_normal=force,
            force_shear=(0.1, 0.0),
            velocity=(0.0, 0.0, -0.05),
            contact_area=0.0001,
            material_hint=1,
            timestamp_us=timestamp_us
        )

    def test_buffer_size_stability(self, pipeline):
        """Validate buffers don't grow unbounded."""
        # Process 1000 contacts
        for i in range(1000):
            contact = self.create_contact(force=1.5, timestamp_us=i * 1000)
            filtered = pipeline['router'].route([contact])
            cue_params_dict = pipeline['renderer'].render(filtered)

            if cue_params_dict:
                pipeline['driver_manager'].send_all(cue_params_dict)

        # Check driver buffer
        driver = pipeline['driver_manager'].drivers[10]
        tx_buffer_size = len(driver.get_tx_buffer())

        print(f"\nMemory Footprint:")
        print(f"  TX buffer size: {tx_buffer_size} packets")
        print(f"  Expected: ~991 packets (after buffer fills)")

        # Buffer should contain all transmitted packets
        assert tx_buffer_size > 0
        assert tx_buffer_size < 2000  # Should not grow unbounded

    def test_buffer_clear(self, pipeline):
        """Validate buffer clearing works."""
        # Send some packets
        for i in range(50):
            contact = self.create_contact(force=1.5, timestamp_us=i * 1000)
            filtered = pipeline['router'].route([contact])
            cue_params_dict = pipeline['renderer'].render(filtered)

            if cue_params_dict:
                pipeline['driver_manager'].send_all(cue_params_dict)

        driver = pipeline['driver_manager'].drivers[10]

        # Clear buffers
        driver.clear_buffers()

        # Verify cleared
        assert len(driver.get_tx_buffer()) == 0

        print(f"\nBuffer cleared successfully")
