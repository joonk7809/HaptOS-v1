"""
Unit tests for Mock Hardware Driver and Protocol Validator.

Tests hardware simulation layer without requiring real Teensy hardware.
Validates binary protocol, packet transmission, latency simulation, and statistics.
"""

import pytest
import time
from typing import Tuple

from src.hardware.mock_driver import MockHardwareDriver
from src.hardware.protocol_validator import ProtocolValidator
from src.hardware.driver_manager import DriverManager
from src.core.schemas import CueParams


class TestMockDriver:
    """Test suite for MockHardwareDriver functionality."""

    def test_driver_initialization(self):
        """Test driver initializes with correct default parameters."""
        driver = MockHardwareDriver()

        assert driver.port == "MOCK"
        assert driver.baudrate == 115200
        assert driver.simulate_latency is True
        assert driver.packet_loss_rate == 0.0
        assert driver.is_connected is False
        assert driver.packets_sent == 0
        assert driver.packets_received == 0
        assert driver.packets_dropped == 0

    def test_driver_initialization_custom_params(self):
        """Test driver initialization with custom parameters."""
        driver = MockHardwareDriver(
            port="CUSTOM_PORT",
            baudrate=9600,
            simulate_latency=False,
            packet_loss_rate=0.1
        )

        assert driver.port == "CUSTOM_PORT"
        assert driver.baudrate == 9600
        assert driver.simulate_latency is False
        assert driver.packet_loss_rate == 0.1

    def test_connection_lifecycle(self):
        """Test connect/disconnect lifecycle."""
        driver = MockHardwareDriver()

        # Initially disconnected
        assert driver.is_connected is False

        # Connect
        result = driver.connect()
        assert result is True
        assert driver.is_connected is True

        # Disconnect
        driver.disconnect()
        assert driver.is_connected is False
        assert len(driver.tx_buffer) == 0
        assert len(driver.rx_buffer) == 0

    def test_packet_transmission_success(self):
        """Test successful packet transmission."""
        driver = MockHardwareDriver(simulate_latency=False)
        driver.connect()

        # Create test CueParams
        cue_params = CueParams(
            texture_grain_hz=150.0,
            texture_amplitude=0.5,
            shear_direction=(0.707, 0.707),
            shear_magnitude=0.3,
            weight_offset=0.6,
            impact_amplitude=0.8,
            impact_decay_ms=50.0,
            impact_frequency_hz=150.0,
            ring_amplitude=0.4,
            ring_decay_ms=100.0,
            trigger_impulse=True,
            timestamp_us=1000000
        )

        # Send packet
        success = driver.send_cue_params(cue_params)

        assert success is True
        assert driver.packets_sent == 1
        assert driver.packets_dropped == 0
        assert len(driver.tx_buffer) == 1

        # Check transmitted packet
        tx_packet = driver.tx_buffer[0]
        assert 'packet' in tx_packet
        assert 'timestamp' in tx_packet
        assert 'cue_params' in tx_packet
        assert tx_packet['cue_params'] == cue_params

    def test_packet_transmission_requires_connection(self):
        """Test that sending requires active connection."""
        driver = MockHardwareDriver()

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
            timestamp_us=2000000
        )

        # Should raise error when not connected
        with pytest.raises(RuntimeError, match="Driver not connected"):
            driver.send_cue_params(cue_params)

    def test_latency_simulation(self):
        """Test realistic transmission latency calculation."""
        driver = MockHardwareDriver(
            baudrate=115200,
            simulate_latency=True
        )
        driver.connect()

        cue_params = CueParams(
            texture_grain_hz=200.0,
            texture_amplitude=0.7,
            shear_direction=(0.0, 1.0),
            shear_magnitude=0.4,
            weight_offset=0.8,
            impact_amplitude=0.9,
            impact_decay_ms=30.0,
            impact_frequency_hz=200.0,
            ring_amplitude=0.5,
            ring_decay_ms=150.0,
            trigger_impulse=True,
            timestamp_us=3000000
        )

        driver.send_cue_params(cue_params)

        # Expected latency: 52 bytes * 10 bits/byte / 115200 baud * 1000 ms
        # ≈ 4.5 ms
        assert driver.total_latency_ms > 0
        assert 4.0 <= driver.total_latency_ms <= 5.0

    def test_latency_disabled(self):
        """Test latency simulation can be disabled."""
        driver = MockHardwareDriver(
            simulate_latency=False
        )
        driver.connect()

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
            timestamp_us=4000000
        )

        driver.send_cue_params(cue_params)

        # No latency should be accumulated
        assert driver.total_latency_ms == 0.0

    def test_packet_loss_simulation(self):
        """Test configurable packet loss simulation."""
        driver = MockHardwareDriver(
            packet_loss_rate=1.0,  # 100% loss for deterministic test
            simulate_latency=False
        )
        driver.connect()

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
            timestamp_us=5000000
        )

        # Send packet with 100% loss
        success = driver.send_cue_params(cue_params)

        assert success is False
        assert driver.packets_dropped == 1
        assert driver.packets_sent == 0
        assert len(driver.tx_buffer) == 0

    def test_ack_response(self):
        """Test ACK/NACK response generation."""
        driver = MockHardwareDriver()
        driver.connect()

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
            timestamp_us=6000000
        )

        driver.send_cue_params(cue_params)

        # Should have ACK in rx_buffer
        assert len(driver.rx_buffer) == 1
        ack = driver.read_ack()
        assert ack is True  # ACK = 0x06

    def test_ack_read_empty_buffer(self):
        """Test reading ACK from empty buffer."""
        driver = MockHardwareDriver()
        driver.connect()

        # No packets sent, no ACKs
        ack = driver.read_ack()
        assert ack is None

    def test_buffer_management(self):
        """Test tx/rx buffer management."""
        driver = MockHardwareDriver()
        driver.connect()

        # Send multiple packets
        for i in range(3):
            cue_params = CueParams(
                texture_grain_hz=100.0 + i * 10,
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
                timestamp_us=7000000 + i * 1000
            )
            driver.send_cue_params(cue_params)

        # Check buffers
        assert len(driver.tx_buffer) == 3
        assert len(driver.rx_buffer) == 3

        # Get transmitted packets
        tx_packets = driver.get_tx_buffer()
        assert len(tx_packets) == 3
        assert tx_packets[0]['cue_params'].texture_grain_hz == 100.0
        assert tx_packets[1]['cue_params'].texture_grain_hz == 110.0
        assert tx_packets[2]['cue_params'].texture_grain_hz == 120.0

        # Clear buffers
        driver.clear_buffers()
        assert len(driver.tx_buffer) == 0
        assert len(driver.rx_buffer) == 0

    def test_statistics_tracking(self):
        """Test transmission statistics tracking."""
        driver = MockHardwareDriver(
            packet_loss_rate=0.0,
            simulate_latency=False
        )
        driver.connect()

        # Send 10 packets
        for i in range(10):
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
                timestamp_us=8000000 + i * 1000
            )
            driver.send_cue_params(cue_params)

        stats = driver.get_stats()

        assert stats['packets_sent'] == 10
        assert stats['packets_dropped'] == 0
        assert stats['packets_received'] == 10  # ACKs
        assert stats['success_rate'] == 1.0
        assert stats['avg_latency_ms'] == 0.0  # Latency disabled

    def test_statistics_with_packet_loss(self):
        """Test statistics with simulated packet loss."""
        # Use 50% loss rate, but need to account for randomness
        # Send many packets to get statistical average
        driver = MockHardwareDriver(
            packet_loss_rate=0.5,
            simulate_latency=False
        )
        driver.connect()

        # Send 100 packets for statistical significance
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
                timestamp_us=9000000 + i * 1000
            )
            driver.send_cue_params(cue_params)

        stats = driver.get_stats()

        # Should have some packets sent and some dropped
        total_attempts = stats['packets_sent'] + stats['packets_dropped']
        assert total_attempts == 100

        # Success rate should be around 0.5 (±20% for randomness)
        assert 0.3 <= stats['success_rate'] <= 0.7

    def test_reset_stats(self):
        """Test statistics reset."""
        driver = MockHardwareDriver()
        driver.connect()

        # Send packet
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
            timestamp_us=10000000
        )
        driver.send_cue_params(cue_params)

        # Stats should be non-zero
        assert driver.packets_sent > 0

        # Reset
        driver.reset_stats()

        # Stats should be zero
        assert driver.packets_sent == 0
        assert driver.packets_received == 0
        assert driver.packets_dropped == 0
        assert driver.total_latency_ms == 0.0

    def test_repr(self):
        """Test string representation."""
        driver = MockHardwareDriver(port="TEST_PORT")

        # Disconnected state
        repr_str = repr(driver)
        assert "MockHardwareDriver" in repr_str
        assert "TEST_PORT" in repr_str
        assert "disconnected" in repr_str

        # Connected state
        driver.connect()
        repr_str = repr(driver)
        assert "connected" in repr_str


class TestProtocolValidator:
    """Test suite for ProtocolValidator binary protocol validation."""

    def test_valid_packet_structure(self):
        """Test validation of correctly formatted packet."""
        # Create valid CueParams
        cue_params = CueParams(
            texture_grain_hz=150.0,
            texture_amplitude=0.5,
            shear_direction=(0.707, 0.707),
            shear_magnitude=0.3,
            weight_offset=0.6,
            impact_amplitude=0.8,
            impact_decay_ms=50.0,
            impact_frequency_hz=150.0,
            ring_amplitude=0.4,
            ring_decay_ms=100.0,
            trigger_impulse=True,
            timestamp_us=1000000
        )

        # Serialize to packet
        packet = cue_params.to_bytes()

        # Validate
        is_valid, error_msg = ProtocolValidator.validate_packet(packet)

        assert is_valid is True
        assert error_msg == "Valid packet"

    def test_invalid_packet_length(self):
        """Test detection of incorrect packet length."""
        # Too short
        short_packet = b'\xAA\x55' + b'\x00' * 40
        is_valid, error_msg = ProtocolValidator.validate_packet(short_packet)

        assert is_valid is False
        assert "Invalid packet size" in error_msg

        # Too long
        long_packet = b'\xAA\x55' + b'\x00' * 60
        is_valid, error_msg = ProtocolValidator.validate_packet(long_packet)

        assert is_valid is False
        assert "Invalid packet size" in error_msg

    def test_invalid_header_detection(self):
        """Test detection of incorrect header magic bytes."""
        # Create packet with wrong header
        packet = b'\xFF\xFF' + b'\x00' * 49 + b'\xFF'

        is_valid, error_msg = ProtocolValidator.validate_packet(packet)

        assert is_valid is False
        assert "Invalid header" in error_msg

    def test_checksum_validation_success(self):
        """Test checksum validation with correct checksum."""
        # Valid packet from CueParams
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
            timestamp_us=2000000
        )

        packet = cue_params.to_bytes()
        is_valid, error_msg = ProtocolValidator.validate_packet(packet)

        assert is_valid is True

    def test_checksum_validation_failure(self):
        """Test detection of corrupted checksum."""
        # Create valid packet
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
            timestamp_us=3000000
        )

        packet = bytearray(cue_params.to_bytes())

        # Corrupt checksum (last byte)
        packet[-1] ^= 0xFF

        is_valid, error_msg = ProtocolValidator.validate_packet(bytes(packet))

        assert is_valid is False
        assert "Checksum mismatch" in error_msg

    def test_round_trip_serialization(self):
        """Test serialize → deserialize preserves data."""
        # Create original CueParams
        original = CueParams(
            texture_grain_hz=175.5,
            texture_amplitude=0.65,
            shear_direction=(0.6, 0.8),
            shear_magnitude=0.35,
            weight_offset=0.72,
            impact_amplitude=0.88,
            impact_decay_ms=45.5,
            impact_frequency_hz=165.0,
            ring_amplitude=0.42,
            ring_decay_ms=125.0,
            trigger_impulse=True,
            timestamp_us=1234567
        )

        # Serialize
        packet = original.to_bytes()

        # Deserialize
        is_valid, deserialized = ProtocolValidator.deserialize_and_validate(packet)

        assert is_valid is True
        assert deserialized is not None

        # Compare (with small tolerance for float precision)
        is_equal = ProtocolValidator.compare_cue_params(original, deserialized)
        assert is_equal is True

    def test_field_range_validation(self):
        """Test validation of CueParams field value ranges."""
        # Valid params
        valid_params = CueParams(
            texture_grain_hz=100.0,
            texture_amplitude=0.5,
            shear_direction=(0.707, 0.707),
            shear_magnitude=0.5,
            weight_offset=0.5,
            impact_amplitude=0.5,
            impact_decay_ms=50.0,
            impact_frequency_hz=100.0,
            ring_amplitude=0.5,
            ring_decay_ms=100.0,
            trigger_impulse=False,
            timestamp_us=1000000
        )

        is_valid, error_msg = ProtocolValidator.validate_field_ranges(valid_params)
        assert is_valid is True

        # Invalid amplitude (>1.0)
        invalid_params = CueParams(
            texture_grain_hz=100.0,
            texture_amplitude=1.5,  # Invalid
            shear_direction=(1.0, 0.0),
            shear_magnitude=0.5,
            weight_offset=0.5,
            impact_amplitude=0.5,
            impact_decay_ms=50.0,
            impact_frequency_hz=100.0,
            ring_amplitude=0.5,
            ring_decay_ms=100.0,
            trigger_impulse=False,
            timestamp_us=1000000
        )

        is_valid, error_msg = ProtocolValidator.validate_field_ranges(invalid_params)
        assert is_valid is False
        assert "texture_amplitude out of range" in error_msg

    def test_compare_cue_params_equal(self):
        """Test comparison of identical CueParams."""
        params1 = CueParams(
            texture_grain_hz=100.0,
            texture_amplitude=0.5,
            shear_direction=(1.0, 0.0),
            shear_magnitude=0.2,
            weight_offset=0.5,
            impact_amplitude=0.8,
            impact_decay_ms=50.0,
            impact_frequency_hz=150.0,
            ring_amplitude=0.4,
            ring_decay_ms=100.0,
            trigger_impulse=True,
            timestamp_us=1000000
        )

        params2 = CueParams(
            texture_grain_hz=100.0,
            texture_amplitude=0.5,
            shear_direction=(1.0, 0.0),
            shear_magnitude=0.2,
            weight_offset=0.5,
            impact_amplitude=0.8,
            impact_decay_ms=50.0,
            impact_frequency_hz=150.0,
            ring_amplitude=0.4,
            ring_decay_ms=100.0,
            trigger_impulse=True,
            timestamp_us=1000000
        )

        is_equal = ProtocolValidator.compare_cue_params(params1, params2)
        assert is_equal is True

    def test_compare_cue_params_different(self):
        """Test comparison of different CueParams."""
        params1 = CueParams(
            texture_grain_hz=100.0,
            texture_amplitude=0.5,
            shear_direction=(1.0, 0.0),
            shear_magnitude=0.2,
            weight_offset=0.5,
            impact_amplitude=0.8,
            impact_decay_ms=50.0,
            impact_frequency_hz=150.0,
            ring_amplitude=0.4,
            ring_decay_ms=100.0,
            trigger_impulse=True,
            timestamp_us=1000000
        )

        params2 = CueParams(
            texture_grain_hz=150.0,  # Different
            texture_amplitude=0.5,
            shear_direction=(1.0, 0.0),
            shear_magnitude=0.2,
            weight_offset=0.5,
            impact_amplitude=0.8,
            impact_decay_ms=50.0,
            impact_frequency_hz=150.0,
            ring_amplitude=0.4,
            ring_decay_ms=100.0,
            trigger_impulse=True,
            timestamp_us=1000000
        )

        is_equal = ProtocolValidator.compare_cue_params(params1, params2)
        assert is_equal is False


class TestDriverManager:
    """Test suite for DriverManager multi-driver orchestration."""

    def test_driver_manager_initialization(self):
        """Test DriverManager initializes correctly."""
        manager = DriverManager(driver_type="mock")

        assert manager.driver_type == "mock"
        assert len(manager.drivers) == 0

    def test_register_single_driver(self):
        """Test registering a single driver."""
        manager = DriverManager(driver_type="mock")

        manager.register_driver(
            body_part_id=10,
            port="MOCK_INDEX",
            config={'simulate_latency': False}
        )

        assert 10 in manager.drivers
        assert manager.drivers[10].port == "MOCK_INDEX"
        assert manager.drivers[10].is_connected is True

    def test_register_multiple_drivers(self):
        """Test registering multiple drivers for different body parts."""
        manager = DriverManager(driver_type="mock")

        # Register 3 drivers
        manager.register_driver(10, "MOCK_INDEX")
        manager.register_driver(11, "MOCK_THUMB")
        manager.register_driver(12, "MOCK_PALM")

        assert len(manager.drivers) == 3
        assert 10 in manager.drivers
        assert 11 in manager.drivers
        assert 12 in manager.drivers

    def test_send_all_success(self):
        """Test sending CueParams to all registered drivers."""
        manager = DriverManager(driver_type="mock")

        # Register 2 drivers
        manager.register_driver(10, "MOCK_INDEX")
        manager.register_driver(11, "MOCK_THUMB")

        # Create CueParams for each
        cue_dict = {
            10: CueParams(
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
                timestamp_us=1000000
            ),
            11: CueParams(
                texture_grain_hz=150.0,
                texture_amplitude=0.6,
                shear_direction=(0.0, 1.0),
                shear_magnitude=0.3,
                weight_offset=0.6,
                impact_amplitude=0.0,
                impact_decay_ms=0.0,
                impact_frequency_hz=0.0,
                ring_amplitude=0.0,
                ring_decay_ms=0.0,
                trigger_impulse=False,
                timestamp_us=1000000
            )
        }

        # Send to all
        results = manager.send_all(cue_dict)

        assert results[10] is True
        assert results[11] is True

    def test_send_all_with_missing_driver(self):
        """Test send_all with CueParams for unregistered body part."""
        manager = DriverManager(driver_type="mock")

        # Only register driver for body_part_id=10
        manager.register_driver(10, "MOCK_INDEX")

        # Try to send to both 10 and 99 (not registered)
        cue_dict = {
            10: CueParams(
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
                timestamp_us=1000000
            ),
            99: CueParams(
                texture_grain_hz=150.0,
                texture_amplitude=0.6,
                shear_direction=(0.0, 1.0),
                shear_magnitude=0.3,
                weight_offset=0.6,
                impact_amplitude=0.0,
                impact_decay_ms=0.0,
                impact_frequency_hz=0.0,
                ring_amplitude=0.0,
                ring_decay_ms=0.0,
                trigger_impulse=False,
                timestamp_us=1000000
            )
        }

        results = manager.send_all(cue_dict)

        assert results[10] is True
        assert results[99] is False  # No driver registered

    def test_get_aggregate_stats(self):
        """Test aggregated statistics from multiple drivers."""
        manager = DriverManager(driver_type="mock")

        # Register 2 drivers
        manager.register_driver(10, "MOCK_INDEX", config={'simulate_latency': False})
        manager.register_driver(11, "MOCK_THUMB", config={'simulate_latency': False})

        # Send packets to both
        for i in range(5):
            cue_dict = {
                10: CueParams(
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
                    timestamp_us=1000000 + i * 1000
                ),
                11: CueParams(
                    texture_grain_hz=150.0,
                    texture_amplitude=0.6,
                    shear_direction=(0.0, 1.0),
                    shear_magnitude=0.3,
                    weight_offset=0.6,
                    impact_amplitude=0.0,
                    impact_decay_ms=0.0,
                    impact_frequency_hz=0.0,
                    ring_amplitude=0.0,
                    ring_decay_ms=0.0,
                    trigger_impulse=False,
                    timestamp_us=1000000 + i * 1000
                )
            }
            manager.send_all(cue_dict)

        # Get aggregate stats
        stats = manager.get_aggregate_stats()

        assert stats['driver_count'] == 2
        assert stats['total_sent'] == 10  # 5 packets * 2 drivers
        assert stats['total_dropped'] == 0
        assert stats['total_received'] == 10
        assert stats['success_rate'] == 1.0

    def test_disconnect_all(self):
        """Test disconnecting all drivers."""
        manager = DriverManager(driver_type="mock")

        # Register 2 drivers
        manager.register_driver(10, "MOCK_INDEX")
        manager.register_driver(11, "MOCK_THUMB")

        # Disconnect all
        manager.disconnect_all()

        # Drivers should be cleared
        assert len(manager.drivers) == 0

    def test_reset_all_stats(self):
        """Test resetting statistics for all drivers."""
        manager = DriverManager(driver_type="mock")

        # Register driver and send packet
        manager.register_driver(10, "MOCK_INDEX")

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
            timestamp_us=1000000
        )

        manager.send_all({10: cue_params})

        # Stats should be non-zero
        stats_before = manager.get_aggregate_stats()
        assert stats_before['total_sent'] > 0

        # Reset
        manager.reset_all_stats()

        # Stats should be zero
        stats_after = manager.get_aggregate_stats()
        assert stats_after['total_sent'] == 0

    def test_repr(self):
        """Test string representation."""
        manager = DriverManager(driver_type="mock")
        manager.register_driver(10, "MOCK_INDEX")

        repr_str = repr(manager)
        assert "DriverManager" in repr_str
        assert "type=mock" in repr_str
        assert "drivers=1" in repr_str
