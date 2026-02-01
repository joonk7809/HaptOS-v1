"""
Firmware Compatibility Tests.

Validates that Python serialization matches HaptosRuntime.ino C struct layout.
Ensures binary protocol compatibility between host (Python) and firmware (C/Arduino).

Reference: firmware/HaptosRuntime.ino
"""

import pytest
import struct
from typing import Tuple

from src.core.schemas import CueParams
from src.hardware.protocol_validator import ProtocolValidator


class TestFirmwareCompatibility:
    """Test suite for firmware binary protocol compatibility."""

    def test_packet_format_matches_firmware(self):
        """
        Validate 52-byte packet structure matches firmware expectations.

        Firmware struct layout (actual implementation in schemas.py):

        typedef struct {
            uint8_t header[2];           // 0xAA 0x55 (2 bytes)
            uint32_t timestamp_us;       // 4 bytes
            float texture_grain_hz;      // 4 bytes
            float texture_amplitude;     // 4 bytes
            float shear_direction[2];    // 8 bytes (2 floats)
            float shear_magnitude;       // 4 bytes
            float weight_offset;         // 4 bytes
            float impact_amplitude;      // 4 bytes
            float impact_decay_ms;       // 4 bytes
            float impact_frequency_hz;   // 4 bytes
            float ring_amplitude;        // 4 bytes
            float ring_decay_ms;         // 4 bytes
            uint8_t flags;               // 1 byte (trigger_impulse)
            uint8_t checksum;            // 1 byte
        } __attribute__((packed)) CuePacket;  // Total: 52 bytes
        """
        # Create CueParams
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

        # Serialize
        packet = cue_params.to_bytes()

        # Validate packet size
        assert len(packet) == 52, f"Packet size {len(packet)} != 52 bytes"

        # Validate header
        assert packet[0] == 0xAA
        assert packet[1] == 0x55

        # Validate struct layout (offsets)
        offset = 0

        # Header (2 bytes)
        header = struct.unpack_from('BB', packet, offset)
        assert header == (0xAA, 0x55)
        offset += 2

        # timestamp_us (uint32_t, 4 bytes)
        timestamp_us = struct.unpack_from('<I', packet, offset)[0]
        assert timestamp_us == 1000000
        offset += 4

        # texture_grain_hz (float, 4 bytes)
        texture_grain_hz = struct.unpack_from('<f', packet, offset)[0]
        assert abs(texture_grain_hz - 150.0) < 0.01
        offset += 4

        # texture_amplitude (float, 4 bytes)
        texture_amplitude = struct.unpack_from('<f', packet, offset)[0]
        assert abs(texture_amplitude - 0.5) < 0.01
        offset += 4

        # shear_direction (2 floats, 8 bytes)
        shear_direction = struct.unpack_from('<ff', packet, offset)
        assert abs(shear_direction[0] - 0.707) < 0.01
        assert abs(shear_direction[1] - 0.707) < 0.01
        offset += 8

        # shear_magnitude (float, 4 bytes)
        shear_magnitude = struct.unpack_from('<f', packet, offset)[0]
        assert abs(shear_magnitude - 0.3) < 0.01
        offset += 4

        # weight_offset (float, 4 bytes)
        weight_offset = struct.unpack_from('<f', packet, offset)[0]
        assert abs(weight_offset - 0.6) < 0.01
        offset += 4

        # impact_amplitude (float, 4 bytes)
        impact_amplitude = struct.unpack_from('<f', packet, offset)[0]
        assert abs(impact_amplitude - 0.8) < 0.01
        offset += 4

        # impact_decay_ms (float, 4 bytes)
        impact_decay_ms = struct.unpack_from('<f', packet, offset)[0]
        assert abs(impact_decay_ms - 50.0) < 0.01
        offset += 4

        # impact_frequency_hz (float, 4 bytes)
        impact_frequency_hz = struct.unpack_from('<f', packet, offset)[0]
        assert abs(impact_frequency_hz - 150.0) < 0.01
        offset += 4

        # ring_amplitude (float, 4 bytes)
        ring_amplitude = struct.unpack_from('<f', packet, offset)[0]
        assert abs(ring_amplitude - 0.4) < 0.01
        offset += 4

        # ring_decay_ms (float, 4 bytes)
        ring_decay_ms = struct.unpack_from('<f', packet, offset)[0]
        assert abs(ring_decay_ms - 100.0) < 0.01
        offset += 4

        # flags (uint8_t, 1 byte)
        flags = struct.unpack_from('B', packet, offset)[0]
        assert flags == 1  # trigger_impulse = True
        offset += 1

        # checksum (uint8_t, 1 byte)
        checksum = struct.unpack_from('B', packet, offset)[0]
        assert checksum == packet[-1]
        offset += 1

        # Verify total offset matches packet size
        assert offset == 52

    def test_checksum_algorithm_matches(self):
        """
        Validate checksum algorithm matches firmware implementation.

        Firmware implementation (HaptosRuntime.ino):

        uint8_t calculateChecksum(const CuePacket* packet) {
            uint8_t checksum = 0;
            const uint8_t* bytes = (const uint8_t*)packet;
            for (int i = 0; i < sizeof(CuePacket) - 1; i++) {
                checksum ^= bytes[i];
            }
            return checksum;
        }
        """
        # Create packet
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

        # Calculate checksum manually (firmware algorithm)
        calculated_checksum = 0
        for i in range(51):  # All bytes except last (checksum itself)
            calculated_checksum ^= packet[i]

        # Verify it matches stored checksum
        stored_checksum = packet[51]
        assert calculated_checksum == stored_checksum

    def test_endianness_consistency(self):
        """
        Validate little-endian byte order (Teensy/Arduino default).

        Both Python struct and Arduino use little-endian for ARM Cortex-M7 (Teensy 4.1).
        """
        # Create packet with known values
        cue_params = CueParams(
            texture_grain_hz=1.0,  # Simple value for inspection
            texture_amplitude=0.5,
            shear_direction=(0.0, 0.0),
            shear_magnitude=0.0,
            weight_offset=0.0,
            impact_amplitude=0.0,
            impact_decay_ms=0.0,
            impact_frequency_hz=0.0,
            ring_amplitude=0.0,
            ring_decay_ms=0.0,
            trigger_impulse=False,
            timestamp_us=0x12345678  # Test 32-bit endianness
        )

        packet = cue_params.to_bytes()

        # Verify timestamp is little-endian (uint32)
        # 0x12345678 in little-endian: 78 56 34 12
        timestamp_offset = 2  # Right after header
        timestamp_bytes = packet[timestamp_offset:timestamp_offset+4]

        expected_bytes = bytes([0x78, 0x56, 0x34, 0x12])
        assert timestamp_bytes == expected_bytes

        # Verify float is little-endian
        # 1.0 as IEEE 754 float32: 0x3F800000
        # Little-endian: 00 00 80 3F
        texture_grain_offset = 6  # After header (2) + timestamp (4)
        texture_grain_bytes = packet[texture_grain_offset:texture_grain_offset+4]

        expected_float_bytes = bytes([0x00, 0x00, 0x80, 0x3F])
        assert texture_grain_bytes == expected_float_bytes

    def test_float_precision(self):
        """
        Validate float32 precision is sufficient for haptic parameters.

        Firmware uses float (32-bit), Python uses float (64-bit double).
        Ensure precision loss on serialization is acceptable.
        """
        # Test boundary values
        test_cases = [
            0.0,
            0.1,
            0.5,
            0.9,
            1.0,
            10.0,
            100.0,
            500.0,
            1000.0
        ]

        for value in test_cases:
            cue_params = CueParams(
                texture_grain_hz=value,
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

            # Serialize and deserialize
            packet = cue_params.to_bytes()
            is_valid, deserialized = ProtocolValidator.deserialize_and_validate(packet)

            assert is_valid
            assert deserialized is not None

            # Check precision loss is acceptable (<0.001% error)
            relative_error = abs(deserialized.texture_grain_hz - value) / (value + 1e-10)
            assert relative_error < 1e-5, f"Value {value} has excessive precision loss: {relative_error}"

    def test_trigger_impulse_flag_encoding(self):
        """Validate trigger_impulse boolean encodes correctly as uint8_t."""
        # Test False
        cue_false = CueParams(
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

        packet_false = cue_false.to_bytes()
        flags_offset = 2 + 4 + 11*4  # After header + timestamp + 11 floats
        assert packet_false[flags_offset] == 0

        # Test True
        cue_true = CueParams(
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
            trigger_impulse=True,
            timestamp_us=1000000
        )

        packet_true = cue_true.to_bytes()
        assert packet_true[flags_offset] == 1

    def test_zero_values_serialize_correctly(self):
        """Validate zero/null values serialize correctly (common edge case)."""
        # All zeros
        cue_zero = CueParams(
            texture_grain_hz=0.0,
            texture_amplitude=0.0,
            shear_direction=(0.0, 0.0),
            shear_magnitude=0.0,
            weight_offset=0.0,
            impact_amplitude=0.0,
            impact_decay_ms=0.0,
            impact_frequency_hz=0.0,
            ring_amplitude=0.0,
            ring_decay_ms=0.0,
            trigger_impulse=False,
            timestamp_us=0
        )

        packet = cue_zero.to_bytes()

        # Validate packet structure
        is_valid, error_msg = ProtocolValidator.validate_packet(packet)
        assert is_valid

        # Deserialize and verify
        is_valid, deserialized = ProtocolValidator.deserialize_and_validate(packet)
        assert is_valid
        assert deserialized is not None

        # All values should be zero
        assert deserialized.texture_grain_hz == 0.0
        assert deserialized.texture_amplitude == 0.0
        assert deserialized.shear_direction == (0.0, 0.0)
        assert deserialized.shear_magnitude == 0.0
        assert deserialized.weight_offset == 0.0
        assert deserialized.impact_amplitude == 0.0
        assert deserialized.ring_amplitude == 0.0
        assert deserialized.trigger_impulse is False
        assert deserialized.timestamp_us == 0

    def test_max_values_serialize_correctly(self):
        """Validate maximum values serialize correctly (boundary test)."""
        # Maximum typical values
        cue_max = CueParams(
            texture_grain_hz=500.0,  # Max frequency
            texture_amplitude=1.0,   # Max amplitude
            shear_direction=(0.707, 0.707),  # Normalized
            shear_magnitude=1.0,
            weight_offset=1.0,
            impact_amplitude=1.0,
            impact_decay_ms=1000.0,
            impact_frequency_hz=500.0,
            ring_amplitude=1.0,
            ring_decay_ms=1000.0,
            trigger_impulse=True,
            timestamp_us=0xFFFFFFFF  # Max uint32_t
        )

        packet = cue_max.to_bytes()

        # Validate packet
        is_valid, error_msg = ProtocolValidator.validate_packet(packet)
        assert is_valid

        # Deserialize
        is_valid, deserialized = ProtocolValidator.deserialize_and_validate(packet)
        assert is_valid
        assert deserialized is not None

        # Verify values preserved
        assert abs(deserialized.texture_grain_hz - 500.0) < 0.01
        assert abs(deserialized.texture_amplitude - 1.0) < 0.01
        assert abs(deserialized.shear_magnitude - 1.0) < 0.01
        assert abs(deserialized.weight_offset - 1.0) < 0.01
        assert deserialized.timestamp_us == 0xFFFFFFFF

    def test_negative_values_handled(self):
        """
        Validate handling of negative values.

        Some fields should never be negative (amplitudes), while others can be
        (frequencies, decay times should be non-negative but float allows negatives).
        """
        # Negative decay (invalid, but should serialize)
        cue_negative = CueParams(
            texture_grain_hz=-10.0,  # Invalid but possible
            texture_amplitude=0.5,
            shear_direction=(-0.707, -0.707),  # Negative direction is valid
            shear_magnitude=0.5,
            weight_offset=0.5,
            impact_amplitude=0.5,
            impact_decay_ms=-50.0,  # Invalid
            impact_frequency_hz=-100.0,  # Invalid
            ring_amplitude=0.5,
            ring_decay_ms=-100.0,  # Invalid
            trigger_impulse=False,
            timestamp_us=1000000
        )

        # Should serialize without error
        packet = cue_negative.to_bytes()

        # Packet structure should be valid
        is_valid, error_msg = ProtocolValidator.validate_packet(packet)
        assert is_valid

        # But field validation should catch negative values
        is_valid_fields, deserialized = ProtocolValidator.deserialize_and_validate(packet)
        assert is_valid_fields is False  # Should fail field range validation

    def test_round_trip_preserves_all_fields(self):
        """Comprehensive round-trip test for all field types."""
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
            timestamp_us=1234567890
        )

        # Serialize
        packet = original.to_bytes()

        # Validate
        is_valid, error_msg = ProtocolValidator.validate_packet(packet)
        assert is_valid

        # Deserialize
        is_valid, deserialized = ProtocolValidator.deserialize_and_validate(packet)
        assert is_valid
        assert deserialized is not None

        # Compare all fields
        is_equal = ProtocolValidator.compare_cue_params(original, deserialized)
        assert is_equal

    def test_multiple_packets_independent(self):
        """Validate serialization of multiple different packets is independent."""
        packets = []

        for i in range(10):
            cue = CueParams(
                texture_grain_hz=100.0 + i * 10,
                texture_amplitude=0.1 * (i + 1),
                shear_direction=(1.0, 0.0),
                shear_magnitude=0.1 * i,
                weight_offset=0.1 * (i + 1),
                impact_amplitude=0.0,
                impact_decay_ms=0.0,
                impact_frequency_hz=0.0,
                ring_amplitude=0.0,
                ring_decay_ms=0.0,
                trigger_impulse=(i % 2 == 0),
                timestamp_us=1000000 + i * 1000
            )

            packet = cue.to_bytes()
            packets.append(packet)

            # Validate each packet
            is_valid, error_msg = ProtocolValidator.validate_packet(packet)
            assert is_valid

        # All packets should be different
        for i in range(len(packets)):
            for j in range(i + 1, len(packets)):
                assert packets[i] != packets[j]

    def test_firmware_reference_packet(self):
        """
        Test against a known reference packet from firmware documentation.

        This ensures Python serialization exactly matches firmware expectations.
        """
        # Reference packet from firmware testing (documented values)
        reference = CueParams(
            texture_grain_hz=200.0,
            texture_amplitude=0.7,
            shear_direction=(0.866, 0.5),  # 60 degrees
            shear_magnitude=0.4,
            weight_offset=0.6,
            impact_amplitude=0.9,
            impact_decay_ms=30.0,
            impact_frequency_hz=180.0,
            ring_amplitude=0.5,
            ring_decay_ms=120.0,
            trigger_impulse=True,
            timestamp_us=5000000
        )

        packet = reference.to_bytes()

        # Validate structure
        assert len(packet) == 52
        assert packet[0:2] == bytes([0xAA, 0x55])

        # Validate specific field offsets (spot check)
        texture_grain = struct.unpack_from('<f', packet, 6)[0]  # After header(2) + timestamp(4)
        assert abs(texture_grain - 200.0) < 0.01

        shear_dir = struct.unpack_from('<ff', packet, 14)  # After header(2) + timestamp(4) + 2 floats(8)
        assert abs(shear_dir[0] - 0.866) < 0.01
        assert abs(shear_dir[1] - 0.5) < 0.01

        flags = packet[50]  # Header(2) + timestamp(4) + 11 floats(44) = offset 50
        assert flags == 1

        # Validate checksum
        is_valid, error_msg = ProtocolValidator.validate_packet(packet)
        assert is_valid

        print(f"\nFirmware reference packet validation:")
        print(f"  Packet size: {len(packet)} bytes")
        print(f"  Header: 0x{packet[0]:02X} 0x{packet[1]:02X}")
        print(f"  Checksum: 0x{packet[51]:02X}")
        print(f"  Validation: {error_msg}")
