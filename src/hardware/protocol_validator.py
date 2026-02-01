"""
Protocol Validator - Binary packet validation

Validates CueParams binary packet format and integrity.
Ensures Python serialization matches firmware expectations.
"""

from typing import Tuple, Optional
import struct

from src.core.schemas import CueParams


class ProtocolValidator:
    """Validates CueParams binary packet format."""

    # Protocol constants
    HEADER_MAGIC = (0xAA, 0x55)
    PACKET_SIZE = 52  # bytes
    HEADER_SIZE = 2   # bytes

    @staticmethod
    def validate_packet(packet: bytes) -> Tuple[bool, str]:
        """
        Validate binary packet structure.

        Checks:
        - Length (exactly 52 bytes)
        - Header magic bytes (0xAA 0x55)
        - Checksum integrity (XOR of all bytes)

        Args:
            packet: Binary packet to validate

        Returns:
            (is_valid, error_message)
        """
        # Check length
        if len(packet) != ProtocolValidator.PACKET_SIZE:
            return False, f"Invalid packet size: {len(packet)} bytes (expected {ProtocolValidator.PACKET_SIZE})"

        # Check header magic bytes
        if packet[0] != ProtocolValidator.HEADER_MAGIC[0] or packet[1] != ProtocolValidator.HEADER_MAGIC[1]:
            return False, f"Invalid header: 0x{packet[0]:02X} 0x{packet[1]:02X} (expected 0xAA 0x55)"

        # Validate checksum (XOR of all bytes except checksum itself)
        calculated_checksum = 0
        for i in range(ProtocolValidator.PACKET_SIZE - 1):
            calculated_checksum ^= packet[i]

        stored_checksum = packet[ProtocolValidator.PACKET_SIZE - 1]

        if calculated_checksum != stored_checksum:
            return False, f"Checksum mismatch: calculated=0x{calculated_checksum:02X}, stored=0x{stored_checksum:02X}"

        return True, "Valid packet"

    @staticmethod
    def deserialize_and_validate(packet: bytes) -> Tuple[bool, Optional[CueParams]]:
        """
        Deserialize packet and validate field values.

        Args:
            packet: Binary packet to deserialize

        Returns:
            (is_valid, cue_params or None)
        """
        # First validate packet structure
        is_valid, error_msg = ProtocolValidator.validate_packet(packet)
        if not is_valid:
            return False, None

        # Deserialize
        try:
            cue_params = CueParams.from_bytes(packet)
        except Exception as e:
            return False, None

        # Validate field ranges
        is_valid, error_msg = ProtocolValidator.validate_field_ranges(cue_params)
        if not is_valid:
            return False, None

        return True, cue_params

    @staticmethod
    def validate_field_ranges(cue_params: CueParams) -> Tuple[bool, str]:
        """
        Validate CueParams field value ranges.

        Args:
            cue_params: Deserialized CueParams

        Returns:
            (is_valid, error_message)
        """
        # Amplitudes should be 0-1
        if not (0.0 <= cue_params.texture_amplitude <= 1.0):
            return False, f"texture_amplitude out of range: {cue_params.texture_amplitude}"

        if not (0.0 <= cue_params.shear_magnitude <= 1.0):
            return False, f"shear_magnitude out of range: {cue_params.shear_magnitude}"

        if not (0.0 <= cue_params.weight_offset <= 1.0):
            return False, f"weight_offset out of range: {cue_params.weight_offset}"

        if not (0.0 <= cue_params.impact_amplitude <= 1.0):
            return False, f"impact_amplitude out of range: {cue_params.impact_amplitude}"

        if not (0.0 <= cue_params.ring_amplitude <= 1.0):
            return False, f"ring_amplitude out of range: {cue_params.ring_amplitude}"

        # Frequencies should be non-negative
        if cue_params.texture_grain_hz < 0.0:
            return False, f"texture_grain_hz negative: {cue_params.texture_grain_hz}"

        if cue_params.impact_frequency_hz < 0.0:
            return False, f"impact_frequency_hz negative: {cue_params.impact_frequency_hz}"

        # Decay times should be non-negative
        if cue_params.impact_decay_ms < 0.0:
            return False, f"impact_decay_ms negative: {cue_params.impact_decay_ms}"

        if cue_params.ring_decay_ms < 0.0:
            return False, f"ring_decay_ms negative: {cue_params.ring_decay_ms}"

        # Shear direction should be normalized (magnitude close to 1)
        dx, dy = cue_params.shear_direction
        magnitude = (dx**2 + dy**2)**0.5
        if magnitude > 0.01 and abs(magnitude - 1.0) > 0.1:
            return False, f"shear_direction not normalized: magnitude={magnitude}"

        # Timestamp should be positive
        if cue_params.timestamp_us < 0:
            return False, f"timestamp_us negative: {cue_params.timestamp_us}"

        return True, "Valid fields"

    @staticmethod
    def compare_cue_params(
        original: CueParams,
        deserialized: CueParams,
        tolerance: float = 1e-5
    ) -> bool:
        """
        Compare original vs deserialized CueParams (round-trip test).

        Validates serialization preserves data integrity.

        Args:
            original: Original CueParams
            deserialized: Deserialized CueParams
            tolerance: Floating-point comparison tolerance

        Returns:
            True if equal within tolerance
        """
        # Compare floats with tolerance
        def close(a, b):
            return abs(a - b) < tolerance

        if not close(original.texture_grain_hz, deserialized.texture_grain_hz):
            return False
        if not close(original.texture_amplitude, deserialized.texture_amplitude):
            return False
        if not close(original.shear_magnitude, deserialized.shear_magnitude):
            return False
        if not close(original.weight_offset, deserialized.weight_offset):
            return False
        if not close(original.impact_amplitude, deserialized.impact_amplitude):
            return False
        if not close(original.impact_decay_ms, deserialized.impact_decay_ms):
            return False
        if not close(original.impact_frequency_hz, deserialized.impact_frequency_hz):
            return False
        if not close(original.ring_amplitude, deserialized.ring_amplitude):
            return False
        if not close(original.ring_decay_ms, deserialized.ring_decay_ms):
            return False

        # Compare tuples
        if not (close(original.shear_direction[0], deserialized.shear_direction[0]) and
                close(original.shear_direction[1], deserialized.shear_direction[1])):
            return False

        # Compare booleans and integers
        if original.trigger_impulse != deserialized.trigger_impulse:
            return False

        if original.timestamp_us != deserialized.timestamp_us:
            return False

        return True
