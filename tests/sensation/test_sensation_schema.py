#!/usr/bin/env python3
"""
Unit tests for sensation model schemas (SensationParams and SynthesisCommand).

Tests cover:
- Schema creation and defaults
- Binary serialization/deserialization
- float16 precision and bandwidth savings
- Checksum validation
- Helper methods (clamp_all, copy, to_dict)
- Zero/silent parameter generation
"""

import pytest
import numpy as np
import struct
from src.core.schemas import SensationParams, SynthesisCommand


class TestSensationParamsCreation:
    """Test SensationParams creation and basic properties."""

    def test_sensation_params_creation(self):
        """Test creating a SensationParams with all fields."""
        sp = SensationParams(
            impact_intensity=0.7,
            impact_sharpness=0.8,
            impact_trigger=True,
            resonance_intensity=0.5,
            resonance_brightness=0.6,
            resonance_sustain=0.4,
            texture_roughness=0.3,
            texture_density=0.5,
            texture_depth=0.6,
            slip_speed=0.4,
            slip_direction=(0.7071, 0.7071),
            slip_grip=0.3,
            pressure_magnitude=0.8,
            pressure_spread=0.2,
            body_part_id=10,
            timestamp_us=1234567890
        )

        assert sp.impact_intensity == 0.7
        assert sp.impact_sharpness == 0.8
        assert sp.impact_trigger is True
        assert sp.resonance_intensity == 0.5
        assert sp.body_part_id == 10
        assert sp.timestamp_us == 1234567890

    def test_sensation_params_zero(self):
        """Test creating a zero/silent SensationParams."""
        sp = SensationParams.zero(body_part_id=15, timestamp_us=999)

        assert sp.impact_intensity == 0.0
        assert sp.impact_sharpness == 0.0
        assert sp.impact_trigger is False
        assert sp.resonance_intensity == 0.0
        assert sp.texture_roughness == 0.0
        assert sp.slip_speed == 0.0
        assert sp.pressure_magnitude == 0.0
        assert sp.body_part_id == 15
        assert sp.timestamp_us == 999


class TestSensationParamsSerialization:
    """Test binary serialization and deserialization."""

    def test_serialization_packet_size(self):
        """Test that serialized packet is exactly 38 bytes."""
        sp = SensationParams.zero()
        data = sp.to_bytes()

        assert len(data) == 38, f"Expected 38 bytes, got {len(data)}"

    def test_serialization_header(self):
        """Test that header is 0xBB 0x66 (distinguishes from CueParams)."""
        sp = SensationParams.zero()
        data = sp.to_bytes()

        assert data[0] == 0xBB, f"Expected header[0]=0xBB, got {data[0]:02X}"
        assert data[1] == 0x66, f"Expected header[1]=0x66, got {data[1]:02X}"

    def test_serialization_version(self):
        """Test that version field is 1."""
        sp = SensationParams.zero()
        data = sp.to_bytes()

        assert data[2] == 1, f"Expected version=1, got {data[2]}"

    def test_round_trip_serialization(self):
        """Test that serialization → deserialization preserves values."""
        sp_original = SensationParams(
            impact_intensity=0.75,
            impact_sharpness=0.85,
            impact_trigger=True,
            resonance_intensity=0.55,
            resonance_brightness=0.65,
            resonance_sustain=0.45,
            texture_roughness=0.35,
            texture_density=0.55,
            texture_depth=0.65,
            slip_speed=0.45,
            slip_direction=(0.6, 0.8),
            slip_grip=0.35,
            pressure_magnitude=0.85,
            pressure_spread=0.25,
            body_part_id=12,
            timestamp_us=987654321
        )

        # Serialize and deserialize
        data = sp_original.to_bytes()
        sp_recovered = SensationParams.from_bytes(data)

        # Check metadata
        assert sp_recovered.body_part_id == 12
        assert sp_recovered.timestamp_us == 987654321
        assert sp_recovered.impact_trigger is True

        # Check float values (allow small tolerance due to float16 precision)
        tolerance = 0.01  # float16 gives ~3 decimal places
        assert abs(sp_recovered.impact_intensity - 0.75) < tolerance
        assert abs(sp_recovered.impact_sharpness - 0.85) < tolerance
        assert abs(sp_recovered.resonance_intensity - 0.55) < tolerance
        assert abs(sp_recovered.texture_roughness - 0.35) < tolerance
        assert abs(sp_recovered.slip_speed - 0.45) < tolerance
        assert abs(sp_recovered.pressure_magnitude - 0.85) < tolerance

    def test_float16_precision(self):
        """Test that float16 provides sufficient precision (≥3 decimals in [0,1])."""
        # Test a range of values
        test_values = [0.0, 0.1, 0.25, 0.5, 0.75, 0.999, 1.0]

        for val in test_values:
            sp = SensationParams.zero()
            sp.impact_intensity = val

            data = sp.to_bytes()
            sp_recovered = SensationParams.from_bytes(data)

            # float16 should preserve at least 3 decimal places
            assert abs(sp_recovered.impact_intensity - val) < 0.005, \
                f"Value {val} not preserved with sufficient precision"

    def test_checksum_validation(self):
        """Test that checksum correctly validates data integrity."""
        sp = SensationParams.zero()
        data = bytearray(sp.to_bytes())

        # Valid checksum should pass
        sp_valid = SensationParams.from_bytes(bytes(data))
        assert sp_valid is not None

        # Corrupt a byte (change impact_intensity)
        data[10] ^= 0x01  # Flip a bit

        # Invalid checksum should raise ValueError
        with pytest.raises(ValueError, match="Checksum mismatch"):
            SensationParams.from_bytes(bytes(data))

    def test_invalid_header(self):
        """Test that invalid header raises ValueError."""
        sp = SensationParams.zero()
        data = bytearray(sp.to_bytes())

        # Corrupt header
        data[0] = 0xAA  # Wrong header (CueParams header)

        # Recompute checksum for corrupted data
        checksum = 0
        for byte in data[:-1]:
            checksum ^= byte
        data[-1] = checksum

        with pytest.raises(ValueError, match="Invalid header"):
            SensationParams.from_bytes(bytes(data))

    def test_invalid_packet_size(self):
        """Test that wrong packet size raises ValueError."""
        sp = SensationParams.zero()
        data = sp.to_bytes()

        # Truncate packet
        with pytest.raises(ValueError, match="Expected 38 bytes"):
            SensationParams.from_bytes(data[:30])

        # Extend packet
        with pytest.raises(ValueError, match="Expected 38 bytes"):
            SensationParams.from_bytes(data + b'\x00')


class TestSensationParamsHelpers:
    """Test helper methods (clamp_all, copy, to_dict)."""

    def test_clamp_all_within_range(self):
        """Test that clamp_all keeps values within [0,1]."""
        sp = SensationParams.zero()
        sp.impact_intensity = 1.5  # Above range
        sp.resonance_brightness = -0.2  # Below range
        sp.texture_roughness = 0.5  # Within range

        sp.clamp_all(0.0, 1.0)

        assert sp.impact_intensity == 1.0  # Clamped to max
        assert sp.resonance_brightness == 0.0  # Clamped to min
        assert sp.texture_roughness == 0.5  # Unchanged

    def test_clamp_all_custom_range(self):
        """Test clamp_all with custom range."""
        sp = SensationParams.zero()
        sp.impact_intensity = 0.8
        sp.texture_density = 0.3

        sp.clamp_all(0.4, 0.6)

        assert sp.impact_intensity == 0.6  # Clamped to max
        assert sp.texture_density == 0.4  # Clamped to min

    def test_clamp_all_slip_direction_not_clamped(self):
        """Test that slip_direction (unit vector) is not clamped."""
        sp = SensationParams.zero()
        sp.slip_direction = (-0.5, 1.2)  # Outside [0,1]

        sp.clamp_all(0.0, 1.0)

        # Direction should be unchanged (can be negative, >1)
        assert sp.slip_direction == (-0.5, 1.2)

    def test_copy_deep_copy(self):
        """Test that copy() creates an independent copy."""
        sp_original = SensationParams(
            impact_intensity=0.7,
            impact_sharpness=0.8,
            impact_trigger=True,
            resonance_intensity=0.5,
            resonance_brightness=0.6,
            resonance_sustain=0.4,
            texture_roughness=0.3,
            texture_density=0.5,
            texture_depth=0.6,
            slip_speed=0.4,
            slip_direction=(0.7, 0.7),
            slip_grip=0.3,
            pressure_magnitude=0.8,
            pressure_spread=0.2,
            body_part_id=10,
            timestamp_us=123
        )

        sp_copy = sp_original.copy()

        # Modify copy
        sp_copy.impact_intensity = 0.9
        sp_copy.slip_direction = (0.5, 0.5)

        # Original should be unchanged
        assert sp_original.impact_intensity == 0.7
        assert sp_original.slip_direction == (0.7, 0.7)

        # Copy should have new values
        assert sp_copy.impact_intensity == 0.9
        assert sp_copy.slip_direction == (0.5, 0.5)

    def test_to_dict_structure(self):
        """Test that to_dict() produces expected structure."""
        sp = SensationParams.zero(body_part_id=10, timestamp_us=999)
        sp.impact_intensity = 0.7
        sp.texture_roughness = 0.5

        d = sp.to_dict()

        assert 'impact' in d
        assert 'resonance' in d
        assert 'texture' in d
        assert 'slip' in d
        assert 'pressure' in d
        assert 'body_part_id' in d
        assert 'timestamp_us' in d

        assert d['impact']['intensity'] == 0.7
        assert d['texture']['roughness'] == 0.5
        assert d['body_part_id'] == 10
        assert d['timestamp_us'] == 999


class TestSynthesisCommand:
    """Test SynthesisCommand schema."""

    def test_synthesis_command_creation(self):
        """Test creating a SynthesisCommand with default values."""
        cmd = SynthesisCommand()

        assert cmd.impact_amplitude == 0.0
        assert cmd.impact_fire is False
        assert cmd.resonance_amplitude == 0.0
        assert cmd.texture_amplitude == 0.0
        assert cmd.erm_duty_cycle == 0.0

    def test_synthesis_command_with_values(self):
        """Test creating a SynthesisCommand with specific values."""
        cmd = SynthesisCommand(
            impact_amplitude=0.8,
            impact_frequency_hz=250.0,
            impact_decay_ms=10.0,
            impact_fire=True,
            resonance_amplitude=0.5,
            texture_center_hz=150.0,
            erm_duty_cycle=0.7
        )

        assert cmd.impact_amplitude == 0.8
        assert cmd.impact_frequency_hz == 250.0
        assert cmd.impact_fire is True
        assert cmd.resonance_amplitude == 0.5
        assert cmd.erm_duty_cycle == 0.7

    def test_synthesis_command_to_dict(self):
        """Test to_dict() method."""
        cmd = SynthesisCommand(
            impact_amplitude=0.6,
            impact_fire=True,
            texture_center_hz=200.0
        )

        d = cmd.to_dict()

        assert 'impact' in d
        assert 'resonance' in d
        assert 'texture' in d
        assert 'slip' in d
        assert 'pressure' in d
        assert 'erm' in d

        assert d['impact']['amplitude'] == 0.6
        assert d['impact']['fire'] is True
        assert d['texture']['center_hz'] == 200.0


class TestBandwidthSavings:
    """Test bandwidth savings vs CueParams."""

    def test_packet_size_comparison(self):
        """Test that SensationParams is smaller than CueParams."""
        from src.core.schemas import CueParams

        sp = SensationParams.zero()
        sp_data = sp.to_bytes()

        cp = CueParams.zero()
        cp_data = cp.to_bytes()

        assert len(sp_data) == 38  # New format
        assert len(cp_data) == 52  # Old format

        # 27% bandwidth savings
        savings = (len(cp_data) - len(sp_data)) / len(cp_data)
        assert savings > 0.25  # At least 25% savings


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
