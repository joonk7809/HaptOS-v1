#!/usr/bin/env python3
"""
Core data schemas for HAPTOS Platform.

Defines the primary data structures that flow through the three-layer architecture:
- ContactPatch: Raw physics contact data (Layer 1 output)
- FilteredContact: Routed contact with biological filtering (Layer 1→2)
- CueParams: Haptic synthesis parameters (Layer 2 output)
"""

from dataclasses import dataclass
from typing import Tuple, Optional
import struct
import numpy as np


@dataclass
class ContactPatch:
    """
    Raw contact from physics simulation.

    Emitted by Layer 1 (Simulation Engine) at 1kHz.
    Contains mechanical contact properties without biological filtering.
    """
    body_part_id: int           # Which body part contacted (e.g., index fingertip)
    force_normal: float         # Normal force magnitude [N]
    force_shear: Tuple[float, float]  # Tangential force [Fx, Fy] in contact plane [N]
    velocity: Tuple[float, float, float]  # Contact point velocity [vx, vy, vz] [m/s]
    contact_area: float         # Estimated contact area [m²]
    material_hint: int          # Material ID (0 = unknown, >0 = known material)
    timestamp_us: int           # Microsecond timestamp

    def to_dict(self) -> dict:
        """Convert to dictionary for feature extraction."""
        return {
            'body_part_id': self.body_part_id,
            'force_normal': self.force_normal,
            'force_shear_x': self.force_shear[0],
            'force_shear_y': self.force_shear[1],
            'velocity_x': self.velocity[0],
            'velocity_y': self.velocity[1],
            'velocity_z': self.velocity[2],
            'contact_area': self.contact_area,
            'material_hint': self.material_hint,
            'timestamp_us': self.timestamp_us
        }


@dataclass
class FilteredContact:
    """
    Biologically-filtered contact for neural rendering.

    Output of Somatotopic Router, input to Neural Renderer.
    Includes perceptual filtering based on Homunculus table.
    """
    patch: ContactPatch         # Original physics contact
    rendering_tier: int         # Hardware capability tier (1=VCA, 2=LRA, 3=ERM)
    cue_mask: int              # Bitmask of enabled haptic cues

    # Cue type bitmask constants
    CUE_IMPACT = 0b00001       # Transient impact event
    CUE_RING = 0b00010         # Material resonance/ring
    CUE_TEXTURE = 0b00100      # Surface texture vibration
    CUE_SHEAR = 0b01000        # Tangential sliding sensation
    CUE_WEIGHT = 0b10000       # Sustained pressure/weight
    CUE_ALL = 0b11111          # All cues enabled

    def has_cue(self, cue_type: int) -> bool:
        """Check if a specific cue type is enabled."""
        return bool(self.cue_mask & cue_type)

    def enabled_cues(self) -> list:
        """Get list of enabled cue names."""
        cues = []
        if self.has_cue(self.CUE_IMPACT):
            cues.append('impact')
        if self.has_cue(self.CUE_RING):
            cues.append('ring')
        if self.has_cue(self.CUE_TEXTURE):
            cues.append('texture')
        if self.has_cue(self.CUE_SHEAR):
            cues.append('shear')
        if self.has_cue(self.CUE_WEIGHT):
            cues.append('weight')
        return cues


@dataclass
class CueParams:
    """
    Haptic synthesis parameters for hardware rendering.

    Output of Neural Renderer (Layer 2), input to Hardware Driver (Layer 3).
    Contains both continuous (interpolated) and transient (event-triggered) cues.
    """
    # Continuous cues (interpolated by hardware at 2kHz+)
    texture_grain_hz: float         # Texture vibration frequency [Hz]
    texture_amplitude: float        # Texture vibration amplitude [0-1]
    shear_direction: Tuple[float, float]  # Shear force direction [x, y] (normalized)
    shear_magnitude: float          # Shear force magnitude [0-1]
    weight_offset: float            # Sustained pressure offset [0-1]

    # Transient cues (event-triggered, not interpolated)
    impact_amplitude: float         # Impact event amplitude [0-1]
    impact_decay_ms: float          # Impact decay time constant [ms]
    impact_frequency_hz: float      # Impact dominant frequency [Hz]
    ring_amplitude: float           # Material ring amplitude [0-1]
    ring_decay_ms: float            # Ring decay time constant [ms]
    trigger_impulse: bool           # Flag: fire transient event this frame

    timestamp_us: int               # Microsecond timestamp

    def to_bytes(self) -> bytes:
        """
        Serialize CueParams to binary format for serial transmission.

        Packet format (52 bytes):
        - Header: 0xAA 0x55 (2 bytes)
        - Timestamp: uint32_t (4 bytes)
        - Continuous params (20 bytes):
          - texture_grain_hz: float32 (4 bytes)
          - texture_amplitude: float32 (4 bytes)
          - shear_direction[2]: float32[2] (8 bytes)
          - shear_magnitude: float32 (4 bytes)
          - weight_offset: float32 (4 bytes)
        - Transient params (24 bytes):
          - impact_amplitude: float32 (4 bytes)
          - impact_decay_ms: float32 (4 bytes)
          - impact_frequency_hz: float32 (4 bytes)
          - ring_amplitude: float32 (4 bytes)
          - ring_decay_ms: float32 (4 bytes)
        - Flags: uint8_t (1 byte, bit 0 = trigger_impulse)
        - Checksum: uint8_t (1 byte)

        Returns:
            bytes: 52-byte packed binary data
        """
        # Pack all fields
        data = struct.pack(
            '<BBIfffffffffffB',  # 2 bytes, 1 uint32, 11 floats, 1 byte
            0xAA, 0x55,                    # Header
            self.timestamp_us & 0xFFFFFFFF,  # Timestamp (truncate to 32-bit)
            self.texture_grain_hz,
            self.texture_amplitude,
            self.shear_direction[0],
            self.shear_direction[1],
            self.shear_magnitude,
            self.weight_offset,
            self.impact_amplitude,
            self.impact_decay_ms,
            self.impact_frequency_hz,
            self.ring_amplitude,
            self.ring_decay_ms,
            int(self.trigger_impulse)
        )

        # Compute checksum (simple XOR of all bytes)
        checksum = 0
        for byte in data:
            checksum ^= byte

        # Append checksum
        return data + struct.pack('B', checksum)

    @staticmethod
    def from_bytes(data: bytes) -> 'CueParams':
        """
        Deserialize CueParams from binary format.

        Args:
            data: 52-byte packed binary data

        Returns:
            CueParams: Deserialized parameters

        Raises:
            ValueError: If header invalid or checksum mismatch
        """
        if len(data) != 52:
            raise ValueError(f"Expected 52 bytes, got {len(data)}")

        # Verify checksum
        checksum = 0
        for byte in data[:-1]:
            checksum ^= byte

        if checksum != data[-1]:
            raise ValueError(f"Checksum mismatch: expected {checksum}, got {data[-1]}")

        # Unpack data (including checksum byte at end)
        unpacked = struct.unpack('<BBIfffffffffffBB', data)

        # Verify header
        if unpacked[0] != 0xAA or unpacked[1] != 0x55:
            raise ValueError(f"Invalid header: {unpacked[0]:02X} {unpacked[1]:02X}")

        return CueParams(
            timestamp_us=unpacked[2],
            texture_grain_hz=unpacked[3],
            texture_amplitude=unpacked[4],
            shear_direction=(unpacked[5], unpacked[6]),
            shear_magnitude=unpacked[7],
            weight_offset=unpacked[8],
            impact_amplitude=unpacked[9],
            impact_decay_ms=unpacked[10],
            impact_frequency_hz=unpacked[11],
            ring_amplitude=unpacked[12],
            ring_decay_ms=unpacked[13],
            trigger_impulse=bool(unpacked[14])
        )

    def to_dict(self) -> dict:
        """Convert to dictionary for logging/debugging."""
        return {
            'continuous': {
                'texture_grain_hz': self.texture_grain_hz,
                'texture_amplitude': self.texture_amplitude,
                'shear_direction': self.shear_direction,
                'shear_magnitude': self.shear_magnitude,
                'weight_offset': self.weight_offset
            },
            'transient': {
                'impact_amplitude': self.impact_amplitude,
                'impact_decay_ms': self.impact_decay_ms,
                'impact_frequency_hz': self.impact_frequency_hz,
                'ring_amplitude': self.ring_amplitude,
                'ring_decay_ms': self.ring_decay_ms,
                'trigger_impulse': self.trigger_impulse
            },
            'timestamp_us': self.timestamp_us
        }

    @staticmethod
    def zero(timestamp_us: int = 0) -> 'CueParams':
        """
        Create a zero/silent CueParams (no haptic output).

        Args:
            timestamp_us: Optional timestamp

        Returns:
            CueParams: All parameters set to zero
        """
        return CueParams(
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
            timestamp_us=timestamp_us
        )


# Convenience function for creating cue masks
def make_cue_mask(*cue_types: int) -> int:
    """
    Create a cue mask from individual cue type flags.

    Example:
        mask = make_cue_mask(FilteredContact.CUE_IMPACT, FilteredContact.CUE_TEXTURE)
        # Result: 0b00101 (impact + texture enabled)

    Args:
        *cue_types: Variable number of cue type constants

    Returns:
        int: Combined bitmask
    """
    mask = 0
    for cue_type in cue_types:
        mask |= cue_type
    return mask
