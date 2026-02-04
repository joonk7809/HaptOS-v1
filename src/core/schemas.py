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


# ============================================================================
# SENSATION MODEL SCHEMAS (v2.0 - Perceptually-grounded architecture)
# ============================================================================

@dataclass
class SensationParams:
    """
    Universal haptic sensation description.

    Output of Layer 2 (Neural Renderer), input to Layer 3 (Hardware Driver).
    All float values normalized [0.0, 1.0].

    This replaces CueParams with a perceptually-grounded, hardware-agnostic format.
    Parameters map to human-discriminable perceptual dimensions rather than
    synthesis instructions.

    Binary packet size: 38 bytes (vs 52 for CueParams)
    """

    # Impact Channel (transient events)
    impact_intensity: float     # How hard: 0=none, 1=max perceivable
    impact_sharpness: float     # Character: 0=soft thud, 1=hard click
    impact_trigger: bool        # Onset event flag (rising edge = fire)

    # Resonance Channel (material vibration)
    resonance_intensity: float  # How much ringing after impact
    resonance_brightness: float # Spectral quality: 0=dark/low freq, 1=bright/high freq
    resonance_sustain: float    # Duration: 0=dead/damped, 1=long ring

    # Texture Channel (surface feel)
    texture_roughness: float    # Coarseness: 0=smooth, 1=rough
    texture_density: float      # Grain: 0=sparse bumps, 1=fine grain
    texture_depth: float        # Relief: 0=shallow, 1=deep grooves

    # Slip Channel (lateral motion)
    slip_speed: float           # Velocity: 0=static, 1=fast slide
    slip_direction: Tuple[float, float]  # Unit vector (x,y) in contact plane
    slip_grip: float            # Friction: 0=sticky/high friction, 1=slippery

    # Pressure Channel (sustained force)
    pressure_magnitude: float   # Force: 0=light touch, 1=heavy press
    pressure_spread: float      # Distribution: 0=point contact, 1=diffuse/wide area

    # Metadata
    body_part_id: int
    timestamp_us: int

    def to_bytes(self) -> bytes:
        """
        Serialize SensationParams to binary format for serial transmission.

        Packet format (38 bytes):
        - Header: 0xBB 0x66 (2 bytes) - distinguishes from CueParams (0xAA 0x55)
        - Version: uint8_t (1 byte) - schema version for forward compatibility
        - Body Part ID: uint8_t (1 byte)
        - Timestamp: uint32_t (4 bytes)
        - Impact: 2×float16 + uint8_t (5 bytes)
          - impact_intensity: float16 (2)
          - impact_sharpness: float16 (2)
          - impact_trigger: uint8_t (1)
        - Resonance: 3×float16 (6 bytes)
          - resonance_intensity: float16 (2)
          - resonance_brightness: float16 (2)
          - resonance_sustain: float16 (2)
        - Texture: 3×float16 (6 bytes)
          - texture_roughness: float16 (2)
          - texture_density: float16 (2)
          - texture_depth: float16 (2)
        - Slip: 4×float16 (8 bytes)
          - slip_speed: float16 (2)
          - slip_direction[0]: float16 (2)
          - slip_direction[1]: float16 (2)
          - slip_grip: float16 (2)
        - Pressure: 2×float16 (4 bytes)
          - pressure_magnitude: float16 (2)
          - pressure_spread: float16 (2)
        - Checksum: uint8_t (1 byte) - XOR of all bytes

        Returns:
            bytes: 38-byte packed binary data
        """
        # Pack header and metadata
        data = struct.pack(
            '<BBBBI',  # Header (2), version (1), body_part_id (1), timestamp (4)
            0xBB, 0x66,  # New header for SensationParams
            1,  # Schema version
            self.body_part_id & 0xFF,
            self.timestamp_us & 0xFFFFFFFF
        )

        # Convert float values to float16 (half precision)
        # float16 gives ~3 decimal places in [0,1] range, sufficient for perception
        data += np.array([self.impact_intensity], dtype=np.float16).tobytes()
        data += np.array([self.impact_sharpness], dtype=np.float16).tobytes()
        data += struct.pack('B', int(self.impact_trigger))

        data += np.array([self.resonance_intensity], dtype=np.float16).tobytes()
        data += np.array([self.resonance_brightness], dtype=np.float16).tobytes()
        data += np.array([self.resonance_sustain], dtype=np.float16).tobytes()

        data += np.array([self.texture_roughness], dtype=np.float16).tobytes()
        data += np.array([self.texture_density], dtype=np.float16).tobytes()
        data += np.array([self.texture_depth], dtype=np.float16).tobytes()

        data += np.array([self.slip_speed], dtype=np.float16).tobytes()
        data += np.array([self.slip_direction[0]], dtype=np.float16).tobytes()
        data += np.array([self.slip_direction[1]], dtype=np.float16).tobytes()
        data += np.array([self.slip_grip], dtype=np.float16).tobytes()

        data += np.array([self.pressure_magnitude], dtype=np.float16).tobytes()
        data += np.array([self.pressure_spread], dtype=np.float16).tobytes()

        # Compute checksum (XOR of all bytes)
        checksum = 0
        for byte in data:
            checksum ^= byte

        # Append checksum
        return data + struct.pack('B', checksum)

    @staticmethod
    def from_bytes(data: bytes) -> 'SensationParams':
        """
        Deserialize SensationParams from binary format.

        Args:
            data: 38-byte packed binary data

        Returns:
            SensationParams: Deserialized parameters

        Raises:
            ValueError: If header invalid, version unsupported, or checksum mismatch
        """
        if len(data) != 38:
            raise ValueError(f"Expected 38 bytes, got {len(data)}")

        # Verify checksum
        checksum = 0
        for byte in data[:-1]:
            checksum ^= byte

        if checksum != data[-1]:
            raise ValueError(f"Checksum mismatch: expected {checksum}, got {data[-1]}")

        # Unpack header and metadata
        header1, header2, version, body_part_id, timestamp_us = struct.unpack('<BBBBI', data[0:8])

        # Verify header
        if header1 != 0xBB or header2 != 0x66:
            raise ValueError(f"Invalid header: {header1:02X} {header2:02X}")

        # Check version
        if version != 1:
            raise ValueError(f"Unsupported schema version: {version}")

        # Unpack float16 values
        offset = 8

        impact_intensity = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2
        impact_sharpness = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2
        impact_trigger = bool(data[offset])
        offset += 1

        resonance_intensity = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2
        resonance_brightness = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2
        resonance_sustain = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2

        texture_roughness = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2
        texture_density = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2
        texture_depth = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2

        slip_speed = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2
        slip_dir_x = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2
        slip_dir_y = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2
        slip_grip = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2

        pressure_magnitude = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2
        pressure_spread = np.frombuffer(data[offset:offset+2], dtype=np.float16)[0]
        offset += 2

        return SensationParams(
            impact_intensity=float(impact_intensity),
            impact_sharpness=float(impact_sharpness),
            impact_trigger=impact_trigger,
            resonance_intensity=float(resonance_intensity),
            resonance_brightness=float(resonance_brightness),
            resonance_sustain=float(resonance_sustain),
            texture_roughness=float(texture_roughness),
            texture_density=float(texture_density),
            texture_depth=float(texture_depth),
            slip_speed=float(slip_speed),
            slip_direction=(float(slip_dir_x), float(slip_dir_y)),
            slip_grip=float(slip_grip),
            pressure_magnitude=float(pressure_magnitude),
            pressure_spread=float(pressure_spread),
            body_part_id=body_part_id,
            timestamp_us=timestamp_us
        )

    def clamp_all(self, min_val: float, max_val: float):
        """
        Clamp all float parameters to [min_val, max_val] in-place.

        Used after somatotopic shaping to ensure values stay in valid range.
        Gain amplification can push values above 1.0, which is clamped here.

        Args:
            min_val: Minimum value (typically 0.0)
            max_val: Maximum value (typically 1.0)
        """
        self.impact_intensity = max(min_val, min(self.impact_intensity, max_val))
        self.impact_sharpness = max(min_val, min(self.impact_sharpness, max_val))

        self.resonance_intensity = max(min_val, min(self.resonance_intensity, max_val))
        self.resonance_brightness = max(min_val, min(self.resonance_brightness, max_val))
        self.resonance_sustain = max(min_val, min(self.resonance_sustain, max_val))

        self.texture_roughness = max(min_val, min(self.texture_roughness, max_val))
        self.texture_density = max(min_val, min(self.texture_density, max_val))
        self.texture_depth = max(min_val, min(self.texture_depth, max_val))

        self.slip_speed = max(min_val, min(self.slip_speed, max_val))
        self.slip_grip = max(min_val, min(self.slip_grip, max_val))

        self.pressure_magnitude = max(min_val, min(self.pressure_magnitude, max_val))
        self.pressure_spread = max(min_val, min(self.pressure_spread, max_val))

        # Note: slip_direction is not clamped as it's a unit vector (can be negative)

    def copy(self) -> 'SensationParams':
        """
        Create a deep copy of this SensationParams.

        Returns:
            SensationParams: Independent copy
        """
        return SensationParams(
            impact_intensity=self.impact_intensity,
            impact_sharpness=self.impact_sharpness,
            impact_trigger=self.impact_trigger,
            resonance_intensity=self.resonance_intensity,
            resonance_brightness=self.resonance_brightness,
            resonance_sustain=self.resonance_sustain,
            texture_roughness=self.texture_roughness,
            texture_density=self.texture_density,
            texture_depth=self.texture_depth,
            slip_speed=self.slip_speed,
            slip_direction=(self.slip_direction[0], self.slip_direction[1]),
            slip_grip=self.slip_grip,
            pressure_magnitude=self.pressure_magnitude,
            pressure_spread=self.pressure_spread,
            body_part_id=self.body_part_id,
            timestamp_us=self.timestamp_us
        )

    def to_dict(self) -> dict:
        """Convert to dictionary for logging/debugging."""
        return {
            'impact': {
                'intensity': self.impact_intensity,
                'sharpness': self.impact_sharpness,
                'trigger': self.impact_trigger
            },
            'resonance': {
                'intensity': self.resonance_intensity,
                'brightness': self.resonance_brightness,
                'sustain': self.resonance_sustain
            },
            'texture': {
                'roughness': self.texture_roughness,
                'density': self.texture_density,
                'depth': self.texture_depth
            },
            'slip': {
                'speed': self.slip_speed,
                'direction': self.slip_direction,
                'grip': self.slip_grip
            },
            'pressure': {
                'magnitude': self.pressure_magnitude,
                'spread': self.pressure_spread
            },
            'body_part_id': self.body_part_id,
            'timestamp_us': self.timestamp_us
        }

    @staticmethod
    def zero(body_part_id: int = 0, timestamp_us: int = 0) -> 'SensationParams':
        """
        Create a zero/silent SensationParams (no haptic output).

        Args:
            body_part_id: Body part identifier
            timestamp_us: Optional timestamp

        Returns:
            SensationParams: All parameters set to zero
        """
        return SensationParams(
            impact_intensity=0.0,
            impact_sharpness=0.0,
            impact_trigger=False,
            resonance_intensity=0.0,
            resonance_brightness=0.0,
            resonance_sustain=0.0,
            texture_roughness=0.0,
            texture_density=0.0,
            texture_depth=0.0,
            slip_speed=0.0,
            slip_direction=(0.0, 0.0),
            slip_grip=0.0,
            pressure_magnitude=0.0,
            pressure_spread=0.0,
            body_part_id=body_part_id,
            timestamp_us=timestamp_us
        )


@dataclass
class SynthesisCommand:
    """
    Device-specific synthesis instructions.

    Output of sensation translator, input to synthesis engine.
    This is an intermediate format between perceptual SensationParams
    and hardware-specific waveform generation.

    Different hardware tiers (VCA/LRA/ERM) produce different SynthesisCommands
    from the same SensationParams input.
    """

    # Impact (one-shot envelope)
    impact_amplitude: float = 0.0
    impact_frequency_hz: float = 0.0
    impact_decay_ms: float = 0.0
    impact_fire: bool = False

    # Resonance (damped oscillation)
    resonance_amplitude: float = 0.0
    resonance_frequency_hz: float = 0.0
    resonance_decay_ms: float = 0.0

    # Texture (continuous filtered noise)
    texture_amplitude: float = 0.0
    texture_center_hz: float = 0.0
    texture_bandwidth_hz: float = 0.0

    # Slip (amplitude-modulated noise)
    slip_amplitude: float = 0.0
    slip_direction: Tuple[float, float] = (0.0, 0.0)
    slip_modulation_hz: float = 0.0

    # Pressure (sustained low-freq)
    pressure_amplitude: float = 0.0
    pressure_frequency_hz: float = 0.0

    # ERM-specific (Tier 3 only)
    erm_duty_cycle: float = 0.0  # 0-1, only used for ERM translators

    def to_dict(self) -> dict:
        """Convert to dictionary for logging/debugging."""
        return {
            'impact': {
                'amplitude': self.impact_amplitude,
                'frequency_hz': self.impact_frequency_hz,
                'decay_ms': self.impact_decay_ms,
                'fire': self.impact_fire
            },
            'resonance': {
                'amplitude': self.resonance_amplitude,
                'frequency_hz': self.resonance_frequency_hz,
                'decay_ms': self.resonance_decay_ms
            },
            'texture': {
                'amplitude': self.texture_amplitude,
                'center_hz': self.texture_center_hz,
                'bandwidth_hz': self.texture_bandwidth_hz
            },
            'slip': {
                'amplitude': self.slip_amplitude,
                'direction': self.slip_direction,
                'modulation_hz': self.slip_modulation_hz
            },
            'pressure': {
                'amplitude': self.pressure_amplitude,
                'frequency_hz': self.pressure_frequency_hz
            },
            'erm': {
                'duty_cycle': self.erm_duty_cycle
            }
        }


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
