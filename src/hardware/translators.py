#!/usr/bin/env python3
"""
Sensation Translators: Perceptual → Hardware-Specific Synthesis

Converts perceptual SensationParams to device-specific SynthesisCommand.
Each translator is optimized for a specific actuator tier:

Tier 1: VCA (Voice Coil Actuator) - Full bandwidth, all channels
Tier 2: LRA (Linear Resonant Actuator) - Narrowband, amplitude modulation
Tier 3: ERM (Eccentric Rotating Mass) - Magnitude only, collapsed channels

Architecture:
    SensationParams (perceptual) → Translator → SynthesisCommand (synthesis)

Usage:
    translator = VCATranslator()
    synthesis_cmd = translator.translate(sensation, body_properties)
"""

from typing import Tuple
from src.core.schemas import SensationParams, SynthesisCommand
from src.routing.somatotopic_router import BodyPartProperties


class SensationTranslator:
    """
    Base class for hardware-specific translators.

    Each translator maps perceptual sensations to synthesis parameters
    based on actuator capabilities.
    """

    def translate(self, sensation: SensationParams, body: BodyPartProperties) -> SynthesisCommand:
        """
        Translate perceptual sensation to synthesis command.

        Args:
            sensation: Perceptual sensation parameters (shaped for body part)
            body: Body part properties (for frequency range, etc.)

        Returns:
            SynthesisCommand with device-specific synthesis parameters
        """
        raise NotImplementedError("Subclasses must implement translate()")

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        """
        Linear interpolation.

        Args:
            a: Start value
            b: End value
            t: Interpolation factor [0, 1]

        Returns:
            Interpolated value
        """
        return a + (b - a) * t


class VCATranslator(SensationTranslator):
    """
    Voice Coil Actuator (Tier 1) - Full Bandwidth.

    Capabilities:
    - Frequency range: 20-500Hz (full perceptual range)
    - All 5 sensation channels supported
    - Direct perceptual → synthesis mapping

    Synthesis Strategy:
    - Impact: Decaying sine burst (frequency from sharpness)
    - Resonance: Damped oscillation (frequency from brightness)
    - Texture: Bandpass-filtered noise (frequency from density)
    - Slip: Amplitude-modulated noise (direction preserved)
    - Pressure: Sustained low-frequency tone
    """

    def translate(self, sensation: SensationParams, body: BodyPartProperties) -> SynthesisCommand:
        """
        Translate sensation to VCA synthesis commands.

        VCA has full bandwidth → direct mapping of all channels.

        Args:
            sensation: Shaped sensation parameters
            body: Body part properties (for frequency range)

        Returns:
            SynthesisCommand with all channels mapped
        """
        freq_min, freq_max = body.freq_range_hz
        cmd = SynthesisCommand()

        # === Impact Channel ===
        # Decaying sine burst triggered by impact_trigger
        cmd.impact_fire = sensation.impact_trigger
        cmd.impact_amplitude = sensation.impact_intensity

        # Sharpness → frequency (sharp = high freq, soft = low freq)
        cmd.impact_frequency_hz = self._lerp(freq_min, freq_max, sensation.impact_sharpness)

        # Sharpness → decay time (sharp = fast decay, soft = slow decay)
        cmd.impact_decay_ms = self._lerp(50.0, 2.0, sensation.impact_sharpness)

        # === Resonance Channel ===
        # Damped oscillation (ring after impact)
        cmd.resonance_amplitude = sensation.resonance_intensity

        # Brightness → frequency (bright = high freq, dark = low freq)
        cmd.resonance_frequency_hz = self._lerp(freq_min, freq_max, sensation.resonance_brightness)

        # Sustain → decay time (long ring vs dead)
        cmd.resonance_decay_ms = self._lerp(5.0, 200.0, sensation.resonance_sustain)

        # === Texture Channel ===
        # Bandpass-filtered noise
        cmd.texture_amplitude = sensation.texture_depth

        # Density → center frequency (fine grain = high freq, coarse = low freq)
        cmd.texture_center_hz = self._lerp(freq_min, freq_max, sensation.texture_density)

        # Roughness → bandwidth (rough = wide band, smooth = narrow band)
        cmd.texture_bandwidth_hz = self._lerp(10.0, 200.0, sensation.texture_roughness)

        # === Slip Channel ===
        # Amplitude-modulated noise with directional component
        # slip_grip inverted: high grip = low slip sensation
        cmd.slip_amplitude = sensation.slip_speed * (1.0 - sensation.slip_grip)

        # Direction preserved from sensation
        cmd.slip_direction = sensation.slip_direction

        # Speed → modulation rate (fast slip = fast AM, slow = slow AM)
        cmd.slip_modulation_hz = self._lerp(5.0, 50.0, sensation.slip_speed)

        # === Pressure Channel ===
        # Sustained low-frequency tone
        cmd.pressure_amplitude = sensation.pressure_magnitude

        # Pressure uses lowest frequency (SA-II mechanoreceptors)
        cmd.pressure_frequency_hz = freq_min

        return cmd


class LRATranslator(SensationTranslator):
    """
    Linear Resonant Actuator (Tier 2) - Narrowband.

    Capabilities:
    - Fixed resonant frequency (~150-200Hz)
    - No frequency control (all signals modulate resonant carrier)
    - Amplitude and timing control only

    Synthesis Strategy:
    - All channels map to amplitude modulation of resonant carrier
    - Impact: Sharp spike at resonant freq
    - Resonance: Duration modulation at resonant freq
    - Texture: Low-frequency AM of carrier (loss of detail)
    - Slip: Low-rate AM
    - Pressure: Sustained low-amplitude carrier

    Trade-offs:
    - Brightness/density information lost (fixed frequency)
    - Reduced texture fidelity (no bandpass filtering)
    - Simpler, cheaper hardware
    """

    def __init__(self, resonant_freq_hz: float = 175.0):
        """
        Initialize LRA translator.

        Args:
            resonant_freq_hz: Resonant frequency of LRA (default: 175Hz)
        """
        self.f_res = resonant_freq_hz

    def translate(self, sensation: SensationParams, body: BodyPartProperties) -> SynthesisCommand:
        """
        Translate sensation to LRA synthesis commands.

        LRA clamps all frequencies to resonance → amplitude/duration only.

        Args:
            sensation: Shaped sensation parameters
            body: Body part properties (unused for LRA, fixed frequency)

        Returns:
            SynthesisCommand with all frequencies = resonant freq
        """
        cmd = SynthesisCommand()

        # === Impact Channel ===
        # Sharp amplitude spike at resonant frequency
        cmd.impact_fire = sensation.impact_trigger
        cmd.impact_amplitude = sensation.impact_intensity
        cmd.impact_frequency_hz = self.f_res  # Fixed to resonance

        # Sharpness → decay time only (frequency fixed)
        cmd.impact_decay_ms = self._lerp(30.0, 5.0, sensation.impact_sharpness)

        # === Resonance Channel ===
        # Same carrier, modulated duration
        cmd.resonance_amplitude = sensation.resonance_intensity
        cmd.resonance_frequency_hz = self.f_res  # Fixed to resonance

        # Sustain → decay time
        cmd.resonance_decay_ms = self._lerp(5.0, 200.0, sensation.resonance_sustain)

        # === Texture Channel ===
        # Amplitude modulation of resonant carrier
        # Roughness and depth collapsed to single amplitude
        cmd.texture_amplitude = sensation.texture_roughness * sensation.texture_depth
        cmd.texture_center_hz = self.f_res  # Fixed to resonance
        cmd.texture_bandwidth_hz = 20.0  # Narrow band around resonance

        # === Slip Channel ===
        # Low-rate amplitude modulation
        cmd.slip_amplitude = sensation.slip_speed * (1.0 - sensation.slip_grip) * 0.5
        cmd.slip_direction = sensation.slip_direction
        cmd.slip_modulation_hz = self._lerp(5.0, 30.0, sensation.slip_speed)

        # === Pressure Channel ===
        # Sustained low-amplitude at resonance
        cmd.pressure_amplitude = sensation.pressure_magnitude * 0.3
        cmd.pressure_frequency_hz = self.f_res  # Fixed to resonance

        return cmd


class ERMTranslator(SensationTranslator):
    """
    Eccentric Rotating Mass (Tier 3) - Magnitude Only.

    Capabilities:
    - Magnitude control only (duty cycle)
    - ~50ms spin-up time (no fast transients)
    - No frequency control, no directionality

    Synthesis Strategy:
    - Collapse all sensation channels to single intensity value
    - Weighted sum prioritizes salient channels
    - Output: duty_cycle ∈ [0, 1]

    Trade-offs:
    - All perceptual detail lost (frequency, texture, direction)
    - Slow response (spin-up latency)
    - Cheapest, most robust hardware
    - Acceptable for low-resolution body parts (torso, upper arm)
    """

    def translate(self, sensation: SensationParams, body: BodyPartProperties) -> SynthesisCommand:
        """
        Translate sensation to ERM synthesis commands.

        ERM collapses all channels to single magnitude value.

        Args:
            sensation: Shaped sensation parameters
            body: Body part properties (unused for ERM)

        Returns:
            SynthesisCommand with only erm_duty_cycle set
        """
        cmd = SynthesisCommand()

        # Collapse all channels to single intensity value
        # Weighted sum prioritizes perceptually salient channels
        intensity = 0.0

        # Impact: highest weight (most salient)
        intensity += sensation.impact_intensity * 1.0

        # Texture: medium weight (secondary importance)
        intensity += sensation.texture_roughness * sensation.texture_depth * 0.5

        # Pressure: medium-low weight (sustained, less salient)
        intensity += sensation.pressure_magnitude * 0.3

        # Slip: low weight (directional, lost in ERM)
        intensity += sensation.slip_speed * 0.4

        # Resonance: low weight (frequency-dependent, lost in ERM)
        intensity += sensation.resonance_intensity * 0.2

        # Clamp to [0, 1]
        cmd.erm_duty_cycle = min(intensity, 1.0)

        # All other fields remain at default (0.0 or False)
        return cmd


def create_translator(tier: int, **kwargs) -> SensationTranslator:
    """
    Factory function to create translator based on hardware tier.

    Args:
        tier: Hardware tier (1=VCA, 2=LRA, 3=ERM)
        **kwargs: Optional parameters for specific translators
                  (e.g., resonant_freq_hz for LRA)

    Returns:
        SensationTranslator instance

    Raises:
        ValueError: If tier is not 1, 2, or 3

    Example:
        >>> translator = create_translator(tier=1)  # VCA
        >>> translator = create_translator(tier=2, resonant_freq_hz=180.0)  # LRA
    """
    if tier == 1:
        return VCATranslator()
    elif tier == 2:
        resonant_freq_hz = kwargs.get('resonant_freq_hz', 175.0)
        return LRATranslator(resonant_freq_hz=resonant_freq_hz)
    elif tier == 3:
        return ERMTranslator()
    else:
        raise ValueError(f"Unknown hardware tier: {tier}. Must be 1 (VCA), 2 (LRA), or 3 (ERM).")


if __name__ == '__main__':
    # Test all three translators
    print("Testing Sensation Translators\n")
    print("="*70)

    from src.routing.somatotopic_router import Homunculus

    # Create test sensation (metal impact on fingertip)
    sensation = SensationParams(
        impact_intensity=0.7,
        impact_sharpness=0.9,
        impact_trigger=True,
        resonance_intensity=0.5,
        resonance_brightness=0.8,
        resonance_sustain=0.7,
        texture_roughness=0.4,
        texture_density=0.6,
        texture_depth=0.5,
        slip_speed=0.3,
        slip_direction=(0.7, 0.7),
        slip_grip=0.4,
        pressure_magnitude=0.6,
        pressure_spread=0.3,
        body_part_id=10,
        timestamp_us=1000000
    )

    # Get body part properties
    homunculus = Homunculus()
    body = homunculus.table['index_tip']

    print("Input Sensation:")
    print(f"  Impact: intensity={sensation.impact_intensity:.2f}, sharpness={sensation.impact_sharpness:.2f}")
    print(f"  Resonance: intensity={sensation.resonance_intensity:.2f}, brightness={sensation.resonance_brightness:.2f}")
    print(f"  Texture: roughness={sensation.texture_roughness:.2f}, density={sensation.texture_density:.2f}")
    print(f"  Slip: speed={sensation.slip_speed:.2f}, grip={sensation.slip_grip:.2f}")
    print(f"  Pressure: magnitude={sensation.pressure_magnitude:.2f}")

    print("\n" + "="*70)

    # Test VCA Translator
    print("\n1. VCA Translator (Tier 1 - Full Bandwidth):")
    vca = VCATranslator()
    vca_cmd = vca.translate(sensation, body)

    print(f"  Impact: amp={vca_cmd.impact_amplitude:.2f}, freq={vca_cmd.impact_frequency_hz:.0f}Hz, decay={vca_cmd.impact_decay_ms:.1f}ms")
    print(f"  Resonance: amp={vca_cmd.resonance_amplitude:.2f}, freq={vca_cmd.resonance_frequency_hz:.0f}Hz, decay={vca_cmd.resonance_decay_ms:.1f}ms")
    print(f"  Texture: amp={vca_cmd.texture_amplitude:.2f}, center={vca_cmd.texture_center_hz:.0f}Hz, bw={vca_cmd.texture_bandwidth_hz:.0f}Hz")
    print(f"  Slip: amp={vca_cmd.slip_amplitude:.2f}, mod={vca_cmd.slip_modulation_hz:.1f}Hz")
    print(f"  Pressure: amp={vca_cmd.pressure_amplitude:.2f}, freq={vca_cmd.pressure_frequency_hz:.0f}Hz")

    # Test LRA Translator
    print("\n2. LRA Translator (Tier 2 - Narrowband):")
    lra = LRATranslator(resonant_freq_hz=175.0)
    lra_cmd = lra.translate(sensation, body)

    print(f"  Impact: amp={lra_cmd.impact_amplitude:.2f}, freq={lra_cmd.impact_frequency_hz:.0f}Hz (fixed), decay={lra_cmd.impact_decay_ms:.1f}ms")
    print(f"  Resonance: amp={lra_cmd.resonance_amplitude:.2f}, freq={lra_cmd.resonance_frequency_hz:.0f}Hz (fixed)")
    print(f"  Texture: amp={lra_cmd.texture_amplitude:.2f}, freq={lra_cmd.texture_center_hz:.0f}Hz (fixed)")
    print(f"  Slip: amp={lra_cmd.slip_amplitude:.2f}")
    print(f"  Pressure: amp={lra_cmd.pressure_amplitude:.2f}")

    # Test ERM Translator
    print("\n3. ERM Translator (Tier 3 - Magnitude Only):")
    erm = ERMTranslator()
    erm_cmd = erm.translate(sensation, body)

    print(f"  Duty Cycle: {erm_cmd.erm_duty_cycle:.2f} (collapsed from all channels)")
    print(f"  All other parameters: 0.0 (not supported)")

    print("\n" + "="*70)

    # Verify frequency clamping
    print("\nFrequency Verification:")
    print(f"  VCA impact freq: {vca_cmd.impact_frequency_hz:.0f}Hz (in range {body.freq_range_hz})")
    print(f"  LRA impact freq: {lra_cmd.impact_frequency_hz:.0f}Hz (fixed resonance)")
    assert body.freq_range_hz[0] <= vca_cmd.impact_frequency_hz <= body.freq_range_hz[1]
    assert lra_cmd.impact_frequency_hz == 175.0
    print("  ✓ Frequency ranges correct")

    # Verify amplitude clamping
    print("\nAmplitude Verification:")
    assert 0.0 <= vca_cmd.impact_amplitude <= 1.0
    assert 0.0 <= lra_cmd.impact_amplitude <= 1.0
    assert 0.0 <= erm_cmd.erm_duty_cycle <= 1.0
    print("  ✓ All amplitudes in [0, 1]")

    # Test factory function
    print("\nFactory Function Test:")
    translator1 = create_translator(tier=1)
    translator2 = create_translator(tier=2, resonant_freq_hz=180.0)
    translator3 = create_translator(tier=3)
    assert isinstance(translator1, VCATranslator)
    assert isinstance(translator2, LRATranslator)
    assert isinstance(translator3, ERMTranslator)
    print("  ✓ Factory function works")

    print("\n" + "="*70)
    print("✓ All translator tests passed!")
