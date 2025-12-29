"""
Analyzes audio to extract cue parameters.
This implements the "audio prior" rules from your design doc.
"""

import numpy as np
from scipy import signal
from scipy.fft import rfft, rfftfreq
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class ImpactCue:
    """Impact transient parameters"""
    A: float            # Amplitude (0-1)
    rise_ms: float      # Attack time
    fall_ms: float      # Decay time
    hf_weight: float    # High-frequency content weight (0-1)

@dataclass
class RingCue:
    """Resonant ring parameters"""
    f_Hz: List[float]   # Modal frequencies
    tau_ms: List[float] # Decay time constants
    a: List[float]      # Modal amplitudes

@dataclass
class ShearCue:
    """Shear/friction rumble parameters"""
    A: float                # Amplitude
    band_Hz: List[float]    # Center frequencies
    dir2: List[float]       # Direction vector [x, y]

@dataclass
class WeightCue:
    """Weight/pressure sensation"""
    A: float        # Amplitude (tied to normal force)
    rate_ms: float  # Ramp rate

@dataclass
class TextureCue:
    """High-frequency texture noise"""
    color: str      # Noise color: "COLOR_WHITE", "COLOR_PINK", "COLOR_BROWN"
    A: float        # Amplitude

class AudioAnalyzer:
    """
    Analyzes 10ms audio chunks to extract haptic cue parameters.
    This is the "audio prior" that teaches the NN about perceptual qualities.
    """
    
    def __init__(self, sample_rate: int = 48000):
        self.sr = sample_rate
        self.window_size = int(0.01 * sample_rate)  # 10ms = 480 samples
        
        # Frequency bands for analysis
        self.IMPACT_BAND = (100, 2000)      # Hz
        self.RING_BAND = (150, 800)         # Hz
        self.TEXTURE_BAND = (4000, 12000)   # Hz
        self.SHEAR_BAND = (30, 300)         # Hz
        
    def analyze(self, audio_chunk: np.ndarray, 
                normal_force: float) -> Tuple[ImpactCue, RingCue, ShearCue, WeightCue, TextureCue]:
        """
        Analyze 10ms audio chunk and extract all cue parameters.
        
        Args:
            audio_chunk: 480 samples @ 48kHz
            normal_force: Concurrent normal force (for weight cue)
            
        Returns:
            Tuple of (impact, ring, shear, weight, texture) cues
        """
        # Ensure correct size
        if len(audio_chunk) != self.window_size:
            audio_chunk = np.pad(audio_chunk, (0, max(0, self.window_size - len(audio_chunk))))
            if len(audio_chunk) > self.window_size:
                audio_chunk = audio_chunk[:self.window_size]
        
        # Compute spectrum
        spectrum = rfft(audio_chunk * signal.windows.hann(len(audio_chunk)))
        freqs = rfftfreq(len(audio_chunk), 1/self.sr)
        magnitude = np.abs(spectrum)
        
        # Extract each cue type
        impact = self._extract_impact(audio_chunk, magnitude, freqs)
        ring = self._extract_ring(magnitude, freqs)
        shear = self._extract_shear(magnitude, freqs)
        weight = self._extract_weight(normal_force)
        texture = self._extract_texture(magnitude, freqs)
        
        return impact, ring, shear, weight, texture
    
    def _extract_impact(self, audio: np.ndarray, magnitude: np.ndarray, 
                        freqs: np.ndarray) -> ImpactCue:
        """
        Extract impact transient parameters from audio envelope.
        Rule 2: Derived from contact-gated audio envelope.
        """
        # Envelope detection (absolute value + lowpass)
        envelope = np.abs(audio)
        sos = signal.butter(2, 500, 'low', fs=self.sr, output='sos')
        envelope_smooth = signal.sosfilt(sos, envelope)
        
        # Amplitude: peak of envelope
        A = np.max(envelope_smooth)
        A = np.clip(A, 0, 1.0)
        
        # Rise time: time to reach peak
        peak_idx = np.argmax(envelope_smooth)
        rise_samples = peak_idx
        rise_ms = (rise_samples / self.sr) * 1000
        rise_ms = np.clip(rise_ms, 0.5, 10.0)
        
        # Fall time: exponential decay fit (simplified)
        if peak_idx < len(envelope_smooth) - 10:
            tail = envelope_smooth[peak_idx:]
            # Estimate decay by finding when signal drops to 37% (1/e)
            threshold = A * 0.37
            fall_idx = np.where(tail < threshold)[0]
            if len(fall_idx) > 0:
                fall_samples = fall_idx[0]
                fall_ms = (fall_samples / self.sr) * 1000
                fall_ms = np.clip(fall_ms, 2.0, 50.0)
            else:
                fall_ms = 20.0  # Default
        else:
            fall_ms = 20.0
        
        # High-frequency weight: energy above 1kHz
        hf_mask = freqs > 1000
        hf_energy = np.sum(magnitude[hf_mask]**2)
        total_energy = np.sum(magnitude**2) + 1e-10
        hf_weight = np.clip(hf_energy / total_energy, 0, 1.0)
        
        return ImpactCue(
            A=float(A),
            rise_ms=float(rise_ms),
            fall_ms=float(fall_ms),
            hf_weight=float(hf_weight)
        )
    
    def _extract_ring(self, magnitude: np.ndarray, freqs: np.ndarray) -> RingCue:
        """
        Extract resonant modes from spectrum.
        Rule 2: Derived from spectral peaks in ring band.
        """
        # Focus on ring frequency band
        band_mask = (freqs >= self.RING_BAND[0]) & (freqs <= self.RING_BAND[1])
        band_mag = magnitude[band_mask]
        band_freqs = freqs[band_mask]
        
        if len(band_mag) == 0 or len(band_mag) < 10:
            return RingCue(f_Hz=[], tau_ms=[], a=[])
        
        # Calculate minimum distance between peaks (at least 50 Hz apart)
        # Convert 50 Hz to number of samples in spectrum
        freq_resolution = freqs[1] - freqs[0] if len(freqs) > 1 else 1.0
        min_peak_distance = max(1, int(50.0 / freq_resolution))
        
        # Find spectral peaks
        try:
            peaks, properties = signal.find_peaks(
                band_mag, 
                height=np.max(band_mag) * 0.1,  # At least 10% of max
                distance=min_peak_distance
            )
        except (ValueError, Exception):
            # Fallback: no distance constraint
            try:
                peaks, properties = signal.find_peaks(
                    band_mag, 
                    height=np.max(band_mag) * 0.1
                )
            except:
                return RingCue(f_Hz=[], tau_ms=[], a=[])
        
        # Extract up to 3 strongest modes
        n_modes = min(3, len(peaks))
        if n_modes == 0:
            return RingCue(f_Hz=[], tau_ms=[], a=[])
        
        # Sort by amplitude
        peak_heights = properties['peak_heights']
        sorted_indices = np.argsort(peak_heights)[::-1][:n_modes]
        
        f_Hz = []
        tau_ms = []
        a = []
        
        for idx in sorted_indices:
            peak_idx = peaks[idx]
            freq = float(band_freqs[peak_idx])
            amp = float(band_mag[peak_idx])
            
            # Normalize amplitude
            amp_normalized = np.clip(amp / (np.max(band_mag) + 1e-10), 0, 1.0)
            
            # Estimate decay time (higher freq = faster decay, typically)
            # Simplified model: tau inversely proportional to frequency
            tau = 100.0 * (200.0 / (freq + 50))  # ms
            tau = np.clip(tau, 20.0, 200.0)
            
            f_Hz.append(freq)
            tau_ms.append(float(tau))
            a.append(float(amp_normalized))
        
        return RingCue(f_Hz=f_Hz, tau_ms=tau_ms, a=a)
    
    def _extract_shear(self, magnitude: np.ndarray, freqs: np.ndarray) -> ShearCue:
        """
        Extract low-frequency shear/rumble parameters.
        """
        # Energy in shear band
        band_mask = (freqs >= self.SHEAR_BAND[0]) & (freqs <= self.SHEAR_BAND[1])
        band_energy = np.sum(magnitude[band_mask]**2)
        total_energy = np.sum(magnitude**2) + 1e-10
        
        A = np.sqrt(band_energy / total_energy)
        A = np.clip(A, 0, 1.0)
        
        # Find dominant frequency in shear band
        if np.sum(band_mask) > 0:
            band_mag = magnitude[band_mask]
            band_freqs = freqs[band_mask]
            peak_idx = np.argmax(band_mag)
            center_freq = float(band_freqs[peak_idx])
        else:
            center_freq = 100.0
        
        return ShearCue(
            A=float(A),
            band_Hz=[center_freq],
            dir2=[0.0, 1.0]  # Default direction
        )
    
    def _extract_weight(self, normal_force: float) -> WeightCue:
        """
        Extract weight sensation from force.
        Rule 1: ONLY derived from normal_force_N (physics backbone).
        """
        # Direct mapping: force -> amplitude
        # Scale force to 0-1 range (assume max ~20N for fingertip)
        A = np.clip(normal_force / 20.0, 0, 1.0)
        
        # Ramp rate: faster rise for harder impacts
        # Higher force -> faster ramp (shorter time)
        rate_ms = 100.0 / (1.0 + normal_force)
        rate_ms = np.clip(rate_ms, 10.0, 200.0)
        
        return WeightCue(
            A=float(A),
            rate_ms=float(rate_ms)
        )
    
    def _extract_texture(self, magnitude: np.ndarray, freqs: np.ndarray) -> TextureCue:
        """
        Extract texture noise parameters.
        Rule 3: Derived from high-frequency noise floor.
        """
        # Energy in texture band
        band_mask = (freqs >= self.TEXTURE_BAND[0]) & (freqs <= self.TEXTURE_BAND[1])
        band_energy = np.sum(magnitude[band_mask]**2)
        total_energy = np.sum(magnitude**2) + 1e-10
        
        A = np.sqrt(band_energy / total_energy)
        A = np.clip(A * 2.0, 0, 1.0)  # Scale up texture contribution
        
        # Determine noise color from spectral slope in texture band
        if np.sum(band_mask) > 10:
            band_mag = magnitude[band_mask]
            band_freqs = freqs[band_mask]
            
            # Fit line to log-log spectrum
            log_freqs = np.log10(band_freqs + 1)
            log_mags = np.log10(band_mag + 1e-10)
            
            try:
                slope = np.polyfit(log_freqs, log_mags, 1)[0]
                
                # Classify: slope ~0 = white, ~-1 = pink, ~-2 = brown
                if slope > -0.5:
                    color = "COLOR_WHITE"
                elif slope > -1.5:
                    color = "COLOR_PINK"
                else:
                    color = "COLOR_BROWN"
            except:
                color = "COLOR_PINK"  # Default on error
        else:
            color = "COLOR_PINK"  # Default
        
        return TextureCue(
            color=color,
            A=float(A)
        )