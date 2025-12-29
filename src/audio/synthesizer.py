import numpy as np
from dataclasses import dataclass
from typing import Tuple
from scipy import signal

@dataclass
class AudioConfig:
    sample_rate: int = 48000  # 48 kHz
    buffer_size: int = 480    # 10 ms buffer (synchronized with physics)

class ContactAudioSynthesizer:
    """
    Generates procedural audio synchronized with physics contact.
    This is your 'effect proxy' - the audio that your Converter will analyze.
    """
    
    def __init__(self, config: AudioConfig = AudioConfig()):
        self.config = config
        self.sr = config.sample_rate
        self.dt = 1.0 / self.sr
        
        # Internal state
        self.phase = 0.0
        self.time_samples = 0
        
        # Resonator bank (modal synthesis)
        self.resonators = self._init_resonators()
        
    def _init_resonators(self) -> list:
        """Initialize modal resonators for impact sounds"""
        # These are the modes that will ring when struck
        modes = [
            {'freq': 180, 'decay': 0.05, 'amp': 1.0},   # Fundamental
            {'freq': 310, 'decay': 0.08, 'amp': 0.6},   # Second mode
            {'freq': 520, 'decay': 0.03, 'amp': 0.3},   # Third mode
        ]
        
        resonators = []
        for mode in modes:
            # Create biquad filter for each mode
            Q = mode['freq'] * mode['decay'] * np.pi
            b, a = signal.iirpeak(mode['freq'], Q, fs=self.sr)
            zi = signal.lfilter_zi(b, a)
            resonators.append({
                'b': b, 'a': a, 'zi': zi * 0,
                'amp': mode['amp'], 'decay': mode['decay']
            })
        
        return resonators
    
    def synthesize_step(self, 
                       normal_force: float,
                       shear_force: float, 
                       slip_speed: float,
                       in_contact: bool) -> np.ndarray:
        """
        Synthesize 10ms of audio (480 samples) based on physics state.
        
        This runs at 100 Hz (every 10ms) to match your converter sampling rate.
        Each call processes the physics state and returns 480 audio samples.
        """
        n_samples = self.config.buffer_size
        audio = np.zeros(n_samples)
        
        if not in_contact:
            self.time_samples += n_samples
            return audio
        
        # 1. IMPACT: Sharp transient proportional to force
        # Generate impact excitation (short noise burst)
        impact_duration = int(0.002 * self.sr)  # 2ms
        if self.time_samples % impact_duration < impact_duration:
            impact_noise = np.random.randn(n_samples) * normal_force * 0.1
            # High-pass filter to add brightness
            sos = signal.butter(4, 2000, 'high', fs=self.sr, output='sos')
            impact_noise = signal.sosfilt(sos, impact_noise)
        else:
            impact_noise = np.zeros(n_samples)
        
        # 2. RING: Resonant modes excited by impact
        ring_audio = np.zeros(n_samples)
        excitation = impact_noise + np.random.randn(n_samples) * normal_force * 0.01
        
        for res in self.resonators:
            # Filter excitation through resonator
            filtered, res['zi'] = signal.lfilter(
                res['b'], res['a'], excitation, zi=res['zi']
            )
            ring_audio += filtered * res['amp']
        
        # 3. TEXTURE: Noise floor modulated by slip
        # High-frequency noise that increases with slip speed
        texture_noise = np.random.randn(n_samples) * slip_speed * 0.02
        # Band-pass filter for "roughness"
        sos = signal.butter(2, [4000, 12000], 'band', fs=self.sr, output='sos')
        texture_noise = signal.sosfilt(sos, texture_noise)
        
        # 4. SHEAR: Low-frequency rumble
        shear_audio = np.random.randn(n_samples) * shear_force * 0.05
        sos = signal.butter(2, [30, 200], 'band', fs=self.sr, output='sos')
        shear_audio = signal.sosfilt(sos, shear_audio)
        
        # Mix all components
        audio = impact_noise + ring_audio + texture_noise + shear_audio
        
        # Normalize to prevent clipping
        audio = np.clip(audio, -1.0, 1.0)
        
        self.time_samples += n_samples
        return audio
    
    def reset(self):
        """Reset synthesizer state"""
        self.time_samples = 0
        for res in self.resonators:
            res['zi'] = signal.lfilter_zi(res['b'], res['a']) * 0