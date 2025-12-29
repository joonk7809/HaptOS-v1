import numpy as np
from dataclasses import replace
from typing import List
import random
from physics.mujoco_engine import ContactPatch

class NoiseInjector:
    """
    Simulates real-world sensor imperfections for Sim-to-Real transfer.
    Target Hardware: Teensy 4.1 + AD620 Load Cell Amp
    """
    
    def __init__(self, 
                 thermal_noise_std: float = 0.005,  # ~5mV noise on 3.3V scale
                 mains_hum_amp: float = 0.002,      # 60Hz hum amplitude
                 jitter_prob: float = 0.005):       # 0.5% chance of dropped/dup frame
        
        self.thermal_std = thermal_noise_std
        self.mains_amp = mains_hum_amp
        self.jitter_prob = jitter_prob
        self.mains_phase = 0.0
        
    def apply_noise(self, history: List[ContactPatch]) -> List[ContactPatch]:
        """
        Apply all noise sources to a sequence of contact patches.
        """
        noisy_history = []
        
        # 1. Clock Jitter (Time Domain Distortion)
        # We iterate through the original history and randomly skip or duplicate
        i = 0
        while i < len(history):
            patch = history[i]
            
            # Roll for jitter
            r = random.random()
            
            if r < self.jitter_prob:
                # Drop frame (skip this index)
                i += 1
                continue
            elif r < self.jitter_prob * 2:
                # Duplicate frame (add twice)
                noisy_history.append(self._corrupt_patch(patch))
                noisy_history.append(self._corrupt_patch(patch))
                i += 1
            else:
                # Normal frame
                noisy_history.append(self._corrupt_patch(patch))
                i += 1
                
        return noisy_history

    def _corrupt_patch(self, patch: ContactPatch) -> ContactPatch:
        """Apply amplitude noise (Thermal + Mains) to a single patch"""
        
        # Update Mains Hum Phase (60Hz at 1kHz sample rate)
        # 60 Hz * 2pi / 1000 Hz = 0.12 * pi per step
        self.mains_phase += 0.12 * np.pi
        hum = self.mains_amp * np.sin(self.mains_phase)
        
        # Thermal Noise
        noise_n = np.random.normal(0, self.thermal_std)
        noise_s = np.random.normal(0, self.thermal_std)
        
        # Apply to forces (Rectified because AD620 is usually unipolar or rectified in software)
        # But for simulation, we just add it. Note: Real sensors drift, but we'll stick to AC noise.
        
        new_normal = max(0.0, patch.normal_force_N + noise_n + hum)
        new_shear = max(0.0, patch.shear_force_N + noise_s + hum)
        
        # Create new patch with corrupted values
        return replace(patch, 
                      normal_force_N=new_normal,
                      shear_force_N=new_shear)
