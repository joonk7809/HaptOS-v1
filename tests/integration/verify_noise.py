import sys
import os
sys.path.append(os.getcwd() + "/src")

import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from data_generator.noise_injector import NoiseInjector
from physics.mujoco_engine import ContactPatch

def verify_noise():
    print("Running Spectrogram Test...")
    
    # 1. Generate 1 second of "Silence" (Zero Force)
    fs = 1000
    duration_s = 1.0
    n_steps = int(fs * duration_s)
    
    # Create dummy contact patches
    clean_history = []
    for i in range(n_steps):
        patch = ContactPatch(
            timestamp_us=i*1000,
            normal_force_N=0.0,
            shear_force_N=0.0,
            slip_speed_mms=0.0,
            contact_pos=np.zeros(3),
            contact_normal=np.array([0,0,1]),
            in_contact=False,
            mu_static=0.0,
            mu_dynamic=0.0,
            solref_damping=0.0
        )
        clean_history.append(patch)

    # 2. Inject Noise
    # Use higher mains amp to be clearly visible in test
    injector = NoiseInjector(thermal_noise_std=0.005, mains_hum_amp=0.01, jitter_prob=0.0)
    noisy_history = injector.apply_noise(clean_history)
    
    # Extract signal
    noisy_signal = np.array([p.normal_force_N for p in noisy_history])
    t = np.linspace(0, duration_s, len(noisy_signal))

    # 3. Frequency Analysis (FFT)
    yf = fft(noisy_signal)
    xf = fftfreq(len(noisy_signal), 1 / fs)
    
    # Get positive frequencies
    n_half = len(noisy_signal) // 2
    xf_plot = xf[:n_half]
    yf_plot = 2.0/len(noisy_signal) * np.abs(yf[0:n_half])

    # 4. Plot and Save
    plt.figure(figsize=(10, 6))
    
    plt.subplot(2, 1, 1)
    plt.title("Time Domain: What the Teensy Sees")
    plt.plot(t[:200], noisy_signal[:200]) # Zoom in
    plt.ylabel("Force (N)")
    plt.grid(True, alpha=0.3)

    plt.subplot(2, 1, 2)
    plt.title("Frequency Domain: The 'Fingerprint'")
    plt.plot(xf_plot, yf_plot)
    plt.axvline(x=60, color='r', linestyle='--', label='60Hz Mains Hum')
    plt.xlim(0, 200) # Focus on low freq
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    output_path = "tests/spectrogram_test.png"
    plt.tight_layout()
    plt.savefig(output_path)
    print(f"Plot saved to {output_path}")
    
    # Automated check
    # Find peak frequency
    peak_idx = np.argmax(yf_plot[1:]) + 1 # Ignore DC
    peak_freq = xf_plot[peak_idx]
    print(f"Peak Frequency: {peak_freq:.1f} Hz")
    
    if abs(peak_freq - 60.0) < 5.0:
        print("SUCCESS: 60Hz Mains Hum detected.")
    else:
        print(f"FAILURE: Expected 60Hz, found {peak_freq:.1f} Hz")

if __name__ == "__main__":
    verify_noise()
