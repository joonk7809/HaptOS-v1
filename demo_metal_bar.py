import sys
sys.path.append('src')

import torch
import numpy as np
import matplotlib.pyplot as plt
from inference.combined_predictor import CombinedPredictor

def generate_metal_bar_profile(duration_ms=100):
    """
    Generates a sequence of feature vectors simulating a metal bar dropping.
    """
    timeline = []
    
    # Material properties for Metal
    # Hard, smooth, somewhat slippery
    material = {
        'hardness_01': 0.95,
        'mu_01': 0.3,
        'roughness_rms_um': 0.2
    }
    
    for t in range(duration_ms):
        # Default: No Contact
        phase = "PHASE_NO_CONTACT"
        force = 0.0
        
        # 0-10ms: Air (No Contact)
        if t < 10:
            phase = "PHASE_NO_CONTACT"
            force = 0.0
            
        # 10-25ms: Impact (Sharp spike)
        elif 10 <= t < 25:
            phase = "PHASE_IMPACT"
            # Sharp force spike
            progress = (t - 10) / 15.0
            if progress < 0.2:
                force = 10.0 * (progress / 0.2)  # Rise to 10N
            else:
                force = 10.0 * (1.0 - (progress - 0.2) / 0.8)  # Fall
                
        # 25ms+: Hold (Settled weight)
        else:
            phase = "PHASE_HOLD"
            force = 2.0  # Weight of the bar
            
        # Construct Feature Vector
        fv = {
            'timestamp_us': t * 1000,
            'phase': phase,
            'phase_confidence_01': 1.0,
            'normal_force_N': float(force),
            'shear_force_N': 0.0,
            'slip_speed_mms': 0.0,
            'material_features': material,
            'uncertainty_pct': 0.0
        }
        timeline.append(fv)
        
    return timeline

def main():
    print("Initializing Combined Predictor...")
    predictor = CombinedPredictor(device='cpu')
    
    print("Generating 'Metal Bar Drop' physics profile...")
    timeline = generate_metal_bar_profile(duration_ms=100)
    
    print("Running Inference...")
    results = []
    for fv in timeline:
        cues = predictor.predict(fv)
        results.append(cues)
        
    print("Plotting results...")
    
    # Extract data for plotting
    times = range(len(timeline))
    forces = [fv['normal_force_N'] for fv in timeline]
    
    impact_amp = [r['impact']['A'] for r in results]
    impact_rise = [r['impact']['rise_ms'] for r in results]
    
    ring_freq = [r['ring']['f_Hz'][0] for r in results] # Mode 0
    ring_amp = [r['ring']['a'][0] for r in results]     # Mode 0
    
    # Plot
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    
    # 1. Physics Input
    axs[0].plot(times, forces, 'k-', linewidth=2, label='Input Force (N)')
    axs[0].set_ylabel('Force (N)')
    axs[0].set_title('Input Physics: Metal Bar Drop')
    axs[0].grid(True, alpha=0.3)
    axs[0].legend()
    
    # 2. Impact Response
    axs[1].plot(times, impact_amp, 'r-', linewidth=2, label='Impact Amplitude (0-1)')
    axs[1].set_ylabel('Amplitude')
    axs[1].set_title('Predicted Impact Sensation')
    axs[1].grid(True, alpha=0.3)
    axs[1].legend()
    
    # 3. Ring Response (Metal should ring)
    ax3 = axs[2]
    ax3.plot(times, ring_amp, 'b-', linewidth=2, label='Ring Amplitude')
    ax3.set_ylabel('Ring Amp', color='b')
    ax3.tick_params(axis='y', labelcolor='b')
    
    ax3_twin = ax3.twinx()
    ax3_twin.plot(times, ring_freq, 'g--', linewidth=1.5, label='Ring Freq (Hz)')
    ax3_twin.set_ylabel('Frequency (Hz)', color='g')
    ax3_twin.tick_params(axis='y', labelcolor='g')
    
    axs[2].set_title('Predicted Ring/Resonance')
    axs[2].set_xlabel('Time (ms)')
    axs[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('metal_bar_demo.png')
    print("✓ Saved demo plot to: metal_bar_demo.png")

if __name__ == "__main__":
    main()
