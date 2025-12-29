import sys
import os
sys.path.append(os.getcwd() + "/src")

import numpy as np
import matplotlib.pyplot as plt
from data_generator.texture_scanner import TextureScanner

def verify_texture():
    print("Running Texture Synthesis Verification...")
    
    scanner = TextureScanner()
    
    # 1. Generate White Noise Surface
    print("Generating White Noise Surface...")
    scanner.generate_surface(roughness_scale=1.0, pattern="white_noise")
    
    # 2. Run Scan
    print("Scanning at 10 cm/s...")
    data = scanner.run_scan(velocity_cms=10.0, duration_s=0.5)
    
    # 3. Extract Signals
    t = [s.timestamp_us / 1e6 for s in data]
    z_acc = [s.stylus_z_acc for s in data]
    shear = [s.shear_force for s in data]
    
    # 4. Plot
    plt.figure(figsize=(10, 8))
    
    plt.subplot(2, 1, 1)
    plt.title("Stylus Z-Acceleration (Vibration)")
    plt.plot(t, z_acc, color='orange')
    plt.ylabel("Acceleration (m/s^2)")
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 2)
    plt.title("Shear Force (Friction)")
    plt.plot(t, shear, color='blue')
    plt.ylabel("Force (N)")
    plt.xlabel("Time (s)")
    plt.grid(True, alpha=0.3)
    
    output_path = "tests/texture_verification.png"
    plt.tight_layout()
    plt.savefig(output_path)
    print(f"Plot saved to {output_path}")
    
    # 5. Check for AC component
    # We expect significant variance in Z-acc if texture is working
    z_std = np.std(z_acc)
    print(f"Z-Acc Std Dev: {z_std:.4f}")
    
    if z_std > 1.0:
        print("SUCCESS: Significant texture vibration detected.")
    else:
        print("FAILURE: Signal is too flat. Texture physics might be off.")

if __name__ == "__main__":
    verify_texture()
