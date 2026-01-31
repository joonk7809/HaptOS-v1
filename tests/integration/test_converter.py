import sys
sys.path.append('src')

from sync_runner import SynchronizedRunner
from converter.converter import Converter
import json
import matplotlib.pyplot as plt
import numpy as np

def test_converter():
    """Test the complete converter pipeline"""
    print("Initializing systems...")
    runner = SynchronizedRunner("assets/contact_scene.xml")
    converter = Converter()
    
    # Run simulation
    print("Running 1-second simulation...")
    contact_history, audio = runner.run_episode(duration_s=1.0)
    
    print("\nProcessing with converter (100 Hz)...")
    
    # Process in 10ms windows
    samples_per_window_physics = 10  # 10 samples @ 1kHz
    samples_per_window_audio = 480   # 480 samples @ 48kHz
    
    n_windows = len(contact_history) // samples_per_window_physics
    
    training_samples = []
    
    for i in range(n_windows):
        # Extract windows
        phys_start = i * samples_per_window_physics
        phys_end = phys_start + samples_per_window_physics
        physics_window = contact_history[phys_start:phys_end]
        
        audio_start = i * samples_per_window_audio
        audio_end = audio_start + samples_per_window_audio
        audio_window = audio[audio_start:audio_end]
        
        window_start_us = physics_window[0].timestamp_us
        
        # Convert to training sample
        sample = converter.process_window(
            physics_window,
            audio_window,
            window_start_us
        )
        
        training_samples.append(sample)
        
        # Print progress
        if i % 10 == 0:
            progress = 100 * i / n_windows
            print(f"  Progress: {progress:.0f}%", end='\r')
    
    print(f"  Progress: 100%  ")
    print(f"\n✓ Generated {len(training_samples)} training samples")
    
    # Save first 5 samples as examples
    print("\nExample training samples:")
    for i, sample in enumerate(training_samples[:5]):
        print(f"\n--- Sample {i} ---")
        print(f"X (Input):")
        print(f"  Timestamp: {sample.X['timestamp_us']/1e3:.1f}ms")
        print(f"  Phase: {sample.X['phase']}")
        print(f"  Force: {sample.X['normal_force_N']:.3f}N")
        print(f"Y (Target):")
        print(f"  Impact A: {sample.Y['impact']['A']:.3f}")
        print(f"  Ring modes: {len(sample.Y['ring']['f_Hz'])}")
        print(f"  Weight A: {sample.Y['weight']['A']:.3f}")
    
    # Save to JSON
    output_file = "logs/training_samples.json"
    with open(output_file, 'w') as f:
        # Convert to serializable format
        serializable = [{
            'X': sample.X,
            'Y': sample.Y
        } for sample in training_samples]
        json.dump(serializable, f, indent=2)
    
    print(f"\n✓ Saved all samples to: {output_file}")
    
    # Visualize
    visualize_samples(training_samples)

def visualize_samples(samples):
    """Create visualization of training data"""
    times = [s.X['timestamp_us']/1e6 for s in samples]
    forces = [s.X['normal_force_N'] for s in samples]
    impact_amps = [s.Y['impact']['A'] for s in samples]
    weight_amps = [s.Y['weight']['A'] for s in samples]
    phases = [s.X['phase'] for s in samples]
    
    fig, axes = plt.subplots(4, 1, figsize=(14, 10))
    
    # Force
    axes[0].plot(times, forces, 'b-', linewidth=1.5)
    axes[0].set_ylabel('Normal Force (N)')
    axes[0].set_title('Physics Input (X)', fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    
    # Impact amplitude
    axes[1].plot(times, impact_amps, 'r-', linewidth=1.5)
    axes[1].set_ylabel('Impact Amplitude')
    axes[1].set_title('Target: Impact Cue (Y)', fontweight='bold')
    axes[1].grid(True, alpha=0.3)
    
    # Weight amplitude
    axes[2].plot(times, weight_amps, 'g-', linewidth=1.5)
    axes[2].set_ylabel('Weight Amplitude')
    axes[2].set_title('Target: Weight Cue (Y)', fontweight='bold')
    axes[2].grid(True, alpha=0.3)
    
    # Phase (categorical)
    phase_map = {
        "PHASE_NO_CONTACT": 0,
        "PHASE_IMPACT": 1,
        "PHASE_HOLD": 2,
        "PHASE_SLIP": 3,
        "PHASE_RELEASE": 4
    }
    phase_nums = [phase_map.get(p, 0) for p in phases]
    axes[3].plot(times, phase_nums, 'k-', linewidth=2, drawstyle='steps-post')
    axes[3].set_ylabel('Phase')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_title('Detected Contact Phase', fontweight='bold')
    axes[3].set_yticks([0, 1, 2, 3, 4])
    axes[3].set_yticklabels(['No Contact', 'Impact', 'Hold', 'Slip', 'Release'])
    axes[3].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('logs/converter_output.png', dpi=150)
    print(f"✓ Saved visualization to: logs/converter_output.png")
    plt.close()

if __name__ == "__main__":
    test_converter()