import sys
sys.path.append('src')

from sync_runner import SynchronizedRunner
import soundfile as sf
import matplotlib.pyplot as plt
import numpy as np

def test_synchronized_runner():
    runner = SynchronizedRunner("assets/contact_scene.xml")
    
    # Run 1 second simulation
    contact_history, audio = runner.run_episode(duration_s=1.0)
    
    # Save audio
    sf.write('logs/sync_test.wav', audio, runner.audio.sr)
    
    # Extract force over time
    times = [p.timestamp_us / 1e6 for p in contact_history]
    forces = [p.normal_force_N for p in contact_history]
    
    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    
    # Physics
    ax1.plot(times, forces, linewidth=0.5)
    ax1.set_ylabel('Normal Force (N)')
    ax1.set_title('Physics: Contact Force @ 1 kHz')
    ax1.grid(True, alpha=0.3)
    
    # Audio
    audio_time = np.arange(len(audio)) / runner.audio.sr
    ax2.plot(audio_time, audio, linewidth=0.3, alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Amplitude')
    ax2.set_title('Audio: Synthesized Sound @ 48 kHz')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('logs/sync_test.png', dpi=150)
    plt.close()
    
    print(f"\n✓ Synchronized system test complete")
    print(f"  - Physics samples: {len(contact_history)} @ 1 kHz")
    print(f"  - Audio samples: {len(audio)} @ 48 kHz")
    print(f"  - Duration: {times[-1]:.3f}s")
    print(f"  - Files saved:")
    print(f"    • logs/sync_test.wav")
    print(f"    • logs/sync_test.png")
    
    # Verify timing alignment
    expected_audio_samples = int(times[-1] * runner.audio.sr)
    actual_audio_samples = len(audio)
    error_ms = abs(expected_audio_samples - actual_audio_samples) / runner.audio.sr * 1000
    
    print(f"\n  Synchronization check:")
    print(f"    Expected audio samples: {expected_audio_samples}")
    print(f"    Actual audio samples: {actual_audio_samples}")
    print(f"    Timing error: {error_ms:.2f} ms")
    
    if error_ms < 1.0:
        print(f"    ✓ Synchronization is GOOD (<1ms error)")
    else:
        print(f"    ⚠ Synchronization drift detected!")

if __name__ == "__main__":
    test_synchronized_runner()