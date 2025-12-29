import sys
sys.path.append('src')

from audio.synthesizer import ContactAudioSynthesizer, AudioConfig
import numpy as np
import soundfile as sf
import matplotlib.pyplot as plt

def test_audio_synthesizer():
    synth = ContactAudioSynthesizer()
    
    # Simulate a tap: force rises then decays
    duration_s = 0.5
    n_steps = int(duration_s * 100)  # 100 Hz control rate
    
    audio_chunks = []
    force_profile = []
    
    for i in range(n_steps):
        t = i / 100.0
        
        # Simulated force: exponential decay
        force = 10.0 * np.exp(-t * 20) if t < 0.3 else 0.0
        shear = 0.5 * force
        slip = 2.0 if force > 0.1 else 0.0
        in_contact = force > 0.01
        
        force_profile.append(force)
        
        # Generate audio
        audio_chunk = synth.synthesize_step(force, shear, slip, in_contact)
        audio_chunks.append(audio_chunk)
    
    # Concatenate all audio
    audio = np.concatenate(audio_chunks)
    
    # Save to file
    sf.write('logs/audio_test.wav', audio, synth.sr)
    
    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))
    
    # Force profile
    time_force = np.arange(len(force_profile)) / 100.0
    ax1.plot(time_force, force_profile)
    ax1.set_ylabel('Force (N)')
    ax1.set_title('Input Force Profile')
    ax1.grid(True)
    
    # Audio waveform
    time_audio = np.arange(len(audio)) / synth.sr
    ax2.plot(time_audio, audio)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Amplitude')
    ax2.set_title('Generated Audio')
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig('logs/audio_test.png')
    plt.close()
    
    print(f"\n✓ Audio synthesizer test complete")
    print(f"  - Saved audio to: logs/audio_test.wav")
    print(f"  - Saved plot to: logs/audio_test.png")
    print(f"  - Audio length: {len(audio)/synth.sr:.2f}s")
    print(f"  - Sample rate: {synth.sr} Hz")

if __name__ == "__main__":
    test_audio_synthesizer()