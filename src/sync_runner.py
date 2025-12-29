import numpy as np
from physics.mujoco_engine import MuJoCoEngine, ContactPatch
from audio.synthesizer import ContactAudioSynthesizer
from typing import List, Tuple

class SynchronizedRunner:
    """
    Runs physics (1 kHz) and audio (48 kHz) in lockstep.
    
    The key insight: Audio synthesizer runs at 100 Hz (every 10 physics steps),
    generating 480 audio samples per call. This maintains perfect sync.
    """
    
    def __init__(self, model_path: str):
        self.physics = MuJoCoEngine(model_path)
        self.audio = ContactAudioSynthesizer()
        
        # Timing
        self.physics_hz = 1000
        self.audio_control_hz = 100  # How often we update audio synthesis
        self.physics_per_audio = self.physics_hz // self.audio_control_hz  # 10 steps
        
        self.step_count = 0
        
    def run_episode(self, duration_s: float) -> Tuple[List[ContactPatch], np.ndarray]:
        """
        Run synchronized simulation for given duration.
        
        Returns:
            contact_history: List of ContactPatch data at 1 kHz
            audio: Complete audio array at 48 kHz
        """
        n_physics_steps = int(duration_s * self.physics_hz)
        n_audio_updates = n_physics_steps // self.physics_per_audio
        
        contact_history = []
        audio_chunks = []
        
        # Reset both systems
        self.physics.reset()
        self.audio.reset()
        
        print(f"Running {duration_s}s simulation...")
        print(f"  Physics steps: {n_physics_steps} @ {self.physics_hz} Hz")
        print(f"  Audio updates: {n_audio_updates} @ {self.audio_control_hz} Hz")
        
        for audio_step in range(n_audio_updates):
            # Run 10 physics steps (10 ms)
            physics_batch = []
            for _ in range(self.physics_per_audio):
                patch = self.physics.step()
                contact_history.append(patch)
                physics_batch.append(patch)
            
            # Average the physics state over this 10ms window
            # (This is a simple aggregation - you might want something smarter)
            avg_normal = np.mean([p.normal_force_N for p in physics_batch])
            avg_shear = np.mean([p.shear_force_N for p in physics_batch])
            avg_slip = np.mean([p.slip_speed_mms for p in physics_batch])
            any_contact = any(p.in_contact for p in physics_batch)
            
            # Generate 10ms of audio (480 samples @ 48kHz)
            audio_chunk = self.audio.synthesize_step(
                avg_normal, avg_shear, avg_slip, any_contact
            )
            audio_chunks.append(audio_chunk)
            
            # Progress indicator
            if audio_step % 10 == 0:
                progress = 100 * audio_step / n_audio_updates
                print(f"  Progress: {progress:.0f}%", end='\r')
        
        print(f"  Progress: 100%  ")
        
        # Concatenate all audio
        audio = np.concatenate(audio_chunks)
        
        return contact_history, audio