"""
Main Converter: Combines physics and audio analysis to create training pairs.
This is the "Procedural Teacher" for Phase 1.
"""

import numpy as np
from typing import List, Tuple
from dataclasses import dataclass, asdict
import json

from .feature_extractor import FeatureExtractor, FeatureVec
from .audio_analyzer import (
    AudioAnalyzer, ImpactCue, RingCue, ShearCue, WeightCue, TextureCue
)

@dataclass
class CueParams_v0:
    """
    Target output (Y) for Phase 1 training.
    This is what NN_v0 learns to predict.
    """
    impact: dict  # ImpactCue as dict
    ring: dict    # RingCue as dict
    shear: dict   # ShearCue as dict
    weight: dict  # WeightCue as dict
    texture: dict # TextureCue as dict

@dataclass
class TrainingSample:
    """Complete (X, Y) training pair"""
    X: dict  # FeatureVec as dict
    Y: dict  # CueParams_v0 as dict

class Converter:
    """
    The Procedural Teacher: Converts physics + audio -> training samples.
    
    Operates at 100 Hz:
    - Receives 10ms windows of physics (10 samples @ 1kHz)
    - Receives 10ms of audio (480 samples @ 48kHz)
    - Outputs one (X, Y) training pair
    """
    
    def __init__(self, audio_sample_rate: int = 48000):
        self.feature_extractor = FeatureExtractor()
        self.audio_analyzer = AudioAnalyzer(audio_sample_rate)
        
        # Timing
        self.sample_count = 0
        
    def process_window(self,
                      physics_window: List,
                      audio_window: np.ndarray,
                      window_start_us: int) -> TrainingSample:
        """
        Process one 10ms window to create a training sample.
        
        Args:
            physics_window: List of 10 ContactPatch objects (1kHz)
            audio_window: 480 audio samples (48kHz)
            window_start_us: Timestamp of window start
            
        Returns:
            TrainingSample containing (X, Y) pair
        """
        # Extract X: FeatureVec from physics
        feature_vec = self.feature_extractor.extract(physics_window, window_start_us)
        
        # Extract Y: CueParams from audio + physics
        # AudioAnalyzer expects linear force, so un-log it: 10^log_val - 1
        normal_force_linear = (10.0 ** feature_vec.normal_force_log) - 1.0
        
        impact, ring, shear, weight, texture = self.audio_analyzer.analyze(
            audio_window,
            normal_force_linear
        )
        
        # Package as training sample
        X_dict = self._feature_vec_to_dict(feature_vec)
        Y_dict = {
            "impact": asdict(impact),
            "ring": asdict(ring),
            "shear": asdict(shear),
            "weight": asdict(weight),
            "texture": asdict(texture)
        }
        
        sample = TrainingSample(X=X_dict, Y=Y_dict)
        self.sample_count += 1
        
        return sample
    
    def _feature_vec_to_dict(self, fv: FeatureVec) -> dict:
        """Convert FeatureVec to JSON-serializable dict"""
        # Decode phase from one-hot for JSON readability (optional, but good for debugging)
        phase_idx = np.argmax(fv.phase_one_hot)
        phase_map = ["PHASE_NO_CONTACT", "PHASE_IMPACT", "PHASE_HOLD", "PHASE_SLIP", "PHASE_RELEASE"]
        phase_str = phase_map[phase_idx]
        
        # Un-log forces for JSON readability/compatibility with other tools
        normal_force_linear = (10.0 ** fv.normal_force_log) - 1.0
        shear_force_linear = (10.0 ** fv.shear_force_log) - 1.0
        
        return {
            "timestamp_us": fv.timestamp_us,
            "phase": phase_str,  # Keep string for JSON
            "phase_confidence_01": fv.phase_confidence_01,
            "normal_force_N": float(normal_force_linear),
            "shear_force_N": float(shear_force_linear),
            "slip_speed_mms": fv.slip_speed_mms,
            "material_features": asdict(fv.material_features),
            "uncertainty_pct": fv.uncertainty_pct
        }
    
    def reset(self):
        """Reset converter state"""
        self.feature_extractor.reset()
        self.sample_count = 0