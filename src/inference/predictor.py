"""
Inference engine for NN_v0.
Loads trained model and makes predictions on new data.
"""

import torch
import numpy as np
from pathlib import Path
import sys
sys.path.append('src')

from models.nn_v0 import NN_v0

class HapticPredictor:
    """
    Real-time haptic cue predictor using trained NN_v0.
    """
    
    def __init__(self, model_path: str, device='cpu'):
        self.device = device
        
        # Load model
        self.model = NN_v0(input_dim=9)
        self.model.load_state_dict(torch.load(model_path, map_location=device))
        self.model.eval()
        self.model.to(device)
        
        print(f"✓ Loaded model from: {model_path}")
        print(f"  Parameters: {self.model.count_parameters():,}")
    
    def preprocess_features(self, feature_vec: dict) -> torch.Tensor:
        """
        Convert FeatureVec dict to model input tensor.
        
        Args:
            feature_vec: Dict with keys matching FeatureVec schema
            
        Returns:
            Tensor of shape [1, 9]
        """
        # Extract features in correct order
        features = [
            self._encode_phase(feature_vec['phase']) / 4.0,  # Normalize 0-1
            feature_vec['phase_confidence_01'],
            feature_vec['normal_force_N'],
            feature_vec['shear_force_N'],
            feature_vec['slip_speed_mms'],
            feature_vec['material_features']['hardness_01'],
            feature_vec['material_features']['mu_01'],
            feature_vec['material_features']['roughness_rms_um'],
            feature_vec['uncertainty_pct']
        ]
        
        return torch.tensor([features], dtype=torch.float32)
    
    def _encode_phase(self, phase_str: str) -> float:
        """Convert phase string to integer"""
        phase_map = {
            "PHASE_NO_CONTACT": 0,
            "PHASE_IMPACT": 1,
            "PHASE_HOLD": 2,
            "PHASE_SLIP": 3,
            "PHASE_RELEASE": 4
        }
        return float(phase_map.get(phase_str, 0))
    
    def predict(self, feature_vec: dict) -> dict:
        """
        Predict haptic cues from input features.
        
        Args:
            feature_vec: FeatureVec as dict
            
        Returns:
            CueParams as dict
        """
        # Preprocess
        x = self.preprocess_features(feature_vec).to(self.device)
        
        # Predict
        with torch.no_grad():
            pred = self.model(x)
        
        # Convert to CPU and extract values
        cue_params = {
            'impact': {
                'A': float(pred['impact']['A'][0].cpu()),
                'rise_ms': float(pred['impact']['rise_ms'][0].cpu()),
                'fall_ms': float(pred['impact']['fall_ms'][0].cpu()),
                'hf_weight': float(pred['impact']['hf_weight'][0].cpu())
            },
            'ring': {
                'f_Hz': pred['ring']['f_Hz'][0].cpu().numpy().tolist(),
                'tau_ms': pred['ring']['tau_ms'][0].cpu().numpy().tolist(),
                'a': pred['ring']['a'][0].cpu().numpy().tolist()
            },
            'shear': {
                'A': float(pred['shear']['A'][0].cpu()),
                'band_Hz': [float(pred['shear']['band_Hz'][0].cpu())]
            },
            'weight': {
                'A': float(pred['weight']['A'][0].cpu()),
                'rate_ms': float(pred['weight']['rate_ms'][0].cpu())
            },
            'texture': {
                'A': float(pred['texture']['A'][0].cpu()),
                'color': self._decode_color(pred['texture']['color'][0].cpu().numpy())
            }
        }
        
        return cue_params
    
    def _decode_color(self, color_probs: np.ndarray) -> str:
        """Convert color probabilities to string"""
        color_map = {0: "COLOR_WHITE", 1: "COLOR_PINK", 2: "COLOR_BROWN"}
        return color_map[int(np.argmax(color_probs))]
    
    def predict_batch(self, feature_vecs: list) -> list:
        """Predict for a batch of feature vectors"""
        return [self.predict(fv) for fv in feature_vecs]