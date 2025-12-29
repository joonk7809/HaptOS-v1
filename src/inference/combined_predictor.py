"""
Combined predictor: NN_v0 (baseline) + NN_v1 (delta refinement)
This is the Phase 2 runtime system.
"""

import sys
sys.path.append('src')

import torch
import numpy as np
from pathlib import Path

from models.nn_v0 import NN_v0, NN_v1_Delta

class CombinedPredictor:
    """
    Phase 2 combined predictor.
    Runtime: CueParams_final = clamp(NN_v0(X) + NN_v1(X))
    """
    
    def __init__(self, 
                 nn_v0_path: str = "models/checkpoints/nn_v0_best.pt",
                 nn_v1_path: str = "models/checkpoints/nn_v1_best.pt",
                 device='cpu'):
        self.device = device
        
        # Load NN_v0 (baseline predictor)
        self.nn_v0 = NN_v0(input_dim=13)
        self.nn_v0.load_state_dict(torch.load(nn_v0_path, map_location=device))
        self.nn_v0.eval()
        self.nn_v0.to(device)
        
        # Load NN_v1 (delta predictor)
        self.nn_v1 = NN_v1_Delta(input_dim=13)
        self.nn_v1.load_state_dict(torch.load(nn_v1_path, map_location=device))
        self.nn_v1.eval()
        self.nn_v1.to(device)
        
        print(f"✓ Loaded combined predictor:")
        print(f"  NN_v0: {nn_v0_path}")
        print(f"  NN_v1: {nn_v1_path}")
        print(f"  Total parameters: {self.nn_v0.count_parameters() + self.nn_v1.count_parameters():,}")
    
    def preprocess_features(self, feature_vec: dict) -> torch.Tensor:
        """Convert FeatureVec dict to model input tensor"""
        # Handle One-Hot Phase
        if 'phase_one_hot' in feature_vec:
            phase_one_hot = feature_vec['phase_one_hot']
        else:
            # Encode from string if one-hot not present
            phase_map = {
                "PHASE_NO_CONTACT": 0,
                "PHASE_IMPACT": 1,
                "PHASE_HOLD": 2,
                "PHASE_SLIP": 3,
                "PHASE_RELEASE": 4
            }
            phase_idx = phase_map.get(feature_vec.get('phase', 'PHASE_NO_CONTACT'), 0)
            phase_one_hot = [0.0] * 5
            phase_one_hot[phase_idx] = 1.0
        
        # Handle Log Forces
        if 'normal_force_log' in feature_vec:
            normal_force_log = feature_vec['normal_force_log']
        else:
            normal_force_log = np.log10(feature_vec['normal_force_N'] + 1.0)
            
        if 'shear_force_log' in feature_vec:
            shear_force_log = feature_vec['shear_force_log']
        else:
            shear_force_log = np.log10(feature_vec['shear_force_N'] + 1.0)
        
        features = (
            phase_one_hot + 
            [
                feature_vec['phase_confidence_01'],
                normal_force_log,
                shear_force_log,
                feature_vec['slip_speed_mms'],
                feature_vec['material_features']['hardness_01'],
                feature_vec['material_features']['mu_01'],
                feature_vec['material_features']['roughness_rms_um'],
                feature_vec['uncertainty_pct']
            ]
        )
        
        return torch.tensor([features], dtype=torch.float32)
    
    def predict(self, feature_vec: dict, return_components: bool = False) -> dict:
        """
        Predict final haptic cues using combined model.
        
        Args:
            feature_vec: FeatureVec as dict
            return_components: If True, return (baseline, delta, final)
            
        Returns:
            CueParams dict (or tuple if return_components=True)
        """
        # Preprocess
        x = self.preprocess_features(feature_vec).to(self.device)
        
        with torch.no_grad():
            # Get baseline from NN_v0
            baseline = self.nn_v0(x)
            
            # Get delta from NN_v1
            delta = self.nn_v1(x)
            
            # Combine: final = baseline + delta (with clamping)
            final = self._add_and_clamp(baseline, delta)
        
        # Convert to dict
        final_cues = self._to_dict(final)
        
        if return_components:
            baseline_cues = self._to_dict(baseline)
            delta_cues = self._to_dict(delta)
            return baseline_cues, delta_cues, final_cues
        
        return final_cues
    
    def _add_and_clamp(self, baseline, delta):
        """Add delta to baseline and clamp to valid ranges"""
        final = {}
        
        # Impact
        final['impact'] = {
            'A': torch.clamp(baseline['impact']['A'] + delta['impact']['A'], 0.0, 1.0),
            'rise_ms': torch.clamp(baseline['impact']['rise_ms'] + delta['impact']['rise_ms'], 0.5, 10.0),
            'fall_ms': torch.clamp(baseline['impact']['fall_ms'] + delta['impact']['fall_ms'], 2.0, 50.0),
            'hf_weight': torch.clamp(baseline['impact']['hf_weight'] + delta['impact']['hf_weight'], 0.0, 1.0)
        }
        
        # Ring (f_Hz locked, tau_ms and a have deltas)
        final['ring'] = {
            'f_Hz': baseline['ring']['f_Hz'],  # No delta
            'tau_ms': torch.clamp(baseline['ring']['tau_ms'] + delta['ring']['tau_ms'], 20.0, 1000.0),
            'a': torch.clamp(baseline['ring']['a'] + delta['ring']['a'], 0.0, 1.0)
        }
        
        # Shear
        final['shear'] = {
            'A': torch.clamp(baseline['shear']['A'] + delta['shear']['A'], 0.0, 1.0),
            'band_Hz': baseline['shear']['band_Hz']
        }
        
        # Weight
        final['weight'] = {
            'A': torch.clamp(baseline['weight']['A'] + delta['weight']['A'], 0.0, 1.0),
            'rate_ms': torch.clamp(baseline['weight']['rate_ms'] + delta['weight']['rate_ms'], 10.0, 200.0)
        }
        
        # Texture
        final['texture'] = {
            'A': torch.clamp(baseline['texture']['A'] + delta['texture']['A'], 0.0, 1.0),
            'color': baseline['texture']['color']
        }
        
        return final
    
    def _to_dict(self, cues):
        """Convert tensor predictions to dict"""
        return {
            'impact': {
                'A': float(cues['impact']['A'][0].cpu()),
                'rise_ms': float(cues['impact']['rise_ms'][0].cpu()),
                'fall_ms': float(cues['impact']['fall_ms'][0].cpu()),
                'hf_weight': float(cues['impact']['hf_weight'][0].cpu())
            },
            'ring': {
                'f_Hz': cues['ring']['f_Hz'][0].cpu().numpy().tolist(),
                'tau_ms': cues['ring']['tau_ms'][0].cpu().numpy().tolist(),
                'a': cues['ring']['a'][0].cpu().numpy().tolist()
            },
            'shear': {
                'A': float(cues['shear']['A'][0].cpu()),
                'band_Hz': [float(cues['shear']['band_Hz'][0].cpu())]
            },
            'weight': {
                'A': float(cues['weight']['A'][0].cpu()),
                'rate_ms': float(cues['weight']['rate_ms'][0].cpu())
            },
            'texture': {
                'A': float(cues['texture']['A'][0].cpu()),
                'color': self._decode_color(cues['texture']['color'][0].cpu().numpy())
            }
        }
    
    def _decode_color(self, color_probs: np.ndarray) -> str:
        color_map = {0: "COLOR_WHITE", 1: "COLOR_PINK", 2: "COLOR_BROWN"}
        return color_map[int(np.argmax(color_probs))]