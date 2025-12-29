"""
NN_v0: Baseline neural network for Phase 1.
Maps FeatureVec (X) -> CueParams (Y)
"""

import torch
import torch.nn as nn
import numpy as np
from typing import Dict, Tuple

class CueHead(nn.Module):
    """Generic prediction head for one cue type"""
    def __init__(self, input_dim: int, output_dim: int, hidden_dims: list = [64, 32]):
        super().__init__()
        
        layers = []
        prev_dim = input_dim
        
        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(),
                nn.Dropout(0.1)
            ])
            prev_dim = hidden_dim
        
        # Output layer
        layers.append(nn.Linear(prev_dim, output_dim))
        
        self.net = nn.Sequential(*layers)
    
    def forward(self, x):
        return self.net(x)

class NN_v0(nn.Module):
    """
    Phase 1 baseline network.
    
    Architecture:
    - Shared trunk: processes raw features
    - Multiple heads: one per cue type (impact, ring, shear, weight, texture)
    """
    
    def __init__(self, input_dim: int = 13):
        super().__init__()
        
        # Shared feature encoder
        self.trunk = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(128, 64),
            nn.ReLU()
        )
        
        # Specialized heads for each cue type
        self.impact_head = CueHead(64, 4)  # A, rise_ms, fall_ms, hf_weight
        self.ring_head = CueHead(64, 9)    # 3 modes × (f_Hz, tau_ms, a)
        self.shear_head = CueHead(64, 2)   # A, band_Hz
        self.weight_head = CueHead(64, 2)  # A, rate_ms
        self.texture_head = CueHead(64, 4) # A, color (one-hot 3)
        
    def forward(self, x):
        """
        Args:
            x: [batch, 13] tensor of input features
            
        Returns:
            Dict of predictions for each cue type
        """
        # Shared encoding
        features = self.trunk(x)
        
        # Predict each cue type
        impact = self.impact_head(features)
        ring = self.ring_head(features)
        shear = self.shear_head(features)
        weight = self.weight_head(features)
        texture_raw = self.texture_head(features)
        
        return {
            'impact': {
                'A': torch.sigmoid(impact[:, 0]),  # 0-1
                'rise_ms': torch.relu(impact[:, 1]) + 0.5,  # > 0.5
                'fall_ms': torch.relu(impact[:, 2]) + 2.0,  # > 2.0
                'hf_weight': torch.sigmoid(impact[:, 3])  # 0-1
            },
            'ring': {
                'f_Hz': torch.relu(ring[:, 0:3]) + 100,  # 3 frequencies > 100
                'tau_ms': torch.relu(ring[:, 3:6]) + 20,  # 3 decays > 20
                'a': torch.sigmoid(ring[:, 6:9])  # 3 amplitudes 0-1
            },
            'shear': {
                'A': torch.sigmoid(shear[:, 0]),
                'band_Hz': torch.relu(shear[:, 1]) + 30
            },
            'weight': {
                'A': torch.sigmoid(weight[:, 0]),
                'rate_ms': torch.relu(weight[:, 1]) + 10
            },
            'texture': {
                'A': torch.sigmoid(texture_raw[:, 0]),
                'color': torch.softmax(texture_raw[:, 1:4], dim=1)  # 3-class
            }
        }
    
    def count_parameters(self):
        """Count trainable parameters"""
        return sum(p.numel() for p in self.parameters() if p.requires_grad)


class DatasetLoader:
    """
    Loads HDF5 datasets and prepares them for PyTorch training.
    """
    
    def __init__(self, dataset_path: str):
        import h5py
        self.dataset_path = dataset_path
        
        # Load data
        with h5py.File(dataset_path, 'r') as f:
            # Load X (New 13-dim format)
            # phase_one_hot (5) + confidence (1) + log_forces (2) + slip (1) + material (3) + uncertainty (1)
            self.X = np.concatenate([
                f['X/phase_one_hot'][:],
                f['X/phase_confidence'][:].reshape(-1, 1),
                f['X/normal_force_log'][:].reshape(-1, 1),
                f['X/shear_force_log'][:].reshape(-1, 1),
                f['X/slip_speed_mms'][:].reshape(-1, 1),
                f['X/hardness'][:].reshape(-1, 1),
                f['X/mu'][:].reshape(-1, 1),
                f['X/roughness'][:].reshape(-1, 1),
                f['X/uncertainty'][:].reshape(-1, 1)
            ], axis=1).astype(np.float32)
            
            # Load Y
            self.Y = {
                'impact': {
                    'A': f['Y/impact/A'][:].astype(np.float32),
                    'rise_ms': f['Y/impact/rise_ms'][:].astype(np.float32),
                    'fall_ms': f['Y/impact/fall_ms'][:].astype(np.float32),
                    'hf_weight': f['Y/impact/hf_weight'][:].astype(np.float32)
                },
                'ring': {
                    'f_Hz': f['Y/ring/f_Hz'][:].astype(np.float32),
                    'tau_ms': f['Y/ring/tau_ms'][:].astype(np.float32),
                    'a': f['Y/ring/a'][:].astype(np.float32)
                },
                'shear': {
                    'A': f['Y/shear/A'][:].astype(np.float32),
                    'band_Hz': f['Y/shear/band_Hz'][:].astype(np.float32)
                },
                'weight': {
                    'A': f['Y/weight/A'][:].astype(np.float32),
                    'rate_ms': f['Y/weight/rate_ms'][:].astype(np.float32)
                },
                'texture': {
                    'A': f['Y/texture/A'][:].astype(np.float32),
                    'color': f['Y/texture/color'][:].astype(np.int64)
                }
            }
            
            self.n_samples = len(self.X)
    
    def get_train_val_split(self, val_split: float = 0.2):
        """Split into train/val sets"""
        n_val = int(self.n_samples * val_split)
        indices = np.random.permutation(self.n_samples)
        
        train_indices = indices[n_val:]
        val_indices = indices[:n_val]
        
        return train_indices, val_indices
    
    def get_batch(self, indices):
        """Get a batch of data"""
        X_batch = torch.from_numpy(self.X[indices])
        
        Y_batch = {
            'impact': {
                'A': torch.from_numpy(self.Y['impact']['A'][indices]),
                'rise_ms': torch.from_numpy(self.Y['impact']['rise_ms'][indices]),
                'fall_ms': torch.from_numpy(self.Y['impact']['fall_ms'][indices]),
                'hf_weight': torch.from_numpy(self.Y['impact']['hf_weight'][indices])
            },
            'ring': {
                'f_Hz': torch.from_numpy(self.Y['ring']['f_Hz'][indices]),
                'tau_ms': torch.from_numpy(self.Y['ring']['tau_ms'][indices]),
                'a': torch.from_numpy(self.Y['ring']['a'][indices])
            },
            'shear': {
                'A': torch.from_numpy(self.Y['shear']['A'][indices]),
                'band_Hz': torch.from_numpy(self.Y['shear']['band_Hz'][indices])
            },
            'weight': {
                'A': torch.from_numpy(self.Y['weight']['A'][indices]),
                'rate_ms': torch.from_numpy(self.Y['weight']['rate_ms'][indices])
            },
            'texture': {
                'A': torch.from_numpy(self.Y['texture']['A'][indices]),
                'color': torch.from_numpy(self.Y['texture']['color'][indices])
            }
        }
        
        return X_batch, Y_batch

class NN_v1_Delta(nn.Module):
    """
    Phase 2 delta predictor.
    Key difference: Linear outputs (no sigmoid/relu) because deltas can be negative.
    """
    
    def __init__(self, input_dim: int = 13):
        super().__init__()
        
        # Shared trunk
        self.trunk = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(128, 64),
            nn.ReLU()
        )
        
        # Simple linear heads (no activation functions on output!)
        self.impact_head = nn.Linear(64, 4)   # A, rise_ms, fall_ms, hf_weight
        self.weight_head = nn.Linear(64, 2)   # A, rate_ms
        self.shear_head = nn.Linear(64, 1)    # A only
        self.texture_head = nn.Linear(64, 1)  # A only (color not delta'd)
        
        # Ring head now predicts deltas for tau_ms (3) and a (3)
        # f_Hz is still locked
        self.ring_head = nn.Linear(64, 6)     # tau_ms x3, a x3
    
    def forward(self, x):
        """Forward pass - returns DELTAS (can be negative)"""
        features = self.trunk(x)
        
        # Get raw linear outputs
        impact = self.impact_head(features)
        weight = self.weight_head(features)
        shear = self.shear_head(features)
        texture = self.texture_head(features)
        ring = self.ring_head(features)
        
        # Return deltas (all linear - no activation functions!)
        batch_size = x.shape[0]
        device = x.device
        
        return {
            'impact': {
                'A': impact[:, 0],           # Delta can be negative
                'rise_ms': impact[:, 1],     # Delta can be negative
                'fall_ms': impact[:, 2],     # Delta can be negative
                'hf_weight': impact[:, 3]    # Delta can be negative
            },
            'ring': {
                # f_Hz locked (zeros), tau_ms and a have deltas
                'f_Hz': torch.zeros(batch_size, 3, device=device),
                'tau_ms': ring[:, 0:3],
                'a': ring[:, 3:6]
            },
            'shear': {
                'A': shear[:, 0],
                'band_Hz': torch.zeros(batch_size, device=device) + 100.0  # Dummy
            },
            'weight': {
                'A': weight[:, 0],          # Delta (usually positive for Weber's law)
                'rate_ms': weight[:, 1]     # Delta can be negative
            },
            'texture': {
                'A': texture[:, 0],         # Delta can be negative
                'color': torch.ones(batch_size, 3, device=device) / 3  # Dummy uniform dist
            }
        }
    
    def count_parameters(self):
        """Count trainable parameters"""
        return sum(p.numel() for p in self.parameters() if p.requires_grad)