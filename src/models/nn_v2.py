#!/usr/bin/env python3
"""
NeuralRenderer_v2: 14-dim Feature → SensationParams

Maps physics features to perceptual sensation parameters.
Body-agnostic: outputs assume maximum sensitivity (fingertip reference).
Somatotopic shaping is applied downstream by the driver.

Architecture:
- Input: 14-dim feature vector
- Trunk: 14 → 128 → 128 → 64 (shared features)
- Heads: 5 perceptual channels (14 outputs total)
- Total params: 27,598

Perceptual Heads:
1. Impact (2): intensity, sharpness
2. Resonance (3): intensity, brightness, sustain
3. Texture (3): roughness, density, depth
4. Slip (4): speed, direction_x, direction_y, grip
5. Pressure (2): magnitude, spread

Cue Mask Gating:
- Disabled channels output zero (no computation)
- Reduces inference cost for limited hardware
"""

import torch
import torch.nn as nn
from typing import Dict, Tuple
import numpy as np


class NeuralRenderer_v2(nn.Module):
    """
    Neural renderer for perceptually-grounded haptic sensations.

    Maps 14-dim physics features to SensationParams (14 perceptual parameters).
    """

    # Cue mask constants (from FilteredContact schema)
    CUE_IMPACT = 0b00001
    CUE_RESONANCE = 0b00010
    CUE_TEXTURE = 0b00100
    CUE_SLIP = 0b01000
    CUE_PRESSURE = 0b10000
    CUE_ALL = 0b11111

    def __init__(self, input_dim: int = 14, hidden_dim: int = 128, bottleneck_dim: int = 64):
        """
        Initialize NeuralRenderer_v2.

        Args:
            input_dim: Input feature dimension (default: 14)
            hidden_dim: Hidden layer dimension (default: 128)
            bottleneck_dim: Bottleneck dimension before heads (default: 64)
        """
        super().__init__()

        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.bottleneck_dim = bottleneck_dim

        # Shared trunk (3 layers)
        self.trunk = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, bottleneck_dim),
            nn.ReLU()
        )

        # Perceptual heads (gated by cue_mask)
        self.impact_head = nn.Linear(bottleneck_dim, 2)       # intensity, sharpness
        self.resonance_head = nn.Linear(bottleneck_dim, 3)    # intensity, brightness, sustain
        self.texture_head = nn.Linear(bottleneck_dim, 3)      # roughness, density, depth
        self.slip_head = nn.Linear(bottleneck_dim, 4)         # speed, dir_x, dir_y, grip
        self.pressure_head = nn.Linear(bottleneck_dim, 2)     # magnitude, spread

    def forward(self, features: torch.Tensor, cue_mask: int = CUE_ALL) -> Dict[str, torch.Tensor]:
        """
        Forward pass: 14-dim features → SensationParams dict.

        Args:
            features: (batch, 14) tensor of physics features
            cue_mask: Bitmask of enabled channels (default: all enabled)

        Returns:
            Dict with 14 sensation parameters (all in [0,1]):
            {
                'impact_intensity': (batch,),
                'impact_sharpness': (batch,),
                'resonance_intensity': (batch,),
                'resonance_brightness': (batch,),
                'resonance_sustain': (batch,),
                'texture_roughness': (batch,),
                'texture_density': (batch,),
                'texture_depth': (batch,),
                'slip_speed': (batch,),
                'slip_direction_x': (batch,),
                'slip_direction_y': (batch,),
                'slip_grip': (batch,),
                'pressure_magnitude': (batch,),
                'pressure_spread': (batch,)
            }
        """
        batch_size = features.shape[0]

        # Shared trunk
        trunk_out = self.trunk(features)

        # Initialize output dict
        output = {}

        # Impact channel
        if cue_mask & self.CUE_IMPACT:
            imp = torch.sigmoid(self.impact_head(trunk_out))
            output['impact_intensity'] = imp[:, 0]
            output['impact_sharpness'] = imp[:, 1]
        else:
            output['impact_intensity'] = torch.zeros(batch_size, device=features.device)
            output['impact_sharpness'] = torch.zeros(batch_size, device=features.device)

        # Resonance channel
        if cue_mask & self.CUE_RESONANCE:
            res = torch.sigmoid(self.resonance_head(trunk_out))
            output['resonance_intensity'] = res[:, 0]
            output['resonance_brightness'] = res[:, 1]
            output['resonance_sustain'] = res[:, 2]
        else:
            output['resonance_intensity'] = torch.zeros(batch_size, device=features.device)
            output['resonance_brightness'] = torch.zeros(batch_size, device=features.device)
            output['resonance_sustain'] = torch.zeros(batch_size, device=features.device)

        # Texture channel
        if cue_mask & self.CUE_TEXTURE:
            tex = torch.sigmoid(self.texture_head(trunk_out))
            output['texture_roughness'] = tex[:, 0]
            output['texture_density'] = tex[:, 1]
            output['texture_depth'] = tex[:, 2]
        else:
            output['texture_roughness'] = torch.zeros(batch_size, device=features.device)
            output['texture_density'] = torch.zeros(batch_size, device=features.device)
            output['texture_depth'] = torch.zeros(batch_size, device=features.device)

        # Slip channel
        if cue_mask & self.CUE_SLIP:
            slip = self.slip_head(trunk_out)

            # speed and grip: [0, 1]
            output['slip_speed'] = torch.sigmoid(slip[:, 0])
            output['slip_grip'] = torch.sigmoid(slip[:, 3])

            # direction: normalized to unit vector (tanh for [-1, 1])
            dir_x = torch.tanh(slip[:, 1])
            dir_y = torch.tanh(slip[:, 2])

            # Normalize to unit vector
            norm = torch.sqrt(dir_x**2 + dir_y**2 + 1e-8)
            output['slip_direction_x'] = dir_x / norm
            output['slip_direction_y'] = dir_y / norm
        else:
            output['slip_speed'] = torch.zeros(batch_size, device=features.device)
            output['slip_direction_x'] = torch.zeros(batch_size, device=features.device)
            output['slip_direction_y'] = torch.zeros(batch_size, device=features.device)
            output['slip_grip'] = torch.zeros(batch_size, device=features.device)

        # Pressure channel
        if cue_mask & self.CUE_PRESSURE:
            pres = torch.sigmoid(self.pressure_head(trunk_out))
            output['pressure_magnitude'] = pres[:, 0]
            output['pressure_spread'] = pres[:, 1]
        else:
            output['pressure_magnitude'] = torch.zeros(batch_size, device=features.device)
            output['pressure_spread'] = torch.zeros(batch_size, device=features.device)

        return output

    def count_parameters(self) -> int:
        """
        Count trainable parameters.

        Returns:
            Total number of trainable parameters
        """
        return sum(p.numel() for p in self.parameters() if p.requires_grad)

    def get_parameter_breakdown(self) -> Dict[str, int]:
        """
        Get parameter count breakdown by component.

        Returns:
            Dict with parameter counts for each layer
        """
        breakdown = {
            'trunk': sum(p.numel() for p in self.trunk.parameters()),
            'impact_head': sum(p.numel() for p in self.impact_head.parameters()),
            'resonance_head': sum(p.numel() for p in self.resonance_head.parameters()),
            'texture_head': sum(p.numel() for p in self.texture_head.parameters()),
            'slip_head': sum(p.numel() for p in self.slip_head.parameters()),
            'pressure_head': sum(p.numel() for p in self.pressure_head.parameters()),
        }
        breakdown['total'] = sum(breakdown.values())
        return breakdown

    def predict_sensation(self, features: np.ndarray, cue_mask: int = CUE_ALL) -> Dict[str, float]:
        """
        Inference helper: single feature vector → SensationParams dict.

        Args:
            features: (14,) numpy array
            cue_mask: Bitmask of enabled channels

        Returns:
            Dict with 14 sensation parameters (floats in [0,1])
        """
        self.eval()
        with torch.no_grad():
            features_tensor = torch.tensor(features, dtype=torch.float32).unsqueeze(0)
            output_tensors = self.forward(features_tensor, cue_mask)

            # Convert to float dict
            output = {k: v.item() for k, v in output_tensors.items()}

        return output

    def save(self, path: str):
        """
        Save model state dict.

        Args:
            path: Path to save file (.pt)
        """
        torch.save({
            'model_state_dict': self.state_dict(),
            'input_dim': self.input_dim,
            'hidden_dim': self.hidden_dim,
            'bottleneck_dim': self.bottleneck_dim,
        }, path)

    @classmethod
    def load(cls, path: str) -> 'NeuralRenderer_v2':
        """
        Load model from checkpoint.

        Args:
            path: Path to checkpoint file (.pt)

        Returns:
            Loaded NeuralRenderer_v2 instance
        """
        checkpoint = torch.load(path, map_location='cpu')

        model = cls(
            input_dim=checkpoint['input_dim'],
            hidden_dim=checkpoint['hidden_dim'],
            bottleneck_dim=checkpoint['bottleneck_dim']
        )

        model.load_state_dict(checkpoint['model_state_dict'])
        model.eval()

        return model


def create_model(input_dim: int = 14) -> NeuralRenderer_v2:
    """
    Factory function to create NeuralRenderer_v2.

    Args:
        input_dim: Input feature dimension (default: 14)

    Returns:
        Initialized NeuralRenderer_v2 model
    """
    model = NeuralRenderer_v2(input_dim=input_dim)

    # Print parameter summary
    breakdown = model.get_parameter_breakdown()
    print("NeuralRenderer_v2 Parameter Breakdown:")
    print(f"  Trunk:          {breakdown['trunk']:,} params")
    print(f"  Impact Head:    {breakdown['impact_head']:,} params")
    print(f"  Resonance Head: {breakdown['resonance_head']:,} params")
    print(f"  Texture Head:   {breakdown['texture_head']:,} params")
    print(f"  Slip Head:      {breakdown['slip_head']:,} params")
    print(f"  Pressure Head:  {breakdown['pressure_head']:,} params")
    print(f"  Total:          {breakdown['total']:,} params")

    return model


if __name__ == '__main__':
    # Test model creation
    print("Creating NeuralRenderer_v2...\n")
    model = create_model()

    print("\n" + "="*50)

    # Test forward pass
    print("Testing forward pass...")
    batch_size = 4
    features = torch.randn(batch_size, 14)

    output = model(features, cue_mask=NeuralRenderer_v2.CUE_ALL)

    print(f"\nInput shape: {features.shape}")
    print(f"Output keys: {list(output.keys())}")
    print(f"Output shape (impact_intensity): {output['impact_intensity'].shape}")

    # Check value ranges
    print("\nValue ranges:")
    for key, value in output.items():
        print(f"  {key:25s}: [{value.min():.4f}, {value.max():.4f}]")

    print("\n" + "="*50)
    print("✓ NeuralRenderer_v2 test passed!")
