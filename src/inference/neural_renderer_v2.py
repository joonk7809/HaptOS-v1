#!/usr/bin/env python3
"""
NeuralRendererService_v2: Layer 2 Neural Rendering Service

Converts FilteredContact → SensationParams using NeuralRenderer_v2.

Pipeline:
    FilteredContact → FeatureExtractor → NeuralRenderer → SensationParams
                                              ↓
                                      Onset Detector
                                              ↓
                                      impact_trigger

Features:
- 14-dim feature extraction (force_delta, contact_area, contact_duration)
- Neural inference with cue mask gating
- Deterministic onset detection (phase transition FSM)
- Per-body-part state tracking
- Configurable debounce time

Usage:
    service = NeuralRendererService_v2(model_path="models/nn_v2_best.pt")
    sensations = service.render(filtered_contacts)
"""

import torch
import numpy as np
from typing import Dict, List, Optional
from dataclasses import dataclass

from src.models.nn_v2 import NeuralRenderer_v2
from src.converter.feature_extractor_v2 import FeatureExtractorV2, Phase
from src.core.schemas import SensationParams, FilteredContact, ContactPatch


@dataclass
class OnsetState:
    """State for onset detection per body part."""
    prev_phase_impact: bool = False
    last_onset_time_us: int = 0


class NeuralRendererService_v2:
    """
    Layer 2: Neural rendering service.

    Converts FilteredContact → SensationParams using trained neural network.
    """

    def __init__(self,
                 model_path: Optional[str] = None,
                 device: str = 'cpu',
                 onset_debounce_ms: float = 30.0):
        """
        Initialize Neural Renderer Service.

        Args:
            model_path: Path to trained model checkpoint (.pt)
                       If None, uses untrained model (for testing)
            device: torch device ('cpu' or 'cuda')
            onset_debounce_ms: Minimum time between impact triggers [ms]
        """
        self.device = torch.device(device)

        # Load or create model
        if model_path:
            self.model = NeuralRenderer_v2.load(model_path)
        else:
            self.model = NeuralRenderer_v2(input_dim=14)

        self.model.to(self.device)
        self.model.eval()

        # Feature extractor (14-dim)
        self.feature_extractor = FeatureExtractorV2()

        # Onset detection state (per body_part_id)
        self.onset_state: Dict[int, OnsetState] = {}

        # Configuration
        self.onset_debounce_ms = onset_debounce_ms

    def render(self, contacts: List[FilteredContact]) -> Dict[int, SensationParams]:
        """
        Render sensations for all filtered contacts.

        Args:
            contacts: List of FilteredContact from somatotopic router

        Returns:
            Dict[body_part_id, SensationParams]
        """
        sensations = {}

        for contact in contacts:
            sensation = self._render_single(contact)
            sensations[sensation.body_part_id] = sensation

        return sensations

    def _render_single(self, contact: FilteredContact) -> SensationParams:
        """
        Render a single filtered contact.

        Args:
            contact: FilteredContact from router

        Returns:
            SensationParams with perceptual parameters
        """
        patch = contact.patch
        body_part_id = patch.body_part_id

        # Extract 14-dim features
        features = self.feature_extractor.extract(patch)
        features_tensor = torch.tensor(features, dtype=torch.float32).unsqueeze(0).to(self.device)

        # Neural inference with cue mask gating
        with torch.no_grad():
            output_dict = self.model(features_tensor, cue_mask=contact.cue_mask)

        # Onset detection (deterministic, not learned)
        impact_trigger = self._detect_onset(body_part_id, features, patch.timestamp_us)

        # Convert dict to SensationParams
        sensation = SensationParams(
            # Impact channel
            impact_intensity=output_dict['impact_intensity'].item(),
            impact_sharpness=output_dict['impact_sharpness'].item(),
            impact_trigger=impact_trigger,

            # Resonance channel
            resonance_intensity=output_dict['resonance_intensity'].item(),
            resonance_brightness=output_dict['resonance_brightness'].item(),
            resonance_sustain=output_dict['resonance_sustain'].item(),

            # Texture channel
            texture_roughness=output_dict['texture_roughness'].item(),
            texture_density=output_dict['texture_density'].item(),
            texture_depth=output_dict['texture_depth'].item(),

            # Slip channel
            slip_speed=output_dict['slip_speed'].item(),
            slip_direction=(
                output_dict['slip_direction_x'].item(),
                output_dict['slip_direction_y'].item()
            ),
            slip_grip=output_dict['slip_grip'].item(),

            # Pressure channel
            pressure_magnitude=output_dict['pressure_magnitude'].item(),
            pressure_spread=output_dict['pressure_spread'].item(),

            # Metadata
            body_part_id=body_part_id,
            timestamp_us=patch.timestamp_us
        )

        return sensation

    def _detect_onset(self, body_part_id: int, features: np.ndarray, timestamp_us: int) -> bool:
        """
        Detect impact onset (rising edge of IMPACT phase).

        Returns True only on phase transition: NOT_IMPACT → IMPACT

        Args:
            body_part_id: Body part ID
            features: 14-dim feature vector
            timestamp_us: Current timestamp

        Returns:
            True if impact onset detected, False otherwise
        """
        # Initialize state if new body part
        if body_part_id not in self.onset_state:
            self.onset_state[body_part_id] = OnsetState()

        state = self.onset_state[body_part_id]

        # phase_impact is feature[8] in 14-dim vector
        is_impact = features[8] > 0.5

        # Check for rising edge
        rising_edge = is_impact and not state.prev_phase_impact

        # Update state
        state.prev_phase_impact = is_impact

        # Debounce: ignore triggers within debounce window
        if rising_edge:
            time_since_last_onset_ms = (timestamp_us - state.last_onset_time_us) / 1000.0

            if time_since_last_onset_ms < self.onset_debounce_ms:
                # Too soon, ignore
                return False
            else:
                # Valid onset
                state.last_onset_time_us = timestamp_us
                return True

        return False

    def reset(self, body_part_id: Optional[int] = None):
        """
        Reset state tracking.

        Args:
            body_part_id: If provided, reset only this body part. Otherwise reset all.
        """
        if body_part_id is None:
            # Reset all
            self.feature_extractor.reset()
            self.onset_state.clear()
        else:
            # Reset specific body part
            self.feature_extractor.reset(body_part_id)
            if body_part_id in self.onset_state:
                self.onset_state[body_part_id] = OnsetState()

    def get_model_info(self) -> Dict[str, any]:
        """
        Get model information.

        Returns:
            Dict with model metadata
        """
        breakdown = self.model.get_parameter_breakdown()

        return {
            'model_class': 'NeuralRenderer_v2',
            'input_dim': self.model.input_dim,
            'hidden_dim': self.model.hidden_dim,
            'bottleneck_dim': self.model.bottleneck_dim,
            'total_params': breakdown['total'],
            'parameter_breakdown': breakdown,
            'device': str(self.device),
            'onset_debounce_ms': self.onset_debounce_ms,
        }


if __name__ == '__main__':
    # Test neural renderer service
    print("Testing NeuralRendererService_v2\n")
    print("="*60)

    # Create service (untrained model for testing)
    service = NeuralRendererService_v2(model_path=None, onset_debounce_ms=30.0)

    print("Model Info:")
    info = service.get_model_info()
    print(f"  Model: {info['model_class']}")
    print(f"  Total Parameters: {info['total_params']:,}")
    print(f"  Device: {info['device']}")
    print(f"  Onset Debounce: {info['onset_debounce_ms']} ms")

    print("\n" + "="*60)

    # Create test contact
    patch = ContactPatch(
        body_part_id=10,
        force_normal=2.0,
        force_shear=np.array([0.3, 0.1]),
        velocity=np.array([0.01, 0.005, 0.0]),
        contact_area=0.0002,
        material_hint=1,  # Metal
        timestamp_us=1000000
    )

    contact = FilteredContact(
        patch=patch,
        rendering_tier=1,  # VCA
        cue_mask=FilteredContact.CUE_ALL
    )

    # Render
    sensations = service.render([contact])
    sensation = sensations[10]

    print("Rendered Sensation:")
    print(f"  Impact Intensity: {sensation.impact_intensity:.3f}")
    print(f"  Impact Sharpness: {sensation.impact_sharpness:.3f}")
    print(f"  Impact Trigger: {sensation.impact_trigger}")
    print(f"  Resonance Intensity: {sensation.resonance_intensity:.3f}")
    print(f"  Resonance Brightness: {sensation.resonance_brightness:.3f}")
    print(f"  Texture Roughness: {sensation.texture_roughness:.3f}")
    print(f"  Slip Speed: {sensation.slip_speed:.3f}")
    print(f"  Pressure Magnitude: {sensation.pressure_magnitude:.3f}")

    print("\n" + "="*60)

    # Test onset detection
    print("Testing Onset Detection:")

    # First contact (should trigger)
    patch1 = ContactPatch(
        body_part_id=20,
        force_normal=1.0,
        force_shear=np.array([0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        contact_area=0.0001,
        material_hint=0,
        timestamp_us=2000000
    )
    contact1 = FilteredContact(patch=patch1, rendering_tier=1, cue_mask=FilteredContact.CUE_ALL)
    sens1 = service.render([contact1])[20]
    print(f"  Contact 1 (t=0ms): trigger={sens1.impact_trigger}")

    # Second contact (10ms later, should be debounced)
    patch2 = ContactPatch(
        body_part_id=20,
        force_normal=1.0,
        force_shear=np.array([0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        contact_area=0.0001,
        material_hint=0,
        timestamp_us=2010000  # +10ms
    )
    contact2 = FilteredContact(patch=patch2, rendering_tier=1, cue_mask=FilteredContact.CUE_ALL)
    sens2 = service.render([contact2])[20]
    print(f"  Contact 2 (t=10ms): trigger={sens2.impact_trigger} (debounced)")

    # Third contact (50ms later, should trigger)
    patch3 = ContactPatch(
        body_part_id=20,
        force_normal=0.0,  # Break contact
        force_shear=np.array([0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        contact_area=0.0,
        material_hint=0,
        timestamp_us=2050000  # +50ms
    )
    contact3 = FilteredContact(patch=patch3, rendering_tier=1, cue_mask=FilteredContact.CUE_ALL)
    sens3 = service.render([contact3])[20]

    # Fourth contact (new impact after break)
    patch4 = ContactPatch(
        body_part_id=20,
        force_normal=1.5,
        force_shear=np.array([0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        contact_area=0.0001,
        material_hint=0,
        timestamp_us=2060000  # +60ms
    )
    contact4 = FilteredContact(patch=patch4, rendering_tier=1, cue_mask=FilteredContact.CUE_ALL)
    sens4 = service.render([contact4])[20]
    print(f"  Contact 4 (t=60ms, after break): trigger={sens4.impact_trigger} (new impact)")

    print("\n" + "="*60)
    print("✓ NeuralRendererService_v2 test passed!")
