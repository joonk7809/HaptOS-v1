"""
haptos.Renderer - Neural Haptic Renderer (Layer 2)

Converts filtered contacts to haptic cue parameters via ML inference.
"""

from typing import List, Dict, Optional
from pathlib import Path

from src.core.schemas import FilteredContact, CueParams
from src.inference.neural_renderer import NeuralRenderer as _NeuralRenderer


class Renderer:
    """
    Neural renderer for haptic cue generation.

    Uses trained ML models to convert contact physics to perceptual cues.

    Example:
        >>> renderer = Renderer()
        >>> cues = renderer.render(filtered_contacts)
        >>> print(f"Generated {len(cues)} cue parameter sets")

    Args:
        model_v0: Path to baseline model checkpoint (default: auto-detect)
        model_v1: Path to delta model checkpoint (default: auto-detect)
        device: Device for inference ('cpu' or 'cuda', default: 'cpu')
        batch_size: Batch size for inference (default: 1)
    """

    def __init__(
        self,
        model_v0: Optional[str] = None,
        model_v1: Optional[str] = None,
        device: str = 'cpu',
        batch_size: int = 1
    ):
        """Initialize renderer with trained models."""
        # Auto-detect model paths if not provided
        if model_v0 is None:
            model_v0 = "models/checkpoints/nn_v0_best.pt"
        if model_v1 is None:
            model_v1 = "models/checkpoints/nn_v1_best.pt"

        # Validate model files
        v0_path = Path(model_v0)
        v1_path = Path(model_v1)

        if not v0_path.exists():
            raise FileNotFoundError(f"Model v0 not found: {model_v0}")
        if not v1_path.exists():
            raise FileNotFoundError(f"Model v1 not found: {model_v1}")

        # Initialize neural renderer
        self._renderer = _NeuralRenderer(
            nn_v0_path=str(v0_path),
            nn_v1_path=str(v1_path),
            device=device
        )

        self.device = device
        self.batch_size = batch_size

        print(f"✓ Renderer initialized")
        print(f"  Model v0: {v0_path.name}")
        print(f"  Model v1: {v1_path.name}")
        print(f"  Device: {device}")

    def render(
        self,
        filtered_contacts: List[FilteredContact],
        timestamp_us: Optional[int] = None
    ) -> Dict[int, CueParams]:
        """
        Render haptic cue parameters from filtered contacts.

        Args:
            filtered_contacts: List of FilteredContact from router
            timestamp_us: Current time in microseconds (optional)

        Returns:
            Dict mapping body_part_id → CueParams

        The returned CueParams contain:
        - Continuous cues: texture, shear, weight
        - Transient cues: impact, ring
        - trigger_impulse: Flag for impact/release events
        """
        if timestamp_us is None:
            # Use microsecond timestamp from first contact
            if filtered_contacts:
                timestamp_us = filtered_contacts[0].patch.timestamp_us
            else:
                timestamp_us = 0

        return self._renderer.render(filtered_contacts, timestamp_us)

    def reset(self):
        """
        Reset renderer state.

        Clears all contact buffers and FSM state.
        Useful when starting a new simulation run.
        """
        self._renderer.reset()

    def get_stats(self) -> dict:
        """
        Get rendering statistics.

        Returns:
            Dict with:
            - total_inferences: Number of inference calls
            - avg_inference_time_ms: Average inference time
            - contacts_rendered: Total contacts processed
        """
        return self._renderer.get_stats()

    def load_model(self, model_path: str, version: str = "v0"):
        """
        Load a different model checkpoint.

        Args:
            model_path: Path to .pt checkpoint file
            version: Model version ("v0" for baseline, "v1" for delta)

        Useful for:
        - A/B testing different models
        - Loading custom trained models
        - Hot-swapping models at runtime
        """
        model_file = Path(model_path)
        if not model_file.exists():
            raise FileNotFoundError(f"Model not found: {model_path}")

        if version == "v0":
            self._renderer.predictor.predictor_v0.load_checkpoint(str(model_file))
        elif version == "v1":
            self._renderer.predictor.predictor_v1.load_checkpoint(str(model_file))
        else:
            raise ValueError(f"Unknown model version: {version}")

        print(f"✓ Loaded {version} model: {model_file.name}")

    def __repr__(self) -> str:
        """String representation."""
        stats = self.get_stats()
        return (
            f"Renderer(device={self.device}, "
            f"inferences={stats.get('total_inferences', 0)})"
        )
