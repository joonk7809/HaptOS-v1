"""
haptos.Homunculus - Perceptual Body Model

Represents biological sensor distribution and sensitivity.
"""

from typing import Optional
from pathlib import Path

from src.routing.somatotopic_router import Homunculus as _Homunculus


class Homunculus:
    """
    Perceptual body model representing biological touch sensitivity.

    The Homunculus maps body parts to perceptual properties:
    - Spatial resolution (receptive field size)
    - Frequency response range
    - Sensitivity (force detection threshold)
    - Rendering tier (VCA/LRA/ERM capability)
    - Cue mask (which cues are perceptible)

    Example:
        >>> homunculus = Homunculus()
        >>> props = homunculus.lookup(10)  # Index fingertip
        >>> print(f"Spatial resolution: {props.spatial_res_mm}mm")

    Args:
        config_path: Path to custom Homunculus config (optional)
    """

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize Homunculus.

        Args:
            config_path: Path to JSON config file (optional)
                        If None, uses default human perceptual model
        """
        if config_path is None:
            # Use default Homunculus
            self._homunculus = _Homunculus()
        else:
            # Load custom configuration
            config_file = Path(config_path)
            if not config_file.exists():
                raise FileNotFoundError(f"Homunculus config not found: {config_path}")

            self._homunculus = _Homunculus.load(str(config_file))

        print(f"✓ Homunculus initialized")
        print(f"  Body parts mapped: {len(self._homunculus.table)}")

    def lookup(self, body_part_id: int):
        """
        Get perceptual properties for a body part.

        Args:
            body_part_id: Body part identifier (from MuJoCo model)

        Returns:
            BodyPartProperties with:
            - spatial_res_mm: Spatial resolution (mm)
            - freq_range_hz: Frequency response range (Hz)
            - sensitivity: Relative sensitivity (0-2, nominal=1.0)
            - rendering_tier: Hardware tier (1=VCA, 2=LRA, 3=ERM)
            - cue_mask: Bitmask of perceptible cues
        """
        return self._homunculus.lookup(body_part_id)

    def save(self, filepath: str):
        """
        Save Homunculus configuration to JSON file.

        Args:
            filepath: Path to save config file

        Useful for:
        - Saving calibrated user profiles
        - Exporting custom perceptual models
        - Version control of Homunculus configs
        """
        self._homunculus.save(filepath)
        print(f"✓ Saved Homunculus to {filepath}")

    @staticmethod
    def load(filepath: str) -> 'Homunculus':
        """
        Load Homunculus configuration from JSON file.

        Args:
            filepath: Path to JSON config file

        Returns:
            Homunculus instance with loaded configuration

        Example:
            >>> custom = Homunculus.load("configs/user_profile.json")
            >>> sim = Simulation("model.xml", homunculus=custom)
        """
        config_file = Path(filepath)
        if not config_file.exists():
            raise FileNotFoundError(f"Config file not found: {filepath}")

        return Homunculus(config_path=str(config_file))

    def get_table(self) -> dict:
        """
        Get complete Homunculus table.

        Returns:
            Dict mapping body part name → BodyPartProperties

        Useful for:
        - Inspecting all mapped body parts
        - Creating custom configurations
        - Debugging perceptual filtering
        """
        return self._homunculus.table.copy()

    def __repr__(self) -> str:
        """String representation."""
        return f"Homunculus(body_parts={len(self._homunculus.table)})"
