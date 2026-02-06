"""
Rendering module for HAPTOS.

This module handles somatotopic shaping - adapting body-agnostic sensations
to specific body parts based on biological sensitivity and spatial resolution.
"""

from .somatotopic_shaper import shape, get_shaping_factors

__all__ = ['shape', 'get_shaping_factors']
