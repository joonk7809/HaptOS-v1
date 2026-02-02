"""
HAPTOS SDK - Public API

Simulation-first haptics platform for developers.

Quick Start:
    >>> import haptos
    >>>
    >>> # Create simulation
    >>> sim = haptos.Simulation("assets/hand_models/simple_hand.xml")
    >>>
    >>> # Setup renderer and driver
    >>> renderer = haptos.Renderer()
    >>> driver = haptos.Driver(driver_type="mock")
    >>>
    >>> # Run simulation loop
    >>> for _ in range(100):
    >>>     contacts = sim.step()
    >>>     cues = renderer.render(contacts)
    >>>     driver.send(cues)

Architecture:
    Layer 1 (Physics): Simulation @ 1kHz → ContactPatch
    Layer 2 (Rendering): Renderer @ 100Hz → CueParams
    Layer 3 (Hardware): Driver @ 2kHz+ → Actuator

Documentation: https://github.com/anthropics/haptos
"""

__version__ = "0.3.0"  # Phase 2 complete, Phase 3 in progress
__author__ = "HAPTOS Team"
__license__ = "MIT"

# Public API imports
from .simulation import Simulation
from .renderer import Renderer
from .driver import Driver
from .homunculus import Homunculus

# Core data structures (for advanced users)
from src.core.schemas import ContactPatch, FilteredContact, CueParams

# Convenience functions
from .quickstart import demo, calibrate_user

__all__ = [
    # Main API classes
    "Simulation",
    "Renderer",
    "Driver",
    "Homunculus",

    # Core schemas
    "ContactPatch",
    "FilteredContact",
    "CueParams",

    # Convenience
    "demo",
    "calibrate_user",

    # Metadata
    "__version__",
]
