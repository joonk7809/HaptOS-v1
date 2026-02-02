"""
haptos.Simulation - Physics Engine Wrapper (Layer 1)

Provides clean API for MuJoCo physics simulation with contact tracking.
"""

from typing import List, Optional
from pathlib import Path

from src.core.schemas import ContactPatch
from src.physics.multi_contact_engine import MultiContactEngine
from src.routing.somatotopic_router import SomatotopicRouter, Homunculus


class Simulation:
    """
    Physics simulation with automatic contact detection.

    Wraps MuJoCo engine and somatotopic router for easy contact tracking.

    Example:
        >>> sim = Simulation("assets/hand_models/simple_hand.xml")
        >>> contacts = sim.step()
        >>> print(f"Detected {len(contacts)} contacts")

    Args:
        model_path: Path to MuJoCo XML model file
        max_contacts: Maximum simultaneous contacts to track (default: 20)
        homunculus: Custom perceptual model (default: standard human)
        timestep: Physics timestep in seconds (default: 0.001 = 1ms)
    """

    def __init__(
        self,
        model_path: str,
        max_contacts: int = 20,
        homunculus: Optional[Homunculus] = None,
        timestep: float = 0.001
    ):
        """Initialize simulation."""
        # Validate model path
        model_file = Path(model_path)
        if not model_file.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")

        # Initialize physics engine
        self.engine = MultiContactEngine(
            model_path=str(model_file),
            max_contacts=max_contacts
        )

        # Initialize somatotopic router
        self.router = SomatotopicRouter(
            homunculus=homunculus or Homunculus(),
            max_contacts=max_contacts
        )

        self.timestep = timestep
        self.time = 0.0
        self.step_count = 0

        print(f"✓ Simulation initialized: {model_file.name}")
        print(f"  Max contacts: {max_contacts}")
        print(f"  Timestep: {timestep*1000:.1f}ms")

    def step(self) -> List[ContactPatch]:
        """
        Advance simulation by one timestep and return raw contacts.

        Returns:
            List of ContactPatch objects from physics engine

        Note:
            Use step_filtered() if you want perceptually-filtered contacts.
        """
        contacts = self.engine.step_v2()
        self.time += self.timestep
        self.step_count += 1
        return contacts

    def step_filtered(self):
        """
        Advance simulation and return perceptually-filtered contacts.

        Returns:
            List of FilteredContact objects (routed through Homunculus)

        This applies biological filtering:
        - Sensitivity thresholds (weak forces filtered out)
        - Rendering tier assignment (VCA/LRA/ERM)
        - Cue mask assignment (which cues to render)
        """
        contacts = self.step()
        filtered = self.router.route(contacts)
        return filtered

    def reset(self):
        """
        Reset simulation to initial state.

        Clears all contact history and resets time to 0.
        """
        self.engine.reset()
        self.router.reset_stats()
        self.time = 0.0
        self.step_count = 0

    def get_state(self) -> dict:
        """
        Get current simulation state.

        Returns:
            Dict with:
            - time: Current simulation time (seconds)
            - step_count: Number of steps taken
            - active_contacts: Number of current contacts
            - router_stats: Statistics from somatotopic router
        """
        router_stats = self.router.get_stats()

        return {
            'time': self.time,
            'step_count': self.step_count,
            'active_contacts': len(self.engine.get_contacts()),
            'router_stats': router_stats
        }

    def set_qpos(self, qpos):
        """
        Set joint positions directly.

        Args:
            qpos: Array of joint positions (size depends on model)

        Useful for:
        - Setting initial configuration
        - Teleporting objects
        - Testing specific poses
        """
        self.engine.data.qpos[:] = qpos

    def set_qvel(self, qvel):
        """
        Set joint velocities directly.

        Args:
            qvel: Array of joint velocities (size depends on model)
        """
        self.engine.data.qvel[:] = qvel

    def get_qpos(self):
        """Get current joint positions."""
        return self.engine.data.qpos.copy()

    def get_qvel(self):
        """Get current joint velocities."""
        return self.engine.data.qvel.copy()

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"Simulation(time={self.time:.3f}s, "
            f"steps={self.step_count}, "
            f"contacts={len(self.engine.get_contacts())})"
        )
