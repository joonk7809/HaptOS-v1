"""
haptos.quickstart - Convenience functions for getting started quickly
"""

from typing import Optional
from pathlib import Path

from .simulation import Simulation
from .renderer import Renderer
from .driver import Driver
from .homunculus import Homunculus


def demo(duration: float = 5.0, render_output: bool = True):
    """
    Run a quick demo of the HAPTOS platform.

    Creates a simple simulation with mock hardware and runs for the specified duration.

    Args:
        duration: Duration to run demo in seconds (default: 5.0)
        render_output: Whether to print output to console (default: True)

    Example:
        >>> import haptos
        >>> haptos.demo()  # Run 5-second demo

    This creates:
    - Simple hand model simulation @ 1kHz
    - Neural renderer with default models
    - Mock hardware driver (no physical device needed)
    """
    if render_output:
        print("=" * 60)
        print("  HAPTOS Platform Demo")
        print("=" * 60)
        print()

    # Find model file
    model_paths = [
        "assets/hand_models/simple_hand.xml",
        "assets/hand_models/detailed_hand.xml"
    ]

    model_path = None
    for path in model_paths:
        if Path(path).exists():
            model_path = path
            break

    if model_path is None:
        print("❌ Error: No hand model found in assets/hand_models/")
        print("   Please ensure MuJoCo models are available")
        return

    # Create simulation
    if render_output:
        print("Setting up simulation...")
    sim = Simulation(model_path, max_contacts=6)

    # Create renderer
    if render_output:
        print()
    renderer = Renderer()

    # Create mock driver (6 channels for full hand)
    if render_output:
        print()
    driver = Driver(driver_type="mock")

    # Register channels for fingertips + palm
    body_parts = {
        10: "index",
        11: "thumb",
        12: "middle",
        13: "ring",
        14: "pinky",
        15: "palm"
    }

    for body_part_id, name in body_parts.items():
        driver.register(body_part_id, port=f"MOCK_{name.upper()}")

    # Run simulation
    if render_output:
        print()
        print(f"Running demo for {duration:.1f} seconds...")
        print()

    steps = int(duration / 0.001)  # 1ms timestep
    contact_count = 0
    rendered_count = 0
    sent_count = 0

    for step in range(steps):
        # Step physics
        filtered_contacts = sim.step_filtered()

        # Render cues (every 10 steps = 100Hz)
        if step % 10 == 0 and filtered_contacts:
            cues = renderer.render(filtered_contacts)

            # Send to hardware
            results = driver.send(cues)

            contact_count += len(filtered_contacts)
            rendered_count += len(cues)
            sent_count += sum(results.values())

        # Print progress
        if render_output and step % 1000 == 0 and step > 0:
            elapsed = step * 0.001
            print(f"  {elapsed:.1f}s - Contacts: {contact_count}, Rendered: {rendered_count}, Sent: {sent_count}")

    # Print final statistics
    if render_output:
        print()
        print("=" * 60)
        print("  Demo Complete!")
        print("=" * 60)
        print()

        sim_stats = sim.get_state()
        render_stats = renderer.get_stats()
        driver_stats = driver.get_stats()

        print(f"Simulation:")
        print(f"  Steps: {sim_stats['step_count']}")
        print(f"  Total contacts: {sim_stats['router_stats']['total_contacts']}")
        print(f"  Filtered contacts: {sim_stats['router_stats']['filtered_contacts']}")
        print()

        print(f"Renderer:")
        print(f"  Inferences: {render_stats.get('total_inferences', 0)}")
        print(f"  Avg inference time: {render_stats.get('avg_inference_time_ms', 0):.2f}ms")
        print()

        print(f"Driver:")
        print(f"  Channels: {driver_stats['driver_count']}")
        print(f"  Packets sent: {driver_stats['total_sent']}")
        print(f"  Success rate: {driver_stats['success_rate']:.1%}")
        print(f"  Bandwidth: {driver_stats.get('bandwidth_hz', 0):.1f} packets/s")
        print()

    # Cleanup
    driver.disconnect_all()

    if render_output:
        print("✓ Demo finished successfully")
        print()


def calibrate_user(
    interactive: bool = True,
    save_path: Optional[str] = None
) -> Homunculus:
    """
    Calibrate a custom Homunculus for a user.

    Runs interactive tests to determine user's perceptual thresholds
    and creates a personalized Homunculus configuration.

    Args:
        interactive: Run interactive calibration (default: True)
                    If False, returns default Homunculus
        save_path: Path to save calibrated config (default: None)

    Returns:
        Calibrated Homunculus instance

    Example:
        >>> import haptos
        >>> homunculus = haptos.calibrate_user(save_path="my_profile.json")
        >>> sim = haptos.Simulation("model.xml", homunculus=homunculus)

    Note:
        Phase 3 implementation returns default Homunculus.
        Full calibration wizard coming in Phase 4.
    """
    print("=" * 60)
    print("  HAPTOS Calibration Wizard")
    print("=" * 60)
    print()

    if not interactive:
        print("Using default Homunculus (standard human perceptual model)")
        print()
        return Homunculus()

    # Phase 3: Return default (full calibration in Phase 4)
    print("Interactive calibration not yet implemented.")
    print("Returning default Homunculus configuration.")
    print()
    print("In Phase 4, this will run:")
    print("  1. Force detection threshold tests")
    print("  2. Spatial acuity measurements")
    print("  3. Frequency discrimination tests")
    print("  4. Create personalized perceptual model")
    print()

    homunculus = Homunculus()

    if save_path:
        homunculus.save(save_path)
        print(f"✓ Saved default Homunculus to: {save_path}")
        print()

    return homunculus


def load_environment(name: str) -> str:
    """
    Load a standard environment from the library.

    Args:
        name: Environment name:
            - "flat_surface": Simple flat ground
            - "textured_floor": Wood/metal/rubber textures
            - "table_objects": Table with graspable objects
            - "forest_medium": Outdoor scene (trees, rocks, grass)

    Returns:
        Path to MuJoCo XML file

    Example:
        >>> model_path = haptos.load_environment("table_objects")
        >>> sim = haptos.Simulation(model_path)

    Note:
        Environment library coming in Phase 3.
        Currently returns path hints only.
    """
    environments = {
        "flat_surface": "environments/flat_surface.xml",
        "textured_floor": "environments/textured_floor.xml",
        "table_objects": "environments/table_with_objects.xml",
        "forest_medium": "environments/forest_medium.xml"
    }

    if name not in environments:
        available = ", ".join(environments.keys())
        raise ValueError(f"Unknown environment: {name}. Available: {available}")

    env_path = environments[name]

    # Check if environment exists
    if not Path(env_path).exists():
        print(f"⚠️  Environment '{name}' not yet available")
        print(f"   Expected path: {env_path}")
        print(f"   Using fallback...")

        # Try to find any available model
        fallback_paths = [
            "assets/hand_models/simple_hand.xml",
            "assets/hand_models/detailed_hand.xml"
        ]

        for fallback in fallback_paths:
            if Path(fallback).exists():
                print(f"   → Using: {fallback}")
                return fallback

        raise FileNotFoundError(
            f"Environment '{name}' not found and no fallback available"
        )

    return env_path
