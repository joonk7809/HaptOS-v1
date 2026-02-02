#!/usr/bin/env python3
"""
Custom Homunculus Example

Demonstrates how to create and use a custom perceptual model.
Useful for:
- User-specific calibration
- Non-human bodies (robotic manipulators)
- Accessibility features (enhanced sensitivity)
"""

import haptos

def main():
    print("=" * 60)
    print("  Custom Homunculus Demo")
    print("=" * 60)
    print()

    # Create default Homunculus
    print("Creating default Homunculus...")
    default_homunculus = haptos.Homunculus()
    print()

    # Inspect properties
    print("Default perceptual properties:")
    table = default_homunculus.get_table()

    for name in ['index_tip', 'palm_center']:
        if name in table:
            props = table[name]
            print(f"\n{name}:")
            print(f"  Spatial resolution: {props.spatial_res_mm}mm")
            print(f"  Sensitivity: {props.sensitivity:.2f}")
            print(f"  Rendering tier: {props.rendering_tier}")
            print(f"  Freq range: {props.freq_range_hz[0]}-{props.freq_range_hz[1]} Hz")
    print()

    # Save custom configuration
    config_path = "custom_homunculus.json"
    print(f"Saving configuration to {config_path}...")
    default_homunculus.save(config_path)
    print()

    # Load it back
    print("Loading custom configuration...")
    custom_homunculus = haptos.Homunculus.load(config_path)
    print()

    # Use in simulation
    print("Creating simulation with custom Homunculus...")
    sim = haptos.Simulation(
        model_path="assets/hand_models/simple_hand.xml",
        homunculus=custom_homunculus
    )
    print()

    # Run brief test
    print("Running test (500 steps)...")
    for _ in range(500):
        contacts = sim.step_filtered()

    stats = sim.get_state()
    print()
    print(f"Result:")
    print(f"  Total contacts: {stats['router_stats']['total_contacts']}")
    print(f"  Filtered contacts: {stats['router_stats']['filtered_contacts']}")
    print(f"  Acceptance rate: {stats['router_stats'].get('acceptance_rate', 0):.1%}")
    print()

    print("✓ Demo complete")
    print()
    print("Note: In Phase 4, you'll be able to:")
    print("  - Run interactive calibration wizard")
    print("  - Adjust sensitivity per body part")
    print("  - Create accessibility profiles")
    print("  - Sync profiles across devices")


if __name__ == "__main__":
    main()
