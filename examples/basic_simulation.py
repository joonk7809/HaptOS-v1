#!/usr/bin/env python3
"""
Basic Simulation Example

Demonstrates the core three-layer architecture:
1. Simulation (Physics)
2. Renderer (Neural Inference)
3. Driver (Hardware Output)
"""

import haptos

def main():
    print("=" * 60)
    print("  Basic HAPTOS Simulation")
    print("=" * 60)
    print()

    # Layer 1: Create physics simulation
    sim = haptos.Simulation(
        model_path="assets/hand_models/simple_hand.xml",
        max_contacts=6
    )
    print()

    # Layer 2: Create neural renderer
    renderer = haptos.Renderer()
    print()

    # Layer 3: Create mock hardware driver
    driver = haptos.Driver(driver_type="mock")
    print()

    # Register channel for index fingertip
    driver.register(body_part_id=10, port="MOCK")
    print()

    # Main simulation loop
    print("Running simulation (1000 steps = 1 second)...")
    print()

    for step in range(1000):
        # Step 1: Advance physics (1kHz)
        filtered_contacts = sim.step_filtered()

        # Step 2: Render cues (100Hz - every 10 physics steps)
        if step % 10 == 0 and filtered_contacts:
            cues = renderer.render(filtered_contacts)

            # Step 3: Send to hardware
            results = driver.send(cues)

            # Print status every 100ms
            if step % 100 == 0:
                print(f"  Step {step}: {len(filtered_contacts)} contacts, {len(cues)} cues, {sum(results.values())} sent")

    # Print final statistics
    print()
    print("Simulation complete!")
    print()

    sim_stats = sim.get_state()
    print(f"Simulation: {sim_stats['step_count']} steps")
    print(f"Total contacts: {sim_stats['router_stats']['total_contacts']}")
    print(f"Filtered: {sim_stats['router_stats']['filtered_contacts']}")
    print()

    render_stats = renderer.get_stats()
    print(f"Renderer: {render_stats.get('total_inferences', 0)} inferences")
    print(f"Avg time: {render_stats.get('avg_inference_time_ms', 0):.2f}ms")
    print()

    driver_stats = driver.get_stats()
    print(f"Driver: {driver_stats['total_sent']} packets sent")
    print(f"Success rate: {driver_stats['success_rate']:.1%}")
    print()

    # Cleanup
    driver.disconnect_all()
    print("✓ Done")


if __name__ == "__main__":
    main()
