#!/usr/bin/env python3
"""
Grasp Scenario Demo

Demonstrates multi-contact hand grasping with all 5 fingers + palm.
This is Canonical Test 5 from Phase 2.
"""

import haptos

def main():
    print("=" * 60)
    print("  Multi-Contact Grasp Demo")
    print("=" * 60)
    print()

    # Create simulation
    sim = haptos.Simulation(
        model_path="assets/hand_models/simple_hand.xml",
        max_contacts=20  # Allow many simultaneous contacts
    )
    print()

    # Create renderer
    renderer = haptos.Renderer()
    print()

    # Create driver with synchronization enabled
    driver = haptos.Driver(
        driver_type="mock",
        enable_sync=True  # Synchronize multi-channel output
    )
    print()

    # Register all 6 channels (5 fingertips + palm)
    channels = {
        10: "index",
        11: "thumb",
        12: "middle",
        13: "ring",
        14: "pinky",
        15: "palm"
    }

    print("Registering channels:")
    for body_part_id, name in channels.items():
        driver.register(body_part_id, port=f"MOCK_{name.upper()}")
        print(f"  ✓ {name.capitalize()} (ID {body_part_id})")
    print()

    # Run simulation
    duration = 2.0  # 2 seconds
    steps = int(duration / 0.001)

    print(f"Simulating {duration}s grasp...")
    print()

    max_simultaneous = 0
    total_renders = 0

    for step in range(steps):
        # Advance physics
        filtered_contacts = sim.step_filtered()

        # Track max simultaneous contacts
        if len(filtered_contacts) > max_simultaneous:
            max_simultaneous = len(filtered_contacts)

        # Render and send (100Hz)
        if step % 10 == 0 and filtered_contacts:
            cues = renderer.render(filtered_contacts)
            results = driver.send(cues)
            total_renders += 1

            # Report status every 200ms
            if step % 200 == 0 and step > 0:
                elapsed = step * 0.001
                active_channels = len(cues)
                print(f"  {elapsed:.1f}s - Active channels: {active_channels}, Max simultaneous: {max_simultaneous}")

    # Final statistics
    print()
    print("=" * 60)
    print("  Grasp Complete!")
    print("=" * 60)
    print()

    bandwidth = driver.get_bandwidth_stats()
    print(f"Performance:")
    print(f"  Max simultaneous contacts: {max_simultaneous}")
    print(f"  Total renders: {total_renders}")
    print(f"  Bandwidth: {bandwidth['bandwidth_hz']:.1f} packets/s ({bandwidth['bandwidth_kbps']:.1f} kbps)")
    print()

    driver_stats = driver.get_stats()
    print(f"Hardware:")
    print(f"  Channels active: {driver_stats['driver_count']}")
    print(f"  Packets sent: {driver_stats['total_sent']}")
    print(f"  Success rate: {driver_stats['success_rate']:.1%}")
    print()

    # Per-channel breakdown
    print("Per-channel stats:")
    for body_part_id, name in channels.items():
        if body_part_id in driver_stats['per_channel_stats']:
            ch_stats = driver_stats['per_channel_stats'][body_part_id]
            print(f"  {name.capitalize():8s}: {ch_stats['packets_sent']} packets")
    print()

    # Cleanup
    driver.disconnect_all()
    print("✓ Demo complete")


if __name__ == "__main__":
    main()
