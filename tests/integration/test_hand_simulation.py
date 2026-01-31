#!/usr/bin/env python3
"""
Test hand simulation without viewer.
Runs physics simulation and prints contact information.
"""

import sys
sys.path.append('src')

import time
from physics.multi_contact_engine import MultiContactEngine


def main():
    """Test hand simulation without viewer."""

    print("=" * 70)
    print(" " * 20 + "Hand Simulation Test")
    print("=" * 70)
    print()

    # Load hand model
    print("Loading hand model...")
    try:
        engine = MultiContactEngine("assets/hand_models/simple_hand.xml")
        print("✓ Hand model loaded successfully")
        print(f"  Body parts: {list(engine.geom_to_body.values())}")
        print()
    except Exception as e:
        print(f"ERROR: Failed to load hand model: {e}")
        print("\nMake sure MuJoCo is installed:")
        print("  pip install mujoco")
        return 1

    print("Running 3-second simulation...")
    print("The hand will fall and make contact with the floor")
    print()

    # Run simulation for 3 seconds
    n_steps = 3000  # 3 seconds @ 1kHz
    contact_started = {}

    try:
        for step in range(n_steps):
            # Step physics
            contacts = engine.step_multi()

            # Track new contacts
            for body_part, contact in contacts.items():
                if body_part not in contact_started and contact.normal_force_N > 0.1:
                    contact_started[body_part] = step
                    time_s = step / 1000.0
                    print(f"[{time_s:.3f}s] ✓ {body_part} contact! Force: {contact.normal_force_N:.3f}N")

            # Print status every 500ms
            if step % 500 == 0:
                time_s = step / 1000.0
                n_contacts = len(contacts)
                total_force = sum(c.normal_force_N for c in contacts.values())
                print(f"[{time_s:.1f}s] Active contacts: {n_contacts}, Total force: {total_force:.2f}N")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user (Ctrl+C)")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1

    print()
    print("=" * 70)
    print("Simulation complete!")
    print()
    print("Contact Summary:")
    for body_part in sorted(contact_started.keys()):
        time_s = contact_started[body_part] / 1000.0
        print(f"  {body_part}: First contact at {time_s:.3f}s")

    if not contact_started:
        print("  No contacts detected")
        print("  (This is expected if hand spawned above floor)")

    print("=" * 70)

    return 0


if __name__ == "__main__":
    sys.exit(main())
