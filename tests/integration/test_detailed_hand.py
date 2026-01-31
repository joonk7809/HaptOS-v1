#!/usr/bin/env python3
"""
Test the detailed hand model with 18 contact sensors.
Shows significantly more contact information than the simple 6-sensor model.
"""

import sys
sys.path.append('src')

import time
from physics.multi_contact_engine import MultiContactEngine

def main():
    print("=" * 70)
    print(" " * 15 + "DETAILED HAND MODEL TEST")
    print(" " * 10 + "(18 Contact Sensors vs 6 Simple)")
    print("=" * 70)
    print()

    # Load detailed hand model
    print("Loading detailed hand model...")
    model_path = "assets/hand_models/detailed_hand.xml"

    try:
        engine = MultiContactEngine(model_path, max_contacts=20)
        print(f"✓ Multi-contact engine initialized:")
        print(f"  Max contacts: {engine.max_contacts}")
        print(f"  Body parts tracked: {len(engine.geom_to_body)}")

        # Show all tracked body parts
        body_parts = sorted(set(engine.geom_to_body.values()))
        print(f"\n✓ Hand model loaded with {len(body_parts)} contact sensors:")

        # Organize by category
        palm_parts = [p for p in body_parts if 'palm' in p]
        thumb_parts = [p for p in body_parts if 'thumb' in p and 'palm' not in p]
        index_parts = [p for p in body_parts if 'index' in p]
        middle_parts = [p for p in body_parts if 'middle' in p and not p.startswith('middle')]
        ring_parts = [p for p in body_parts if 'ring' in p]
        pinky_parts = [p for p in body_parts if 'pinky' in p]

        print(f"\n  Palm zones ({len(palm_parts)}): {', '.join(palm_parts)}")
        print(f"  Thumb segments ({len(thumb_parts)}): {', '.join(thumb_parts)}")
        print(f"  Index segments ({len(index_parts)}): {', '.join(index_parts)}")
        print(f"  Middle segments ({len(middle_parts)}): {', '.join(middle_parts)}")
        print(f"  Ring segments ({len(ring_parts)}): {', '.join(ring_parts)}")
        print(f"  Pinky segments ({len(pinky_parts)}): {', '.join(pinky_parts)}")

    except Exception as e:
        print(f"✗ Error loading hand model: {e}")
        return 1

    # Run simulation
    duration_s = 3.0
    print(f"\n{'='*70}")
    print(f"Running {duration_s}-second simulation...")
    print("The hand will fall and make contact with the floor")
    print("Watch for detailed contact information per finger segment!")
    print(f"{'='*70}\n")

    start_time = time.time()
    step_count = 0
    contact_history = {}  # Track when each body part first made contact

    while time.time() - start_time < duration_s:
        # Step physics
        contact_dict = engine.step_multi()
        step_count += 1

        sim_time = time.time() - start_time

        # Log new contacts
        for body_part, contact in contact_dict.items():
            if body_part not in contact_history:
                contact_history[body_part] = sim_time
                force = contact.normal_force_N
                print(f"[{sim_time:.3f}s] ✓ {body_part} contact! Force: {force:.3f}N")

        # Print summary every 0.5s
        if step_count % 500 == 0:
            total_force = sum(c.normal_force_N for c in contact_dict.values())
            n_contacts = len(contact_dict)
            print(f"[{sim_time:.1f}s] Active contacts: {n_contacts}, Total force: {total_force:.2f}N")

    print(f"\n{'='*70}")
    print("Simulation complete!")
    print(f"\nContact Summary ({len(contact_history)} unique contacts detected):")

    # Sort by first contact time
    sorted_contacts = sorted(contact_history.items(), key=lambda x: x[1])

    for body_part, first_time in sorted_contacts:
        print(f"  {body_part:25s}: First contact at {first_time:.3f}s")

    print(f"{'='*70}")

    # Compare with simple hand
    print("\n💡 Comparison:")
    print(f"  Simple hand:   6 contact sensors")
    print(f"  Detailed hand: {len(body_parts)} contact sensors")
    print(f"  Improvement:   {len(body_parts) - 6} additional sensors ({((len(body_parts)-6)/6*100):.0f}% more detail)")
    print()

    return 0

if __name__ == "__main__":
    sys.exit(main())
