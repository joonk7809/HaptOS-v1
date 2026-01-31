#!/usr/bin/env python3
"""
Simple haptics test - No GUI version
Tests multi-contact system without Tkinter dependency.
"""

import sys
sys.path.append('src')

import time
from sync_multi_runner import MultiContactSyncRunner
from visualization.contact_logger import ContactEventLogger


def main():
    """Run simple multi-contact test without GUI."""

    print("=" * 70)
    print(" " * 15 + "HaptOS Multi-Contact Test (No GUI)")
    print("=" * 70)
    print()

    # Initialize system
    print("Initializing multi-contact system...")
    try:
        runner = MultiContactSyncRunner(
            model_path="assets/hand_models/simple_hand.xml",
            nn_v0_path="models/checkpoints/nn_v0_best.pt",
            nn_v1_path="models/checkpoints/nn_v1_best.pt",
            device='cpu'
        )
    except Exception as e:
        print(f"ERROR: Failed to initialize: {e}")
        print("\nMake sure you have installed dependencies:")
        print("  pip install mujoco torch matplotlib scipy numpy")
        return 1

    # Initialize logger
    logger = ContactEventLogger()

    print("\n" + "=" * 70)
    print("Running 5-second simulation...")
    print("Watch the hand fall and make contact with the floor")
    print("=" * 70)
    print()

    # Run simulation for 5 seconds
    n_steps = 500  # 5 seconds @ 100Hz

    try:
        for step in range(n_steps):
            # Step simulation
            result = runner.step()

            # Get data
            contacts = result['contacts']
            predictions = result['predictions']
            timestamp_s = result['timestamp_us'] / 1e6

            # Log contact events
            for body_part, contact in contacts.items():
                if contact.normal_force_N > 0.1:
                    logger.log_contact_start(body_part, contact.normal_force_N, timestamp_s)

            # Print status every second
            if step % 100 == 0:
                n_contacts = len(contacts)
                print(f"[{timestamp_s:.2f}s] Active contacts: {n_contacts}")
                for body_part, contact in contacts.items():
                    pred = predictions.get(body_part, {})
                    impact_A = pred.get('impact', {}).get('A', 0)
                    weight_A = pred.get('weight', {}).get('A', 0)
                    print(f"  {body_part}: Force={contact.normal_force_N:.3f}N, "
                          f"Impact={impact_A:.3f}, Weight={weight_A:.3f}")

            # Small delay to maintain real-time (10ms per step)
            time.sleep(0.01)

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
    print(f"Log saved to: {logger.get_log_path()}")
    print("=" * 70)

    return 0


if __name__ == "__main__":
    sys.exit(main())
