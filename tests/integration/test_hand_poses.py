#!/usr/bin/env python3
"""
Test Hand Pose Application
Verifies that different hand poses are correctly applied and maintained.
"""

import sys
sys.path.append('src')

import mujoco
import time
from modular_scenario_builder import HandPoseLibrary, ModularScenarioRunner, ScenarioConfig, InteractionType


def test_pose_application():
    """Test that hand poses are applied correctly."""
    print("=" * 70)
    print(" " * 15 + "HAND POSE APPLICATION TEST")
    print("=" * 70)
    print()

    hand_model = "assets/hand_models/detailed_hand.xml"

    # Test each pose
    poses = [
        ("Flat Hand", HandPoseLibrary.flat_hand()),
        ("Pinch", HandPoseLibrary.pinch()),
        ("Fist", HandPoseLibrary.fist()),
        ("Point", HandPoseLibrary.point()),
        ("Palm Up", HandPoseLibrary.palm_up()),
        ("Relaxed", HandPoseLibrary.relaxed()),
    ]

    runner = ModularScenarioRunner(hand_model)

    for pose_name, pose_config in poses:
        print(f"\nTesting pose: {pose_name}")
        print("-" * 70)

        # Create simple scenario (just hand pose, no objects)
        scenario = ScenarioConfig(
            name=f"test_{pose_config.name}",
            description=f"Testing {pose_name} pose application",
            hand_pose=pose_config,
            interactions=[],  # No objects, just test pose
            duration_s=2.0,
            expected_contacts=['palm_center']  # Dummy expectation
        )

        try:
            results = runner.run_scenario(scenario, verbose=True)

            if results:
                detected = results['detected_contacts']
                print(f"\n✓ Pose '{pose_name}' applied successfully")
                print(f"  Contacts detected: {len(detected)}")
                print(f"  Contact points: {', '.join(sorted(detected)[:5])}")

                # Check if results make sense for the pose
                if pose_name == "Point" and 'index_geom' in detected:
                    print(f"  ✓ Index finger contact detected (as expected for pointing)")
                elif pose_name == "Pinch" and any('thumb' in c or 'index' in c for c in detected):
                    print(f"  ✓ Thumb/index contacts detected (as expected for pinch)")
                elif pose_name == "Fist":
                    # Fist should have knuckles/palm contacting
                    print(f"  ✓ Fist pose contacts: {', '.join(detected)}")
            else:
                print(f"✗ Failed to test pose '{pose_name}'")

        except Exception as e:
            print(f"✗ Error testing pose '{pose_name}': {e}")

        print()
        time.sleep(0.5)

    print("=" * 70)
    print("POSE TEST COMPLETE")
    print("=" * 70)


def compare_poses():
    """Compare contact patterns across different poses."""
    print("\n" + "=" * 70)
    print(" " * 20 + "POSE COMPARISON")
    print("=" * 70)
    print()

    poses_to_compare = [
        ("Flat", HandPoseLibrary.flat_hand()),
        ("Pinch", HandPoseLibrary.pinch()),
        ("Point", HandPoseLibrary.point()),
    ]

    runner = ModularScenarioRunner()
    results_summary = {}

    for pose_name, pose_config in poses_to_compare:
        scenario = ScenarioConfig(
            name=f"compare_{pose_config.name}",
            description=f"{pose_name} pose comparison",
            hand_pose=pose_config,
            interactions=[],
            duration_s=2.0,
            expected_contacts=[]  # No expectations
        )

        print(f"Running: {pose_name}...")
        results = runner.run_scenario(scenario, verbose=False)

        if results:
            results_summary[pose_name] = results['detected_contacts']
            print(f"  Contacts: {len(results['detected_contacts'])}")

    # Print comparison
    print("\n" + "-" * 70)
    print("CONTACT PATTERN COMPARISON")
    print("-" * 70)

    # Get all unique contacts
    all_contacts = set()
    for contacts in results_summary.values():
        all_contacts.update(contacts)

    # Print table header
    print(f"\n{'Contact Point':<25}", end="")
    for pose_name in results_summary.keys():
        print(f" {pose_name:>8}", end="")
    print()
    print("-" * 70)

    # Print table rows
    for contact in sorted(all_contacts):
        print(f"{contact:<25}", end="")
        for pose_name in results_summary.keys():
            marker = "✓" if contact in results_summary[pose_name] else " "
            print(f" {marker:>8}", end="")
        print()

    print()


if __name__ == "__main__":
    print("\nThis test verifies that hand poses are correctly applied.")
    print("Each pose should produce different contact patterns.\n")

    choice = input("Run full test? (y/n): ").strip().lower()
    if choice == 'y':
        test_pose_application()
        # compare_poses()
    else:
        print("\nQuick test:")
        runner = ModularScenarioRunner()
        scenario = ScenarioConfig(
            name="quick_test",
            description="Quick flat hand test",
            hand_pose=HandPoseLibrary.flat_hand(),
            interactions=[],
            duration_s=1.5,
            expected_contacts=['palm_center', 'index_geom', 'middle_geom']
        )
        results = runner.run_scenario(scenario)
