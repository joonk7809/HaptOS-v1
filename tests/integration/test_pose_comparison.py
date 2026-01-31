#!/usr/bin/env python3
"""
Compare Contact Patterns Across Different Hand Poses
Verifies that each pose produces distinct contact patterns.
"""

import sys
sys.path.append('src')

from modular_scenario_builder import *


def test_all_poses():
    """Test all available hand poses and compare contact patterns."""
    print("=" * 70)
    print(" " * 15 + "HAND POSE CONTACT PATTERN TEST")
    print("=" * 70)
    print("\nTesting each pose to verify distinct contact patterns...")
    print()

    poses = [
        ("Flat", HandPoseLibrary.flat_hand(), ['palm_center', 'index_geom', 'middle_geom', 'ring_geom', 'pinky_geom']),
        ("Pinch", HandPoseLibrary.pinch(), ['thumb_geom', 'index_geom']),
        ("Fist", HandPoseLibrary.fist(), ['palm_center']),
        ("Point", HandPoseLibrary.point(), ['index_geom']),
        ("Palm Up", HandPoseLibrary.palm_up(), ['palm_center']),
        ("Relaxed", HandPoseLibrary.relaxed(), ['palm_center']),
    ]

    runner = ModularScenarioRunner()
    all_results = {}

    for pose_name, pose_config, expected in poses:
        print(f"Testing: {pose_name:12s} ", end="", flush=True)

        scenario = ScenarioConfig(
            name=f"test_{pose_config.name}",
            description=f"Testing {pose_name}",
            hand_pose=pose_config,
            interactions=[],
            duration_s=2.0,
            expected_contacts=expected
        )

        try:
            results = runner.run_scenario(scenario, verbose=False)

            if results:
                contacts = results['detected_contacts']
                f1 = results['f1_score']
                all_results[pose_name] = results

                # Quick assessment
                if f1 > 0.5:
                    status = "✓ GOOD"
                elif f1 > 0.3:
                    status = "~ OK"
                else:
                    status = "✗ POOR"

                print(f"F1={f1:5.1%}  {status}  Contacts: {len(contacts)}")
            else:
                print("✗ FAILED")

        except Exception as e:
            print(f"✗ ERROR: {e}")

    # Print detailed comparison
    print("\n" + "=" * 70)
    print("DETAILED CONTACT COMPARISON")
    print("=" * 70)

    # Collect all unique contacts
    all_contacts = set()
    for results in all_results.values():
        all_contacts.update(results['detected_contacts'])

    # Print table header
    print(f"\n{'Contact':20s}", end="")
    for pose_name in all_results.keys():
        print(f" {pose_name[:8]:>8s}", end="")
    print()
    print("-" * 70)

    # Print each contact's presence across poses
    for contact in sorted(all_contacts):
        print(f"{contact:20s}", end="")
        for pose_name in all_results.keys():
            detected = all_results[pose_name]['detected_contacts']
            expected = all_results[pose_name]['expected_contacts']

            if contact in detected and contact in expected:
                marker = "✓"  # True positive
            elif contact in detected:
                marker = "x"  # False positive
            else:
                marker = " "  # Not detected

            print(f" {marker:>8s}", end="")
        print()

    # Print summary statistics
    print("\n" + "-" * 70)
    print("SUMMARY")
    print("-" * 70)

    for pose_name, results in all_results.items():
        tp = len(results['true_positives'])
        fp = len(results['false_positives'])
        fn = len(results['false_negatives'])
        f1 = results['f1_score']

        print(f"{pose_name:12s}: F1={f1:5.1%}  TP={tp:2d}  FP={fp:2d}  FN={fn:2d}")

    print("\n" + "=" * 70)
    print("INTERPRETATION")
    print("=" * 70)
    print("""
✓ = Contact detected and expected (TRUE POSITIVE - good!)
x = Contact detected but not expected (FALSE POSITIVE - extra contact)
  = Contact not detected (could be correct or FALSE NEGATIVE)

GOOD F1 score (>50%): Pose is working correctly
OK F1 score (30-50%): Pose partially working, needs adjustment
POOR F1 score (<30%): Pose not working as intended

Each pose should show a DIFFERENT pattern of contacts.
If multiple poses show identical patterns, poses aren't being applied correctly.
""")


if __name__ == "__main__":
    test_all_poses()
