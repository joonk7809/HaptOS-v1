#!/usr/bin/env python3
"""
Scenario Testing Framework
Predefined test scenarios for systematic evaluation.
"""

import sys
sys.path.append('src')

import time
import json
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Dict
import numpy as np

from sync_multi_runner import MultiContactSyncRunner
from visualization.contact_logger import ContactEventLogger


@dataclass
class TestScenario:
    """Test scenario configuration."""
    name: str
    description: str
    hand_model: str
    duration_s: float
    hand_pose: Dict[str, float]  # joint_name -> angle_rad
    expected_contacts: List[str]  # Expected body parts in contact


class ScenarioLibrary:
    """Library of predefined test scenarios."""

    @staticmethod
    def get_all_scenarios():
        """Get all available test scenarios."""
        return [
            ScenarioLibrary.flat_hand_impact(),
            ScenarioLibrary.pinch_pose(),
            ScenarioLibrary.grasp_pose(),
            ScenarioLibrary.finger_tap(),
            ScenarioLibrary.palm_pressure()
        ]

    @staticmethod
    def flat_hand_impact():
        """Flat hand hitting floor (palm + all fingertips)."""
        return TestScenario(
            name="flat_hand_impact",
            description="Hand falls flat onto surface, all contacts expected",
            hand_model="assets/hand_models/detailed_hand.xml",
            duration_s=3.0,
            hand_pose={
                'thumb_cmc': 0.0, 'thumb_mcp': 0.0, 'thumb_ip': 0.0,
                'index_mcp': 0.0, 'index_pip': 0.0, 'index_dip': 0.0,
                'middle_mcp': 0.0, 'middle_pip': 0.0, 'middle_dip': 0.0,
                'ring_mcp': 0.0, 'ring_pip': 0.0, 'ring_dip': 0.0,
                'pinky_mcp': 0.0, 'pinky_pip': 0.0, 'pinky_dip': 0.0
            },
            expected_contacts=['palm_center', 'thumb_geom', 'index_geom',
                             'middle_geom', 'ring_geom', 'pinky_geom']
        )

    @staticmethod
    def pinch_pose():
        """Thumb and index finger pinch."""
        return TestScenario(
            name="pinch_pose",
            description="Thumb and index finger extended, others closed",
            hand_model="assets/hand_models/detailed_hand.xml",
            duration_s=3.0,
            hand_pose={
                'thumb_cmc': 0.8, 'thumb_mcp': 0.5, 'thumb_ip': 0.3,
                'index_mcp': 0.8, 'index_pip': 0.5, 'index_dip': 0.3,
                'middle_mcp': 1.4, 'middle_pip': 1.2, 'middle_dip': 1.0,
                'ring_mcp': 1.4, 'ring_pip': 1.2, 'ring_dip': 1.0,
                'pinky_mcp': 1.4, 'pinky_pip': 1.2, 'pinky_dip': 1.0
            },
            expected_contacts=['thumb_geom', 'index_geom']
        )

    @staticmethod
    def grasp_pose():
        """All fingers closed (grasp)."""
        return TestScenario(
            name="grasp_pose",
            description="All fingers flexed for grasping",
            hand_model="assets/hand_models/detailed_hand.xml",
            duration_s=3.0,
            hand_pose={
                'thumb_cmc': 1.2, 'thumb_mcp': 1.0, 'thumb_ip': 0.8,
                'index_mcp': 1.4, 'index_pip': 1.2, 'index_dip': 1.0,
                'middle_mcp': 1.4, 'middle_pip': 1.2, 'middle_dip': 1.0,
                'ring_mcp': 1.4, 'ring_pip': 1.2, 'ring_dip': 1.0,
                'pinky_mcp': 1.4, 'pinky_pip': 1.2, 'pinky_dip': 1.0
            },
            expected_contacts=['palm_center', 'thumb_mid_geom', 'index_mid_geom',
                             'middle_mid_geom', 'ring_mid_geom', 'pinky_mid_geom']
        )

    @staticmethod
    def finger_tap():
        """Index finger tap (point)."""
        return TestScenario(
            name="finger_tap",
            description="Index finger extended, others closed",
            hand_model="assets/hand_models/detailed_hand.xml",
            duration_s=3.0,
            hand_pose={
                'thumb_cmc': 1.2, 'thumb_mcp': 1.0, 'thumb_ip': 0.8,
                'index_mcp': 0.0, 'index_pip': 0.0, 'index_dip': 0.0,
                'middle_mcp': 1.4, 'middle_pip': 1.2, 'middle_dip': 1.0,
                'ring_mcp': 1.4, 'ring_pip': 1.2, 'ring_dip': 1.0,
                'pinky_mcp': 1.4, 'pinky_pip': 1.2, 'pinky_dip': 1.0
            },
            expected_contacts=['index_geom']
        )

    @staticmethod
    def palm_pressure():
        """Palm pressure (fingers up)."""
        return TestScenario(
            name="palm_pressure",
            description="Fingers extended up, palm pushes down",
            hand_model="assets/hand_models/detailed_hand.xml",
            duration_s=3.0,
            hand_pose={
                'thumb_cmc': -0.2, 'thumb_mcp': -0.2, 'thumb_ip': 0.0,
                'index_mcp': -0.1, 'index_pip': -0.1, 'index_dip': 0.0,
                'middle_mcp': -0.1, 'middle_pip': -0.1, 'middle_dip': 0.0,
                'ring_mcp': -0.1, 'ring_pip': -0.1, 'ring_dip': 0.0,
                'pinky_mcp': -0.1, 'pinky_pip': -0.1, 'pinky_dip': 0.0
            },
            expected_contacts=['palm_center', 'palm_base']
        )


class ScenarioRunner:
    """Run and evaluate test scenarios."""

    def __init__(self):
        """Initialize scenario runner."""
        self.results = []

    def run_scenario(self, scenario: TestScenario, verbose=True):
        """
        Run a single scenario and collect results.

        Args:
            scenario: TestScenario to run
            verbose: Print detailed output

        Returns:
            Dict with results
        """
        if verbose:
            print("\n" + "=" * 70)
            print(f"SCENARIO: {scenario.name}")
            print("=" * 70)
            print(f"Description: {scenario.description}")
            print(f"Duration: {scenario.duration_s}s")
            print(f"Expected contacts: {', '.join(scenario.expected_contacts)}")
            print()

        # Initialize system
        try:
            runner = MultiContactSyncRunner(
                model_path=scenario.hand_model,
                nn_v0_path="models/checkpoints/nn_v0_best.pt",
                nn_v1_path="models/checkpoints/nn_v1_best.pt",
                device='cpu'
            )

            # Set hand pose
            self._set_hand_pose(runner, scenario.hand_pose)

        except Exception as e:
            print(f"ERROR: Failed to initialize: {e}")
            return None

        # Run simulation
        n_steps = int(scenario.duration_s * 100)
        contact_data = {}
        force_peaks = {}

        try:
            for step in range(n_steps):
                result = runner.step()

                # Collect contact data
                for body_part, contact in result['contacts'].items():
                    if body_part not in contact_data:
                        contact_data[body_part] = {
                            'forces': [],
                            'first_contact_time': result['timestamp_us'] / 1e6,
                            'total_time': 0
                        }

                    contact_data[body_part]['forces'].append(contact.normal_force_N)

                    # Track peak force
                    if body_part not in force_peaks or contact.normal_force_N > force_peaks[body_part]:
                        force_peaks[body_part] = contact.normal_force_N

                # Progress
                if verbose and step % 50 == 0:
                    progress = 100 * step / n_steps
                    n_contacts = len(result['contacts'])
                    print(f"[{progress:.0f}%] Active contacts: {n_contacts}")

        except Exception as e:
            print(f"ERROR during simulation: {e}")
            import traceback
            traceback.print_exc()
            return None

        # Analyze results
        results = self._analyze_results(scenario, contact_data, force_peaks)

        if verbose:
            self._print_results(results)

        self.results.append(results)
        return results

    def _set_hand_pose(self, runner, hand_pose):
        """Set hand joint angles."""
        model = runner.physics.model
        data = runner.physics.data

        for joint_name, angle_rad in hand_pose.items():
            try:
                joint_id = model.joint(joint_name).id
                data.qpos[joint_id] = angle_rad
            except Exception as e:
                print(f"Warning: Could not set joint {joint_name}: {e}")

    def _analyze_results(self, scenario, contact_data, force_peaks):
        """Analyze scenario results."""
        detected_contacts = list(contact_data.keys())
        expected = set(scenario.expected_contacts)
        actual = set(detected_contacts)

        # Calculate metrics
        true_positives = expected & actual
        false_positives = actual - expected
        false_negatives = expected - actual

        precision = len(true_positives) / len(actual) if actual else 0
        recall = len(true_positives) / len(expected) if expected else 0
        f1_score = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

        return {
            'scenario': scenario.name,
            'description': scenario.description,
            'expected_contacts': list(expected),
            'detected_contacts': list(actual),
            'true_positives': list(true_positives),
            'false_positives': list(false_positives),
            'false_negatives': list(false_negatives),
            'precision': precision,
            'recall': recall,
            'f1_score': f1_score,
            'force_peaks': force_peaks,
            'total_contacts': len(actual)
        }

    def _print_results(self, results):
        """Print scenario results."""
        print("\n" + "-" * 70)
        print("RESULTS")
        print("-" * 70)

        print(f"\nDetected contacts: {results['total_contacts']}")
        for contact in results['detected_contacts']:
            peak = results['force_peaks'].get(contact, 0)
            marker = "✓" if contact in results['true_positives'] else "✗"
            print(f"  {marker} {contact}: peak force = {peak:.2f}N")

        if results['false_negatives']:
            print(f"\nMissed contacts (expected but not detected):")
            for contact in results['false_negatives']:
                print(f"  ✗ {contact}")

        print(f"\nMetrics:")
        print(f"  Precision: {results['precision']:.2%}")
        print(f"  Recall: {results['recall']:.2%}")
        print(f"  F1 Score: {results['f1_score']:.2%}")

    def print_summary(self):
        """Print summary of all scenarios."""
        if not self.results:
            print("No scenarios run yet")
            return

        print("\n" + "=" * 70)
        print("SCENARIO SUMMARY")
        print("=" * 70)

        for result in self.results:
            print(f"\n{result['scenario']}:")
            print(f"  Contacts: {result['total_contacts']} detected, "
                  f"{len(result['expected_contacts'])} expected")
            print(f"  F1 Score: {result['f1_score']:.2%}")
            print(f"  TP: {len(result['true_positives'])}, "
                  f"FP: {len(result['false_positives'])}, "
                  f"FN: {len(result['false_negatives'])}")

        # Overall statistics
        avg_f1 = np.mean([r['f1_score'] for r in self.results])
        avg_precision = np.mean([r['precision'] for r in self.results])
        avg_recall = np.mean([r['recall'] for r in self.results])

        print(f"\n" + "-" * 70)
        print(f"Overall Averages:")
        print(f"  Precision: {avg_precision:.2%}")
        print(f"  Recall: {avg_recall:.2%}")
        print(f"  F1 Score: {avg_f1:.2%}")

    def save_results(self, filename="test_results.json"):
        """Save results to JSON file."""
        output_dir = Path("data/test_results")
        output_dir.mkdir(parents=True, exist_ok=True)

        output_path = output_dir / filename

        with open(output_path, 'w') as f:
            json.dump(self.results, f, indent=2)

        print(f"\n✓ Results saved to {output_path}")


def main():
    """Main entry point."""
    print("=" * 70)
    print(" " * 20 + "SCENARIO TESTING FRAMEWORK")
    print("=" * 70)

    scenarios = ScenarioLibrary.get_all_scenarios()

    print(f"\nAvailable scenarios: {len(scenarios)}")
    for i, scenario in enumerate(scenarios, 1):
        print(f"  {i}) {scenario.name} - {scenario.description}")

    print("\nOptions:")
    print("  0) Run all scenarios")
    print("  1-5) Run specific scenario")
    print("  q) Quit")

    choice = input("\nEnter choice: ").strip()

    if choice.lower() == 'q':
        return 0

    runner = ScenarioRunner()

    if choice == '0':
        # Run all
        for scenario in scenarios:
            runner.run_scenario(scenario)
            input("\nPress Enter to continue to next scenario...")
    else:
        # Run specific
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(scenarios):
                runner.run_scenario(scenarios[idx])
            else:
                print("Invalid choice")
                return 1
        except ValueError:
            print("Invalid choice")
            return 1

    # Summary
    runner.print_summary()

    # Save results
    save = input("\nSave results to file? (y/n): ").strip().lower()
    if save == 'y':
        runner.save_results()

    return 0


if __name__ == "__main__":
    sys.exit(main())
