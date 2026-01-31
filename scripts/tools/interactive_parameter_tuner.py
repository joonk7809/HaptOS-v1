#!/usr/bin/env python3
"""
Interactive Parameter Tuner
Allows real-time adjustment of simulation parameters via CLI.
"""

import sys
sys.path.append('src')

import yaml
import time
from pathlib import Path
from sync_multi_runner import MultiContactSyncRunner
from visualization.contact_logger import ContactEventLogger


class ParameterTuner:
    """Interactive parameter tuning system."""

    def __init__(self, config_path="config/simulation_config.yaml"):
        """Initialize parameter tuner."""
        self.config_path = config_path
        self.config = self.load_config()
        self.runner = None
        self.logger = None

    def load_config(self):
        """Load configuration from YAML file."""
        try:
            with open(self.config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"Warning: Could not load config: {e}")
            return self.get_default_config()

    def save_config(self):
        """Save current configuration to YAML file."""
        try:
            with open(self.config_path, 'w') as f:
                yaml.dump(self.config, f, default_flow_style=False, sort_keys=False)
            print(f"✓ Configuration saved to {self.config_path}")
        except Exception as e:
            print(f"ERROR: Could not save config: {e}")

    def get_default_config(self):
        """Get default configuration."""
        return {
            'hand': {'model_path': 'assets/hand_models/detailed_hand.xml'},
            'physics': {'timestep': 0.001, 'gravity': [0, 0, -9.81]},
            'contact': {'force_threshold': 0.01, 'slip_threshold': 5.0, 'max_contacts': 20},
            'phase_detection': {
                'impact_threshold': 0.5,
                'hold_time_ms': 20,
                'slip_speed_threshold': 5.0,
                'release_threshold': 0.25
            },
            'testing': {'duration_s': 5.0}
        }

    def print_menu(self):
        """Print main menu."""
        print("\n" + "=" * 70)
        print(" " * 20 + "HAPTIC PARAMETER TUNER")
        print("=" * 70)
        print("\nOptions:")
        print("  1) View current parameters")
        print("  2) Adjust physics parameters")
        print("  3) Adjust contact detection parameters")
        print("  4) Adjust phase detection parameters")
        print("  5) Run simulation with current parameters")
        print("  6) Run interactive test (adjust during simulation)")
        print("  7) Save configuration")
        print("  8) Reload configuration")
        print("  9) Reset to defaults")
        print("  0) Exit")
        print("=" * 70)

    def print_current_params(self):
        """Display all current parameters."""
        print("\n" + "=" * 70)
        print("CURRENT PARAMETERS")
        print("=" * 70)

        print("\n[Physics]")
        print(f"  Timestep: {self.config['physics']['timestep']*1000:.1f} ms ({1/self.config['physics']['timestep']:.0f} Hz)")
        print(f"  Gravity: {self.config['physics']['gravity']}")

        print("\n[Contact Detection]")
        print(f"  Force threshold: {self.config['contact']['force_threshold']:.3f} N")
        print(f"  Slip threshold: {self.config['contact']['slip_threshold']:.1f} mm/s")
        print(f"  Max contacts: {self.config['contact']['max_contacts']}")

        print("\n[Phase Detection]")
        print(f"  Impact threshold: {self.config['phase_detection']['impact_threshold']:.2f} N")
        print(f"  Hold time: {self.config['phase_detection']['hold_time_ms']:.0f} ms")
        print(f"  Slip speed threshold: {self.config['phase_detection']['slip_speed_threshold']:.1f} mm/s")
        print(f"  Release threshold: {self.config['phase_detection']['release_threshold']:.2f} N")

        print("\n[Testing]")
        print(f"  Duration: {self.config['testing']['duration_s']:.1f} s")

        print("=" * 70)

    def adjust_physics_params(self):
        """Adjust physics parameters."""
        print("\n" + "=" * 70)
        print("ADJUST PHYSICS PARAMETERS")
        print("=" * 70)

        # Timestep
        current = self.config['physics']['timestep'] * 1000
        print(f"\nCurrent timestep: {current:.2f} ms")
        new = input(f"New timestep (ms) [press Enter to keep]: ").strip()
        if new:
            try:
                self.config['physics']['timestep'] = float(new) / 1000
                print(f"✓ Set to {float(new):.2f} ms ({1000/float(new):.0f} Hz)")
            except ValueError:
                print("Invalid value")

        # Gravity
        print(f"\nCurrent gravity: {self.config['physics']['gravity']}")
        new = input(f"New gravity Z component (default -9.81) [press Enter to keep]: ").strip()
        if new:
            try:
                self.config['physics']['gravity'][2] = float(new)
                print(f"✓ Set to {float(new):.2f} m/s²")
            except ValueError:
                print("Invalid value")

    def adjust_contact_params(self):
        """Adjust contact detection parameters."""
        print("\n" + "=" * 70)
        print("ADJUST CONTACT DETECTION PARAMETERS")
        print("=" * 70)

        # Force threshold
        current = self.config['contact']['force_threshold']
        print(f"\nCurrent force threshold: {current:.3f} N")
        print("(Minimum force to register contact)")
        new = input(f"New threshold (N) [press Enter to keep]: ").strip()
        if new:
            try:
                self.config['contact']['force_threshold'] = float(new)
                print(f"✓ Set to {float(new):.3f} N")
            except ValueError:
                print("Invalid value")

        # Slip threshold
        current = self.config['contact']['slip_threshold']
        print(f"\nCurrent slip threshold: {current:.1f} mm/s")
        print("(Minimum velocity to detect slip)")
        new = input(f"New threshold (mm/s) [press Enter to keep]: ").strip()
        if new:
            try:
                self.config['contact']['slip_threshold'] = float(new)
                print(f"✓ Set to {float(new):.1f} mm/s")
            except ValueError:
                print("Invalid value")

    def adjust_phase_params(self):
        """Adjust phase detection parameters."""
        print("\n" + "=" * 70)
        print("ADJUST PHASE DETECTION PARAMETERS")
        print("=" * 70)

        params = [
            ('impact_threshold', "Impact threshold (N)", "Force to trigger IMPACT phase"),
            ('hold_time_ms', "Hold time (ms)", "Time before entering HOLD phase"),
            ('slip_speed_threshold', "Slip speed threshold (mm/s)", "Velocity to trigger SLIP phase"),
            ('release_threshold', "Release threshold (N)", "Force below which RELEASE phase triggers")
        ]

        for key, label, description in params:
            current = self.config['phase_detection'][key]
            print(f"\nCurrent {label}: {current}")
            print(f"({description})")
            new = input(f"New value [press Enter to keep]: ").strip()
            if new:
                try:
                    self.config['phase_detection'][key] = float(new)
                    print(f"✓ Set to {float(new)}")
                except ValueError:
                    print("Invalid value")

    def run_simulation(self):
        """Run simulation with current parameters."""
        print("\n" + "=" * 70)
        print("RUNNING SIMULATION")
        print("=" * 70)
        print()

        # Initialize system
        try:
            model_path = self.config['hand']['model_path']
            print(f"Loading hand model: {model_path}")
            runner = MultiContactSyncRunner(
                model_path=model_path,
                nn_v0_path="models/checkpoints/nn_v0_best.pt",
                nn_v1_path="models/checkpoints/nn_v1_best.pt",
                device='cpu'
            )
            logger = ContactEventLogger()
        except Exception as e:
            print(f"ERROR: Failed to initialize: {e}")
            return

        # Run simulation
        duration = self.config['testing']['duration_s']
        n_steps = int(duration * 100)  # 100 Hz

        print(f"Running {duration:.1f}s simulation...")
        print("Press Ctrl+C to stop early")
        print()

        try:
            contact_history = {}

            for step in range(n_steps):
                result = runner.step()

                # Track contacts
                for body_part, contact in result['contacts'].items():
                    if body_part not in contact_history:
                        contact_history[body_part] = []
                    contact_history[body_part].append(contact.normal_force_N)

                    # Log strong contacts
                    if contact.normal_force_N > 0.5:
                        timestamp_s = result['timestamp_us'] / 1e6
                        pred = result['predictions'].get(body_part, {})
                        impact_A = pred.get('impact', {}).get('A', 0)
                        weight_A = pred.get('weight', {}).get('A', 0)
                        print(f"[{timestamp_s:.2f}s] {body_part}: F={contact.normal_force_N:.2f}N, "
                              f"Impact={impact_A:.2f}, Weight={weight_A:.2f}")

                # Progress
                if step % 50 == 0:
                    progress = 100 * step / n_steps
                    n_contacts = len(result['contacts'])
                    print(f"[{progress:.0f}%] Active contacts: {n_contacts}")

                time.sleep(0.01)  # Real-time

        except KeyboardInterrupt:
            print("\n\nStopped by user")

        # Summary
        print("\n" + "=" * 70)
        print("SIMULATION SUMMARY")
        print("=" * 70)
        print(f"\nContacts detected: {len(contact_history)}")
        for body_part, forces in contact_history.items():
            max_force = max(forces) if forces else 0
            avg_force = sum(forces) / len(forces) if forces else 0
            print(f"  {body_part}: max={max_force:.2f}N, avg={avg_force:.3f}N")

    def run_interactive_test(self):
        """Run interactive test with real-time parameter adjustment."""
        print("\n" + "=" * 70)
        print("INTERACTIVE TEST MODE")
        print("=" * 70)
        print("\nThis mode runs simulation continuously.")
        print("Type commands to adjust parameters on-the-fly:")
        print("  'impact <value>' - Set impact threshold")
        print("  'slip <value>' - Set slip threshold")
        print("  'hold <value>' - Set hold time (ms)")
        print("  'stop' - Stop simulation")
        print()

        # TODO: Implement real-time adjustment
        print("Feature coming soon!")
        print("For now, adjust parameters in menu and run simulation.")

    def run(self):
        """Run interactive tuner."""
        while True:
            self.print_menu()
            choice = input("\nEnter choice (0-9): ").strip()

            if choice == '0':
                print("\nExiting...")
                break
            elif choice == '1':
                self.print_current_params()
                input("\nPress Enter to continue...")
            elif choice == '2':
                self.adjust_physics_params()
            elif choice == '3':
                self.adjust_contact_params()
            elif choice == '4':
                self.adjust_phase_params()
            elif choice == '5':
                self.run_simulation()
                input("\nPress Enter to continue...")
            elif choice == '6':
                self.run_interactive_test()
                input("\nPress Enter to continue...")
            elif choice == '7':
                self.save_config()
                input("\nPress Enter to continue...")
            elif choice == '8':
                self.config = self.load_config()
                print("✓ Configuration reloaded")
                input("\nPress Enter to continue...")
            elif choice == '9':
                self.config = self.get_default_config()
                print("✓ Reset to defaults")
                input("\nPress Enter to continue...")
            else:
                print("Invalid choice")
                time.sleep(1)


def main():
    """Main entry point."""
    tuner = ParameterTuner()
    tuner.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
