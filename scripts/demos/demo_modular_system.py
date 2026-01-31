#!/usr/bin/env python3
"""
Modular Testing System Demo
Demonstrates all new features in one script.
"""

import sys
sys.path.append('src')

import yaml
from pathlib import Path


def print_header(title):
    """Print section header."""
    print("\n" + "=" * 70)
    print(f" {title}")
    print("=" * 70)


def demo_config_system():
    """Demo 1: Configuration system."""
    print_header("DEMO 1: Configuration System")

    config_path = "config/simulation_config.yaml"

    if Path(config_path).exists():
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        print("\n✓ Configuration loaded from:", config_path)
        print("\nKey parameters:")
        print(f"  Physics timestep: {config['physics']['timestep']*1000:.1f} ms")
        print(f"  Contact force threshold: {config['contact']['force_threshold']:.3f} N")
        print(f"  Impact threshold: {config['phase_detection']['impact_threshold']:.2f} N")
        print(f"  Max contacts: {config['contact']['max_contacts']}")

        print("\n✓ Configuration is fully editable!")
        print(f"  Edit: {config_path}")
        print(f"  Or use: python interactive_parameter_tuner.py")
    else:
        print(f"\n⚠ Config file not found: {config_path}")


def demo_hand_models():
    """Demo 2: Hand model comparison."""
    print_header("DEMO 2: Hand Model Comparison")

    models = [
        ("simple_hand.xml", "Simple hand with 6 sensors"),
        ("detailed_hand.xml", "Detailed hand with 18 sensors")
    ]

    for model_file, description in models:
        model_path = f"assets/hand_models/{model_file}"
        if Path(model_path).exists():
            print(f"\n✓ {model_file}")
            print(f"   {description}")
            print(f"   Path: {model_path}")

            # Try to load and show stats
            try:
                import mujoco
                model = mujoco.MjModel.from_xml_path(model_path)
                print(f"   Geoms: {model.ngeom}")
                print(f"   Bodies: {model.nbody}")
                print(f"   Joints: {model.njnt}")
                print(f"   Sensors: {model.nsensor}")
            except Exception as e:
                print(f"   (Could not load: {e})")
        else:
            print(f"\n✗ {model_file} not found")

    print("\n✓ Detailed hand provides:")
    print("  • 3x more contact sensors (18 vs 6)")
    print("  • 3 segments per finger (proximal, middle, tip)")
    print("  • 3 palm zones (center, thumb base, base)")
    print("  • Better visual quality (capsules vs spheres)")


def demo_testing_tools():
    """Demo 3: Testing tools."""
    print_header("DEMO 3: Testing Tools")

    tools = [
        ("interactive_parameter_tuner.py", "Adjust parameters interactively", "Parameter tuning"),
        ("test_scenarios.py", "Run predefined test scenarios", "Scenario testing"),
        ("visualize_contacts.py", "Create 2D contact diagrams", "Visualization"),
        ("test_hand_simulation.py", "Simple simulation test", "Basic testing")
    ]

    print("\nAvailable testing tools:")
    for i, (script, description, category) in enumerate(tools, 1):
        exists = Path(script).exists()
        status = "✓" if exists else "✗"
        print(f"\n{i}. {status} {script}")
        print(f"   {description}")
        print(f"   Category: {category}")
        if exists:
            print(f"   Run: python {script}")


def demo_scenarios():
    """Demo 4: Test scenarios."""
    print_header("DEMO 4: Predefined Test Scenarios")

    try:
        from test_scenarios import ScenarioLibrary

        scenarios = ScenarioLibrary.get_all_scenarios()

        print(f"\n✓ {len(scenarios)} predefined scenarios available:")

        for i, scenario in enumerate(scenarios, 1):
            print(f"\n{i}. {scenario.name}")
            print(f"   Description: {scenario.description}")
            print(f"   Duration: {scenario.duration_s}s")
            print(f"   Expected contacts: {len(scenario.expected_contacts)}")
            print(f"   Contacts: {', '.join(scenario.expected_contacts[:3])}", end="")
            if len(scenario.expected_contacts) > 3:
                print(f" + {len(scenario.expected_contacts)-3} more")
            else:
                print()

        print("\n✓ Run scenarios with: python test_scenarios.py")

    except Exception as e:
        print(f"\n⚠ Could not load scenarios: {e}")


def demo_features():
    """Demo 5: Key features."""
    print_header("DEMO 5: Key Features")

    features = {
        "Parameter Control": [
            "Adjust contact detection thresholds",
            "Tune phase FSM timing",
            "Modify physics parameters",
            "Save/load configurations"
        ],
        "Testing Framework": [
            "5 predefined scenarios",
            "Automated metrics (Precision/Recall/F1)",
            "Force peak tracking",
            "JSON export of results"
        ],
        "Visualization": [
            "2D hand contact diagrams",
            "Force-scaled contact markers",
            "Statistical summaries",
            "PNG export for documentation"
        ],
        "Improved Hand Model": [
            "18 contact sensors (3x more)",
            "Realistic finger segments",
            "Multiple palm zones",
            "Better visual quality"
        ]
    }

    for category, items in features.items():
        print(f"\n{category}:")
        for item in items:
            print(f"  ✓ {item}")


def show_quick_start():
    """Show quick start guide."""
    print_header("QUICK START GUIDE")

    steps = [
        ("Test new hand model", "python test_hand_simulation.py"),
        ("Adjust parameters", "python interactive_parameter_tuner.py"),
        ("Run scenario tests", "python test_scenarios.py"),
        ("Visualize contacts", "python visualize_contacts.py")
    ]

    print("\nGet started in 4 steps:")
    for i, (description, command) in enumerate(steps, 1):
        print(f"\n{i}. {description}")
        print(f"   $ {command}")


def show_documentation():
    """Show available documentation."""
    print_header("DOCUMENTATION")

    docs = [
        ("MODULAR_TESTING_GUIDE.md", "Complete testing system guide (detailed)"),
        ("TESTING_SYSTEM_README.md", "Quick reference (start here)"),
        ("MACOS_SETUP.md", "macOS-specific instructions"),
        ("README_INTERACTIVE.md", "Interactive system overview"),
        ("config/simulation_config.yaml", "Editable parameters")
    ]

    print("\nAvailable documentation:")
    for doc, description in docs:
        exists = Path(doc).exists()
        status = "✓" if exists else "✗"
        print(f"  {status} {doc}")
        print(f"     {description}")


def main():
    """Run complete demo."""
    print("\n" + "=" * 70)
    print(" " * 15 + "MODULAR TESTING SYSTEM DEMO")
    print("=" * 70)
    print("\nThis demo shows all new features of the testing system.")
    print("Press Ctrl+C to exit at any time.")

    try:
        # Run demos
        demo_config_system()
        input("\nPress Enter to continue...")

        demo_hand_models()
        input("\nPress Enter to continue...")

        demo_testing_tools()
        input("\nPress Enter to continue...")

        demo_scenarios()
        input("\nPress Enter to continue...")

        demo_features()
        input("\nPress Enter to continue...")

        show_quick_start()
        input("\nPress Enter to continue...")

        show_documentation()

        # Final message
        print("\n" + "=" * 70)
        print("DEMO COMPLETE!")
        print("=" * 70)
        print("\nNext steps:")
        print("  1. Read: TESTING_SYSTEM_README.md")
        print("  2. Try: python test_hand_simulation.py")
        print("  3. Explore: python interactive_parameter_tuner.py")
        print("\nHappy testing! 🚀")
        print("=" * 70)

    except KeyboardInterrupt:
        print("\n\n✓ Demo interrupted by user")
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
