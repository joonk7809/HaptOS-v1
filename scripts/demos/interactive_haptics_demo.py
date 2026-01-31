#!/usr/bin/env python3
"""
Interactive Haptics Demo - Virtual Test Environment
Creates a real-time multi-contact haptic simulation with GUI controls.
"""

import sys
sys.path.append('src')

import argparse
from pathlib import Path

from sync_multi_runner import MultiContactSyncRunner
from gui.threaded_controller import ThreadedSimulationController
from visualization.realtime_plotter import RealtimePlotter
from visualization.contact_logger import ContactEventLogger


def main():
    """Main entry point for interactive haptics demo."""

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="Interactive Haptics Demo - Virtual Test Environment"
    )
    parser.add_argument(
        '--model',
        type=str,
        default='assets/hand_models/simple_hand.xml',
        help='Path to MuJoCo hand model XML (default: assets/hand_models/simple_hand.xml)'
    )
    parser.add_argument(
        '--nn-v0',
        type=str,
        default='models/checkpoints/nn_v0_best.pt',
        help='Path to baseline model checkpoint'
    )
    parser.add_argument(
        '--nn-v1',
        type=str,
        default='models/checkpoints/nn_v1_best.pt',
        help='Path to delta model checkpoint'
    )
    parser.add_argument(
        '--device',
        type=str,
        default='cpu',
        choices=['cpu', 'cuda'],
        help='Device for inference (default: cpu)'
    )
    parser.add_argument(
        '--no-plots',
        action='store_true',
        help='Disable real-time plotting'
    )
    parser.add_argument(
        '--no-logging',
        action='store_true',
        help='Disable contact event logging'
    )

    args = parser.parse_args()

    # Print banner
    print("=" * 70)
    print(" " * 15 + "HaptOS Interactive Test Environment")
    print("=" * 70)
    print()

    # Check if model file exists
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"ERROR: Model file not found: {args.model}")
        print()
        print("Available models:")
        hand_models_dir = Path("assets/hand_models")
        if hand_models_dir.exists():
            for xml_file in hand_models_dir.glob("*.xml"):
                print(f"  - {xml_file}")
        else:
            print("  No hand models found in assets/hand_models/")
        return 1

    # Check if model checkpoints exist
    nn_v0_path = Path(args.nn_v0)
    nn_v1_path = Path(args.nn_v1)

    if not nn_v0_path.exists():
        print(f"WARNING: Baseline model not found: {args.nn_v0}")
        print("The system will not be able to run inference.")
        print()

    if not nn_v1_path.exists():
        print(f"WARNING: Delta model not found: {args.nn_v1}")
        print("The system will not be able to run inference.")
        print()

    # Initialize multi-contact system
    print("Initializing multi-contact system...")
    print(f"  Model: {args.model}")
    print(f"  NN_v0: {args.nn_v0}")
    print(f"  NN_v1: {args.nn_v1}")
    print(f"  Device: {args.device}")
    print()

    try:
        runner = MultiContactSyncRunner(
            model_path=args.model,
            nn_v0_path=args.nn_v0,
            nn_v1_path=args.nn_v1,
            device=args.device
        )
    except Exception as e:
        print(f"ERROR: Failed to initialize system: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Create visualization components
    plotter = None
    logger = None

    if not args.no_plots:
        print("Initializing real-time plotter...")
        plotter = RealtimePlotter(max_history=500)  # 5 seconds @ 100Hz
        print()

    if not args.no_logging:
        print("Initializing contact event logger...")
        logger = ContactEventLogger()
        print()

    # Launch GUI with threaded controller
    print("Launching interactive GUI...")
    print()
    print("=" * 70)
    print("Instructions:")
    print("  1. Use the control panel to spawn objects")
    print("  2. Adjust spawn height, mass, size, and velocity")
    print("  3. Objects will fall onto the hand")
    print("  4. Watch real-time plots for haptic parameters")
    print("  5. Check console for contact event logs")
    print()
    print("Controls:")
    print("  - Spawn Object: Drop an object at specified height")
    print("  - Hand Pose: Change hand configuration (Open/Grasp/Pinch)")
    print("  - Pause: Pause/resume simulation")
    print("  - Reset: Reset simulation to initial state")
    print("  - Exit: Close application")
    print("=" * 70)
    print()

    try:
        controller = ThreadedSimulationController(runner, plotter, logger)
        controller.start()  # Blocks until GUI closes
    except KeyboardInterrupt:
        print("\n\nInterrupted by user (Ctrl+C)")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Cleanup
    print()
    print("=" * 70)
    print("Shutting down...")

    if plotter:
        plotter.close()

    if logger:
        print(f"Contact log saved to: {logger.get_log_path()}")

    print("✓ Goodbye!")
    print("=" * 70)

    return 0


if __name__ == "__main__":
    sys.exit(main())
