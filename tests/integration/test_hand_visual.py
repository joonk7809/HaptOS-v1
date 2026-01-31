#!/usr/bin/env python3
"""
Simple visual test of the hand model.
Opens MuJoCo viewer to visualize the hand - no GUI, no ML models needed.
"""

import sys
sys.path.append('src')

import mujoco
import mujoco.viewer
import time


def main():
    """Visualize hand model in MuJoCo viewer."""

    print("=" * 70)
    print(" " * 20 + "Hand Model Visualization")
    print("=" * 70)
    print()

    # Load hand model
    print("Loading hand model...")
    try:
        model = mujoco.MjModel.from_xml_path("assets/hand_models/simple_hand.xml")
        data = mujoco.MjData(model)
        print("✓ Hand model loaded successfully")
    except Exception as e:
        print(f"ERROR: Failed to load hand model: {e}")
        print("\nMake sure MuJoCo is installed:")
        print("  pip install mujoco")
        return 1

    print()
    print("Opening MuJoCo viewer...")
    print("\nControls:")
    print("  - Left mouse: rotate view")
    print("  - Right mouse: zoom")
    print("  - Middle mouse: pan")
    print("  - Space: pause/resume")
    print("  - Backspace: reset simulation")
    print("  - ESC: close viewer")
    print()
    print("Watch the hand fall and make contact with the floor!")
    print()

    # Launch viewer
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Run until user closes viewer
            while viewer.is_running():
                step_start = time.time()

                # Step physics at 1kHz
                mujoco.mj_step(model, data)

                # Sync viewer
                viewer.sync()

                # Maintain real-time
                elapsed = time.time() - step_start
                if elapsed < 0.001:
                    time.sleep(0.001 - elapsed)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1

    print("\n✓ Viewer closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
