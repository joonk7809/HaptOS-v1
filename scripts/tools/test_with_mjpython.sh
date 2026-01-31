#!/bin/bash
# Script to run tests with mjpython on macOS
# mjpython is MuJoCo's Python wrapper that enables the viewer on macOS

echo "========================================================================"
echo "                    Running with mjpython (macOS)"
echo "========================================================================"
echo ""

# Check if mjpython is available
if ! command -v mjpython &> /dev/null; then
    echo "ERROR: mjpython not found!"
    echo ""
    echo "To install mjpython:"
    echo "  pip install mujoco"
    echo ""
    echo "Then mjpython should be available in your PATH."
    echo "If not, try:"
    echo "  python -m mujoco.viewer"
    exit 1
fi

echo "✓ mjpython found"
echo ""

# Show menu
echo "Select test to run:"
echo "  1) Hand visualization (with MuJoCo viewer)"
echo "  2) Multi-contact test (console output only)"
echo "  3) Simple simulation (no viewer, just contacts)"
echo ""
read -p "Enter choice (1-3): " choice

case $choice in
    1)
        echo ""
        echo "Running hand visualization with mjpython..."
        mjpython test_hand_visual.py
        ;;
    2)
        echo ""
        echo "Running multi-contact test..."
        mjpython simple_haptics_test.py
        ;;
    3)
        echo ""
        echo "Running simulation test (no viewer)..."
        python test_hand_simulation.py
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac
